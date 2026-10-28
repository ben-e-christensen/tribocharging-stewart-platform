"""
CameraController - wraps the Basler acA1440-220um (USB3, mono) via pypylon.
Replaces the old picamera2/imx477 version from the Raspberry Pi days.

Two ways to use it:

  - capture(): one frame, as a numpy array. This is what the raster cycle
    uses for its "what did the particles look like right before this
    raster" photo (gui.py). If the camera is already streaming, it just
    takes the newest streamed frame instead of interrupting the stream.

  - start_stream() / next_frame(): continuous acquisition for particle
    tracking (wobble_track.py). Uses pylon's LatestImageOnly strategy, so
    next_frame() always returns the NEWEST frame - if the consumer falls
    behind, frames are dropped rather than queued, so tracking stays
    real-time instead of drifting further and further behind the platform.
    Skipped frames show up as gaps in Frame.frame_id (wobble_track.py
    counts and displays them) - lower the frame rate cap (fps) or shrink
    the ROI if that number keeps climbing.
    No grab thread of our own: pylon already acquires in the background,
    and an extra Python thread copying every frame at ~225 fps (when the
    tracker only consumes ~60-80) measurably cost throughput (62 vs 76 fps
    with ~500 particles in view) through GIL/memory-bandwidth contention.

Frames are Mono8 uint8 2-D arrays. PIL/Tkinter/OpenCV all accept those
directly, so nothing downstream needs to convert to RGB.

`pypylon` is imported LAZILY (inside open(), not at module load time) -
same pattern the old picamera2/daqhats wrappers used - so importing this
module on a machine without the Pylon SDK doesn't blow up.

Frame timestamps: each frame carries both the host's time.time() at
retrieval (comparable to the platform/command log) and the camera's own
hardware timestamp in ns (precise frame-to-frame spacing, unaffected by
USB/thread jitter - use this one for velocities).
"""

import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    image: np.ndarray       # Mono8, (H, W)
    frame_id: int           # increments per grabbed frame (gaps = camera-side skips)
    host_time: float        # time.time() when retrieved
    cam_time_ns: int        # camera hardware timestamp


class CameraController:
    def __init__(self, serial=None, exposure_us=3000.0, gain_db=0.0,
                 fps=None, roi=None):
        """serial: pick a specific camera by serial number (default: first found).
        exposure_us: keep this SHORT for moving particles - motion blur
            smears a particle over (speed * exposure) pixels.
        fps: cap the acquisition frame rate (None = as fast as the sensor
            and exposure allow, ~220fps full-frame on this model).
        roi: (offset_x, offset_y, width, height) sensor crop, or None for
            the full sensor. A smaller ROI raises the max frame rate and
            cuts processing cost proportionally."""
        self.serial = serial
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self.fps = fps
        self.roi = roi

        self.cam = None
        self._pylon = None
        self._lock = threading.Lock()          # guards cam access between threads
        self._streaming = False

    # ---------------- open / configure ----------------

    def open(self):
        if self.cam is not None:
            return
        from pypylon import pylon
        self._pylon = pylon
        tl = pylon.TlFactory.GetInstance()
        devs = tl.EnumerateDevices()
        if self.serial:
            devs = [d for d in devs if d.GetSerialNumber() == str(self.serial)]
        if not devs:
            raise RuntimeError("No Basler camera found" +
                               (f" with serial {self.serial}" if self.serial else "") +
                               " - check the USB3 cable and udev rules.")
        self.cam = pylon.InstantCamera(tl.CreateDevice(devs[0]))
        self.cam.Open()
        self._configure()
        print(f"[camera] {self.cam.DeviceModelName.Value} SN={self.cam.DeviceSerialNumber.Value} "
              f"{self.cam.Width.Value}x{self.cam.Height.Value} "
              f"exp={self.cam.ExposureTime.Value:.0f}us "
              f"-> {self.cam.ResultingFrameRate.Value:.1f} fps max")

    def _configure(self):
        cam = self.cam
        # Offsets must go to 0 BEFORE widening, or Width.Max is clipped by
        # whatever offset a previous session left behind.
        cam.OffsetX.Value = 0
        cam.OffsetY.Value = 0
        if self.roi:
            x, y, w, h = self.roi
            cam.Width.Value = self._snap(cam.Width, w)
            cam.Height.Value = self._snap(cam.Height, h)
            cam.OffsetX.Value = self._snap(cam.OffsetX, x)
            cam.OffsetY.Value = self._snap(cam.OffsetY, y)
        else:
            cam.Width.Value = cam.Width.Max
            cam.Height.Value = cam.Height.Max
        cam.PixelFormat.Value = "Mono8"
        cam.ExposureAuto.Value = "Off"
        cam.GainAuto.Value = "Off"
        cam.ExposureTime.Value = self.exposure_us
        cam.Gain.Value = self.gain_db
        cam.AcquisitionFrameRateEnable.Value = bool(self.fps)
        if self.fps:
            cam.AcquisitionFrameRate.Value = self.fps

    @staticmethod
    def _snap(node, value):
        """Clamp + round down to the node's increment (sensor ROI values
        must be multiples of e.g. 4 on this model)."""
        inc = max(1, node.Inc)
        value = max(node.Min, min(node.Max, int(value)))
        return node.Min + (value - node.Min) // inc * inc

    def set_exposure(self, exposure_us):
        """Safe to call while streaming."""
        self.exposure_us = exposure_us
        if self.cam is not None:
            with self._lock:
                self.cam.ExposureTime.Value = exposure_us

    def set_gain(self, gain_db):
        self.gain_db = gain_db
        if self.cam is not None:
            with self._lock:
                self.cam.Gain.Value = gain_db

    @property
    def resulting_fps(self):
        return self.cam.ResultingFrameRate.Value if self.cam is not None else None

    # ---------------- single still (raster cycle) ----------------

    def capture(self):
        """Returns ONE frame as a numpy array (Mono8)."""
        if self._streaming:
            f = self.next_frame(timeout=5.0)
            if f is not None:
                return f.image
        self.open()
        with self._lock:
            res = self.cam.GrabOne(5000)
            try:
                if not res.GrabSucceeded():
                    raise RuntimeError(f"grab failed: {res.ErrorDescription}")
                return res.Array.copy()
            finally:
                res.Release()

    # ---------------- continuous streaming (tracking) ----------------

    def start_stream(self):
        self.open()
        if self._streaming:
            return
        with self._lock:
            self.cam.MaxNumBuffer.Value = 16
            self.cam.StartGrabbing(self._pylon.GrabStrategy_LatestImageOnly)
            self._streaming = True

    def next_frame(self, after_id=None, timeout=1.0):
        """Blocks until the next frame is available and returns the newest
        one (or None on timeout / not streaming). Frames that arrived while
        the caller was busy are skipped - they show up as a jump in
        frame_id. after_id is accepted for interface compatibility with
        wobble_track.VideoSource; LatestImageOnly never repeats a frame."""
        pylon = self._pylon
        with self._lock:
            if not self._streaming:
                return None
            try:
                res = self.cam.RetrieveResult(int(timeout * 1000), pylon.TimeoutHandling_Return)
            except Exception as e:
                print(f"[camera] RetrieveResult error: {e}")
                return None
            if res is None or not res.IsValid():
                return None
            try:
                if not res.GrabSucceeded():
                    print(f"[camera] grab error {res.ErrorCode}: {res.ErrorDescription}")
                    return None
                return Frame(res.Array.copy(), res.BlockID, time.time(), res.TimeStamp)
            finally:
                res.Release()

    def stop_stream(self):
        with self._lock:
            if not self._streaming:
                return
            self._streaming = False
            try:
                self.cam.StopGrabbing()
            except Exception as e:
                print(f"[camera] StopGrabbing error (usually harmless): {e}")

    def close(self):
        self.stop_stream()
        if self.cam is not None:
            try:
                self.cam.Close()
            except Exception as e:
                print(f"[camera] Error closing camera (usually harmless): {e}")
            self.cam = None
