"""
wobble_track.py - wobble the Stewart platform and track the particles on it
with the Basler camera, live.

    python3 wobble_track.py                         # camera + platform, manual control
    python3 wobble_track.py --home H --tilt 8 --period 3
    python3 wobble_track.py --mask 728,544,450 --record --video
    python3 wobble_track.py --no-platform           # camera/tracking only
    python3 wobble_track.py --replay tracking_data/session_.../frames.avi

Standalone from gui.py on purpose: no CNC/Klipper needed, so it runs on the
bench with just the platform + camera. Uses the same PlatformController,
and the new CameraController/particle_tracker modules gui.py can share.

Keys (click the video window first so it has focus):
    h        auto-home platform (H, hall sensors) - needed once per power-up
    w        start orbit wobble (W tilt period)
    j        start joint-wave wibble (J amplitude period) - uses the same
             tilt value as the amplitude
    s        stop + level (S)
    + / -    tilt / amplitude +-1 deg (re-sends if already moving)
    ] / [    period +-0.5 s (re-sends if already moving)
    r        start/stop recording to tracking_data/session_<time>/
    v        toggle the thresholded "what the detector sees" overlay
    c        clear all tracks
    t        toggle trails
    q / Esc  quit (levels the platform first)
Mouse: left-drag on the image = circular detection mask (center -> edge),
       right-click = clear mask.
Sliders: threshold / blob size / background size / exposure / gain /
         display brightness - tune these live while looking at the 'v' view.

THREADS: the camera grab thread (inside CameraController) only keeps the
newest frame; a TrackingWorker thread runs detect -> track -> CSV on every
frame it can get (skipped frames are counted, see HUD "skipped"); the main
thread only renders a ~30 Hz preview and handles keys. Platform commands
run on short-lived threads (H blocks for up to 2 min). A slow preview or a
blocking homing call can never stall tracking.

OUTPUT (when recording) - tracking_data/session_<YYYYmmdd_HHMMSS>/:
    tracks.csv  one row per tracked particle per frame:
    tracks.npy    frame_id, host_time, cam_time_s, track_id, x_px, y_px,
                  area_px, vx_px_s, vy_px_s, platform_mode, tilt_deg,
                  period_s, phase
                Same data twice: .npy (np.load, instant) and .csv. Both are
                written when recording STOPS - during recording rows go to
                a raw tracks.bin because formatting CSV live can't keep up
                with ~100k rows/s. platform_mode is numeric:
                0 offline, 1 idle, 2 homing, 3 orbit, 4 jointwave
                (legend also in meta.json).
                `phase` (0-1) is the fraction of the orbit/wave cycle since
                the W/J command was sent, ESTIMATED host-side - the firmware
                starts its phase at 0 when it receives the command, and USB
                serial latency is ~ms, so it's good for "which way was it
                tilting", not for sub-ms timing.
    events.csv  every platform command sent + the firmware's reply
    meta.json   camera / detector / tracker settings at record start
    frames.avi  (--video) the raw frames, for re-tracking offline with
                --replay; frames.csv maps video index -> frame_id/timestamps
                (video frames can be dropped under load, the index says which)
Positions are in IMAGE PIXELS. No pixel->mm / platform-frame calibration
yet - the platform tilts under a fixed camera, so that will need the
commanded pose (logged here) plus a camera calibration.
"""

import argparse
import csv
import json
import os
import queue
import threading
import time

import cv2
import numpy as np

from camera_controller import CameraController, Frame
from particle_tracker import ParticleDetector, ParticleTracker
from platform_controller import PlatformController

WINDOW = "wobble_track"
HERE = os.path.dirname(os.path.abspath(__file__))
MAX_ORBIT_TILT_DEG = 16.5  # largest tilt reachable all the way round - see octo/README.md
TRACK_COLUMNS = ["frame_id", "host_time", "cam_time_s", "track_id", "x_px", "y_px",
                 "area_px", "vx_px_s", "vy_px_s", "platform_mode", "tilt_deg",
                 "period_s", "phase"]
TRACK_CSV_FMT = ["%d", "%.6f", "%.6f", "%d", "%.2f", "%.2f",
                 "%d", "%.1f", "%.1f", "%d", "%g", "%g", "%.4f"]
# platform_mode is stored as a number in tracks.npy/csv (keeps rows purely
# numeric so they can be dumped in one vectorized write per frame).
PLATFORM_MODES = {"offline": 0, "idle": 1, "homing": 2, "orbit": 3, "jointwave": 4}


# ---------------- frame sources ----------------

class VideoSource:
    """Plays back a frames.avi recorded by this script (or any video) through
    the same next_frame() interface as CameraController, paced at the
    original rate. Uses the sibling frames.csv for exact timestamps when
    present."""

    def __init__(self, path, speed=1.0):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Can't open {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 100.0
        self.speed = speed
        self.index = None
        idx_path = os.path.join(os.path.dirname(path), "frames.csv")
        if os.path.exists(idx_path):
            with open(idx_path) as f:
                self.index = [(int(r["frame_id"]), int(r["cam_time_ns"])) for r in csv.DictReader(f)]
        self.n = 0
        self.done = False
        self._t0 = None

    def start_stream(self):
        pass

    def next_frame(self, after_id=None, timeout=1.0):
        ok, img = self.cap.read()
        if not ok:
            self.done = True
            return None
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.index and self.n < len(self.index):
            frame_id, cam_ns = self.index[self.n]
        else:
            frame_id, cam_ns = self.n, int(self.n / self.fps * 1e9)
        self.n += 1
        if self._t0 is None:
            self._t0 = (time.time(), cam_ns)
        wait = self._t0[0] + (cam_ns - self._t0[1]) * 1e-9 / self.speed - time.time()
        if wait > 0:
            time.sleep(wait)
        return Frame(img, frame_id, time.time(), cam_ns)

    def close(self):
        self.cap.release()


# ---------------- platform ----------------

class PlatformState:
    """What we last told the platform to do - so every tracked frame can be
    tagged with it. Written by command threads, read by the tracking
    worker; plain attribute reads/writes, consistent enough for logging."""

    def __init__(self, platform, tilt, period, on_event):
        self.platform = platform
        self.tilt = tilt
        self.period = period
        self.mode = "idle" if platform and platform.connected else "offline"
        self.start_time = None
        self.last_reply = ""
        self._on_event = on_event

    def phase(self, t):
        if self.mode in ("orbit", "jointwave") and self.start_time is not None:
            return ((t - self.start_time) / self.period) % 1.0
        return None

    def _run(self, cmd, new_mode):
        sent = time.time()
        replies = self.platform.command(cmd)
        reply = " | ".join(replies)
        self.last_reply = reply or "(no reply)"
        print(f"[platform] {cmd} -> {self.last_reply}")
        if any("Not homed" in r or "error" in r.lower() for r in replies):
            new_mode = "idle"
        self.mode = new_mode
        self.start_time = sent if new_mode in ("orbit", "jointwave") else None
        self._on_event(sent, cmd, self.last_reply)

    def send(self, cmd, new_mode):
        if not (self.platform and self.platform.connected):
            self.last_reply = "platform not connected"
            return
        threading.Thread(target=self._run, args=(cmd, new_mode), daemon=True).start()

    def wobble(self):
        self.send(f"W {self.tilt:g} {self.period:g}", "orbit")

    def wibble(self):
        self.send(f"J {self.tilt:g} {self.period:g}", "jointwave")

    def stop(self):
        self.send("S", "idle")

    def resend(self):
        if self.mode == "orbit":
            self.wobble()
        elif self.mode == "jointwave":
            self.wibble()

    def hall_home(self):
        if not (self.platform and self.platform.connected):
            self.last_reply = "platform not connected"
            return

        def run():
            self.mode = "homing"
            self.last_reply = "auto-homing (H)..."
            sent = time.time()
            ok, last = self.platform.hall_home()
            self.last_reply = ("homed" if ok else "HOMING FAILED: ") + ("" if ok else last)
            self.mode = "idle"
            self._on_event(sent, "H", last)
        threading.Thread(target=run, daemon=True).start()


# ---------------- recording ----------------

class SessionRecorder:
    def __init__(self, out_root, meta, save_video, video_codec, video_fps):
        session = time.strftime("session_%Y%m%d_%H%M%S")
        self.dir = os.path.join(out_root, session)
        os.makedirs(self.dir, exist_ok=True)
        with open(os.path.join(self.dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        # Rows go to a raw float64 file during recording (one tofile() per
        # frame) and get converted to tracks.npy + tracks.csv at close().
        # Formatting CSV rows inline couldn't keep up: ~500 particles at
        # 100-200 fps is 50-100k rows/s, which cut tracking from ~80 to
        # ~50 fps.
        self._bin_path = os.path.join(self.dir, "tracks.bin")
        self._tracks_f = open(self._bin_path, "wb")
        self.rows = 0
        self._events_f = open(os.path.join(self.dir, "events.csv"), "w", newline="")
        self._events = csv.writer(self._events_f)
        self._events.writerow(["host_time", "command", "reply"])
        self._events_lock = threading.Lock()

        # Video encoding is the one thing here too slow to do inline at
        # ~200 fps (MJPG ~150 fps/core, FFV1 ~40) - so it gets its own
        # thread + bounded queue, and drops video frames (counted, and the
        # frames.csv index says exactly which made it) rather than ever
        # back-pressuring tracking.
        self.video_dropped = 0
        self._video_q = None
        if save_video:
            self._video_q = queue.Queue(maxsize=128)
            self._video_thread = threading.Thread(
                target=self._video_loop, args=(video_codec, video_fps), daemon=True)
            self._video_thread.start()
        print(f"[record] -> {self.dir}")

    def log_frame(self, frame, matched, pstate):
        if matched:
            phase = pstate.phase(frame.host_time)
            per_track = np.array([(tr.id, tr.x, tr.y, tr.area, tr.vx, tr.vy) for tr in matched])
            rows = np.empty((len(matched), len(TRACK_COLUMNS)))
            rows[:, 0] = frame.frame_id
            rows[:, 1] = frame.host_time
            rows[:, 2] = frame.cam_time_ns * 1e-9
            rows[:, 3:9] = per_track
            rows[:, 9] = PLATFORM_MODES.get(pstate.mode, -1)
            rows[:, 10] = pstate.tilt
            rows[:, 11] = pstate.period
            rows[:, 12] = np.nan if phase is None else phase
            rows.tofile(self._tracks_f)
            self.rows += len(matched)
        if self._video_q is not None:
            try:
                self._video_q.put_nowait(frame)
            except queue.Full:
                self.video_dropped += 1

    def log_event(self, host_time, command, reply):
        with self._events_lock:
            self._events.writerow([f"{host_time:.6f}", command, reply])
            self._events_f.flush()

    def _video_loop(self, codec, fps):
        writer = None
        idx_f = open(os.path.join(self.dir, "frames.csv"), "w", newline="")
        idx = csv.writer(idx_f)
        idx.writerow(["video_index", "frame_id", "cam_time_ns", "host_time"])
        n = 0
        try:
            while True:
                frame = self._video_q.get()
                if frame is None:
                    break
                if writer is None:
                    h, w = frame.image.shape
                    writer = cv2.VideoWriter(os.path.join(self.dir, "frames.avi"),
                                             cv2.VideoWriter_fourcc(*codec), fps, (w, h), isColor=False)
                    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, 95)
                writer.write(frame.image)
                idx.writerow([n, frame.frame_id, frame.cam_time_ns, f"{frame.host_time:.6f}"])
                n += 1
        finally:
            if writer is not None:
                writer.release()
            idx_f.close()

    def close(self):
        if self._video_q is not None:
            self._video_q.put(None)
            self._video_thread.join(timeout=30)
        self._tracks_f.close()
        self._events_f.close()
        print(f"[record] closed {self.dir}" +
              (f" ({self.video_dropped} video frames dropped)" if self.video_dropped else ""))
        # CSV export is ~4 s per million rows - run it off the UI thread.
        threading.Thread(target=self._export_tracks, daemon=False).start()

    def _export_tracks(self):
        rows = np.fromfile(self._bin_path).reshape(-1, len(TRACK_COLUMNS))
        np.save(os.path.join(self.dir, "tracks.npy"), rows)
        print(f"[record] exporting {len(rows)} rows to tracks.csv...")
        np.savetxt(os.path.join(self.dir, "tracks.csv"), rows, fmt=TRACK_CSV_FMT,
                   delimiter=",", header=",".join(TRACK_COLUMNS), comments="")
        os.remove(self._bin_path)
        print(f"[record] tracks.csv + tracks.npy written ({os.path.basename(self.dir)})")


# ---------------- tracking worker ----------------

class TrackingWorker(threading.Thread):
    def __init__(self, source, detector, tracker, pstate):
        super().__init__(daemon=True)
        self.source = source
        self.detector = detector
        self.tracker = tracker
        self.pstate = pstate
        self.recorder = None
        self.lock = threading.Lock()   # guards tracker vs. reset() from the UI thread
        self.stop_event = threading.Event()
        self.skipped = 0
        self.proc_fps = 0.0
        self.snapshot = None           # latest (frame, n_dets, [(id, trail, live)]) for display
        self._snap_every = 1 / 30

    def run(self):
        last_id = None
        last_snap = 0.0
        window_start, window_frames = time.perf_counter(), 0
        while not self.stop_event.is_set():
            frame = self.source.next_frame(last_id, timeout=1.0)
            if frame is None:
                if getattr(self.source, "done", False):
                    break
                continue
            if last_id is not None and frame.frame_id > last_id + 1:
                self.skipped += frame.frame_id - last_id - 1
            last_id = frame.frame_id

            dets = self.detector.detect(frame.image)
            with self.lock:
                matched = self.tracker.update(dets, frame.cam_time_ns * 1e-9)
                rec = self.recorder
                if rec is not None:
                    rec.log_frame(frame, matched, self.pstate)
                now = time.perf_counter()
                if now - last_snap >= self._snap_every:
                    last_snap = now
                    tracks = [(tr.id, list(tr.trail), tr.missed == 0)
                              for tr in self.tracker.tracks.values()]
                    self.snapshot = (frame, len(dets), tracks)

            window_frames += 1
            if now - window_start >= 1.0:
                self.proc_fps = window_frames / (now - window_start)
                window_start, window_frames = now, 0

    def reset_tracks(self):
        with self.lock:
            self.tracker.reset()

    def set_recorder(self, rec):
        with self.lock:
            old, self.recorder = self.recorder, rec
        return old


# ---------------- UI ----------------

def track_color(track_id):
    hue = (track_id * 47) % 180  # spread consecutive IDs around the hue wheel
    bgr = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(c) for c in bgr)


class App:
    def __init__(self, args):
        self.args = args
        self.out_root = os.path.join(HERE, "tracking_data")

        if args.replay:
            self.camera = None
            self.source = VideoSource(args.replay, speed=args.replay_speed)
        else:
            roi = tuple(int(v) for v in args.roi.split(",")) if args.roi else None
            self.camera = CameraController(serial=args.serial, exposure_us=args.exposure,
                                           gain_db=args.gain, fps=args.fps, roi=roi)
            self.camera.open()
            self.source = self.camera

        self.platform = None
        if not (args.no_platform or args.replay):
            self.platform = PlatformController()
            if args.port:
                port = args.port if self.platform.connect(args.port) else None
            else:
                port = self.platform.auto_connect()
            print(f"[platform] {'connected on ' + port if port else 'NOT connected - wobble keys disabled'}")
        self.pstate = PlatformState(self.platform, args.tilt, args.period, self._log_event)

        self.detector = ParticleDetector(polarity=args.polarity, threshold=args.threshold,
                                         background_px=args.background_px,
                                         min_area=args.min_area, max_area=args.max_area)
        self.tracker = ParticleTracker(max_link_px=args.max_link, max_missed=args.max_missed)
        self.worker = TrackingWorker(self.source, self.detector, self.tracker, self.pstate)

        self.display_gain = args.display_gain
        self.show_binary = False
        self.show_trails = True
        self.scale = 1.0
        self._drag_center = None
        self._drag_radius = 0
        self._mask_spec = args.mask

    # -- recording --

    def _log_event(self, host_time, command, reply):
        rec = self.worker.recorder
        if rec is not None:
            rec.log_event(host_time, command, reply)

    def _meta(self):
        a = self.args
        cam = {}
        if self.camera is not None:
            c = self.camera.cam
            cam = {"model": c.DeviceModelName.Value, "serial": c.DeviceSerialNumber.Value,
                   "width": c.Width.Value, "height": c.Height.Value,
                   "offset_x": c.OffsetX.Value, "offset_y": c.OffsetY.Value,
                   "exposure_us": c.ExposureTime.Value, "gain_db": c.Gain.Value,
                   "resulting_fps": c.ResultingFrameRate.Value}
        d = self.detector
        return {
            "started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source": a.replay or "camera",
            "camera": cam,
            "detector": {"polarity": d.polarity, "threshold": d.threshold,
                         "background_px": d.background_px, "min_area": d.min_area,
                         "max_area": d.max_area, "smooth_px": d.smooth_px,
                         "mask_circle": d.mask_circle},
            "tracker": {"max_link_px": self.tracker.max_link_px,
                        "max_missed": self.tracker.max_missed,
                        "velocity_smoothing": self.tracker.velocity_smoothing},
            "platform": {"mode": self.pstate.mode, "tilt_deg": self.pstate.tilt,
                         "period_s": self.pstate.period},
            "tracks_columns": TRACK_COLUMNS,
            "platform_mode_codes": PLATFORM_MODES,
        }

    def toggle_recording(self):
        if self.worker.recorder is None:
            fps = self.camera.resulting_fps if self.camera else self.source.fps
            rec = SessionRecorder(self.out_root, self._meta(),
                                  save_video=self.args.video and self.camera is not None,
                                  video_codec=self.args.video_codec, video_fps=fps)
            self.worker.set_recorder(rec)
        else:
            self.worker.set_recorder(None).close()

    # -- UI setup --

    def _setup_window(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.setMouseCallback(WINDOW, self._on_mouse)

        def slider(name, value, maximum, setter):
            cv2.createTrackbar(name, WINDOW, int(value), maximum, setter)

        d = self.detector
        slider("threshold", d.threshold, 100, lambda v: setattr(d, "threshold", max(1, v)))
        slider("min area", d.min_area, 200, lambda v: setattr(d, "min_area", max(1, v)))
        slider("max area", min(d.max_area, 5000), 5000, lambda v: setattr(d, "max_area", max(1, v)))
        slider("bg px", d.background_px, 151, lambda v: setattr(d, "background_px", max(3, v)))
        slider("max link px", self.tracker.max_link_px, 200,
               lambda v: setattr(self.tracker, "max_link_px", max(1, v)))
        if self.camera is not None:
            slider("exposure x100us", max(1, round(self.args.exposure / 100)), 200,
                   lambda v: self.camera.set_exposure(max(1, v) * 100.0))
            slider("gain dB", self.args.gain, 24, lambda v: self.camera.set_gain(float(v)))
        slider("display x", self.display_gain, 20,
               lambda v: setattr(self, "display_gain", max(1, v)))

    def _on_mouse(self, event, x, y, flags, _):
        ix, iy = x / self.scale, y / self.scale
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag_center = (ix, iy)
            self._drag_radius = 0
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_center is not None:
            self._drag_radius = np.hypot(ix - self._drag_center[0], iy - self._drag_center[1])
        elif event == cv2.EVENT_LBUTTONUP and self._drag_center is not None:
            snap = self.worker.snapshot
            if snap is not None and self._drag_radius > 5:
                self.detector.set_mask_circle(snap[0].image.shape, *self._drag_center, self._drag_radius)
                print(f"[mask] --mask {self.detector.mask_circle[0]},{self.detector.mask_circle[1]},"
                      f"{self.detector.mask_circle[2]}")
            self._drag_center = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.detector.clear_mask()

    # -- render --

    def render(self, snap):
        frame, n_dets, tracks = snap
        img = frame.image
        disp = cv2.convertScaleAbs(img, alpha=self.display_gain)
        disp = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        b = self.detector.last_binary
        if self.show_binary and b is not None:
            x0, y0 = self.detector.last_offset
            region = disp[y0:y0 + b.shape[0], x0:x0 + b.shape[1]]
            if region.shape[:2] == b.shape:
                region[b > 0] = (0, 0, 255)

        max_w = self.args.display_width
        self.scale = min(1.0, max_w / disp.shape[1])
        if self.scale < 1.0:
            disp = cv2.resize(disp, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        s = self.scale

        if self.detector.mask_circle:
            cx, cy, r = self.detector.mask_circle
            cv2.circle(disp, (int(cx * s), int(cy * s)), int(r * s), (0, 200, 0), 1)
        if self._drag_center is not None:
            cx, cy = self._drag_center
            cv2.circle(disp, (int(cx * s), int(cy * s)), int(self._drag_radius * s), (0, 255, 255), 1)

        n_live = 0
        for tid, trail, live in tracks:
            color = track_color(tid)
            if self.show_trails and len(trail) > 1:
                pts = (np.array(trail) * s).astype(np.int32)
                cv2.polylines(disp, [pts], False, color, 1, cv2.LINE_AA)
            if live:
                n_live += 1
                x, y = trail[-1]
                cv2.circle(disp, (int(x * s), int(y * s)), 5, color, 1, cv2.LINE_AA)

        p = self.pstate
        phase = p.phase(time.time())
        cam_fps = self.camera.resulting_fps if self.camera else self.source.fps
        lines = [
            f"proc {self.worker.proc_fps:5.1f} fps (cam {cam_fps:.0f})  skipped {self.worker.skipped}"
            f"  detections {n_dets}  tracks {n_live}",
            f"platform: {p.mode}" + (f" phase {phase:.2f}" if phase is not None else "") +
            f"   tilt {p.tilt:g} deg   period {p.period:g} s   | {p.last_reply[:70]}",
        ]
        rec = self.worker.recorder
        if rec is not None:
            lines.append(f"REC {os.path.basename(rec.dir)}" +
                         (f"  video dropped {rec.video_dropped}" if rec._video_q is not None else ""))
        # Dark box behind the HUD rather than outlined text - OpenCV 5's
        # putText renders a thick "outline" pass at a different advance
        # width, so the two passes don't line up.
        cv2.rectangle(disp, (0, 0), (disp.shape[1], 10 + 22 * len(lines)), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv2.putText(disp, text, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (80, 80, 255) if i == 2 else (255, 255, 255), 1, cv2.LINE_AA)
        return disp

    # -- main loop --

    def handle_key(self, key):
        p = self.pstate
        if key in (ord("q"), 27):
            return False
        elif key == ord("h"):
            p.hall_home()
        elif key == ord("w"):
            p.tilt = min(p.tilt, MAX_ORBIT_TILT_DEG)
            p.wobble()
        elif key == ord("j"):
            p.wibble()
        elif key == ord("s"):
            p.stop()
        elif key in (ord("+"), ord("=")):
            limit = MAX_ORBIT_TILT_DEG if p.mode == "orbit" else 45.0
            p.tilt = min(limit, p.tilt + 1)
            p.resend()
        elif key == ord("-"):
            p.tilt = max(0.0, p.tilt - 1)
            p.resend()
        elif key == ord("]"):
            p.period += 0.5
            p.resend()
        elif key == ord("["):
            p.period = max(0.5, p.period - 0.5)
            p.resend()
        elif key == ord("r"):
            self.toggle_recording()
        elif key == ord("v"):
            self.show_binary = not self.show_binary
        elif key == ord("c"):
            self.worker.reset_tracks()
        elif key == ord("t"):
            self.show_trails = not self.show_trails
        return True

    def run(self):
        self.source.start_stream()
        if self._mask_spec:
            cx, cy, r = (float(v) for v in self._mask_spec.split(","))
            first = self.source.next_frame(timeout=5.0)
            if first is not None:
                self.detector.set_mask_circle(first.image.shape, cx, cy, r)
        self.worker.start()
        self._setup_window()
        if self.args.home == "H":
            self.pstate.hall_home()
        if self.args.record:
            self.toggle_recording()

        try:
            while True:
                snap = self.worker.snapshot
                if snap is not None:
                    cv2.imshow(WINDOW, self.render(snap))
                key = cv2.waitKey(30) & 0xFF
                if key != 255 and not self.handle_key(key):
                    break
                if not self.worker.is_alive():
                    print("[track] source finished.")
                    cv2.waitKey(0)
                    break
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        if self.platform is not None and self.platform.connected:
            if self.pstate.mode in ("orbit", "jointwave"):
                print("[platform] leveling before exit (S)")
                self.platform.level()
            self.platform.disconnect()
        self.worker.stop_event.set()
        self.worker.join(timeout=3)
        rec = self.worker.set_recorder(None)
        if rec is not None:
            rec.close()
        if self.camera is not None:
            self.camera.close()
        else:
            self.source.close()
        cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description="Wobble the Stewart platform and track particles with the Basler camera.")
    g = p.add_argument_group("platform")
    g.add_argument("--port", help="Octopus serial port (default: auto-detect by USB descriptor)")
    g.add_argument("--no-platform", action="store_true", help="camera + tracking only")
    g.add_argument("--home", choices=["H", "none"], default="none",
                   help="auto-home (hall sensors) on launch; default none - press h")
    g.add_argument("--tilt", type=float, default=8.0, help="orbit tilt / joint-wave amplitude, deg")
    g.add_argument("--period", type=float, default=4.0, help="seconds per orbit / wave cycle")

    g = p.add_argument_group("camera")
    g.add_argument("--serial", help="Basler serial number (default: first camera)")
    g.add_argument("--exposure", type=float, default=3000, help="us - keep short to avoid motion blur")
    g.add_argument("--gain", type=float, default=6.0, help="dB")
    g.add_argument("--fps", type=float, default=None, help="cap camera frame rate (default: max)")
    g.add_argument("--roi", help="sensor crop x,y,w,h - smaller = faster")
    g.add_argument("--replay", help="track a recorded frames.avi instead of the live camera")
    g.add_argument("--replay-speed", type=float, default=1.0)

    g = p.add_argument_group("detection / tracking")
    g.add_argument("--polarity", choices=["bright", "dark"], default="bright",
                   help="particles lighter or darker than the platform under them")
    g.add_argument("--threshold", type=int, default=12, help="gray levels above local background")
    g.add_argument("--background-px", type=int, default=31, help="local background window, px")
    g.add_argument("--min-area", type=int, default=3)
    g.add_argument("--max-area", type=int, default=2000)
    g.add_argument("--mask", help="circular detection region cx,cy,r in image px (or drag with mouse)")
    g.add_argument("--max-link", type=float, default=25.0, help="max px a particle may move between frames")
    g.add_argument("--max-missed", type=int, default=3, help="frames a track survives undetected")

    g = p.add_argument_group("output")
    g.add_argument("--record", action="store_true", help="start recording immediately")
    g.add_argument("--video", action="store_true", help="also save raw frames when recording")
    g.add_argument("--video-codec", default="MJPG",
                   help="MJPG (fast, default) or FFV1 (lossless, ~40 fps full-frame - use with --fps/--roi)")
    g.add_argument("--display-width", type=int, default=1200)
    g.add_argument("--display-gain", type=int, default=4, help="preview brightness multiplier")
    App(p.parse_args()).run()


if __name__ == "__main__":
    main()
