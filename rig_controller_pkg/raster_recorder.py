"""
RasterRecorder - runs one raster pass while sampling the DAQ and tagging
each reading with a POSITION COMPUTED FROM THE MOTION MODEL (see
gcode_motion_model.py).

DAQ sampling itself does NOT happen in this class anymore - it happens
continuously, on its own dedicated thread, inside DAQController
(start_continuous()). This class just DRAINS whatever's been buffered
(via daq.pop_new_samples()) at a fast, purely-local pace and tags each
sample with the modeled position at its timestamp. This is a deliberate
fix: an earlier version called daq.read_once() directly inside the same
loop that also did a periodic network position check, and a slow/stalled
Moonraker HTTP call could block that loop for however long the request
took - starving DAQ sampling of the time it needed, since the loop's exit
condition (elapsed wall-clock time) kept advancing even while blocked on
network I/O. Now DAQ sampling is fully decoupled onto its own thread, and
the (still useful, but no longer timing-critical) drift check runs on ITS
OWN separate thread too - so neither can ever block the other.

This class does NOT touch Tkinter/matplotlib directly - it only pushes
(x, y, charge) tuples into whatever queue.Queue you give it. Something on
the main GUI thread is responsible for draining that queue (see gui.py).
"""

import time
import queue
import threading

from gcode_motion_model import GCodeMotionModel


class RasterRecorder:
    def __init__(self, klipper, daq, point_queue, max_accel=3000.0,
                 drift_check_interval=1.0, drift_warn_tolerance=3.0):
        """
        klipper: a KlipperController.
        daq: a DAQController that's already had start_continuous() called
             on it (typically once, at app startup - see gui.py), OR None
             to run without charge sampling.
        point_queue: a queue.Queue() the GUI thread will drain.
        max_accel: mm/s^2 - MUST match your printer.cfg's max_accel, since
                   it feeds directly into the motion model's timing.
        drift_check_interval: seconds between real position checks during
                   the raster - runs on its OWN thread, never blocks DAQ
                   sampling even if Moonraker is slow to respond.
        drift_warn_tolerance: mm - how far modeled vs actual position can
                   diverge before printing a warning.
        """
        self.klipper = klipper
        self.daq = daq
        self.point_queue = point_queue
        self.max_accel = max_accel
        self.drift_check_interval = drift_check_interval
        self.drift_warn_tolerance = drift_warn_tolerance

    def run_raster(self, setup_gcode, raster_gcode, end_gcode,
                   start_x, start_y, end_x, end_y,
                   stop_flag=None, position_tolerance=1.5, timeout=300):
        """Blocking. Moves to the raster's start (confirmed via a real
        position check - this part is NOT time-critical), fires the raster
        gcode, samples while it runs USING THE MOTION MODEL for position,
        returns to 0,0. Returns the full list of (x, y, charge, timestamp)
        samples collected (empty list if no DAQ), or None if it was stopped
        early or failed to reach the start."""

        self.klipper.send_gcode(setup_gcode)
        if not self._wait_for_pos(start_x, start_y, position_tolerance, timeout, stop_flag):
            return None

        model = GCodeMotionModel(max_accel=self.max_accel)
        modeled_duration = model.load_gcode(raster_gcode, start_x, start_y)

        if self.daq is not None:
            self.daq.pop_new_samples()  # discard anything stale from before this raster

        dispatch_time = time.time()
        self.klipper.send_gcode(raster_gcode)

        if self.daq is None:
            samples = []
            self._sleep_with_stop_check(modeled_duration, stop_flag)
            ok = self._wait_for_pos(end_x, end_y, position_tolerance, timeout, stop_flag)
        else:
            # Drift-checking runs on its OWN thread - a slow network call
            # here can never block the fast, purely-local sample-draining
            # loop below.
            drift_stop = threading.Event()
            drift_thread = threading.Thread(
                target=self._drift_check_loop,
                args=(model, modeled_duration, dispatch_time, drift_stop),
                daemon=True
            )
            drift_thread.start()

            samples = self._drain_during_raster(model, modeled_duration, dispatch_time, stop_flag)

            drift_stop.set()
            drift_thread.join(timeout=2)

            ok = samples is not None
            if ok:
                arrived = self._wait_for_pos(end_x, end_y, position_tolerance,
                                              timeout=10, stop_flag=stop_flag)
                if not arrived:
                    print("[raster] WARNING: modeled duration elapsed but "
                          "real position doesn't confirm arrival - motion "
                          "may be slower than modeled, or something stalled.")

        self.klipper.send_gcode(end_gcode)
        self._wait_for_pos(0, 0, position_tolerance, timeout, stop_flag)

        return samples if ok else None

    def _drain_during_raster(self, model, modeled_duration, dispatch_time, stop_flag):
        """Fast, purely-local loop: repeatedly drains whatever the DAQ's
        continuous background thread has buffered, tags each sample with
        its modeled position, and queues it for the heatmap. No network
        calls happen here at all."""
        samples = []
        while True:
            if stop_flag is not None and stop_flag.is_set():
                return samples

            now = time.time()
            elapsed = now - dispatch_time

            new_samples = self.daq.pop_new_samples()
            for ts, charge in new_samples:
                local_elapsed = max(0.0, min(ts - dispatch_time, modeled_duration))
                px, py = model.position_at(local_elapsed)
                samples.append((px, py, charge, ts))
                try:
                    self.point_queue.put_nowait((px, py, charge))
                except queue.Full:
                    pass  # GUI is behind on rendering - drop, don't block sampling

            if elapsed > modeled_duration:
                return samples

            time.sleep(0.005)  # short poll - just checking the buffer, no hardware I/O here

    def _drift_check_loop(self, model, modeled_duration, dispatch_time, stop_event):
        """Runs on its own thread for the duration of the raster. Slow or
        variable-latency Moonraker calls here can NEVER affect DAQ sampling,
        since this thread is completely separate from the draining loop."""
        while not stop_event.is_set():
            elapsed = time.time() - dispatch_time
            if elapsed > modeled_duration:
                return
            real_x, real_y = self.klipper.get_position()
            if real_x is not None:
                px, py = model.position_at(elapsed)
                drift = ((real_x - px) ** 2 + (real_y - py) ** 2) ** 0.5
                if drift > self.drift_warn_tolerance:
                    print(f"[raster] WARNING: modeled position ({px:.1f},{py:.1f}) "
                          f"vs actual ({real_x:.1f},{real_y:.1f}) - {drift:.1f}mm drift.")
            stop_event.wait(self.drift_check_interval)

    def _sleep_with_stop_check(self, duration, stop_flag, poll=0.1):
        elapsed = 0.0
        while elapsed < duration:
            if stop_flag is not None and stop_flag.is_set():
                return
            time.sleep(poll)
            elapsed += poll

    def _wait_for_pos(self, target_x, target_y, tolerance, timeout, stop_flag):
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if stop_flag is not None and stop_flag.is_set():
                return False
            x, y = self.klipper.get_position()
            if x is not None and y is not None:
                if abs(x - target_x) <= tolerance and abs(y - target_y) <= tolerance:
                    return True
            time.sleep(0.1)
        print("[raster] Warning: position polling timed out (hit E-Stop or machine stalled).")
        return False
