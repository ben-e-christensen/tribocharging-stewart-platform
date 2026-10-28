"""
CycleController - the actual state machine:
    wiggle -> level -> raster (sampled live) -> re-home CNC -> repeat

Runs on its own background thread so the GUI stays responsive. Talks to the
platform and CNC through their controllers, and to the DAQ/heatmap through
a RasterRecorder - this class itself doesn't know anything about matplotlib
or Tkinter, it just calls raster_start_cb / raster_done_cb at the right
moments so the GUI layer can do its own thing (clear the heatmap, save a
CSV, update a title) without CycleController needing to know how.
"""

import time
import threading

from raster_gcode import build_raster_gcode


class CycleController:
    def __init__(self, klipper, platform, raster_recorder,
                 status_callback, raster_start_callback=None, raster_done_callback=None):
        self.klipper = klipper
        self.platform = platform
        self.raster_recorder = raster_recorder
        self.status_cb = status_callback
        self.raster_start_cb = raster_start_callback  # called with no args, right before a raster starts
        self.raster_done_cb = raster_done_callback    # called with (samples,) after a raster finishes
        self._stop_flag = threading.Event()
        self._thread = None
        self.running = False

    def start(self, wiggle_tilt_deg, wiggle_period_s, wiggle_rotations,
              raster_params, num_cycles=None, rehome_every_n=1,
              level_settle_delay_s=3.0):
        """num_cycles=None means run indefinitely until stop() is called.
        rehome_every_n: re-home the CNC every N cycles instead of every
        single one (sensorless homing physically stalls against a hard
        stop each time - re-homing less often means less mechanical wear,
        at the cost of not catching missed-step drift as quickly).
        level_settle_delay_s: TEMPORARY fixed delay after sending S, used
        in place of a real settle/level confirmation until the
        accelerometer is wired in (see _run for the swap-back point)."""
        if self.running:
            return
        self._stop_flag.clear()
        self.running = True
        self._thread = threading.Thread(
            target=self._run,
            args=(wiggle_tilt_deg, wiggle_period_s, wiggle_rotations,
                  raster_params, num_cycles, rehome_every_n, level_settle_delay_s),
            daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_flag.set()

    def _run(self, tilt_deg, period_s, rotations, raster_params, num_cycles,
              rehome_every_n, level_settle_delay_s):
        cycle_count = 0
        try:
            while not self._stop_flag.is_set():
                if num_cycles is not None and cycle_count >= num_cycles:
                    break
                cycle_count += 1
                self.status_cb(f"Cycle {cycle_count}: wiggling...")

                # --- WIGGLE ---
                self.platform.wiggle_start(tilt_deg, period_s)
                wiggle_duration = period_s * rotations
                elapsed = 0.0
                step = 0.2
                while elapsed < wiggle_duration:
                    if self._stop_flag.is_set():
                        self.platform.level()
                        return
                    time.sleep(step)
                    elapsed += step

                # --- LEVEL ---
                self.status_cb(f"Cycle {cycle_count}: leveling...")
                self.platform.level()
                # TEMPORARY: no accelerometer yet, so there's no real signal
                # to confirm actual level - just wait a fixed, generous delay
                # instead of polling for a "SETTLED" motor-status confirmation.
                # Swap back to the real check once the accelerometer is in:
                #   settled = self.platform.wait_until_settled(timeout=30)
                #   if not settled:
                #       self.status_cb(f"Cycle {cycle_count}: ERROR - platform did not settle. Stopping.")
                #       return
                # ...and add a real roll/pitch-vs-zero check right here too.
                time.sleep(level_settle_delay_s)

                if self._stop_flag.is_set():
                    return

                # --- RASTER (sampled live via RasterRecorder) ---
                self.status_cb(f"Cycle {cycle_count}: rastering...")
                start_x, start_y, circle_radius, stepover, fast_speed = raster_params
                setup_g, raster_g, end_g, last_x, last_y = build_raster_gcode(
                    start_x, start_y, circle_radius, stepover, fast_speed
                )

                if self.raster_start_cb:
                    self.raster_start_cb()

                samples = self.raster_recorder.run_raster(
                    setup_g, raster_g, end_g, start_x, start_y, last_x, last_y,
                    stop_flag=self._stop_flag
                )

                if samples is None:
                    self.status_cb(f"Cycle {cycle_count}: ERROR - raster failed. Stopping.")
                    return

                if self.raster_done_cb:
                    self.raster_done_cb(samples)

                if self._stop_flag.is_set():
                    return

                # --- RE-HOME CNC (every rehome_every_n cycles, not every
                # single one - sensorless homing deliberately stalls the
                # motor against a hard mechanical stop each time). ---
                if cycle_count % rehome_every_n == 0:
                    self.status_cb(f"Cycle {cycle_count}: re-homing CNC...")
                    self.klipper.send_gcode("G28")
                    homed_ok = self._wait_for_homed(timeout=60)
                    if not homed_ok:
                        self.status_cb(f"Cycle {cycle_count}: ERROR - CNC did not confirm home. Stopping.")
                        return
                else:
                    self.status_cb(f"Cycle {cycle_count}: skipping re-home ({cycle_count % rehome_every_n} of {rehome_every_n}).")

                self.status_cb(f"Cycle {cycle_count}: complete.")
        finally:
            self.running = False
            if num_cycles is not None and cycle_count >= num_cycles and not self._stop_flag.is_set():
                self.status_cb(f"Done - completed {cycle_count} cycles.")
            else:
                self.status_cb("Stopped.")

    def _wait_for_homed(self, timeout=60):
        start = time.time()
        while time.time() - start < timeout:
            if self.klipper.is_fully_homed("xy"):
                return True
            time.sleep(0.3)
        return False
