"""
gui.py - the Tkinter application. Wires together every controller module
into one window:
    - CNC (Klipper) panel                   (your original functionality, jog buttons removed)
    - Stewart platform panel                (auto-connects on launch; Z/H/S)
    - Charge Heatmap panel                  (ONE persistent embedded plot)
    - Auto Cycle panel                      (the wiggle/level/raster/rehome loop)

No charge ADC is wired in right now: the MCC118 DAQ HAT went away with the
move off the Raspberry Pi, and its replacement isn't coded yet. self.daq is
None until then, which RasterRecorder already treats as "move the CNC
through the raster, collect no samples" - see the ADC note in __init__.

Threading model: background threads (the cycle, the manual raster) only
ever push data into thread-safe queues or call plain Python functions
(status callbacks, file saving). Anything that touches a Tkinter widget or
the matplotlib canvas happens ONLY on the main thread, either directly (in
a button handler) or via self.root.after(...) hopping back from a
background thread.
"""

import tkinter as tk
from tkinter import ttk
import threading
import queue
import time

from klipper_controller import KlipperController
from platform_controller import PlatformController
from raster_gcode import build_raster_gcode
from raster_recorder import RasterRecorder
from live_heatmap import LiveHeatmap
from cycle_controller import CycleController
from data_export import SessionCSVLogger
from camera_controller import CameraController
from camera_view import CameraView


class JogGUI:
    def __init__(self, root, klipper_controller):
        self.klipper = klipper_controller
        self.root = root
        self.root.title("CNC + Stewart Platform + Charge Mapping")

        # ==========================================
        # --- CENTRAL CONTROL PANEL (VARIABLES) ---
        # ==========================================
        self.start_x = 117.5
        self.start_y = 15
        self.circle_radius = 90
        self.fast_speed = 18000

        self.sensor_radius = 15.75  # 31.5mm probe diameter / 2 - used to paint
                                    # each sample as a disc of this radius on
                                    # the heatmap, matching the sensor's real
                                    # physical footprint (see live_heatmap.py)
        self.raster_stepover = 25.0
        # ==========================================

        self.platform = PlatformController()
        # >>> NEW ADC GOES HERE <<< (the MCC118 HAT's DAQController was
        # removed with the Pi). Whatever replaces it needs the same small
        # interface RasterRecorder/this class used:
        #   start_continuous()  - sample on its own background thread
        #   pop_new_samples()   - return + clear buffered (timestamp, charge)
        #   stop_scan(), close()
        # Keep sampling decoupled from CNC/network calls the same way (see
        # CLAUDE.md, "DAQ sampling is fully decoupled") - then start it on
        # a background thread here like the old one was.
        self.daq = None
        self.point_queue = queue.Queue(maxsize=5000)
        self.session_logger = SessionCSVLogger()  # ONE file for this whole
        # app session - every raster appends to it with an incrementing
        # raster_number column, rather than a new file every raster.
        print(f"[data] Logging this session's raster data to {self.session_logger.path}")
        self.camera = CameraController()  # Basler; opens lazily on first capture()
        # >>> MUST match your printer.cfg's [printer] max_accel <<<
        # This feeds directly into the motion model used to tag DAQ samples
        # with position - if it drifts out of sync with the real config,
        # the modeled positions will be subtly wrong.
        CNC_MAX_ACCEL = 3000.0

        self.raster_recorder = RasterRecorder(self.klipper, self.daq, self.point_queue,
                                               max_accel=CNC_MAX_ACCEL)
        # ^ daq=None -> rasters run without charge sampling until the new
        # ADC is wired in (see above).

        self.cycle = CycleController(
            self.klipper, self.platform, self.raster_recorder,
            status_callback=self._on_cycle_status,
            raster_start_callback=self._on_raster_start,
            raster_done_callback=self._on_raster_done,
        )

        self.setup_ui()
        self.update_coordinates()
        self._drain_heatmap_queue()  # kick off the periodic queue drain
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        threading.Thread(target=self._startup_sequence, daemon=True).start()
        threading.Thread(target=self._poll_accel_loop, daemon=True).start()

    # ---------------- layout ----------------

    def setup_ui(self):
        outer = tk.Frame(self.root)
        outer.pack(padx=15, pady=15, fill="both", expand=True)

        left_col = tk.Frame(outer)
        left_col.grid(row=0, column=0, sticky="n", padx=(0, 15))

        cnc_frame = tk.LabelFrame(left_col, text="CNC (Klipper)", padx=10, pady=10)
        cnc_frame.pack(fill="x", pady=(0, 15))
        self._setup_cnc_ui(cnc_frame)

        platform_frame = tk.LabelFrame(left_col, text="Stewart Platform", padx=10, pady=10)
        platform_frame.pack(fill="x", pady=(0, 15))
        self._setup_platform_ui(platform_frame)

        cycle_frame = tk.LabelFrame(left_col, text="Auto Cycle", padx=10, pady=10)
        cycle_frame.pack(fill="x")
        self._setup_cycle_ui(cycle_frame)

        heatmap_frame = tk.LabelFrame(outer, text="Charge Heatmap (live, one window)", padx=5, pady=5)
        heatmap_frame.grid(row=0, column=1, sticky="nsew")
        outer.grid_columnconfigure(1, weight=3)
        outer.grid_rowconfigure(0, weight=1)

        camera_frame = tk.LabelFrame(outer, text="Last Image (captured before each raster)", padx=5, pady=5)
        camera_frame.grid(row=0, column=2, sticky="nsew", padx=(10, 0))
        outer.grid_columnconfigure(2, weight=2)

        extent = (
            self.start_x - self.circle_radius, self.start_x + self.circle_radius,
            self.start_y, self.start_y + 2 * self.circle_radius,
        )
        # >>> TUNABLE: fixed color-scale range. Since it's not yet known how
        # strongly charged these particles will actually get, this is a
        # generously wide placeholder - narrow it once you've seen real
        # readings, so the color scale actually resolves meaningful detail
        # instead of compressing everything into a small slice of the range.
        self.heatmap = LiveHeatmap(heatmap_frame, extent=extent, resolution=150,
                            point_radius=self.sensor_radius)
        self.camera_view = CameraView(camera_frame, display_size=(320, 240))

    def _setup_cnc_ui(self, frame):
        self.coord_label = tk.Label(frame, text="X: 0.00  |  Y: 0.00", font=('Helvetica', 16, 'bold'), fg="blue")
        self.coord_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        tk.Button(frame, text="Home All (G28)", bg="yellow", command=self.home_machine).grid(row=1, column=0, columnspan=3, sticky="we")
        tk.Button(frame, text="EMERGENCY STOP", bg="red", fg="white", font=('Helvetica', 12, 'bold'), command=self.e_stop).grid(row=2, column=0, columnspan=3, pady=(10, 5), sticky="we")
        tk.Button(frame, text="Firmware Restart", bg="orange", command=self.firmware_restart).grid(row=3, column=0, columnspan=3, pady=5, sticky="we")

        tk.Button(frame, text=f"Draw {self.circle_radius*2}mm Circle", bg="lightblue", command=self.draw_circle).grid(row=4, column=0, columnspan=3, pady=(10, 5), sticky="we")
        tk.Button(frame, text="Raster Area (manual)", bg="lightgreen", command=self.raster_sensor).grid(row=5, column=0, columnspan=3, pady=5, sticky="we")

        self.timer_label = tk.Label(frame, text="Last Raster Time: --.-- s", font=('Helvetica', 12, 'bold'), fg="green")
        self.timer_label.grid(row=6, column=0, columnspan=3, pady=(10, 0))

    def _setup_platform_ui(self, frame):
        tk.Label(frame, text="Port:").grid(row=0, column=0, sticky="e")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=0, column=1, padx=5)
        self._refresh_ports()
        tk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2)

        self.platform_status_label = tk.Label(frame, text="Not connected", fg="red")
        self.platform_status_label.grid(row=1, column=0, columnspan=3, pady=(5, 5))

        tk.Button(frame, text="Connect", bg="lightyellow", command=self._connect_platform).grid(row=2, column=0, columnspan=3, sticky="we", pady=(0, 10))

        tk.Button(frame, text="Home (Z - manual vertical)", command=self._platform_home).grid(row=3, column=0, columnspan=3, sticky="we")
        tk.Button(frame, text="Auto-Home (H - hall sensors)", command=self._platform_hall_home).grid(row=4, column=0, columnspan=3, sticky="we", pady=(5, 0))
        tk.Button(frame, text="Level (S)", command=lambda: self.platform.level()).grid(row=5, column=0, columnspan=3, sticky="we", pady=(5, 0))

        self.accel_label = tk.Label(frame, text="Accel: -- / -- / -- g", fg="gray")
        self.accel_label.grid(row=6, column=0, columnspan=3, pady=(10, 0))

    def _setup_cycle_ui(self, frame):
        tk.Label(frame, text="Wiggle tilt (deg):").grid(row=0, column=0, sticky="e")
        self.wiggle_tilt_var = tk.StringVar(value="10")
        tk.Entry(frame, textvariable=self.wiggle_tilt_var, width=8).grid(row=0, column=1)

        tk.Label(frame, text="Wiggle period (s):").grid(row=1, column=0, sticky="e")
        self.wiggle_period_var = tk.StringVar(value="4")
        tk.Entry(frame, textvariable=self.wiggle_period_var, width=8).grid(row=1, column=1)

        tk.Label(frame, text="Rotations per wiggle:").grid(row=2, column=0, sticky="e")
        self.wiggle_rotations_var = tk.StringVar(value="3")
        tk.Entry(frame, textvariable=self.wiggle_rotations_var, width=8).grid(row=2, column=1)

        tk.Label(frame, text="Level settle delay (s):").grid(row=3, column=0, sticky="e")
        self.settle_delay_var = tk.StringVar(value="3")
        tk.Entry(frame, textvariable=self.settle_delay_var, width=8).grid(row=3, column=1)

        tk.Label(frame, text="Re-home CNC every N cycles:").grid(row=4, column=0, sticky="e")
        self.rehome_every_n_var = tk.StringVar(value="1")
        tk.Entry(frame, textvariable=self.rehome_every_n_var, width=8).grid(row=4, column=1)

        self.indefinite_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame, text="Run indefinitely", variable=self.indefinite_var,
                       command=self._toggle_cycle_count_entry).grid(row=5, column=0, columnspan=2, sticky="w")

        tk.Label(frame, text="Num cycles:").grid(row=6, column=0, sticky="e")
        self.num_cycles_var = tk.StringVar(value="10")
        self.num_cycles_entry = tk.Entry(frame, textvariable=self.num_cycles_var, width=8, state="disabled")
        self.num_cycles_entry.grid(row=6, column=1)

        tk.Button(frame, text="Start Auto Cycle", bg="lightgreen", command=self._start_cycle).grid(row=7, column=0, columnspan=2, sticky="we", pady=(10, 2))
        tk.Button(frame, text="Stop Auto Cycle", bg="salmon", command=self._stop_cycle).grid(row=8, column=0, columnspan=2, sticky="we")

        self.cycle_status_label = tk.Label(frame, text="Idle", font=('Helvetica', 11, 'bold'), fg="gray")
        self.cycle_status_label.grid(row=9, column=0, columnspan=2, pady=(10, 0))

    # ---------------- platform ----------------

    def _refresh_ports(self):
        ports = PlatformController.list_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _startup_sequence(self):
        """Runs once, on a background thread, at app launch, IN ORDER:
        1. Home the CNC (G28) and wait for real confirmation it's homed.
        2. Connect to the Stewart platform (auto-detect, with a hardcoded
           fallback path if that doesn't find it).
        3. Auto-home the platform via its hall sensors (H).
        Consolidated into one sequential flow (rather than the previous
        independent CNC-home + platform-connect calls) because step 3 now
        depends on step 2 having succeeded, and it reads more clearly this
        way than three separately-triggered background threads."""
        print("Homing CNC...")
        self.klipper.send_gcode("G28")
        homed = self._wait_for_cnc_homed(timeout=60)
        if not homed:
            print("[startup] CNC did not confirm home within timeout - "
                  "skipping platform auto-home. Home manually once it's ready.")
            return

        print("[startup] CNC homed. Connecting to Stewart platform...")
        port = self.platform.auto_connect()
        if port:
            self.root.after(0, lambda: (
                self.port_var.set(port),
                self.platform_status_label.config(text=f"Auto-connected: {port}", fg="green")
            ))
        else:
            self.root.after(0, lambda: self.platform_status_label.config(
                text="Auto-connect failed - select port manually", fg="orange"))
            print("[startup] Platform auto-connect failed - skipping auto-home. "
                  "Connect manually, then use the Auto-Home button.")
            return

        print("[startup] Platform connected. Auto-homing via hall sensors...")
        self.platform.hall_home()

    def _wait_for_cnc_homed(self, timeout=60):
        start = time.time()
        while time.time() - start < timeout:
            if self.klipper.is_fully_homed("xy"):
                return True
            time.sleep(0.3)
        return False

    def _connect_platform(self):
        port = self.port_var.get()
        if not port:
            self.platform_status_label.config(text="No port selected", fg="red")
            return
        ok = self.platform.connect(port)
        if ok:
            self.platform_status_label.config(text=f"Connected: {port}", fg="green")
        else:
            self.platform_status_label.config(text=f"Failed to connect: {port}", fg="red")

    def _platform_home(self):
        threading.Thread(target=self.platform.home, daemon=True).start()

    def _platform_hall_home(self):
        threading.Thread(target=self.platform.hall_home, daemon=True).start()

    def _poll_accel_loop(self):
        """Runs forever on its own background thread (serial reads are
        blocking calls, so this can't run on the main thread). Polls the
        MMA8451 at ~2Hz whenever the platform is connected and only hops to
        the main thread to update the label text."""
        while True:
            if self.platform.connected:
                reading = self.platform.read_accel(timeout=1.0)
                if reading is not None:
                    x, y, z = reading
                    text = f"Accel: {x:.2f} / {y:.2f} / {z:.2f} g"
                    self.root.after(0, lambda t=text: self.accel_label.config(text=t, fg="black"))
                else:
                    self.root.after(0, lambda: self.accel_label.config(
                        text="Accel: -- / -- / -- g (no reading)", fg="red"))
            else:
                self.root.after(0, lambda: self.accel_label.config(
                    text="Accel: -- / -- / -- g", fg="gray"))
            time.sleep(0.5)

    def _on_close(self):
        """Clean shutdown: release the DAQ hardware handle, the camera, and
        the platform's serial connection before the window actually closes."""
        try:
            self.cycle.stop()
        except Exception:
            pass
        if self.daq is not None:
            try:
                self.daq.close()
            except Exception:
                pass
        try:
            self.camera.close()
        except Exception:
            pass
        try:
            self.platform.disconnect()
        except Exception:
            pass
        self.root.destroy()

    # ---------------- heatmap plumbing ----------------

    def _drain_heatmap_queue(self):
        """Runs on the MAIN thread via root.after - the only place points
        from the queue actually get drawn."""
        drained_any = False
        try:
            while True:
                x, y, charge = self.point_queue.get_nowait()
                self.heatmap.add_point(x, y, charge)
                drained_any = True
        except queue.Empty:
            pass
        if drained_any:
            self.heatmap.refresh()
        self.root.after(100, self._drain_heatmap_queue)  # ~10Hz redraw rate

    def _on_raster_start(self):
        """Called from a BACKGROUND THREAD (either the cycle's thread or
        the manual raster's thread - see raster_sensor()/_run_manual_raster
        below, which now call this off the main thread specifically so this
        capture can't freeze the GUI). Captures a photo right at the
        'confirmed level, about to raster' moment (a blocking hardware
        call, fine here since we're off the main thread), saves it to disk
        tagged with the raster_number it'll be paired with in the CSV, then
        hops to the main thread to update both the heatmap and the camera
        view."""
        frame = None
        try:
            frame = self.camera.capture()
        except Exception as e:
            print(f"[camera] Capture failed: {e}")

        next_n = self.session_logger.peek_next_raster_number()
        self.last_image_path = self.session_logger.save_image(frame, next_n)

        self.root.after(0, lambda: self._apply_raster_start_ui(frame))

    def _apply_raster_start_ui(self, frame):
        self.heatmap.clear(title="Rastering...")
        if frame is not None:
            self.camera_view.show_array(frame)

    def _on_raster_done(self, samples):
        """Called from the cycle's background thread. Saving to disk is
        plain file I/O (no GUI involved) so it's fine to do it right here,
        off the main thread. Only the final heatmap title update needs to
        hop back."""
        if samples:
            image_path = getattr(self, "last_image_path", "")
            path, raster_number = self.session_logger.log_raster(samples, image_path=image_path)
            print(f"[data] Raster #{raster_number}: appended {len(samples)} samples to {path}")
            title = f"Raster #{raster_number}: {len(samples)} samples"
        else:
            title = "Last raster: no charge samples (no ADC connected yet)"
        self.root.after(0, lambda: self.heatmap.refresh(title=title))

    # ---------------- auto cycle ----------------

    def _toggle_cycle_count_entry(self):
        self.num_cycles_entry.config(state="disabled" if self.indefinite_var.get() else "normal")

    def _start_cycle(self):
        try:
            tilt = float(self.wiggle_tilt_var.get())
            period = float(self.wiggle_period_var.get())
            rotations = float(self.wiggle_rotations_var.get())
            settle_delay = float(self.settle_delay_var.get())
            rehome_every_n = int(self.rehome_every_n_var.get())
            if rehome_every_n < 1:
                raise ValueError("rehome_every_n must be >= 1")
            num_cycles = None if self.indefinite_var.get() else int(self.num_cycles_var.get())
        except ValueError:
            self.cycle_status_label.config(text="Invalid cycle settings", fg="red")
            return

        raster_params = (self.start_x, self.start_y, self.circle_radius,
                          self.raster_stepover, self.fast_speed)
        self.cycle.start(tilt, period, rotations, raster_params, num_cycles,
                          rehome_every_n=rehome_every_n,
                          level_settle_delay_s=settle_delay)

    def _stop_cycle(self):
        self.cycle.stop()

    def _on_cycle_status(self, text):
        self.root.after(0, lambda: self.cycle_status_label.config(text=text, fg="orange"))

    # ---------------- original CNC methods (unchanged logic, manual raster
    # now goes through RasterRecorder too, so it shares the exact same path
    # and heatmap as the auto-cycle) ----------------

    def update_coordinates(self):
        x, y = self.klipper.get_position()
        if x is None or y is None:
            self.coord_label.config(text="X: --.--  |  Y: --.--", fg="red")
        else:
            self.coord_label.config(text=f"X: {x:.2f}  |  Y: {y:.2f}", fg="blue")
        self.root.after(500, self.update_coordinates)

    def draw_circle(self):
        gcode = f"G90\nG1 X{self.start_x} Y{self.start_y} F{self.fast_speed}\nG3 I0 J{self.circle_radius} F{self.fast_speed}\nG1 X0 Y0 F{self.fast_speed}"
        print(f"Snapping to X{self.start_x} Y{self.start_y}, drawing {self.circle_radius*2}mm Circle, returning to 0,0...")
        self.klipper.send_gcode(gcode)

    def raster_sensor(self):
        setup_gcode, raster_gcode, end_gcode, last_x, last_y = build_raster_gcode(
            self.start_x, self.start_y, self.circle_radius, self.raster_stepover, self.fast_speed
        )
        self.timer_label.config(text="Rastering... Timer Active", fg="orange")
        # NOTE: _on_raster_start() now does a camera capture (a blocking
        # hardware call), so it's called from the background thread below,
        # not directly here - otherwise a slow capture would freeze the GUI.
        threading.Thread(
            target=self._run_manual_raster,
            args=(setup_gcode, raster_gcode, end_gcode, self.start_x, self.start_y, last_x, last_y),
            daemon=True
        ).start()

    def _run_manual_raster(self, setup_gcode, raster_gcode, end_gcode, start_x, start_y, end_x, end_y):
        self._on_raster_start()
        start_time = time.time()
        samples = self.raster_recorder.run_raster(
            setup_gcode, raster_gcode, end_gcode, start_x, start_y, end_x, end_y
        )
        elapsed_time = time.time() - start_time
        print(f"Raster complete! Total physical time: {elapsed_time:.2f} seconds.")
        self.root.after(0, lambda: self.timer_label.config(text=f"Last Raster Time: {elapsed_time:.2f} s", fg="green"))
        if samples:
            self._on_raster_done(samples)

    def home_machine(self):
        print("Homing machine...")
        self.klipper.send_gcode("G28")

    def e_stop(self):
        print("EMERGENCY STOP ACTIVATED")
        self.klipper.send_gcode("M112")
        self.cycle.stop()
        self.platform.stop_now()
        if self.daq is not None:
            self.daq.stop_scan()
        self.timer_label.config(text="E-STOP TRIGGERED", fg="red")

    def firmware_restart(self):
        print("Restarting firmware to clear shutdown state...")
        self.klipper.send_gcode("FIRMWARE_RESTART")