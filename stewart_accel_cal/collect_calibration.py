"""
collect_calibration.py - runs one calibration sweep against the Stewart
platform's accelerometer-calibration firmware (this project's src/main.cpp,
NOT the ../octo driving firmware) and logs the result to a timestamped CSV.

Deliberately standalone (no import from rig_controller_pkg) - this project
is meant to stay separate from the main rig-control app, and is small enough
that duplicating the ~20-line serial-port-finding helper is cheaper than
coupling the two.

This is an OFFLINE data-collection tool, not a live control loop: connect,
home, run one sweep, save a CSV, disconnect. Look at the CSV, decide what to
change (geometry constants, mounting, etc.), reflash/rerun as needed.

Usage:
    python collect_calibration.py [--port PORT] [--home Z|H]

Output: data/session_<timestamp>.csv, columns:
    cmd_roll_deg, cmd_pitch_deg, cmd_yaw_deg, accel_x_g, accel_y_g,
    accel_z_g, n_samples, timestamp
Poses the firmware skipped (unreachable) or failed to sample are logged to
data/session_<timestamp>_issues.csv instead (cmd_roll_deg, cmd_pitch_deg,
status), so a partial sweep still produces a clean primary CSV.
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

DATA_DIR = Path(__file__).parent / "data"


def find_platform_port(match_substrings=("F446", "STMicroelectronics"),
                        exclude_substrings=("Klipper", "rp2040")):
    """Same descriptor-matching approach as rig_controller_pkg's
    PlatformController - scans for the Octopus's USB descriptor while
    excluding anything that looks like a different board's MCU."""
    for p in serial.tools.list_ports.comports():
        haystack = f"{p.description} {p.hwid}".lower()
        if any(ex.lower() in haystack for ex in exclude_substrings):
            continue
        if any(m.lower() in haystack for m in match_substrings):
            return p.device
    return None


def connect(port, baud=115200):
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2)  # let the board settle after opening the port
    ser.reset_input_buffer()
    return ser


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()


def read_line(ser):
    return ser.readline().decode(errors="ignore").strip()


def wait_for(ser, markers, error_markers, timeout):
    """Reads lines until one contains a marker or error_marker substring.
    Prints everything as it comes in. Returns (ok, last_line)."""
    start = time.time()
    while time.time() - start < timeout:
        line = read_line(ser)
        if not line:
            continue
        print(f"[platform] {line}")
        if any(m in line for m in error_markers):
            return False, line
        if any(m in line for m in markers):
            return True, line
    return False, "TIMEOUT"


def run_home(ser, mode):
    cmd = "H" if mode == "H" else "Z"
    print(f"Homing ({cmd})...")
    send(ser, cmd)
    ok, last = wait_for(ser, ["Ready."], ["ERROR", "ABORTED"], timeout=120)
    return ok


def run_sweep(ser, rows, issues, timeout_s=2400):
    send(ser, "C")
    start = time.time()
    while time.time() - start < timeout_s:
        line = read_line(ser)
        if not line:
            continue
        print(f"[platform] {line}")

        if line == "CAL_START":
            continue
        if line == "CAL_DONE":
            return True
        if line.startswith("CAL_ABORT"):
            print("[!] Firmware aborted the sweep (mid-sweep rehome failed).")
            return False
        if line.startswith("CAL_SKIP"):
            parts = line.split()
            issues.append({"cmd_roll_deg": parts[1], "cmd_pitch_deg": parts[2],
                            "status": "unreachable"})
            continue
        if line.startswith("CAL_ERR"):
            parts = line.split()
            issues.append({"cmd_roll_deg": parts[1], "cmd_pitch_deg": parts[2],
                            "status": "accel_sample_failed"})
            continue
        if line.startswith("CAL "):
            parts = line.split()
            if len(parts) != 8:
                print(f"[!] unexpected CAL line shape, skipping: {line}")
                continue
            _, roll, pitch, yaw, ax, ay, az, n = parts
            rows.append({
                "cmd_roll_deg": roll,
                "cmd_pitch_deg": pitch,
                "cmd_yaw_deg": yaw,
                "accel_x_g": ax,
                "accel_y_g": ay,
                "accel_z_g": az,
                "n_samples": n,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            })
            continue
        # anything else (UART test_connection lines, WARNINGs, etc.) - just
        # printed above for visibility, nothing to log.
    print("[!] Sweep timed out waiting for CAL_DONE.")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port; auto-detected if omitted")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--home", choices=["Z", "H"], default="H",
                     help="Z = you've already set horns vertical by hand; "
                          "H = auto-home via hall sensors (default)")
    args = ap.parse_args()

    port = args.port or find_platform_port()
    if port is None:
        print("[!] Could not auto-detect the platform's serial port. "
              "Pass --port explicitly.")
        sys.exit(1)

    print(f"Connecting on {port}...")
    ser = connect(port, args.baud)

    if not run_home(ser, args.home):
        print("[!] Homing failed or timed out - aborting.")
        ser.close()
        sys.exit(1)

    rows = []
    issues = []
    ok = run_sweep(ser, rows, issues)
    ser.close()

    DATA_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    csv_path = DATA_DIR / f"session_{stamp}.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "cmd_roll_deg", "cmd_pitch_deg", "cmd_yaw_deg",
            "accel_x_g", "accel_y_g", "accel_z_g", "n_samples", "timestamp",
        ])
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {len(rows)} rows to {csv_path}")

    if issues:
        issues_path = DATA_DIR / f"session_{stamp}_issues.csv"
        with open(issues_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["cmd_roll_deg", "cmd_pitch_deg", "status"])
            writer.writeheader()
            writer.writerows(issues)
        print(f"Wrote {len(issues)} skipped/failed poses to {issues_path}")

    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
