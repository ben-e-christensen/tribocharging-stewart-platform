#!/usr/bin/env python3
"""
flash_octo.py - one-command build + flash for the BTT Octopus (STM32F446)
firmware projects in this repo (octo/, stewart_accel_cal/ - both go onto
the same physical board).

    python3 flash_octo.py                     # build + flash octo/
    python3 flash_octo.py stewart_accel_cal   # build + flash the cal firmware
    python3 flash_octo.py --monitor           # ...then open a serial monitor
    python3 flash_octo.py --jumper            # skip 'B', use BOOT0 jumper

Pipeline:
  1. `pio run` - build first, so a compile error never leaves the board
     sitting in DFU mode with nothing to flash.
  2. Get the board into its ROM DFU bootloader (USB 0483:df11):
       - already there?             -> use it
       - running our firmware?      -> send 'B' over serial (see
                                       firmware_common/dfu_reboot.h)
       - neither / 'B' didn't work  -> print BOOT0-jumper instructions and
                                       wait for the DFU device to appear
     The jumper path is needed exactly once: the first flash onto a board
     whose current firmware predates the 'B' command.
  3. `pio run -t upload` (dfu-util with :leave, so the new firmware starts
     immediately - no reset press needed).
  4. Wait for the USB serial port to come back and report it.

Requires PlatformIO Core (`pip install platformio`) and pyserial. Needs USB
permission on the DFU device - if missing, this prints the one-time udev
fix (see firmware_common/99-octopus-dfu.rules).
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import serial
import serial.tools.list_ports

REPO = os.path.dirname(os.path.abspath(__file__))
UDEV_RULES = os.path.join(REPO, "firmware_common", "99-octopus-dfu.rules")

DFU_VID, DFU_PID = 0x0483, 0xDF11   # STM32 ROM bootloader
CDC_VID, CDC_PID = 0x0483, 0x5740   # STM32duino USB-CDC (our firmware)
# The CNC's Pico (Klipper) also shows up as a USB serial port - never touch it.
EXCLUDE = ("klipper", "rp2040")


def find_dfu_device():
    """Returns the /dev/bus/usb/BBB/DDD path of the first STM32 DFU device,
    or None. Reads sysfs directly so there's no pyusb/lsusb dependency."""
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                vid = int(f.read(), 16)
            with open(os.path.join(dev, "idProduct")) as f:
                pid = int(f.read(), 16)
            if (vid, pid) != (DFU_VID, DFU_PID):
                continue
            with open(os.path.join(dev, "busnum")) as f:
                bus = int(f.read())
            with open(os.path.join(dev, "devnum")) as f:
                devnum = int(f.read())
            return f"/dev/bus/usb/{bus:03d}/{devnum:03d}"
        except (OSError, ValueError):
            continue
    return None


def find_octopus_port():
    for p in serial.tools.list_ports.comports():
        haystack = f"{p.description} {p.hwid} {p.manufacturer or ''}".lower()
        if any(ex in haystack for ex in EXCLUDE):
            continue
        if (p.vid, p.pid) == (CDC_VID, CDC_PID):
            return p.device
    return None


def wait_for(fn, timeout, poll=0.25):
    start = time.time()
    while time.time() - start < timeout:
        result = fn()
        if result:
            return result
        time.sleep(poll)
    return None


def run(cmd):
    print(f"$ {' '.join(cmd)}")
    return subprocess.call(cmd)


def request_dfu_over_serial(port):
    """Sends 'B'. Returns True if the firmware acknowledged it (older
    firmware without 'B' replies 'Unknown command...' instead)."""
    print(f"Asking firmware on {port} to reboot into DFU ('B')...")
    try:
        # Opening with DTR asserted matters - see monitor_dtr in platformio.ini.
        with serial.Serial(port, 115200, timeout=1) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.write(b"B\n")
            ser.flush()
            deadline = time.time() + 2
            while time.time() < deadline:
                try:
                    line = ser.readline().decode(errors="ignore").strip()
                except serial.SerialException:
                    return True  # port vanished mid-read - that's the reboot
                if not line:
                    continue
                print(f"  [board] {line}")
                if "DFU" in line:
                    return True
                if "Unknown command" in line:
                    return False
    except serial.SerialException as e:
        # Port dropping out from under us right after the write is expected.
        if find_dfu_device():
            return True
        print(f"  serial error: {e}")
    return False


def get_into_dfu(port_override, use_jumper):
    dev = find_dfu_device()
    if dev:
        print(f"Board already in DFU mode ({dev}).")
        return dev

    if not use_jumper:
        port = port_override or find_octopus_port()
        if port:
            if request_dfu_over_serial(port):
                dev = wait_for(find_dfu_device, timeout=8)
                if dev:
                    print(f"Board is in DFU mode ({dev}).")
                    return dev
                print("Board acknowledged 'B' but no DFU device appeared.")
            else:
                print("Firmware on the board doesn't support 'B' yet "
                      "(expected the first time) - falling back to the jumper.")
        else:
            print("No Octopus serial port found.")

    print("\n  >>> Put the board in DFU mode by hand:\n"
          "      1. Fit the J75 BOOT0 jumper\n"
          "      2. Press the reset button once\n"
          "      Waiting up to 2 minutes for the DFU device (Ctrl-C to abort)...\n")
    dev = wait_for(find_dfu_device, timeout=120)
    if dev:
        print(f"Board is in DFU mode ({dev}). "
              "REMEMBER to remove the BOOT0 jumper after flashing.")
    return dev


def check_dfu_permission(dev):
    if os.access(dev, os.W_OK):
        return True
    print(f"\nNo write permission on {dev} - dfu-util will fail.\n"
          "One-time fix (installs a udev rule for the STM32 DFU bootloader):\n"
          f"  sudo cp {UDEV_RULES} /etc/udev/rules.d/\n"
          "  sudo udevadm control --reload-rules && sudo udevadm trigger\n"
          "Then re-run this script (the board is still in DFU mode, so it'll "
          "go straight to flashing).")
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("project", nargs="?", default="octo",
                   help="PlatformIO project dir (default: octo)")
    p.add_argument("-e", "--env", default="octopus", help="PlatformIO env (default: octopus)")
    p.add_argument("--port", help="serial port of the running firmware (default: auto-detect)")
    p.add_argument("--jumper", action="store_true",
                   help="don't send 'B'; wait for a manual BOOT0-jumper DFU entry")
    p.add_argument("--no-build", action="store_true", help="skip the separate build step")
    p.add_argument("--monitor", action="store_true", help="open `pio device monitor` afterwards")
    a = p.parse_args()

    project = a.project if os.path.isabs(a.project) else os.path.join(REPO, a.project)
    if not os.path.isfile(os.path.join(project, "platformio.ini")):
        sys.exit(f"{project} is not a PlatformIO project (no platformio.ini).")
    pio = shutil.which("pio") or shutil.which("platformio")
    if not pio:
        sys.exit("PlatformIO not found - `pip install platformio`.")

    if not a.no_build:
        if run([pio, "run", "-d", project, "-e", a.env]) != 0:
            sys.exit("Build failed - board left untouched.")

    dev = get_into_dfu(a.port, a.jumper)
    if not dev:
        sys.exit("Board never entered DFU mode.")
    if not check_dfu_permission(dev):
        sys.exit(1)

    if run([pio, "run", "-d", project, "-e", a.env, "-t", "upload"]) != 0:
        sys.exit("Upload failed. The board is probably still in DFU mode - "
                 "fix the problem and re-run.")

    print("\nFlashed. Waiting for the firmware's serial port...")
    port = wait_for(find_octopus_port, timeout=15)
    if port:
        print(f"Firmware is up on {port}.")
    else:
        print("Serial port didn't come back. If you used the BOOT0 jumper, "
              "remove it and press reset.")
        sys.exit(1)

    if a.monitor:
        os.execvp(pio, [pio, "device", "monitor", "-d", project, "-e", a.env, "-p", port])


if __name__ == "__main__":
    main()
