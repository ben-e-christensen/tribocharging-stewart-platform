# Octopus Stewart Platform Firmware - Stage 4

Bring-up project for driving your rotary Stewart platform's six NEMA17
steppers from a BigTreeTech Octopus V1.1 (STM32F446ZET6), using PlatformIO
+ the Arduino/STM32Duino framework instead of Klipper or Marlin.

This stage adds real inverse kinematics: send a target platform pose over
serial, and all six legs move to reach it. All the geometry (base pivots,
shaft directions, platform attachment points, the confirmed motor<->hole
pairing, the 60-degree assembly twist, the rod length and crank throw) lives
in `kinematics.h` - see that file's header comment for the full derivation.

## Hard-won lessons baked into this version (from stages 1-3)

1. **`driver.toff(4)` is mandatory** - TMC2209's output stage defaults to
   disabled at power-up; only a UART write turns it on.
2. **Use `SoftwareSerial`, not `HardwareSerial`** for the per-driver UART
   pins - none of them are real hardware USART pins on this chip.
3. **Each driver socket's jumper block must be set to UART mode, not SPI
   mode** - TMC2209 has no SPI interface at all.

## Serial commands (115200 baud)

- `Z` - home/zero. **Physically position the platform level, centered, and
  at the neutral height (~171.74mm) before sending this.** It tells the
  firmware "this is the neutral pose" and computes each leg's reference
  crank angle from it - everything after is tracked relative to this.
- `P x y z roll pitch yaw` - command a new target pose. Position in mm,
  relative to the neutral position (so `P 0 0 0 0 0 0` returns to neutral),
  angles in degrees. Example: `P 0 0 0 5 0 0` tilts 5 degrees of roll.
  Sending a `P` command cancels any active orbit (`W`).
- `W tilt_deg [period_sec]` - orbital tilt: holds a constant tilt magnitude
  while continuously rotating the tilt *direction* around a full circle -
  like swirling a marble around the rim of a bowl, rather than tipping the
  platform once and holding still. `period_sec` is how long one full
  revolution takes (default 4 seconds if omitted). Example: `W 10 3` orbits
  a 10-degree tilt once every 3 seconds. **16.5 degrees is the largest tilt
  magnitude that stays reachable at every point around the full circle**,
  confirmed via `workspace_sweep.py`'s `max_safe_orbit_radius()` - go
  higher and the platform will stutter/hold at whichever phase angles push
  past the envelope, since those particular poses come back unreachable.
- `S` - stop an active orbit and hold the current position.
- `B` - reboot into the STM32's ROM DFU bootloader so the board can be
  reflashed **without the BOOT0 jumper**. The USB serial port disappears
  and a `0483:df11` DFU device appears in its place. `../flash_octo.py`
  sends this for you. Mechanism: `../firmware_common/dfu_reboot.h`.
- `A` - read the MMA8451 accelerometer wired to the board's onboard I2C
  header (PB8 = SCL, PB9 = SDA). Replies `ACCEL x y z` (g's, 3 decimals),
  or `ACCEL ERR` if the sensor didn't initialize at boot. On-demand only -
  polled by the PC side, not streamed continuously.

You must send `Z` before any `P` or `W` command will be accepted.

## Known gotcha: float parsing in `sscanf`

Some minimal embedded libc configurations disable float support in
`scanf`/`printf` by default to save flash space. If the `P` command's
`sscanf` silently returns fewer than 6 parsed values even with correctly
formatted input, this is almost certainly why - not a firmware logic bug.
Fix: add `-u _scanf_float` (newlib) to your linker flags. Worth checking
early, since the symptom ("parse error" on obviously-correct input) doesn't
point at the real cause on its own.

## Motion model

Each leg moves toward its target step position at a fixed max rate (500
steps/sec) - no acceleration/deceleration profile yet. Fine for a
slow-moving balancing platform; revisit if you want fast, smooth pose
changes later.

## Flashing

From the repo root:
```
python3 flash_octo.py                     # build + flash this project (octo/)
python3 flash_octo.py stewart_accel_cal   # the calibration firmware instead
python3 flash_octo.py --monitor           # ...and open a serial monitor after
```
It builds first (a compile error never leaves the board stuck in DFU),
sends `B` to reboot the running firmware into DFU, flashes with dfu-util
(`:leave`, so the new firmware starts immediately), and waits for the
serial port to come back.

**The BOOT0 jumper is only needed when the board's current firmware
doesn't have `B` yet** (i.e. the first flash after this was added), or to
recover a board that won't boot. The script detects that and prompts:
set `J75`, press reset once, it continues by itself; remove the jumper
afterwards.

One-time setup on a new machine:
1. `pip install platformio` (it fetches its own toolchain + dfu-util).
2. USB permission for the DFU device (the script prints this if missing):
   ```
   sudo cp firmware_common/99-octopus-dfu.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

Manual equivalent: `pio run -e octopus -t upload` with the board already in
DFU mode.

After flashing:

1. Open the serial monitor:
   ```
   pio device monitor -b 115200
   ```
2. Physically level the platform at its neutral height, send `Z`, then try
   `P 0 0 0 5 0 0` to test a small roll motion.

## Project layout

- `platformio.ini` - build config; targets the `octopus` environment.
- `boards/genericSTM32F446ZE.json` - custom board definition (Arduino
  framework support for this exact chip/flash/RAM combination).
- `STM32F446ZETx_FLASH.ld` - linker script STM32Duino doesn't ship for this
  variant.
- `src/kinematics.h` - all geometry constants and the per-leg IK solve.
- `src/main.cpp` - motor/UART setup, homing, serial commands, motion loop.

## Next stages (not yet in this project)

- Acceleration/deceleration profiling for smoother, faster pose changes.
- A richer command set (e.g. smooth interpolated moves between poses,
  rather than each leg independently racing to its own target at a fixed
  rate - fine for small moves, but large simultaneous multi-axis moves
  will currently arrive at slightly different times per leg).


