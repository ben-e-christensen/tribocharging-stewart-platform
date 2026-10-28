# Stewart Platform Accelerometer Calibration/Validation

A deliberately separate PlatformIO project from `../octo` (the driving
firmware). This one exists to answer a single question: **how accurate is
the IK's assumption that the platform actually reaches the roll/pitch it
was commanded to?**

`../octo/src/kinematics.h` solves crank angles from a commanded pose and
just trusts that the physical platform gets there - no closed-loop check
against anything. This project drives a grid of known test poses, reads
the onboard MMA8451 accelerometer at each one once it settles, and logs
both the commanded pose and the raw measurement so the two can be compared
offline.

**This is not live feedback/control.** Nothing here corrects the platform
while it runs. The workflow is: run a sweep, collect a CSV, look at it,
change something (geometry constants, `NEUTRAL_Z`, sensor mounting
assumptions, whatever the data points at), reflash/rerun. Iterate.

## Why a whole separate project instead of adding a mode to `../octo`

- Keeps the driving firmware's command set (`P`/`W`/`J`/`S`) untouched and
  uncluttered by calibration-only logic.
- The two firmwares are mutually exclusive on the same physical board
  anyway (only one build is flashed at a time) - no benefit to sharing a
  binary, and real cost (flash space, command-set collisions, harder to
  reason about) to trying.
- `src/kinematics.h` is copied verbatim from `../octo` (same physical
  machine, same geometry) - if you change the geometry there, copy the
  updated file over here too before your next sweep, or the two firmwares'
  IK will silently diverge.

## Hardware

Identical to `../octo`: BigTreeTech Octopus V1.1 (STM32F446ZET6), six
TMC2209-driven NEMA17 legs, MMA8451 accelerometer on the onboard I2C header
(PB8 = SCL, PB9 = SDA). Same board quirks (12MHz crystal, custom linker
script/board manifest, DFU upload) - see `../octo/README.md` if you need
the details, they're not repeated here.

## Firmware serial commands (115200 baud)

- `Z` - home from hand-set vertical (see `../octo/README.md` for the
  physical procedure).
- `H` - auto-home via hall sensors.
- `P x y z roll pitch yaw` - command a single pose. Useful for manual spot
  checks between sweeps.
- `S` - stop, return to neutral.
- `Q` - status query (`SETTLED` / `MOVING`).
- `A` - one-shot raw accelerometer read: `ACCEL x y z` (g's) or `ACCEL ERR`.
- `C` - **run the calibration sweep.** Requires homing first. Drives
  through a 17x17 grid of roll/pitch combinations (2deg steps, -16deg to
  +16deg each way - 289 poses; yaw is skipped entirely - gravity's
  direction doesn't change with yaw, so there's nothing for an
  accelerometer to check there). At each pose: settles, waits an extra
  350ms for mechanical ringing to damp out, averages 12 accelerometer
  samples, and prints:
  ```
  CAL <cmd_roll> <cmd_pitch> <cmd_yaw> <ax> <ay> <az> <n_good_samples>
  ```
  Unreachable poses (mostly the extreme combined-tilt corners) print
  `CAL_SKIP <roll> <pitch>` instead; poses where every accelerometer
  sample failed print `CAL_ERR <roll> <pitch>`. The sweep is bracketed by
  `CAL_START` / `CAL_DONE` and returns to neutral when finished. 289 poses
  at ~530ms of fixed dwell/sampling each (plus a much smaller amount of
  move time between adjacent 2deg poses) comes out to roughly 3 minutes
  total.

  The sweep is open-loop stepping with no per-leg position feedback except
  at a home - a single missed step anywhere would otherwise silently offset
  every pose logged after it for the rest of the run. To bound that, the
  platform returns to level and re-runs the hall auto-home search after
  every spoke before continuing. If that mid-sweep rehome fails (a leg's
  hall sensor problem), the sweep aborts early with `CAL_ABORT` instead of
  `CAL_DONE`, and you need to home again (`Z`/`H`) before sending another
  command.

  **Why stop-and-sample instead of continuous readings during travel:**
  the firmware bit-bangs every step pulse itself, so it always knows each
  leg's exact crank angle - but converting six independent, asynchronously
  -arriving crank angles back into the platform's actual roll/pitch
  (forward kinematics) has no closed form for a Stewart platform and isn't
  implemented here. At a *held* pose, that problem doesn't exist - the
  commanded roll/pitch you solved IK for **is** the answer, so stopping to
  sample sidesteps needing forward kinematics at all. Denser stops (289
  instead of the original 49) is the practical middle ground for getting
  more data points without adding a numerical FK solve.

**Important:** `C` reports **raw** `(ax, ay, az)`, not a computed
roll/pitch. The MMA8451's mounting orientation relative to the platform's
own roll/pitch/yaw axes hasn't been independently confirmed - baking in a
guessed sign convention in firmware would quietly corrupt every row with
no way to tell from the data alone. Work out the actual x/y/z-to-roll/pitch
mapping from the logged data itself (the neutral-pose row should read
close to gravity straight down one axis; a couple of known single-axis
tilts pin down the rest), then apply that mapping on the PC side, where
it's cheap to fix if it turns out wrong.

## PC-side data collection

```
python collect_calibration.py [--port PORT] [--home Z|H]
```

Auto-detects the Octopus's serial port the same way `rig_controller_pkg`'s
`PlatformController` does (USB descriptor match on `F446`/
`STMicroelectronics`, excluding anything that looks like the CNC's Klipper
board), homes it (`H` by default - auto via hall sensors), runs one full
`C` sweep, and writes:

- `data/session_<timestamp>.csv` - one row per successfully-sampled pose:
  `cmd_roll_deg, cmd_pitch_deg, cmd_yaw_deg, accel_x_g, accel_y_g,
  accel_z_g, n_samples, timestamp`
- `data/session_<timestamp>_issues.csv` - only written if any poses were
  skipped (unreachable) or failed to sample: `cmd_roll_deg, cmd_pitch_deg,
  status`

This script is standalone on purpose - it doesn't import from
`rig_controller_pkg`, even though the port-finding logic overlaps. Keeping
this project decoupled from the main rig-control app was an explicit
choice, not an oversight.

Requires `pyserial` (`pip install pyserial`).

## One-time setup

1. Install PlatformIO Core: `pip install platformio`
2. Put the Octopus into DFU bootloader mode (set the `J75` BOOT0 jumper,
   press reset).
3. Build and upload:
   ```
   pio run -e octopus -t upload
   ```
4. Remove the BOOT0 jumper, reset the board again.
5. Physically level the platform, then either send `Z` yourself over a
   serial monitor, or just run `collect_calibration.py` with `--home H`
   (default) to let the hall sensors find vertical automatically.
6. Run `python collect_calibration.py`.

## Project layout

- `platformio.ini` - build config; same board target as `../octo`.
- `boards/genericSTM32F446ZE.json`, `STM32F446ZETx_FLASH.ld` - copied
  verbatim from `../octo` (same board).
- `src/kinematics.h` - copied verbatim from `../octo` (same machine
  geometry) - **keep in sync manually if the geometry ever changes.**
- `src/main.cpp` - homing, pose commands, and the `C` calibration sweep.
- `collect_calibration.py` - PC-side sweep runner + CSV logger.
- `data/` - sweep output CSVs land here.

## Next steps (not yet done)

- Work out the MMA8451's actual mounting-orientation mapping to
  roll/pitch from a first collected dataset (see note above).
- Once that mapping is known, a small offline analysis script (measured vs.
  commanded tilt, per-axis error) would be the natural next addition here -
  intentionally not built yet, since it needs real data first to know
  what's actually worth plotting.
