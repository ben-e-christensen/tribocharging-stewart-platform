# CLAUDE.md — Octopus Stewart Platform Firmware (`octo/`)

Recap of the STM32 firmware driving the six-leg Stewart platform: what's
non-obvious, and bugs/decisions worth knowing before touching it again.
For build/upload steps and the full serial command reference, see
`README.md` — this file focuses on the "why" and the traps, not setup.

## What this firmware does

Runs on a BigTreeTech Octopus V1.1 (STM32F446ZET6) via PlatformIO +
Arduino/STM32Duino. Takes a target platform pose (position + roll/pitch/yaw)
over USB serial, solves inverse kinematics per leg (`kinematics.h`), and
drives six TMC2209-controlled NEMA17 steppers to reach it. Serial protocol:
`Z`/`H` (home), `P` (move), `W` (orbit), `J` (joint wave), `S` (stop), `Q`
(status poll), `A` (read accelerometer), `B` (reboot to DFU for flashing)
— full semantics in `README.md` and the header comment of `main.cpp`.

## Non-obvious bugs/decisions worth knowing

**MMA8451 accelerometer: stock library masked I2C failures as frozen
garbage (fixed 2026-07-30).** The onboard I2C header (PB8 SCL / PB9 SDA)
drives an MMA8451, meant for eventual platform-level confirmation.
`Adafruit_MMA8451::read()` (called internally by `getEvent()`) never
checks whether its I2C transaction actually succeeded; on failure it
silently leaves its read buffer at its initializer value, so a flaky
connection (loose header pin, EMI from the TMC2209s chopping current
nearby) produced the *exact same bogus reading every call* instead of
erroring — and `getEvent()` always returned `true` regardless. The `A`
handler in `main.cpp` now uses a custom `read_accel_checked()` that does
the raw I2C transaction directly (`Wire.beginTransmission`/
`endTransmission`/`requestFrom`) and only reports success when the full
6-byte read actually completed, so a real comms failure now honestly
returns `ACCEL ERR` instead of frozen garbage. `mma_ok` remains only a
boot-time "sensor detected at all" latch by design — a comms failure
after boot doesn't clear it, so a flaky connection can recover on its own
next successful read without needing a full re-`begin()`.

**Hall-sensor homing wires were moved off the DIAG/endstop headers.**
Original wiring used the board's DIAG/endstop pins, but those have
hardware pull-ups baked in that fought the sensor's own pull-up and only
sank the triggered voltage to ~1.6-1.8V (not a clean LOW). Moved to EXP1
(plain GPIO, unused since there's no LCD panel attached), which reads a
clean 0V triggered. Don't put these back on DIAG/endstop headers.

**Hall sensor polarity is a tunable, not a given.** `HALL_ACTIVE_LOW` in
`main.cpp` assumes the sensor pulls LOW near a magnet (true for most cheap
breakout modules) — flip it if a bench test shows the opposite.

**Hall pins are plain `INPUT`, not `INPUT_PULLUP`.** The sensor board
actively drives both states itself; an internal pull-up would just add an
unwanted second pull-up path.

**`sscanf` float parsing can silently fail** — see `README.md`'s "Known
gotcha" section. The symptom (`P` command parses fewer than 6 values on
correctly-formatted input) looks like a firmware logic bug but is
actually a linker/libc flag issue (`-u _scanf_float`).

**Motion has no accel/decel profile yet.** Each leg races toward its
target at a fixed max step rate independently. Fine for this slow-moving
platform; would need reworking for fast, smooth multi-axis pose changes
(legs currently can arrive at slightly different times on large moves).

**`B` (reboot-to-DFU) jumps to ROM from a priority-100 constructor, not
from the command handler.** `B` only stores a magic word in RTC backup
register `BKP19R` and resets; `maybe_jump_to_dfu()` in
`../firmware_common/dfu_reboot.h` runs as `constructor(100)` — after
`SystemInit()` but before STM32duino's own `premain()` (`constructor(101)`)
sets up clocks/USB/SysTick — so the ROM bootloader starts from
essentially reset-state hardware. Jumping directly from the running app
would mean hand-unwinding USB and every enabled IRQ. Verified in the ELF
that it's ordered before `premain` in `.init_array`. The header is shared
with `../stewart_accel_cal` via `-I../firmware_common` in both
`platformio.ini` files. BOOT0 jumper + reset remains the recovery path.

**`env:servo_test` in `platformio.ini` is stale** — it points at
`src/servo_test/`, which no longer exists, so that env won't build.

## Where to look

- `src/main.cpp` — setup, serial command handling, per-leg motion loop,
  accelerometer read.
- `src/kinematics.h` — all geometry constants + the per-leg IK solve
  derivation.
- `README.md` — build/upload steps, full serial command reference, and
  the TMC2209 hard-won lessons (`toff(4)`, `SoftwareSerial` not
  `HardwareSerial`, UART jumper mode).
