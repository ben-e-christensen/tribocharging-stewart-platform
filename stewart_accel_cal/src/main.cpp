// Stewart platform accelerometer calibration/validation firmware.
//
// Purpose: the IK in kinematics.h ASSUMES the platform reaches exactly the
// commanded roll/pitch - no closed-loop verification of that assumption
// exists anywhere in the driving firmware. This build's only job is to
// command a grid of known test poses, read what the onboard MMA8451
// actually measures at each one once things settle, and stream both back
// over serial so the PC side can log it for offline comparison. No live
// correction here on purpose - collect a dataset, then reiterate.
//
// Homing, pose commands, and the raw accelerometer read are carried over
// unchanged from ../octo/src/main.cpp (same board, same wiring). The W
// (orbit) and J (joint-wave) drive modes are deliberately NOT included -
// this firmware only needs to hold still poses long enough to sample.
//
// SERIAL COMMANDS (115200 baud):
//   Z                        - home/zero from hand-set vertical (see ../octo
//                              README for the physical procedure).
//   H                        - auto-home via hall sensors.
//   P x y z roll pitch yaw   - command a single pose (mm / degrees,
//                              relative to neutral). Handy for manual spot
//                              checks between sweeps.
//   S                        - stop, return to neutral.
//   Q                        - status query ("SETTLED" / "MOVING").
//   A                        - one-shot raw accelerometer read: "ACCEL x y z"
//                              (g's) or "ACCEL ERR".
//   C                        - run the full calibration sweep (see below).
//     Requires homing first. Drives level, then out along a set of radial
//     roll/pitch spokes from level (yaw is skipped - gravity alone can't
//     observe yaw, so there's nothing for the accelerometer to check there),
//     settling and averaging several accelerometer samples at each stop, and
//     prints one line per pose:
//       CAL <cmd_roll> <cmd_pitch> <cmd_yaw> <ax> <ay> <az> <n_good_samples>
//     or, if that pose was unreachable:
//       CAL_SKIP <cmd_roll> <cmd_pitch>
//     or, if every sample at a reachable pose failed:
//       CAL_ERR <cmd_roll> <cmd_pitch>
//     After each spoke, the platform returns to level and re-runs the hall
//     auto-home search (see rehome_mid_sweep()) before starting the next
//     spoke, to bound how far open-loop step loss can drift the whole run.
//     If that mid-sweep rehome fails, the sweep aborts early with
//     "CAL_ABORT" instead of "CAL_DONE" and homed state is cleared - rehome
//     (Z/H) before sending another P, S, or C.
//     Sweep starts with "CAL_START" and ends with "CAL_DONE" (after
//     returning to neutral). Deliberately reports raw (ax,ay,az) rather
//     than a computed roll/pitch: the MMA8451's mounting orientation
//     relative to the platform's own roll/pitch/yaw axes hasn't been
//     independently confirmed, so baking in a possibly-wrong sign
//     convention here would quietly corrupt every row. Work out that
//     mapping from the logged data itself (e.g. the neutral-pose row and a
//     couple of known single-axis tilts), then apply it on the PC side
//     where it's easy to fix if it turns out wrong.
//   B                        - reboot into DFU bootloader for reflashing
//                              (no BOOT0 jumper) - see ../flash_octo.py.

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <TMCStepper.h>
#include "kinematics.h"
#include "dfu_reboot.h" // 'B' command: reboot into DFU for flashing, no BOOT0 jumper

constexpr uint8_t NUM_MOTORS = 6;

// Indexed 0-5, matching Driver0-Driver5 sockets. Identical to ../octo.
constexpr uint8_t STEP_PINS[NUM_MOTORS]   = { PF13, PG0,  PF11, PG4, PF9,  PC13 };
constexpr uint8_t DIR_PINS[NUM_MOTORS]    = { PF12, PG1,  PG3,  PC1, PF10, PF0  };
constexpr uint8_t ENABLE_PINS[NUM_MOTORS] = { PF14, PF15, PG5,  PA0, PG2,  PF1  }; // all active-low

// Hall-effect homing sensors, one per leg, on the EXP1 header (see ../octo
// README for why - the DIAG headers' built-in pull-ups fight the sensor).
constexpr uint8_t HALL_PINS[NUM_LEGS] = { PE8, PE9, PE12, PE13, PE14, PE15 };
constexpr bool HALL_ACTIVE_LOW = true;

constexpr float R_SENSE = 0.11f;
constexpr uint8_t DRIVER_ADDRESS = 0b00;
constexpr uint16_t TEST_CURRENT_MA = 1500; // confirmed good running current

// --- MMA8451 accelerometer, on the Octopus's onboard I2C header ---
// PB8 = SCL, PB9 = SDA (that header's pinout on this board).
constexpr uint8_t MMA_SCL_PIN = PB8;
constexpr uint8_t MMA_SDA_PIN = PB9;
Adafruit_MMA8451 mma = Adafruit_MMA8451();
bool mma_ok = false;

SoftwareSerial uartM0(PC4, PC4);
SoftwareSerial uartM1(PD11, PD11);
SoftwareSerial uartM2(PC6, PC6);
SoftwareSerial uartM3(PC7, PC7);
SoftwareSerial uartM4(PF2, PF2);
SoftwareSerial uartM5(PE4, PE4);

TMC2209Stepper driver0(&uartM0, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver1(&uartM1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&uartM2, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver3(&uartM3, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver4(&uartM4, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver5(&uartM5, R_SENSE, DRIVER_ADDRESS);

TMC2209Stepper* drivers[NUM_MOTORS] = { &driver0, &driver1, &driver2,
                                         &driver3, &driver4, &driver5 };
SoftwareSerial*  uarts[NUM_MOTORS]  = { &uartM0, &uartM1, &uartM2,
                                         &uartM3, &uartM4, &uartM5 };

// --- Motion / kinematics state ---
constexpr uint16_t MICROSTEPS = 16;
constexpr float STEP_ANGLE_DEG = 1.8f;
constexpr float STEPS_PER_REV = (360.0f / STEP_ANGLE_DEG) * MICROSTEPS; // 3200
constexpr float STEPS_PER_RAD = STEPS_PER_REV / (2.0f * PI);

constexpr uint32_t STEP_PULSE_US = 20;
constexpr uint32_t MAX_STEP_INTERVAL_US = 2000; // max step rate per leg (500 steps/sec)
constexpr uint32_t SNAP_STEP_INTERVAL_US = 6000;  // ~167 steps/sec for big jumps
constexpr int32_t SNAP_THRESHOLD_STEPS = 45;      // ~5 degrees of remaining travel

float neutral_angle[NUM_LEGS];      // crank angle at the homed neutral pose
float current_angle[NUM_LEGS];      // best-known current angle (from step count)
int32_t current_step_pos[NUM_LEGS] = {0};
int32_t target_step_pos[NUM_LEGS]  = {0};
uint32_t last_step_time[NUM_LEGS]  = {0};
bool homed = false;

// Shared by set_target_pose() and run_calibration_sweep(): solves all six
// legs for a given position+rotation and, if reachable, converts to step
// targets. If unreachable, leaves target_step_pos[] alone (holds position).
bool commit_pose(const Vec3& pos, const Mat3& rot) {
    uint8_t fail_mask = solve_all_legs(pos, rot, current_angle);
    if (fail_mask) return false;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        float delta_from_neutral = current_angle[i] - neutral_angle[i];
        target_step_pos[i] = (int32_t) roundf(delta_from_neutral * STEPS_PER_RAD);
    }
    return true;
}

bool set_target_pose(float x, float y, float z, float roll, float pitch, float yaw) {
    Vec3 pos(x, y, z + NEUTRAL_Z);
    Mat3 rot = rotation_from_rpy_deg(roll, pitch, yaw);
    if (!commit_pose(pos, rot)) {
        Serial.println("WARNING: pose unreachable for at least one leg.");
        return false;
    }
    return true;
}

// One pass of per-leg target-seeking motion, identical to the block in
// loop() below. Factored out so run_calibration_sweep() can block on it
// directly (it isn't running inside loop() while a sweep is in progress -
// process_command() is called synchronously from loop()'s serial-read
// section, same pattern ../octo's homing routines already use).
void step_legs_once() {
    uint32_t now = micros();
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        int32_t delta = target_step_pos[i] - current_step_pos[i];
        if (delta == 0) continue;
        int32_t abs_delta = (delta > 0) ? delta : -delta;
        uint32_t interval = (abs_delta > SNAP_THRESHOLD_STEPS)
                                 ? SNAP_STEP_INTERVAL_US
                                 : MAX_STEP_INTERVAL_US;
        if (now - last_step_time[i] < interval) continue;

        bool forward = delta > 0;
        digitalWrite(DIR_PINS[i], forward ? HIGH : LOW);
        digitalWrite(STEP_PINS[i], HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP_PINS[i], LOW);

        current_step_pos[i] += forward ? 1 : -1;
        last_step_time[i] = now;
    }
}

bool legs_settled() {
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (current_step_pos[i] != target_step_pos[i]) return false;
    }
    return true;
}

// Blocks until every leg has reached its current target_step_pos.
void drive_until_settled() {
    while (!legs_settled()) step_legs_once();
}

// Homing (VERTICAL-REFERENCE model - see ../octo/README.md for the full
// explanation): set every horn dead vertical by hand, then Z drives each
// leg down to the level home computed from NEUTRAL_Z.
constexpr uint32_t HOMING_STEP_INTERVAL_US = 6000; // slow: ~167 steps/sec
constexpr float HORN_VERTICAL_ANGLE_DEG = 90.0f;

void drive_to_level_home_from_vertical() {
    Vec3 neutral_pos(0, 0, NEUTRAL_Z);
    Mat3 identity = rotation_from_rpy_deg(0, 0, 0);
    for (uint8_t i = 0; i < NUM_LEGS; i++) current_angle[i] = 0;
    uint8_t fail_mask = solve_all_legs(neutral_pos, identity, current_angle);
    if (fail_mask) {
        Serial.println("ERROR: neutral pose failed to solve - check geometry / NEUTRAL_Z.");
        return;
    }

    Serial.println("Driving slowly to level home - watch for any horn turning the wrong way.");
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        neutral_angle[i] = current_angle[i];
        float off_rad = HOMING_DIR_SIGN * (current_angle[i] - radians(HORN_VERTICAL_ANGLE_DEG));
        target_step_pos[i] = (int32_t) roundf(off_rad * STEPS_PER_RAD);
    }

    bool moving = true;
    while (moving) {
        moving = false;
        uint32_t now = micros();
        for (uint8_t i = 0; i < NUM_LEGS; i++) {
            if (current_step_pos[i] == target_step_pos[i]) continue;
            moving = true;
            if (now - last_step_time[i] < HOMING_STEP_INTERVAL_US) continue;
            bool forward = target_step_pos[i] > current_step_pos[i];
            digitalWrite(DIR_PINS[i], forward ? HIGH : LOW);
            digitalWrite(STEP_PINS[i], HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP_PINS[i], LOW);
            current_step_pos[i] += forward ? 1 : -1;
            last_step_time[i] = micros();
        }
    }

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        current_step_pos[i] = 0;
        target_step_pos[i] = 0;
    }
    homed = true;
    Serial.println("Homed at level position. Ready.");
}

void do_homing() {
    Serial.println("Homing from VERTICAL (set by hand). Make sure all horns are straight up.");
    for (uint8_t i = 0; i < NUM_LEGS; i++) current_step_pos[i] = 0;
    drive_to_level_home_from_vertical();
}

// --- Hall-sensor auto-homing (identical to ../octo - see that file's
// header comment for the full state-machine rationale). ---
constexpr uint32_t HALL_SEARCH_STEP_INTERVAL_US = 6000;
constexpr int32_t HALL_SEARCH_MAX_STEPS = 1200;

enum HallSearchState { HS_BACK_OUT, HS_SEARCH_FWD, HS_COUNT_WINDOW, HS_CENTER_BACK, HS_DONE, HS_ERROR };

bool hall_triggered(uint8_t leg) {
    bool raw = digitalRead(HALL_PINS[leg]);
    return HALL_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

void hall_search_step(uint8_t leg, bool forward) {
    digitalWrite(DIR_PINS[leg], forward ? HIGH : LOW);
    digitalWrite(STEP_PINS[leg], HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PINS[leg], LOW);
    current_step_pos[leg] += forward ? 1 : -1;
}

// Returns true if every leg was found and the platform reached level home;
// false on any sensor error (caller must not assume homed/neutral state).
bool do_hall_auto_homing() {
    Serial.println("Auto-homing via hall sensors - finding true vertical, all legs together...");

    HallSearchState state[NUM_LEGS];
    int32_t progress[NUM_LEGS]  = {0};
    int32_t window[NUM_LEGS]    = {0};
    int32_t center_left[NUM_LEGS] = {0};

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        state[i] = hall_triggered(i) ? HS_BACK_OUT : HS_SEARCH_FWD;
        progress[i] = 0;
    }

    bool any_active = true;
    while (any_active) {
        any_active = false;
        for (uint8_t i = 0; i < NUM_LEGS; i++) {
            switch (state[i]) {
                case HS_BACK_OUT:
                    if (!hall_triggered(i)) {
                        state[i] = HS_SEARCH_FWD;
                        progress[i] = 0;
                    } else {
                        hall_search_step(i, false);
                        if (++progress[i] > HALL_SEARCH_MAX_STEPS) {
                            Serial.print("ERROR leg "); Serial.print(i);
                            Serial.println(": sensor stuck LOW, could not back out. Check wiring/polarity.");
                            state[i] = HS_ERROR;
                        }
                    }
                    break;

                case HS_SEARCH_FWD:
                    if (hall_triggered(i)) {
                        state[i] = HS_COUNT_WINDOW;
                        window[i] = 0;
                        progress[i] = 0;
                    } else {
                        hall_search_step(i, true);
                        if (++progress[i] > HALL_SEARCH_MAX_STEPS) {
                            Serial.print("ERROR leg "); Serial.print(i);
                            Serial.println(": magnet not found within search range. Check wiring/magnet/sensitivity.");
                            state[i] = HS_ERROR;
                        }
                    }
                    break;

                case HS_COUNT_WINDOW:
                    if (!hall_triggered(i)) {
                        center_left[i] = window[i] / 2;
                        state[i] = HS_CENTER_BACK;
                    } else {
                        hall_search_step(i, true);
                        window[i]++;
                        if (++progress[i] > HALL_SEARCH_MAX_STEPS) {
                            Serial.print("ERROR leg "); Serial.print(i);
                            Serial.println(": window never closed. Check wiring/polarity/magnet.");
                            state[i] = HS_ERROR;
                        }
                    }
                    break;

                case HS_CENTER_BACK:
                    if (center_left[i] <= 0) {
                        current_step_pos[i] = 0;
                        Serial.print("Leg "); Serial.print(i);
                        Serial.print(": magnet window = "); Serial.print(window[i]);
                        Serial.println(" steps wide, centered.");
                        state[i] = HS_DONE;
                    } else {
                        hall_search_step(i, false);
                        center_left[i]--;
                    }
                    break;

                case HS_DONE:
                case HS_ERROR:
                    break;
            }
            if (state[i] != HS_DONE && state[i] != HS_ERROR) any_active = true;
        }
        if (any_active) delayMicroseconds(HALL_SEARCH_STEP_INTERVAL_US);
    }

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (state[i] == HS_ERROR) {
            Serial.println("Auto-homing ABORTED. Fix the flagged leg(s) and try again.");
            return false;
        }
    }

    Serial.println("All six legs centered on their magnets. Driving to level home...");
    drive_to_level_home_from_vertical();
    return true;
}

// Raw, error-checked accelerometer read (see ../octo/src/main.cpp for why
// this bypasses Adafruit_MMA8451::read()/getEvent() rather than using them -
// getEvent() can't distinguish a real reading from a failed I2C transaction).
bool read_accel_checked(float &xg, float &yg, float &zg) {
    Wire.beginTransmission(MMA8451_DEFAULT_ADDRESS);
    Wire.write(MMA8451_REG_OUT_X_MSB);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)MMA8451_DEFAULT_ADDRESS, (uint8_t)6) != 6) {
        return false;
    }
    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();

    int16_t x = buf[0]; x <<= 8; x |= buf[1]; x >>= 2;
    int16_t y = buf[2]; y <<= 8; y |= buf[3]; y >>= 2;
    int16_t z = buf[4]; z <<= 8; z |= buf[5]; z >>= 2;

    constexpr float DIVIDER = 4096.0f; // matches MMA8451_RANGE_2_G set in setup()
    xg = x / DIVIDER;
    yg = y / DIVIDER;
    zg = z / DIVIDER;
    return true;
}

// Averages N good accelerometer samples, spaced out a bit so they're not
// just re-reading the same I2C-buffered instant. Returns the count of
// samples that actually succeeded (0 if every single one failed).
constexpr uint8_t CAL_SAMPLES_PER_POSE = 12;
constexpr uint32_t CAL_SAMPLE_SPACING_MS = 15;

uint8_t sample_accel_avg(float &xg, float &yg, float &zg) {
    float sx = 0, sy = 0, sz = 0;
    uint8_t good = 0;
    for (uint8_t i = 0; i < CAL_SAMPLES_PER_POSE; i++) {
        float x, y, z;
        if (read_accel_checked(x, y, z)) {
            sx += x; sy += y; sz += z;
            good++;
        }
        delay(CAL_SAMPLE_SPACING_MS);
    }
    if (good == 0) return 0;
    xg = sx / good;
    yg = sy / good;
    zg = sz / good;
    return good;
}

// Extra dwell after the steppers report "settled" (target reached), before
// sampling - lets any mechanical ringing from the move damp out. Steppers
// have no springiness of their own, but the platform/rods/rig frame do.
constexpr uint32_t CAL_SETTLE_DWELL_MS = 350;

// Radial sweep of test roll/pitch poses. Yaw is intentionally not swept -
// gravity's direction doesn't change with yaw, so the accelerometer has
// nothing to say about it.
//
// The reachable envelope at NEUTRAL_Z is NOT a circle or a square - it's a
// lobed, direction-dependent shape (offline probing of this exact geometry
// found single-axis limits from ~11.5deg to ~16.5deg depending on which way
// you tilt). A square roll/pitch grid sized to clear the worst-case
// direction wastes most of its poses on the corners in every other
// direction (this is why the old +-16deg, 2deg-step square grid skipped
// 154/289 poses - more than half). Instead, walk outward from level along
// CAL_N_DIRECTIONS spokes in CAL_RADIUS_STEP_DEG increments; the moment a
// spoke's pose comes back unreachable, stop that spoke and move to the next
// one, so every direction gets sampled out to its own real limit instead of
// either the smallest shared limit or a fixed guess. This assumes the
// envelope is star-shaped from level (no direction has an unreachable pose
// closer to level than a reachable one further out) - confirmed by offline
// probing of this geometry; if NEUTRAL_Z or the physical geometry changes
// enough to break that assumption, a spoke would just stop short of its true
// limit, not produce wrong data.
constexpr uint8_t CAL_N_DIRECTIONS = 24;      // 15deg apart
constexpr float CAL_RADIUS_STEP_DEG = 0.5f;
constexpr float CAL_MAX_RADIUS_DEG = 20.0f;   // hard cap in case a spoke never reports unreachable

// Drives to (roll, pitch), settles, and samples/logs one pose. Returns false
// if the pose itself was unreachable (caller should stop walking this
// spoke); returns true otherwise, including when the pose was reachable but
// every accel sample at it failed (CAL_ERR) - that's a sampling problem, not
// a reachability one, so the spoke keeps going.
bool sample_cal_pose(float roll, float pitch) {
    if (!set_target_pose(0, 0, 0, roll, pitch, 0)) {
        Serial.print("CAL_SKIP ");
        Serial.print(roll, 1); Serial.print(' '); Serial.println(pitch, 1);
        return false;
    }

    drive_until_settled();
    delay(CAL_SETTLE_DWELL_MS);

    float ax, ay, az;
    uint8_t n_good = sample_accel_avg(ax, ay, az);
    if (n_good == 0) {
        Serial.print("CAL_ERR ");
        Serial.print(roll, 1); Serial.print(' '); Serial.println(pitch, 1);
        return true;
    }

    Serial.print("CAL ");
    Serial.print(roll, 1); Serial.print(' ');
    Serial.print(pitch, 1); Serial.print(' ');
    Serial.print(0.0f, 1); Serial.print(' '); // yaw, always 0 in this sweep
    Serial.print(ax, 4); Serial.print(' ');
    Serial.print(ay, 4); Serial.print(' ');
    Serial.print(az, 4); Serial.print(' ');
    Serial.println(n_good);
    return true;
}

// Returns to level and re-runs the hall auto-home search. The sweep is
// open-loop stepping with no per-leg position feedback between homes - each
// spoke walks out to its limit and back dozens of times, and a single
// missed step anywhere silently offsets every pose after it for the rest of
// the sweep with no way to tell from the logged data alone. Re-anchoring to
// the physical hall sensors after every spoke bounds how much drift can
// accumulate to "at most one spoke's worth" instead of "the whole sweep".
bool rehome_mid_sweep() {
    set_target_pose(0, 0, 0, 0, 0, 0);
    drive_until_settled();
    return do_hall_auto_homing();
}

void run_calibration_sweep() {
    if (!homed) {
        Serial.println("Not homed yet - send Z or H first.");
        return;
    }
    if (!mma_ok) {
        Serial.println("Accelerometer not initialized - can't calibrate. Check I2C wiring and reset.");
        return;
    }

    Serial.println("CAL_START");

    sample_cal_pose(0, 0); // level center, shared starting point for every spoke

    for (uint8_t di = 0; di < CAL_N_DIRECTIONS; di++) {
        float theta = radians(di * (360.0f / CAL_N_DIRECTIONS));
        for (float r = CAL_RADIUS_STEP_DEG; r <= CAL_MAX_RADIUS_DEG; r += CAL_RADIUS_STEP_DEG) {
            float roll = r * cosf(theta);
            float pitch = r * sinf(theta);
            if (!sample_cal_pose(roll, pitch)) break; // out of reach - next spoke
        }

        if (!rehome_mid_sweep()) {
            Serial.println("CAL_ABORT rehoming failed mid-sweep - fix the flagged leg and rerun.");
            homed = false; // hall search left position tracking in an unknown state
            return;
        }
    }

    Serial.println("CAL_DONE");
}

char cmd_buf[64];
uint8_t cmd_len = 0;

void process_command(char* line) {
    if (line[0] == 'Z' || line[0] == 'z') {
        do_homing();
        return;
    }
    if (line[0] == 'H' || line[0] == 'h') {
        do_hall_auto_homing();
        return;
    }
    if (line[0] == 'P' || line[0] == 'p') {
        float x, y, z, roll, pitch, yaw;
        int n = sscanf(line + 1, "%f %f %f %f %f %f", &x, &y, &z, &roll, &pitch, &yaw);
        if (n != 6) {
            Serial.println("Parse error. Usage: P x y z roll pitch yaw");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        if (set_target_pose(x, y, z, roll, pitch, yaw)) {
            Serial.println("Target pose accepted.");
        }
        return;
    }
    if (line[0] == 'S' || line[0] == 's') {
        if (homed) {
            set_target_pose(0, 0, 0, 0, 0, 0);
            Serial.println("Stopped. Returning to neutral pose.");
        } else {
            Serial.println("Stopped, holding position.");
        }
        return;
    }
    if (line[0] == 'A' || line[0] == 'a') {
        float xg, yg, zg;
        if (!mma_ok || !read_accel_checked(xg, yg, zg)) {
            Serial.println("ACCEL ERR");
            return;
        }
        Serial.print("ACCEL ");
        Serial.print(xg, 3);
        Serial.print(' ');
        Serial.print(yg, 3);
        Serial.print(' ');
        Serial.println(zg, 3);
        return;
    }
    if (line[0] == 'B' || line[0] == 'b') {
        // Reboot into the ROM DFU bootloader so the PC can reflash without
        // the BOOT0 jumper - see firmware_common/dfu_reboot.h. Used by
        // flash_octo.py; the USB serial port disappears and a 0483:df11
        // DFU device shows up in its place.
        Serial.println("Rebooting to DFU bootloader...");
        Serial.flush();
        dfu_reboot::request_dfu_reboot();
        return;
    }
    if (line[0] == 'Q' || line[0] == 'q') {
        Serial.println(legs_settled() ? "SETTLED" : "MOVING");
        return;
    }
    if (line[0] == 'C' || line[0] == 'c') {
        run_calibration_sweep();
        return;
    }
    Serial.println("Unknown command. Use Z (home from hand-set vertical), H (auto-home via hall sensors), P x y z roll pitch yaw (move), S (stop), Q (status query), A (read accelerometer), C (run calibration sweep), or B (reboot to DFU bootloader for flashing).");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    init_crank_bases();

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(STEP_PINS[i], OUTPUT);
        pinMode(DIR_PINS[i], OUTPUT);
        pinMode(ENABLE_PINS[i], OUTPUT);
        pinMode(HALL_PINS[i], INPUT);
        digitalWrite(ENABLE_PINS[i], LOW);
        digitalWrite(DIR_PINS[i], LOW);
        digitalWrite(STEP_PINS[i], LOW);

        uarts[i]->begin(115200);
        drivers[i]->begin();
        drivers[i]->toff(4);
        drivers[i]->rms_current(TEST_CURRENT_MA);
        drivers[i]->microsteps(MICROSTEPS);
        drivers[i]->pwm_autoscale(true);

        uint8_t result = drivers[i]->test_connection();
        Serial.print("Motor "); Serial.print(i);
        Serial.print(" UART test_connection(): "); Serial.println(result);
    }

    Wire.setSCL(MMA_SCL_PIN);
    Wire.setSDA(MMA_SDA_PIN);
    Wire.begin();
    mma_ok = mma.begin();
    if (mma_ok) {
        mma.setRange(MMA8451_RANGE_2_G);
        Serial.println("MMA8451 accelerometer found.");
    } else {
        Serial.println("MMA8451 accelerometer NOT found - check I2C wiring. 'A'/'C' will report errors.");
    }

    Serial.println("Ready. Send Z or H to home, then C to run the calibration sweep.");
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                process_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }

    step_legs_once();
}
