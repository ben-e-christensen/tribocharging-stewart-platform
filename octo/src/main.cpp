// Stage 4: full inverse kinematics. Target platform pose -> six crank
// angles -> six step targets, with a simple serial command interface.
//
// Builds on the confirmed-working stage 3 firmware (UART bring-up, toff,
// current, all six motors spinning). This stage replaces the "just spin"
// test loop with real target-seeking motion driven by kinematics.h.
//
// Everything about the geometry (base pivots, shaft directions, platform
// points, the confirmed motor<->hole pairing, the 60-degree assembly twist)
// lives in kinematics.h - see that file's header comment for the full
// derivation trail.
//
// SERIAL COMMANDS (115200 baud):
//   Z                        - home/zero. Physically position the platform
//                              level, centered, at the neutral height
//                              (~171.74mm) BEFORE sending this. It tells the
//                              firmware "this is the neutral pose," and
//                              everything after is tracked relative to it.
//   P x y z roll pitch yaw   - command a new target pose. Position in mm
//                              (relative to neutral position, so P 0 0 0 0 0 0
//                              means "go back to neutral"), angles in degrees.
//   W tilt_deg [period_sec]  - orbital tilt: holds a constant tilt magnitude
//                              (tilt_deg) while continuously rotating the
//                              tilt DIRECTION around a full circle, like
//                              swirling a marble around the rim of a bowl.
//                              period_sec is how long one full revolution
//                              takes (default 4s if omitted). Workspace
//                              sweep confirms 16.5 deg is the largest radius
//                              reachable at every point around the circle -
//                              go higher and it'll stutter at some phases.
//   J amplitude_deg [period_sec] - joint-space traveling wave: drives raw
//                              crank angles directly (no IK at all). Each
//                              motor rises to amplitude_deg, HOLDS there
//                              for one full segment, falls, then rests -
//                              phase-shifted 1/6 of a cycle per motor index
//                              so consecutive pairs (0,1),(1,2),(2,3),
//                              (3,4),(4,5),(5,0) are each briefly BOTH held
//                              at full amplitude simultaneously as the
//                              active pair steps around the ring.
//   S                        - stop any active orbital tilt or joint wave
//                              and immediately return to the neutral pose.
//   A                        - read the MMA8451 accelerometer wired to the
//                              board's onboard I2C header (PB8 SCL / PB9
//                              SDA). Replies "ACCEL x y z" (g's, 3 decimal
//                              places) or "ACCEL ERR" if the sensor never
//                              initialized. On-demand only (no continuous
//                              streaming) so it can't collide with the
//                              per-leg step timing in the main loop.
//   B                        - reboot into the STM32 ROM DFU bootloader for
//                              reflashing (no BOOT0 jumper needed). Used by
//                              ../flash_octo.py - see firmware_common/dfu_reboot.h.
//
// MOTION NOTE: this stage moves each leg toward its target at a fixed max
// step rate - no acceleration profile yet. Fine for a slow-moving balancing
// platform; worth revisiting if you ever want fast, smooth pose changes.

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <TMCStepper.h>
#include "kinematics.h"
#include "dfu_reboot.h" // 'B' command: reboot into DFU for flashing, no BOOT0 jumper

constexpr uint8_t NUM_MOTORS = 6;

// Indexed 0-5, matching Driver0-Driver5 sockets.
constexpr uint8_t STEP_PINS[NUM_MOTORS]   = { PF13, PG0,  PF11, PG4, PF9,  PC13 };
constexpr uint8_t DIR_PINS[NUM_MOTORS]    = { PF12, PG1,  PG3,  PC1, PF10, PF0  };
constexpr uint8_t ENABLE_PINS[NUM_MOTORS] = { PF14, PF15, PG5,  PA0, PG2,  PF1  }; // all active-low

// Hall-effect homing sensors, one per leg, on the board's free DIAG/endstop
// headers. Default order below - VERIFY against how you actually wired
// them (swap entries if leg N's sensor is on a different header).
// Hall-effect homing sensors, one per leg. MOVED from the DIAG/endstop
// headers to the EXP1 header - the DIAG pins have hardware pull-up
// circuitry baked into the board itself, which fought the sensor's own
// pull-up and only sank the triggered voltage to ~1.6-1.8V (not a clean
// LOW). EXP1 is plain GPIO with no display attached in this build (EXP1/
// EXP2 are only used for an LCD panel, which we don't have), so it's free
// and doesn't have that extra pull-up in the way. Confirmed clean 0V here.
constexpr uint8_t HALL_PINS[NUM_LEGS] = { PE8, PE9, PE12, PE13, PE14, PE15 };
// >>> TUNABLE: sensor polarity. Most cheap hall breakout modules pull the
// output LOW when a magnet is present. If your bench test shows the
// opposite (reads HIGH near the magnet), flip this to false.
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

// >>> TUNABLE: slower speed for big jumps (S return-to-neutral, P moves, and
// the first snap when W/J starts), so those don't feel abrupt. Continuous
// orbit/wave TRACKING stays at the fast rate above - each ~50Hz update only
// moves the target a little, so it's automatically under SNAP_THRESHOLD_STEPS
// and never uses this slower rate. Raise SNAP_STEP_INTERVAL_US for gentler
// jumps; lower SNAP_THRESHOLD_STEPS if continuous moves start feeling slow.
constexpr uint32_t SNAP_STEP_INTERVAL_US = 6000;  // ~167 steps/sec for big jumps
constexpr int32_t SNAP_THRESHOLD_STEPS = 45;      // ~5 degrees of remaining travel

float neutral_angle[NUM_LEGS];      // crank angle at the homed neutral pose
float current_angle[NUM_LEGS];      // best-known current angle (from step count)
int32_t current_step_pos[NUM_LEGS] = {0};
int32_t target_step_pos[NUM_LEGS]  = {0};
uint32_t last_step_time[NUM_LEGS]  = {0};
bool homed = false;

// --- Orbital tilt ("W") state ---
bool orbit_active = false;
float orbit_tilt_deg = 0;
float orbit_period_ms = 4000;
uint32_t orbit_start_millis = 0;
uint32_t last_orbit_update = 0;
constexpr uint32_t ORBIT_UPDATE_INTERVAL_MS = 20; // ~50Hz pose recompute rate

// --- Joint-space traveling wave ("J") state ---
bool jointwave_active = false;
float jointwave_amplitude_deg = 0;
float jointwave_period_ms = 4000;
uint32_t jointwave_start_millis = 0;
uint32_t last_jointwave_update = 0;
constexpr uint32_t JOINTWAVE_UPDATE_INTERVAL_MS = 20; // ~50Hz

// Shared by set_target_pose() and update_orbit(): solves all six legs for
// a given position+rotation and, if reachable, converts to step targets.
// If unreachable, just leaves the existing target_step_pos[] alone (holds
// position) rather than erroring - callers decide whether to warn.
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

// Builds a rotation for tilting by angle_deg around a horizontal axis that
// itself points in the rotating compass direction phase_rad - i.e. a true
// single-axis "conical" tilt, not two composed Euler rotations. This is
// what update_orbit() needs: composing roll=A*cos(phase) and
// pitch=A*sin(phase) as sequential Euler angles doesn't commute, and the
// resulting tilt axis wobbles off a clean circle (visible as a
// back-and-forth rock) once the angle isn't tiny. Built via Rodrigues'
// rotation formula around axis (sin(phase), -cos(phase), 0).
Mat3 rotation_conical_tilt(float angle_deg, float phase_rad) {
    float theta = radians(angle_deg);
    float ax = sinf(phase_rad);
    float ay = -cosf(phase_rad);
    float s = sinf(theta), c = cosf(theta);
    Mat3 out;
    out.m[0][0] = c + ax*ax*(1-c);
    out.m[0][1] = ax*ay*(1-c);
    out.m[0][2] = ay*s;
    out.m[1][0] = ay*ax*(1-c);
    out.m[1][1] = c + ay*ay*(1-c);
    out.m[1][2] = -ax*s;
    out.m[2][0] = -ay*s;
    out.m[2][1] = ax*s;
    out.m[2][2] = c;
    return out;
}

// Called at ~50Hz while an orbital tilt is active. Advances the phase and
// recomputes the pose for a constant-magnitude, rotating-direction tilt -
// a genuine single-axis conical rotation now, not composed roll+pitch.
// If a particular phase happens to be unreachable (only possible above the
// ~12 deg full-circle-safe radius the sweep tool found), it just holds the
// previous target for that one tick - the sweep moves on a moment later.
void update_orbit() {
    float elapsed_ms = (float)(millis() - orbit_start_millis);
    float phase = fmodf(elapsed_ms / orbit_period_ms, 1.0f) * 2.0f * PI;
    Mat3 rot = rotation_conical_tilt(orbit_tilt_deg, phase);
    Vec3 pos(0, 0, NEUTRAL_Z);
    commit_pose(pos, rot); // silently keeps old target if this phase is unreachable
}

// Shape for one motor's traveling-wave cycle, as a function of that motor's
// own local phase (wrapped into [0,1)). Segments, in order:
//   [0,   1/6): falling from D to 0
//   [1/6, 4/6): holding at 0 (three segments - half the period)
//   [4/6, 5/6): rising from 0 to D
//   [5/6, 1  ): holding at D
// Each motor is phase-shifted by 1/6 of the cycle relative to the next, so
// consecutive motors (i, i+1) are BOTH in their hold-at-D segment at the
// same moment - i falling out of it just as i+1 settles into it - giving
// the six overlapping pairs (0,1),(1,2),(2,3),(3,4),(4,5),(5,0) in turn.
float trapezoid_wave(float local_phase) {
    if (local_phase < 1.0f/6.0f) {
        return 1.0f - local_phase * 6.0f; // falling
    } else if (local_phase < 4.0f/6.0f) {
        return 0.0f; // holding at 0
    } else if (local_phase < 5.0f/6.0f) {
        return (local_phase - 4.0f/6.0f) * 6.0f; // rising
    } else {
        return 1.0f; // holding at D
    }
}

// Called at ~50Hz while a joint-space wave is active. Directly sets each
// motor's target crank angle from the traveling trapezoidal wave - no IK,
// no platform pose involved at all, since this is defined purely in terms
// of each motor's own angle relative to its homed neutral.
void update_jointwave() {
    float elapsed_ms = (float)(millis() - jointwave_start_millis);
    float global_phase = fmodf(elapsed_ms / jointwave_period_ms, 1.0f);
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        float local_phase = global_phase - (float)i / 6.0f;
        local_phase -= floorf(local_phase); // wrap into [0,1)
        float value = trapezoid_wave(local_phase);
        float delta_deg = jointwave_amplitude_deg * value;
        target_step_pos[i] = (int32_t) roundf(radians(delta_deg) * STEPS_PER_RAD);
    }
}

// Homing, VERTICAL-REFERENCE model:
// You physically set every horn dead VERTICAL (tip straight up) by eye - the
// one horn angle that's easy to judge accurately. Then Z drives each leg from
// vertical DOWN to the level home. The offset is COMPUTED at runtime from the
// solved home angle (home_crank_angle - 90deg, since vertical = +90deg in our
// convention), so it always matches whatever NEUTRAL_Z is set to - change
// NEUTRAL_Z in kinematics.h and this stays correct automatically.
//
// The move is done SLOWLY and deliberately (HOMING_STEP_INTERVAL_US) so you
// can watch it and hit reset if a horn turns the wrong way. If a horn rotates
// the WRONG direction from vertical, flip HOMING_DIR_SIGN in kinematics.h.
constexpr uint32_t HOMING_STEP_INTERVAL_US = 6000; // slow: ~167 steps/sec
constexpr float HORN_VERTICAL_ANGLE_DEG = 90.0f;   // vertical horn = +90deg (unit-circle convention)

// Shared by BOTH homing paths: assumes current_step_pos[i]==0 already means
// "vertical" for every leg (either because you set it by hand, or because
// do_hall_auto_homing()'s search state machine just measured it), then
// drives slowly from there to the level home at NEUTRAL_Z. Offset is
// computed live so it always tracks whatever NEUTRAL_Z is currently set to.
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
        neutral_angle[i] = current_angle[i]; // reference for the pose solver
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
    Serial.println("Homed at level position. Horn angles now (deg from horizontal):");
    Serial.println("  legs 0/2/4 ~ -11.9, legs 1/3/5 ~ +24.4");
    Serial.println("Ready. S or 'P 0 0 0 0 0 0' returns here.");
}

void do_homing() {
    orbit_active = false;
    jointwave_active = false;
    Serial.println("Homing from VERTICAL (set by hand). Make sure all horns are straight up.");
    // Treat wherever the horns physically are RIGHT NOW as vertical (step 0).
    // (The hall-sensor path doesn't need this - do_hall_auto_homing()'s
    // search state machine already zeroes current_step_pos per leg once
    // it centers on that leg's magnet.)
    for (uint8_t i = 0; i < NUM_LEGS; i++) current_step_pos[i] = 0;
    drive_to_level_home_from_vertical();
}

// --- Hall-sensor auto-homing ---
// Finds the TRUE vertical position for each leg automatically, using the
// magnet-on-horn + hall-sensor-on-faceplate pair, instead of relying on you
// eyeballing vertical. Since the magnet (6mm) is wider than a single step,
// the sensor stays LOW across a RANGE of steps, not just one instant - so
// the search finds the whole window and centers on its midpoint:
//   1. If already inside the window at the start, back out of it first, so
//      the search always begins from outside (avoids capturing only half
//      the window).
//   2. Step forward until the sensor goes LOW (entering the window).
//   3. Keep stepping forward, counting, until it goes HIGH again (exiting).
//   4. Step backward by half that count, landing in the middle of the
//      window - the true magnet-center reference.
//   5. That position becomes step 0 (vertical) for that leg.
//
// ALL SIX LEGS RUN THIS TOGETHER, one step at a time, round-robin - NOT one
// leg fully finished before the next starts. Reason: this is a closed-loop
// parallel mechanism, so if the other five legs are already sitting near
// their correct home angle and stay frozen while just one leg gets dragged
// through a wide sweep, the platform can't actually accommodate that (the
// rods can't violate their fixed length) - the moving leg ends up fighting
// the whole rigid structure and stalls. Stepping all six together, a little
// at a time, keeps every leg's position close to the others throughout the
// search instead of letting one wander far while five sit locked.
constexpr uint32_t HALL_SEARCH_STEP_INTERVAL_US = 6000; // same slow pace as homing
constexpr int32_t HALL_SEARCH_MAX_STEPS = 1200; // ~135deg safety cap per phase -
                                                 // if exceeded, something's wrong
                                                 // (wiring, sensitivity, no magnet)

enum HallSearchState { HS_BACK_OUT, HS_SEARCH_FWD, HS_COUNT_WINDOW, HS_CENTER_BACK, HS_DONE, HS_ERROR };

bool hall_triggered(uint8_t leg) {
    bool raw = digitalRead(HALL_PINS[leg]);
    return HALL_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

// Steps one leg once, in the given direction, at the slow search pace.
void hall_search_step(uint8_t leg, bool forward) {
    digitalWrite(DIR_PINS[leg], forward ? HIGH : LOW);
    digitalWrite(STEP_PINS[leg], HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PINS[leg], LOW);
    current_step_pos[leg] += forward ? 1 : -1;
}

void do_hall_auto_homing() {
    orbit_active = false;
    jointwave_active = false;
    Serial.println("Auto-homing via hall sensors - finding true vertical, all legs together...");

    HallSearchState state[NUM_LEGS];
    int32_t progress[NUM_LEGS]  = {0}; // safety-cap counter, reset each phase
    int32_t window[NUM_LEGS]    = {0}; // steps counted across the window
    int32_t center_left[NUM_LEGS] = {0}; // steps still to back up while centering

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
                        current_step_pos[i] = 0; // vertical reference found
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
                    break; // nothing to do this round
            }
            if (state[i] != HS_DONE && state[i] != HS_ERROR) any_active = true;
        }
        if (any_active) delayMicroseconds(HALL_SEARCH_STEP_INTERVAL_US);
    }

    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (state[i] == HS_ERROR) {
            Serial.println("Auto-homing ABORTED. Fix the flagged leg(s) and try again.");
            return;
        }
    }

    Serial.println("All six legs centered on their magnets. Driving to level home...");
    drive_to_level_home_from_vertical();
}


// Raw, error-checked accelerometer read. Adafruit_MMA8451::read() (the
// stock library call behind getEvent()) never checks whether its I2C
// transaction actually succeeded - on failure it silently leaves its
// buffer at the initialized {register_addr, 0, 0, 0, 0, 0}, so a flaky
// bus (loose header pin, EMI from the TMC2209s chopping current right
// next to these wires, etc.) produces the exact same bogus "reading"
// forever, and getEvent() always returns true regardless. This bypasses
// that by doing the transaction directly and only reporting success when
// all 6 bytes genuinely came back.
bool read_accel_checked(float &xg, float &yg, float &zg) {
    Wire.beginTransmission(MMA8451_DEFAULT_ADDRESS);
    Wire.write(MMA8451_REG_OUT_X_MSB);
    if (Wire.endTransmission(false) != 0) return false; // repeated start, no stop

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
        orbit_active = false; // a direct pose command overrides any running orbit
        jointwave_active = false;
        if (set_target_pose(x, y, z, roll, pitch, yaw)) {
            Serial.println("Target pose accepted.");
        }
        return;
    }
    if (line[0] == 'W' || line[0] == 'w') {
        float tilt_deg, period_sec;
        int n = sscanf(line + 1, "%f %f", &tilt_deg, &period_sec);
        if (n < 1) {
            Serial.println("Parse error. Usage: W tilt_deg [period_sec]");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        jointwave_active = false;
        orbit_tilt_deg = tilt_deg;
        orbit_period_ms = (n == 2) ? (period_sec * 1000.0f) : 4000.0f;
        orbit_start_millis = millis();
        orbit_active = true;
        Serial.print("Orbit started: tilt=");
        Serial.print(orbit_tilt_deg);
        Serial.print(" deg, period=");
        Serial.print(orbit_period_ms / 1000.0f);
        Serial.println(" sec. Send S to stop.");
        return;
    }
    if (line[0] == 'J' || line[0] == 'j') {
        float amp_deg, period_sec;
        int n = sscanf(line + 1, "%f %f", &amp_deg, &period_sec);
        if (n < 1) {
            Serial.println("Parse error. Usage: J amplitude_deg [period_sec]");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        orbit_active = false;
        jointwave_amplitude_deg = amp_deg;
        jointwave_period_ms = (n == 2) ? (period_sec * 1000.0f) : 4000.0f;
        jointwave_start_millis = millis();
        jointwave_active = true;
        Serial.print("Joint wave started: amplitude=");
        Serial.print(jointwave_amplitude_deg);
        Serial.print(" deg, period=");
        Serial.print(jointwave_period_ms / 1000.0f);
        Serial.println(" sec. Send S to stop.");
        return;
    }
    if (line[0] == 'S' || line[0] == 's') {
        orbit_active = false;
        jointwave_active = false;
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
        // Status query for external scripts (e.g. a PC-side orchestration
        // GUI) to poll instead of guessing a sleep duration - mirrors how
        // the CNC side already polls Klipper for position/homed state.
        bool settled = true;
        for (uint8_t i = 0; i < NUM_LEGS; i++) {
            if (current_step_pos[i] != target_step_pos[i]) { settled = false; break; }
        }
        Serial.println(settled ? "SETTLED" : "MOVING");
        return;
    }
    Serial.println("Unknown command. Use Z (home from hand-set vertical), H (auto-home via hall sensors), P x y z roll pitch yaw (move), W tilt_deg [period_sec] (orbit), J amplitude_deg [period_sec] (joint wave), S (stop), Q (status query), A (read accelerometer), or B (reboot to DFU bootloader for flashing).");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    init_crank_bases();

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(STEP_PINS[i], OUTPUT);
        pinMode(DIR_PINS[i], OUTPUT);
        pinMode(ENABLE_PINS[i], OUTPUT);
        pinMode(HALL_PINS[i], INPUT); // plain INPUT, NOT INPUT_PULLUP - the
                                      // sensor board actively drives both
                                      // states itself (~3.3V idle / 0V
                                      // triggered), so there's no floating
                                      // condition to protect against, and an
                                      // internal pull-up would only add a
                                      // second (unwanted) pull-up path
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
        Serial.println("MMA8451 accelerometer NOT found - check I2C wiring. 'A' will report ACCEL ERR.");
    }

    Serial.println("Ready. Send Z to home, then P x y z roll pitch yaw to move.");
}

void loop() {
    // --- serial command intake ---
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

    // --- orbital tilt: recompute target pose at ~50Hz while active ---
    if (orbit_active && (millis() - last_orbit_update >= ORBIT_UPDATE_INTERVAL_MS)) {
        last_orbit_update = millis();
        update_orbit();
    }

    // --- joint-space wave: recompute crank angles directly at ~50Hz while active ---
    if (jointwave_active && (millis() - last_jointwave_update >= JOINTWAVE_UPDATE_INTERVAL_MS)) {
        last_jointwave_update = millis();
        update_jointwave();
    }

    // --- per-leg target-seeking motion ---
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