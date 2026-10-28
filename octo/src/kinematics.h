// kinematics.h - Rotary Stewart Platform inverse kinematics
//
// Converts a target platform pose (position + orientation) into six crank
// angles, one per leg, given the fixed geometry of this specific machine.
//
// All the constants below were derived, corner by corner, from the actual
// base_plate.scad / clamping_motor_horn.scad / top-plate geometry and the
// physically-confirmed motor-to-platform-hole pairing (see chat history for
// the full derivation - this was not guessed, every number traces back to
// either a CAD dimension or a directly-measured/confirmed physical fact):
//
//   - Base pivots: motor shaft position + the horn's real offset along the
//     shaft (20mm out of the 24mm shaft, accounting for the horn hub sitting
//     flush with the shaft tip and the M5 hole being centered 4mm in from
//     that face - unaffected by the horn tip taper going from 4mm to 5mm,
//     since that's a tip-strength change, not a hub/throw change).
//   - Shaft directions: horizontal, ~21 degrees off true radial, due to the
//     two motors per corner being offset +-31mm sideways from the corner's
//     centerline.
//   - Platform-local points: latest latch design, ball joints as actual
//     sphere centers (~99.5mm radius, ~12mm below the plate). BOTH the motor
//     numbering AND the rod pairing were re-verified from scratch against
//     the physical build using lettered-position diagrams, after we found
//     the earlier revisions had motors cross-wired. Confirmed mapping:
//     motor 0->S, 1->P, 2->Q, 3->T, 4->U, 5->R (top-joint letters), with
//     motors 2/3 at the lower-left corner and 4/5 at the upper-left - the
//     swap from earlier assumptions. All six legs solve to an identical
//     171.94mm neutral height (zero spread), the check that it's correct.
//   - Rod length: 140mm. Crank throw: 20mm (from the horn file itself).
//
// Neutral pose (flat platform, no tilt, centered) sits at Z = 171.9403mm
// above the base frame's Z=0 reference, confirmed by all six legs agreeing
// exactly.

#pragma once
#include <Arduino.h>
#include <math.h>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
    }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return Vec3(x/l, y/l, z/l); }
};

// A 3x3 rotation matrix, row-major, built from roll/pitch/yaw (degrees),
// applied in the order Rz(yaw) * Ry(pitch) * Rx(roll) - i.e. roll about the
// platform's own X axis, then pitch about Y, then yaw about Z, matching the
// same intrinsic-rotation convention used throughout the geometry derivation.
struct Mat3 {
    float m[3][3];
    Vec3 apply(const Vec3& v) const {
        return Vec3(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        );
    }
};

inline Mat3 rotation_from_rpy_deg(float roll_deg, float pitch_deg, float yaw_deg) {
    float r = radians(roll_deg), p = radians(pitch_deg), y = radians(yaw_deg);
    float cr=cosf(r), sr=sinf(r), cp=cosf(p), sp=sinf(p), cy=cosf(y), sy=sinf(y);
    // Rz(y) * Ry(p) * Rx(r)
    Mat3 out;
    out.m[0][0] = cy*cp;
    out.m[0][1] = cy*sp*sr - sy*cr;
    out.m[0][2] = cy*sp*cr + sy*sr;
    out.m[1][0] = sy*cp;
    out.m[1][1] = sy*sp*sr + cy*cr;
    out.m[1][2] = sy*sp*cr - cy*sr;
    out.m[2][0] = -sp;
    out.m[2][1] = cp*sr;
    out.m[2][2] = cp*cr;
    return out;
}

constexpr uint8_t NUM_LEGS = 6;
constexpr float ROD_LENGTH = 140.0f;
constexpr float CRANK_THROW = 30.0f;  // 30mm horn (up from 20mm) for more tilt range

// ===========================================================================
// >>> TUNABLE: NEUTRAL_Z - the resting platform height. <<<
// This is the main knob for rod stress. RAISE it to relieve rod stress (lifts
// the horns further above horizontal at home); LOWER it if you want more
// upward tilt headroom. Change ONLY this number - the homing offsets are
// computed from it automatically, so the platform stays level at whatever you
// pick. Reasonable range ~170-184mm. Re-home (Z) after changing.
//   170.18 = horns at -11.9deg / +24.4deg (lowest, most rod stress)
//   178.0  = horns at ~+3deg / +37deg    (both above horizontal - default)
//   182.0  = horns at ~+11deg / +44deg   (highest, least stress, less headroom)
constexpr float NEUTRAL_Z = 180.0f;
// ===========================================================================

// HOMING MODEL (see main.cpp do_homing): you physically set every horn dead
// VERTICAL (tip straight up - the one angle you can eyeball accurately), then
// send Z. The firmware computes how far each leg must rotate FROM vertical to
// reach the level home at NEUTRAL_Z, and drives there slowly. Because the
// offsets are derived from NEUTRAL_Z at runtime, changing NEUTRAL_Z above is
// all you need - no other constant to keep in sync.

// Base pivot points, world/base frame (mm). Index = motor/leg number.
// RE-CONFIRMED motor numbering (verified against the physical build via
// lettered-position diagrams): motors 2/3 lower-left, 4/5 upper-left.
const Vec3 BASE_PIVOT[NUM_LEGS] = {
    {101.825f, 31.000f, 27.350f},   // motor 0
    {101.825f, -31.000f, 27.350f},  // motor 1
    {-24.066f, -103.683f, 27.350f}, // motor 2
    {-77.759f, -72.683f, 27.350f},  // motor 3
    {-77.759f, 72.683f, 27.350f},   // motor 4
    {-24.066f, 103.683f, 27.350f},  // motor 5
};

// Unit vector along each motor's shaft (the crank's axis of rotation).
const Vec3 SHAFT_DIR[NUM_LEGS] = {
    {1.000000f, 0.000000f, 0.000000f},   // motor 0
    {1.000000f, 0.000000f, 0.000000f},   // motor 1
    {-0.500000f, -0.866025f, 0.000000f}, // motor 2
    {-0.500000f, -0.866025f, 0.000000f}, // motor 3
    {-0.500000f, 0.866025f, 0.000000f},  // motor 4
    {-0.500000f, 0.866025f, 0.000000f},  // motor 5
};

// Platform ball-joint attachment points, in the platform's own local frame.
// Latest top plate: ball joints at ~104.2mm radius, top plate ROTATED 60deg
// vs the base (reintroduces a tangential rod lean for yaw stiffness - keeps
// the platform from spinning freely, at the cost of some yaw RANGE). 12.5mm
// below the plate. Motor->ball pairing verified via the layout diagram +
// "motor 0's rod goes +Y" confirmation. All six legs solve to an identical
// 170.18mm neutral height. Gives +-17.5deg roll/pitch, +-30deg yaw.
const Vec3 PLATFORM_LOCAL[NUM_LEGS] = {
    {73.681f, 73.681f, -12.500f},    // leg 0
    {73.681f, -73.681f, -12.500f},   // leg 1
    {26.969f, -100.649f, -12.500f},  // leg 2
    {-100.649f, -26.969f, -12.500f}, // leg 3
    {-100.649f, 26.969f, -12.500f},  // leg 4
    {26.969f, 100.649f, -12.500f},   // leg 5
};

// Global sign flip for the homing rotation direction. If horns rotate the
// wrong way from vertical on the first Z, change 1.0 to -1.0. (The per-leg
// offset itself is computed at runtime from NEUTRAL_Z in do_homing, so there's
// no offset table to maintain here.)
constexpr float HOMING_DIR_SIGN = 1.0f;

// Per-leg in-plane basis vectors (u,v), spanning the plane perpendicular to
// the shaft axis. v is always world-up, since every shaft direction here is
// horizontal (z=0) - safe, no singularity. u completes a consistent
// right-handed (u, v, shaft_dir) frame. Computed once at startup.
Vec3 CRANK_U[NUM_LEGS];
Vec3 CRANK_V[NUM_LEGS];

inline void init_crank_bases() {
    Vec3 world_up(0, 0, 1);
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        CRANK_V[i] = world_up;
        CRANK_U[i] = world_up.cross(SHAFT_DIR[i]).normalized();
    }
}

// Solves for the crank angle (radians) that puts leg i's horn tip exactly
// ROD_LENGTH away from world_target. Two solutions generally exist (like an
// elbow-up/elbow-down pair); prev_angle picks whichever is closer, for
// continuity as the platform moves. Returns false if the target is out of
// reach at the current rod length (writes NAN to out_angle in that case).
inline bool solve_leg_angle(uint8_t leg, const Vec3& world_target, float prev_angle, float& out_angle) {
    Vec3 d = world_target - BASE_PIVOT[leg];
    float d_u = d.dot(CRANK_U[leg]);
    float d_v = d.dot(CRANK_V[leg]);
    float d_len_sq = d.dot(d);

    float C = (CRANK_THROW*CRANK_THROW + d_len_sq - ROD_LENGTH*ROD_LENGTH) / (2.0f*CRANK_THROW);
    float R = sqrtf(d_u*d_u + d_v*d_v);

    if (R < 1e-6f || fabsf(C/R) > 1.0f) {
        out_angle = NAN;
        return false; // unreachable: target too far or too close given rod length
    }

    float phi = atan2f(d_v, d_u);
    float delta = acosf(C / R);
    float sol1 = phi + delta;
    float sol2 = phi - delta;

    // pick whichever solution is angularly closer to prev_angle
    auto wrap_diff = [](float a, float b) {
        float d = fmodf(a - b + PI, 2.0f*PI);
        if (d < 0) d += 2.0f*PI;
        return d - PI;
    };
    float diff1 = fabsf(wrap_diff(sol1, prev_angle));
    float diff2 = fabsf(wrap_diff(sol2, prev_angle));
    out_angle = (diff1 <= diff2) ? sol1 : sol2;
    return true;
}

// Computes all six leg angles for a given platform pose. prev_angles[] is
// both input (for continuity) and gets updated in place with the new
// solved angles. Returns a bitmask of which legs failed to solve (0 = all OK).
inline uint8_t solve_all_legs(const Vec3& platform_pos, const Mat3& platform_rot, float prev_angles[NUM_LEGS]) {
    uint8_t fail_mask = 0;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        Vec3 world_point = platform_pos + platform_rot.apply(PLATFORM_LOCAL[i]);
        float angle;
        bool ok = solve_leg_angle(i, world_point, prev_angles[i], angle);
        if (ok) {
            prev_angles[i] = angle;
        } else {
            fail_mask |= (1 << i);
        }
    }
    return fail_mask;
}
