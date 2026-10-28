"""
workspace_sweep.py - Map the reachable roll/pitch/yaw envelope offline.

Replicates the exact same math as kinematics.h / solve_leg_angle(), so results
here should match what the firmware would report - without needing to
actually move the platform through hundreds of test poses.

Usage: just run it. Edit ROLL_RANGE / PITCH_RANGE / YAW_SLICES below to
explore a different region, or import is_pose_reachable() to check single
poses from your own scripts.
"""

import numpy as np
import matplotlib.pyplot as plt

ROD_LENGTH = 140.0
CRANK_THROW = 30.0
NEUTRAL_Z = 178.0

BASE_PIVOT = np.array([
    [101.825, 31.000, 27.350],
    [101.825, -31.000, 27.350],
    [-24.066, -103.683, 27.350],
    [-77.759, -72.683, 27.350],
    [-77.759, 72.683, 27.350],
    [-24.066, 103.683, 27.350],
])

SHAFT_DIR = np.array([
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [-0.5, -0.866025, 0.0],
    [-0.5, -0.866025, 0.0],
    [-0.5, 0.866025, 0.0],
    [-0.5, 0.866025, 0.0],
])

# Outer latches, measured 116mm radius, confirmed pairing 0->S,1->P,2->Q,3->T,4->U,5->R
PLATFORM_LOCAL = np.array([
    [73.681, 73.681, -12.5],
    [73.681, -73.681, -12.5],
    [26.969, -100.649, -12.5],
    [-100.649, -26.969, -12.5],
    [-100.649, 26.969, -12.5],
    [26.969, 100.649, -12.5],
])

WORLD_UP = np.array([0, 0, 1])
CRANK_V = np.tile(WORLD_UP, (6, 1))
CRANK_U = np.cross(WORLD_UP, SHAFT_DIR)
CRANK_U = CRANK_U / np.linalg.norm(CRANK_U, axis=1, keepdims=True)


def rotation_from_rpy_deg(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])


def leg_reachable(leg, world_target):
    d = world_target - BASE_PIVOT[leg]
    d_u = np.dot(d, CRANK_U[leg])
    d_v = np.dot(d, CRANK_V[leg])
    d_len_sq = np.dot(d, d)
    C = (CRANK_THROW**2 + d_len_sq - ROD_LENGTH**2) / (2 * CRANK_THROW)
    R = np.sqrt(d_u**2 + d_v**2)
    if R < 1e-6:
        return False
    return abs(C / R) <= 1.0


def is_pose_reachable(x, y, z, roll, pitch, yaw):
    pos = np.array([x, y, z + NEUTRAL_Z])
    rot = rotation_from_rpy_deg(roll, pitch, yaw)
    for leg in range(6):
        world_point = pos + rot @ PLATFORM_LOCAL[leg]
        if not leg_reachable(leg, world_point):
            return False
    return True


if __name__ == "__main__":
    ROLL_RANGE = np.arange(-20, 20.5, 1.0)
    PITCH_RANGE = np.arange(-20, 20.5, 1.0)
    YAW_SLICES = [-15, -10, -5, 0, 5, 10, 15]

    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()

    for idx, yaw in enumerate(YAW_SLICES):
        grid = np.zeros((len(PITCH_RANGE), len(ROLL_RANGE)))
        for i, pitch in enumerate(PITCH_RANGE):
            for j, roll in enumerate(ROLL_RANGE):
                grid[i, j] = 1.0 if is_pose_reachable(0, 0, 0, roll, pitch, yaw) else 0.0

        ax = axes[idx]
        ax.imshow(grid, origin='lower', extent=[ROLL_RANGE[0], ROLL_RANGE[-1], PITCH_RANGE[0], PITCH_RANGE[-1]],
                  cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
        ax.set_title(f"yaw = {yaw} deg")
        ax.set_xlabel("roll (deg)")
        ax.set_ylabel("pitch (deg)")
        ax.axhline(0, color='k', linewidth=0.3)
        ax.axvline(0, color='k', linewidth=0.3)

    axes[-1].axis('off')
    fig.suptitle("Reachable envelope (green = OK, red = unreachable) at x=y=z=0", fontsize=14)
    fig.tight_layout()
    fig.savefig("/home/claude/workspace_envelope.png", dpi=120)
    print("Saved plot to /home/claude/workspace_envelope.png")

    # Quick numeric summary: largest pure single-axis range at yaw=0
    print("\nPure single-axis limits at yaw=0, pitch=0 / roll=0:")
    max_roll = max(r for r in np.arange(0, 30, 0.5) if is_pose_reachable(0,0,0,r,0,0))
    max_pitch = max(p for p in np.arange(0, 30, 0.5) if is_pose_reachable(0,0,0,0,p,0))
    max_yaw = max(yv for yv in np.arange(0, 30, 0.5) if is_pose_reachable(0,0,0,0,0,yv))
    print(f"  max roll alone:  +/- {max_roll} deg")
    print(f"  max pitch alone: +/- {max_pitch} deg")
    print(f"  max yaw alone:   +/- {max_yaw} deg")

    print("\nExample combined poses:")
    for pose in [(10,10,10), (5,5,5), (8,8,8), (7,7,0), (0,7,7)]:
        ok = is_pose_reachable(0,0,0,*pose)
        print(f"  roll={pose[0]:>3} pitch={pose[1]:>3} yaw={pose[2]:>3} -> {'OK' if ok else 'UNREACHABLE'}")

def max_safe_orbit_radius(step=0.5, phase_steps=72):
    """Largest tilt magnitude that stays reachable at every phase angle
    around a full 360-degree sweep (roll=N*cos(phi), pitch=N*sin(phi))."""
    radius = 0.0
    while True:
        next_radius = radius + step
        ok = True
        for k in range(phase_steps):
            phi = 2 * np.pi * k / phase_steps
            roll = next_radius * np.cos(phi)
            pitch = next_radius * np.sin(phi)
            if not is_pose_reachable(0, 0, 0, roll, pitch, 0):
                ok = False
                break
        if not ok:
            return radius
        radius = next_radius

print("\n--- max safe orbit radius (full 360 deg sweep, yaw=0) ---")
r = max_safe_orbit_radius()
print(f"  Largest tilt magnitude safe at EVERY phase angle: {r} deg")
