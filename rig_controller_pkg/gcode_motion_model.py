"""
GCodeMotionModel - computes WHERE the CNC head should be at any elapsed
time, purely from the G-code we sent and known kinematics - no network
polling required. This is what lets sample tagging run at the DAQ's full
rate instead of being capped by how often we can afford to ask Moonraker
for position.

MODEL / APPROXIMATION, stated plainly:
Each G1 segment is modeled as an independent trapezoidal (or triangular, if
too short to reach cruise speed) accel/cruise/decel move: start and end at
v=0, accelerate at max_accel up to the segment's commanded feed rate (or as
close to it as the segment length allows), then decelerate back to 0.

Real Klipper motion is a bit smoother than this at corners - its junction
deviation setting lets some velocity carry through a direction change
rather than braking fully to zero every time. Treating every segment as an
independent stop-start move is a small, LOCALIZED approximation right at
each corner; through the bulk of a long straight segment (where the head
has time to reach full cruise speed anyway) this model matches reality
closely. Given the alternative (replicating Klipper's actual corner-
blending planner) is a much bigger undertaking for a small accuracy gain,
this is the practical trade-off.
"""

import re


class GCodeMotionModel:
    def __init__(self, max_accel=3000.0):
        """max_accel: mm/s^2 - MUST match your printer.cfg's max_accel."""
        self.max_accel = max_accel
        self.segments = []  # each: dict with all the per-segment timing/geometry below
        self.total_duration = 0.0

    def load_gcode(self, gcode_text, start_x, start_y):
        """Parses a block of G1 X.. Y.. F.. lines (as produced by
        build_raster_gcode) into a timed sequence of segments, starting
        from (start_x, start_y)."""
        self.segments = []
        cx, cy = start_x, start_y
        cumulative_t = 0.0

        for line in gcode_text.strip().splitlines():
            line = line.strip()
            if not line.startswith("G1"):
                continue
            mx = re.search(r'X([-\d.]+)', line)
            my = re.search(r'Y([-\d.]+)', line)
            mf = re.search(r'F([-\d.]+)', line)
            if mx is None or my is None:
                continue
            x1, y1 = float(mx.group(1)), float(my.group(1))
            feed_mm_s = float(mf.group(1)) / 60.0 if mf else 0.0

            dx, dy = x1 - cx, y1 - cy
            distance = (dx**2 + dy**2) ** 0.5

            if distance < 1e-9 or feed_mm_s <= 0:
                # zero-length or feed-less move - nothing to time, just
                # update position and move on
                cx, cy = x1, y1
                continue

            dir_x, dir_y = dx / distance, dy / distance
            seg = self._build_segment(cx, cy, dir_x, dir_y, distance, feed_mm_s)
            seg["start_offset"] = cumulative_t
            seg["end_offset"] = cumulative_t + seg["t_total"]
            cumulative_t = seg["end_offset"]
            self.segments.append(seg)

            cx, cy = x1, y1

        self.total_duration = cumulative_t
        return self.total_duration

    def _build_segment(self, x0, y0, dir_x, dir_y, distance, feed_mm_s):
        a = self.max_accel
        v = feed_mm_s

        t_accel = v / a
        d_accel = 0.5 * a * t_accel**2

        if 2 * d_accel <= distance:
            # trapezoidal: reaches full cruise speed
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / v
            t_total = 2 * t_accel + t_cruise
            profile = "trapezoid"
            v_peak = v
        else:
            # triangular: too short to reach the commanded feed rate
            v_peak = (a * distance) ** 0.5
            t_accel = v_peak / a
            t_cruise = 0.0
            d_accel = distance / 2.0
            t_total = 2 * t_accel
            profile = "triangle"

        return {
            "x0": x0, "y0": y0, "dir_x": dir_x, "dir_y": dir_y,
            "distance": distance, "feed_mm_s": feed_mm_s,
            "profile": profile, "v_peak": v_peak,
            "t_accel": t_accel, "t_cruise": t_cruise,
            "d_accel": d_accel, "t_total": t_total,
        }

    def position_at(self, t):
        """Returns (x, y) at elapsed time t since the gcode block was
        dispatched. Clamps to the first/last point if t is outside the
        modeled range (before dispatch, or after the whole block finishes)."""
        if not self.segments:
            return None, None
        if t <= 0:
            s = self.segments[0]
            return s["x0"], s["y0"]
        if t >= self.total_duration:
            last = self.segments[-1]
            return (last["x0"] + last["dir_x"] * last["distance"],
                    last["y0"] + last["dir_y"] * last["distance"])

        # find the segment containing t (linear scan - segment counts here
        # are small, tens to low hundreds, so this is cheap; switch to
        # bisect on start_offset if you ever raster at much higher resolution)
        for seg in self.segments:
            if seg["start_offset"] <= t <= seg["end_offset"]:
                t_local = t - seg["start_offset"]
                dist_covered = self._distance_at_local_t(seg, t_local)
                x = seg["x0"] + seg["dir_x"] * dist_covered
                y = seg["y0"] + seg["dir_y"] * dist_covered
                return x, y

        # shouldn't get here, but fail safe to the last known point
        last = self.segments[-1]
        return (last["x0"] + last["dir_x"] * last["distance"],
                last["y0"] + last["dir_y"] * last["distance"])

    @staticmethod
    def _distance_at_local_t(seg, t_local):
        a_phase_end = seg["t_accel"]
        c_phase_end = seg["t_accel"] + seg["t_cruise"]
        accel = seg["v_peak"] / seg["t_accel"] if seg["t_accel"] > 0 else 0.0

        if t_local <= a_phase_end:
            return 0.5 * accel * t_local**2
        elif t_local <= c_phase_end:
            return seg["d_accel"] + seg["v_peak"] * (t_local - a_phase_end)
        else:
            t_dec = t_local - c_phase_end
            return (seg["d_accel"] + seg["v_peak"] * seg["t_cruise"]
                    + seg["v_peak"] * t_dec - 0.5 * accel * t_dec**2)
