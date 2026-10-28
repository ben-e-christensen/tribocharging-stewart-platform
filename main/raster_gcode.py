"""
build_raster_gcode - pure function, no side effects. Produces the exact
G-code for a raster scan over a circular area. Shared by the manual "Raster
Area" button and the automated cycle, so there's exactly one place this
math lives.
"""

import math


def build_raster_gcode(start_x, start_y, circle_radius, raster_stepover, fast_speed):
    cx = start_x
    cy = start_y + circle_radius

    setup_gcode = f"G90\nG1 X{start_x:.3f} Y{start_y:.3f} F{fast_speed}\n"

    raster_gcode = ""
    y_offsets = []
    current_y = -circle_radius
    while current_y <= circle_radius + 0.1:
        y_offsets.append(current_y)
        current_y += raster_stepover

    direction = 1
    last_x, last_y = start_x, start_y

    for y_off in y_offsets:
        val_under_root = circle_radius**2 - y_off**2
        if val_under_root < 0:
            val_under_root = 0
        x_val = math.sqrt(val_under_root)
        x_start = cx - (x_val * direction)
        x_end = cx + (x_val * direction)
        y_actual = cy + y_off

        raster_gcode += f"G1 X{x_start:.3f} Y{y_actual:.3f} F{fast_speed}\n"
        raster_gcode += f"G1 X{x_end:.3f} Y{y_actual:.3f} F{fast_speed}\n"

        last_x = x_end
        last_y = y_actual
        direction *= -1

    end_gcode = f"G1 X0 Y0 F{fast_speed}\n"
    return setup_gcode, raster_gcode, end_gcode, last_x, last_y
