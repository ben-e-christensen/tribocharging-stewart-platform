"""
data_export - saving raw raster samples (and now, captured images) to disk.
Kept separate from RasterRecorder on purpose: recording/sampling and
exporting are different concerns, and you may want to change how/where
data gets saved (a different format, a database, etc.) without touching
the sampling logic.
"""

import csv
import os
import time


def save_samples_csv(samples, out_dir="raster_data", filename=None):
    """One-off saver: samples -> a single new CSV file. Kept for standalone/
    manual use; the GUI's automatic per-raster saving uses SessionCSVLogger
    below instead, which appends to one file across a whole session."""
    os.makedirs(out_dir, exist_ok=True)
    if filename is None:
        filename = f"raster_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    path = os.path.join(out_dir, filename)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x_mm", "y_mm", "charge_v", "timestamp"])
        writer.writerows(samples)
    return path


class SessionCSVLogger:
    """Creates ONE CSV file when constructed (i.e. once per app launch), and
    every subsequent raster's samples get APPENDED to that same file, each
    row tagged with an incrementing raster_number column - so a whole
    session's worth of rasters lives in one file, and you can tell them
    apart / group by raster_number afterward, rather than getting a new
    file every single raster.

    Also owns saving the "before this raster" photo, into a per-session
    images subfolder, named by raster number so it's easy to trace a CSV
    row back to exactly the image taken right before that raster ran."""

    def __init__(self, out_dir="raster_data"):
        os.makedirs(out_dir, exist_ok=True)
        session_id = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(out_dir, f"session_{session_id}.csv")
        self.images_dir = os.path.join(out_dir, "images", f"session_{session_id}")
        os.makedirs(self.images_dir, exist_ok=True)
        self.raster_number = 0
        with open(self.path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["raster_number", "x_mm", "y_mm", "charge_v", "timestamp", "image_path"])

    def peek_next_raster_number(self):
        """The raster_number that the NEXT log_raster() call will assign -
        used to name/save the pre-raster image with the same number it'll
        end up tagged with in the CSV, since the image is captured before
        log_raster() runs (which only happens once the raster finishes)."""
        return self.raster_number + 1

    def save_image(self, frame, raster_number):
        """Saves a captured frame (numpy array) into this session's images
        folder, named by raster number. Returns the path written, or "" if
        frame is None (e.g. the capture failed) - callers should treat ""
        as 'no image for this raster', not an error."""
        if frame is None:
            return ""
        from PIL import Image
        path = os.path.join(self.images_dir, f"raster_{raster_number:04d}.jpg")
        Image.fromarray(frame).save(path)
        return path

    def log_raster(self, samples, image_path=""):
        """Appends one raster's samples to the session file, tagged with
        the next raster_number and the given image_path (repeated on every
        row for that raster, same pattern as raster_number). Returns
        (path, raster_number)."""
        self.raster_number += 1
        with open(self.path, "a", newline="") as f:
            writer = csv.writer(f)
            for (x, y, charge, ts) in samples:
                writer.writerow([self.raster_number, x, y, charge, ts, image_path])
        return self.path, self.raster_number