"""
CameraView - displays the LAST captured still image as a panel embedded in
the Tkinter window (sitting alongside the charge heatmap). Needs Pillow
(pip install pillow) to convert a numpy array (Mono8 from the Basler, see
camera_controller.py - PIL takes 2-D grayscale arrays as-is) into
something Tkinter can display.

IMPORTANT: like LiveHeatmap, show_array() touches a Tkinter widget, so it
must only ever be called from the MAIN thread. Capturing the image itself
(CameraController.capture()) is a blocking hardware call that's fine to do
on a background thread - just hop to the main thread before calling
show_array() with the result (see gui.py).
"""

import tkinter as tk
from PIL import Image, ImageTk


class CameraView:
    def __init__(self, parent_frame, display_size=(320, 240)):
        self.display_size = display_size
        self.label = tk.Label(parent_frame, text="No image yet", bg="gray20", fg="white")
        self.label.pack(fill="both", expand=True)
        # Tkinter/PIL gotcha: PhotoImage gets garbage-collected if nothing
        # keeps a reference to it, and the label would silently go blank -
        # keeping it as an instance attribute prevents that.
        self._imgtk = None

    def show_array(self, frame):
        """frame: a numpy array (Mono8 or RGB) from CameraController.capture()."""
        img = Image.fromarray(frame)
        img = img.resize(self.display_size)
        self._imgtk = ImageTk.PhotoImage(img)
        self.label.config(image=self._imgtk, text="")
