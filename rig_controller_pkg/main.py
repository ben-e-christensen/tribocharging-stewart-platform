"""
main.py - entry point. Run this file to launch the whole rig controller.
"""

import tkinter as tk

from klipper_controller import KlipperController
from gui import JogGUI


if __name__ == "__main__":
    root = tk.Tk()
    printer = KlipperController("127.0.0.1")
    app = JogGUI(root, printer)
    root.mainloop()
