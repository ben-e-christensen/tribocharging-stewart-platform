"""
LiveHeatmap - a SINGLE persistent charge-map panel, embedded directly inside
the Tkinter window (via FigureCanvasTkAgg). This is what guarantees "one
window" no matter how many rasters run: there's exactly one Figure, created
once, and every update just repaints its existing grid - nothing new ever
gets opened.

IMPORTANT: every method here touches Tkinter/matplotlib GUI state, so they
must only ever be called from the MAIN thread (the one running
root.mainloop()). Background threads (like the raster sampling loop) should
never call these directly - they should hand data off through a queue
instead, and something on the main thread (see gui.py's queue-draining
timer) should be the only thing calling add_point()/refresh().
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class LiveHeatmap:
    def __init__(self, parent_frame, extent, resolution=150, cmap="viridis",
                 point_radius=0.0, vmin=None, vmax=None):
        """
        parent_frame: a Tkinter frame/widget to embed the plot into.
        extent: (xmin, xmax, ymin, ymax) - the physical coordinate range
                this heatmap covers (typically the raster's bounding box).
        resolution: grid is resolution x resolution cells.
        point_radius: physical radius (mm) of the sensor's footprint. Each
                sample paints a DISC of this radius, not a single cell -
                since the probe has real physical size, it's actually
                "seeing" an area around its center, not an infinitesimal
                point. This matters especially when the raster's line
                spacing is smaller than the sensor's diameter (adjacent
                lines' footprints genuinely overlap in reality) - painting
                single pixels per line would leave misleading gaps between
                lines that the sensor never actually had.
        vmin, vmax: FIXED color-scale limits. If both given, every raster
                uses the SAME color scale, so a given reading always maps
                to the same color regardless of what else happened in that
                particular raster - makes readings comparable ACROSS
                rasters. If left as None (default), the scale auto-fits to
                each raster's own min/max, which is easier to read for any
                single raster in isolation but not comparable between them.
        """
        self.extent = extent
        self.resolution = resolution
        self.xmin, self.xmax, self.ymin, self.ymax = extent
        self.point_radius = point_radius
        self.vmin = vmin
        self.vmax = vmax

        self.grid = np.full((resolution, resolution), np.nan)

        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.im = self.ax.imshow(
            self.grid, extent=extent, origin="lower", cmap=cmap, aspect="equal"
        )
        if self.vmin is not None and self.vmax is not None:
            self.im.set_clim(vmin=self.vmin, vmax=self.vmax)
        self.fig.colorbar(self.im, ax=self.ax, label="Charge (V)")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_title("Waiting for data...")

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Physical setup orientation: the origin (smallest x, smallest y)
        # should appear at the TOP-RIGHT of the displayed plot, not the
        # default bottom-left. This only flips how it's DRAWN - the
        # underlying grid/index math (and disc-painting) are untouched.
        self.ax.invert_xaxis()
        self.ax.invert_yaxis()

    def _xy_to_index(self, x, y):
        col = int((x - self.xmin) / (self.xmax - self.xmin) * (self.resolution - 1))
        row = int((y - self.ymin) / (self.ymax - self.ymin) * (self.resolution - 1))
        col = max(0, min(self.resolution - 1, col))
        row = max(0, min(self.resolution - 1, row))
        return row, col

    def add_point(self, x, y, value):
        """Paints a disc of radius self.point_radius (mm) around (x,y) with
        this value - or a single cell if point_radius is 0. Does NOT redraw
        by itself - batch several add_point() calls and follow with one
        refresh(), rather than refreshing after every single point (which
        would be needlessly expensive at high sample rates).

        Later points overwrite earlier ones in any overlapping area (simple
        last-write-wins) - fine for a live monitoring view; if you wanted
        the overlap zones averaged instead, that'd need per-cell counters,
        which isn't implemented here."""
        if self.point_radius <= 0:
            row, col = self._xy_to_index(x, y)
            self.grid[row, col] = value
            return

        cell_w = (self.xmax - self.xmin) / (self.resolution - 1)
        cell_h = (self.ymax - self.ymin) / (self.resolution - 1)
        row_c, col_c = self._xy_to_index(x, y)
        row_span = int(np.ceil(self.point_radius / cell_h))
        col_span = int(np.ceil(self.point_radius / cell_w))

        for dr in range(-row_span, row_span + 1):
            r = row_c + dr
            if r < 0 or r >= self.resolution:
                continue
            phys_dy = dr * cell_h
            for dc in range(-col_span, col_span + 1):
                c = col_c + dc
                if c < 0 or c >= self.resolution:
                    continue
                phys_dx = dc * cell_w
                if phys_dx * phys_dx + phys_dy * phys_dy <= self.point_radius * self.point_radius:
                    self.grid[r, c] = value

    def refresh(self, title=None):
        """Pushes the current grid to the on-screen canvas."""
        self.im.set_data(self.grid)
        if self.vmin is not None and self.vmax is not None:
            self.im.set_clim(vmin=self.vmin, vmax=self.vmax)
        else:
            finite = self.grid[np.isfinite(self.grid)]
            if finite.size > 0:
                self.im.set_clim(vmin=float(finite.min()), vmax=float(finite.max()))
        if title:
            self.ax.set_title(title)
        self.canvas.draw_idle()

    def clear(self, title="Waiting for data..."):
        """Blanks the grid for a fresh raster pass - same window, fresh
        contents, so you're always looking at the CURRENT raster, not a
        blend of every pass ever run."""
        self.grid[:] = np.nan
        self.refresh(title=title)