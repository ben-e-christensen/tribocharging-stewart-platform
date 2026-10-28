"""
Particle detection + frame-to-frame tracking for the Basler stream.

Pure numpy + OpenCV (no scipy/trackpy) and pure functions of the frames
you hand it - no camera, Tkinter, or threading in here. wobble_track.py
drives it live; the same classes can re-run on a recorded video offline to
tune parameters.

DETECTION (ParticleDetector) - local background subtraction, not a global
threshold and not a static background model:
  - A static "empty platform" background frame doesn't work here: the
    platform itself tilts and shifts in the image while it wobbles, so
    every edge/hole/screw on it would show up as a "particle".
  - A single global threshold doesn't work either: illumination across the
    platform isn't even (see test_scripts/basler_test.png - most of the
    frame is near-black with brighter patches).
  - So each pixel is compared against a box-blurred copy of its own
    neighborhood (`background_px` wide). Anything much brighter (or darker,
    for `polarity="dark"`) than its surroundings by more than `threshold`
    gray levels, and between min_area..max_area pixels in size, is a
    particle. background_px should be a few times bigger than a particle,
    or big particles get partly subtracted away as "background".
  - cv2.blur (box filter) rather than GaussianBlur for the background:
    its cost doesn't grow with kernel size, which matters at ~225 fps.
  - Optional circular mask (set_mask_circle) to ignore everything off the
    platform surface - the frame edges are full of rig hardware.

TRACKING (ParticleTracker) - greedy nearest-neighbor linking with a
constant-velocity prediction:
  - Each live track predicts where it'll be at the new frame's timestamp
    (last position + velocity * dt), and detections are matched to the
    closest prediction within `max_link_px`, closest pairs first. Only
    pairs that are within max_link_px in x are ever considered (sorted-x
    window search), so this stays ~linear with hundreds of particles
    instead of building a dense tracks x detections matrix. With
    the camera at 100-200 fps, particles move a few px per frame, so this
    is reliable as long as particles aren't packed closer together than
    they move per frame - if IDs start swapping, raise the frame rate (or
    shrink the ROI so the camera CAN go faster) before reaching for a
    fancier (Hungarian / multi-hypothesis) linker.
  - A track survives up to `max_missed` frames without a detection (brief
    occlusion, a particle dimming as it rolls) before being retired.
  - Velocity uses the camera's hardware timestamp deltas, so frames the
    pipeline skipped don't corrupt it - dt just gets bigger.
"""

import collections
import itertools

import cv2
import numpy as np


class ParticleDetector:
    def __init__(self, polarity="bright", threshold=12, background_px=31,
                 min_area=3, max_area=2000, smooth_px=3):
        self.polarity = polarity        # "bright" = particles lighter than surroundings
        self.threshold = threshold      # gray levels above local background
        self.background_px = background_px
        self.min_area = min_area
        self.max_area = max_area
        self.smooth_px = smooth_px      # small blur to stop sensor noise splitting blobs
        self.mask = None                # uint8 (H, W), 255 = detect here
        self.mask_circle = None         # (cx, cy, r) in image px, for display
        self.last_binary = None         # last thresholded image (cropped to the mask's
        self.last_offset = (0, 0)       # bounding box, at this offset), for the debug view

    def set_mask_circle(self, shape, cx, cy, r):
        mask = np.zeros(shape[:2], np.uint8)
        cv2.circle(mask, (int(cx), int(cy)), int(r), 255, -1)
        self.mask = mask
        self.mask_circle = (int(cx), int(cy), int(r))

    def clear_mask(self):
        self.mask = None
        self.mask_circle = None

    def detect(self, img):
        """img: Mono8 (H, W). Returns float32 array (N, 3) of [x, y, area]
        in full-image pixel coordinates."""
        # With a mask set, only process its bounding box - every step below
        # is per-pixel, so this is a straight speedup (~40% for a typical
        # platform-sized circle).
        x0 = y0 = 0
        mask = self.mask if self.mask is not None and self.mask.shape == img.shape else None
        if mask is not None:
            cx, cy, r = self.mask_circle
            x0, y0 = max(0, cx - r), max(0, cy - r)
            x1, y1 = min(img.shape[1], cx + r + 1), min(img.shape[0], cy + r + 1)
            img = img[y0:y1, x0:x1]
            mask = mask[y0:y1, x0:x1]
        self.last_offset = (x0, y0)

        k = max(3, int(self.background_px) | 1)  # odd
        bg = cv2.blur(img, (k, k))
        diff = cv2.subtract(img, bg) if self.polarity == "bright" else cv2.subtract(bg, img)
        if self.smooth_px > 1:
            s = int(self.smooth_px) | 1
            diff = cv2.GaussianBlur(diff, (s, s), 0)
        _, binary = cv2.threshold(diff, int(self.threshold), 255, cv2.THRESH_BINARY)
        if mask is not None:
            binary = cv2.bitwise_and(binary, mask)
        self.last_binary = binary  # cropped - see last_offset

        # Fixed per-pixel cost regardless of particle count: label, then do
        # all per-blob sums with bincount over just the lit pixels, so 500
        # particles cost about the same as 5. (A findContours + per-blob
        # cv2.moments loop is faster for a handful of blobs but ~10ms at the
        # ~500 the real platform shows; cv2.findNonZero is ~3x faster than
        # np.nonzero here.) Centroids are weighted by the background-
        # subtracted intensity -> sub-pixel positions.
        n, labels = cv2.connectedComponents(binary, connectivity=8, ltype=cv2.CV_32S)
        pts = cv2.findNonZero(binary)
        if n <= 1 or pts is None:
            return np.empty((0, 3), np.float32)
        pts = pts.reshape(-1, 2)  # OpenCV 4 gives (K,1,2), OpenCV 5 (K,2)
        xs, ys = pts[:, 0], pts[:, 1]
        lab = labels[ys, xs]
        w = diff[ys, xs].astype(np.float64)
        areas = np.bincount(lab, minlength=n)
        sw = np.bincount(lab, w, minlength=n)
        sx = np.bincount(lab, w * xs, minlength=n)
        sy = np.bincount(lab, w * ys, minlength=n)
        keep = (areas >= self.min_area) & (areas <= self.max_area) & (sw > 0)
        keep[0] = False  # label 0 is the background
        return np.column_stack([x0 + sx[keep] / sw[keep], y0 + sy[keep] / sw[keep],
                                areas[keep]]).astype(np.float32)


class Track:
    __slots__ = ("id", "x", "y", "vx", "vy", "area", "t", "missed", "age", "trail")

    def __init__(self, track_id, x, y, area, t, trail_len):
        self.id = track_id
        self.x, self.y = x, y
        self.vx = self.vy = 0.0     # px/s
        self.area = area
        self.t = t
        self.missed = 0
        self.age = 1                # frames with a detection
        self.trail = collections.deque([(x, y)], maxlen=trail_len)


class ParticleTracker:
    def __init__(self, max_link_px=25.0, max_missed=3, trail_len=60,
                 velocity_smoothing=0.5):
        self.max_link_px = max_link_px
        self.max_missed = max_missed
        self.trail_len = trail_len
        self.velocity_smoothing = velocity_smoothing  # 0 = raw per-frame velocity
        self.tracks = {}
        self._ids = itertools.count(1)

    def reset(self):
        self.tracks.clear()

    def update(self, detections, t):
        """detections: (N, 3) [x, y, area] from ParticleDetector.detect().
        t: frame time in seconds (camera hardware timestamp).
        Returns the tracks matched/created THIS frame (list of Track)."""
        tracks = list(self.tracks.values())
        n_det = len(detections)
        det_used = np.zeros(n_det, bool)
        matched = []

        track_used = np.zeros(len(tracks), bool)
        if tracks and n_det:
            state = np.array([(tr.x, tr.y, tr.vx, tr.vy, tr.t) for tr in tracks])
            dt = t - state[:, 4]
            px = state[:, 0] + state[:, 2] * dt
            py = state[:, 1] + state[:, 3] * dt
            ti, di, dist = self._candidate_pairs(px, py, detections)
            # Greedy, closest pairs first. Plain Python lists - with only a
            # few candidates per track this loop is far cheaper than any
            # dense T x N matrix work.
            det_used_l = [False] * n_det
            track_used_l = [False] * len(tracks)
            for k in np.argsort(dist).tolist():
                i, j = ti[k], di[k]
                if track_used_l[i] or det_used_l[j]:
                    continue
                track_used_l[i] = det_used_l[j] = True
                self._advance(tracks[i], detections[j], t)
                matched.append(tracks[i])
            track_used = np.array(track_used_l)
            det_used = np.array(det_used_l)

        for tr, used in zip(tracks, track_used):
            if not used:
                tr.missed += 1
                if tr.missed > self.max_missed:
                    del self.tracks[tr.id]

        for d in detections[~det_used]:
            tr = Track(next(self._ids), float(d[0]), float(d[1]), float(d[2]), t, self.trail_len)
            self.tracks[tr.id] = tr
            matched.append(tr)

        return matched

    def _candidate_pairs(self, px, py, detections):
        """All (track, detection) pairs within max_link_px, without building
        a dense T x N distance matrix: sort detections by x, binary-search
        each prediction's [x - L, x + L] window, expand those index ranges
        (vectorized), then distance-check only those few pairs. Returns
        (track_idx list, det_idx list, distances array)."""
        L = self.max_link_px
        order = np.argsort(detections[:, 0])
        sx = detections[order, 0]
        lo = np.searchsorted(sx, px - L, "left")
        hi = np.searchsorted(sx, px + L, "right")
        counts = hi - lo
        total = int(counts.sum())
        if total == 0:
            return [], [], np.empty(0)
        ti = np.repeat(np.arange(len(px)), counts)
        starts = np.repeat(lo - (np.cumsum(counts) - counts), counts)
        di = order[starts + np.arange(total)]
        dist = np.hypot(px[ti] - detections[di, 0], py[ti] - detections[di, 1])
        ok = dist <= L
        return ti[ok].tolist(), di[ok].tolist(), dist[ok]

    def _advance(self, tr, det, t):
        x, y, area = det.tolist()
        dt = t - tr.t
        if dt > 0:
            vx, vy = (x - tr.x) / dt, (y - tr.y) / dt
            a = self.velocity_smoothing if tr.age > 1 else 0.0
            tr.vx = a * tr.vx + (1 - a) * vx
            tr.vy = a * tr.vy + (1 - a) * vy
        tr.x, tr.y, tr.area, tr.t = x, y, area, t
        tr.missed = 0
        tr.age += 1
        tr.trail.append((x, y))
