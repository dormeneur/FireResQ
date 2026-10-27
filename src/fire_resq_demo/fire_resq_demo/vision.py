"""What one camera frame says: where the bottles are, and where the floor stops. Pure (numpy + OpenCV, no ROS).

There is ONE camera and no range sensor, so every distance here comes from one assumption: things stand on a
flat floor. A pixel row below the horizon is then a distance (`Camera.ground_range`). That needs two numbers
nobody has measured yet, the lens height and the horizon row; until they are, distances are relative, which is
enough to say which bottle is closer.

Bottles are found two ways, because a black bottle has no colour to key on:
  - saturated blobs, grouped by hue ("red", "blue", ...);
  - very dark blobs ("black").
Either must be upright, solid and standing on the floor. Transparent, white or grey bottles are NOT found.

Obstacles are found by appearance (after Ulrich & Nourbakhsh, 2000): learn what the floor just in front of the
robot looks like, and call the first thing up each image column that does not look like it an obstacle.
It is a colour test, not a range measurement: a floor-coloured object is invisible to it, and a mark on the
floor that is big and dark enough is an obstacle to it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .colours import PICKED, colour_name

@dataclass(frozen=True)
class Camera:
    """Pinhole camera looking along the floor. `horizon_row`: the image row at the camera's own height (the centre
    row if the camera is level; higher up the image if it is tilted down)."""
    f: float = 768.0
    cx: float = 320.0
    width: int = 640
    height: int = 480
    height_m: float = 0.05
    horizon_row: float = 200.0

    def bearing(self, u: float) -> float:
        """Angle of image column u, radians, positive to the LEFT (ROS convention)."""
        return math.atan2(self.cx - u, self.f)

    def ground_range(self, row: float) -> Optional[float]:
        """Forward distance (m) of the floor point seen at this row; None at or above the horizon."""
        below = row - self.horizon_row
        return self.f * self.height_m / below if below > 2.0 else None

    @property
    def blind_range(self) -> float:
        """The nearest floor the camera can see (the bottom row)."""
        return self.ground_range(self.height - 1) or 0.0


@dataclass
class VisionParams:
    sat_min: int = 120                  # a "coloured" pixel is at least this saturated ...
    val_min: int = 60                   # ... and this bright (dark pixels have meaningless hue)
    dark_max: int = 40                  # a "black" pixel is at most this bright
    detect_black: bool = True
    min_area_px: int = 400
    min_aspect: float = 1.2             # height / width of an upright bottle (relaxed when it runs off the image)
    max_aspect: float = 8.0
    min_fill: float = 0.55              # blob area / bounding box area: a bottle is a filled rectangle
    hue_merge: float = 12.0             # pixels within this many hue units belong to one object
    min_height_m: float = 0.06          # size on the floor-plane assumption; 0 disables the size gate
    max_height_m: float = 0.60
    horizon_slack_px: int = 30          # the horizon row is a guess (the camera's tilt changes when it is nudged): a blob
                                        # whose base is up to this far ABOVE it still counts as standing on the floor
    size_gate_px: int = 30              # ... and its size is only judged when its base is this far BELOW it
    pick_l_tol: float = 40.0            # the clicked object: pixels this close to the clicked colour (Lab L, then a/b)
    pick_ab_tol: float = 12.0
    # floor / obstacles
    floor_l_tol: float = 45.0           # darker than the floor by more than this (Lab L, 0..255) = not floor
    floor_ab_tol: float = 7.0           # differently coloured by more than this (Lab a/b) = not floor
    obstacle_run_px: int = 14           # an obstacle is at least this tall in the image (grout lines are not)
    strips: int = 32


@dataclass
class Blob:
    """One bottle candidate. `hue` is None for a black one. Pixel box x, y, w, h; `why` is set when rejected."""
    hue: Optional[float]
    x: int
    y: int
    w: int
    h: int
    area: int
    cut_top: bool = False
    cut_bottom: bool = False
    cut_side: bool = False
    partial: bool = False               # would be a bottle, but runs off the side of the picture: width is a lower bound
    bearing: float = 0.0
    range_m: float = 0.0
    why: str = ''

    @property
    def name(self) -> str:
        return colour_name(self.hue)


def _circular_mean(hues: np.ndarray) -> float:
    ang = hues.astype(np.float32) * (math.pi / 90.0)
    return float(math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0 % 180.0)


FAR_M = 3.0                             # the range given to something whose base is at the horizon: "far away"


def _place(blob: Blob, cam: Camera) -> Blob:
    """Fill in the cut flags, bearing and range."""
    bottom = blob.y + blob.h
    blob.cut_top, blob.cut_bottom = blob.y <= 1, bottom >= cam.height - 1
    blob.cut_side = blob.x <= 1 or blob.x + blob.w >= cam.width - 1
    blob.bearing = cam.bearing(blob.x + blob.w / 2.0)
    ground = cam.ground_range(bottom)
    blob.range_m = min(ground, cam.blind_range) if blob.cut_bottom and ground else min(ground or FAR_M, FAR_M)
    return blob


def _judge(blob: Blob, cam: Camera, p: VisionParams) -> Blob:
    """Fill in bearing and range, and say why this is not a bottle (if it is not)."""
    _place(blob, cam)
    bottom = blob.y + blob.h
    aspect = blob.h / max(blob.w, 1)
    cut = blob.cut_top or blob.cut_bottom
    if bottom < cam.horizon_row - p.horizon_slack_px:
        blob.why = 'not on the floor'
    elif not blob.cut_side and (aspect < (0.45 if cut else p.min_aspect) or aspect > p.max_aspect):
        blob.why = 'shape'
    elif blob.area < p.min_fill * blob.w * blob.h:
        blob.why = 'not solid'
    elif p.min_height_m > 0 and not blob.cut_bottom and not blob.cut_top and bottom - cam.horizon_row >= p.size_gate_px:
        # Only when the whole object is in view and its base is well below the horizon: otherwise this division is
        # mostly the error in the horizon guess (it called a 26 cm bottle "too big" on the floor, 2026-10-13).
        tall = cam.height_m * blob.h / (bottom - cam.horizon_row)
        if tall < p.min_height_m:
            blob.why = 'too small'
        elif tall > p.max_height_m:
            blob.why = 'too big'
    if blob.cut_side and not blob.why:                           # its width, so its shape and centre, are unknown:
        blob.why, blob.partial = 'half out of view', True        # not a bottle to go and find, but enough to keep
    return blob                                                  # steering at one the robot is already next to


def _components(mask: np.ndarray, min_area: int):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area >= min_area:
            yield i, labels, (x, y, w, h, area)


def find_bottles(bgr: np.ndarray, cam: Camera, p: VisionParams) -> List[Blob]:
    """Every bottle-like blob, accepted (`why == ''`) and rejected, nearest first."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    blobs: List[Blob] = []

    coloured = ((sat >= p.sat_min) & (val >= p.val_min)).astype(np.uint8)
    coloured = cv2.morphologyEx(coloured, cv2.MORPH_OPEN, kernel)
    for i, labels, (x, y, w, h, _) in _components(coloured, p.min_area_px):
        inside = labels[y:y + h, x:x + w] == i
        hues = hue[y:y + h, x:x + w]
        left = inside.copy()
        for _ in range(3):                                           # one region may be two touching bottles
            if int(left.sum()) < p.min_area_px:
                break
            hist = np.bincount(hues[left] // 6, minlength=30).astype(np.float32)
            hist = hist + np.roll(hist, 1) + np.roll(hist, -1)
            peak = float(np.argmax(hist)) * 6.0 + 3.0
            d = np.abs(hues.astype(np.float32) - peak)
            near = left & (np.minimum(d, 180.0 - d) <= p.hue_merge)
            left = left & ~near
            part = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
            best = max(_components(part, p.min_area_px), key=lambda c: c[2][4], default=None)
            if best is None:
                continue
            j, sub, (bx, by, bw, bh, area) = best
            mean_hue = _circular_mean(hues[(sub == j) & near])
            blobs.append(_judge(Blob(mean_hue, x + bx, y + by, bw, bh, area), cam, p))

    if p.detect_black:
        dark = cv2.morphologyEx((val <= p.dark_max).astype(np.uint8), cv2.MORPH_OPEN, kernel)
        for _, _, (x, y, w, h, area) in _components(dark, p.min_area_px):
            blobs.append(_judge(Blob(None, x, y, w, h, area), cam, p))
    return sorted(blobs, key=lambda b: b.range_m)


def find_picked(bgr: np.ndarray, lab_ref: np.ndarray, near: Tuple[float, float], cam: Camera,
                p: VisionParams) -> Optional[Blob]:
    """The object the operator clicked: the region coloured like the clicked spot that is nearest to where it was
    last seen (`near`, pixels). No shape rules at all: the operator said it is the target."""
    lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    d = np.abs(lab - lab_ref)
    mask = ((d[..., 0] <= p.pick_l_tol) & (d[..., 1] <= p.pick_ab_tol) & (d[..., 2] <= p.pick_ab_tol)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    best, best_score = None, math.inf
    for _, _, (x, y, w, h, area) in _components(mask, max(60, p.min_area_px // 4)):
        dx = max(x - near[0], 0.0, near[0] - (x + w))             # how far the last position is outside this box
        dy = max(y - near[1], 0.0, near[1] - (y + h))
        score = math.hypot(dx, dy) - 0.002 * area                 # nearest first, then the bigger region
        if score < best_score:
            best, best_score = Blob(PICKED, x, y, w, h, area), score
    return _place(best, cam) if best is not None else None


def pick_colour(bgr: np.ndarray, u: float, v: float, half: int = 7) -> np.ndarray:
    """The Lab colour of the spot the operator clicked (median of a small patch)."""
    h, w = bgr.shape[:2]
    u, v = int(min(max(u, half), w - half - 1)), int(min(max(v, half), h - half - 1))
    patch = cv2.cvtColor(bgr[v - half:v + half + 1, u - half:u + half + 1], cv2.COLOR_BGR2LAB)
    return np.median(patch.reshape(-1, 3), axis=0).astype(np.float32)


# ---------------------------------------------------------------------------------------------------- floor
@dataclass
class FloorScan:
    """Where the floor stops, per image strip (left to right). `row` is NaN where the floor runs to the horizon;
    `x`, `y` are that point on the floor in metres (forward, left) and `bearing` the strip's direction."""
    u: np.ndarray
    row: np.ndarray
    x: np.ndarray
    y: np.ndarray
    bearing: np.ndarray
    learned: bool = True

    def points(self) -> np.ndarray:
        ok = np.isfinite(self.x)
        return np.stack([self.x[ok], self.y[ok], self.bearing[ok]], axis=1)


@dataclass
class FloorModel:
    """The floor's colour, learned from the patch of floor right in front of the robot and tracked slowly."""
    mean: Optional[np.ndarray] = None
    scale: int = 4
    patch: Tuple[float, float, float, float] = (0.30, 0.86, 0.70, 0.98)     # x0, y0, x1, y1 as image fractions
    _history: List[float] = field(default_factory=list)

    def forget(self) -> None:
        self.mean = None

    def scan(self, bgr: np.ndarray, cam: Camera, p: VisionParams) -> Tuple[FloorScan, np.ndarray]:
        """Returns the scan and the small not-floor mask (for drawing)."""
        s = self.scale
        small = cv2.resize(bgr, (cam.width // s, cam.height // s), interpolation=cv2.INTER_AREA)
        lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB).astype(np.float32)
        hh, ww = lab.shape[:2]
        x0, y0, x1, y1 = self.patch
        patch = lab[int(y0 * hh):int(y1 * hh), int(x0 * ww):int(x1 * ww)].reshape(-1, 3)
        if self.mean is None:
            self.mean = np.median(patch, axis=0)
        not_floor = self._not_floor(lab, p)
        patch_ok = ~self._not_floor(patch[None], p)[0]
        if patch_ok.mean() > 0.6:                                    # still floor in front: follow slow light changes
            self.mean = 0.9 * self.mean + 0.1 * patch[patch_ok].mean(axis=0)

        n = p.strips
        per = ww // n
        strip = not_floor[:, :per * n].reshape(hh, n, per).mean(axis=2) >= 0.6
        k = max(1, int(round(p.obstacle_run_px / s)))
        run = strip.copy()
        for i in range(1, k):                                        # run[r]: rows r-k+1 .. r are all not-floor
            run[i:] &= strip[:-i]
        run[:k - 1] = False
        run[:max(0, int(math.ceil(cam.horizon_row / s)) + k)] = False      # nothing above the horizon is on the floor
        rows = np.arange(hh)[:, None]
        lowest = np.where(run, rows, -1).max(axis=0)                 # nearest obstacle = lowest such row
        u = (np.arange(n) + 0.5) * per * s
        row = np.where(lowest >= 0, (lowest + 1) * s, np.nan).astype(np.float32)
        with np.errstate(invalid='ignore', divide='ignore'):
            x = cam.f * cam.height_m / (row - cam.horizon_row)
        y = (cam.cx - u) * x / cam.f
        bearing = np.arctan2(cam.cx - u, cam.f)
        return FloorScan(u, row, x.astype(np.float32), y.astype(np.float32), bearing.astype(np.float32)), not_floor

    def _not_floor(self, lab: np.ndarray, p: VisionParams) -> np.ndarray:
        d = lab - self.mean
        # Brighter than the floor is allowed: a glossy floor seen from 5 cm up is full of reflected lights.
        return (d[..., 0] < -p.floor_l_tol) | (np.abs(d[..., 1]) > p.floor_ab_tol) | (np.abs(d[..., 2]) > p.floor_ab_tol)
