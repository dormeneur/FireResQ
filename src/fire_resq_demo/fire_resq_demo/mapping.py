"""Where the robot believes it is, and the map it draws on the way. Pure (numpy + OpenCV, no ROS).

THIS IS NOT SLAM, and must not be called that. The robot has no encoders, no lidar and no depth camera:
  - the pose is /odom (dead reckoning from the power sent to the motors), with its turning rescaled by what the
    bottles looked like before and after each turn pulse (`yaw_gain`, set by the seeker). Nothing ever corrects
    its position, so it drifts;
  - the map is that pose plus the camera's "where the floor stops" points. An error in the pose is drawn
    straight into the map.
It is a live sketch of what the robot has seen and where it thinks it went. That is all.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .colours import colour_rgb


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PoseTracker:
    """Integrates /odom's increments. `raw_yaw` is /odom's own heading, unwrapped; `yaw` is the rescaled one."""

    def __init__(self):
        self.yaw_gain = 1.0
        self.reset()

    def reset(self) -> None:
        self.x = self.y = self.yaw = self.raw_yaw = 0.0
        self.trail: List[Tuple[float, float]] = [(0.0, 0.0)]
        self._last: Optional[Tuple[float, float, float]] = None

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def on_odom(self, ox: float, oy: float, oyaw: float) -> None:
        last, self._last = self._last, (ox, oy, oyaw)
        if last is None:
            return
        dyaw = wrap(oyaw - last[2])
        dx, dy = ox - last[0], oy - last[1]
        if math.hypot(dx, dy) > 0.5:                             # the driver restarted and /odom jumped: not motion
            return
        heading = last[2] + dyaw / 2.0
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        turned = self.yaw_gain * dyaw
        self.raw_yaw += dyaw
        self.x += forward * math.cos(self.yaw + turned / 2.0)
        self.y += forward * math.sin(self.yaw + turned / 2.0)
        self.yaw = wrap(self.yaw + turned)
        if math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.02:
            self.trail.append((self.x, self.y))
            del self.trail[:-4000]


class FloorMap:
    """A grid of how often each cell was seen as clear floor and as the place the floor stopped."""

    def __init__(self, size_m: float = 10.0, cell_m: float = 0.05, max_range_m: float = 1.5):
        self.cell, self.max_range = cell_m, max_range_m
        self.n = int(round(size_m / cell_m))
        self.reset()

    def reset(self) -> None:
        self.free = np.zeros((self.n, self.n), np.uint16)
        self.hit = np.zeros((self.n, self.n), np.uint16)

    def _index(self, wx: np.ndarray, wy: np.ndarray):
        ix = np.floor(wx / self.cell).astype(int) + self.n // 2
        iy = np.floor(wy / self.cell).astype(int) + self.n // 2
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return iy[ok], ix[ok]

    def add_scan(self, pose: Tuple[float, float, float], scan: np.ndarray) -> None:
        """scan: rows of (x forward, y left, bearing) in the robot frame; x NaN = floor clear to the horizon."""
        px, py, yaw = pose
        seen = np.isfinite(scan[:, 0])
        rng = np.where(seen, np.hypot(scan[:, 0], np.where(seen, scan[:, 1], 0.0)), np.inf)
        reach = np.minimum(rng, self.max_range)
        ang = yaw + scan[:, 2]
        steps = np.arange(0.0, self.max_range, self.cell * 0.7)
        d = steps[None, :]
        inside = d < (reach[:, None] - self.cell * 0.5)
        wx = px + d * np.cos(ang)[:, None]
        wy = py + d * np.sin(ang)[:, None]
        clear = np.zeros_like(self.free, dtype=bool)
        clear[self._index(wx[inside], wy[inside])] = True
        self.free[clear & (self.free < 60000)] += 1
        ends = seen & (rng <= self.max_range)
        stop = np.zeros_like(clear)
        stop[self._index(px + rng[ends] * np.cos(ang[ends]), py + rng[ends] * np.sin(ang[ends]))] = True
        self.hit[stop & (self.hit < 60000)] += 1

    def occupied(self) -> np.ndarray:
        return (self.hit >= 2) & (self.hit * 3 >= self.free)


# ------------------------------------------------------------------------------------------------- drawing
BACKGROUND, FLOOR, WALL, GRID, TRAIL, ROBOT = (28, 22, 18), (62, 50, 40), (225, 225, 235), (48, 40, 32), (240, 190, 60), (255, 255, 255)


def _bgr(rgb: Sequence[int]) -> Tuple[int, int, int]:
    return int(rgb[2]), int(rgb[1]), int(rgb[0])


def draw_map(size: int, tracker: PoseTracker, floor: FloorMap, bottles: Sequence[dict], hfov: float) -> np.ndarray:
    """Top-down picture, `size` px square. bottles: dicts with x, y, hue, status ('seen'/'target'/'visited'/'lost'),
    name. Forward at the start is UP."""
    occ = floor.occupied()
    xs = [p[0] for p in tracker.trail[::8]] + [tracker.x] + [b['x'] for b in bottles]
    ys = [p[1] for p in tracker.trail[::8]] + [tracker.y] + [b['y'] for b in bottles]
    cy, cx = np.nonzero(occ)
    if len(cx):
        ox, oy = (cx - floor.n // 2 + 0.5) * floor.cell, (cy - floor.n // 2 + 0.5) * floor.cell
        near = np.hypot(ox - tracker.x, oy - tracker.y) < 3.0
        xs += [float(ox[near].min()), float(ox[near].max())] if near.any() else []
        ys += [float(oy[near].min()), float(oy[near].max())] if near.any() else []
    span = max(max(xs) - min(xs), max(ys) - min(ys)) + 1.0
    span = max(2.5, math.ceil(span * 2.0) / 2.0)                  # in half-metre steps, so the view does not shimmer
    mid_x, mid_y = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    k = size / span                                               # px per metre

    def px(wx: float, wy: float) -> Tuple[int, int]:              # world +x (forward) is up, +y (left) is left
        return int(round(size / 2 - (wy - mid_y) * k)), int(round(size / 2 - (wx - mid_x) * k))

    cells = np.full((floor.n, floor.n, 3), BACKGROUND, np.uint8)
    cells[floor.free > 0] = FLOOR
    cells[occ] = WALL
    c, h = floor.cell * k, floor.n // 2                           # cell (ix, iy) -> picture: u = -c*iy + ..., v = -c*ix + ...
    m = np.float32([[0.0, -c, size / 2 + (h * floor.cell + mid_y) * k],
                    [-c, 0.0, size / 2 + (h * floor.cell + mid_x) * k]])
    img = cv2.warpAffine(cells, m, (size, size), flags=cv2.INTER_NEAREST, borderValue=BACKGROUND)

    first = math.floor((mid_x - span / 2) * 2.0) / 2.0
    for i in range(int(span * 2) + 3):                            # half-metre grid
        g = first + i * 0.5
        cv2.line(img, px(g, mid_y - span), px(g, mid_y + span), GRID, 1)
        gy = math.floor((mid_y - span / 2) * 2.0) / 2.0 + i * 0.5
        cv2.line(img, px(mid_x - span, gy), px(mid_x + span, gy), GRID, 1)

    if len(tracker.trail) > 1:
        pts = np.int32([px(*p) for p in tracker.trail]).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, TRAIL, 2, cv2.LINE_AA)
    cv2.circle(img, px(0.0, 0.0), 4, TRAIL, -1, cv2.LINE_AA)

    x, y, yaw = tracker.pose
    wedge = np.int32([px(x, y)] + [px(x + 1.2 * math.cos(yaw + a), y + 1.2 * math.sin(yaw + a))
                                   for a in np.linspace(hfov / 2, -hfov / 2, 9)])
    overlay = img.copy()
    cv2.fillPoly(overlay, [wedge], (120, 110, 60), cv2.LINE_AA)
    img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)

    for b in bottles:
        centre, colour = px(b['x'], b['y']), _bgr(colour_rgb(b['hue']))
        status = b['status']
        if status == 'target':
            cv2.line(img, px(x, y), centre, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(img, centre, 17, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(img, centre, 10, colour, -1, cv2.LINE_AA)
        cv2.circle(img, centre, 10, (150, 150, 150) if status == 'lost' else (255, 255, 255), 2, cv2.LINE_AA)
        if status == 'visited':
            tick = np.int32([(centre[0] - 6, centre[1]), (centre[0] - 2, centre[1] + 5), (centre[0] + 7, centre[1] - 6)])
            cv2.polylines(img, [tick.reshape(-1, 1, 2)], False, (90, 230, 120), 3, cv2.LINE_AA)
        elif status == 'lost':
            cv2.line(img, (centre[0] - 6, centre[1] - 6), (centre[0] + 6, centre[1] + 6), (150, 150, 150), 2, cv2.LINE_AA)
        cv2.putText(img, b['name'], (centre[0] + 15, centre[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)

    nose = np.float32([[0.13, 0.0], [-0.08, 0.075], [-0.04, 0.0], [-0.08, -0.075]])
    rot = np.float32([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    body = nose @ rot.T + np.float32([x, y])
    scale = max(1.0, 0.04 * span)                                 # keep the robot visible when zoomed out
    body = (body - np.float32([x, y])) * scale + np.float32([x, y])
    cv2.fillPoly(img, [np.int32([px(*p) for p in body])], ROBOT, cv2.LINE_AA)

    bar = int(round(k * 0.5))
    cv2.line(img, (16, size - 18), (16 + bar, size - 18), (235, 235, 235), 2)
    cv2.putText(img, '0.5 m', (22 + bar, size - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
    return img
