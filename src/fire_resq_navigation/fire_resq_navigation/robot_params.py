"""Robot geometry and limits for navigation, derived from ONE source: parameters.xacro.

Nav2 needs a footprint and velocity limits; the robot description already owns those numbers
(wheel radius/separation, chassis, magnet, drive limits). Restating them in a Nav2 YAML would
create a second copy that silently drifts, so navigation READS them from the description at
launch time and never hardcodes them.

Pure Python (no ROS import beyond an optional ament lookup), so it is unit-testable.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

from fire_resq_description.geometry import find_parameters_xacro, read_properties  # noqa: F401  (re-exported)

Point = Tuple[float, float]


def convex_hull(points: List[Point]) -> List[Point]:
    """Andrew's monotone chain; returns the hull counter-clockwise."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _rect(x0: float, x1: float, y0: float, y1: float) -> List[Point]:
    return [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]


@dataclass(frozen=True)
class RobotParams:
    wheel_radius: float
    wheel_separation: float
    max_linear_velocity: float
    max_angular_velocity: float
    max_linear_acceleration: float
    max_angular_acceleration: float
    footprint: Tuple[Point, ...]

    @property
    def circumscribed_radius(self) -> float:
        """Farthest footprint point from base_link (the rotation centre)."""
        return max(math.hypot(x, y) for x, y in self.footprint)

    @property
    def inscribed_radius(self) -> float:
        """Nearest footprint EDGE to base_link."""
        best, n = 1e9, len(self.footprint)
        for i in range(n):
            (x1, y1), (x2, y2) = self.footprint[i], self.footprint[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
        return best

    def footprint_string(self) -> str:
        """Nav2 footprint parameter format."""
        return '[' + ', '.join(f'[{x:.3f}, {y:.3f}]' for x, y in self.footprint) + ']'


def load_robot_params(path: Optional[Path] = None) -> RobotParams:
    p = read_properties(path)
    half_track = p['wheel_separation'] / 2 + p['wheel_width'] / 2
    pts: List[Point] = []
    # chassis (offset from the axle by chassis_x_offset; base_link is the axle midpoint)
    cx = p['chassis_x_offset']
    pts += _rect(cx - p['chassis_length'] / 2, cx + p['chassis_length'] / 2, -p['chassis_width'] / 2, p['chassis_width'] / 2)
    # drive wheels: the widest part of the robot
    pts += _rect(-p['wheel_radius'], p['wheel_radius'], -half_track, half_track)
    # magnet: cylinder along +x at the front
    pts += _rect(p['magnet_x'] - p['magnet_length'] / 2, p['magnet_x'] + p['magnet_length'] / 2,
                 -p['magnet_radius'], p['magnet_radius'])
    # camera mount
    h = p['camera_size'] * 2.4 / 2
    pts += _rect(p['camera_x'] - p['camera_size'] / 2, p['camera_x'] + p['camera_size'] / 2, -h, h)
    # passive caster
    pts += _rect(p['caster_x'] - p['caster_radius'], p['caster_x'] + p['caster_radius'],
                 -p['caster_radius'], p['caster_radius'])
    hull = tuple((round(x, 4), round(y, 4)) for x, y in convex_hull(pts))
    return RobotParams(p['wheel_radius'], p['wheel_separation'], p['max_linear_velocity'],
                       p['max_angular_velocity'], p['max_linear_acceleration'],
                       p['max_angular_acceleration'], hull)
