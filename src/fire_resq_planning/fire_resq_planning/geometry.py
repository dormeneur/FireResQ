"""Pure geometry for the rescue procedure. No ROS, no clock, no simulator: ordinary trigonometry, unit-tested.

Everything is in the `map` frame. `Pose2` is base_link (the drive-axle midpoint), so the magnet's front face is
`magnet_reach_m` ahead of it along the heading, and a victim in flush contact has its centre
`magnet_reach_m + victim_ring_radius_m` ahead. The same contact geometry the simulation's magnet driver applies
(`fire_resq_simulation.magnet_geometry`) and Phase 7/8's standoff guard derives - a unit test pins the three together.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from .config import PlanningConfig

XY = Tuple[float, float]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float

    @property
    def xy(self) -> XY:
        return (self.x, self.y)


@dataclass(frozen=True)
class Zone:
    """The safe zone: a rectangle in the map frame (centre, yaw, size). Known infrastructure, configured, never discovered."""
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float

    def local(self, p: XY) -> XY:
        dx, dy = p[0] - self.x, p[1] - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return (dx * c - dy * s, dx * s + dy * c)

    def world(self, local: XY) -> XY:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + local[0] * c - local[1] * s, self.y + local[0] * s + local[1] * c)

    def contains(self, p: XY, margin: float = 0.0) -> bool:
        """Inside the rectangle shrunk by `margin` on every side (a negative margin grows it)."""
        lx, ly = self.local(p)
        return abs(lx) <= self.size_x / 2 - margin and abs(ly) <= self.size_y / 2 - margin

    def depth(self, p: XY) -> float:
        """How far inside the nearest edge the point is (negative outside): the release margin actually achieved."""
        lx, ly = self.local(p)
        return min(self.size_x / 2 - abs(lx), self.size_y / 2 - abs(ly))


def bearing(frm: XY, to: XY) -> float:
    return math.atan2(to[1] - frm[1], to[0] - frm[0])


def dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------------------------------- the magnet
def magnet_face(pose: Pose2, cfg: PlanningConfig) -> XY:
    return (pose.x + cfg.magnet_reach_m * math.cos(pose.yaw), pose.y + cfg.magnet_reach_m * math.sin(pose.yaw))


def contact_gap(pose: Pose2, victim: XY, cfg: PlanningConfig) -> float:
    """Horizontal gap between the magnet's front face and the victim's ring (0 = flush, negative = past it)."""
    return dist(magnet_face(pose, cfg), victim) - cfg.victim_ring_radius_m


def expected_carried_xy(pose: Pose2, cfg: PlanningConfig) -> XY:
    """Where a victim held by the magnet is, given only the robot's pose: flush against the face, dead ahead.
    (An attached victim keeps the relative pose it had when the joint formed - i.e. this, to within the creep's precision.)"""
    d = cfg.contact_center_dist_m
    return (pose.x + d * math.cos(pose.yaw), pose.y + d * math.sin(pose.yaw))


def approach_pose(robot: XY, victim: XY, standoff_m: float) -> Pose2:
    """`standoff_m` short of the victim's centre on the robot->victim line, facing it - the contract cognition's
    RescueTarget.approach_pose fixes (a unit test compares the two). Already inside the standoff: stay, but face it."""
    d = dist(robot, victim)
    yaw = bearing(robot, victim)
    if d <= standoff_m or d <= 1e-9:
        return Pose2(robot[0], robot[1], yaw)
    f = (d - standoff_m) / d
    return Pose2(robot[0] + (victim[0] - robot[0]) * f, robot[1] + (victim[1] - robot[1]) * f, yaw)


# ------------------------------------------------------------------------------------------------ the safe zone
@dataclass(frozen=True)
class ReleasePlan:
    spot: XY            # where the victim's centre should end up (map frame)
    pose: Pose2         # where the robot must stand, facing the spot, for it to be there


def release_spots(zone: Zone, cfg: PlanningConfig) -> List[XY]:
    """Candidate victim positions ringing the zone's centre (never the centre itself: the return-leg planner query and
    the next victim's approach both need it free), generated from the zone's own geometry - no coordinates."""
    outer = min(zone.size_x, zone.size_y) / 2 - cfg.release_margin_m - cfg.release_spot_buffer_m
    radii = sorted({min(cfg.release_spot_radius_m, outer), outer})      # two rings: real releases land several cm off a
    n = cfg.release_spot_count                                          # planned spot, and one ring ran out of room at 3
    return [zone.world((r * math.cos(2 * math.pi * (k + 0.5 * i) / n), r * math.sin(2 * math.pi * (k + 0.5 * i) / n)))
            for i, r in enumerate(radii) for k in range(n)]


def plan_release(zone: Zone, robot: XY, used: Sequence[XY], cfg: PlanningConfig) -> Optional[ReleasePlan]:
    """Choose where to put the next victim down and where the robot must stand to do it.

    Spot: at least `release_spot_min_sep_m` from every victim already released; the INNER ring first (deepest in the zone:
    measured, a victim planned onto the outer ring, ~10 cm inside the edge, landed 8 cm outside it after 17 cm of
    localisation error on a long carry), and within a ring the one farthest from the robot - the zone fills from the far
    side, so a victim put down never stands between the entrance and the next spot (measured: the first victim left at the
    entry side made the controller abort the next return leg, "collision ahead"). Pose: `contact distance + a little` short of the spot on the line from the robot,
    facing it; the robot itself must also be inside the zone, or the carried victim would be dragged across its edge
    while the robot turns to face it. None if the zone has no room left."""
    best: Optional[Tuple[Tuple[float, float], XY]] = None
    centre = zone.world((0.0, 0.0))
    for s in release_spots(zone, cfg):
        if min((dist(s, u) for u in used), default=math.inf) < cfg.release_spot_min_sep_m:
            continue
        inner = dist(s, centre) <= cfg.release_spot_radius_m + 1e-6
        key = (1.0 if inner else 0.0, dist(robot, s))
        if best is None or key > best[0]:
            best = (key, s)
    if best is None:
        return None
    spot = best[1]
    d = cfg.contact_center_dist_m
    yaw0 = bearing(robot, spot)
    for k in range(cfg.release_spot_count):          # the direct line first, then fan out either side
        off = ((k + 1) // 2) * (2 * math.pi / cfg.release_spot_count) * (1 if k % 2 else -1) if k else 0.0
        yaw = wrap(yaw0 + off)
        pose = Pose2(spot[0] - d * math.cos(yaw), spot[1] - d * math.sin(yaw), yaw)
        if zone.contains(pose.xy, margin=0.05):
            return ReleasePlan(spot, pose)
    return None


def release_ready(zone: Zone, pose: Pose2, used: Iterable[XY], cfg: PlanningConfig) -> Tuple[bool, str]:
    """May the carried victim be put down HERE? True only if where it is (derived from the robot's pose, not from the
    robot merely being 'in the safe zone') lies inside the zone by the release margin, clear of victims already there."""
    v = expected_carried_xy(pose, cfg)
    depth = zone.depth(v)
    if depth < cfg.release_margin_m:
        return False, f'the carried victim would be {depth * 100:.0f} cm inside the zone edge (need {cfg.release_margin_m * 100:.0f})'
    for u in used:
        if dist(v, u) < cfg.release_spot_min_sep_m - 0.10:
            return False, f'the carried victim would land {dist(v, u) * 100:.0f} cm from one already released'
    return True, 'inside the zone'
