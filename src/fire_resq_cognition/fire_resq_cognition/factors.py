"""The individual factors of the utility. Pure functions of numbers: each is unit-tested on its own."""
from __future__ import annotations

import math
from typing import Tuple

from .config import PrioritizerConfig

XY = Tuple[float, float]


def distance(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm(d: float, extent_m: float) -> float:
    """Length -> [0, 1] by the configured extent. Clamped: a longer path is 'as bad as it gets', not a negative score."""
    return max(0.0, min(1.0, d / extent_m))


def fire_risk(d_fire_m: float, cfg: PrioritizerConfig) -> float:
    """Danger to a victim at `d_fire_m` from the fire, in [0, 1]: 1 inside the hazard radius, exponentially less beyond.

    TODO(future): dynamic fire-risk modelling. This is a STATIC function of distance today: it does not know the fire
    is spreading, that some routes are blocked by it, or how long the victim has been exposed."""
    if d_fire_m <= cfg.fire_hazard_radius_m:
        return 1.0
    return math.exp(-(d_fire_m - cfg.fire_hazard_radius_m) / cfg.fire_risk_decay_m)


def fire_proximity(d_fire_m: float, cfg: PrioritizerConfig) -> float:
    """How close to the fire, linearly over the arena extent: 1 at the fire, 0 at one extent away. Unlike fire_risk it
    keeps discriminating between two victims that are both far from the fire."""
    return 1.0 - norm(d_fire_m, cfg.extent_m)


def robot_proximity(d_robot_m: float, cfg: PrioritizerConfig) -> float:
    """How cheap the trip out is: 1 at the robot, 0 an extent away."""
    return 1.0 - norm(d_robot_m, cfg.extent_m)


def accessibility(straight_m: float, path_m: float) -> float:
    """How direct the route is: straight-line / planned length, in (0, 1]. 1 = a straight run; smaller = a detour round
    obstacles. (Unreachable candidates never get here: they are excluded, not scored 0.)"""
    if path_m <= 1e-9:
        return 1.0
    return max(0.0, min(1.0, straight_m / path_m))


def rescue_cost_norm(cost_m: float, cfg: PrioritizerConfig) -> float:
    """Total trip (robot -> victim -> safe zone) normalised by TWO extents, since it is two legs."""
    return norm(cost_m, 2.0 * cfg.extent_m)


def approach_point(robot: XY, victim: XY, standoff_m: float) -> XY:
    """The point the planner is asked about: `standoff_m` short of the victim, on the robot->victim line.

    Not the victim itself: a victim is a solid object, and Nav2 rejects goals inside its inflation (Phase 4 finding).
    If the robot is already within the standoff, it is the approach point. The executable approach pose (yaw, magnet
    standoff) is the rescue FSM's to compute in Phase 10 the same way; cognition only needs a plausible goal to query."""
    d = distance(robot, victim)
    if d <= standoff_m or d <= 1e-9:
        return robot
    f = (d - standoff_m) / d
    return (robot[0] + (victim[0] - robot[0]) * f, robot[1] + (victim[1] - robot[1]) * f)


def approach_pose(robot: XY, victim: XY, standoff_m: float) -> Tuple[float, float, float]:
    """The approach point of `approach_point` plus the heading that faces the victim: (x, y, yaw), in the world frame.

    The contract `RescueTarget.approach_pose` carries (see docs/Implementation_Plan.md, Phase 7 "approach pose contract"): a point `standoff_m` from the
    victim's CENTRE on the robot->victim line, facing it. Nav2 can plan to it (it is outside the costmap's inflation of the victim);
    the last stretch to the magnet (ALIGN, Phase 10) is open-loop and not Nav2's."""
    x, y = approach_point(robot, victim, standoff_m)
    d = distance((x, y), victim)
    yaw = math.atan2(victim[1] - y, victim[0] - x) if d > 1e-6 else math.atan2(victim[1] - robot[1], victim[0] - robot[0])
    return x, y, yaw

