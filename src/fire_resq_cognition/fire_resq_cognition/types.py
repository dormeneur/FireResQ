"""Plain-data inputs and outputs of the decision layer. No ROS, no Gazebo, no hardware.

The node converts fire_resq_interfaces messages into these; the prioritizers see only these. That is what keeps the
decision models replaceable and testable without a ROS graph.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

XY = Tuple[float, float]

# Mirrors fire_resq_interfaces/msg/VictimState.msg (a unit test compares them), so this package needs no ROS to read it.
UNKNOWN, DETECTED, TARGETED, CARRIED, RESCUED, UNREACHABLE = 0, 1, 2, 3, 4, 5
STATUS_NAMES = {UNKNOWN: 'UNKNOWN', DETECTED: 'DETECTED', TARGETED: 'TARGETED', CARRIED: 'CARRIED',
                RESCUED: 'RESCUED', UNREACHABLE: 'UNREACHABLE'}
# Statuses a victim may be selected in. TARGETED is the current target: it stays a candidate so a re-decision can either
# confirm it or (deliberately) change it. Everything else has a stated reason for exclusion (see prioritizer.py).
CANDIDATE_STATUSES = frozenset({DETECTED, TARGETED})


@dataclass(frozen=True)
class VictimView:
    id: str
    x: float
    y: float
    confidence: float               # already decayed by the world model
    status: int


@dataclass(frozen=True)
class WorldView:
    """The slice of the world model cognition reasons over, in the world frame (`map`)."""
    stamp: float
    victims: Tuple[VictimView, ...]
    fire_known: bool
    fire_xy: XY
    safe_zone_xy: XY
    robot_xy: XY
    robot_yaw: float = 0.0


@dataclass(frozen=True)
class PathInfo:
    """The answer to one path query. `reachable`/`unreachable` are the planner's verdict; `unknown` means it could not
    be asked (server down, timeout) - which is NOT the same as unreachable and is never treated as reachable."""
    status: str                     # 'reachable' | 'unreachable' | 'unknown'
    length_m: Optional[float] = None
    reason: str = ''

    @staticmethod
    def reachable(length_m: float) -> 'PathInfo':
        return PathInfo('reachable', float(length_m))

    @staticmethod
    def unreachable(reason: str = 'no path') -> 'PathInfo':
        return PathInfo('unreachable', None, reason)

    @staticmethod
    def unknown(reason: str) -> 'PathInfo':
        return PathInfo('unknown', None, reason)

    @property
    def ok(self) -> bool:
        return self.status == 'reachable' and self.length_m is not None and math.isfinite(self.length_m)


PathQuery = Callable[[XY, XY], PathInfo]


@dataclass(frozen=True)
class PlanningContext:
    """What a prioritizer may ask of the outside world. The path query hides Nav2: implementations get navigation
    information without importing it, and unit tests hand in a fake."""
    path_query: PathQuery


@dataclass
class ScoredVictim:
    victim_id: str
    position: XY
    eligible: bool
    utility: Optional[float]                       # None when not eligible
    exclusion: str = ''                            # why not eligible ('' when eligible)
    path_status: str = ''                          # the planner's verdict behind an exclusion: 'unreachable' | 'unknown' | ''
    factors: Dict[str, float] = field(default_factory=dict)         # the raw, unweighted numbers (metres, [0,1] scores)
    contributions: Dict[str, float] = field(default_factory=dict)   # weight x normalised factor, signed; sums to utility
    approach_xy: Optional[XY] = None
    rationale: str = ''


@dataclass
class Decision:
    model: str
    stamp: float
    weights: Dict[str, float]
    ranked: List[ScoredVictim]                     # eligible first, best first; then the excluded, each with its reason
    selected: Optional[ScoredVictim]
    reason: str = ''                               # why nothing was selected ('' when something was)

    def to_dict(self) -> dict:
        def one(s: ScoredVictim) -> dict:
            return {'victim_id': s.victim_id, 'position': list(s.position), 'eligible': s.eligible, 'utility': s.utility,
                    'exclusion': s.exclusion, 'path_status': s.path_status, 'factors': s.factors, 'contributions': s.contributions,
                    'approach_xy': list(s.approach_xy) if s.approach_xy else None, 'rationale': s.rationale}
        return {'model': self.model, 'stamp': self.stamp, 'weights': self.weights,
                'selected': self.selected.victim_id if self.selected else None, 'reason': self.reason,
                'candidates': [one(s) for s in self.ranked]}
