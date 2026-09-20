"""Tracked entities and the victim status lifecycle."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet

# Mirrors fire_resq_interfaces/msg/VictimState.msg (a unit test compares them), so this module needs no ROS.
UNKNOWN, DETECTED, TARGETED, CARRIED, RESCUED, UNREACHABLE = 0, 1, 2, 3, 4, 5
STATUS_NAMES = {UNKNOWN: 'UNKNOWN', DETECTED: 'DETECTED', TARGETED: 'TARGETED', CARRIED: 'CARRIED',
                RESCUED: 'RESCUED', UNREACHABLE: 'UNREACHABLE'}

# What the rescue side may request through UpdateVictimStatus. UNKNOWN is never requested: a track leaves it only by
# being observed again (confirmation). RESCUED has no exits - that is what makes the rescue loop terminate.
TRANSITIONS: Dict[int, FrozenSet[int]] = {
    UNKNOWN: frozenset(),
    DETECTED: frozenset({TARGETED, UNREACHABLE}),
    TARGETED: frozenset({DETECTED, CARRIED, UNREACHABLE}),       # DETECTED = the attempt was abandoned
    CARRIED: frozenset({RESCUED, DETECTED}),                     # DETECTED = dropped on the way
    RESCUED: frozenset(),
    UNREACHABLE: frozenset({DETECTED}),                          # an explicit retry; automatic recovery is a TODO
}
ACTIVE = frozenset({TARGETED, CARRIED})                          # at most one victim may be in these at a time
# Observations are still ABSORBED by these (so they cannot spawn duplicates) but must not move the track.
FROZEN = frozenset({CARRIED, RESCUED})


@dataclass
class VictimTrack:
    id: str
    x: float
    y: float
    z: float
    confidence: float            # the latest detection's, undecayed
    first_seen: float
    last_seen: float
    hits: int = 1
    status: int = UNKNOWN


@dataclass
class FireTrack:
    x: float
    y: float
    z: float
    confidence: float
    first_seen: float
    last_seen: float
    hits: int = 1
    confirmed: bool = False


@dataclass(frozen=True)
class Observation:
    """One PLACED detection, already in the world frame. Unplaced (`position_valid=false`) detections never get here."""
    class_id: str
    x: float
    y: float
    z: float
    confidence: float
