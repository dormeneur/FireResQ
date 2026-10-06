"""The FSM's vocabulary: what it is told (Observation, Event) and what it asks for (Effect). No ROS, no clock.

The FSM is a pure function of these: given the same sequence of Observations (each carrying its own time) it produces the
same sequence of Effects and the same transition log. The node turns ROS traffic into Observations and Effects into ROS
traffic; nothing else is allowed to touch mission state. That is what makes the mission testable without a simulator.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Optional, Tuple

from .geometry import Pose2, XY, Zone

# Mirrors fire_resq_interfaces/msg/VictimState.msg (a unit test compares them), so this module needs no ROS.
UNKNOWN, DETECTED, TARGETED, CARRIED, RESCUED, UNREACHABLE = 0, 1, 2, 3, 4, 5
STATUS_NAMES = {UNKNOWN: 'UNKNOWN', DETECTED: 'DETECTED', TARGETED: 'TARGETED', CARRIED: 'CARRIED',
                RESCUED: 'RESCUED', UNREACHABLE: 'UNREACHABLE'}


# ------------------------------------------------------------------------------------------------ observations
@dataclass(frozen=True)
class VictimBelief:
    """One victim as the WORLD MODEL believes it (ids are the world model's, e.g. 'V2'; never a scenario id)."""
    id: str
    x: float
    y: float
    confidence: float
    status: int
    last_seen: float           # seconds; the world model's clock (simulation time)

    @property
    def xy(self) -> XY:
        return (self.x, self.y)


@dataclass(frozen=True)
class World:
    victims: Tuple[VictimBelief, ...]
    zone: Optional[Zone]
    stamp: float               # when the world model published this belief (its clock == the FSM's)

    def victim(self, victim_id: str) -> Optional[VictimBelief]:
        return next((v for v in self.victims if v.id == victim_id), None)


@dataclass(frozen=True)
class Sighting:
    """One 2D victim detection from the perception stream, camera-agnostic: where in the image, no position needed."""
    u: float                   # bbox centre, pixels
    width: float               # bbox width, pixels
    stamp: float


@dataclass(frozen=True)
class Camera:
    fx: float
    cx: float
    hfov: float


@dataclass(frozen=True)
class MagnetReading:
    energized: bool
    attached: bool
    stamp: float


@dataclass(frozen=True)
class Event:
    """The answer to an earlier Effect that needed one. `kind` is one of the *_DONE constants below."""
    kind: str
    req: int                                   # the request id the FSM issued
    ok: bool = False
    detail: str = ''
    victim_id: str = ''                        # SELECT_DONE
    approach: Optional[Pose2] = None           # SELECT_DONE
    attached: Optional[bool] = None            # MAGNET_DONE: the state the driver reported
    nav_status: str = ''                       # NAV_DONE: 'succeeded' | 'failed' | 'canceled' | 'rejected'


SELECT_DONE, STATUS_DONE, MAGNET_DONE, NAV_DONE, COSTMAP_DONE = 'select_done', 'status_done', 'magnet_done', 'nav_done', 'costmap_done'


@dataclass(frozen=True)
class Observation:
    t: float                                   # seconds; monotonic; the node's (simulation) clock
    ready: bool = True                         # every dependency (TF, world state, Nav2, services) is up
    robot: Optional[Pose2] = None              # base_link in map; None = no localisation right now
    robot_stamp: float = 0.0
    world: Optional[World] = None
    sightings: Tuple[Sighting, ...] = ()       # 2D victim detections seen recently (the node keeps a short window)
    camera: Optional[Camera] = None
    magnet: Optional[MagnetReading] = None
    grid: Optional[object] = None              # coverage.GridMap once /map has arrived
    nav_active: bool = False
    events: Tuple[Event, ...] = ()


# ------------------------------------------------------------------------------------------------------ effects
@dataclass(frozen=True)
class Effect:
    pass


@dataclass(frozen=True)
class Drive(Effect):
    """Command the base directly. Only PERCEIVE, ALIGN and the carry tug ever do; the node repeats it every tick."""
    v: float
    w: float


@dataclass(frozen=True)
class Stop(Effect):
    """Zero velocity (published a few times: the simulated drive holds the last command indefinitely)."""


@dataclass(frozen=True)
class SendNav(Effect):
    goal: Pose2
    tag: int


@dataclass(frozen=True)
class CancelNav(Effect):
    pass


@dataclass(frozen=True)
class CallSelect(Effect):
    req: int


@dataclass(frozen=True)
class SetStatus(Effect):
    req: int
    victim_id: str
    status: int


@dataclass(frozen=True)
class ClearCostmaps(Effect):
    """Forget every obstacle mark within `radius` of the robot (both Nav2 costmaps). A victim the robot has just picked up, or
    just put down, is no longer an obstacle to plan around - it is in contact with the robot - but its marks are still in the
    costmaps, and a planner whose START lies inside an inflated obstacle finds no path at all."""
    req: int
    radius: float


@dataclass(frozen=True)
class SetMagnet(Effect):
    req: int
    attach: bool


class State(str, enum.Enum):
    INIT = 'INIT'
    PERCEIVE = 'PERCEIVE'
    UPDATE_WORLD = 'UPDATE_WORLD'
    SELECT_TARGET = 'SELECT_TARGET'
    SEARCH = 'SEARCH'
    NAVIGATE_TO_APPROACH = 'NAVIGATE_TO_APPROACH'
    ALIGN = 'ALIGN'
    ATTACH = 'ATTACH'
    VERIFY_CARRY = 'VERIFY_CARRY'
    NAVIGATE_TO_SAFE_ZONE = 'NAVIGATE_TO_SAFE_ZONE'
    RELEASE = 'RELEASE'
    VERIFY_RELEASE = 'VERIFY_RELEASE'
    REASSESS = 'REASSESS'
    ABANDON = 'ABANDON'                        # a target attempt failed: stand down safely, tell the world model, reselect
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    MISSION_ABORTED = 'MISSION_ABORTED'        # a fatal inconsistency, or the mission's time budget: motion is locked


TERMINAL = frozenset({State.MISSION_COMPLETE, State.MISSION_ABORTED})


@dataclass
class Transition:
    t: float
    frm: str
    to: str
    reason: str
    target: str = ''
