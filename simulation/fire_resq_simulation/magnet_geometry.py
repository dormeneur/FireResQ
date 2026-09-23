"""Pure-Python electromagnet contact geometry (Phase 9).

Answers exactly one physical question: given the robot's pose and a candidate victim's
position, is the victim close enough for the electromagnet to actually be touching it? Gazebo's
`DetachableJoint` plugin does NOT answer this itself - it has no distance or contact check at
all (measured, see docs/Implementation_Plan.md Phase 9): asking it to attach a victim two
metres away silently welds a rigid joint across that gap, no error, no warning. `sim_magnet_bridge.py`
uses this module to decide WHICH declared victim (if any) is a legitimate attach target before
it ever asks Gazebo to attach anything.

No ROS, no Gazebo import here - it is ordinary trigonometry, unit-tested standalone.
"""
from __future__ import annotations

import math
from typing import Tuple

XY = Tuple[float, float]

# The steel attachment ring's radius, bottom 55 mm of the victim model (models/victim/model.sdf).
# The ring is coaxial with the model origin and radially symmetric, so the magnet can make contact
# from ANY approach bearing without knowing the victim's yaw (the model comment's own claim) -
# unit-tested against the SDF so this cannot silently drift from the model.
VICTIM_RING_RADIUS_M = 0.056

# How much slack beyond exact flush contact still counts as "touching", to absorb the sub-cm noise
# in a live pose estimate. Deliberately small: a real electromagnet's effective range is a couple
# of millimetres, and a large tolerance would let the sim attach victims the real robot could not
# reach, silently making Phase 10's ALIGN look easier than it is on hardware.
CONTACT_TOLERANCE_M = 0.02


def magnet_contact_distance_m(magnet_x_m: float, magnet_length_m: float,
                               ring_radius_m: float = VICTIM_RING_RADIUS_M) -> float:
    """Centre-to-centre (base_link -> victim origin) distance at which the magnet's front face is
    exactly flush with the ring's surface. `magnet_x_m`/`magnet_length_m` come from
    parameters.xacro (fire_resq_description.geometry.read_properties) - never restated as a
    literal, so a robot or victim geometry change is picked up automatically."""
    return magnet_x_m + magnet_length_m / 2.0 + ring_radius_m


def magnet_face_world_xy(robot_xy: XY, robot_yaw: float, magnet_x_m: float, magnet_length_m: float) -> XY:
    """World (x, y) of the magnet's front face: base_link, offset forward along the robot's own
    heading by the magnet's mount distance plus half its length (magnet_link points along the
    robot's local +x; see fire_resq_description/urdf/magnet.xacro)."""
    reach = magnet_x_m + magnet_length_m / 2.0
    return (robot_xy[0] + reach * math.cos(robot_yaw), robot_xy[1] + reach * math.sin(robot_yaw))


def contact_gap_m(robot_xy: XY, robot_yaw: float, victim_xy: XY,
                   magnet_x_m: float, magnet_length_m: float,
                   ring_radius_m: float = VICTIM_RING_RADIUS_M) -> float:
    """Horizontal gap between the magnet's front face and the ring's surface, at whatever bearing
    the robot is actually facing (not just the victim's local +x). Negative means the face has
    passed the ring surface (overlapping/penetrating, still solid contact); positive is open air.
    A victim is a legitimate attach target iff this is <= CONTACT_TOLERANCE_M."""
    fx, fy = magnet_face_world_xy(robot_xy, robot_yaw, magnet_x_m, magnet_length_m)
    return math.hypot(fx - victim_xy[0], fy - victim_xy[1]) - ring_radius_m


def is_in_contact(robot_xy: XY, robot_yaw: float, victim_xy: XY,
                   magnet_x_m: float, magnet_length_m: float,
                   ring_radius_m: float = VICTIM_RING_RADIUS_M,
                   tolerance_m: float = CONTACT_TOLERANCE_M) -> bool:
    return contact_gap_m(robot_xy, robot_yaw, victim_xy, magnet_x_m, magnet_length_m, ring_radius_m) <= tolerance_m
