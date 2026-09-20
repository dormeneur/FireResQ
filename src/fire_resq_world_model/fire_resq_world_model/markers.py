"""What to draw for a world snapshot, as plain descriptions (no ROS), so the colours and labels are unit-testable.

This is how the system gets demonstrated: victims coloured by status with their id and confidence, the fire, the safe
zone. The node turns each spec into a visualization_msgs/Marker.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from .entities import CARRIED, DETECTED, RESCUED, STATUS_NAMES, TARGETED, UNKNOWN, UNREACHABLE
from .world_model import WorldSnapshot

STATUS_RGB = {UNKNOWN: (0.6, 0.6, 0.6), DETECTED: (0.15, 0.45, 1.0), TARGETED: (1.0, 0.85, 0.0),
              CARRIED: (0.85, 0.2, 0.9), RESCUED: (0.1, 0.8, 0.2), UNREACHABLE: (0.9, 0.1, 0.1)}
FIRE_RGB = (1.0, 0.4, 0.0)
ZONE_RGB = (0.1, 0.8, 0.2)


@dataclass(frozen=True)
class MarkerSpec:
    ns: str
    id: int
    shape: str                       # 'sphere' | 'cylinder' | 'cube' | 'text'
    x: float
    y: float
    z: float
    scale: Tuple[float, float, float]
    rgba: Tuple[float, float, float, float]
    text: str = ''
    yaw: float = 0.0


def marker_specs(snap: WorldSnapshot) -> List[MarkerSpec]:
    out: List[MarkerSpec] = []
    z = snap.safe_zone
    out.append(MarkerSpec('safe_zone', 0, 'cube', z.x, z.y, 0.005, (z.size_x, z.size_y, 0.01), (*ZONE_RGB, 0.35), yaw=z.yaw))
    if snap.fire.known:
        f = snap.fire
        out.append(MarkerSpec('fire', 0, 'cylinder', f.x, f.y, 0.2, (0.26, 0.26, 0.4), (*FIRE_RGB, max(0.35, f.confidence))))
        out.append(MarkerSpec('fire_label', 0, 'text', f.x, f.y, 0.6, (0.0, 0.0, 0.12), (1.0, 1.0, 1.0, 1.0),
                              text=f'fire {f.confidence:.2f}'))
    for i, v in enumerate(snap.victims):
        rgb = STATUS_RGB[v.status]
        out.append(MarkerSpec('victim', i, 'sphere', v.x, v.y, 0.15, (0.18, 0.18, 0.3), (*rgb, max(0.3, min(1.0, v.confidence)))))
        out.append(MarkerSpec('victim_label', i, 'text', v.x, v.y, 0.5, (0.0, 0.0, 0.12), (1.0, 1.0, 1.0, 1.0),
                              text=f'{v.id} {STATUS_NAMES[v.status]} {v.confidence:.2f}'))
    return out
