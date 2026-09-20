"""World-model parameters. One validated dataclass, so a bad value fails at start-up, not mid-mission."""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from typing import Any, Dict


@dataclass(frozen=True)
class SafeZone:
    """The safe zone in the WORLD frame (normally `map`). Known infrastructure, configured, never discovered
    (Implementation_Plan.md section 9). `yaw` rotates the rectangle."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    size_x: float = 1.0
    size_y: float = 1.0

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx, ly = dx * c - dy * s, dx * s + dy * c
        return abs(lx) <= self.size_x / 2 + margin and abs(ly) <= self.size_y / 2 + margin


@dataclass(frozen=True)
class WorldModelConfig:
    victim_gate_m: float = 0.3            # association radius; the scenario requires victims further apart than this
    fire_gate_m: float = 0.5              # a fire detection further than this from the tracked fire is ignored (one fire)
    position_alpha: float = 0.3           # EMA weight of a new observation (1 = no smoothing)
    min_confidence: float = 0.2           # = perception's own floor: a far victim's pixel-count confidence is legitimately low
    confirm_hits: int = 2                 # observations (distinct messages) before a track leaves UNKNOWN
    tentative_timeout_s: float = 5.0      # an UNKNOWN track never confirmed within this is dropped (a ghost)
    confidence_decay_tau_s: float = 30.0  # published confidence = latest * exp(-time since last seen / tau)
    safe_zone: SafeZone = field(default_factory=SafeZone)

    def __post_init__(self):
        if not (self.victim_gate_m > 0 and self.fire_gate_m > 0):
            raise ValueError('gates must be positive')
        if not 0 < self.position_alpha <= 1:
            raise ValueError('position_alpha must be in (0, 1]')
        if not 0 <= self.min_confidence <= 1:
            raise ValueError('min_confidence must be in [0, 1]')
        if self.confirm_hits < 1:
            raise ValueError('confirm_hits must be >= 1')
        if self.tentative_timeout_s <= 0 or self.confidence_decay_tau_s <= 0:
            raise ValueError('timeouts must be positive')
        if self.safe_zone.size_x <= 0 or self.safe_zone.size_y <= 0:
            raise ValueError('safe zone must have positive size')

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> 'WorldModelConfig':
        """Flat parameters (`safe_zone.x`, ... as dotted keys or a nested dict); unknown keys are an error."""
        known = {f.name for f in fields(cls)} - {'safe_zone'}
        zone_names = {f.name for f in fields(SafeZone)}
        flat: Dict[str, Any] = {}
        for k, v in params.items():
            if k == 'safe_zone' and isinstance(v, dict):
                flat.update({f'safe_zone.{kk}': vv for kk, vv in v.items()})
            else:
                flat[k] = v
        zone = {k.split('.', 1)[1]: float(v) for k, v in flat.items() if k.startswith('safe_zone.')}
        bad = [k for k in flat if not k.startswith('safe_zone.') and k not in known]
        bad += [f'safe_zone.{k}' for k in zone if k not in zone_names]
        if bad:
            raise ValueError(f'unknown world-model parameters: {sorted(bad)}')
        rest = {k: (int(v) if k == 'confirm_hits' else float(v)) for k, v in flat.items() if k in known}
        return cls(safe_zone=SafeZone(**zone), **rest)
