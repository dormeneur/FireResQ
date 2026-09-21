"""Prioritizer parameters. One validated dataclass, so a bad value fails at start-up, not mid-mission.

The weights are the policy: they say how much a metre of travel is worth against a metre closer to the fire. They are
parameters with documented defaults - no weight is a literal in the scoring code - and the defaults are a stated stance
(safety first), not a fitted result. See docs/Implementation_Plan.md for the decision this is.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Dict

WEIGHT_NAMES = ('risk', 'fire_proximity', 'reach', 'accessibility', 'confidence', 'cost')

# Priority tiers, not fitted numbers: fire risk is the safety-critical criterion (tier 1); how near the fire, how near the
# robot and how expensive the rescue are practical criteria (tier 2); how direct the route is and how sure we are that
# the victim exists are tie-breaking criteria (tier 3). Every term is normalised to [0, 1] before weighting.
DEFAULT_WEIGHTS = {'risk': 3.0, 'fire_proximity': 1.0, 'reach': 1.0, 'accessibility': 0.5, 'confidence': 0.5, 'cost': 1.0}


@dataclass(frozen=True)
class PrioritizerConfig:
    weights: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_WEIGHTS))
    extent_m: float = 7.07                 # normalisation length: the arena diagonal (a configured extent, not a coordinate)
    fire_hazard_radius_m: float = 1.0      # within this of the fire a victim is in immediate danger (risk = 1)
    fire_risk_decay_m: float = 1.0         # beyond it, risk falls off exponentially with this length scale
    approach_standoff_m: float = 0.4       # the path is queried to a point this far short of the victim (see approach_point)

    def __post_init__(self):
        unknown = set(self.weights) - set(WEIGHT_NAMES)
        missing = set(WEIGHT_NAMES) - set(self.weights)
        if unknown or missing:
            raise ValueError(f'weights must be exactly {WEIGHT_NAMES}; unknown {sorted(unknown)}, missing {sorted(missing)}')
        if any(v < 0 for v in self.weights.values()):
            raise ValueError('weights must be non-negative (the sign of each term is fixed by its meaning)')
        if not any(v > 0 for v in self.weights.values()):
            raise ValueError('at least one weight must be positive')
        for name in ('extent_m', 'fire_hazard_radius_m', 'fire_risk_decay_m'):
            if not getattr(self, name) > 0:
                raise ValueError(f'{name} must be positive')
        if self.approach_standoff_m < 0:
            raise ValueError('approach_standoff_m must be >= 0')

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> 'PrioritizerConfig':
        """Flat parameters: `weights.risk` ... as dotted keys (ROS parameter style) or a nested `weights` dict.
        Unknown keys are an error, so a typo cannot silently leave a default in place."""
        scalar = {f.name for f in fields(cls)} - {'weights'}
        weights = dict(DEFAULT_WEIGHTS)
        rest: Dict[str, float] = {}
        bad = []
        for k, v in params.items():
            if k == 'weights' and isinstance(v, dict):
                for kk, vv in v.items():
                    (weights.__setitem__(kk, float(vv)) if kk in WEIGHT_NAMES else bad.append(f'weights.{kk}'))
            elif k.startswith('weights.'):
                (weights.__setitem__(k[8:], float(v)) if k[8:] in WEIGHT_NAMES else bad.append(k))
            elif k in scalar:
                rest[k] = float(v)
            else:
                bad.append(k)
        if bad:
            raise ValueError(f'unknown prioritizer parameters: {sorted(bad)}')
        return cls(weights=weights, **rest)
