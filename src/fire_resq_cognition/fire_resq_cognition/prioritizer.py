"""Victim prioritizers: the interface, the MVP weighted utility, and the nearest-victim baseline.

Both implementations share ONE candidate-assessment step (status eligibility, then the planner's verdict on the way out
and the way back), so they differ only in how they RANK the candidates that are eligible. That is what makes the
comparison in PRD section 11 about the decision model and not about what each was allowed to consider.

Nothing here imports ROS, Gazebo, hardware, a camera or the planner: navigation arrives as a callable in PlanningContext.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional

from . import factors as F
from .config import WEIGHT_NAMES, PrioritizerConfig
from .types import (CANDIDATE_STATUSES, CARRIED, RESCUED, STATUS_NAMES, UNKNOWN, UNREACHABLE, XY, Decision, PathInfo,
                    PlanningContext, ScoredVictim, VictimView, WorldView)

_STATUS_EXCLUSION = {UNKNOWN: 'not yet confirmed (tentative track)', CARRIED: 'already being carried',
                     RESCUED: 'already rescued', UNREACHABLE: 'marked UNREACHABLE by the rescue side'}


@dataclass
class Assessment:
    """Everything the ranking needs about one victim, or the reason it is out."""
    victim: VictimView
    exclusion: str = ''
    path_status: str = ''                       # 'unreachable' | 'unknown' when the planner is why it is excluded
    approach: Optional[XY] = None
    straight_m: float = 0.0
    robot_path_m: float = 0.0
    return_path_m: float = 0.0
    d_fire_m: float = -1.0                     # -1 = the fire is not known

    @property
    def eligible(self) -> bool:
        return not self.exclusion

    @property
    def cost_m(self) -> float:
        return self.robot_path_m + self.return_path_m


class VictimPrioritizer(ABC):
    """Scores every victim in a WorldView and selects one, with the full explanation.

        score(world, ctx)   -> one ScoredVictim per victim (eligible or not, each with its factors and reason)
        select(world, ctx)  -> a Decision: the ranking, the selected victim (or None and why)

    Implementations differ only in `_rank`/`_score_one`; eligibility is shared, so no model can pick what another may not."""

    name = 'abstract'

    def __init__(self, config: Optional[PrioritizerConfig] = None):
        self.cfg = config or PrioritizerConfig()

    @property
    def weights(self) -> Dict[str, float]:
        return dict(self.cfg.weights)

    # ---------------------------------------------------------------- the interface
    def score(self, world: WorldView, ctx: PlanningContext) -> List[ScoredVictim]:
        return [self._score_one(a, world) for a in self._assess(world, ctx)]

    def select(self, world: WorldView, ctx: PlanningContext) -> Decision:
        scored = self.score(world, ctx)
        eligible = sorted((s for s in scored if s.eligible), key=self._rank)
        excluded = sorted((s for s in scored if not s.eligible), key=lambda s: s.victim_id)
        selected = eligible[0] if eligible else None
        reason = '' if selected else self._nothing_reason(world, excluded)
        return Decision(self.name, world.stamp, self.weights, eligible + excluded, selected, reason)

    @abstractmethod
    def _score_one(self, a: Assessment, world: WorldView) -> ScoredVictim: ...

    @abstractmethod
    def _rank(self, s: ScoredVictim):
        """Sort key: smaller sorts first = selected first. MUST end in the victim id so ties are deterministic."""

    # ---------------------------------------------------------------- shared eligibility
    def _assess(self, world: WorldView, ctx: PlanningContext) -> List[Assessment]:
        out = []
        for v in world.victims:
            a = Assessment(v)
            if world.fire_known:
                a.d_fire_m = F.distance((v.x, v.y), world.fire_xy)
            if v.status not in CANDIDATE_STATUSES:
                a.exclusion = _STATUS_EXCLUSION.get(v.status, f'status {STATUS_NAMES.get(v.status, v.status)} is not selectable')
                out.append(a)
                continue
            a.approach = F.approach_point(world.robot_xy, (v.x, v.y), self.cfg.approach_standoff_m)
            a.straight_m = F.distance(world.robot_xy, a.approach)      # same endpoints as the planned path, so the ratio is honest
            out_leg = self._ask(ctx, world.robot_xy, a.approach)
            if not out_leg.ok:
                a.exclusion, a.path_status = self._path_exclusion('to an approach point', out_leg), out_leg.status
                out.append(a)
                continue
            back_leg = self._ask(ctx, a.approach, world.safe_zone_xy)
            if not back_leg.ok:
                a.exclusion, a.path_status = self._path_exclusion('from the victim back to the safe zone', back_leg), back_leg.status
                out.append(a)
                continue
            a.robot_path_m, a.return_path_m = out_leg.length_m, back_leg.length_m
            out.append(a)
        return out

    @staticmethod
    def _ask(ctx: PlanningContext, start: XY, goal: XY) -> PathInfo:
        try:
            info = ctx.path_query(start, goal)
        except Exception as e:                                        # a broken planner must not become "reachable"
            return PathInfo.unknown(f'path query raised {type(e).__name__}: {e}')
        return info if isinstance(info, PathInfo) else PathInfo.unknown('path query returned no PathInfo')

    @staticmethod
    def _path_exclusion(what: str, info: PathInfo) -> str:
        if info.status == 'unreachable':
            return f'no path {what} ({info.reason or "planner found none"})'
        return f'could not ask the planner for a path {what} ({info.reason}); not assumed reachable'

    @staticmethod
    def _nothing_reason(world: WorldView, excluded: List[ScoredVictim]) -> str:
        if not world.victims:
            return 'no victims are known'
        return 'no eligible victim: ' + '; '.join(f'{s.victim_id}: {s.exclusion}' for s in excluded)

    # ---------------------------------------------------------------- shared reporting
    def _base_factors(self, a: Assessment) -> Dict[str, float]:
        f = {'confidence': a.victim.confidence, 'distance_to_fire_m': a.d_fire_m}
        f['fire_risk'] = F.fire_risk(a.d_fire_m, self.cfg) if a.d_fire_m >= 0 else 0.0
        f['fire_proximity'] = F.fire_proximity(a.d_fire_m, self.cfg) if a.d_fire_m >= 0 else 0.0
        if a.approach is not None and a.eligible:
            f.update(straight_distance_m=a.straight_m, robot_path_m=a.robot_path_m, return_path_m=a.return_path_m,
                     rescue_cost_m=a.cost_m, accessibility=F.accessibility(a.straight_m, a.robot_path_m))
        return f


class WeightedUtilityPrioritizer(VictimPrioritizer):
    """The MVP model (Implementation_Plan.md section 10). For each eligible victim:

        U =  w_risk           * fire_risk(d_fire)              in [0, 1]   danger to the victim
           + w_fire_proximity * (1 - norm(d_fire))             in [0, 1]   nearer the fire = more urgent
           + w_reach          * (1 - norm(robot -> victim))    in [0, 1]   cheaper trip out
           + w_accessibility  * straight / path                in (0, 1]   how direct the route is
           + w_confidence     * perception confidence          in [0, 1]
           - w_cost           * norm(robot -> victim -> safe)  in [0, 1]   the whole rescue

    Transparent by construction: every term is a documented function of measured numbers, every weight is a parameter,
    and the contributions in the explanation sum exactly to the utility."""

    name = 'weighted_utility'

    def _score_one(self, a: Assessment, world: WorldView) -> ScoredVictim:
        v = a.victim
        f = self._base_factors(a)
        if not a.eligible:
            return ScoredVictim(v.id, (v.x, v.y), False, None, a.exclusion, a.path_status, f, {}, None, f'{v.id} excluded: {a.exclusion}')
        w = self.cfg.weights
        f['robot_proximity'] = F.robot_proximity(a.robot_path_m, self.cfg)
        f['rescue_cost_norm'] = F.rescue_cost_norm(a.cost_m, self.cfg)
        c = {'risk': w['risk'] * f['fire_risk'],
             'fire_proximity': w['fire_proximity'] * f['fire_proximity'],
             'reach': w['reach'] * f['robot_proximity'],
             'accessibility': w['accessibility'] * f['accessibility'],
             'confidence': w['confidence'] * f['confidence'],
             'cost': -w['cost'] * f['rescue_cost_norm']}
        u = sum(c[k] for k in WEIGHT_NAMES)
        return ScoredVictim(v.id, (v.x, v.y), True, u, '', '', f, c, a.approach, self._rationale(v.id, u, f, c, world))

    def _rank(self, s: ScoredVictim):
        return (-s.utility, s.factors['rescue_cost_m'], s.victim_id)

    @staticmethod
    def _rationale(vid: str, u: float, f: Dict[str, float], c: Dict[str, float], world: WorldView) -> str:
        fire = (f"{f['distance_to_fire_m']:.2f} m from the fire (risk {f['fire_risk']:.2f})" if world.fire_known
                else 'fire position unknown (fire terms are 0)')
        return (f"{vid}: U={u:+.3f} = risk {c['risk']:+.2f} + fire-proximity {c['fire_proximity']:+.2f} + reach {c['reach']:+.2f}"
                f" + accessibility {c['accessibility']:+.2f} + confidence {c['confidence']:+.2f} + cost {c['cost']:+.2f}"
                f" | {fire}; trip out {f['robot_path_m']:.2f} m, rescue {f['rescue_cost_m']:.2f} m, "
                f"directness {f['accessibility']:.2f}, confidence {f['confidence']:.2f}")


class NearestVictimPrioritizer(VictimPrioritizer):
    """The evaluation baseline (PRD section 11): rescue the victim with the shortest trip out.

    "Nearest" is by the planner's path length from the robot, which is what nearest-first costs a real robot, and it
    shares the weighted model's eligibility rules (status, reachable out and back) so the comparison isolates the ranking.
    The fire, confidence, directness and return leg are REPORTED (so the two models' tables are comparable) but ignored."""

    name = 'nearest'

    @property
    def weights(self) -> Dict[str, float]:
        return {k: (1.0 if k == 'reach' else 0.0) for k in WEIGHT_NAMES}

    def _score_one(self, a: Assessment, world: WorldView) -> ScoredVictim:
        v = a.victim
        f = self._base_factors(a)
        if not a.eligible:
            return ScoredVictim(v.id, (v.x, v.y), False, None, a.exclusion, a.path_status, f, {}, None, f'{v.id} excluded: {a.exclusion}')
        u = -a.robot_path_m                                           # utility in metres, negated: nearer scores higher
        c = {k: 0.0 for k in WEIGHT_NAMES}
        c['reach'] = u
        return ScoredVictim(v.id, (v.x, v.y), True, u, '', '', f, c, a.approach,
                            f'{v.id}: trip out {a.robot_path_m:.2f} m (nearest-first ignores fire, confidence and the return leg)')

    def _rank(self, s: ScoredVictim):
        return (s.factors['robot_path_m'], s.victim_id)


_MODELS = {WeightedUtilityPrioritizer.name: WeightedUtilityPrioritizer, NearestVictimPrioritizer.name: NearestVictimPrioritizer}


def make_prioritizer(model: str, config: Optional[PrioritizerConfig] = None) -> VictimPrioritizer:
    """Build the decision model named by the `decision_model` parameter."""
    if model not in _MODELS:
        raise ValueError(f'unknown decision_model {model!r}; choose one of {sorted(_MODELS)}')
    return _MODELS[model](config)
