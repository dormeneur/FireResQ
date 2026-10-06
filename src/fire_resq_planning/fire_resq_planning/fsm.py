"""The rescue mission as an explicit finite-state machine.

    INIT -> PERCEIVE -> UPDATE_WORLD -> SELECT_TARGET -+-> NAVIGATE_TO_APPROACH -> ALIGN -> ATTACH -> VERIFY_CARRY
                            ^   ^                       |         -> NAVIGATE_TO_SAFE_ZONE -> RELEASE -> VERIFY_RELEASE
                            |   +---- REASSESS <--------+---------------------------------------------------+
                            |                           |                       (any failed attempt: ABANDON -> REASSESS)
                            +--- PERCEIVE <--- SEARCH <-+ no target now             MISSION_COMPLETE / MISSION_ABORTED

What this module is, and is not:
  * PURE and DETERMINISTIC. `RescueFSM.step(obs)` takes an Observation (which carries its own time) and returns Effects; the
    same sequence of Observations always yields the same Effects and the same transition log. It reads no clock, no ROS
    topic, no file, no scenario and no ground truth, and it commands nothing itself: fire_resq_planning.rescue_node turns
    ROS traffic into Observations and Effects into ROS traffic.
  * It does NOT rank victims. SELECT_TARGET asks cognition (`SelectTarget`) and takes the answer; the FSM only owns the
    procedure for carrying one victim to the safe zone and the decision of when a target must be given up.
  * Every transition is declared in ALLOWED and logged with its reason; an undeclared one is a programming error.
  * "Requested" is never "done". Each physical claim waits for an independent reading: a magnet request for the driver's
    hold sensor (MagnetState), a navigation goal for Nav2's result AND the release geometry, a victim's rescue for the
    magnet reporting it released AND the victim's expected position inside the zone. Only then does the world model hear it.

Failure handling is uniform: any failed attempt on a target goes through ABANDON (stop, cancel navigation, de-energise the
magnet, put the victim back to DETECTED in the world model, count the attempt; a second failure gives it up as UNREACHABLE),
then REASSESS re-runs cognition. A fatal inconsistency (the magnet contradicting the FSM, a victim that will not release,
the mission time budget) goes to MISSION_ABORTED, which locks motion for good.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple

from .config import PlanningConfig
from .coverage import CoveragePlanner, GridMap
from .geometry import (Pose2, Zone, approach_pose, bearing, contact_gap, dist, expected_carried_xy, magnet_face, plan_release,
                       release_ready, wrap)
from .types import (CARRIED, COSTMAP_DONE, DETECTED, MAGNET_DONE, NAV_DONE, RESCUED, SELECT_DONE, STATUS_DONE, STATUS_NAMES, TARGETED, TERMINAL,
                    UNKNOWN, UNREACHABLE, CallSelect, CancelNav, ClearCostmaps, Drive, Effect, Event, Observation, SendNav,
                    SetMagnet, SetStatus, State, Stop, Transition, VictimBelief)

S = State
ALLOWED: Dict[State, FrozenSet[State]] = {
    S.INIT: frozenset({S.PERCEIVE, S.MISSION_ABORTED}),
    S.PERCEIVE: frozenset({S.UPDATE_WORLD, S.MISSION_ABORTED}),
    S.UPDATE_WORLD: frozenset({S.SELECT_TARGET, S.MISSION_ABORTED}),
    S.SELECT_TARGET: frozenset({S.NAVIGATE_TO_APPROACH, S.SEARCH, S.MISSION_ABORTED}),
    S.SEARCH: frozenset({S.PERCEIVE, S.SELECT_TARGET, S.MISSION_COMPLETE, S.MISSION_ABORTED}),
    S.NAVIGATE_TO_APPROACH: frozenset({S.ALIGN, S.ABANDON, S.MISSION_ABORTED}),
    S.ALIGN: frozenset({S.ATTACH, S.ABANDON, S.MISSION_ABORTED}),
    S.ATTACH: frozenset({S.VERIFY_CARRY, S.ABANDON, S.MISSION_ABORTED}),
    S.VERIFY_CARRY: frozenset({S.NAVIGATE_TO_SAFE_ZONE, S.ABANDON, S.MISSION_ABORTED}),
    S.NAVIGATE_TO_SAFE_ZONE: frozenset({S.RELEASE, S.ABANDON, S.MISSION_ABORTED}),
    S.RELEASE: frozenset({S.VERIFY_RELEASE, S.MISSION_ABORTED}),
    S.VERIFY_RELEASE: frozenset({S.REASSESS, S.MISSION_ABORTED}),
    S.REASSESS: frozenset({S.UPDATE_WORLD, S.MISSION_ABORTED}),
    S.ABANDON: frozenset({S.REASSESS, S.MISSION_ABORTED}),
    S.MISSION_COMPLETE: frozenset(),
    S.MISSION_ABORTED: frozenset(),
}

# States in which NO victim may be held. The magnet reporting one is a contradiction, not a surprise to be absorbed.
NOT_CARRYING = frozenset({S.PERCEIVE, S.UPDATE_WORLD, S.SELECT_TARGET, S.SEARCH, S.NAVIGATE_TO_APPROACH, S.ALIGN, S.REASSESS})
# States in which the base is driven by the FSM itself (everywhere else Nav2 drives, or nothing does).
DRIVING = frozenset({S.PERCEIVE, S.ALIGN, S.ATTACH, S.VERIFY_CARRY})


class IllegalTransition(RuntimeError):
    pass


@dataclass
class _Mission:
    """Everything that outlives a state."""
    target: str = ''
    target_xy: Tuple[float, float] = (0.0, 0.0)
    target_conf: float = 0.0                          # its confidence when cognition chose it
    approach: Optional[Pose2] = None
    attempts: Dict[str, int] = field(default_factory=dict)
    rescued: List[str] = field(default_factory=list)
    given_up: List[str] = field(default_factory=list)
    used_spots: List[Tuple[float, float]] = field(default_factory=list)
    release_plan: Optional[object] = None
    released_xy: Optional[Tuple[float, float]] = None
    abandon: Optional[Tuple[str, bool]] = None       # (reason, counts as a failed attempt on the target?)
    no_target_streak: int = 0
    select_timeouts: int = 0
    status_refusals: int = 0
    exhausted_retries: int = 0
    viewpoint: Optional[int] = None
    viewpoint_failures: int = 0
    targeted_t: float = -1e9                          # when the world model confirmed TARGETED (its belief may lag a status change)


class RescueFSM:
    def __init__(self, cfg: PlanningConfig):
        self.cfg = cfg
        self.state = S.INIT
        self.history: List[Transition] = []
        self.m = _Mission()
        self.d: Dict[str, object] = {}                # the current state's data; cleared on every entry
        self.planner: Optional[CoveragePlanner] = None
        self.metrics: Dict[str, float] = {k: 0 for k in (
            'select_calls', 'nav_goals', 'nav_failures', 'status_calls', 'magnet_requests', 'attach_refusals',
            'align_attempts', 'abandons', 'transitions', 'costmap_clears')}
        self.reason = ''
        self.t0: Optional[float] = None
        self._entered = 0.0
        self._req = itertools.count(1)
        self._tag = itertools.count(1)
        self._now = 0.0
        self._fx: List[Effect] = []
        self._driving = False

    # ============================================================================================ public
    def step(self, obs: Observation) -> List[Effect]:
        """Advance on one Observation; returns the Effects to execute. Never blocks, never reads anything else."""
        self._now = obs.t
        self._fx = []
        if self.t0 is None:
            self.t0, self._entered = obs.t, obs.t
        if self.state in TERMINAL:
            return self._finish_motion([])
        if obs.t - self.t0 > self.cfg.mission_timeout_s:
            self._abort(f'the mission time budget ({self.cfg.mission_timeout_s:.0f} s) ran out')
            return self._finish_motion(self._fx)
        if obs.magnet is not None and obs.magnet.attached and self.state in NOT_CARRYING:
            self._abort(f'the magnet reports a victim held in {self.state.value}, where none can be')
            return self._finish_motion(self._fx)
        getattr(self, '_tick_' + self.state.value.lower())(obs)
        return self._finish_motion(self._fx)

    def snapshot(self, now: float) -> Dict[str, object]:
        return {
            'state': self.state.value, 'since_s': round(now - self._entered, 2), 'reason': self.reason,
            'target': self.m.target, 'rescued': list(self.m.rescued), 'given_up': list(self.m.given_up),
            'attempts': dict(self.m.attempts), 'used_spots': [list(map(lambda v: round(v, 3), s)) for s in self.m.used_spots],
            'coverage': None if self.planner is None else round(self.planner.coverage(), 3),
            'metrics': dict(self.metrics), 'elapsed_s': round(now - (self.t0 or now), 2),
            'history': [[round(t.t, 2), t.frm, t.to, t.reason, t.target] for t in self.history],
        }

    # ============================================================================================ machinery
    def _req_id(self) -> int:
        return next(self._req)

    def _emit(self, *fx: Effect) -> None:
        self._fx.extend(fx)

    def _drive(self, v: float, w: float) -> None:
        """The only way the FSM moves the base. Never backwards (the robot is blind behind itself), never past its limits."""
        v = min(max(v, 0.0), self.cfg.max_linear_mps)
        w = min(max(w, -self.cfg.max_angular_rps), self.cfg.max_angular_rps)
        self._driving = True
        self._emit(Drive(v, w))

    def _finish_motion(self, fx: List[Effect]) -> List[Effect]:
        """A driving state that issued no Drive this tick leaves the base at rest: emit Stop when it stops driving."""
        out = list(fx)
        drove = any(isinstance(e, Drive) for e in out)
        if getattr(self, '_was_driving', False) and not drove and not any(isinstance(e, Stop) for e in out):
            out.append(Stop())
        self._was_driving = drove
        return out

    def _go(self, to: State, reason: str) -> None:
        if to not in ALLOWED[self.state]:
            raise IllegalTransition(f'{self.state.value} -> {to.value} is not a declared transition ({reason})')
        self.history.append(Transition(self._now, self.state.value, to.value, reason, self.m.target))
        self.metrics['transitions'] += 1
        self.state, self.reason, self._entered, self.d = to, reason, self._now, {}

    def _abort(self, reason: str) -> None:
        self._emit(Stop(), CancelNav())
        if self.state != S.MISSION_ABORTED:
            # A fatal state is reachable from anywhere (a declared edge for every state, see ALLOWED).
            self.history.append(Transition(self._now, self.state.value, S.MISSION_ABORTED.value, reason, self.m.target))
            self.metrics['transitions'] += 1
            self.state, self.reason, self._entered, self.d = S.MISSION_ABORTED, reason, self._now, {}

    def _depart_done(self, obs: Observation) -> bool:
        """Leave the victim's spot. The saved map has the victim in it, so a robot touching where it lies starts inside that
        obstacle's inflation and Nav2's planner finds no path from there. Turn round and drive forward - never reverse - the way
        the robot came in, until the planner's start is clear of it. Bounded in time and distance; VERIFY_CARRY checks the hold
        on every tick of it."""
        d, cfg = self.d, self.cfg
        if d.get('depart') == 'done':
            return True
        if 'depart' not in d:
            d.update(depart='turn', yaw_away=wrap(d['yaw0'] + math.pi), depart_t=obs.t)
        if obs.t - d['depart_t'] > cfg.depart_timeout_s:
            self._emit(Stop())
            if self.state == S.ABANDON:                             # already standing down: go on, cognition will see what is reachable
                d['depart'] = 'done'
                return True
            self._abandon('could not move clear of the pick-up spot in time', counts=True)
            return False
        if d['depart'] == 'turn':
            err = wrap(d['yaw_away'] - obs.robot.yaw)
            if abs(err) <= 0.05:
                d.update(depart='drive', depart_from=obs.robot.xy)
                self._emit(Stop())
                return False
            self._drive(0.0, math.copysign(min(cfg.tug_turn_rps, max(0.12, 2.0 * abs(err))), err))
            return False
        if dist(obs.robot.xy, d['depart_from']) >= cfg.depart_distance_m:
            d['depart'] = 'done'
            self._emit(Stop())
            return True
        self._drive(cfg.align_speed_mps, 0.0)
        return False

    def _clear_done(self, obs: Observation) -> bool:
        """Ask Nav2 to forget the obstacle marks around the robot and wait for it (bounded). True once the cells have had time
        to reach the planner. A failed clear is not fatal: the next planner query will say so."""
        d, cfg = self.d, self.cfg
        if 'clr' not in d:
            d['clr'], d['clr_t'] = self._req_id(), obs.t
            self.metrics['costmap_clears'] += 1
            self._emit(ClearCostmaps(d['clr'], cfg.costmap_clear_radius_m))
            return False
        ev = self._event(obs, COSTMAP_DONE, d['clr'])
        if ev is not None and 'clr_ok' not in d:
            d['clr_ok'] = obs.t
        settled = 'clr_ok' in d and obs.t - d['clr_ok'] >= cfg.costmap_settle_s
        return settled or obs.t - d['clr_t'] > cfg.costmap_timeout_s + cfg.costmap_settle_s

    def _elapsed(self) -> float:
        return self._now - self._entered

    @staticmethod
    def _event(obs: Observation, kind: str, req: Optional[int]) -> Optional[Event]:
        return next((e for e in obs.events if e.kind == kind and e.req == req), None)

    def _abandon(self, reason: str, counts: bool = True) -> None:
        """A target attempt failed (or its target was invalidated): stand down through ABANDON. The FSM never carries on
        blindly - it stops, tells the world model, and lets cognition choose again."""
        self.m.abandon = (reason, counts)
        self._emit(Stop(), CancelNav())
        self._go(S.ABANDON, reason)

    def _belief(self, obs: Observation, victim_id: str) -> Optional[VictimBelief]:
        return obs.world.victim(victim_id) if obs.world is not None else None

    def _zone(self, obs: Observation) -> Optional[Zone]:
        return obs.world.zone if obs.world is not None else None

    def _robot_fresh(self, obs: Observation, max_age: float = 1.0) -> bool:
        return obs.robot is not None and obs.t - obs.robot_stamp <= max_age

    # ============================================================================================ INIT
    def _tick_init(self, obs: Observation) -> None:
        ok = (obs.ready and obs.robot is not None and obs.world is not None and obs.world.zone is not None
              and obs.grid is not None and obs.magnet is not None)
        if not ok:
            self.d.pop('ready_since', None)
            if self._elapsed() > self.cfg.init_timeout_s:
                self._abort('the system never became ready (TF, world model, map, Nav2, services or the magnet)')
            return
        if obs.magnet.attached:
            self._abort('the magnet reports a victim held at start-up')
            return
        if isinstance(obs.grid, GridMap) and self.planner is None:
            self.planner = CoveragePlanner(obs.grid, self.cfg)
        since = self.d.setdefault('ready_since', obs.t)
        if obs.t - since >= self.cfg.init_settle_s:          # let localisation and the belief settle before believing either
            self._go(S.PERCEIVE, 'system ready')

    # ============================================================================================ PERCEIVE
    def _tick_perceive(self, obs: Observation) -> None:
        """A stationary 360-degree look: turn a step, stop, look. (Positions are withheld while the camera turns.)"""
        if not self._robot_fresh(obs):
            self._emit(Stop())
            if self._elapsed() > 30.0:
                self._look_failed('no localisation during the look')
            return
        cfg, yaw = self.cfg, obs.robot.yaw
        if 'n' not in self.d:
            self.d['n'] = max(2, math.ceil(2 * math.pi / cfg.sweep_max_step_rad))
            self.d['step'] = 2 * math.pi / self.d['n']
            self.d['k'] = 0
            self.d['yaw0'] = yaw
            self.d['phase'] = 'dwell'
            self.d['until'] = obs.t + cfg.sweep_dwell_s
            self.d['pos'] = obs.robot.xy
        budget = 30.0 + self.d['n'] * (self.d['step'] / cfg.sweep_turn_rps * 2.0 + cfg.sweep_dwell_s)
        if self._elapsed() > budget:
            self._emit(Stop())
            self._look_failed(f'the look took longer than {budget:.0f} s')
            return
        if self.d['phase'] == 'dwell':
            self._emit(Stop())
            if obs.t >= self.d['until']:
                if self.d['k'] >= self.d['n']:
                    if self.planner is not None:
                        new = self.planner.mark_looked(*self.d['pos'])
                        self.reason = f'looked all round from ({self.d["pos"][0]:.2f}, {self.d["pos"][1]:.2f}): {new} new cells'
                    self._go(S.UPDATE_WORLD, 'looked all the way round')
                    return
                self.d['k'] += 1
                self.d['phase'] = 'turn'
                self.d['target_yaw'] = wrap(self.d['yaw0'] + self.d['k'] * self.d['step'])
            return
        err = wrap(self.d['target_yaw'] - yaw)
        if abs(err) <= cfg.sweep_yaw_tol_rad:
            self.d['phase'] = 'dwell'
            self.d['until'] = obs.t + cfg.sweep_dwell_s
            self._emit(Stop())
            return
        self._drive(0.0, math.copysign(min(cfg.sweep_turn_rps, max(0.15, 2.0 * abs(err))), err))

    def _look_failed(self, why: str) -> None:
        if self.m.viewpoint is not None and self.planner is not None:
            self.planner.mark_failed(self.m.viewpoint)
        self._go(S.UPDATE_WORLD, f'look abandoned: {why}')

    # ============================================================================================ UPDATE_WORLD
    def _tick_update_world(self, obs: Observation) -> None:
        """Wait until the world model has published a belief newer than what just happened (a look, a status change)."""
        if obs.world is not None and obs.world.stamp >= self._entered + self.cfg.world_settle_s and self._robot_fresh(obs, 2.0):
            self._go(S.SELECT_TARGET, 'the world model has caught up')
        elif self._elapsed() > max(10.0, self.cfg.world_stale_s * 2):
            self._abort('the world model stopped publishing')

    # ============================================================================================ SELECT_TARGET
    def _tick_select_target(self, obs: Observation) -> None:
        d = self.d
        if 'req' not in d:
            d['req'] = self._req_id()
            self.metrics['select_calls'] += 1
            self._emit(CallSelect(d['req']))
            return
        if 'status_req' in d:                                      # the target chosen; now the world model must agree
            ev = self._event(obs, STATUS_DONE, d['status_req'])
            if ev is None:
                if self._elapsed() > d['status_deadline']:
                    self._abort('the world model did not answer a TARGETED request')
                return
            if not ev.ok:
                self.m.status_refusals += 1
                if self.m.status_refusals >= 3:
                    self._abort(f'the world model keeps refusing TARGETED: {ev.detail}')
                    return
                d.clear()                                          # ask cognition again, from the refreshed belief
                return
            self.m.status_refusals = 0
            self.m.targeted_t = obs.t
            self._go(S.NAVIGATE_TO_APPROACH, f'target {self.m.target} chosen by cognition')
            return
        ev = self._event(obs, SELECT_DONE, d['req'])
        if ev is None:
            if self._elapsed() > self.cfg.select_timeout_s:
                self.m.select_timeouts += 1
                if self.m.select_timeouts >= 2:
                    self._abort('cognition is not answering SelectTarget')
                else:
                    d.clear()
            return
        self.m.select_timeouts = 0
        if not ev.ok or not ev.victim_id or ev.approach is None:
            self.m.no_target_streak += 1
            self.reason = ev.detail
            self._go(S.SEARCH, f'no target now ({ev.detail[:120]})')
            return
        b = self._belief(obs, ev.victim_id)
        if b is None or b.status not in (DETECTED, TARGETED):
            d.clear()                                              # cognition answered from a belief that has since moved on
            return
        self.m.no_target_streak = 0
        self.m.target, self.m.target_xy, self.m.target_conf = ev.victim_id, b.xy, b.confidence
        self.m.approach = ev.approach
        if b.status == TARGETED:
            self.m.targeted_t = obs.t
            self._go(S.NAVIGATE_TO_APPROACH, f'target {ev.victim_id} (already TARGETED)')
            return
        d['status_req'] = self._req_id()
        d['status_deadline'] = self._elapsed() + 8.0
        self.metrics['status_calls'] += 1
        self._emit(SetStatus(d['status_req'], ev.victim_id, TARGETED))

    # ============================================================================================ SEARCH
    def _tick_search(self, obs: Observation) -> None:
        """No target now. Look somewhere new - or, if everywhere that could hold a victim has been looked at, say so."""
        d, cfg = self.d, self.cfg
        if self.planner is None and isinstance(obs.grid, GridMap):
            self.planner = CoveragePlanner(obs.grid, cfg)
        if self.planner is None or not self._robot_fresh(obs, 2.0):
            if self._elapsed() > 30.0:
                self._abort('SEARCH has no map or no localisation')
            return
        if 'tag' not in d and d.get('wait_until') is None:
            pick = None if self.m.viewpoint_failures >= cfg.max_viewpoint_failures else self.planner.next_viewpoint(obs.robot.xy)
            if pick is None:
                unresolved = [v.id for v in (obs.world.victims if obs.world else ()) if v.status in (UNKNOWN, DETECTED, TARGETED)]
                cov = self.planner.coverage()
                if unresolved and self.m.exhausted_retries < cfg.select_retries:
                    self.m.exhausted_retries += 1
                    d['wait_until'] = obs.t + cfg.select_retry_s
                    self.reason = f'search exhausted (coverage {cov:.2f}) but {unresolved} are known and not selectable: retrying'
                    return
                left = [v.id for v in (obs.world.victims if obs.world else ()) if v.status not in (RESCUED, UNREACHABLE)]
                why = (f'coverage {cov:.2f} of the free space (goal {cfg.coverage_goal:.2f}), {len(self.planner.viewpoints())} viewpoints, '
                       f'{len(self.planner.failed)} unreachable, {self.m.viewpoint_failures} failures in a row')
                if left:                                            # victims are known and none can be acted on: that is not completion
                    self._abort(f'search exhausted ({why}) but {left} are still unresolved and none is selectable')
                    return
                self._go(S.MISSION_COMPLETE, f'search exhausted ({why}); rescued {self.m.rescued}, given up {self.m.given_up}')
                return
            idx, (vx, vy), gain = pick
            self.m.viewpoint = idx
            if dist(obs.robot.xy, (vx, vy)) < 0.4:                 # already there: just look
                self._go(S.PERCEIVE, f'looking from here (viewpoint {idx}, +{gain} cells)')
                return
            d['tag'] = next(self._tag)
            d['t_send'] = obs.t
            d['idx'] = idx
            self.metrics['nav_goals'] += 1
            self.reason = f'searching: viewpoint {idx} at ({vx:.2f}, {vy:.2f}), +{gain} cells, coverage {self.planner.coverage():.2f}'
            self._emit(SendNav(Pose2(vx, vy, bearing(obs.robot.xy, (vx, vy))), d['tag']))
            return
        if d.get('wait_until') is not None:
            if obs.t >= d['wait_until']:
                self._go(S.SELECT_TARGET, 'retrying selection after a full search')
            return
        ev = self._event(obs, NAV_DONE, d['tag'])
        if ev is not None:
            if ev.nav_status == 'succeeded':
                self.m.viewpoint_failures = 0
                self._go(S.PERCEIVE, f'reached viewpoint {d["idx"]}')
            else:
                self.metrics['nav_failures'] += 1
                self.m.viewpoint_failures += 1
                self.planner.mark_failed(d['idx'])
                d.clear()                                          # pick another viewpoint
            return
        if obs.t - d['t_send'] > cfg.search_nav_timeout_s:
            self._emit(CancelNav())
            self.metrics['nav_failures'] += 1
            self.m.viewpoint_failures += 1
            self.planner.mark_failed(d['idx'])
            d.clear()

    # ============================================================================================ NAVIGATE_TO_APPROACH
    def _target_invalid(self, obs: Observation, confidence: bool = True) -> Optional[str]:
        """Why the current target can no longer be pursued (None = still valid). Cognition owns the ranking; this only
        notices that the belief the choice was made from has changed under it."""
        b = self._belief(obs, self.m.target)
        if b is None:
            return 'the target is gone from the world model'
        if b.status != TARGETED and obs.world.stamp > self.m.targeted_t + 0.6:      # the published belief may predate our own status change
            return f'the target is now {STATUS_NAMES.get(b.status, b.status)}, not TARGETED'
        # Materially, and SINCE cognition chose it: a belief that was already faint when chosen is cognition's call (it weighs
        # confidence), and driving towards it is how it gets seen again. Abandoning it at once only had cognition re-pick it.
        if confidence and b.confidence < self.cfg.min_target_confidence \
                and b.confidence < self.m.target_conf * self.cfg.confidence_drop_ratio:
            return f'the target\'s confidence decayed to {b.confidence:.2f} (it was {self.m.target_conf:.2f} when chosen)'
        if dist(b.xy, self.m.target_xy) > self.cfg.target_move_m:
            return f'the target\'s track moved {dist(b.xy, self.m.target_xy):.2f} m: a different belief'
        return None

    def _tick_navigate_to_approach(self, obs: Observation) -> None:
        d, cfg = self.d, self.cfg
        if 'tag' not in d:
            if not self._robot_fresh(obs, 2.0):
                if self._elapsed() > 20.0:
                    self._abandon('no localisation to plan the approach from')
                return
            b = self._belief(obs, self.m.target)
            goal = self.m.approach
            if b is not None and goal is not None and dist(b.xy, self.m.target_xy) > 0.05:
                goal = approach_pose(obs.robot.xy, b.xy, cfg.approach_standoff_m)      # the belief moved: re-derive, never store
            d['tag'] = next(self._tag)
            d['t_send'] = obs.t
            self.metrics['nav_goals'] += 1
            self._emit(SendNav(goal, d['tag']))
            return
        bad = self._target_invalid(obs)
        if bad:
            self._abandon(bad, counts='confidence' in bad)            # a target that keeps fading out is bounded like any failure
            return
        ev = self._event(obs, NAV_DONE, d['tag'])
        if ev is not None:
            if ev.nav_status == 'succeeded':
                self._go(S.ALIGN, 'at the approach pose')
            else:
                self.metrics['nav_failures'] += 1
                self._abandon(f'navigation to the approach pose {ev.nav_status}: {ev.detail}'.rstrip(': '), counts=True)
            return
        if obs.t - d['t_send'] > cfg.nav_timeout_s:
            self.metrics['nav_failures'] += 1
            self._abandon('navigation to the approach pose timed out', counts=True)

    # ============================================================================================ ALIGN
    def _best_sighting(self, obs: Observation, expected_rel: float, range_m: float) -> Optional[Tuple[float, float]]:
        """(bearing relative to the heading, age) of the victim in the camera - the one consistent with where the stored
        position says the target is: at its bearing, AND about as wide as a victim at that range (a blob at the right bearing
        but far narrower is something farther away - measured: a victim moved off behind the target's spot kept ALIGN
        creeping onto empty floor). None = not in view."""
        if obs.camera is None:
            return None
        best = None
        for s in obs.sightings:
            age = obs.t - s.stamp
            if age > 0.6:
                continue
            rel = -math.atan2(s.u - obs.camera.cx, obs.camera.fx)
            off = abs(wrap(rel - expected_rel))
            expected_w = obs.camera.fx * self.cfg.victim_width_m / max(range_m, 0.05)   # range from base_link: lenient
            if s.width < self.cfg.align_width_ratio * expected_w:
                continue
            if off <= self.cfg.align_bearing_gate_rad and (best is None or off < best[0]):
                best = (off, rel, age)
        return None if best is None else (best[1], best[2])

    def _tick_align(self, obs: Observation) -> None:
        """From the approach pose into magnet contact, with the base driven directly and slowly, never backwards.

        Range comes from the world model's STORED position (a colour blob touching the image border, which is every
        blob closer than about a metre, has no placed position, and inside 0.3 m the camera's near clip hides the victim
        altogether). Bearing and 'is it still there' come from the FRESH 2D detection while the victim is visible.
        Nothing here drives without a way to notice the victim is gone: in the visible zone a missing sighting stops the
        robot within `align_lost_timeout_s`; the last stretch inside the blind zone is bounded in distance and time."""
        d, cfg = self.d, self.cfg
        if not self._robot_fresh(obs):
            self._emit(Stop())
            d['nopose'] = d.get('nopose', obs.t)
            if obs.t - d['nopose'] > 2.0:
                self._abandon('localisation lost during ALIGN')
            return
        d.pop('nopose', None)
        bad = self._target_invalid(obs, confidence=False)          # confidence legitimately decays once the blob is too close to place
        if bad:
            self._abandon(bad, counts=False)
            return
        b = self._belief(obs, self.m.target)
        r = obs.robot
        vxy = b.xy
        est = dist(r.xy, vxy)
        if 'phase' not in d:
            self.metrics['align_attempts'] += 1
            if abs(est - cfg.approach_standoff_m) > cfg.align_max_start_error_m:
                self._abandon(f'ALIGN starts {est:.2f} m from the target, not the {cfg.approach_standoff_m:.2f} m standoff', counts=True)
                return
            d.update(phase='turn', start=r.xy, d0=est, seen_t=obs.t, ok_since=None)
        if self._elapsed() > cfg.align_timeout_s:
            self._abandon('ALIGN timed out', counts=True)
            return
        stored_rel = wrap(bearing(r.xy, vxy) - r.yaw)
        sight = self._best_sighting(obs, stored_rel, est)
        visible_zone = est > cfg.align_blind_center_dist_m
        if sight is not None:
            d['seen_t'] = obs.t - sight[1]
        if visible_zone and obs.t - d['seen_t'] > cfg.align_lost_timeout_s:
            self._abandon(f'the victim is not in view ({obs.t - d["seen_t"]:.1f} s) with {est:.2f} m to go: lost', counts=True)
            return
        e = sight[0] if sight is not None else stored_rel
        gap = contact_gap(r, vxy, cfg)
        clash = self._other_within_reach(obs, r)
        if clash:
            self._abandon(f'ambiguous target: {clash} is within reach of the magnet too', counts=False)
            return
        if d['phase'] == 'turn':
            if abs(e) <= cfg.align_heading_tol_rad:
                d['ok_since'] = d['ok_since'] if d['ok_since'] is not None else obs.t
                self._emit(Stop())
                if obs.t - d['ok_since'] >= 0.3:
                    d['phase'] = 'creep'
                return
            d['ok_since'] = None
            self._drive(0.0, math.copysign(min(cfg.align_turn_rps, max(0.12, 2.0 * abs(e))), e))
            return
        if d['phase'] == 'creep':
            travelled = dist(r.xy, d['start'])
            limit = max(0.0, d['d0'] - cfg.contact_center_dist_m) + cfg.align_max_extra_creep_m
            if gap <= cfg.align_stop_gap_m:
                d['phase'], d['settle_until'] = 'settle', obs.t + 0.6
                self._emit(Stop())
                return
            if travelled > limit:
                self._abandon(f'ALIGN crept {travelled:.2f} m without reaching contact (limit {limit:.2f} m)', counts=True)
                return
            v = cfg.align_speed_mps if gap > 0.03 else 0.5 * cfg.align_speed_mps
            self._drive(v, max(-0.25, min(0.25, cfg.align_heading_gain * e)))
            return
        self._emit(Stop())                                          # settle: rest before the coil is energised
        if obs.t >= d['settle_until']:
            self._go(S.ATTACH, f'in contact range (estimated gap {gap * 100:.1f} cm)')

    def _other_within_reach(self, obs: Observation, r: Pose2) -> str:
        """Another believed victim closer to the magnet than the identity clearance: which one would attach is not certain."""
        if obs.world is None:
            return ''
        face = magnet_face(r, self.cfg)
        for v in obs.world.victims:
            if v.id != self.m.target and v.status not in (RESCUED,) and dist(v.xy, face) < self.cfg.identity_clearance_m:
                return v.id
        return ''

    # ============================================================================================ ATTACH
    def _tick_attach(self, obs: Observation) -> None:
        """Ask the magnet to hold, and believe only its answer. A refusal de-energises the coil (a refused coil must not
        stay live), creeps a little further in and asks again, a bounded number of times; then the attempt is given up."""
        d, cfg = self.d, self.cfg
        if 'phase' not in d:
            d.update(phase='reset' if (obs.magnet is not None and obs.magnet.energized) else 'request', tries=0)
        if self._elapsed() > cfg.attach_timeout_s * (3 + 2 * cfg.attach_retries):
            self._abandon('ATTACH timed out', counts=True)
            return
        ph = d['phase']
        if ph in ('reset', 'retract'):
            if d.get('off_req') is None:
                d['off_req'] = self._req_id()
                d['off_deadline'] = obs.t + cfg.attach_timeout_s
                self.metrics['magnet_requests'] += 1
                self._emit(SetMagnet(d['off_req'], False))
                return
            ev = self._event(obs, MAGNET_DONE, d['off_req'])
            if ev is None and obs.t < d['off_deadline']:
                return
            d['off_req'] = None
            if ph == 'reset':
                d['phase'] = 'request'
                return
            d['tries'] += 1
            if d['tries'] > cfg.attach_retries:
                self._abandon(f'the magnet refused to attach ({d["tries"]} tries)', counts=True)
                return
            d.update(phase='creep', start=obs.robot.xy if obs.robot else (0.0, 0.0))
            return
        if ph == 'request':
            # Stand still first: ALIGN ends pressing against the victim, and a weld made while it is still pushed (or tilted)
            # is frozen that way. Measured: such a weld lifted the robot 1 mm, the wheels lost the floor, and the robot sat
            # still while its odometry - and so its localisation - "drove" the victim to the safe zone.
            d.setdefault('settle_until', obs.t + cfg.attach_settle_s)
            if obs.t < d['settle_until']:
                self._emit(Stop())
                return
            d['req'] = self._req_id()
            d['deadline'] = obs.t + cfg.attach_timeout_s
            d['phase'] = 'wait'
            self.metrics['magnet_requests'] += 1
            self._emit(SetMagnet(d['req'], True))
            return
        if ph == 'wait':
            ev = self._event(obs, MAGNET_DONE, d['req'])
            if ev is None:
                if obs.t > d['deadline']:
                    d['phase'] = 'retract'
                return
            if ev.ok and ev.attached:
                self._verify_identity(obs)
                return
            self.metrics['attach_refusals'] += 1
            d['phase'] = 'retract'
            return
        if ph == 'creep':                                            # a little further in, then ask again
            if not self._robot_fresh(obs):
                self._emit(Stop())
                return
            if dist(obs.robot.xy, d['start']) >= cfg.attach_retry_creep_m:
                self._emit(Stop())
                d['phase'] = 'request'
                d.pop('settle_until', None)
                return
            self._drive(0.5 * cfg.align_speed_mps, 0.0)

    def _verify_identity(self, obs: Observation) -> None:
        """The magnet says it holds something. Is it the victim we came for? Geometry, not the driver's label: the belief
        nearest where a held victim must be (flush ahead of the magnet) has to be the target, and only the target."""
        if not self._robot_fresh(obs, 2.0) or obs.world is None:
            self._abandon('cannot verify what was picked up: no pose or belief', counts=True)
            return
        held = expected_carried_xy(obs.robot, self.cfg)
        near = sorted(obs.world.victims, key=lambda v: dist(v.xy, held))
        if not near or near[0].id != self.m.target or dist(near[0].xy, held) > self.cfg.identity_gate_m:
            got = near[0].id if near else 'nothing'
            self._abandon(f'identity check failed: the belief at the magnet is {got}, expected {self.m.target}', counts=False)
            return
        self._go(S.VERIFY_CARRY, f'the magnet holds {self.m.target} (belief {dist(near[0].xy, held) * 100:.0f} cm from where it should be)')

    # ============================================================================================ VERIFY_CARRY
    def _tick_verify_carry(self, obs: Observation) -> None:
        """Is the victim STILL held once the robot moves? The hold must be reported continuously, through a small turn."""
        d, cfg = self.d, self.cfg
        m = obs.magnet
        if m is None or not m.attached or not m.energized:
            self._abandon('the hold was lost while verifying the carry', counts=True)
            return
        if not self._robot_fresh(obs) and 'status_req' not in d:
            self._emit(Stop())
            if self._elapsed() > 10.0:
                self._abandon('no localisation to verify the carry', counts=True)
            return
        if 'status_req' in d:
            self._emit(Stop())
            if not d.get('carried'):
                ev = self._event(obs, STATUS_DONE, d['status_req'])
                if ev is None:
                    if self._elapsed() > d['status_deadline']:
                        self._abort('the world model did not answer a CARRIED request')
                    return
                if not ev.ok:
                    self._abandon(f'the world model refused CARRIED: {ev.detail}', counts=False)
                    return
                d['carried'] = True
            if not self._depart_done(obs):                           # the victim's old spot is an obstacle in the SAVED map
                return
            if self._clear_done(obs):                                # the carried victim is part of the robot now, not an obstacle
                self._go(S.NAVIGATE_TO_SAFE_ZONE, f'{self.m.target} is held and verified')
            return
        if 'phase' not in d:
            d.update(phase='hold', until=obs.t + cfg.verify_carry_s / 2, yaw0=obs.robot.yaw)
        if d['phase'] == 'hold':
            self._emit(Stop())
            if obs.t >= d['until']:
                d['phase'] = 'tug_out'
            return
        if d['phase'] in ('tug_out', 'tug_back'):
            target = wrap(d['yaw0'] + (cfg.tug_angle_rad if d['phase'] == 'tug_out' else 0.0))
            err = wrap(target - obs.robot.yaw)
            if abs(err) <= 0.03:
                d['phase'] = 'tug_back' if d['phase'] == 'tug_out' else 'confirm'
                d['until'] = obs.t + 0.5
                self._emit(Stop())
                return
            self._drive(0.0, math.copysign(min(cfg.tug_turn_rps, max(0.12, 2.0 * abs(err))), err))
            return
        self._emit(Stop())                                          # confirm: hold reported through the whole test
        if obs.t >= d['until']:
            d['status_req'] = self._req_id()
            d['status_deadline'] = self._elapsed() + 8.0
            self.metrics['status_calls'] += 1
            self._emit(SetStatus(d['status_req'], self.m.target, CARRIED))

    # ============================================================================================ NAVIGATE_TO_SAFE_ZONE
    def _tick_navigate_to_safe_zone(self, obs: Observation) -> None:
        d, cfg = self.d, self.cfg
        m = obs.magnet
        if m is None or not m.attached:
            self._emit(CancelNav())
            self._abandon('the hold was lost during transport', counts=True)
            return
        zone = self._zone(obs)
        if 'tag' not in d:
            if zone is None or not self._robot_fresh(obs, 2.0):
                if self._elapsed() > 20.0:
                    self._abandon('no safe zone or no localisation to plan the release', counts=True)
                return
            plan = plan_release(zone, obs.robot.xy, self.m.used_spots, cfg)
            if plan is None:
                self._abort('the safe zone has no room left for another victim')
                return
            self.m.release_plan = plan
            d['tag'] = next(self._tag)
            d['t_send'] = obs.t
            d.setdefault('navs', 0)
            d['navs'] += 1
            self.metrics['nav_goals'] += 1
            self.reason = f'carrying {self.m.target}: release at ({plan.spot[0]:.2f}, {plan.spot[1]:.2f}), robot to ({plan.pose.x:.2f}, {plan.pose.y:.2f})'
            self._emit(SendNav(plan.pose, d['tag']))
            return
        ev = self._event(obs, NAV_DONE, d['tag'])
        if ev is not None:
            if ev.nav_status == 'succeeded' and self._robot_fresh(obs, 2.0) and zone is not None:
                ok, why = release_ready(zone, obs.robot, self.m.used_spots, cfg)
                if ok:
                    self._go(S.RELEASE, 'the carried victim is inside the safe zone')
                    return
                self.reason = why
            else:
                self.metrics['nav_failures'] += 1
            if d['navs'] >= 2:
                self._abandon(f'could not reach a release pose ({ev.nav_status}; {self.reason})', counts=True)
                return
            navs = d['navs']
            d.clear()
            d['navs'] = navs                                        # plan again from where the robot is now
            return
        if obs.t - d['t_send'] > cfg.nav_timeout_s:
            self._emit(CancelNav())
            self.metrics['nav_failures'] += 1
            if d['navs'] >= 2:
                self._abandon('navigation to the release pose timed out', counts=True)
            else:
                navs = d['navs']
                d.clear()
                d['navs'] = navs

    # ============================================================================================ RELEASE
    def _tick_release(self, obs: Observation) -> None:
        d, cfg = self.d, self.cfg
        if 'req' not in d:
            if self._robot_fresh(obs, 2.0):
                self.m.released_xy = expected_carried_xy(obs.robot, cfg)
            d.setdefault('tries', 0)
            d['req'] = self._req_id()
            d['deadline'] = obs.t + cfg.release_timeout_s
            self.metrics['magnet_requests'] += 1
            self._emit(Stop(), SetMagnet(d['req'], False))
            return
        self._emit(Stop())
        ev = self._event(obs, MAGNET_DONE, d['req'])
        if ev is None:
            if obs.t > d['deadline']:
                self._release_failed(f'the magnet did not answer within {cfg.release_timeout_s:.0f} s')
            return
        if ev.ok and ev.attached is False:
            self._go(S.VERIFY_RELEASE, 'the magnet reports the victim released')
        else:
            self._release_failed(f'the magnet still reports the victim held ({ev.detail})')

    def _release_failed(self, why: str) -> None:
        tries = self.d.get('tries', 0) + 1
        if tries >= self.cfg.release_attempts:
            self._abort(f'the victim will not release after {tries} tries: {why}')
            return
        self.d.pop('req', None)
        self.d['tries'] = tries

    # ============================================================================================ VERIFY_RELEASE
    def _tick_verify_release(self, obs: Observation) -> None:
        """Rescued means: the magnet says it let go AND the victim is where the safe zone is - not merely that a command
        succeeded. Only then does the world model hear RESCUED (which is terminal)."""
        d, cfg = self.d, self.cfg
        self._emit(Stop())
        m = obs.magnet
        if 'status_req' in d:
            if 'done' not in d:
                ev = self._event(obs, STATUS_DONE, d['status_req'])
                if ev is None:
                    if self._elapsed() > d['status_deadline']:
                        self._abort('the world model did not answer a RESCUED request')
                    return
                if not ev.ok:
                    self._abort(f'the world model refused RESCUED for {self.m.target}: {ev.detail}')
                    return
                d['done'] = True
                self.m.rescued.append(self.m.target)
                if self.m.released_xy is not None:
                    self.m.used_spots.append(self.m.released_xy)
            if self._clear_done(obs):                                # the victim just put down is not to be planned around either
                self._go(S.REASSESS, f'{self.m.target} rescued')
            return
        if m is None or m.attached:
            self._abort('the magnet reports a victim held again after release')
            return
        d.setdefault('since', obs.t)
        if obs.t - d['since'] < cfg.verify_release_s:
            return
        zone = self._zone(obs)
        if zone is None or self.m.released_xy is None or not zone.contains(self.m.released_xy, margin=cfg.release_margin_m / 2):
            self._abort('the released victim is not verifiably inside the safe zone')
            return
        d['status_req'] = self._req_id()
        d['status_deadline'] = self._elapsed() + 8.0
        self.metrics['status_calls'] += 1
        self._emit(SetStatus(d['status_req'], self.m.target, RESCUED))

    # ============================================================================================ REASSESS
    def _tick_reassess(self, obs: Observation) -> None:
        """After every rescue and every abandoned attempt: forget the target, and let cognition decide again from the
        UPDATED world model. This edge is the whole cognitive claim of the project."""
        self.m.target, self.m.approach, self.m.release_plan, self.m.abandon = '', None, None, None
        self._go(S.UPDATE_WORLD, f'reassessing (rescued {len(self.m.rescued)}, given up {len(self.m.given_up)})')

    # ============================================================================================ ABANDON
    def _tick_abandon(self, obs: Observation) -> None:
        """Stand down from a failed or invalidated attempt, in a fixed order: stop, de-energise, tell the world model."""
        d, cfg = self.d, self.cfg
        reason, counts = self.m.abandon or ('unspecified', True)
        if 'steps' not in d:
            d['steps'] = self._plan_abandon(obs, counts)
            d['i'] = 0
            b = self._belief(obs, self.m.target)
            # Standing against the victim means standing inside its obstacle in the SAVED map, where Nav2 cannot plan from:
            # move clear of it (forward only) before handing back to cognition.
            d['leave'] = bool(b is not None and self._robot_fresh(obs)
                              and dist(obs.robot.xy, b.xy) <= cfg.contact_center_dist_m + cfg.depart_trigger_m)
            d['yaw0'] = obs.robot.yaw if obs.robot is not None else 0.0
            self.metrics['abandons'] += 1
        steps: List[Tuple[str, object]] = d['steps']
        if d['i'] >= len(steps):
            if d['leave']:
                d.setdefault('still_since', obs.t)
                if obs.t - d['still_since'] < cfg.depart_pause_s:    # stop first, and stay stopped a moment, before any move
                    self._emit(Stop())
                    return
                if not self._depart_done(obs):
                    return
            if self._clear_done(obs):
                self._go(S.REASSESS, f'abandoned {self.m.target or "the attempt"}: {reason}')
            else:
                self._emit(Stop())
            return
        self._emit(Stop())
        kind, arg = steps[d['i']]
        if 'req' not in d:
            d['req'] = self._req_id()
            d['deadline'] = obs.t + 6.0
            if kind == 'magnet_off':
                self.metrics['magnet_requests'] += 1
                self._emit(SetMagnet(d['req'], False))
            else:
                self.metrics['status_calls'] += 1
                self._emit(SetStatus(d['req'], self.m.target, arg))
            return
        ev = self._event(obs, MAGNET_DONE if kind == 'magnet_off' else STATUS_DONE, d['req'])
        if ev is not None or obs.t > d['deadline']:
            if kind == 'status' and arg == UNREACHABLE and (ev is None or ev.ok):
                self.m.given_up.append(self.m.target)
            d.pop('req')
            d['i'] += 1

    def _plan_abandon(self, obs: Observation, counts: bool) -> List[Tuple[str, object]]:
        steps: List[Tuple[str, object]] = []
        m = obs.magnet
        if m is not None and (m.energized or m.attached):
            steps.append(('magnet_off', None))
        b = self._belief(obs, self.m.target)
        if self.m.target and b is not None and b.status in (TARGETED, CARRIED):
            steps.append(('status', DETECTED))
        if counts and self.m.target:
            n = self.m.attempts.get(self.m.target, 0) + 1
            self.m.attempts[self.m.target] = n
            if n >= self.cfg.max_target_attempts and b is not None and b.status not in (RESCUED, UNREACHABLE):
                steps.append(('status', UNREACHABLE))
        return steps

    # ============================================================================================ terminal states
    def _tick_mission_complete(self, obs: Observation) -> None:
        self._emit(Stop())

    def _tick_mission_aborted(self, obs: Observation) -> None:
        self._emit(Stop(), CancelNav())
