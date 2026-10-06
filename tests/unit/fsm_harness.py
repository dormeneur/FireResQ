"""A closed-loop stand-in for everything the rescue FSM talks to, so the FSM can be exercised WITHOUT ROS or Gazebo.

`SimEnv` plays the world: a robot that drives on Twist commands and on Nav2-style goals, victims with a true position that
the robot can push and carry, a camera that sees them (or not), a magnet whose hold sensor tells the truth, a world model
that believes noisy positions, and a cognition stand-in that answers SelectTarget. The FSM sees only Observations and its
own Effects come back as Events - exactly the node's contract - so what these tests prove about the FSM's procedure is what
the real node will run. It does NOT prove the physics; tests/sim/test_rescue_loop.py does that against Gazebo.

The victims' TRUE positions live only here. The FSM never receives them: it gets beliefs (with a configurable error), 2D
sightings and the magnet's hold reading, like the real robot.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from fire_resq_planning.config import PlanningConfig
from fire_resq_planning.coverage import GridMap
from fire_resq_planning.fsm import RescueFSM
from fire_resq_planning.geometry import Pose2, Zone, contact_gap, dist, wrap
from fire_resq_planning.types import (CARRIED, COSTMAP_DONE, DETECTED, MAGNET_DONE, NAV_DONE, RESCUED, SELECT_DONE, STATUS_DONE, TARGETED,
                                      Camera, CallSelect, CancelNav, Drive, Event, MagnetReading, Observation,
                                      ClearCostmaps, SendNav, SetMagnet, SetStatus, Sighting, State, Stop, VictimBelief, World)
import numpy as np
from fire_resq_world_model.entities import ACTIVE, TRANSITIONS

DT = 0.05
CAM = Camera(fx=320.0 / math.tan(0.759), cx=320.0, hfov=1.518)


def arena_grid(size_m: float = 5.0, res: float = 0.05, block: bool = True) -> GridMap:
    """A walled square arena (free inside, occupied border, unknown outside) with one partition, in map coordinates."""
    n = int(round(size_m / res)) + 6
    g = np.full((n, n), -1, dtype=np.int8)
    g[3:-3, 3:-3] = 0
    g[3:5, 3:-3] = 100
    g[-5:-3, 3:-3] = 100
    g[3:-3, 3:5] = 100
    g[3:-3, -5:-3] = 100
    if block:
        mid = n // 2
        g[mid - 1:mid + 1, 8:n // 2 + 6] = 100                       # a partition, like the scenario's
    return GridMap(res, -3 * res, -3 * res, g)


@dataclass
class TrueVictim:
    x: float
    y: float
    held: bool = False
    status: int = DETECTED
    known: bool = False                       # has the world model created a track for it yet?
    seen_hits: int = 0


class SimEnv:
    def __init__(self, cfg: PlanningConfig, victims: Dict[str, Tuple[float, float]], zone: Zone, *, grid: Optional[GridMap] = None,
                 pose: Pose2 = Pose2(0.0, 0.0, 0.0), belief_error_m: float = 0.02, tolerance_m: float = 0.02,
                 nav_speed: float = 0.25, hidden: Optional[Dict[str, Callable[[Pose2], bool]]] = None, seed: int = 1,
                 rank: Optional[Callable[[Dict[str, Tuple[float, float]], Pose2], List[str]]] = None):
        self.cfg, self.zone, self.grid = cfg, zone, grid if grid is not None else arena_grid()
        self._blocked = self.grid.blocked()
        self.t = 0.0
        self.robot = pose
        self.tv: Dict[str, TrueVictim] = {k: TrueVictim(*v) for k, v in victims.items()}
        self.belief: Dict[str, VictimBelief] = {}
        self.ids: Dict[str, str] = {}                              # true name -> world-model id (creation order, like the real one)
        self.rng = random.Random(seed)
        self.err = belief_error_m
        self.tol = tolerance_m
        self.nav_speed = nav_speed
        self.hidden = hidden or {}                                 # true name -> "is it visible from this pose?" (occlusion)
        self.rank = rank or (lambda pos, robot: sorted(pos, key=lambda k: dist((pos[k][0], pos[k][1]), robot.xy)))
        self.energized = False
        self.holding: Optional[str] = None
        self.cmd = (0.0, 0.0)
        self.nav: Optional[dict] = None
        self.events: List[Event] = []
        self.pending: List[Tuple[float, Event]] = []
        self.log: List[str] = []
        self.fail_nav = 0                                          # fail the next N navigation goals
        self.refuse_attach = False
        self.drop_after: Optional[float] = None                    # drop the held victim this many seconds after pick-up
        self.picked_at: Optional[float] = None
        self.vanish_on: Optional[Tuple[State, str]] = None         # (FSM state, true name): remove that victim when the FSM enters it
        self.move_on: Optional[Tuple[State, str, float]] = None    # (FSM state, true name, metres): push that victim this much further
                                                                   # away along the robot->victim line when the FSM enters it
        self.cognition_calls = 0
        self.max_speed = 0.3
        self.pose_noise = (0.0, 0.0)                               # constant localisation error (dx, dy) between belief and truth
        self.status_refuse: set = set()

    # ------------------------------------------------------------------------------------------ world
    @property
    def true_robot(self) -> Pose2:
        return self.robot

    def est_robot(self) -> Pose2:
        return Pose2(self.robot.x + self.pose_noise[0], self.robot.y + self.pose_noise[1], self.robot.yaw)

    def _victim_by_id(self, wid: str) -> Optional[str]:
        return next((k for k, v in self.ids.items() if v == wid), None)

    def _visible(self, name: str) -> bool:
        v = self.tv[name]
        if v.held:
            return False
        rel = wrap(math.atan2(v.y - self.robot.y, v.x - self.robot.x) - self.robot.yaw)
        d = dist((v.x, v.y), self.robot.xy)
        if abs(rel) > CAM.hfov / 2 or d > 6.0 or d - 0.056 - 0.05 < 0.10:      # FOV, range, the 10 cm near clip (camera 5 cm ahead)
            return False
        occ = self.hidden.get(name)
        if occ is not None and occ(self.robot):
            return False
        return self._line_of_sight((self.robot.x, self.robot.y), (v.x, v.y))

    def _line_of_sight(self, a, b) -> bool:
        """No occupied or unknown cell of the map between a and b (the same rule the coverage planner uses)."""
        blocked = self._blocked
        g = self.grid
        n = max(2, int(dist(a, b) / (0.5 * g.resolution)))
        for i in range(1, n):
            f = i / n
            r, c = g.cell(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
            if not (0 <= r < g.height and 0 <= c < g.width) or blocked[r, c]:
                return False
        return True

    def _observe(self) -> None:
        for name, v in self.tv.items():
            if v.held or not self._visible(name):
                continue
            d = dist((v.x, v.y), self.robot.xy)
            placed = d > 1.0                                       # closer blobs touch the image border: no position
            if not placed:
                continue
            if name not in self.ids:
                self.ids[name] = f'V{len(self.ids) + 1}'
            wid = self.ids[name]
            b = self.belief.get(wid)
            noise = lambda: self.rng.uniform(-self.err, self.err)
            px, py = v.x + noise() + self.pose_noise[0], v.y + noise() + self.pose_noise[1]
            if b is None:
                self.belief[wid] = VictimBelief(wid, px, py, 0.9, DETECTED, self.t)
            elif b.status not in (CARRIED, RESCUED):
                a = 0.3
                self.belief[wid] = VictimBelief(wid, b.x + a * (px - b.x), b.y + a * (py - b.y), 0.9, b.status, self.t)
            else:
                self.belief[wid] = VictimBelief(wid, b.x, b.y, b.confidence, b.status, self.t)

    def _sightings(self) -> Tuple[Sighting, ...]:
        out = []
        for name in self.tv:
            if self._visible(name):
                v = self.tv[name]
                rel = wrap(math.atan2(v.y - self.robot.y, v.x - self.robot.x) - self.robot.yaw)
                rng = max(math.hypot(v.x - self.robot.x, v.y - self.robot.y) - 0.05, 0.05)    # the camera is 5 cm ahead of the axle
                out.append(Sighting(CAM.cx - CAM.fx * math.tan(rel), CAM.fx * 0.10 / rng, self.t))
        return tuple(out)

    # ------------------------------------------------------------------------------------------ actuation
    def _drive(self) -> None:
        v, w = self.cmd
        v, w = min(max(v, -self.max_speed), self.max_speed), min(max(w, -1.5), 1.5)
        r = self.robot
        yaw = wrap(r.yaw + w * DT)
        x, y = r.x + v * math.cos(yaw) * DT, r.y + v * math.sin(yaw) * DT
        self.robot = Pose2(x, y, yaw)
        # pushing: the magnet face past a free victim's ring pushes it along
        face = (x + self.cfg.magnet_reach_m * math.cos(yaw), y + self.cfg.magnet_reach_m * math.sin(yaw))
        for name, tv in self.tv.items():
            if tv.held:
                continue
            gap = dist(face, (tv.x, tv.y)) - self.cfg.victim_ring_radius_m
            if gap < 0:
                ang = math.atan2(tv.y - face[1], tv.x - face[0])
                tv.x += -gap * math.cos(ang)
                tv.y += -gap * math.sin(ang)
        for name, tv in self.tv.items():
            if tv.held:
                d = self.cfg.contact_center_dist_m
                tv.x, tv.y = x + d * math.cos(yaw), y + d * math.sin(yaw)

    def _navigate(self) -> None:
        n = self.nav
        if n is None:
            return
        if self.t >= n['arrive']:
            self.nav = None
            if n['fail']:
                self.pending.append((self.t, Event(NAV_DONE, n['tag'], nav_status='failed', detail='no plan')))
            else:
                g = n['goal']
                self.robot = Pose2(g.x + self.rng.uniform(-0.05, 0.05), g.y + self.rng.uniform(-0.05, 0.05),
                                   wrap(g.yaw + self.rng.uniform(-0.1, 0.1)))
                self.pending.append((self.t, Event(NAV_DONE, n['tag'], nav_status='succeeded')))
            return
        self.robot = Pose2(self.robot.x + (n['goal'].x - self.robot.x) * DT / max(n['arrive'] - self.t, DT),
                           self.robot.y + (n['goal'].y - self.robot.y) * DT / max(n['arrive'] - self.t, DT), self.robot.yaw)

    # ------------------------------------------------------------------------------------------ effects
    def _apply(self, fx) -> None:
        for e in fx:
            if isinstance(e, Drive):
                self.cmd = (e.v, e.w)
            elif isinstance(e, Stop):
                self.cmd = (0.0, 0.0)
            elif isinstance(e, SendNav):
                fail = self.fail_nav > 0
                self.fail_nav = max(0, self.fail_nav - 1)
                dur = dist(self.robot.xy, e.goal.xy) / self.nav_speed + 1.0
                self.nav = {'tag': e.tag, 'goal': e.goal, 'arrive': self.t + dur, 'fail': fail}
            elif isinstance(e, CancelNav):
                self.nav = None
            elif isinstance(e, CallSelect):
                self._select(e.req)
            elif isinstance(e, SetStatus):
                self._set_status(e)
            elif isinstance(e, ClearCostmaps):
                self.pending.append((self.t + 0.1, Event(COSTMAP_DONE, e.req, ok=True)))
            elif isinstance(e, SetMagnet):
                self._set_magnet(e)

    def _select(self, req: int) -> None:
        self.cognition_calls += 1
        cand = {name: (self.tv[name].x, self.tv[name].y) for name in self.tv
                if name in self.ids and self.belief[self.ids[name]].status in (DETECTED, TARGETED)}
        if not cand:
            self.pending.append((self.t + 0.3, Event(SELECT_DONE, req, ok=False, detail='no eligible victim: none known')))
            return
        order = self.rank({k: (self.belief[self.ids[k]].x, self.belief[self.ids[k]].y) for k in cand}, self.est_robot())
        name = order[0]
        b = self.belief[self.ids[name]]
        from fire_resq_planning.geometry import approach_pose
        self.pending.append((self.t + 0.3, Event(SELECT_DONE, req, ok=True, victim_id=b.id,
                                                  approach=approach_pose(self.est_robot().xy, (b.x, b.y), self.cfg.approach_standoff_m))))

    def _set_status(self, e: SetStatus) -> None:
        b = self.belief.get(e.victim_id)
        ok, msg = False, ''
        if b is None:
            msg = 'unknown victim'
        elif e.victim_id in self.status_refuse:
            msg = 'refused (test)'
        elif e.status == b.status:
            ok = True
        elif e.status not in TRANSITIONS[b.status]:
            msg = f'{b.status} -> {e.status} not allowed'
        elif e.status in ACTIVE and any(o.status in ACTIVE and o.id != b.id for o in self.belief.values()):
            msg = 'one target at a time'
        else:
            ok = True
            self.belief[b.id] = VictimBelief(b.id, b.x, b.y, b.confidence, e.status, b.last_seen)
        self.pending.append((self.t + 0.1, Event(STATUS_DONE, e.req, ok=ok, detail=msg)))

    def _set_magnet(self, e: SetMagnet) -> None:
        if e.attach:
            self.energized = True
            if self.holding is None and not self.refuse_attach:
                near = [(contact_gap(self.robot, (tv.x, tv.y), self.cfg), n) for n, tv in self.tv.items() if not tv.held]
                near = [x for x in near if x[0] <= self.tol]
                if near:
                    self.holding = min(near)[1]
                    self.tv[self.holding].held = True
                    self.picked_at = self.t
            self.pending.append((self.t + 0.2, Event(MAGNET_DONE, e.req, ok=self.holding is not None, attached=self.holding is not None,
                                                     detail='' if self.holding else 'no victim in range')))
        else:
            self.energized = False
            if self.holding is not None:
                self.tv[self.holding].held = False
                self.holding = None
            self.pending.append((self.t + 0.2, Event(MAGNET_DONE, e.req, ok=True, attached=False)))

    # ------------------------------------------------------------------------------------------ the loop
    def observation(self, fsm: RescueFSM) -> Observation:
        events = tuple(e for (ready_t, e) in self.pending if ready_t <= self.t)
        self.pending = [(rt, e) for (rt, e) in self.pending if rt > self.t]
        w = World(tuple(self.belief[k] for k in sorted(self.belief, key=lambda s: int(s[1:]))), self.zone, self.t)
        return Observation(t=self.t, ready=True, robot=self.est_robot(), robot_stamp=self.t, world=w, sightings=self._sightings(),
                           camera=CAM, magnet=MagnetReading(self.energized, self.holding is not None, self.t), grid=self.grid,
                           nav_active=self.nav is not None, events=events)

    def run(self, fsm: RescueFSM, seconds: float, until: Optional[Callable[[], bool]] = None, on_step: Optional[Callable] = None) -> None:
        end = self.t + seconds
        while self.t < end and fsm.state not in (State.MISSION_COMPLETE, State.MISSION_ABORTED):
            if until is not None and until():
                return
            self.t = round(self.t + DT, 6)
            if self.drop_after is not None and self.holding is not None and self.picked_at is not None \
                    and self.t - self.picked_at >= self.drop_after:
                self.tv[self.holding].held = False             # dropped where it is; the coil is still on but nothing is held
                self.holding = None
                self.drop_after = None
            if self.move_on is not None and fsm.state == self.move_on[0] and self.move_on[1] in self.tv:
                v = self.tv[self.move_on[1]]
                b = math.atan2(v.y - self.robot.y, v.x - self.robot.x)
                v.x, v.y = v.x + self.move_on[2] * math.cos(b), v.y + self.move_on[2] * math.sin(b)
                self.move_on = None
            if self.vanish_on is not None and fsm.state == self.vanish_on[0] and self.vanish_on[1] in self.tv:
                del self.tv[self.vanish_on[1]]
                self.vanish_on = None
            self._navigate()
            self._drive()
            self._observe()
            obs = self.observation(fsm)
            fx = fsm.step(obs)
            self._apply(fx)
            if on_step is not None:
                on_step(obs, fx)
