"""ROS glue for cognition: /fire_resq/world_state -> a decision -> /fire_resq/rescue_target (+ the SelectTarget service).

Thin on purpose: the rules live in prioritizer.py (pure, unit-tested). This node
  - converts WorldState into a WorldView (it sees only that message and the planner: no camera, no Gazebo, no hardware),
  - answers path queries through Nav2's ComputePathToPose (Nav2PathQuery),
  - re-decides when the world model changes (a victim's status/position, the fire, or the robot having moved far enough to
    change the paths), rate-limited, and on demand through SelectTarget,
  - publishes the decision with its explanation, and logs the full comparison table each time.

It selects; it does not act. Marking a victim TARGETED, navigating and the rest of the rescue loop are the FSM's (Phase 10).
"""
from __future__ import annotations

import json
import math
import threading
from typing import Optional, Tuple

import rclpy
from fire_resq_interfaces.msg import RescueTarget, ScoreBreakdown, WorldState
from fire_resq_interfaces.srv import SelectTarget
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import DEFAULT_WEIGHTS, WEIGHT_NAMES, PrioritizerConfig
from .factors import approach_pose, distance
from .nav2_path_query import Nav2PathQuery
from .path_cache import CachedPathQuery
from .prioritizer import make_prioritizer
from .types import CANDIDATE_STATUSES, Decision, PlanningContext, VictimView, WorldView

_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
_CONFIG_SCALARS = ('extent_m', 'fire_hazard_radius_m', 'fire_risk_decay_m', 'approach_standoff_m')
_DEFAULTS_SCALAR = {'extent_m': 7.07, 'fire_hazard_radius_m': 1.0, 'fire_risk_decay_m': 1.0, 'approach_standoff_m': 0.4}


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PrioritizerNode(Node):
    def __init__(self, path_query=None, **kwargs):
        """`path_query` overrides the Nav2 adapter (tests inject a fake); by default the node asks Nav2."""
        super().__init__('prioritizer_node', **kwargs)
        for name, val in (('decision_model', 'weighted_utility'), ('world_state_topic', '/fire_resq/world_state'),
                          ('rescue_target_topic', '/fire_resq/rescue_target'),
                          ('decision_report_topic', '/fire_resq/decision_report'),
                          ('select_target_service', '/fire_resq/select_target'),
                          ('planner_action', '/compute_path_to_pose'), ('planner_id', ''), ('world_frame', 'map'),
                          ('path_query_timeout_s', 3.0), ('auto_reassess', True), ('reassess_period_s', 1.0),
                          ('requery_distance_m', 0.5), ('position_change_m', 0.15), ('retry_period_s', 3.0),
                          ('refresh_period_s', 15.0), ('path_cache_ttl_s', 15.0), ('path_cache_quantum_m', 0.05)):
            self.declare_parameter(name, val)
        for k, v in _DEFAULTS_SCALAR.items():
            self.declare_parameter(k, v)
        for k, v in DEFAULT_WEIGHTS.items():
            self.declare_parameter(f'weights.{k}', v)
        p = self.get_parameter
        cfg = PrioritizerConfig.from_dict({**{k: p(k).value for k in _CONFIG_SCALARS},
                                           **{f'weights.{k}': p(f'weights.{k}').value for k in WEIGHT_NAMES}})
        self.prioritizer = make_prioritizer(p('decision_model').value, cfg)
        self.world_frame = p('world_frame').value
        self.auto = bool(p('auto_reassess').value)
        self.period = float(p('reassess_period_s').value)
        self.requery_m, self.change_m = float(p('requery_distance_m').value), float(p('position_change_m').value)
        self.retry_s, self.refresh_s = float(p('retry_period_s').value), float(p('refresh_period_s').value)

        # The planner client has its own node and thread, so a decision may block on a query inside a callback of this
        # single-threaded node without starving the planner's replies.
        inner = path_query or Nav2PathQuery(p('planner_action').value, self.world_frame, p('planner_id').value,
                                            float(p('path_query_timeout_s').value))
        self.path_query = CachedPathQuery(inner, float(p('path_cache_ttl_s').value), float(p('path_cache_quantum_m').value))
        self.ctx = PlanningContext(self.path_query)
        self.lock = threading.Lock()                                # one decision at a time (timer vs service)
        self.world_msg: Optional[WorldState] = None
        self.last_world: Optional[dict] = None
        self.last_robot: Optional[Tuple[float, float]] = None
        self.last_time = -1e9
        self.needs_retry = False
        self.decisions = 0

        self.target_pub = self.create_publisher(RescueTarget, p('rescue_target_topic').value, _LATCHED)
        self.report_pub = self.create_publisher(String, p('decision_report_topic').value, _LATCHED)
        self.create_subscription(WorldState, p('world_state_topic').value, self._on_world, 10)
        self.create_service(SelectTarget, p('select_target_service').value, self._on_select)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"cognition ready: model '{self.prioritizer.name}', weights {self.prioritizer.weights}")

    # ------------------------------------------------------------------------------ world in
    def _on_world(self, msg: WorldState) -> None:
        self.world_msg = msg

    @staticmethod
    def to_view(msg: WorldState) -> WorldView:
        victims = tuple(VictimView(v.id, v.position.x, v.position.y, float(v.confidence), int(v.status)) for v in msg.victims)
        st = msg.header.stamp
        return WorldView(st.sec + st.nanosec * 1e-9, victims, bool(msg.fire_known), (msg.fire_position.x, msg.fire_position.y),
                         (msg.safe_zone_pose.position.x, msg.safe_zone_pose.position.y),
                         (msg.robot_pose.position.x, msg.robot_pose.position.y), yaw_of(msg.robot_pose.orientation))

    def _changed(self, w: WorldView) -> bool:
        """Has the world moved on since the last decision? By DISTANCE from what that decision used, never by rounding
        positions to a grid: a victim resting near a rounding boundary and jittering by a millimetre would otherwise look
        'changed' every tick. Ids, statuses and whether the fire is known count exactly; positions count beyond change_m."""
        last = self.last_world
        if last is None:
            return True
        if {v.id for v in w.victims} != set(last['victims']) or w.fire_known != last['fire_known']:
            return True
        for v in w.victims:
            status, x, y = last['victims'][v.id]
            if v.status != status or math.hypot(v.x - x, v.y - y) > self.change_m:
                return True
        return w.fire_known and math.hypot(w.fire_xy[0] - last['fire'][0], w.fire_xy[1] - last['fire'][1]) > self.change_m

    # ------------------------------------------------------------------------------ deciding
    def _tick(self) -> None:
        """Reassessment: decide again when the world model changed (rate-limited)."""
        msg = self.world_msg
        if not self.auto or msg is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.last_time
        w = self.to_view(msg)
        changed = self._changed(w)
        moved = self.last_robot is None or distance(w.robot_xy, self.last_robot) > self.requery_m
        # Three more reasons to decide again with an unchanged world: the last answer was incomplete (nothing selectable, or
        # the planner could not be asked - typically Nav2 still starting), and a slow refresh so the paths follow the costmap.
        retry = self.needs_retry and age >= self.retry_s
        refresh = bool(w.victims) and age >= self.refresh_s
        if not ((age >= self.period and (changed or (moved and w.victims))) or retry or refresh):
            return
        if not self.lock.acquire(blocking=False):
            return
        try:
            why = ('the world model changed' if changed else 'the robot moved' if moved else
                   'the last decision was incomplete' if retry else 'periodic refresh of the paths')
            self._decide(w, why, fresh=retry or (refresh and not changed and not moved))
        finally:
            self.lock.release()

    def _decide(self, w: WorldView, why: str, fresh: bool = False) -> Decision:
        if fresh:
            self.path_query.invalidate()             # a retry or refresh exists to hear the planner's CURRENT answer
        asked0, hits0 = self.path_query.asked, self.path_query.hits
        d = self.prioritizer.select(w, self.ctx)
        self.decisions += 1
        self.last_world = {'victims': {v.id: (v.status, v.x, v.y) for v in w.victims}, 'fire_known': w.fire_known, 'fire': w.fire_xy}
        self.last_robot = w.robot_xy
        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        cands = any(v.status in CANDIDATE_STATUSES for v in w.victims)
        self.needs_retry = (d.selected is None and cands) or any(x.path_status == 'unknown' for x in d.ranked)
        target = self._to_target(d, w)
        self.target_pub.publish(target)
        report = d.to_dict()
        report['why'] = why
        report['world'] = {'stamp': w.stamp, 'robot': list(w.robot_xy), 'fire_known': w.fire_known, 'fire': list(w.fire_xy),
                           'safe_zone': list(w.safe_zone_xy),          # the exact input, so the decision can be replayed from its own report
                           'victims': [{'id': v.id, 'x': v.x, 'y': v.y, 'confidence': v.confidence, 'status': v.status} for v in w.victims]}
        self.report_pub.publish(String(data=json.dumps(report)))
        self.get_logger().info(f'decision #{self.decisions} ({why}; planner asked {self.path_query.asked - asked0}x, '
                               f'{self.path_query.hits - hits0} answers from memory)\n{self._table(d)}')
        return d

    def _on_select(self, req: SelectTarget.Request, res: SelectTarget.Response):
        msg = self.world_msg
        if msg is None:
            res.found, res.reason = False, 'no world state received yet'
            return res
        with self.lock:
            d = self._decide(self.to_view(msg), 'SelectTarget was called')
        res.target = self._to_target(d, self.to_view(msg))
        res.found = d.selected is not None
        res.reason = d.reason if d.selected is None else ''
        return res

    # ------------------------------------------------------------------------------ decision out
    def _to_target(self, d: Decision, w: WorldView) -> RescueTarget:
        t = RescueTarget()
        t.header.stamp, t.header.frame_id = self.get_clock().now().to_msg(), self.world_frame
        t.approach_pose.header = t.header
        s = d.selected
        if s is None:
            t.rationale = d.reason
            return t
        t.victim_id, t.utility = s.victim_id, float(s.utility)
        # Provisional: the point the planner was asked about, facing the victim. The executable approach pose (magnet
        # standoff, alignment) is the rescue FSM's to compute in Phase 10.
        ax, ay, yaw = approach_pose(w.robot_xy, s.position, self.prioritizer.cfg.approach_standoff_m)
        t.approach_pose.pose.position.x, t.approach_pose.pose.position.y = ax, ay
        t.approach_pose.pose.orientation.z, t.approach_pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = s.factors
        b = ScoreBreakdown()
        b.fire_risk, b.distance_to_fire, b.robot_distance = float(f['fire_risk']), float(f['distance_to_fire_m']), float(f['robot_path_m'])
        b.accessibility, b.rescue_cost, b.perception_confidence = float(f['accessibility']), float(f['rescue_cost_m']), float(f['confidence'])
        b.weight_names, b.weights, b.total = list(WEIGHT_NAMES), [float(d.weights[k]) for k in WEIGHT_NAMES], float(s.utility)
        t.breakdown = b
        t.rationale = self._rationale(d)
        return t

    @staticmethod
    def _rationale(d: Decision) -> str:
        sel = d.selected
        lines = [f'{d.model}: selected {sel.victim_id}. {sel.rationale}']
        for s in d.ranked:
            if s is not sel:
                lines.append(f'  not {s.victim_id}: ' + (s.exclusion if not s.eligible else f'U={s.utility:+.3f} (lower). {s.rationale}'))
        return '\n'.join(lines)

    @staticmethod
    def _table(d: Decision) -> str:
        head = f"model {d.model}, weights {d.weights}\n  {'id':<8}{'utility':>9}  {'risk':>5} {'d_fire':>7} {'trip':>6} {'cost':>6} {'direct':>6} {'conf':>5}  note"
        rows = []
        for s in d.ranked:
            f = s.factors
            if s.eligible:
                rows.append(f"  {s.victim_id:<8}{s.utility:>+9.3f}  {f['fire_risk']:>5.2f} {f['distance_to_fire_m']:>7.2f} {f['robot_path_m']:>6.2f} "
                            f"{f['rescue_cost_m']:>6.2f} {f['accessibility']:>6.2f} {f['confidence']:>5.2f}  {'<- SELECTED' if s is d.selected else ''}")
            else:
                rows.append(f'  {s.victim_id:<8}{"excluded":>9}  {s.exclusion}')
        return head + '\n' + '\n'.join(rows) + (f'\n  nothing selected: {d.reason}' if d.selected is None else '')

    def destroy_node(self):
        try:
            self.path_query.destroy()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PrioritizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'{node.decisions} decisions, {getattr(node.path_query, "queries", 0)} path queries')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
