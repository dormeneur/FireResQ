"""The cognition NODE against synthetic ROS traffic and a FAKE Nav2 planner - no Gazebo.

A rig publishes WorldState; a fake `compute_path_to_pose` action server answers path queries from a rule the test sets
(straight lines, detours, "no path", or nothing at all); the real PrioritizerNode and the real Nav2PathQuery adapter run
in-process. This checks the glue: the ROS conversion, the planner protocol and its failure modes, the published
RescueTarget/explanation, SelectTarget, and reassessment. The decision rules are pinned by tests/unit/test_cognition_lib.py.
"""
import json
import math
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not sourced')
pytest.importorskip('fire_resq_interfaces', reason='workspace not sourced: source install/setup.bash')

from action_msgs.msg import GoalStatus  # noqa: E402
from fire_resq_interfaces.msg import RescueTarget, VictimState, WorldState  # noqa: E402
from fire_resq_interfaces.srv import SelectTarget  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from fire_resq_cognition.nav2_path_query import Nav2PathQuery  # noqa: E402
from fire_resq_cognition.prioritizer_node import PrioritizerNode  # noqa: E402

CFG = Path(__file__).resolve().parents[2] / 'src' / 'fire_resq_cognition' / 'config' / 'prioritizer.yaml'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
R = ComputePathToPose.Result
NEAR, FAR = ('near-safe', 2.0, 0.0), ('far-endangered', 5.0, 0.0)          # the fire is at (6, 0): FAR is 1 m from it
DETECTED, TARGETED, RESCUED = VictimState.DETECTED, VictimState.TARGETED, VictimState.RESCUED


def straight(a, b):
    return ('ok', math.hypot(a[0] - b[0], a[1] - b[1]))


class FakePlanner(Node):
    """Nav2's ComputePathToPose, answering from `self.rule(start, goal)` -> ('ok', length) | ('code', error_code)."""

    def __init__(self):
        super().__init__('fake_planner')
        self.rule, self.calls = straight, []
        self.server = ActionServer(self, ComputePathToPose, '/compute_path_to_pose', self._execute)

    def _execute(self, goal_handle):
        g = goal_handle.request
        start = (g.start.pose.position.x, g.start.pose.position.y)
        goal = (g.goal.pose.position.x, g.goal.pose.position.y)
        self.calls.append((start, goal, g.use_start, g.goal.header.frame_id))
        kind, val = self.rule(start, goal)
        res = ComputePathToPose.Result()
        if kind == 'ok':
            n = 5
            for i in range(n + 1):                                          # a polyline of the requested length along the line
                p = g.goal.__class__()
                f = i / n
                p.pose.position.x = start[0] + (goal[0] - start[0]) * f
                p.pose.position.y = start[1] + (goal[1] - start[1]) * f
                res.path.poses.append(p)
            scale = val / max(math.hypot(goal[0] - start[0], goal[1] - start[1]), 1e-9)
            for p in res.path.poses:                                        # stretch it so the polyline has length `val` (a detour)
                p.pose.position.x = start[0] + (p.pose.position.x - start[0]) * scale
                p.pose.position.y = start[1] + (p.pose.position.y - start[1]) * scale
            goal_handle.succeed()
        else:
            res.error_code = int(val)
            goal_handle.abort()
        return res


def world_state(victims, fire=(6.0, 0.0), robot=(0.0, 0.0), stamp=None):
    ws = WorldState()
    ws.header.frame_id = 'map'
    if stamp is not None:
        ws.header.stamp = stamp
    for vid, x, y, *rest in victims:
        v = VictimState()
        v.id, v.confidence, v.status = vid, 0.9, DETECTED
        v.position.x, v.position.y = float(x), float(y)
        if rest:
            v.status = rest[0]
        ws.victims.append(v)
    ws.fire_known = fire is not None
    if fire:
        ws.fire_position.x, ws.fire_position.y = fire
    ws.robot_pose.position.x, ws.robot_pose.position.y = robot
    ws.robot_pose.orientation.w = 1.0
    ws.safe_zone_pose.orientation.w = 1.0
    return ws


class Rig(Node):
    def __init__(self):
        super().__init__('cognition_test_rig')
        self.world = None
        self.pub = self.create_publisher(WorldState, '/fire_resq/world_state', 10)
        self.create_timer(0.2, self._send)

    def _send(self):
        if self.world is not None:
            self.world.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.world)


class Sink(Node):
    def __init__(self):
        super().__init__('cognition_test_sink')
        self.targets, self.reports = [], []
        self.create_subscription(RescueTarget, '/fire_resq/rescue_target', self.targets.append, LATCHED)
        self.create_subscription(String, '/fire_resq/decision_report', lambda m: self.reports.append(json.loads(m.data)), LATCHED)
        self.client = self.create_client(SelectTarget, '/fire_resq/select_target')

    def select(self, timeout=8.0):
        fut = self.client.call_async(SelectTarget.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        assert fut.done(), 'SelectTarget timed out'
        return fut.result()


def flatten(d, prefix=''):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f'{prefix}{k}.')
        else:
            yield f'{prefix}{k}', v


@contextmanager
def running(world=None, planner=True, rule=None, **overrides):
    params = yaml.safe_load(CFG.read_text())['prioritizer_node']['ros__parameters']
    params.update({'reassess_period_s': 0.3, **overrides})
    if not rclpy.ok():
        rclpy.init()
    node = PrioritizerNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten(params)])
    rig, sink = Rig(), Sink()
    rig.world = world
    fake = FakePlanner() if planner else None
    if fake and rule:
        fake.rule = rule
    nodes = [n for n in (node, rig, sink, fake) if n is not None]
    ex = MultiThreadedExecutor(num_threads=6)
    for n in nodes:
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        yield node, rig, sink, fake
    finally:
        ex.shutdown()
        th.join(timeout=3)
        for n in nodes:
            n.destroy_node()


def wait(cond, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


# ------------------------------------------------------------------------------ the decision as published
def test_the_world_model_state_becomes_a_rescue_target_with_its_explanation():
    with running(world_state([NEAR, FAR])) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.reports)
        t, rep = sink.targets[-1], sink.reports[-1]
        assert t.victim_id == 'far-endangered' and t.header.frame_id == 'map'
        b = t.breakdown
        assert b.distance_to_fire == pytest.approx(1.0) and b.fire_risk == pytest.approx(1.0)
        assert b.robot_distance == pytest.approx(4.6) and b.rescue_cost == pytest.approx(9.2)     # out 4.6 m + back 4.6 m
        assert b.accessibility == pytest.approx(1.0) and b.perception_confidence == pytest.approx(0.9)
        assert list(b.weight_names) == ['risk', 'fire_proximity', 'reach', 'accessibility', 'confidence', 'cost']
        assert list(b.weights) == pytest.approx([3.0, 1.0, 1.0, 0.5, 0.5, 1.0]) and b.total == pytest.approx(t.utility)
        assert 'far-endangered' in t.rationale and 'near-safe' in t.rationale
        assert rep['model'] == 'weighted_utility' and rep['selected'] == 'far-endangered'
        assert [c['victim_id'] for c in rep['candidates']] == ['far-endangered', 'near-safe']
        assert sum(rep['candidates'][0]['contributions'].values()) == pytest.approx(rep['candidates'][0]['utility'])
        w = rep['world']                                                               # the exact input, for replaying the decision
        assert [v['id'] for v in w['victims']] == ['near-safe', 'far-endangered'] and w['fire'] == [6.0, 0.0] and w['robot'] == [0.0, 0.0]
        assert w['fire_known'] and w['safe_zone'] == [0.0, 0.0] and rep['why']


def test_the_provisional_approach_pose_is_the_queried_point_facing_the_victim():
    with running(world_state([FAR])) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets)
        p = sink.targets[-1].approach_pose
        assert (p.pose.position.x, p.pose.position.y) == pytest.approx((4.6, 0.0))            # 0.4 m short of (5, 0)
        assert 2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w) == pytest.approx(0.0, abs=1e-6)


def test_the_target_pose_faces_the_victim_from_any_bearing():
    with running(world_state([('east', 0.0, 3.0)], fire=None)) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.targets[-1].victim_id == 'east')
        p = sink.targets[-1].approach_pose.pose
        assert (p.position.x, p.position.y) == pytest.approx((0.0, 2.6), abs=1e-6)                     # 0.4 m short, on the line from the robot
        assert 2 * math.atan2(p.orientation.z, p.orientation.w) == pytest.approx(math.pi / 2, abs=1e-6)  # facing +y, at the victim


def test_select_target_right_after_an_automatic_decision_costs_the_planner_nothing():
    with running(world_state([NEAR, FAR])) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and len(fake.calls) >= 4)
        time.sleep(0.5)
        n = len(fake.calls)
        r = sink.select()
        assert r.found and r.target.victim_id == 'far-endangered'
        assert len(fake.calls) == n, 'SelectTarget on an unchanged world asked the planner again'
        assert node.path_query.hits >= 4


def test_a_periodic_refresh_bypasses_the_cache():
    """The cache must not hide a costmap change: a refresh (or retry) asks the planner afresh."""
    with running(world_state([NEAR, FAR]), refresh_period_s=1.0) as (node, rig, sink, fake):
        assert wait(lambda: len(fake.calls) >= 4)
        n = len(fake.calls)
        assert wait(lambda: len(fake.calls) >= n + 4, timeout=8.0), 'the refresh did not re-ask the planner'


def test_the_planner_is_asked_in_the_world_frame_with_an_explicit_start_out_and_back():
    with running(world_state([FAR])) as (node, rig, sink, fake):
        assert wait(lambda: len(fake.calls) >= 2)
        (s1, g1, use1, f1), (s2, g2, use2, f2) = fake.calls[0], fake.calls[1]
        assert s1 == pytest.approx((0.0, 0.0)) and g1 == pytest.approx((4.6, 0.0))            # robot -> approach point
        assert s2 == pytest.approx((4.6, 0.0)) and g2 == pytest.approx((0.0, 0.0))            # approach point -> safe zone
        assert use1 and use2 and f1 == f2 == 'map'


def test_the_nearest_model_can_be_selected_by_parameter_and_picks_the_other_victim():
    with running(world_state([NEAR, FAR]), decision_model='nearest') as (node, rig, sink, fake):
        assert wait(lambda: sink.targets)
        assert sink.targets[-1].victim_id == 'near-safe' and sink.reports[-1]['model'] == 'nearest'


def test_weights_are_parameters_not_literals():
    with running(world_state([NEAR, FAR]), **{'weights.risk': 0.0, 'weights.fire_proximity': 0.0}) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets)
        assert sink.targets[-1].victim_id == 'near-safe'
        assert list(sink.targets[-1].breakdown.weights)[:2] == [0.0, 0.0]


# ------------------------------------------------------------------------------ reassessment
def test_the_node_reassesses_when_the_world_model_changes():
    with running(world_state([NEAR, FAR])) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.targets[-1].victim_id == 'far-endangered')
        rig.world = world_state([NEAR, FAR[:3] and ('far-endangered', 5.0, 0.0, RESCUED)])     # the rescue side finished it
        assert wait(lambda: sink.targets[-1].victim_id == 'near-safe'), 'no re-decision after the world changed'
        assert wait(lambda: 'already rescued' in json.dumps(sink.reports[-1]))
        rig.world = world_state([])                                                            # nothing left
        assert wait(lambda: sink.targets[-1].victim_id == '' and 'no victims' in sink.targets[-1].rationale)


def test_a_newly_found_victim_can_change_the_decision():
    with running(world_state([NEAR])) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.targets[-1].victim_id == 'near-safe')
        rig.world = world_state([NEAR, FAR])
        assert wait(lambda: sink.targets[-1].victim_id == 'far-endangered')


def test_a_static_world_does_not_spam_decisions_but_a_moving_robot_re_asks_the_planner():
    with running(world_state([NEAR, FAR])) as (node, rig, sink, fake):
        assert wait(lambda: node.decisions >= 1)
        n = node.decisions
        time.sleep(3.0)
        assert node.decisions == n, 'an unchanged world must not trigger new decisions'
        rig.world = world_state([NEAR, FAR], robot=(1.5, 0.0))                                  # moved 1.5 m > requery distance
        assert wait(lambda: node.decisions > n), 'the paths changed but the node did not re-decide'


def test_jitter_across_a_rounding_boundary_does_not_retrigger_decisions():
    """Change detection must be by DISTANCE moved, not by rounding positions: a victim resting near a rounding boundary and
    jittering by a centimetre (as an EMA of noisy detections does) would otherwise look 'changed' on every tick."""
    with running(world_state([('near-safe', 0.074, 0.0), FAR])) as (node, rig, sink, fake):
        assert wait(lambda: node.decisions >= 1)
        n = node.decisions
        flip = [0]

        def jitter():
            flip[0] ^= 1
            rig.world = world_state([('near-safe', 0.074 + 0.002 * flip[0], 0.0), FAR])     # straddles 0.075 = 0.5 x 0.15
        rig.create_timer(0.25, jitter)
        time.sleep(4.0)
        assert node.decisions == n, f'{node.decisions - n} spurious decisions from 2 mm of jitter'


def test_with_auto_reassess_off_only_select_target_decides():
    with running(world_state([NEAR, FAR]), auto_reassess=False) as (node, rig, sink, fake):
        assert wait(lambda: node.world_msg is not None)
        time.sleep(1.0)
        assert node.decisions == 0 and sink.targets == []
        r = sink.select()
        assert r.found and r.target.victim_id == 'far-endangered' and node.decisions == 1


def test_a_decision_made_while_nav2_is_still_starting_is_retried_with_no_world_change():
    """Seen live: the first decision came before Nav2's costmap was up ("goal outside the map"), the world then stayed
    unchanged, and the node kept its empty answer. An incomplete decision must be retried on its own."""
    calls = {'n': 0}

    def rule(a, b):
        calls['n'] += 1
        return ('code', R.NO_VALID_PATH) if calls['n'] <= 2 else straight(a, b)        # the first decision's two queries fail
    with running(world_state([NEAR, FAR]), rule=rule, retry_period_s=0.6) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.targets[0].victim_id == '', timeout=6.0), 'the first (failed) decision should be published'
        assert wait(lambda: sink.targets[-1].victim_id == 'far-endangered', timeout=8.0), 'the node never retried'
        assert node.decisions >= 2


def test_the_paths_are_refreshed_periodically_so_they_follow_a_changing_costmap():
    state = {'blocked': False}

    def rule(a, b):
        return ('code', R.NO_VALID_PATH) if state['blocked'] and b[0] > 3.5 else straight(a, b)
    with running(world_state([NEAR, FAR]), rule=rule, refresh_period_s=1.0) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.targets[-1].victim_id == 'far-endangered')
        state['blocked'] = True                                                        # a wall appears; the world MODEL is unchanged
        assert wait(lambda: sink.targets[-1].victim_id == 'near-safe', timeout=8.0), 'the refresh never re-asked the planner'


# ------------------------------------------------------------------------------ SelectTarget
def test_select_target_answers_from_the_current_world_and_publishes_the_same_decision():
    with running(world_state([NEAR, FAR]), auto_reassess=False) as (node, rig, sink, fake):
        assert wait(lambda: node.world_msg is not None)
        r = sink.select()
        assert r.found and r.reason == '' and r.target.victim_id == 'far-endangered'
        assert wait(lambda: sink.targets and sink.targets[-1].victim_id == r.target.victim_id)


def test_select_target_before_any_world_state_says_so():
    with running(None, auto_reassess=False) as (node, rig, sink, fake):
        assert wait(lambda: sink.client.wait_for_service(0.1))
        r = sink.select()
        assert not r.found and 'no world state' in r.reason


# ------------------------------------------------------------------------------ the planner failing
def test_an_unreachable_candidate_is_excluded_and_reported_not_selected():
    def rule(a, b):
        return ('code', R.NO_VALID_PATH) if b[0] > 3.5 else straight(a, b)                     # walls off FAR's approach
    with running(world_state([NEAR, FAR]), rule=rule) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets and sink.reports)
        assert sink.targets[-1].victim_id == 'near-safe'
        ex = [c for c in sink.reports[-1]['candidates'] if not c['eligible']]
        assert [c['victim_id'] for c in ex] == ['far-endangered'] and 'no valid path' in ex[0]['exclusion']
        assert 'far-endangered' in sink.targets[-1].rationale


def test_when_no_candidate_is_reachable_the_target_is_empty_and_says_why():
    with running(world_state([NEAR, FAR]), rule=lambda a, b: ('code', R.NO_VALID_PATH)) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets)
        t = sink.targets[-1]
        assert t.victim_id == '' and 'no valid path' in t.rationale
        assert not sink.select().found


def test_a_missing_planner_is_unknown_never_reachable():
    with running(world_state([NEAR, FAR]), planner=False, path_query_timeout_s=1.0) as (node, rig, sink, fake):
        assert wait(lambda: sink.targets, timeout=10.0)
        t = sink.targets[-1]
        # Either "not available" or, when DDS still remembers a previous planner for a moment, "did not accept in time":
        # both are `unknown`, which is excluded - the safety property, not the wording, is what matters.
        assert t.victim_id == '' and 'not assumed reachable' in t.rationale and 'planner' in t.rationale


def test_a_planner_error_code_other_than_no_path_is_unknown_not_unreachable():
    with running(world_state([NEAR]), rule=lambda a, b: ('code', R.TIMEOUT)) as (node, rig, sink, fake):
        assert wait(lambda: sink.reports)
        c = sink.reports[-1]['candidates'][0]
        assert not c['eligible'] and 'not assumed reachable' in c['exclusion'] and 'timeout' in c['exclusion']


def test_a_missing_planner_is_noticed_once_not_once_per_question():
    """Waiting half a second for a server that is not there, for each of 2 x N questions, would stall a decision for seconds."""
    if not rclpy.ok():
        rclpy.init()
    q = Nav2PathQuery('/no_such_planner', 'map', '', 1.0)
    try:
        t0 = time.time()
        first = q((0.0, 0.0), (1.0, 0.0))
        slow = time.time() - t0
        t1 = time.time()
        rest = [q((0.0, 0.0), (float(i), 0.0)) for i in range(1, 7)]
        fast = time.time() - t1
        assert first.status == 'unknown' and all(r.status == 'unknown' and 'not available' in r.reason for r in rest)
        assert slow >= 0.4 and fast < 0.2, f'first {slow:.2f}s, the next six {fast:.2f}s'
    finally:
        q.destroy()


# ------------------------------------------------------------------------------ the adapter's classification, on its own
def _res(status, code=0, n=0, err=''):
    poses = [SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=float(i), y=0.0))) for i in range(n)]
    return SimpleNamespace(status=status, result=SimpleNamespace(error_code=code, error_msg=err, path=SimpleNamespace(poses=poses)))


def test_the_adapter_measures_the_path_length_and_classifies_the_planners_verdicts():
    ok = Nav2PathQuery._classify(_res(GoalStatus.STATUS_SUCCEEDED, n=6), goal=(5.0, 0.0))
    assert ok.status == 'reachable' and ok.length_m == pytest.approx(5.0)
    for code, expect in ((R.NO_VALID_PATH, 'unreachable'), (R.GOAL_OCCUPIED, 'unreachable'), (R.GOAL_OUTSIDE_MAP, 'unreachable'),
                         (R.TIMEOUT, 'unknown'), (R.TF_ERROR, 'unknown'), (R.START_OCCUPIED, 'unknown'), (R.UNKNOWN, 'unknown')):
        assert Nav2PathQuery._classify(_res(GoalStatus.STATUS_ABORTED, code)).status == expect, code
    assert Nav2PathQuery._classify(_res(GoalStatus.STATUS_SUCCEEDED, n=0)).status == 'unreachable'      # success with no path


def test_a_path_that_stops_short_of_the_goal_is_not_reachable():
    """NavFn's 0.2 m tolerance turns "the goal is inside an obstacle" into a path to the nearest free cell reported as success.
    Measured live: the centre of the 15 cm partition came back reachable. The adapter checks the path actually ENDS at the goal."""
    path = _res(GoalStatus.STATUS_SUCCEEDED, n=6)                                            # ends at (5, 0)
    assert Nav2PathQuery._classify(path, goal=(5.0, 0.0)).status == 'reachable'
    assert Nav2PathQuery._classify(path, goal=(5.04, 0.0)).status == 'reachable'            # within one 5 cm cell
    short = Nav2PathQuery._classify(path, goal=(5.15, 0.0))
    assert short.status == 'unreachable' and 'short' in short.reason and '15 cm' in short.reason
    assert Nav2PathQuery._classify(path, goal=(5.15, 0.0), end_tolerance_m=0.2).status == 'reachable'   # the tolerance is a parameter
    assert Nav2PathQuery._classify(_res(GoalStatus.STATUS_CANCELED, 0)).status == 'unknown'


# ------------------------------------------------------------------------------ start-up
def test_bad_parameters_are_a_clear_startup_error():
    if not rclpy.ok():
        rclpy.init()
    base = yaml.safe_load(CFG.read_text())['prioritizer_node']['ros__parameters']
    with pytest.raises(ValueError, match='decision_model'):
        PrioritizerNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten({**base, 'decision_model': 'lucky_dip'})])
    with pytest.raises(ValueError, match='non-negative'):
        PrioritizerNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten({**base, 'weights.risk': -1.0})])
