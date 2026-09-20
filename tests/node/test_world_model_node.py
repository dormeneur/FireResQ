"""The world-model NODE against synthetic ROS traffic - no Gazebo.

A rig publishes DetectionArray messages and TF (a `map -> odom` that is offset AND rotated, so a missing or wrong
transform shows up as wrong numbers); the real WorldModelNode runs in-process. This checks the glue: frames, stamps,
message fields, the status service, markers. The rules themselves are pinned by tests/unit/test_world_model_lib.py.
"""
import math
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not sourced')
pytest.importorskip('fire_resq_interfaces', reason='workspace not sourced: source install/setup.bash')

from fire_resq_interfaces.msg import Detection, DetectionArray, VictimState, WorldState  # noqa: E402
from fire_resq_interfaces.srv import UpdateVictimStatus  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

from fire_resq_world_model.world_model_node import WorldModelNode  # noqa: E402

CFG = Path(__file__).resolve().parents[2] / 'src' / 'fire_resq_world_model' / 'config' / 'world_model.yaml'
MAP_ODOM = (1.0, 2.0, math.pi / 2)            # map <- odom: offset (1, 2), rotated 90 degrees


def to_map(x, y):
    """Where a point at (x, y) in odom is in map: rotate 90 deg CCW, then offset."""
    return 1.0 - y, 2.0 + x


def tf(parent, child, xy=(0.0, 0.0), yaw=0.0, stamp=None):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    if stamp is not None:
        m.header.stamp = stamp
    m.transform.translation.x, m.transform.translation.y = xy
    m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


class Rig(Node):
    """Publishes TF and whatever detections `objects` currently holds: (class, x, y, confidence, valid, frame)."""

    def __init__(self, map_odom=True):
        super().__init__('world_model_test_rig')
        self.objects = []
        self.publishing = True
        self.pub = self.create_publisher(DetectionArray, '/fire_resq/detections', 10)
        self.static = StaticTransformBroadcaster(self)
        if map_odom:
            self.static.sendTransform([tf('map', 'odom', MAP_ODOM[:2], MAP_ODOM[2])])
        self.dyn = TransformBroadcaster(self)
        self.create_timer(0.02, lambda: self.dyn.sendTransform(tf('odom', 'base_link', (0.5, 0.0), 0.0, self.get_clock().now().to_msg())))
        self.create_timer(0.1, self._send)

    def _send(self):
        if not self.publishing:
            return
        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for cls, x, y, conf, valid, frame in self.objects:
            d = Detection()
            d.header.stamp, d.header.frame_id = msg.header.stamp, frame
            d.class_id, d.confidence, d.position_valid = cls, float(conf), bool(valid)
            d.position.x, d.position.y, d.position.z = float(x), float(y), 0.13
            msg.detections.append(d)
        self.pub.publish(msg)


class Sink(Node):
    def __init__(self):
        super().__init__('world_model_test_sink')
        self.states, self.markers = [], []
        self.create_subscription(WorldState, '/fire_resq/world_state', self.states.append, 10)
        self.create_subscription(MarkerArray, '/fire_resq/world_markers', self.markers.append, 10)
        self.client = self.create_client(UpdateVictimStatus, '/fire_resq/update_victim_status')

    def request(self, victim_id, status, timeout=3.0):
        req = UpdateVictimStatus.Request()
        req.victim_id, req.new_status = victim_id, status
        fut = self.client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        assert fut.done(), 'service call timed out'
        return fut.result()


def flatten(d, prefix=''):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f'{prefix}{k}.')
        else:
            yield f'{prefix}{k}', v


@contextmanager
def running(objects=(), map_odom=True, **overrides):
    params = yaml.safe_load(CFG.read_text())['world_model_node']['ros__parameters']
    params.update(overrides)
    if not rclpy.ok():
        rclpy.init()
    node = WorldModelNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten(params)])
    rig, sink = Rig(map_odom), Sink()
    rig.objects = list(objects)
    ex = MultiThreadedExecutor(num_threads=4)
    for n in (node, rig, sink):
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        yield node, rig, sink
    finally:
        ex.shutdown()
        th.join(timeout=3)
        for n in (node, rig, sink):
            n.destroy_node()


def wait(cond, timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def latest(sink):
    return sink.states[-1]


V1, V2, V3 = (0.5, 0.0), (1.0, -1.0), (2.0, 1.0)        # odom positions, mutually > 0.3 m apart


def victims(*pts, c=0.9):
    return [('victim', x, y, c, True, 'odom') for x, y in pts]


# ------------------------------------------------------------------------------ frames and fields
def test_detections_in_odom_are_published_in_map_through_the_transform():
    with running(victims(V1, V2) + [('fire', 2.0, 0.5, 0.9, True, 'odom')]) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2 and latest(sink).fire_known)
        ws = latest(sink)
        assert ws.header.frame_id == 'map'
        for v, p in zip(ws.victims, (V1, V2)):
            ex, ey = to_map(*p)
            assert (v.position.x, v.position.y) == pytest.approx((ex, ey), abs=1e-6)
            assert v.status == VictimState.DETECTED and 0.85 < v.confidence <= 0.9
        assert (ws.fire_position.x, ws.fire_position.y) == pytest.approx(to_map(2.0, 0.5), abs=1e-6)
        assert ws.fire_confidence > 0.85


def test_detections_already_in_the_world_frame_are_used_as_they_are():
    with running([('victim', 3.0, 4.0, 0.9, True, 'map')]) as (node, rig, sink):
        assert wait(lambda: sink.states and latest(sink).victims and latest(sink).victims[0].status == VictimState.DETECTED)
        v = latest(sink).victims[0]
        assert (v.position.x, v.position.y) == pytest.approx((3.0, 4.0), abs=1e-6)


def test_robot_pose_comes_from_tf_and_the_safe_zone_from_the_configuration():
    with running(victims(V1)) as (node, rig, sink):
        assert wait(lambda: sink.states)
        ws = latest(sink)
        ex, ey = to_map(0.5, 0.0)                                         # base_link is at (0.5, 0) in odom
        assert (ws.robot_pose.position.x, ws.robot_pose.position.y) == pytest.approx((ex, ey), abs=1e-6)
        yaw = 2 * math.atan2(ws.robot_pose.orientation.z, ws.robot_pose.orientation.w)
        assert yaw == pytest.approx(MAP_ODOM[2], abs=1e-6)
        cfg = yaml.safe_load(CFG.read_text())['world_model_node']['ros__parameters']
        assert ws.safe_zone_pose.position.x == pytest.approx(cfg['safe_zone.x'])
        assert 2 * math.atan2(ws.safe_zone_pose.orientation.z, ws.safe_zone_pose.orientation.w) == pytest.approx(cfg['safe_zone.yaw'])


def test_last_seen_is_a_recent_time_and_stamps_advance():
    with running(victims(V1)) as (node, rig, sink):
        assert wait(lambda: sink.states and latest(sink).victims)
        s = latest(sink)
        now = node.get_clock().now().nanoseconds * 1e-9
        seen = s.victims[0].last_seen.sec + s.victims[0].last_seen.nanosec * 1e-9
        assert 0 <= now - seen < 2.0
        n0 = len(sink.states)
        assert wait(lambda: len(sink.states) > n0 + 3)
        a, b = sink.states[-4].header.stamp, sink.states[-1].header.stamp
        assert (b.sec, b.nanosec) > (a.sec, a.nanosec)


# ------------------------------------------------------------------------------ validity
def test_unplaced_detections_create_nothing():
    """`position_valid=false` (seen, not placed - e.g. camera turning, blob clipped) says nothing about WHERE."""
    objs = [('victim', 0.0, 0.0, 0.9, False, 'odom')] * 3
    with running(objs + [('fire', 1.0, 1.0, 0.9, False, 'odom')]) as (node, rig, sink):
        assert wait(lambda: sink.states)
        time.sleep(1.0)
        assert latest(sink).victims == [] and not latest(sink).fire_known
        assert node.counts.get('unplaced', 0) > 5


def test_low_confidence_placed_detections_are_ignored():
    with running(victims(V1, c=0.1)) as (node, rig, sink):
        assert wait(lambda: sink.states)
        time.sleep(0.8)
        assert latest(sink).victims == []


def test_without_a_transform_to_the_world_frame_nothing_is_claimed():
    """No map->odom: positions cannot be expressed in `map`, so the model publishes no belief in it."""
    with running(victims(V1), map_odom=False) as (node, rig, sink):
        time.sleep(1.5)
        assert sink.states == [], 'a WorldState was published without a way to localise it'


# ------------------------------------------------------------------------------ identity
def test_a_hidden_victim_appearing_later_is_a_new_entity_and_the_others_keep_their_ids():
    with running(victims(V1, V2)) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2)
        before = {v.id: (v.position.x, v.position.y) for v in latest(sink).victims}
        rig.objects = victims(V1, V2, V3)
        assert wait(lambda: len(latest(sink).victims) == 3 and latest(sink).victims[2].status == VictimState.DETECTED)
        after = {v.id: (v.position.x, v.position.y) for v in latest(sink).victims}
        assert set(before) < set(after) and len(after) == 3
        for i, p in before.items():
            assert after[i] == pytest.approx(p, abs=1e-6)


def test_a_victim_that_drops_out_of_view_and_returns_keeps_its_identity():
    with running(victims(V1, V2)) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2)
        ids = [v.id for v in latest(sink).victims]
        rig.objects = victims(V2)
        time.sleep(1.0)
        rig.objects = victims(V1, V2)
        time.sleep(1.0)
        assert [v.id for v in latest(sink).victims] == ids


def test_confidence_decays_when_a_victim_is_no_longer_seen_and_recovers_when_it_is():
    with running(victims(V1, V2), confidence_decay_tau_s=1.0) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2)
        rig.objects = victims(V2)                                          # V1 leaves view
        assert wait(lambda: latest(sink).victims[0].confidence < 0.4, timeout=6.0), 'no decay'
        assert latest(sink).victims[1].confidence > 0.8
        rig.objects = victims(V1, V2)
        assert wait(lambda: latest(sink).victims[0].confidence > 0.8), 'confidence did not recover'
        assert len(latest(sink).victims) == 2


# ------------------------------------------------------------------------------ status service
def test_the_status_service_drives_the_rescue_lifecycle_and_sets_the_target():
    with running(victims(V1, V2)) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2 and sink.client.wait_for_service(0.1))
        r = sink.request('V1', VictimState.TARGETED)
        assert r.success, r.message
        assert wait(lambda: latest(sink).current_target_id == 'V1')
        assert latest(sink).victims[0].status == VictimState.TARGETED
        r = sink.request('V2', VictimState.TARGETED)
        assert not r.success and 'one target at a time' in r.message
        assert sink.request('V1', VictimState.CARRIED).success
        assert sink.request('V1', VictimState.RESCUED).success
        assert wait(lambda: latest(sink).current_target_id == '' and latest(sink).victims[0].status == VictimState.RESCUED)


def test_rescued_stays_rescued_while_the_victim_is_still_being_detected():
    with running(victims(V1, V2)) as (node, rig, sink):
        assert wait(lambda: sink.states and len(latest(sink).victims) == 2 and sink.client.wait_for_service(0.1))
        for s in (VictimState.TARGETED, VictimState.CARRIED, VictimState.RESCUED):
            assert sink.request('V1', s).success
        time.sleep(1.5)                                                    # V1 is still "detected" where it was
        ws = latest(sink)
        assert len(ws.victims) == 2 and ws.victims[0].status == VictimState.RESCUED and ws.victims[1].status == VictimState.DETECTED
        r = sink.request('V1', VictimState.DETECTED)
        assert not r.success and 'terminal' in r.message


def test_a_bad_request_is_refused_not_crashed_on():
    with running(victims(V1)) as (node, rig, sink):
        assert wait(lambda: sink.states and sink.client.wait_for_service(0.1))
        assert not sink.request('nope', VictimState.TARGETED).success
        assert not sink.request('V1', 99).success


# ------------------------------------------------------------------------------ markers and start-up
def test_markers_describe_victims_fire_and_safe_zone_in_the_world_frame():
    with running(victims(V1, V2) + [('fire', 2.0, 0.5, 0.9, True, 'odom')]) as (node, rig, sink):
        assert wait(lambda: sink.markers and len({m.ns for m in sink.markers[-1].markers}) >= 5)
        ms = sink.markers[-1].markers
        assert {m.header.frame_id for m in ms} == {'map'}
        assert {m.ns for m in ms} == {'safe_zone', 'fire', 'fire_label', 'victim', 'victim_label'}
        assert len([m for m in ms if m.ns == 'victim']) == 2
        assert all(m.lifetime.sec + m.lifetime.nanosec > 0 for m in ms), 'markers must expire so removed entities disappear'
        assert any(m.text.startswith('V1 DETECTED') for m in ms)


def test_bad_parameters_are_a_clear_startup_error():
    params = yaml.safe_load(CFG.read_text())['world_model_node']['ros__parameters']
    params['position_alpha'] = 0.0
    if not rclpy.ok():
        rclpy.init()
    with pytest.raises(ValueError, match='position_alpha'):
        WorldModelNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten(params)])
