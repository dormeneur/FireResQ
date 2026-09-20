"""The perception NODE against synthetic ROS traffic - no Gazebo.

A fake rig publishes an RGB image, a registered depth image, CameraInfo and TF for a scene whose
true object positions are known; the real PerceptionNode runs in-process. This checks the glue
(topics, frames, stamps, message fields, backend choice, the timing-offset gate) that the pure
unit tests cannot: those check the maths, the simulation tests check the physics.
"""
import math
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not sourced')
pytest.importorskip('fire_resq_interfaces', reason='workspace not sourced: source install/setup.bash')

import perception_scene as S  # noqa: E402
from fire_resq_interfaces.msg import DetectionArray  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

from fire_resq_perception.perception_node import PerceptionNode  # noqa: E402

CFG = Path(__file__).resolve().parents[2] / 'src' / 'fire_resq_perception' / 'config' / 'perception.yaml'
CAM_XY = S.CAM_TO_BASE.t[:2]


def place(rng, bearing_deg=0.0):
    a = math.radians(bearing_deg)
    return CAM_XY[0] + rng * math.cos(a), CAM_XY[1] + rng * math.sin(a)


def flatten(d, prefix=''):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f'{prefix}{k}.')
        else:
            yield f'{prefix}{k}', v


def quat_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def tf(parent, child, t=(0., 0., 0.), q=(0., 0., 0., 1.), stamp=None):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    if stamp is not None:
        m.header.stamp = stamp
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
    return m


class Rig(Node):
    """Publishes a synthetic camera + TF. `rate` turns the robot (rad/s) as seen by TF."""

    def __init__(self, objs, depth=True, camera_tf=True):
        super().__init__('perception_test_rig')
        self.rate, self.t0, self.stamps = 0.0, time.time(), set()
        self.rgb, self.depth = S.render(objs)
        self.with_depth = depth
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.static = StaticTransformBroadcaster(self)
        statics = [tf('map', 'odom')]
        if camera_tf:
            statics.append(tf('base_link', 'camera_optical_frame', S.CAM_TO_BASE.t, quat_from_R(S.CAM_TO_BASE.R)))
        self.static.sendTransform(statics)
        self.dyn = TransformBroadcaster(self)
        self.yaw = 0.0
        self.create_timer(0.02, self._tf)
        self.create_timer(0.05, self._frame)

    def _tf(self):
        now = self.get_clock().now()
        self.yaw = self.rate * (time.time() - self.t0)
        self.dyn.sendTransform(tf('odom', 'base_link', q=(0., 0., math.sin(self.yaw / 2), math.cos(self.yaw / 2)),
                                  stamp=now.to_msg()))

    def _frame(self):
        stamp = self.get_clock().now().to_msg()
        self.stamps.add((stamp.sec, stamp.nanosec))
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, 'camera_optical_frame'
        img.height, img.width, img.encoding, img.step, img.data = S.H, S.W, 'rgb8', S.W * 3, self.rgb.tobytes()
        self.image_pub.publish(img)
        info = CameraInfo()
        info.header, info.width, info.height = img.header, S.W, S.H
        info.k = [S.CAM.fx, 0., S.CAM.cx, 0., S.CAM.fy, S.CAM.cy, 0., 0., 1.]
        self.info_pub.publish(info)
        if self.with_depth:
            d = Image()
            d.header, d.height, d.width, d.encoding, d.step = img.header, S.H, S.W, '32FC1', S.W * 4
            d.data = self.depth.tobytes()
            self.depth_pub.publish(d)


class Sink(Node):
    def __init__(self):
        super().__init__('perception_test_sink')
        self.msgs = []
        self.create_subscription(DetectionArray, '/fire_resq/detections', self.msgs.append, 10)


@contextmanager
def running(objs, depth=True, camera_tf=True, **overrides):
    params = yaml.safe_load(CFG.read_text())['perception_node']['ros__parameters']
    params.update(overrides)
    params.setdefault('floor_offset_m', S.FLOOR_OFFSET)
    if not rclpy.ok():
        rclpy.init()
    ovr = [Parameter(k, value=v) for k, v in flatten(params)]
    node, rig, sink = PerceptionNode(parameter_overrides=ovr), Rig(objs, depth, camera_tf), Sink()
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


def settle(sink, seconds):
    n0 = len(sink.msgs)
    time.sleep(seconds)
    return sink.msgs[n0:]


def by_class(msg):
    return {d.class_id: d for d in msg.detections}


SCENE = [S.victim_at(*place(2.0, 10)), S.fire_at(*place(3.0, -15))]


def test_positions_are_published_in_the_target_frame_and_match_the_truth_with_depth():
    with running(SCENE) as (node, rig, sink):
        msgs = settle(sink, 3.0)
        assert msgs, 'no detections published'
        last = by_class(msgs[-1])
        assert set(last) == {'victim', 'fire'}
        for o in SCENE:
            d = last[o.cls]
            assert d.position_valid and d.source_backend == 'depth'
            assert np.hypot(d.position.x - o.x, d.position.y - o.y) <= 0.05, (o.cls, d.position)
        assert msgs[-1].header.frame_id == 'map'


def test_without_depth_the_same_node_falls_back_to_rgb_only_and_publishes_the_same_message_type():
    with running(SCENE, depth=False) as (node, rig, sink):
        last = by_class(settle(sink, 3.0)[-1])
        for o, cfg in zip(SCENE, (S.VICTIM, S.FIRE)):
            d = last[o.cls]
            rng = math.hypot(o.x - CAM_XY[0], o.y - CAM_XY[1])
            dz = (-S.FLOOR_OFFSET + cfg.top_height_m) - S.CAM_TO_BASE.t[2]
            assert d.position_valid and d.source_backend == 'known_height'
            assert np.hypot(d.position.x - o.x, d.position.y - o.y) <= 1.5 * rng ** 2 / (S.FX * dz) + 0.02


def test_message_fields_bbox_confidence_stamp():
    with running(SCENE) as (node, rig, sink):
        msgs = settle(sink, 2.5)
        m = msgs[-1]
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        assert stamp in rig.stamps, 'a detection carries a time that is not an image stamp'
        v = by_class(m)['victim']
        assert 0.2 < v.confidence <= 1.0 and v.bbox.size_x > 5 and v.bbox.size_y > 20
        assert 0 < v.bbox.center.position.x < S.W and 0 < v.bbox.center.position.y < S.H
        assert v.header.frame_id == 'map' and v.class_id == 'victim'


def test_positions_are_withheld_while_the_camera_turns_and_return_after_it_stops():
    """The measured RGB/depth timing offset in action: 2D detections keep flowing, positions do not."""
    with running(SCENE) as (node, rig, sink):
        settle(sink, 1.5)
        rig.rate = 0.4
        turning = settle(sink, 2.0)[5:]                       # skip the transition
        assert turning and all(d.class_id in ('victim', 'fire') for m in turning for d in m.detections)
        assert not any(d.position_valid for m in turning for d in m.detections), 'a position was published while turning'
        rig.rate = 0.0
        rig.t0 = time.time()
        settle(sink, 1.5)
        assert all(d.position_valid for d in by_class(sink.msgs[-1]).values()), 'positions did not resume'


def test_slow_drift_is_allowed():
    with running(SCENE) as (node, rig, sink):
        rig.rate = 0.05
        last = settle(sink, 3.0)[-1]
        assert all(d.position_valid for d in last.detections)


def test_no_camera_transform_means_2d_detections_but_no_positions():
    with running(SCENE, camera_tf=False) as (node, rig, sink):
        msgs = settle(sink, 2.5)
        assert msgs and len(msgs[-1].detections) == 2
        assert not any(d.position_valid for m in msgs for d in m.detections)


def test_a_strict_depth_backend_never_falls_back():
    with running(SCENE, depth=False, spatial_backend='depth') as (node, rig, sink):
        msgs = settle(sink, 2.5)
        assert msgs and not any(d.position_valid for m in msgs for d in m.detections)


def test_a_missing_required_setting_is_a_clear_startup_error():
    params = yaml.safe_load(CFG.read_text())['perception_node']['ros__parameters']
    del params['classes']['victim']['hsv_hi']
    if not rclpy.ok():
        rclpy.init()
    with pytest.raises(Exception, match='hsv_hi'):
        PerceptionNode(parameter_overrides=[Parameter(k, value=v) for k, v in flatten({**params, 'floor_offset_m': 0.03})])
