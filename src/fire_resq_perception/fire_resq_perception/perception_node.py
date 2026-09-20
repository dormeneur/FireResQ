"""ROS 2 node: RGB (+ depth) camera stream -> /fire_resq/detections.

Thin glue around PerceptionPipeline. It knows ROS message types, TF and topic names; it does NOT
know Gazebo, a specific camera, or where anything is in the world (no scenario, no ground truth).
The same node runs unchanged on the simulated robot and on hardware.

Per RGB frame:  detect -> (only if the camera is steady) place -> transform to target_frame -> publish.
The 2D detection is always published; `position_valid` says whether a position could be trusted.
"""
from __future__ import annotations

import time
from collections import deque
from typing import Optional

import cv2
import numpy as np
import rclpy
from fire_resq_interfaces.msg import Detection, DetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .config import classes_from_dict
from .detectors import ColorBlobDetector
from .image_utils import to_depth_metres, to_rgb
from .motion_gate import MotionGate
from .pipeline import PerceptionPipeline
from .spatial import DepthEstimator, KnownHeightEstimator
from .types import CameraModel, DepthFrame, Transform

_CLASS_FIELDS = {'hsv_lo': Parameter.Type.INTEGER_ARRAY, 'hsv_hi': Parameter.Type.INTEGER_ARRAY,
                 'min_area_px': Parameter.Type.INTEGER, 'area_ref_px': Parameter.Type.DOUBLE,
                 'top_height_m': Parameter.Type.DOUBLE, 'bottom_height_m': Parameter.Type.DOUBLE,
                 'axis_offset_m': Parameter.Type.DOUBLE, 'bottom_axis_offset_m': Parameter.Type.DOUBLE}


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class PerceptionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('perception_node', **kwargs)
        d = self.declare_parameter
        for name, val in (('rgb_topic', '/camera/image_raw'), ('camera_info_topic', '/camera/camera_info'),
                          ('depth_topic', '/camera/depth/image_raw'), ('detections_topic', '/fire_resq/detections'),
                          ('target_frame', 'map'), ('base_frame', 'base_link'), ('odom_frame', 'odom'),
                          ('spatial_backend', 'auto'), ('known_height_anchor', 'top')):
            d(name, val)
        for name, val in (('depth_pair_max_dt', 0.1), ('max_angular_rate', 0.1), ('max_linear_speed', 0.3),
                          ('gate_window_s', 0.3), ('min_confidence', 0.2)):
            d(name, val)
        d('floor_offset_m', Parameter.Type.DOUBLE)                 # the wheel radius: supplied by the launch file
        d('class_names', Parameter.Type.STRING_ARRAY)
        names = self._require('class_names')
        for n in names:
            for field, typ in _CLASS_FIELDS.items():
                d(f'classes.{n}.{field}', typ)
        classes = {n: {f: self._get(f'classes.{n}.{f}') for f in _CLASS_FIELDS
                       if self._get(f'classes.{n}.{f}') is not None} for n in names}
        configs = classes_from_dict(list(names), classes)
        self.classes = {c.name: c for c in configs}

        self.pipeline = PerceptionPipeline(
            ColorBlobDetector(configs), self.classes, DepthEstimator(),
            KnownHeightEstimator(self._require('floor_offset_m'), self._get('known_height_anchor')),
            self._get('spatial_backend'), self._get('min_confidence'))
        self.gate = MotionGate(self._get('max_angular_rate'), self._get('max_linear_speed'), self._get('gate_window_s'))
        self.target, self.base, self.odom = self._get('target_frame'), self._get('base_frame'), self._get('odom_frame')
        self.pair_dt = self._get('depth_pair_max_dt')

        self.cam: Optional[CameraModel] = None
        self.depth_msgs: deque = deque(maxlen=8)
        self.tf_buffer = Buffer()
        # The listener gets its OWN node and thread, so a lookup that waits inside the image callback cannot starve the
        # TF subscription that would satisfy it, and this node stays on a plain single-threaded executor (measured:
        # a MultiThreadedExecutor spent >1 core in wait-set bookkeeping for the same work).
        self.tf_listener = TransformListener(self.tf_buffer, node=None, spin_thread=True)

        self.pub = self.create_publisher(DetectionArray, self._get('detections_topic'), 10)
        self.debug_pub = self.create_publisher(Image, '/fire_resq/perception/debug_image', 1)
        self._info_sub = self.create_subscription(CameraInfo, self._get('camera_info_topic'), self._on_info,
                                                  qos_profile_sensor_data)
        self.create_subscription(Image, self._get('depth_topic'), lambda m: self.depth_msgs.append(m), qos_profile_sensor_data)
        self.create_subscription(Image, self._get('rgb_topic'), self._on_rgb, qos_profile_sensor_data)
        self._frames, self._busy_s = 0, 0.0
        self.get_logger().info(f"perception ready: classes {list(names)}, backend '{self._get('spatial_backend')}', "
                               f"positions in '{self.target}'")

    # -- parameters
    def _get(self, name):
        try:
            return self.get_parameter(name).value
        except Exception:
            return None

    def _require(self, name):
        v = self._get(name)
        if v is None:
            raise RuntimeError(f"required parameter '{name}' is not set (see config/perception.yaml)")
        return v

    # -- callbacks
    def _on_info(self, m: CameraInfo) -> None:
        if m.width and m.k[0]:
            self.cam = CameraModel.from_k(m.k, m.width, m.height)
            if self._info_sub is not None:        # intrinsics are fixed: stop paying for the 25 Hz stream
                self.destroy_subscription(self._info_sub)
                self._info_sub = None

    def _lookup(self, target: str, source: str, stamp) -> Optional[Transform]:
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        q, p = t.transform.rotation, t.transform.translation
        return Transform.from_quaternion(q.x, q.y, q.z, q.w, (p.x, p.y, p.z))

    def _pair_depth(self, stamp) -> Optional[DepthFrame]:
        if not self.depth_msgs:
            return None
        t = stamp_s(stamp)
        best = min(self.depth_msgs, key=lambda m: abs(stamp_s(m.header.stamp) - t))
        if abs(stamp_s(best.header.stamp) - t) > self.pair_dt:
            return None
        try:
            return DepthFrame(to_depth_metres(best), stamp_s(best.header.stamp))
        except ValueError as e:
            self.get_logger().warn(f'unusable depth image: {e}', throttle_duration_sec=10.0)
            return None

    def _on_rgb(self, msg: Image) -> None:
        if self.cam is None:
            return
        t0 = time.perf_counter()
        try:
            rgb = to_rgb(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        cam_to_base = self._lookup(self.base, msg.header.frame_id, msg.header.stamp)
        odom_to_base = self._lookup(self.odom, self.base, msg.header.stamp)
        if odom_to_base is not None:
            self.gate.update(stamp_s(msg.header.stamp), odom_to_base.yaw, odom_to_base.t[0], odom_to_base.t[1])
        motion_ok = odom_to_base is not None and self.gate.allows()
        depth = self._pair_depth(msg.header.stamp) if motion_ok else None
        results = self.pipeline.process(rgb, self.cam, cam_to_base, depth, motion_ok)

        base_to_target = self._lookup(self.target, self.base, msg.header.stamp) if any(r.valid for r in results) else None
        out = DetectionArray()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.target
        for r in results:
            m = Detection()
            m.header = out.header
            m.class_id, m.confidence, m.source_backend = r.det.class_id, float(r.det.confidence), r.backend
            m.bbox.center.position.x, m.bbox.center.position.y = r.det.x + r.det.w / 2.0, r.det.y + r.det.h / 2.0
            m.bbox.size_x, m.bbox.size_y = float(r.det.w), float(r.det.h)
            if r.valid and base_to_target is not None:
                p = base_to_target.apply(r.position_base)
                m.position.x, m.position.y, m.position.z = float(p[0]), float(p[1]), float(p[2])
                m.position_valid = True
            out.detections.append(m)
        self.pub.publish(out)
        if self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(msg, rgb, results)
        self._frames += 1
        self._busy_s += time.perf_counter() - t0

    def _publish_debug(self, msg: Image, rgb: np.ndarray, results) -> None:
        img = np.ascontiguousarray(rgb.copy())
        for r in results:
            d = r.det
            col = (0, 255, 0) if r.valid else (255, 160, 0)
            cv2.rectangle(img, (d.x, d.y), (d.x + d.w, d.y + d.h), col, 1)
            cv2.putText(img, f"{d.class_id} {d.confidence:.2f}" + (f" {r.range_m:.1f}m" if r.valid else ''),
                        (d.x, max(12, d.y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        out = Image()
        out.header, out.height, out.width, out.encoding = msg.header, img.shape[0], img.shape[1], 'rgb8'
        out.step, out.data = img.shape[1] * 3, img.tobytes()
        self.debug_pub.publish(out)

    def destroy_node(self):
        self.tf_listener = None                # its __del__ stops the listener thread and drops its subscriptions
        return super().destroy_node()

    @property
    def mean_ms(self) -> float:
        return 1000.0 * self._busy_s / max(1, self._frames)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'processed {node._frames} frames, {node.mean_ms:.1f} ms each on average')
        node.destroy_node()
        if rclpy.ok():                         # SIGINT has usually shut the context down already
            rclpy.shutdown()


if __name__ == '__main__':
    main()
