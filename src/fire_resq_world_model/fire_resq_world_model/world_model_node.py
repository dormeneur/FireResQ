"""ROS glue for the world model: /fire_resq/detections -> WorldModel -> /fire_resq/world_state.

Thin on purpose: the rules live in world_model.py (pure, unit-tested). This node
  - turns PLACED detections into world-frame Observations (transforming from the detection's frame via TF),
  - publishes WorldState (belief + robot pose from TF) and RViz markers at a fixed rate,
  - serves UpdateVictimStatus, the one way rescue code changes a victim's status.

It sees only DetectionArray, TF and parameters: camera-agnostic (it never looks at which backend produced a position), no Gazebo, no scenario.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import rclpy
from fire_resq_interfaces.msg import DetectionArray, VictimState, WorldState
from fire_resq_interfaces.srv import UpdateVictimStatus
from geometry_msgs.msg import Point, PointStamped, Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .config import WorldModelConfig
from .entities import Observation
from .markers import marker_specs
from .world_model import WorldModel

_PARAMS = {
    'victim_gate_m': 0.3, 'fire_gate_m': 0.5, 'position_alpha': 0.3, 'min_confidence': 0.2, 'confirm_hits': 2,
    'tentative_timeout_s': 5.0, 'confidence_decay_tau_s': 30.0,
    'safe_zone.x': 0.0, 'safe_zone.y': 0.0, 'safe_zone.yaw': 0.0, 'safe_zone.size_x': 1.0, 'safe_zone.size_y': 1.0,
}
_SHAPES = {'sphere': Marker.SPHERE, 'cylinder': Marker.CYLINDER, 'cube': Marker.CUBE, 'text': Marker.TEXT_VIEW_FACING}


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class WorldModelNode(Node):
    def __init__(self, **kwargs):
        super().__init__('world_model_node', **kwargs)
        for name, val in (('detections_topic', '/fire_resq/detections'), ('world_state_topic', '/fire_resq/world_state'),
                          ('markers_topic', '/fire_resq/world_markers'),
                          ('update_status_service', '/fire_resq/update_victim_status'),
                          ('world_frame', 'map'), ('base_frame', 'base_link'), ('publish_rate_hz', 5.0)):
            self.declare_parameter(name, val)
        for name, val in _PARAMS.items():
            self.declare_parameter(name, val)
        self.model = WorldModel(WorldModelConfig.from_dict({k: self.get_parameter(k).value for k in _PARAMS}))
        self.world = self.get_parameter('world_frame').value
        self.base = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        # Own node and thread (as in the perception node): a lookup that waits cannot starve the TF subscription.
        self.tf_listener = TransformListener(self.tf_buffer, node=None, spin_thread=True)

        self.state_pub = self.create_publisher(WorldState, self.get_parameter('world_state_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.create_subscription(DetectionArray, self.get_parameter('detections_topic').value, self._on_detections, 10)
        self.create_service(UpdateVictimStatus, self.get_parameter('update_status_service').value, self._on_status)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value), self._publish)
        self.counts: Dict[str, int] = {}
        self.robot_pose: Optional[Pose] = None
        self.get_logger().info(f"world model ready: positions in '{self.world}', victim gate {self.model.cfg.victim_gate_m} m, "
                               f"safe zone at ({self.model.cfg.safe_zone.x:.2f}, {self.model.cfg.safe_zone.y:.2f})")

    # ------------------------------------------------------------------------------ detections in
    def _count(self, key: str, n: int = 1) -> None:
        self.counts[key] = self.counts.get(key, 0) + n

    def _on_detections(self, msg: DetectionArray) -> None:
        stamp = stamp_s(msg.header.stamp)
        obs: List[Observation] = []
        cache: Dict[str, Optional[object]] = {}
        for d in msg.detections:
            if not d.position_valid:
                self._count('unplaced')                     # seen, not placed: nothing to associate (a TODO: negative/weak evidence)
                continue
            frame = d.header.frame_id or msg.header.frame_id
            if frame not in cache:
                cache[frame] = self._transform_to_world(frame, msg.header.stamp)
            tf = cache[frame]
            if tf is False:
                self._count('no transform')
                continue
            p = Point(x=d.position.x, y=d.position.y, z=d.position.z)
            if tf is not None:
                p = do_transform_point(PointStamped(point=p), tf).point
            obs.append(Observation(d.class_id, p.x, p.y, p.z, float(d.confidence)))
        report = self.model.observe(obs, stamp)
        for reason, n in report.ignored.items():
            self._count(f'ignored: {reason}', n)
        for i in report.created:
            self.get_logger().info(f'new {i}' if i != 'fire' else 'new fire track')
        for i in report.confirmed:
            self.get_logger().info(f'{i} confirmed')
        for i in report.expired:
            self.get_logger().info(f'{i} was never confirmed: dropped')

    def _transform_to_world(self, frame: str, stamp):
        """None = already in the world frame, False = no transform available, else the TransformStamped."""
        if frame == self.world:
            return None
        try:
            return self.tf_buffer.lookup_transform(self.world, frame, Time.from_msg(stamp), timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'cannot express detections from {frame!r} in {self.world!r}: {e}',
                                   throttle_duration_sec=5.0)
            return False

    # ------------------------------------------------------------------------------ status service
    def _on_status(self, req: UpdateVictimStatus.Request, res: UpdateVictimStatus.Response):
        res.success, res.message = self.model.set_status(req.victim_id, int(req.new_status))
        if res.success:
            self.get_logger().info(f'status request: {res.message}')
        else:
            self.get_logger().warn(f'status request refused: {res.message}')
        return res

    # ------------------------------------------------------------------------------ state out
    def _robot_pose(self) -> Optional[Pose]:
        try:
            t = self.tf_buffer.lookup_transform(self.world, self.base, Time())        # latest available
        except TransformException:
            return self.robot_pose                                                     # last known, else None
        p = Pose()
        p.position.x, p.position.y, p.position.z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        p.orientation = t.transform.rotation
        self.robot_pose = p
        return p

    def _publish(self) -> None:
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().info(f"waiting for TF {self.world} -> {self.base}", throttle_duration_sec=10.0)
            return                                    # believed positions are in `world`; without localization say nothing
        now = self.get_clock().now()
        snap = self.model.snapshot(now.nanoseconds * 1e-9)
        ws = WorldState()
        ws.header.stamp, ws.header.frame_id = now.to_msg(), self.world
        for v in snap.victims:
            s = VictimState()
            s.id, s.status, s.confidence = v.id, int(v.status), float(v.confidence)
            s.position.x, s.position.y, s.position.z = v.x, v.y, v.z
            s.last_seen = Time(seconds=v.last_seen).to_msg()
            ws.victims.append(s)                     # risk / accessibility / rescue_cost are cognition's to fill
        f = snap.fire
        ws.fire_known, ws.fire_confidence = f.known, float(f.confidence)
        ws.fire_position.x, ws.fire_position.y, ws.fire_position.z = f.x, f.y, f.z
        z = snap.safe_zone
        ws.safe_zone_pose.position.x, ws.safe_zone_pose.position.y = z.x, z.y
        ws.safe_zone_pose.orientation.z, ws.safe_zone_pose.orientation.w = math.sin(z.yaw / 2.0), math.cos(z.yaw / 2.0)
        ws.robot_pose = pose
        ws.current_target_id = snap.current_target_id
        self.state_pub.publish(ws)
        self._publish_markers(snap, ws.header)

    def _publish_markers(self, snap, header) -> None:
        arr = MarkerArray()
        life = Duration(seconds=1.0).to_msg()                          # stale markers vanish by themselves
        for s in marker_specs(snap):
            m = Marker()
            m.header, m.ns, m.id, m.type, m.action, m.lifetime = header, s.ns, s.id, _SHAPES[s.shape], Marker.ADD, life
            m.pose.position.x, m.pose.position.y, m.pose.position.z = s.x, s.y, s.z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(s.yaw / 2.0), math.cos(s.yaw / 2.0)
            m.scale.x, m.scale.y, m.scale.z = s.scale
            m.color.r, m.color.g, m.color.b, m.color.a = s.rgba
            m.text = s.text
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def destroy_node(self):
        self.tf_listener = None                # its __del__ stops the listener thread and drops its subscriptions
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'counts: {node.counts}; {len(node.model.snapshot(node.get_clock().now().nanoseconds * 1e-9).victims)} victims')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
