"""ROS glue for the rescue mission: turns ROS traffic into the FSM's Observations and its Effects into ROS traffic.

Thin on purpose, like every node in this project. The procedure (fsm.py), the geometry (geometry.py) and the search
(coverage.py) are pure and unit-tested; this file owns exactly four things:
  * the ROS inputs: the world model's belief, the perception stream's 2D victim sightings, the magnet's hold sensor,
    the map, and TF (map -> base_link);
  * the ROS outputs the FSM asks for: Nav2 goals, cognition's SelectTarget, the world model's UpdateVictimStatus, the
    magnet's SetMagnet, and direct cmd_vel (only while the FSM itself is driving: the look-around, ALIGN, the carry tug);
  * a clock (the simulation's), so the FSM never reads one;
  * the mission's outward face: `/fire_resq/mission_state` (JSON: state, target, counts, the full transition history)
    and the `ExecuteRescue` action.

It ranks nothing, plans nothing, and touches no Gazebo, scenario or hardware (unit-guarded). It does publish an initial
pose to AMCL until localisation exists, because a cold start needs the mission's start pose (the map origin, since the
map was built from the start) - that is operator setup, not ground truth.
"""
from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from fire_resq_interfaces.action import ExecuteRescue
from fire_resq_interfaces.msg import DetectionArray, MagnetState, WorldState
from fire_resq_interfaces.srv import SelectTarget, SetMagnet as SetMagnetSrv, UpdateVictimStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearCostmapAroundRobot
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from .config import PlanningConfig
from .coverage import GridMap
from .fsm import RescueFSM
from .geometry import Pose2, Zone
from .types import (COSTMAP_DONE, MAGNET_DONE, NAV_DONE, SELECT_DONE, STATUS_DONE, TERMINAL, Camera, CallSelect, CancelNav, ClearCostmaps, Drive, Effect, Event,
                    MagnetReading, Observation, SendNav, SetMagnet, SetStatus, Sighting, State, Stop, VictimBelief, World)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SERVICE_TIMEOUT_S = 12.0
SIGHTING_WINDOW_S = 1.0


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RescueNode(Node):
    def __init__(self, **kwargs):
        super().__init__('rescue_node', **kwargs)
        cfg_defaults = PlanningConfig()
        for name in cfg_defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(cfg_defaults, name))
        for name, val in (('world_state_topic', '/fire_resq/world_state'), ('detections_topic', '/fire_resq/detections'),
                          ('magnet_state_topic', '/fire_resq/magnet/state'), ('mission_state_topic', '/fire_resq/mission_state'),
                          ('map_topic', '/map'), ('camera_info_topic', '/camera/camera_info'), ('cmd_vel_topic', '/cmd_vel'),
                          ('select_target_service', '/fire_resq/select_target'),
                          ('update_status_service', '/fire_resq/update_victim_status'),
                          ('set_magnet_service', '/fire_resq/set_magnet'), ('navigate_action', 'navigate_to_pose'),
                          ('map_frame', 'map'), ('base_frame', 'base_link'), ('tick_hz', 20.0), ('autostart', True),
                          ('publish_initial_pose', True), ('use_robot_limits', True)):
            self.declare_parameter(name, val)
        p = self.get_parameter
        cfg = {f: p(f).value for f in cfg_defaults.__dataclass_fields__}
        if p('use_robot_limits').value:                    # the description is the source of truth for the robot's geometry and limits
            from fire_resq_description.geometry import read_properties
            props = read_properties()
            cfg.update(magnet_reach_m=props['magnet_x'] + props['magnet_length'] / 2, max_linear_mps=props['max_linear_velocity'],
                       max_angular_rps=props['max_angular_velocity'])
        self.cfg = PlanningConfig.from_dict(cfg)
        self.fsm = RescueFSM(self.cfg)
        self.map_frame, self.base_frame = p('map_frame').value, p('base_frame').value

        # ---- inputs
        self._world: Optional[World] = None
        self._world_rx = -math.inf
        self._sightings: List[Sighting] = []
        self._camera: Optional[Camera] = None
        self._magnet: Optional[MagnetReading] = None
        self._grid: Optional[GridMap] = None
        self.create_subscription(WorldState, p('world_state_topic').value, self._on_world, 10)
        self.create_subscription(DetectionArray, p('detections_topic').value, self._on_detections, 10)
        self.create_subscription(CameraInfo, p('camera_info_topic').value, self._on_camera, 1)
        self.create_subscription(MagnetState, p('magnet_state_topic').value, self._on_magnet, LATCHED)
        self.create_subscription(OccupancyGrid, p('map_topic').value, self._on_map, LATCHED)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node=None, spin_thread=True)     # its own node and thread

        # ---- outputs / clients
        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)
        self.state_pub = self.create_publisher(String, p('mission_state_topic').value, LATCHED)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav = ActionClient(self, NavigateToPose, p('navigate_action').value)
        self.select_cli = self.create_client(SelectTarget, p('select_target_service').value)
        self.status_cli = self.create_client(UpdateVictimStatus, p('update_status_service').value)
        self.magnet_cli = self.create_client(SetMagnetSrv, p('set_magnet_service').value)
        self.clear_clis = [self.create_client(ClearCostmapAroundRobot, f'/{n}_costmap/clear_around_{n}')
                           for n in ('local', 'global')]

        # ---- async plumbing
        self._events: List[Event] = []
        self._inflight: Dict[int, Tuple[str, float]] = {}
        self._nav_handle = None
        self._nav_tag = 0
        self._zero_ticks = 0
        self._logged = 0
        self._pub_len = 0
        self._last_status = -math.inf
        self._done: Optional[Future] = None
        self._started = bool(p('autostart').value)
        self._last_init_pose = -math.inf
        self._ever_localised = False
        self._detections_seen = 0

        self.action = ActionServer(self, ExecuteRescue, 'execute_rescue', self._execute, goal_callback=self._on_goal,
                                   cancel_callback=lambda _: CancelResponse.REJECT)
        self.create_timer(1.0 / float(p('tick_hz').value), self._tick)
        self.get_logger().info(f'rescue_node ready (autostart {self._started}): magnet reach {self.cfg.magnet_reach_m:.3f} m, '
                               f'contact at {self.cfg.contact_center_dist_m:.3f} m, standoff {self.cfg.approach_standoff_m:.2f} m')

    # ================================================================================================ inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_world(self, m: WorldState) -> None:
        z = m.safe_zone_pose
        zone = Zone(z.position.x, z.position.y, yaw_of(z.orientation), self.cfg.safe_zone_size_x, self.cfg.safe_zone_size_y)
        victims = tuple(VictimBelief(v.id, v.position.x, v.position.y, float(v.confidence), int(v.status),
                                     v.last_seen.sec + v.last_seen.nanosec * 1e-9) for v in m.victims)
        self._world = World(victims, zone, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self._world_rx = self._now()

    def _on_detections(self, m: DetectionArray) -> None:
        self._detections_seen += 1
        now = self._now()
        for d in m.detections:
            if d.class_id == 'victim':
                self._sightings.append(Sighting(float(d.bbox.center.position.x), float(d.bbox.size_x), now))
        self._sightings = [s for s in self._sightings if now - s.stamp <= SIGHTING_WINDOW_S]

    def _on_camera(self, m: CameraInfo) -> None:
        fx, cx = float(m.k[0]), float(m.k[2])
        if fx > 0:
            self._camera = Camera(fx, cx, 2.0 * math.atan2(m.width / 2.0, fx))

    def _on_magnet(self, m: MagnetState) -> None:
        self._magnet = MagnetReading(bool(m.energized), bool(m.attached), self._now())

    def _on_map(self, m: OccupancyGrid) -> None:
        data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        self._grid = GridMap(m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, data)

    def _robot(self) -> Tuple[Optional[Pose2], float]:
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None, 0.0
        t, q = tr.transform.translation, tr.transform.rotation
        return Pose2(t.x, t.y, yaw_of(q)), tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9

    def _observation(self) -> Observation:
        now = self._now()
        robot, robot_stamp = self._robot()
        world_fresh = self._world is not None and now - self._world_rx <= max(self.cfg.world_stale_s, 2.0)
        ready = (robot is not None and world_fresh and self._grid is not None and self._magnet is not None
                 and self.nav.server_is_ready() and self.select_cli.service_is_ready() and self.status_cli.service_is_ready()
                 and self.magnet_cli.service_is_ready())
        events, self._events = tuple(self._events), []
        return Observation(t=now, ready=ready, robot=robot, robot_stamp=robot_stamp,
                           world=self._world if world_fresh else None,
                           sightings=tuple(s for s in self._sightings if now - s.stamp <= SIGHTING_WINDOW_S),
                           camera=self._camera, magnet=self._magnet, grid=self._grid, nav_active=self._nav_handle is not None, events=events)

    # ================================================================================================ the loop
    def _tick(self) -> None:
        now = self._now()
        self._maybe_publish_initial_pose(now)
        self._expire_inflight(now)
        if not self._started:
            return
        obs = self._observation()
        effects = self.fsm.step(obs)
        for eff in effects:
            self._execute_effect(eff)
        if self._zero_ticks > 0 and self._nav_handle is None and not any(isinstance(e, (Drive, Stop)) for e in effects):
            self._cmd(0.0, 0.0)                           # keep saying "zero" for a few ticks after a stop (never while Nav2 drives)
            self._zero_ticks -= 1
        self._log_new_transitions()
        if now - self._last_status >= 1.0 or len(self.fsm.history) != self._pub_len:
            self._publish_status(now)
        if self.fsm.state in TERMINAL:
            self._finish_action()

    def _maybe_publish_initial_pose(self, now: float) -> None:
        """Until AMCL has localised the robot, tell it where the mission starts (the map's origin: the map was built from the
        start pose). Nav2's lifecycle bring-up gives up if `map` never appears, so this cannot wait for the operator."""
        if self._ever_localised or not self.get_parameter('publish_initial_pose').value or now - self._last_init_pose < 0.5:
            return
        pose, _ = self._robot()
        if pose is not None:
            self._ever_localised = True                   # ONCE, ever: a later lookup hiccup must never reset AMCL to the origin
            return
        self._last_init_pose = now
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = m.pose.covariance[7] = m.pose.covariance[35] = 0.05
        self.init_pub.publish(m)

    def _log_new_transitions(self) -> None:
        for t in self.fsm.history[self._logged:]:
            self.get_logger().info(f'[{t.t:7.1f}s] {t.frm} -> {t.to}: {t.reason}' + (f' ({t.target})' if t.target else ''))
        self._logged = len(self.fsm.history)

    def _publish_status(self, now: float) -> None:
        snap = self.fsm.snapshot(now)
        snap['detections_seen'] = self._detections_seen
        snap['nav_active'] = self._nav_handle is not None
        self.state_pub.publish(String(data=json.dumps(snap)))
        self._last_status = now
        self._pub_len = len(self.fsm.history)

    # ================================================================================================ effects
    def _execute_effect(self, e: Effect) -> None:
        if isinstance(e, Drive):
            self._zero_ticks = 0
            self._cmd(e.v, e.w)
        elif isinstance(e, Stop):
            self._cmd(0.0, 0.0)
            self._zero_ticks = 3                          # the simulated drive holds its last command: say "zero" a few times
        elif isinstance(e, SendNav):
            self._send_nav(e)
        elif isinstance(e, CancelNav):
            self._cancel_nav()
        elif isinstance(e, CallSelect):
            self._call(self.select_cli, SelectTarget.Request(), e.req, 'select', self._on_select)
        elif isinstance(e, SetStatus):
            r = UpdateVictimStatus.Request()
            r.victim_id, r.new_status = e.victim_id, int(e.status)
            self._call(self.status_cli, r, e.req, 'status', self._on_status)
        elif isinstance(e, ClearCostmaps):
            self._clear_costmaps(e)
        elif isinstance(e, SetMagnet):
            self._call(self.magnet_cli, SetMagnetSrv.Request(attach=bool(e.attach)), e.req, 'magnet', self._on_magnet_done)

    def _cmd(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def _send_nav(self, e: SendNav) -> None:
        if self._nav_handle is not None:
            self._cancel_nav()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = e.goal.x, e.goal.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(e.goal.yaw / 2), math.cos(e.goal.yaw / 2)
        tag = e.tag
        self._nav_tag = tag
        fut = self.nav.send_goal_async(goal)

        def accepted(f):
            h = f.result()
            if h is None or not h.accepted:
                self._events.append(Event(NAV_DONE, tag, nav_status='rejected', detail='Nav2 rejected the goal'))
                if self._nav_tag == tag:
                    self._nav_handle = None
                return
            if self._nav_tag != tag:                       # cancelled before it was even accepted
                h.cancel_goal_async()
                return
            self._nav_handle = h
            h.get_result_async().add_done_callback(lambda rf: finished(rf, h))

        def finished(rf, h):
            status = rf.result().status
            name = {4: 'succeeded', 5: 'canceled', 6: 'failed'}.get(status, f'status {status}')
            if self._nav_handle is h:
                self._nav_handle = None
            self._events.append(Event(NAV_DONE, tag, nav_status=name, detail=''))

        fut.add_done_callback(accepted)

    def _cancel_nav(self) -> None:
        h, self._nav_handle = self._nav_handle, None
        self._nav_tag = -1
        if h is not None:
            h.cancel_goal_async()
        self._cmd(0.0, 0.0)
        self._zero_ticks = 5

    def _call(self, client, request, req: int, kind: str, on_done) -> None:
        if not client.service_is_ready():
            self._events.append(Event({'select': SELECT_DONE, 'status': STATUS_DONE, 'magnet': MAGNET_DONE}[kind], req,
                                      ok=False, detail=f'the {kind} service is not available'))
            return
        self._inflight[req] = (kind, self._now() + SERVICE_TIMEOUT_S)
        client.call_async(request).add_done_callback(lambda f: on_done(req, f))

    def _clear_costmaps(self, e: ClearCostmaps) -> None:
        """Both Nav2 costmaps forget the obstacle marks near the robot; answered once every server has replied (or is missing)."""
        left = [len(self.clear_clis)]

        def one(f):
            left[0] -= 1
            if left[0] == 0:
                self._events.append(Event(COSTMAP_DONE, e.req, ok=True))

        for cli in self.clear_clis:
            if not cli.service_is_ready():
                one(None)                                   # not fatal: the next planner query reports whatever is still in the way
                continue
            r = ClearCostmapAroundRobot.Request()
            r.reset_distance = float(e.radius)
            cli.call_async(r).add_done_callback(one)

    def _expire_inflight(self, now: float) -> None:
        for req, (kind, deadline) in list(self._inflight.items()):
            if now > deadline:
                del self._inflight[req]
                self._events.append(Event({'select': SELECT_DONE, 'status': STATUS_DONE, 'magnet': MAGNET_DONE}[kind], req,
                                          ok=False, detail=f'the {kind} service did not answer in {SERVICE_TIMEOUT_S:.0f} s'))

    def _on_select(self, req: int, f) -> None:
        if self._inflight.pop(req, None) is None:
            return
        r = f.result()
        if r is None:
            self._events.append(Event(SELECT_DONE, req, ok=False, detail='SelectTarget failed'))
            return
        a = r.target.approach_pose.pose
        self._events.append(Event(SELECT_DONE, req, ok=bool(r.found), detail=r.reason or r.target.rationale,
                                  victim_id=r.target.victim_id, approach=Pose2(a.position.x, a.position.y, yaw_of(a.orientation))))

    def _on_status(self, req: int, f) -> None:
        if self._inflight.pop(req, None) is None:
            return
        r = f.result()
        self._events.append(Event(STATUS_DONE, req, ok=bool(r is not None and r.success), detail='' if r is None else r.message))

    def _on_magnet_done(self, req: int, f) -> None:
        if self._inflight.pop(req, None) is None:
            return
        r = f.result()
        if r is None:
            self._events.append(Event(MAGNET_DONE, req, ok=False, detail='SetMagnet failed'))
            return
        self._events.append(Event(MAGNET_DONE, req, ok=bool(r.success), attached=bool(r.state.attached),
                                  detail=f'energized={r.state.energized} attached={r.state.attached}'))

    # ================================================================================================ ExecuteRescue
    def _on_goal(self, goal) -> GoalResponse:
        if goal.victim_id:
            self.get_logger().warn(f'ExecuteRescue({goal.victim_id!r}) refused: which victim is cognition\'s choice, never the caller\'s')
            return GoalResponse.REJECT
        return GoalResponse.REJECT if self._done is not None else GoalResponse.ACCEPT

    async def _execute(self, handle):
        """The action IS the mission: it starts it (unless autostarted), reports the FSM's state as feedback, and ends when the
        mission does. `success` = the mission completed with nothing left unrescued."""
        self._done = Future()
        self._started = True
        fb = ExecuteRescue.Feedback()
        t0 = self._now()

        def report():
            fb.phase, fb.detail = self.fsm.state.value, self.fsm.reason
            handle.publish_feedback(fb)
        timer = self.create_timer(1.0, report)
        await self._done
        timer.cancel()
        self.destroy_timer(timer)
        ok = self.fsm.state == State.MISSION_COMPLETE and not self.fsm.m.given_up
        handle.succeed() if ok else handle.abort()
        res = ExecuteRescue.Result()
        res.success, res.duration_s = ok, float(self._now() - t0)
        res.final_status = 4 if ok else 5                  # VictimState.RESCUED / UNREACHABLE: the mission's outcome, not one victim's
        self._done = None
        return res

    def _finish_action(self) -> None:
        if self._done is not None and not self._done.done():
            self._done.set_result(True)


def main():
    rclpy.init()
    node = RescueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
