#!/usr/bin/env python3
"""sim_magnet_bridge - the Gazebo-specific half of the Phase 9 electromagnet abstraction.

The hardware-neutral half (fire_resq_control.magnet_node) knows nothing about Gazebo: it only
publishes a coil command (`/fire_resq/magnet/energize`, std_msgs/Bool) and reads back
`/fire_resq/magnet/state` (MagnetState) - exactly the same two-topic shape cmd_vel/odom already
use to make the sim<->hardware swap a driver change (see simulation/urdf/gz_drive.xacro). This
node is that driver, for simulation, and fire_resq_hardware's ESP32 bridge (Phase 12, TODO) is
its future hardware counterpart.

WHY THIS NODE EXISTS AT ALL (measured, not assumed - see docs/Implementation_Plan.md Phase 9):
gz-sim's DetachableJoint plugin has NO distance or contact check. Asking it to attach a victim
two metres away silently welds a rigid joint across that gap - no error, no warning, the victim
just becomes a rigid extension of the robot at whatever separation existed at that moment. Real
electromagnets do not do this: they hold only what is actually touching them. So it is THIS node,
not the plugin, that decides whether any victim is close enough to attach, using live ground-truth
poses (legitimate here - this is simulation-side code, not a ROS node cognition depends on) and
the same geometry Phase 7/8 already derive from the robot description
(fire_resq_simulation.magnet_geometry).

ALSO MEASURED: every declared DetachableJoint plugin instance starts ATTACHED by default (it is
built for "breadcrumbs": carried from the start, dropped on command). Left alone, every victim in
the scenario would be rigidly welded to the robot from the first physics step. This node's first
job, before anything else, is to detach all of them.

HOLD SENSING (Phase 10). MagnetState.attached must mean "the victim is physically held", not "a
command once succeeded": the rescue FSM never trusts a request, only the sensor's verdict, and a
real electromagnet needs a hold/contact sensor for exactly this reason (TODO(hardware): current
sense or a hall/reed switch on the ESP32). In simulation this node IS that sensor: it republishes
the state at `state_rate_hz`, and it declares the hold LOST - attached=false, coil still energised -
when (a) the plugin itself reports the joint detached (something else released it) or (b) the held
victim is no longer within `hold_loss_m` of the magnet face (it was knocked off or moved away).
An attach is likewise reported only once the plugin's own state topic says "attached" (the trigger
is re-sent until it does, for ATTACH_CONFIRM_S): a gz-transport message published before discovery
completes is silently dropped, and a robot "holding" a victim still lying on the floor is exactly
the false positive this sensor exists to prevent.
"""
import json
import math
import subprocess
import threading
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from fire_resq_description.geometry import read_properties
from fire_resq_interfaces.msg import MagnetState
from fire_resq_simulation.magnet_geometry import CONTACT_TOLERANCE_M, VICTIM_RING_RADIUS_M, contact_gap_m
from fire_resq_simulation.scenario import WORLD_NAME

ATTACH_CONFIRM_S = 2.0      # the joint plugin must confirm a weld this soon after the request, or nothing is held
LIFT_M = 0.030              # the victim is raised this much to be welded (see SimMagnetBridge._weld); its 55 mm ring still
                            # overlaps the magnet face (14.5-50.5 mm above the floor) anywhere in 0-30 mm
WELD_MIN_Z_M = 0.001        # ...and a weld only counts once the victim is held at least this far off the floor
WELD_TRIES = 5


def _quat_yaw(d):
    q = d.get('orientation', {})
    x, y, z, w = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class _PoseStream:
    """Latest (x, y, yaw) of named models, read from Gazebo ground truth. Simulation-side only -
    the same technique tests/sim/simlib.py's GroundTruth uses, kept independent of the test tree
    since this runs in production launches, not just tests."""

    def __init__(self, world_name, names):
        self._names = set(names)
        self._lock = threading.Lock()
        self._latest = {}
        self._z = {}                     # name -> (z, monotonic time of the sample)
        self._proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', f'/world/{world_name}/dynamic_pose/info', '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        dec, buf = json.JSONDecoder(), ''
        for chunk in self._proc.stdout:
            buf += chunk
            while True:
                buf = buf.lstrip()
                if not buf:
                    break
                try:
                    obj, end = dec.raw_decode(buf)
                except ValueError:
                    break
                buf = buf[end:]
                for e in obj.get('pose', []):
                    name = e.get('name')
                    if name in self._names:
                        pos = e.get('position', {})
                        with self._lock:
                            self._latest[name] = (pos.get('x', 0.0), pos.get('y', 0.0), _quat_yaw(e))
                            self._z[name] = (pos.get('z', 0.0), time.monotonic())

    def get(self, name):
        with self._lock:
            return self._latest.get(name)

    def z(self, name, since=0.0):
        """The model's latest height, if it was sampled after `since` (monotonic): a stale height says nothing."""
        with self._lock:
            v = self._z.get(name)
        return v[0] if v is not None and v[1] > since else None

    def close(self):
        self._proc.terminate()


class _Gz:
    """One gz-transport node for the whole bridge: persistent publishers (attach/detach triggers) and native subscriptions
    (the plugin's own state topics). Cheap: nothing here spawns a process."""

    def __init__(self):
        self._node = GzNode()
        self._pubs = {}

    def advertise(self, topic):
        if topic not in self._pubs:
            self._pubs[topic] = self._node.advertise(topic, Empty)

    def trigger(self, topic):
        self.advertise(topic)
        self._pubs[topic].publish(Empty())

    def set_pose(self, world, name, x, y, z, yaw):
        msg = GzPose()
        msg.name = name
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ok, rep = self._node.request(f'/world/{world}/set_pose', msg, GzPose, Boolean, 1000)
        return bool(ok and rep.data)

    def watch(self, topic, on_state):
        self._node.subscribe(StringMsg, topic, lambda msg: on_state(msg.data))

    def unwatch(self, topic):
        self._node.unsubscribe(topic)


class _JointWatch:
    """Watches one DetachableJoint plugin's own state topic ("attached"/"detached", gz.msgs.StringMsg) so a release that did
    NOT come through this node (a dropped victim) is noticed at once."""

    def __init__(self, gz, topic):
        self.attached = False          # the plugin has CONFIRMED the weld (it publishes "attached" once the joint exists)
        self.detached = False
        self._gz, self._topic = gz, topic
        gz.watch(topic, self._on)

    def _on(self, data):
        if 'detached' in data:
            self.detached = True
        elif 'attached' in data:
            self.attached = True

    def close(self):
        self._gz.unwatch(self._topic)


class SimMagnetBridge(Node):
    """Gazebo-specific driver for the Phase 9 electromagnet abstraction. See module docstring."""

    def __init__(self):
        super().__init__('sim_magnet_bridge')
        self.declare_parameter('world_name', WORLD_NAME)
        self.declare_parameter('victim_names', '')             # space-separated, from the scenario
        self.declare_parameter('contact_tolerance_m', CONTACT_TOLERANCE_M)
        self.declare_parameter('robot_model_name', 'fire_resq')
        self.declare_parameter('state_rate_hz', 5.0)              # MagnetState is a sensor reading, not a one-off reply
        self.declare_parameter('hold_loss_m', 0.06)               # held victim further than this from the face = lost

        world = self.get_parameter('world_name').value
        self._world = world
        self._victims = [v for v in self.get_parameter('victim_names').value.split(' ') if v]
        self._tolerance = float(self.get_parameter('contact_tolerance_m').value)
        self._hold_loss = float(self.get_parameter('hold_loss_m').value)
        self._robot_name = self.get_parameter('robot_model_name').value

        props = read_properties()
        self._magnet_x = props['magnet_x']
        self._magnet_length = props['magnet_length']

        self._held = None     # the victim's Gazebo model name currently attached (CONFIRMED by the plugin), or None
        self._pending = None  # (name, deadline): attach requested, not yet confirmed by the plugin
        self._energized = False
        self._watch = None    # _JointWatch on the held victim's plugin state topic

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(MagnetState, '/fire_resq/magnet/state', latched)
        self.create_subscription(Bool, '/fire_resq/magnet/energize', self._on_energize, 10)

        self._gz = _Gz()
        for name in self._victims:      # advertise every trigger NOW: gz-transport drops messages sent before discovery completes
            for verb in ('attach', 'detach'):
                self._gz.advertise(f'/model/{self._robot_name}/magnet/{name}/{verb}')
        self._startup_done = not self._victims
        self._robot_seen_at = None
        if not self._victims:
            self.get_logger().warn('sim_magnet_bridge: no victim_names configured - nothing to attach to')
            self._poses = None
        else:
            self._poses = _PoseStream(world, self._victims + [self._robot_name])
            # Every DetachableJoint instance starts ATTACHED (measured - see module docstring). Detach all of them before
            # anything else. The plugin only exists once the robot has SPAWNED - which, with the whole stack starting at
            # once, can take far longer than any fixed delay (measured: victims left welded to the robot, which then could
            # not turn) - so keep sending until the robot is seen in the world, and then for a while longer.
            self.create_timer(0.5, self._startup_tick)

        self._publish_state()
        self.create_timer(1.0 / float(self.get_parameter('state_rate_hz').value), self._tick)
        self.get_logger().info(
            f"sim_magnet_bridge up: {len(self._victims)} victim(s), contact tolerance {self._tolerance * 100:.0f} cm")

    def _startup_tick(self):
        if self._startup_done:
            return
        for name in self._victims:
            self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/detach')
        if self._robot_seen_at is None and self._poses.get(self._robot_name) is not None:
            self._robot_seen_at = time.monotonic()
        if self._robot_seen_at is not None and time.monotonic() - self._robot_seen_at > 3.0:
            self._startup_done = True
            self.get_logger().info('every victim detached: the magnet is ready')
            self._publish_state()

    def _on_energize(self, msg: Bool):
        self._energized = bool(msg.data)
        if self._energized:
            self._try_attach()
        else:
            self._try_detach()
        self._publish_state()

    def _try_attach(self):
        if self._held is not None or self._pending is not None or self._poses is None:
            return
        if not self._startup_done:
            self.get_logger().warn('attach refused: the victims are still being detached at start-up')
            return
        robot = self._poses.get(self._robot_name)
        if robot is None:
            self.get_logger().warn('sim_magnet_bridge: no ground-truth pose for the robot yet; attach refused')
            return
        rx, ry, ryaw = robot
        best_name, best_gap = None, math.inf
        for name in self._victims:
            p = self._poses.get(name)
            if p is None:
                continue
            gap = contact_gap_m((rx, ry), ryaw, (p[0], p[1]), self._magnet_x, self._magnet_length,
                                 VICTIM_RING_RADIUS_M)
            if gap < best_gap:
                best_name, best_gap = name, gap
        if best_name is not None and best_gap <= self._tolerance:
            # Watch the plugin's state BEFORE asking, and report nothing held until the plugin itself says "attached":
            # a trigger is a request, not a weld (measured: a lost first message left the robot "holding" a victim that
            # stayed on the floor while the robot turned away from it).
            self._weld_tries = 0
            self._weld(best_name)
            self.get_logger().info(f'attaching {best_name} (gap {best_gap * 100:.1f} cm)')
        else:
            reason = 'no victim in range' if best_name is None else f'nearest victim {best_gap * 100:.1f} cm away'
            self.get_logger().info(f'attach refused: {reason}')

    def _weld(self, name):
        """Lift the victim LIFT_M (keeping x, y, yaw), wait until it is seen raised, then weld it there.

        The real electromagnet LIFTS a lightweight victim off the floor. Gazebo's weld instead freezes the victim wherever it
        is - and resting on the floor it sits a fraction of a millimetre INTO the floor's soft contact. The solver keeps
        pushing it out, the weld passes that force to the robot, and friction then pins robot and victim to the floor.
        Measured (29 attach-then-turn trials, every victim): 13 welds left the robot unable to turn - while its odometry,
        and so its localisation, still "drove" the victim to the safe zone - and every one was a weld made while the victim
        sat sunk. Simulation-side only: ground truth is legitimate here and never in robot code."""
        self._close_watch()
        self._watch = _JointWatch(self._gz, f'/model/{self._robot_name}/magnet/{name}/state')
        self._pending = (name, time.monotonic() + ATTACH_CONFIRM_S)
        self._weld_tries += 1
        p = self._poses.get(name)
        t_lift = time.monotonic()
        if p is None or not self._gz.set_pose(self._world, name, p[0], p[1], LIFT_M, p[2]):
            self.get_logger().warn(f'could not lift {name} for the weld; welding where it rests')
        else:
            t_end = time.monotonic() + 0.3                  # it falls back in ~80 ms: weld the moment it is seen raised
            while time.monotonic() < t_end and (self._poses.z(name, since=t_lift) or 0.0) < LIFT_M / 3:
                time.sleep(0.002)
        self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/attach')
        self._weld_t = time.monotonic()

    def _try_detach(self):
        if self._pending is not None:                   # de-energised before the weld was confirmed: make sure there is none
            name, self._pending = self._pending[0], None
            self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/detach')
            self._close_watch()
        if self._held is None:
            return
        self._gz.trigger(f'/model/{self._robot_name}/magnet/{self._held}/detach')
        self.get_logger().info(f'detached {self._held}')
        self._held = None
        self._close_watch()

    def _close_watch(self):
        if self._watch is not None:
            self._watch.close()
            self._watch = None

    def _tick(self):
        """Attach confirmation, hold sensing, then the periodic state reading (see the module docstring)."""
        if self._pending is not None:
            name, deadline = self._pending
            if self._watch is not None and self._watch.attached:
                z = self._poses.z(name, since=self._weld_t + 0.05)          # a height sampled AFTER the weld took
                if z is None:
                    pass                                                    # not yet: check on the next tick
                elif z < WELD_MIN_Z_M:                  # welded while resting on the floor: undo, redo
                    self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/detach')
                    if self._weld_tries < WELD_TRIES:
                        self._weld(name)
                    else:
                        self._pending = None
                        self._close_watch()
                        self.get_logger().warn(f'{name} could not be welded clear of the floor in {WELD_TRIES} tries: nothing is held')
                else:
                    self._pending, self._held = None, name
                    self.get_logger().info(f'attached {name} (confirmed by the joint plugin, '
                                           f'{(z or 0.0) * 1000:.1f} mm off the floor, try {self._weld_tries})')
            elif time.monotonic() > deadline:
                self._pending = None
                self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/detach')
                self._close_watch()
                self.get_logger().warn(f'attach NOT confirmed for {name} within {ATTACH_CONFIRM_S:.0f} s: nothing is held')
            else:
                self._gz.trigger(f'/model/{self._robot_name}/magnet/{name}/attach')   # repeated: harmless once welded
        if self._held is not None:
            lost = None
            if self._watch is not None and self._watch.detached:
                lost = 'the joint was released from outside'
            else:
                robot, victim = self._poses.get(self._robot_name), self._poses.get(self._held)
                if robot is not None and victim is not None:
                    gap = contact_gap_m((robot[0], robot[1]), robot[2], (victim[0], victim[1]), self._magnet_x,
                                        self._magnet_length, VICTIM_RING_RADIUS_M)
                    if gap > self._hold_loss:
                        lost = f'the victim is {gap * 100:.0f} cm from the magnet face'
            if lost is not None:
                self.get_logger().warn(f'hold LOST on {self._held}: {lost}')
                self._held = None                       # the coil is still energised; nothing is held
                self._close_watch()
        self._publish_state()

    def _publish_state(self):
        if not self._startup_done:
            return                      # a driver that is not ready says nothing: no reading is better than a false "detached"
        if self._pending is not None:
            return                      # an attach is not answered until the plugin confirms (or fails) it: no premature "not held"
        m = MagnetState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.energized = self._energized
        m.attached = self._held is not None
        m.victim_id = self._held or ''
        self._state_pub.publish(m)

    def destroy_node(self):
        self._close_watch()
        if self._poses is not None:
            self._poses.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SimMagnetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
