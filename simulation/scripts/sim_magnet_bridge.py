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
"""
import json
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from fire_resq_description.geometry import read_properties
from fire_resq_interfaces.msg import MagnetState
from fire_resq_simulation.magnet_geometry import CONTACT_TOLERANCE_M, VICTIM_RING_RADIUS_M, contact_gap_m
from fire_resq_simulation.scenario import WORLD_NAME


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

    def get(self, name):
        with self._lock:
            return self._latest.get(name)

    def close(self):
        self._proc.terminate()


def _gz_publish_empty(topic, timeout_s=2.0):
    subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', 'unused: true'],
                    capture_output=True, timeout=timeout_s)


class SimMagnetBridge(Node):
    """Gazebo-specific driver for the Phase 9 electromagnet abstraction. See module docstring."""

    def __init__(self):
        super().__init__('sim_magnet_bridge')
        self.declare_parameter('world_name', WORLD_NAME)
        self.declare_parameter('victim_names', '')             # space-separated, from the scenario
        self.declare_parameter('contact_tolerance_m', CONTACT_TOLERANCE_M)
        self.declare_parameter('robot_model_name', 'fire_resq')

        world = self.get_parameter('world_name').value
        self._victims = [v for v in self.get_parameter('victim_names').value.split(' ') if v]
        self._tolerance = float(self.get_parameter('contact_tolerance_m').value)
        self._robot_name = self.get_parameter('robot_model_name').value

        props = read_properties()
        self._magnet_x = props['magnet_x']
        self._magnet_length = props['magnet_length']

        self._held = None     # the victim's Gazebo model name currently attached, or None
        self._energized = False

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(MagnetState, '/fire_resq/magnet/state', latched)
        self.create_subscription(Bool, '/fire_resq/magnet/energize', self._on_energize, 10)

        if not self._victims:
            self.get_logger().warn('sim_magnet_bridge: no victim_names configured - nothing to attach to')
            self._poses = None
        else:
            self._poses = _PoseStream(world, self._victims + [self._robot_name])
            # Every DetachableJoint instance starts ATTACHED (measured - see module docstring).
            # Detach all of them before anything else, retried because the plugin's subscriber
            # may not exist the instant this node comes up.
            self._detach_all_at_startup()

        self._publish_state()
        self.get_logger().info(
            f"sim_magnet_bridge ready: {len(self._victims)} victim(s), contact tolerance {self._tolerance * 100:.0f} cm")

    def _detach_all_at_startup(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            for name in self._victims:
                _gz_publish_empty(f'/model/{self._robot_name}/magnet/{name}/detach')
            time.sleep(0.2)

    def _on_energize(self, msg: Bool):
        self._energized = bool(msg.data)
        if self._energized:
            self._try_attach()
        else:
            self._try_detach()
        self._publish_state()

    def _try_attach(self):
        if self._held is not None or self._poses is None:
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
            _gz_publish_empty(f'/model/{self._robot_name}/magnet/{best_name}/attach')
            self._held = best_name
            self.get_logger().info(f'attached {best_name} (gap {best_gap * 100:.1f} cm)')
        else:
            reason = 'no victim in range' if best_name is None else f'nearest victim {best_gap * 100:.1f} cm away'
            self.get_logger().info(f'attach refused: {reason}')

    def _try_detach(self):
        if self._held is None:
            return
        _gz_publish_empty(f'/model/{self._robot_name}/magnet/{self._held}/detach')
        self.get_logger().info(f'detached {self._held}')
        self._held = None

    def _publish_state(self):
        m = MagnetState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.energized = self._energized
        m.attached = self._held is not None
        m.victim_id = self._held or ''
        self._state_pub.publish(m)

    def destroy_node(self):
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
