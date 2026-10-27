"""ROBOT SIDE (Raspberry Pi). /cmd_vel -> Arduino motor shield, and an open-loop /odom + odom->base_link TF.

    /cmd_vel (Twist) --DriveModel--> "M m1 m2 m3 m4" --serial--> firmware/base_controller --> 4 motors
    /odom, TF odom->base_link  <--DriveModel-- "S m1 m2 m3 m4" (the power the firmware is really applying)

Safety, in depth (Hardware_Integration.md: required before autonomous testing):
  - this node zeroes the command when /cmd_vel goes quiet for `cmd_timeout_s`;
  - the firmware releases the motors when THIS node goes quiet for 400 ms (crash, unplugged, Pi hung);
  - it will not send a single drive byte unless the board identifies itself as the FireResQ base firmware
    (the same Uno has carried an arm sketch in which 'S' starts a servo sequence).

/odom is DEAD RECKONING FROM COMMANDED POWER - there are no encoders. Read drive.py before using it for anything.
"""
from __future__ import annotations

import dataclasses
import math
import threading
import time
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .drive import TURN_STYLES, DriveModel, MotorMap, integrate
from .protocol import BAUD, PROTOCOL, STOP, Powers, format_drive, parse_banner, parse_status


class WrongFirmware(RuntimeError):
    pass


class BaseNode(Node):
    def __init__(self, **kwargs):
        super().__init__('base_node', **kwargs)
        d = self.declare_parameter
        self.port = d('port', '/dev/ttyUSB0').value
        self.cmd_timeout_s = float(d('cmd_timeout_s', 0.5).value)
        self.send_period_s = 1.0 / float(d('send_rate_hz', 10.0).value)
        self.status_timeout_s = float(d('status_timeout_s', 0.5).value)
        self.odom_frame, self.base_frame = d('odom_frame', 'odom').value, d('base_frame', 'base_link').value
        self.publish_tf = bool(d('publish_tf', True).value)
        separation = d('wheel_separation', rclpy.Parameter.Type.DOUBLE).value     # from the description (launch file)
        if separation is None:
            raise RuntimeError("parameter 'wheel_separation' is required (launch/robot.launch.py supplies it)")
        self.model = DriveModel(
            MotorMap(tuple(d('left_motors', [1, 3]).value), tuple(d('right_motors', [2, 4]).value),
                     tuple(d('motor_signs', [1, 1, 1, 1]).value), tuple(d('front_motors', [1, 2]).value)),
            track_m=float(separation) * float(d('track_slip_factor', 1.0).value),
            full_speed_mps=float(d('full_speed_mps', 0.3).value), min_power=int(d('min_power', 110).value),
            turn_style=d('turn_style', 'skid').value,
            right_turn_idle=tuple(d('right_turn_idle', rclpy.Parameter.Type.INTEGER_ARRAY).value or ()))
        # turn_style is the one parameter that can be changed while running (`ros2 param set /base_node turn_style ...`):
        # which style turns this robot best is found by trying them on the floor, not by restarting the Pi each time.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._lock = threading.Lock()
        self._cmd, self._cmd_time = (0.0, 0.0), -math.inf
        self._applied, self._applied_time = STOP, -math.inf
        self._x = self._y = self._yaw = 0.0
        self._last_odom: Optional[float] = None

        self.odom_pub = self.create_publisher(Odometry, d('odom_topic', '/odom').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, d('cmd_vel_topic', '/cmd_vel').value, self._on_cmd, 10)
        self.create_timer(1.0 / float(d('odom_rate_hz', 20.0).value), self._on_odom)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()

    # -- ROS side
    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'turn_style':
                if p.value not in TURN_STYLES:
                    return SetParametersResult(successful=False, reason=f'turn_style must be one of {sorted(TURN_STYLES)}')
                self.model = dataclasses.replace(self.model, turn_style=p.value)
                self.get_logger().info(f'turn_style: {p.value} ({TURN_STYLES[p.value]})')
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._cmd, self._cmd_time = (msg.linear.x, msg.angular.z), time.monotonic()

    def _wanted(self) -> Powers:
        with self._lock:
            (v, w), fresh = self._cmd, time.monotonic() - self._cmd_time <= self.cmd_timeout_s
        return self.model.motor_powers(v, w) if fresh else STOP

    def _on_odom(self) -> None:
        now = time.monotonic()
        with self._lock:                                         # no status = the firmware's watchdog has released the motors
            applied = self._applied if now - self._applied_time <= self.status_timeout_s else STOP
        v, w = self.model.twist_of(applied)
        if self._last_odom is not None:
            self._x, self._y, self._yaw = integrate(self._x, self._y, self._yaw, v, w, now - self._last_odom)
        self._last_odom = now

        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self._yaw / 2.0), math.cos(self._yaw / 2.0)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.odom_frame, self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self._x, self._y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        # Open-loop: large, and honest about which axes are worst. Nothing may read this as a measurement.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1.0
        odom.pose.covariance[35] = 4.0
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 1.0
        self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.odom_frame, self.base_frame
            t.transform.translation.x, t.transform.translation.y = self._x, self._y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(t)

    # -- serial side (its own thread: a blocked port must never stall the executor)
    def _connect(self) -> serial.Serial:
        ser = serial.Serial(self.port, BAUD, timeout=0.02)       # opening the port resets the Uno
        deadline, buf = time.monotonic() + 5.0, b''
        while time.monotonic() < deadline and not self._stop.is_set():
            buf += ser.read(64)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                version = parse_banner(line.decode('ascii', 'replace'))
                if version == PROTOCOL:
                    return ser
                if version is not None:
                    ser.close()
                    raise WrongFirmware(f'base firmware speaks protocol {version}, this driver speaks {PROTOCOL}: re-flash it')
        ser.close()
        raise WrongFirmware(f'{self.port}: the board did not identify as the FireResQ base firmware '
                            f'(flash src/fire_resq_hardware/firmware/base_controller). Not driving it.')

    def _io_loop(self) -> None:
        ser, buf, next_send, sent = None, b'', 0.0, None
        while not self._stop.is_set():
            try:
                if ser is None:
                    ser, buf, next_send, sent = self._connect(), b'', 0.0, None
                    self.get_logger().info(f'{self.port}: base firmware connected')
                now, wanted = time.monotonic(), self._wanted()
                # Sent at once when it CHANGES (a 0.15 s turn pulse must not wait up to a send period to start or
                # to stop), and repeated continuously: the repeats are what feed the firmware's watchdog.
                if wanted != sent or now >= next_send:
                    ser.write(format_drive(wanted))
                    sent, next_send = wanted, now + self.send_period_s
                buf += ser.read(64)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    powers = parse_status(line.decode('ascii', 'replace'))
                    if powers is not None:
                        with self._lock:
                            self._applied, self._applied_time = powers, time.monotonic()
            except (serial.SerialException, OSError, WrongFirmware) as e:
                if ser is not None:
                    ser.close()
                    ser = None
                self.get_logger().error(f'base: {e}; retrying in 3 s', throttle_duration_sec=30.0)
                self._stop.wait(3.0)
        if ser is not None:
            try:
                ser.write(format_drive(STOP))
                ser.flush()
            except (serial.SerialException, OSError):
                pass                                             # the firmware watchdog stops it within 400 ms anyway
            ser.close()

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
