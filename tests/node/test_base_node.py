"""The base driver NODE against a fake Arduino on a pseudo-terminal - no robot, no serial hardware.

The fake speaks firmware/base_controller's protocol (banner, `M` in, `S` out, watchdog), or impersonates the arm
sketch this very Uno has carried. The real BaseNode runs in-process. What is checked is the safety glue:
/cmd_vel reaches the motors, silence stops them, a board that is not ours is never sent a byte, and /odom
reports what the firmware APPLIED rather than what was asked.
"""
import os
import pty
import select
import threading
import time
import tty
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not sourced')
pytest.importorskip('fire_resq_hardware', reason='workspace not sourced: source install/setup.bash')
pytest.importorskip('serial', reason='pyserial not installed')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from fire_resq_hardware.base_node import BaseNode  # noqa: E402

WATCHDOG_S, STATUS_S = 0.4, 0.1


class FakeArduino:
    """`sketch='base'`: the real protocol. `sketch='arm'`: prints the arm sketch's banner and understands nothing.
    `applies=False`: accepts commands but its motors stay released (a browned-out or disconnected driver stage)."""

    def __init__(self, sketch='base', applies=True):
        self.master, slave = pty.openpty()
        tty.setraw(slave)                                        # no echo: only what the node writes comes back
        self.port = os.ttyname(slave)
        self._slave = slave
        self.sketch, self.applies = sketch, applies
        self.received = b''
        self.commands = []                                       # (monotonic time, powers) for every valid M line
        self.applied = (0, 0, 0, 0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf, last_cmd, last_status, last_banner = b'', None, 0.0, 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if select.select([self.master], [], [], 0.01)[0]:
                data = os.read(self.master, 256)
                self.received += data
                buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                parts = line.decode('ascii', 'replace').split()
                if self.sketch == 'base' and len(parts) == 5 and parts[0] == 'M':
                    powers = tuple(int(p) for p in parts[1:])
                    self.commands.append((now, powers))
                    last_cmd = now
                    if self.applies:
                        self.applied = powers
            if last_cmd is not None and now - last_cmd > WATCHDOG_S:
                self.applied, last_cmd = (0, 0, 0, 0), None
            if self.sketch == 'arm':
                if now - last_banner > 0.5:
                    os.write(self.master, b'\r\nARM READY\r\nPress S to start movement.\r\n')
                    last_banner = now
            else:
                if not self.commands and now - last_banner > 0.3:    # the real board prints it once, at reset
                    os.write(self.master, b'FIRERESQ_BASE 2 wd=400\r\n')
                    last_banner = now
                if now - last_status >= STATUS_S:
                    os.write(self.master, ('S ' + ' '.join(map(str, self.applied)) + '\r\n').encode())
                    last_status = now

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        os.close(self.master)
        os.close(self._slave)


def wait(cond, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@contextmanager
def running(**fake_kwargs):
    if not rclpy.ok():
        rclpy.init()
    fake = FakeArduino(**fake_kwargs)
    params = {'port': fake.port, 'wheel_separation': 0.2, 'full_speed_mps': 0.5, 'min_power': 100,
              'left_motors': [1, 3], 'right_motors': [2, 4], 'motor_signs': [1, 1, 1, 1], 'cmd_timeout_s': 0.5}
    node = BaseNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    probe = Node('base_test_probe')
    probe.odom, probe.tf = [], []
    probe.create_subscription(Odometry, '/odom', probe.odom.append, 10)
    probe.create_subscription(TFMessage, '/tf', probe.tf.append, 10)
    probe.cmd = probe.create_publisher(Twist, '/cmd_vel', 10)
    # Single-threaded, as base_node really runs (rclpy.spin). Measured here: a MultiThreadedExecutor spinning the driver
    # AND a subscriber to its own /odom in one process freezes every Python thread for 0.4-1.5 s at a time.
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        # Discovery first: a /cmd_vel published before the driver's subscription is matched is simply lost.
        assert wait(lambda: probe.cmd.get_subscription_count() > 0 and probe.odom), 'base_node never joined the graph'
        yield fake, probe
    finally:
        ex.shutdown()
        th.join(timeout=3)
        node.destroy_node()
        probe.destroy_node()
        fake.close()


def drive(probe, v, w, seconds):
    msg = Twist()
    msg.linear.x, msg.angular.z = float(v), float(w)
    end = time.time() + seconds
    while time.time() < end:
        probe.cmd.publish(msg)
        time.sleep(0.05)


def test_cmd_vel_reaches_the_motors_and_odometry_follows_the_applied_power():
    with running() as (fake, probe):
        assert wait(lambda: fake.commands), 'the node never started commanding the board'
        assert fake.commands[0][1] == (0, 0, 0, 0)               # with no /cmd_vel yet, what it repeats is STOP
        drive(probe, 0.25, 0.0, 1.5)
        assert all(p > 100 for p in fake.applied), fake.applied  # four wheels forward, above start-up power
        assert wait(lambda: probe.odom and probe.odom[-1].twist.twist.linear.x == pytest.approx(0.25, abs=0.01))
        last = probe.odom[-1]
        assert last.header.frame_id == 'odom' and last.child_frame_id == 'base_link'
        assert 0.15 < last.pose.pose.position.x < 0.5 and abs(last.pose.pose.position.y) < 1e-6
        assert last.pose.covariance[0] >= 1.0                    # dead reckoning must not look like a measurement
        assert any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_link' for m in probe.tf for t in m.transforms)

        drive(probe, 0.0, 1.0, 0.6)                              # +w = turn left: left side back, right side forward
        assert fake.applied[0] < 0 and fake.applied[2] < 0 and fake.applied[1] > 0 and fake.applied[3] > 0


def test_the_command_is_repeated_fast_enough_to_hold_the_firmware_watchdog_off():
    with running() as (fake, probe):
        drive(probe, 0.25, 0.0, 2.0)
        moving = [t for t, p in fake.commands if any(p)]
        gaps = [b - a for a, b in zip(moving, moving[1:])]
        assert len(moving) >= 15 and max(gaps) < WATCHDOG_S / 2, max(gaps)


def test_silence_on_cmd_vel_stops_the_robot_and_the_odometry():
    with running() as (fake, probe):
        drive(probe, 0.25, 0.0, 1.0)
        t_quiet = time.monotonic()
        assert any(fake.applied)
        assert wait(lambda: fake.commands[-1][1] == (0, 0, 0, 0), timeout=2.0)
        stop_cmd = next(t for t, p in fake.commands if t > t_quiet and p == (0, 0, 0, 0))
        assert stop_cmd - t_quiet < 0.5 + 0.2                    # cmd_timeout_s + one send period + slack
        assert wait(lambda: probe.odom[-1].twist.twist.linear.x == 0.0, timeout=2.0)
        x = probe.odom[-1].pose.pose.position.x
        time.sleep(0.5)
        assert probe.odom[-1].pose.pose.position.x == x


def test_a_board_running_another_sketch_is_never_sent_a_single_byte():
    """The same Uno has carried an arm sketch where the byte 'S' starts a servo sequence."""
    with running(sketch='arm') as (fake, probe):
        drive(probe, 0.25, 0.5, 6.5)                             # longer than one whole identify attempt (5 s)
        assert fake.received == b''
        assert all(o.twist.twist.linear.x == 0.0 and o.pose.pose.position.x == 0.0 for o in probe.odom)


def test_odometry_reports_what_the_firmware_applied_not_what_was_asked():
    with running(applies=False) as (fake, probe):
        drive(probe, 0.25, 0.0, 1.5)
        assert any(any(p) for _, p in fake.commands)             # the node did ask
        assert probe.odom and all(o.twist.twist.linear.x == 0.0 for o in probe.odom)
        assert probe.odom[-1].pose.pose.position.x == 0.0


def test_the_turn_style_can_be_changed_while_running_and_a_bad_one_is_refused():
    """What the demo dashboard's "Turning style" does: the driver's own parameter service, no restart."""
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    with running() as (fake, probe):
        client = probe.create_client(SetParameters, '/base_node/set_parameters')
        assert client.wait_for_service(timeout_sec=5.0)

        def set_style(style):
            value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=style)
            future = client.call_async(SetParameters.Request(parameters=[ParameterMsg(name='turn_style', value=value)]))
            assert wait(future.done)
            return future.result().results[0].successful

        def signs():
            return tuple((p > 0) - (p < 0) for p in fake.applied)

        drive(probe, 0.0, 1.5, 0.8)                              # turn left on the spot
        assert signs() == (-1, 1, -1, 1), 'default: skid (left side back, right side forward)'
        assert set_style('pivot_back')
        drive(probe, 0.0, 1.5, 0.8)
        assert signs() == (-1, 0, -1, 0), 'pivot: left side back, right side stopped'
        assert wait(lambda: probe.odom[-1].twist.twist.angular.z > 0 > probe.odom[-1].twist.twist.linear.x, timeout=2.0)
        assert not set_style('crab')
        drive(probe, 0.0, -1.5, 0.8)
        assert signs() == (0, -1, 0, -1), 'a refused style changes nothing; right turn: right side back'
