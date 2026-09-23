"""The magnet NODE against synthetic ROS traffic - no Gazebo.

A fake driver stands in for sim_magnet_bridge.py (or, on hardware, the ESP32 bridge): it answers
`/fire_resq/magnet/energize` from a rule the test sets and publishes `/fire_resq/magnet/state`,
exactly the two-topic protocol magnet_backend.py speaks. The real MagnetNode and the real
RosTopicMagnetBackend run in-process. This checks the glue: attach/detach never reports success
before the driver's OWN state confirms it (Phase 9 hard rule), a missing driver fails cleanly
rather than hanging or crashing, and the reported victim_id/attached flag are exactly what the
driver said.
"""
import threading
import time
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 not sourced')
pytest.importorskip('fire_resq_interfaces', reason='workspace not sourced: source install/setup.bash')

from fire_resq_interfaces.msg import MagnetState  # noqa: E402
from fire_resq_interfaces.srv import SetMagnet  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from fire_resq_control.magnet_node import MagnetNode  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def always_confirms(energize):
    """attach -> attached to 'V1'; detach -> released."""
    return (True, 'V1') if energize else (False, '')


def never_finds_a_victim(energize):
    return (False, '')


class FakeDriver(Node):
    """Stands in for sim_magnet_bridge.py / the ESP32 bridge: answers energize from `self.rule`."""

    def __init__(self, delay_s=0.0):
        super().__init__('fake_magnet_driver')
        self.rule = always_confirms
        self.delay_s = delay_s
        self.commands = []
        self.pub = self.create_publisher(MagnetState, '/fire_resq/magnet/state', LATCHED)
        self.create_subscription(Bool, '/fire_resq/magnet/energize', self._on_energize, 10)

    def _on_energize(self, msg: Bool):
        self.commands.append(bool(msg.data))
        if self.delay_s:
            time.sleep(self.delay_s)
        attached, victim_id = self.rule(bool(msg.data))
        m = MagnetState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.energized = bool(msg.data)
        m.attached, m.victim_id = attached, victim_id
        self.pub.publish(m)


def wait(cond, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@contextmanager
def running(driver=True, rule=None, driver_delay_s=0.0, **overrides):
    if not rclpy.ok():
        rclpy.init()
    params = {'attach_timeout_s': 3.0, 'detach_timeout_s': 3.0, **overrides}
    node = MagnetNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    fake = FakeDriver(delay_s=driver_delay_s) if driver else None
    if fake and rule:
        fake.rule = rule
    nodes = [n for n in (node, fake) if n is not None]
    ex = MultiThreadedExecutor(num_threads=4)
    for n in nodes:
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    sink = Node('magnet_test_sink')
    ex.add_node(sink)
    client = sink.create_client(SetMagnet, '/fire_resq/set_magnet')
    assert client.wait_for_service(timeout_sec=5.0), '/fire_resq/set_magnet never became available'
    try:
        yield client, fake
    finally:
        ex.shutdown()
        th.join(timeout=3)
        for n in nodes + [sink]:
            n.destroy_node()


def call(client, attach, timeout=6.0):
    fut = client.call_async(SetMagnet.Request(attach=attach))
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    assert fut.done(), 'SetMagnet did not answer in time'
    return fut.result()


def test_attach_succeeds_and_reports_the_victim_the_driver_confirmed():
    with running(rule=always_confirms) as (client, driver):
        res = call(client, True)
        assert res.success is True
        assert res.state.attached is True and res.state.victim_id == 'V1'


def test_attach_fails_when_the_driver_says_nothing_was_in_range():
    """The service never reports "attached" just because a request was accepted."""
    with running(rule=never_finds_a_victim) as (client, driver):
        res = call(client, True)
        assert res.success is False
        assert res.state.attached is False and res.state.victim_id == ''


def test_detach_succeeds_when_the_driver_confirms_release():
    with running(rule=always_confirms) as (client, driver):
        call(client, True)
        res = call(client, False)
        assert res.success is True
        assert res.state.attached is False and res.state.victim_id == ''


def test_a_missing_driver_fails_cleanly_rather_than_hanging_or_crashing():
    with running(driver=False, attach_timeout_s=0.5, detach_timeout_s=0.5) as (client, _):
        res = call(client, True, timeout=3.0)
        assert res.success is False, 'no driver was running - this must not report success'


def test_the_energize_command_reaches_the_driver_exactly_once_per_request():
    with running(rule=always_confirms) as (client, driver):
        call(client, True)
        call(client, False)
        assert wait(lambda: driver.commands == [True, False]), driver.commands


def test_a_slow_driver_still_gets_waited_on_not_raced():
    """The response only counts once the driver's OWN state update arrives - proven by a driver
    that deliberately answers late."""
    with running(rule=always_confirms, driver_delay_s=0.3) as (client, driver):
        t0 = time.time()
        res = call(client, True, timeout=5.0)
        elapsed = time.time() - t0
        assert res.success is True
        assert elapsed >= 0.25, f'answered in {elapsed:.2f}s - faster than the driver could have confirmed anything'
