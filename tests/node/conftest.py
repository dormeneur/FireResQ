"""Shared setup for node-level tests (in-process rclpy, no Gazebo)."""
import pytest


@pytest.fixture(scope='module', autouse=True)
def _rclpy_lifecycle():
    """Shut rclpy down after each module (the simulation tier calls rclpy.init() itself, and a context may only be initialised once at a time): the node's TF listener runs its own thread, which would otherwise keep pytest alive."""
    rclpy = pytest.importorskip('rclpy')
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()
