"""REHEARSAL ONLY: a pretend robot in a pretend room, speaking the real robot's topics.

    /cmd_vel -> FakeRobot -> /odom                    (what the real base_node does, with no motors)
    FakeWorld.render -> /camera/image_raw/compressed  (what the real camera_node does, with no camera)

It exists so the demo can be practised, and shown, with no hardware. It must only ever run in the SIMULATION
ROS graph (`FIRE_RESQ_SIM=1`, domain 0, this PC only): on the robot's graph its /odom and camera would collide
with the real ones. demo.sh and the launch file keep it there.
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .fake_world import FakeRobot, FakeWorld, demo_room
from .vision import Camera


class FakeRobotNode(Node):
    def __init__(self, **kwargs):
        super().__init__('fake_robot', **kwargs)
        d = self.declare_parameter
        robot = FakeRobot(turn_gain=float(d('turn_gain', 0.7).value), speed_gain=float(d('speed_gain', 1.0).value),
                          delay_s=float(d('delay_s', 0.2).value))
        self.world = FakeWorld(demo_room(), Camera(height_m=float(d('camera_height_m', 0.05).value),
                                                   horizon_row=float(d('horizon_row', 200.0).value)), robot)
        self.cmd, self.cmd_t = (0.0, 0.0), -math.inf
        self.last = time.monotonic()
        self.rng = np.random.default_rng(1)
        self.image_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.05, self._on_step)
        self.create_timer(1.0 / float(d('fps', 10.0).value), self._on_frame)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd, self.cmd_t = (msg.linear.x, msg.angular.z), time.monotonic()

    def _on_step(self) -> None:
        now = time.monotonic()
        v, w = self.cmd if now - self.cmd_t <= 0.5 else (0.0, 0.0)      # the real driver's /cmd_vel timeout
        self.world.robot.step(now, v, w, now - self.last)
        self.last = now
        x, y, yaw = self.world.robot.odom
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = self.get_clock().now().to_msg(), 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.odom_pub.publish(odom)

    def _on_frame(self) -> None:
        msg = CompressedImage()
        msg.header.stamp, msg.header.frame_id, msg.format = self.get_clock().now().to_msg(), 'camera_optical_frame', 'jpeg'
        msg.data = cv2.imencode('.jpg', self.world.render(rng=self.rng), [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
