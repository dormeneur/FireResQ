"""STATION SIDE (PC). /camera/image_raw/compressed (the robot's JPEG stream) -> /camera/image_raw (bgr8).

Decoding is the first thing that is cheaper on the PC than on the Pi, so it happens here; everything downstream
(perception) reads the same /camera/image_raw it reads in simulation. The image keeps the robot's capture stamp
and frame, so TF lookups refer to when the picture was TAKEN, not when it arrived over Wi-Fi.
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ImageDecoderNode(Node):
    def __init__(self, **kwargs):
        super().__init__('image_decoder_node', **kwargs)
        d = self.declare_parameter
        self.pub = self.create_publisher(Image, d('out_topic', '/camera/image_raw').value, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, d('in_topic', '/camera/image_raw/compressed').value, self._on_image,
                                 qos_profile_sensor_data)
        self.frames, self.latency_sum = 0, 0.0
        self.create_timer(10.0, self._report)

    def _on_image(self, msg: CompressedImage) -> None:
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('undecodable camera frame dropped', throttle_duration_sec=10.0)
            return
        out = Image()
        out.header = msg.header
        out.height, out.width = bgr.shape[:2]
        out.encoding, out.step = 'bgr8', bgr.shape[1] * 3
        out.data = bgr.tobytes()
        self.pub.publish(out)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames += 1
        self.latency_sum += self.get_clock().now().nanoseconds * 1e-9 - stamp

    def _report(self) -> None:
        if self.frames:
            self.get_logger().info(f'{self.frames / 10.0:.1f} fps, {1000.0 * self.latency_sum / self.frames:.0f} ms '
                                   f'capture-to-decoded (robot clock vs this clock)')
        else:
            self.get_logger().warn('no camera frames from the robot in the last 10 s')
        self.frames, self.latency_sum = 0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ImageDecoderNode()
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
