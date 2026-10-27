"""ROBOT SIDE (Raspberry Pi). USB camera -> /camera/image_raw/compressed + /camera/camera_info.

The camera's own MJPEG frames are published untouched: the Pi never decodes or encodes an image (measured on the
Pi 3B with a Logitech C270: 0.2 % of one core for capture, ~22 kB per 640x480 frame). Decoding to the
/camera/image_raw that perception reads happens on the station (launch/station.launch.py, image_transport).

Two things about this camera that are easy to forget (both measured):
  - Its JPEGs carry NO Huffman tables (normal for USB MJPEG). OpenCV and libjpeg-turbo assume the standard ones,
    which is every decoder this project uses; a decoder that refuses the frames is missing that, not the stream.
  - In dim light it silently halves its frame rate to lengthen the exposure (15 requested, 7.6 delivered). Turn that
    off with the `v4l2_controls` parameter if a steady rate matters more than brightness.
"""
from __future__ import annotations

import subprocess
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

from .calibration import load_calibration


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__('camera_node', **kwargs)
        d = self.declare_parameter
        self.device = d('device', '/dev/video0').value
        self.width, self.height = int(d('width', 640).value), int(d('height', 480).value)
        self.fps = float(d('fps', 15.0).value)
        self.frame_id = d('frame_id', 'camera_optical_frame').value
        calibration_file = d('calibration_file', '').value
        controls = list(d('v4l2_controls', ['']).value)          # e.g. ['exposure_dynamic_framerate=0']
        if not calibration_file:
            raise RuntimeError("parameter 'calibration_file' is required (see config/camera_c270.yaml)")
        cal = load_calibration(calibration_file).scaled_to(self.width, self.height)

        self.info = CameraInfo()
        self.info.header.frame_id = self.frame_id
        self.info.width, self.info.height = cal.width, cal.height
        self.info.distortion_model, self.info.d = cal.distortion_model, cal.d
        self.info.k, self.info.r, self.info.p = cal.k, cal.r, cal.p

        self.image_pub = self.create_publisher(CompressedImage, d('image_topic', '/camera/image_raw/compressed').value,
                                               qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, d('camera_info_topic', '/camera/camera_info').value,
                                              qos_profile_sensor_data)
        self._apply_controls([c for c in controls if c])
        self.frames = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.create_timer(10.0, self._report)
        self._last_report = (time.monotonic(), 0)

    def _apply_controls(self, controls) -> None:
        for c in controls:
            r = subprocess.run(['v4l2-ctl', '-d', self.device, '--set-ctrl', c], capture_output=True, text=True)
            if r.returncode != 0:
                self.get_logger().warn(f"v4l2 control '{c}' not applied: {r.stderr.strip()}")

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)                     # hand over the camera's JPEG bytes, undecoded
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                      # never publish a stale queued frame
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if not cap.isOpened() or got != (self.width, self.height):
            cap.release()
            raise RuntimeError(f'{self.device}: wanted MJPEG {self.width}x{self.height}, camera gave {got}')
        return cap

    def _capture_loop(self) -> None:
        cap = None
        while not self._stop.is_set():
            try:
                if cap is None:
                    cap = self._open()
                    self.get_logger().info(f'{self.device}: MJPEG {self.width}x{self.height} @ {self.fps:g} fps requested')
                ok, buf = cap.read()                             # blocks until the camera delivers a frame
                if not ok:
                    raise RuntimeError(f'{self.device}: read failed')
                stamp = self.get_clock().now().to_msg()
                msg = CompressedImage()
                msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.info.header.stamp = stamp
                self.image_pub.publish(msg)
                self.info_pub.publish(self.info)
                self.frames += 1
            except Exception as e:                               # unplugged / busy: say so, retry, never crash the node
                if self._stop.is_set():
                    break
                self.get_logger().error(f'camera: {e}; retrying in 2 s', throttle_duration_sec=10.0)
                if cap is not None:
                    cap.release()
                    cap = None
                self._stop.wait(2.0)
        if cap is not None:
            cap.release()

    def _report(self) -> None:
        t, n = self._last_report
        now = time.monotonic()
        self.get_logger().info(f'{(self.frames - n) / (now - t):.1f} fps delivered')
        self._last_report = (now, self.frames)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
