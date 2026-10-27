"""STATION SIDE (PC). The bottle demo: camera in, /cmd_vel out, dashboard on http://localhost:8080.

    /camera/image_raw/compressed --decode--> vision (bottles, floor edge) --> Seeker --> /cmd_vel
    /odom --> PoseTracker (dead reckoning) --> the map on the dashboard

Stable ROS topics only, so it runs unchanged against the real robot (Pi: camera_node + base_node) and against
fake_robot_node (the rehearsal). The behaviour is seeker.py; this file is plumbing.

`drive:=false` is a DRY RUN: everything works and is shown, but no /cmd_vel publisher is even created.
"""
from __future__ import annotations

import dataclasses
import json
import math
import queue
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool

from .colours import PICKED, colour_name, colour_rgb, hue_distance
from .dashboard import Dashboard, annotate
from .mapping import FloorMap, PoseTracker, draw_map
from .seeker import DONE, IDLE, WAIT, Seeker, SeekerParams, Sighting
from .vision import Camera, FloorModel, VisionParams, find_bottles, find_picked, pick_colour

# What the dashboard may change while running: group.name -> (lowest, highest). Saved to `settings_file`.
TUNABLE = {
    'vision.sat_min': (30, 255), 'vision.dark_max': (5, 150), 'vision.detect_black': (0, 1),
    'vision.floor_l_tol': (10, 120), 'vision.floor_ab_tol': (2, 40),
    'camera.height_m': (0.02, 0.40), 'camera.horizon_row': (60, 420),
    'seek.drive_speed': (0.03, 0.30), 'seek.turn_speed': (0.3, 4.5), 'seek.left_boost': (1.0, 3.0), 'seek.right_boost': (1.0, 3.0),
    'seek.arrive_cover_frac': (0.30, 0.95), 'seek.final_push_s': (0.0, 2.0),
    'seek.wait_s': (1, 20), 'seek.expected_bottles': (0, 9), 'seek.avoid': (0, 1), 'seek.obstacle_range_m': (0.12, 0.90),
}
# The colours the robot can be told to follow (dashboard tick-boxes; all on unless saved otherwise).
COLOURS = ('black', 'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple', 'pink')
# A blob of an already-accepted bottle's colour that fails only these rules is still that bottle.
SOFT = ('shape', 'too big', 'too small', 'not solid')


class DemoNode(Node):
    def __init__(self, **kwargs):
        super().__init__('bottle_demo', **kwargs)
        d = self.declare_parameter
        self.drive = bool(d('drive', True).value)
        self.cam = Camera(height_m=float(d('camera.height_m', 0.05).value),
                          horizon_row=float(d('camera.horizon_row', 200.0).value))
        self.vp, self.sp = VisionParams(), SeekerParams()
        for group, params in (('vision', self.vp), ('seek', self.sp)):   # every field is a ROS parameter: vision.sat_min ...
            for f in dataclasses.fields(params):
                setattr(params, f.name, type(getattr(params, f.name))(d(f'{group}.{f.name}', getattr(params, f.name)).value))
        settings = d('settings_file', '').value
        self.settings_file = Path(settings) if settings else None
        self.follow = set(COLOURS)
        self._load_settings()
        self.last_bgr, self.last_blobs, self.floor_visible = None, [], None
        self.pick_lab, self.pick_at = None, (0.0, 0.0)           # the clicked object's colour, and where it was last seen
        self.forced = []                                         # colours the operator clicked: no shape rules for them

        self.tracker, self.floor_map, self.floor = PoseTracker(), FloorMap(), FloorModel()
        self.seeker = Seeker(self.sp, self.tracker)
        self.dash = Dashboard(d('host', '127.0.0.1').value, int(d('port', 8080).value))
        self.manual, self.manual_until = (0.0, 0.0), 0.0
        self.frame_t = self.odom_t = -math.inf
        self.frames, self.fps, self.fps_t = 0, 0.0, time.monotonic()
        self.quiet_since = 0.0
        self.have_info = False
        self.ir_left, self.ir_right, self.ir_t = False, False, -math.inf

        self.cmd_pub = self.create_publisher(Twist, d('cmd_vel_topic', '/cmd_vel').value, 10) if self.drive else None
        self.create_subscription(CompressedImage, d('image_topic', '/camera/image_raw/compressed').value,
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, d('camera_info_topic', '/camera/camera_info').value, self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, d('odom_topic', '/odom').value, self._on_odom, 10)
        self.create_subscription(Bool, d('ir_left_topic', '/fire_resq/ir_left').value, self._on_ir_left, 10)
        self.create_subscription(Bool, d('ir_right_topic', '/fire_resq/ir_right').value, self._on_ir_right, 10)
        self.create_timer(0.025, self._on_control)
        self.create_timer(0.25, self._on_slow)
        self.get_logger().info(f"dashboard: http://localhost:{self.dash.server.server_port}"
                               f"{'' if self.drive else '   (DRY RUN: the motors are never commanded)'}")

    # ------------------------------------------------------------------ settings
    def _get(self, key: str):
        group, name = key.split('.')
        return getattr({'vision': self.vp, 'seek': self.sp, 'camera': self.cam}[group], name)

    def _set(self, key: str, value) -> None:
        lo, hi = TUNABLE[key]
        group, name = key.split('.')
        kind = type(self._get(key))
        value = kind(min(max(float(value), lo), hi)) if kind is not bool else bool(float(value))
        if group == 'camera':
            self.cam = dataclasses.replace(self.cam, **{name: value})
        else:
            setattr({'vision': self.vp, 'seek': self.sp}[group], name, value)

    def _load_settings(self) -> None:
        if self.settings_file is None or not self.settings_file.exists():
            return
        try:
            saved = json.loads(self.settings_file.read_text())
            for key, value in saved.items():
                if key in TUNABLE:
                    self._set(key, value)
            if isinstance(saved.get('follow'), list):
                self.follow = set(saved['follow']) & set(COLOURS)
            self.get_logger().info(f'settings tuned on the dashboard loaded from {self.settings_file}')
        except (ValueError, OSError) as e:
            self.get_logger().warn(f'{self.settings_file}: ignored ({e})')

    def _save_settings(self) -> None:
        if self.settings_file is not None:
            try:
                self.settings_file.write_text(json.dumps({**{k: self._get(k) for k in TUNABLE},
                                                          'follow': sorted(self.follow)}, indent=1))
            except OSError as e:
                self.get_logger().warn(f'could not save settings: {e}', throttle_duration_sec=30.0)

    # ------------------------------------------------------------------ ROS in
    def _on_info(self, msg: CameraInfo) -> None:
        if self.have_info or msg.k[0] <= 0:
            return
        self.have_info = True
        self.cam = dataclasses.replace(self.cam, f=float(msg.k[0]), cx=float(msg.k[2]), width=int(msg.width), height=int(msg.height))

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.tracker.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y,
                             math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.odom_t = time.monotonic()

    def _on_ir(self) -> None:
        self.ir_t = time.monotonic()
        self.seeker.on_ir(self.ir_t, self.ir_left, self.ir_right)

    def _on_ir_left(self, msg: Bool) -> None:
        self.ir_left = msg.data
        self._on_ir()

    def _on_ir_right(self, msg: Bool) -> None:
        self.ir_right = msg.data
        self._on_ir()

    def _on_image(self, msg: CompressedImage) -> None:
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        cam, t = self.cam, time.monotonic()
        if bgr.shape[1] != cam.width or bgr.shape[0] != cam.height:      # no CameraInfo yet: scale the nominal camera
            k = bgr.shape[1] / cam.width
            cam = self.cam = dataclasses.replace(cam, f=cam.f * k, cx=cam.cx * k, width=bgr.shape[1], height=bgr.shape[0],
                                                 horizon_row=cam.horizon_row * k)
        blobs = find_bottles(bgr, cam, self.vp)
        if self.pick_lab is not None:
            picked = find_picked(bgr, self.pick_lab, self.pick_at, cam, self.vp)
            if picked is not None:
                blobs.append(picked)
                self.pick_at = (picked.x + picked.w / 2.0, picked.y + picked.h / 2.0)
        for b in blobs:                                                  # a colour the operator has switched off
            if not b.why and b.hue != PICKED and colour_name(b.hue) not in self.follow:
                b.why, b.partial = 'colour off', False
        self.last_bgr, self.last_blobs = bgr, blobs
        scan, not_floor = self.floor.scan(bgr, cam, self.vp)
        below = not_floor[int(math.ceil(cam.horizon_row / self.floor.scale)):]     # the part of the picture where floor can be
        self.floor_visible = float(1.0 - below.mean()) if below.size else None
        points = np.stack([scan.x, scan.y, scan.bearing], axis=1)
        seen = [b for b in blobs if self._usable(b) or b.partial]
        look = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (32, 24), interpolation=cv2.INTER_AREA)
        self.seeker.on_frame(t, [self._sighting(b) for b in seen], points, look, self.floor_visible)
        if self.seeker.steady(t) and t - self.odom_t < 1.0:
            self.floor_map.add_scan(self.tracker.pose, points)

        labels = []
        for blob in blobs:                                               # which remembered bottle is this box?
            known = min(self.seeker.bottles, key=lambda k: hue_distance(k.hue, blob.hue), default=None)
            usable = self._usable(blob) or (blob.partial and self.seeker.accepts_partial(blob.hue))
            match = known is not None and usable and hue_distance(known.hue, blob.hue) <= self.sp.hue_tol
            labels.append({'name': known.name, 'status': self.seeker.status_of(known) if known.confirmed else 'new'}
                          if match else None)
        countdown = self.seeker.wait_until - t if self.seeker.state == WAIT else None
        shown = annotate(bgr, cam, blobs, labels, scan if self.sp.avoid else None, self.sp.obstacle_range_m,
                         self.sp.half_width_m, countdown)
        self.dash.camera.put(cv2.imencode('.jpg', shown, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
        self.frame_t, self.frames = t, self.frames + 1

    def _usable(self, b) -> bool:
        """A bottle for the behaviour: passed the rules, or is the colour of one the operator clicked (no rules), or
        of one already accepted and fails only a shape rule (the detector does not get every frame right)."""
        if not b.why:
            return True
        if any(hue_distance(b.hue, hue) <= self.sp.hue_tol for hue in self.forced):
            return not b.cut_side or self.seeker.accepts_partial(b.hue)
        return b.why in SOFT and self.seeker.knows(b.hue) and colour_name(b.hue) in self.follow

    def _sighting(self, b) -> Sighting:
        cam = self.cam
        return Sighting(b.hue, b.bearing, b.range_m, b.w / cam.width, cam.bearing(b.x), cam.bearing(b.x + b.w),
                        b.partial or (bool(b.why) and b.cut_side), b.area / float(cam.width * cam.height))

    def _pick(self, fx: float, fy: float, t: float) -> None:
        """The operator clicked the camera picture at (fx, fy), fractions of its width and height."""
        if self.last_bgr is None:
            return
        cam = self.cam
        u, v = fx * cam.width, fy * cam.height
        under = [b for b in self.last_blobs if b.hue != PICKED and b.x <= u <= b.x + b.w and b.y <= v <= b.y + b.h]
        if under:                                                # a box (accepted or rejected) is under the click
            blob = min(under, key=lambda b: (bool(b.why), b.w * b.h))
            self.pick_lab = None
        else:                                                    # nothing there for the detector: learn that spot's colour
            self.pick_lab, self.pick_at = pick_colour(self.last_bgr, u, v), (u, v)
            blob = find_picked(self.last_bgr, self.pick_lab, self.pick_at, cam, self.vp)
        hue = blob.hue if blob is not None else PICKED
        self.forced = [hue]
        self.seeker.pick(t, hue, self._sighting(blob) if blob is not None else None)

    # ------------------------------------------------------------------ control
    def _on_control(self) -> None:
        t = time.monotonic()
        while True:
            try:
                self._on_button(self.dash.commands.get_nowait(), t)
            except queue.Empty:
                break
        v, w = self.seeker.command(t)
        if not self.seeker.moving and t < self.manual_until:
            v, w = self.manual
        if self.cmd_pub is None:
            return
        if v or w:
            self.quiet_since = t
        if t - self.quiet_since < 1.0:                                   # zeros for a second after the last motion, then silence
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            self.cmd_pub.publish(msg)

    def _on_button(self, cmd: dict, t: float) -> None:
        action = cmd.get('action')
        if action == 'start':
            self.forced, self.pick_lab = [], None
            self.tracker.reset()
            self.floor_map.reset()
            self.floor.forget()
            self.seeker.start(t)
        elif action == 'stop':
            self.seeker.stop()
            self.manual_until = 0.0
        elif action == 'reset':
            self.forced, self.pick_lab = [], None
            self.seeker.reset()
            self.tracker.reset()
            self.floor_map.reset()
        elif action == 'floor':
            self.floor.forget()
        elif action == 'drive' and not self.seeker.running:
            v = max(-1.0, min(1.0, float(cmd.get('v', 0.0)))) * self.sp.drive_speed
            w = max(-1.0, min(1.0, float(cmd.get('w', 0.0)))) * self.sp.turn_speed
            self.manual, self.manual_until = (v, w * (self.sp.left_boost if w > 0 else self.sp.right_boost)), t + 0.4
        elif action == 'follow' and cmd.get('name') in COLOURS:
            (self.follow.add if cmd.get('on') else self.follow.discard)(cmd['name'])
            self._save_settings()
        elif action == 'pick':
            try:
                self._pick(min(max(float(cmd['x']), 0.0), 1.0), min(max(float(cmd['y']), 0.0), 1.0), t)
            except (KeyError, TypeError, ValueError):
                return
        elif action == 'set' and cmd.get('key') in TUNABLE:
            try:
                self._set(cmd['key'], cmd.get('value'))
            except (TypeError, ValueError):
                return
            self._save_settings()

    def _on_slow(self) -> None:
        t, sk = time.monotonic(), self.seeker
        if t - self.fps_t >= 1.0:
            self.fps, self.frames, self.fps_t = self.frames / (t - self.fps_t), 0, t
        x, y, _ = self.tracker.pose
        bottles = [{'id': b.id, 'name': b.name, 'hue': b.hue, 'rgb': colour_rgb(b.hue), 'status': sk.status_of(b),
                    'visible': t - b.seen_t < 0.6, 'x': b.x, 'y': b.y, 'cover': round(b.cover_frac, 2),
                    'range': round(b.range_m if t - b.seen_t < 0.6 and math.isfinite(b.range_m) else math.hypot(b.x - x, b.y - y), 2)}
                   for b in sk.bottles if b.confirmed]
        hfov = 2.0 * math.atan2(self.cam.width / 2.0, self.cam.f)
        self.dash.map.put(cv2.imencode('.jpg', draw_map(560, self.tracker, self.floor_map, bottles, hfov),
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes())
        self.dash.state = json.dumps({
            'state': sk.state, 'headline': sk.headline(t), 'note': sk.note, 'running': sk.running,
            'drive': self.drive, 'camera_ok': t - self.frame_t < 1.5, 'robot_ok': t - self.odom_t < 1.5,
            'fps': round(self.fps, 1), 'elapsed': round(t - sk.t_start, 0) if sk.state not in (IDLE, DONE) else None,
            'bottles': bottles, 'events': sk.events[-12:],
            'turning': {'rate_deg_s': round(math.degrees(sk.turn_rate)), 'odom_gain': round(self.tracker.yaw_gain, 2),
                        'reversed': sk.turn_sign < 0, 'power': round(sk.turn_scale, 2)},
            'ir': {'left': self.ir_left, 'right': self.ir_right, 'fresh': t - self.ir_t < self.sp.ir_fresh_s},
            'settings': {k: self._get(k) for k in TUNABLE}, 'limits': TUNABLE,
            'colours': [[c, c in self.follow] for c in COLOURS],
            'floor_visible': None if self.floor_visible is None else round(self.floor_visible, 2),
        })

    def halt(self) -> None:
        """Tell the robot to stop, several times (the driver and the firmware would stop it anyway within 0.5 s)."""
        if self.cmd_pub is not None:
            for _ in range(5):
                self.cmd_pub.publish(Twist())
                time.sleep(0.02)
        self.dash.close()


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.halt()
        except Exception:                                            # noqa: BLE001 - the context may already be gone
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
