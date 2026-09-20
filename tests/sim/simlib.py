"""Helpers for the simulation regression tests.

Ground truth (Gazebo's own pose stream) is read through the `gz` CLI and used ONLY to judge
the robot - never fed back into a ROS topic. Robot parameters are parsed from the same xacro
files the robot is built from, so a test cannot silently disagree with the source of truth.
"""
import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
from collections import defaultdict, deque
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from fire_resq_interfaces.msg import DetectionArray, WorldState
from fire_resq_interfaces.srv import UpdateVictimStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage

REPO = Path(__file__).resolve().parents[2]
WORLD = 'rescue_arena'


# ------------------------------------------------------------------ source-of-truth parameters
def read_xacro_props(*files):
    """Properties and ARG DEFAULTS from xacro files, as floats where numeric. A property whose
    value is `$(arg X)` resolves to X's default, so tests see what an un-overridden launch uses."""
    out, args = {}, {}
    for f in files:
        text = Path(f).read_text()
        for name, val in re.findall(r'<xacro:arg\s+name="(\w+)"\s+default="([^"]+)"', text):
            args[name] = val
        for name, val in re.findall(r'<xacro:property\s+name="(\w+)"\s+value="([^"]+)"', text):
            out[name] = val
    for name, val in list(out.items()):
        m = re.fullmatch(r'\$\(arg (\w+)\)', val)
        if m:
            val = args[m.group(1)]
        try:
            out[name] = float(val)
        except ValueError:
            out[name] = val
    return out


P = read_xacro_props(REPO / 'src/fire_resq_description/urdf/parameters.xacro',
                     REPO / 'simulation/urdf/gz_sensors.xacro')
WHEEL_R, SEP = P['wheel_radius'], P['wheel_separation']
MAX_LIN, MAX_ANG = P['max_linear_velocity'], P['max_angular_velocity']


def yaw_of(q):
    return math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))


def roll_of(q):
    return math.atan2(2 * (q['w'] * q['x'] + q['y'] * q['z']), 1 - 2 * (q['x'] ** 2 + q['y'] ** 2))


def pitch_of(q):
    return math.asin(max(-1.0, min(1.0, 2 * (q['w'] * q['y'] - q['z'] * q['x']))))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _quat(d):
    q = d.get('orientation', {})
    return {k: q.get(k, 0.0) for k in 'xyzw'} if q else {'x': 0, 'y': 0, 'z': 0, 'w': 1}


def _pose_tuple(e):
    pos, q = e.get('position', {}), _quat(e)
    return (pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), roll_of(q), pitch_of(q), yaw_of(q))


# ------------------------------------------------------------------ simulation process
# Processes that mean "a simulation stack is (still) running". Matched against the command line
# of real programs only: a shell wrapper (`bash -c '... gz sim ...'`) merely CONTAINING these
# words - like the one that launched pytest - must not count.
STACK_RE = (r'gz sim|/lib/(nav2_\w+|slam_toolbox|depthimage_to_laserscan|ros_gz_bridge|robot_state_publisher|rviz2)/'
            r'|async_slam_toolbox_node')


def stray_sim_processes():
    out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
    return [ln.strip() for ln in out.splitlines()
            if re.search(STACK_RE, ln) and 'pytest' not in ln
            and not re.match(r'\s*\d+\s+(/bin/)?bash\b', ln) and 'shell-snapshots' not in ln]


class SimHandle:
    def __init__(self, proc, log_path):
        self.proc, self.log_path = proc, log_path

    def log(self):
        return Path(self.log_path).read_text(errors='replace')


def _terminate(proc):
    pgid = proc.pid
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 8), (signal.SIGKILL, 5)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        t0 = time.time()
        while time.time() - t0 < wait:
            if subprocess.run(['pgrep', '-g', str(pgid)], capture_output=True).returncode != 0:
                return
            time.sleep(0.3)


@contextmanager
def run_sim(args=(), launch_file='arena.launch.py'):
    """Launch the simulation headless in its own process group; always tear it down."""
    stray = stray_sim_processes()
    if stray:
        raise RuntimeError('stray simulation processes are running; stop them first:\n  ' + '\n  '.join(stray))
    env = os.environ.copy()
    if os.path.exists('/dev/dxg'):      # WSL2: use the GPU via D3D12 (docs/Environment.md)
        env.setdefault('GALLIUM_DRIVER', 'd3d12')
        env.setdefault('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA')
    log = tempfile.NamedTemporaryFile('w+', suffix='.simlog', delete=False)
    proc = subprocess.Popen(['ros2', 'launch', 'fire_resq_simulation', launch_file, 'gui:=false', *args],
                            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        yield SimHandle(proc, log.name)
    finally:
        _terminate(proc)


def start_group(cmd):
    """Start a command in its own process group so the WHOLE launch tree can be stopped."""
    env = os.environ.copy()
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, start_new_session=True)


def stop_group(proc, wait=10):
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    t0 = time.time()
    while time.time() - t0 < wait and subprocess.run(['pgrep', '-g', str(proc.pid)], capture_output=True).returncode == 0:
        time.sleep(0.3)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def lifecycle_state(node):
    out = subprocess.run(['ros2', 'lifecycle', 'get', node], capture_output=True, text=True, timeout=15).stdout
    return out.split()[0] if out.split() else 'unknown'


# ------------------------------------------------------------------ ground truth (gz CLI)
class GroundTruth:
    """Streams the robot's true pose. Test-only reference."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.hist = []
        self.p = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info', '--json-output'],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        dec, buf = json.JSONDecoder(), ''
        for chunk in self.p.stdout:
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
                    if e.get('name') == 'fire_resq':
                        rec = _pose_tuple(e)
                        with self.lock:
                            self.latest = rec
                            self.hist.append(rec)

    def get(self):
        with self.lock:
            return self.latest

    def mark(self):
        with self.lock:
            return len(self.hist)

    def since(self, m):
        with self.lock:
            return list(self.hist[m:])

    def stop(self):
        self.p.terminate()


def entity_poses(names, timeout=25):
    """One-shot true poses (x,y,z,roll,pitch,yaw) of the named MODELS, incl. static ones."""
    t0, found = time.time(), {}
    while time.time() - t0 < timeout:
        r = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/pose/info', '-n', '1', '--json-output'],
                           capture_output=True, text=True, timeout=20)
        try:
            for e in json.loads(r.stdout).get('pose', []):
                if e.get('name') in names and e['name'] not in found:
                    found[e['name']] = _pose_tuple(e)
        except ValueError:
            pass
        if set(names) <= set(found):
            return found
        time.sleep(0.5)
    return found


# ------------------------------------------------------------------ colour classes
# Measured from rendered frames (see docs/Implementation_Plan.md Phase 3): every non-target
# pixel has saturation <= 18, so these masks cannot fire on floor, walls, obstacles or sky.
def class_masks(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return {
        'fire': (h <= 30) & (s >= 200) & (v >= 200),
        'victim': (h >= 100) & (h <= 125) & (s >= 100) & (v >= 40),
        'safe_zone': (h >= 55) & (h <= 80) & (s >= 100) & (v >= 60),
    }


# ------------------------------------------------------------------ the test robot client
class Bot(Node):
    def __init__(self, depth=True, scan=False, nav=False, perception=False, world=False):
        super().__init__('sim_test_bot')
        self.detections = deque(maxlen=600)     # (sim time received, DetectionArray)
        self.world_states = deque(maxlen=100)   # (sim time received, WorldState)
        self._status_client = None
        self.grids = deque(maxlen=5)
        self._nav_client = None
        self.map_odom = []          # (t, x, y, yaw) of map->odom, to judge how erratic SLAM's correction is
        self.depth_enabled = depth
        self.scans = deque(maxlen=60)
        self.depth_stamps = deque(maxlen=200)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.simt = 0.0
        self.js = {}
        self.msgs = {}
        self.counts = defaultdict(int)
        self.depth_buf = deque(maxlen=12)
        self.rgb_buf = deque(maxlen=4)
        self.peak_v = self.peak_w = 0.0
        self.tf_dyn = defaultdict(list)
        self.tf_static = set()
        s = qos_profile_sensor_data
        self.create_subscription(Odometry, '/odom', self._odom, s)
        self.create_subscription(JointState, '/joint_states', self._js, s)
        self.create_subscription(Image, '/camera/image_raw', self._rgb, s)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._keep('rgb_info'), s)
        if depth:
            self.create_subscription(Image, '/camera/depth/image_raw', self._depth, s)
            self.create_subscription(CameraInfo, '/camera/depth/camera_info', self._keep('depth_info'), s)
        if scan:
            self.create_subscription(LaserScan, '/scan', self._scan, s)
        if perception:
            self.create_subscription(DetectionArray, '/fire_resq/detections', self._detections, 10)
        if world:
            self.create_subscription(WorldState, '/fire_resq/world_state', self._world, 10)
            self._status_client = self.create_client(UpdateVictimStatus, '/fire_resq/update_victim_status')
        if nav:
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.create_subscription(OccupancyGrid, '/map', lambda m: self.grids.append(m),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TFMessage, '/tf', self._tf, 300)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

    # -- callbacks
    def _odom(self, m):
        self.odom = m
        self.simt = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.counts['odom'] += 1
        self.peak_v = max(self.peak_v, abs(m.twist.twist.linear.x))
        self.peak_w = max(self.peak_w, abs(m.twist.twist.angular.z))

    def _js(self, m):
        self.counts['joint_states'] += 1
        self.js.update(zip(m.name, m.position))

    def _rgb(self, m):
        self.counts['rgb'] += 1
        self.rgb_buf.append(m)
        self.msgs['rgb'] = m

    def _world(self, m):
        self.counts['world_state'] += 1
        self.world_states.append((self.simt, m))

    @property
    def world(self):
        """The newest WorldState, or None."""
        return self.world_states[-1][1] if self.world_states else None

    def set_status(self, victim_id, status, timeout=5.0):
        """Call the world model's UpdateVictimStatus service (what the rescue FSM will do). Returns the response."""
        req = UpdateVictimStatus.Request()
        req.victim_id, req.new_status = victim_id, int(status)
        fut = self._status_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        assert fut.done(), 'UpdateVictimStatus timed out'
        return fut.result()

    def _detections(self, m):
        self.counts['detections'] += 1
        self.detections.append((self.simt, m))

    def _scan(self, m):
        self.counts['scan'] += 1
        self.scans.append(m)

    def _depth(self, m):
        self.counts['depth'] += 1
        self.depth_stamps.append((m.header.stamp.sec, m.header.stamp.nanosec))
        self.depth_buf.append(m)
        self.msgs['depth'] = m

    def _keep(self, k):
        return lambda m: self.msgs.setdefault(k, m)

    def _tf(self, m):
        for t in m.transforms:
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            self.tf_dyn[(t.header.frame_id, t.child_frame_id)].append(stamp)
            if (t.header.frame_id, t.child_frame_id) == ('map', 'odom'):
                q = t.transform.rotation
                self.map_odom.append((stamp, t.transform.translation.x, t.transform.translation.y,
                                      yaw_of({'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w})))

    def _tf_static(self, m):
        for t in m.transforms:
            self.tf_static.add((t.header.frame_id, t.child_frame_id))

    # -- spinning / driving
    def spin_wall(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, cond, timeout, what='condition'):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return
        raise TimeoutError(f'timed out after {timeout}s waiting for {what}')

    def send(self, vx=0.0, wz=0.0):
        m = Twist()
        m.linear.x, m.angular.z = float(vx), float(wz)
        self.pub.publish(m)

    def spin_sim(self, sim_s, vx=0.0, wz=0.0, on_tick=None):
        t0 = self.simt
        while self.simt - t0 < sim_s:
            self.send(vx, wz)
            rclpy.spin_once(self, timeout_sec=0.02)
            if on_tick:
                on_tick()

    def stop(self, sim_s=1.5):
        self.spin_sim(sim_s)

    def pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of({'x': o.orientation.x, 'y': o.orientation.y,
                                                    'z': o.orientation.z, 'w': o.orientation.w})

    def drive_distance(self, dist, speed=0.2):
        """Closed-loop on odometry (valid: see test_robot). Negative dist drives backwards."""
        x0, y0, _ = self.pose()
        sgn = 1.0 if dist >= 0 else -1.0
        while True:
            x, y, _ = self.pose()
            err = abs(dist) - math.hypot(x - x0, y - y0)
            if err < 0.004:
                break
            self.send(sgn * min(speed, max(0.05, 1.5 * err)), 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop()

    def go_to(self, x, y, speed=0.25):
        """Drive to an odom-frame point: face it, then drive straight there (closed loop on odometry).
        A TEST tool for supplying a route; the robot under test never decides where to go."""
        px, py, _ = self.pose()
        dist = math.hypot(x - px, y - py)
        if dist < 0.03:
            return
        self.turn_to(math.atan2(y - py, x - px), rate=1.0, settle=0.4)
        self.drive_distance(dist, speed)

    def navigate_to(self, x, y, yaw=0.0, timeout=90.0, on_tick=None):
        """Send a Nav2 NavigateToPose goal (map frame) and wait for it.

        Returns dict(accepted, status, seconds). `on_tick` is called every spin so a test can
        record the TRUE robot pose while Nav2 works. The goal is chosen by the TEST."""
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            raise TimeoutError('navigate_to_pose action server not available')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t0 = time.time()

        def wait(fut, limit):
            while not fut.done():
                if time.time() - t0 > limit:
                    return False
                rclpy.spin_once(self, timeout_sec=0.05)
                if on_tick:
                    on_tick()
            return True

        send = self._nav_client.send_goal_async(goal)
        if not wait(send, 15.0):
            return dict(accepted=False, status='send timed out', seconds=time.time() - t0)
        handle = send.result()
        if not handle.accepted:
            return dict(accepted=False, status='rejected', seconds=time.time() - t0)
        res = handle.get_result_async()
        if not wait(res, timeout):
            handle.cancel_goal_async()
            self.spin_wall(1.0)
            return dict(accepted=True, status='timed out', seconds=time.time() - t0)
        self.send(0.0, 0.0)
        return dict(accepted=True, status={4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(res.result().status, str(res.result().status)),
                    seconds=time.time() - t0)

    def map_pose(self):
        """Pose of base_link in the map frame (SLAM/AMCL's estimate), or None if not available."""
        try:
            tr = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return t.x, t.y, yaw_of({'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w})

    def turn_to(self, target, rate=1.0, settle=1.5):
        """Rotate to an ABSOLUTE odom yaw (radians). Absolute, so errors never accumulate."""
        target = wrap(target)
        while True:
            err = wrap(target - self.pose()[2])
            if abs(err) < 0.008:
                break
            self.send(0.0, math.copysign(min(rate, max(0.15, 2.0 * abs(err))), err))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(settle)

    def turn_by(self, angle, rate=1.0, settle=1.5):
        self.turn_to(self.pose()[2] + angle, rate, settle)

    # -- sensing
    def frame_pair(self, max_dt=0.04):
        """Newest RGB and the depth frame closest in time to it (or None)."""
        if not self.rgb_buf or not self.depth_buf:
            return None
        rgb = self.rgb_buf[-1]
        t = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        dep = min(self.depth_buf, key=lambda d: abs(d.header.stamp.sec + d.header.stamp.nanosec * 1e-9 - t))
        dt = abs(dep.header.stamp.sec + dep.header.stamp.nanosec * 1e-9 - t)
        return (rgb, dep) if dt <= max_dt else None

    def wait_pair(self, timeout=4.0):
        """Spin until an RGB frame and a depth frame close in time exist (they are separate sensors,
        so a pair is not guaranteed at any instant)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            pair = self.frame_pair()
            if pair:
                return pair
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    @staticmethod
    def rgb_array(m):
        return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)

    @staticmethod
    def depth_array(m):
        return np.frombuffer(m.data, np.float32).reshape(m.height, m.width)

    def intrinsics(self):
        return np.array(self.msgs['rgb_info'].k).reshape(3, 3)

    def cam_to_odom(self, stamp):
        for _ in range(20):
            try:
                tr = self.tfbuf.lookup_transform('odom', 'camera_optical_frame', stamp)
                break
            except (tf2_ros.ExtrapolationException, tf2_ros.LookupException):
                rclpy.spin_once(self, timeout_sec=0.02)
        else:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R, np.array([t.x, t.y, t.z])

    def detect(self, sc, min_area=40):
        """Colour blob -> depth -> back-projection -> odom -> world. Returns
        [(class, world_x, world_y, world_z_above_axle, area_px)]."""
        pair = self.frame_pair()
        if pair is None:
            return []
        rgb_m, dep_m = pair
        rgb, depth, K = self.rgb_array(rgb_m), self.depth_array(dep_m), self.intrinsics()
        tf = self.cam_to_odom(rgb_m.header.stamp)
        if tf is None:
            return []
        R, T = tf
        out = []
        for cls, mask in class_masks(rgb).items():
            cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cs:
                area = cv2.contourArea(c)
                if area < min_area:
                    continue
                M = cv2.moments(c)
                u, v = M['m10'] / M['m00'], M['m01'] / M['m00']
                # RGB and depth are SEPARATE simulated sensors. Measured in Phase 3: while the
                # camera turns at 0.4 rad/s the depth image is offset ~10 px (+-9) from the RGB
                # image EVEN WHEN THEIR TIMESTAMPS MATCH - a roughly constant ~40 ms content lag
                # that stamps do not reveal. So call detect() only while the robot is STATIONARY
                # (see survey()); the low percentile below is just cheap insurance, because an
                # object is always nearer than its backdrop. Phase 5 perception must respect the
                # same limit (process while slow/stopped, or compensate) - a single depth pixel
                # from a moving camera can land on the wall behind a thin object.
                blob = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(blob, [c], -1, 1, -1)
                vals = depth[blob > 0]
                vals = vals[np.isfinite(vals)]
                if vals.size == 0:
                    continue
                Z = float(np.percentile(vals, 25))
                p_odom = R @ np.array([(u - K[0, 2]) * Z / K[0, 0], (v - K[1, 2]) * Z / K[1, 1], Z]) + T
                wx, wy = sc.odom_to_world(p_odom[0], p_odom[1])
                out.append((cls, wx, wy, float(p_odom[2]), area))
        return out


def match_entities(sc, detections, tol_fire=0.25, tol_victim=0.15):
    """Assign fire/victim detections to scenario entities. Returns (seen_ids, unmatched)."""
    seen, unmatched = set(), []
    for cls, wx, wy, _, area in detections:
        if cls == 'fire':
            cands = {'fire': (sc.fire.x, sc.fire.y)}
            tol = tol_fire
        elif cls == 'victim':
            cands = {v.id: (v.x, v.y) for v in sc.victims}
            tol = tol_victim
        else:
            continue
        name, d = min(((n, math.hypot(wx - x, wy - y)) for n, (x, y) in cands.items()), key=lambda t: t[1])
        (seen.add(name) if d <= tol else unmatched.append((cls, round(wx, 2), round(wy, 2), round(d, 2), area)))
    return seen, unmatched


def survey(bot, sc, step_deg=30.0, revolutions=1.0):
    """Look all the way round from where the robot stands: turn a step, STOP, sample. Looking
    only while stationary avoids the RGB/depth content lag (see detect()), and the closed-loop
    turns leave the robot on its original heading."""
    step = math.radians(step_deg)
    yaw0 = bot.pose()[2]
    dets = []
    for k in range(1, int(round(revolutions * 2 * math.pi / step)) + 1):
        bot.turn_to(yaw0 + k * step, rate=1.0, settle=0.7)      # absolute targets: no drift
        bot.spin_sim(0.5)
        for _ in range(2):
            bot.spin_wall(0.12)
            dets.extend(bot.detect(sc))
    return dets
