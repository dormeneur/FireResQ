#!/usr/bin/env python3
"""Quick rescue controller for the small demo arena (./run_sim.sh).

The robot side only: it sees the world through /fire_resq/detections (the real perception node, positions in `odom`),
drives on /odom, and picks up/releases through /fire_resq/set_magnet. It never reads the scenario or Gazebo truth.

  1. From the safe zone (= where it starts) take three still looks and cluster what perception reports.
  2. Rank victims by perceived distance to the fire, closest first.
  3. For each: gate -> stop 0.6 m short, look again to refine -> creep into magnet contact -> attach -> back to the
     gate -> into the safe zone -> release -> reverse out.

ponytail: straight-line driving on odometry, no Nav2/SLAM. Valid because make_scenario.py only accepts layouts whose
gate->victim lines are clear and the arena has no obstacles; a cluttered arena needs the full stack (run_demo.sh).
"""
import math
import sys
import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fire_resq_description.geometry import read_properties
from fire_resq_interfaces.msg import DetectionArray
from fire_resq_interfaces.srv import SetMagnet

RING_R = 0.056                                   # victim steel ring radius (models/victim)
P = read_properties()
CONTACT = P['magnet_x'] + P['magnet_length'] / 2 + RING_R    # base_link -> victim centre when the magnet touches
V_MAX, W_MAX = P['max_linear_velocity'], P['max_angular_velocity']

# All in odom, i.e. relative to the start pose, which is the safe-zone centre facing the arena.
GATE = (0.85, 0.0)
DROPS = [(-0.28, 0.0), (0.0, 0.35), (0.0, -0.35)]   # deepest first, so later drives never pass a dropped victim
SAFE_R = 0.75                                    # detections this close to the start are already safe: ignored
APPROACH = 0.6                                   # stop this far short of a victim and look again


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def cluster(points, gate=0.25, min_hits=3):
    groups = []                                  # [sx, sy, n]
    for x, y in points:
        g = next((g for g in groups if math.hypot(g[0] / g[2] - x, g[1] / g[2] - y) < gate), None)
        if g:
            g[0] += x; g[1] += y; g[2] += 1
        else:
            groups.append([x, y, 1])
    return [(g[0] / g[2], g[1] / g[2]) for g in groups if g[2] >= min_hits]


class Robot(Node):
    def __init__(self):
        super().__init__('quick_rescue')
        self.pose = None                         # x, y, yaw
        self.victims, self.fires, self.recording = [], [], False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(DetectionArray, '/fire_resq/detections', self._dets, 10)
        self.magnet = self.create_client(SetMagnet, '/fire_resq/set_magnet')

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _dets(self, m):
        if not self.recording:
            return
        for d in m.detections:
            if d.position_valid:
                p = (d.position.x, d.position.y)
                if d.class_id == 'fire':
                    self.fires.append(p)
                elif d.class_id == 'victim' and math.hypot(*p) > SAFE_R:
                    self.victims.append(p)

    # ---- motion ------------------------------------------------------------------------------------------------
    def cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def stop(self):
        for _ in range(3):
            self.cmd()
            time.sleep(0.02)

    def turn_to(self, yaw, tol=0.03):
        while abs(e := wrap(yaw - self.pose[2])) > tol:
            self.cmd(0, math.copysign(clamp(2.5 * abs(e), 0.25, W_MAX * 0.9), e))
            time.sleep(0.02)
        self.stop()

    def face(self, p):
        self.turn_to(math.atan2(p[1] - self.pose[1], p[0] - self.pose[0]))

    def drive_to(self, p, stop_short=0.0, speed=V_MAX):
        """Drive forward to `stop_short` metres before p, steering at p the whole way."""
        self.face(p)
        x0, y0 = self.pose[:2]
        L = math.hypot(p[0] - x0, p[1] - y0) or 1e-6
        u = ((p[0] - x0) / L, (p[1] - y0) / L)
        while True:
            x, y, yaw = self.pose
            rem = (p[0] - x) * u[0] + (p[1] - y) * u[1] - stop_short
            if rem < 0.004:
                break
            e = wrap(math.atan2(p[1] - y, p[0] - x) - yaw) if rem > 0.08 else 0.0
            self.cmd(clamp(2.0 * rem, 0.05, speed), clamp(3.0 * e, -W_MAX, W_MAX))
            time.sleep(0.02)
        self.stop()

    def reverse(self, dist):
        x0, y0 = self.pose[:2]
        while math.hypot(self.pose[0] - x0, self.pose[1] - y0) < dist:
            self.cmd(-0.25, 0)
            time.sleep(0.02)
        self.stop()

    # ---- sensing / actuation ---------------------------------------------------------------------------------
    def look(self, settle=0.4, window=1.0):
        """Stand still and collect placed detections (perception withholds positions while turning)."""
        self.stop()
        time.sleep(settle)
        self.victims, self.fires, self.recording = [], [], True
        time.sleep(window)
        self.recording = False
        return list(self.victims), list(self.fires)

    def set_magnet(self, attach):
        fut = self.magnet.call_async(SetMagnet.Request(attach=attach))
        t_end = time.time() + 6
        while not fut.done() and time.time() < t_end:
            time.sleep(0.02)
        return fut.done() and fut.result().success


def log(t0, msg):
    print(f'[{time.time() - t0:6.1f}s] {msg}', flush=True)


def mission(r):
    t_wait = time.time() + 60
    while (r.pose is None or not r.magnet.service_is_ready()) and time.time() < t_wait:
        time.sleep(0.1)
    if r.pose is None or not r.magnet.service_is_ready():
        sys.exit('no /odom or /fire_resq/set_magnet after 60 s - is the simulation up?')
    t0 = time.time()

    log(t0, 'surveying from the safe zone')
    vic, fire = [], []
    for yaw in (0.0, 0.65, -0.65, 1.2, -1.2):
        r.turn_to(yaw)
        v, f = r.look()
        vic += v; fire += f
        if yaw == -0.65 and len(cluster(vic)) >= 3 and cluster(fire):
            break
    victims, fires = cluster(vic), cluster(fire, min_hits=2)
    if not fires or not victims:
        sys.exit(f'survey failed: {len(fires)} fire(s), {len(victims)} victim(s) seen')
    fx, fy = fires[0]
    order = sorted(victims, key=lambda v: math.hypot(v[0] - fx, v[1] - fy))
    log(t0, f'fire at ({fx:.2f}, {fy:.2f}); {len(order)} victims, rescue order (closest to fire first):')
    for i, v in enumerate(order, 1):
        log(t0, f'   #{i}  ({v[0]:.2f}, {v[1]:.2f})  {math.hypot(v[0] - fx, v[1] - fy):.2f} m from fire')

    rescued = 0
    for i, v in enumerate(order, 1):
        log(t0, f'victim #{i}: driving out')
        r.drive_to(GATE)
        r.drive_to(v, stop_short=APPROACH)
        r.face(v)
        near = [c for c in cluster(r.look()[0], min_hits=2) if math.hypot(c[0] - v[0], c[1] - v[1]) < 0.3]
        if near:
            v = min(near, key=lambda c: math.hypot(c[0] - v[0], c[1] - v[1]))
        r.drive_to(v, stop_short=CONTACT - 0.01, speed=0.2)
        ok = r.set_magnet(True)
        for _ in range(4):                       # not flush yet: creep a centimetre and try again
            if ok:
                break
            r.cmd(0.05, 0)
            time.sleep(0.2)
            r.stop()
            ok = r.set_magnet(True)
        if not ok:
            log(t0, f'victim #{i}: magnet could not attach - skipping')
            r.reverse(0.3)
            continue
        log(t0, f'victim #{i}: picked up, carrying to the safe zone')
        r.drive_to(GATE)
        r.drive_to(DROPS[rescued], stop_short=CONTACT)
        if not r.set_magnet(False):
            log(t0, f'victim #{i}: release NOT confirmed')
        rescued += 1
        log(t0, f'victim #{i}: released in the safe zone')
        r.reverse(0.3)

    log(t0, f'MISSION COMPLETE: {rescued}/{len(order)} victims in the safe zone, {time.time() - t0:.0f} s')


def main():
    rclpy.init()
    r = Robot()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(r)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        mission(r)
    except (Exception, KeyboardInterrupt):
        if rclpy.ok():
            raise
        print('interrupted', flush=True)        # Ctrl-C: rclpy has already shut its context down
    finally:
        if rclpy.ok():
            r.stop()
        ex.shutdown()
        spin.join(2)
        r.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
