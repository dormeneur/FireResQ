"""Phase 4 Step 0 experiment: is scan-matching SLAM viable on this robot's depth-derived scan?

NOT a test (pytest does not collect it) - a reproducible measurement. It launches the arena at a
given camera FOV, runs depthimage_to_laserscan and SLAM Toolbox with several matcher presets, and
drives the SAME route from the SAME start pose for each preset (the robot is teleported back
between them). Ground truth (Gazebo pose, scenario geometry) is used to score:

  * scan  : depth->/scan compared with a ray-cast of the true scenario
  * pose  : SLAM map->base_link vs the true pose after every stage of the route
  * map   : fraction of occupied cells within 5/10 cm of a real surface
  * jumps : largest single-step change of map->odom (how erratic the correction is)
  * cost  : CPU cores used per process, and the simulation real-time factor

  python3 tests/sim/experiments/slam_fov_spike.py <hfov_rad> <out.json> [base|gated]
    base  : A default-style, B sparse+tight, C default+loop-closure, D matching OFF (odometry only)
    gated : E/F skip rotation-only scans (translation gate), default vs tight matcher

Findings are recorded in docs/Implementation_Plan.md (Phase 4). Source the workspace first.
"""
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from simlib import P, Bot, GroundTruth, run_sim, wrap  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402

FOV, OUT = float(sys.argv[1]), Path(sys.argv[2])
MODE = sys.argv[3] if len(sys.argv) > 3 else 'base'
WORK = OUT.parent
sc = load_scenario(resolve_scenario('default'))


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def ray_hit(ox, oy, dx, dy):
    best = 1e9
    hx, hy = sc.arena.size_x / 2, sc.arena.size_y / 2
    for nx, ny, off in ((1, 0, hx), (-1, 0, hx), (0, 1, hy), (0, -1, hy)):
        den = dx * nx + dy * ny
        if den > 1e-9:
            best = min(best, (off - (ox * nx + oy * ny)) / den)
    for o in sc.obstacles:
        c, s = math.cos(-o.yaw), math.sin(-o.yaw)
        lox, loy = (ox - o.x) * c - (oy - o.y) * s, (ox - o.x) * s + (oy - o.y) * c
        ldx, ldy = dx * c - dy * s, dx * s + dy * c
        t0, t1, ok = 0.0, 1e9, True
        for lo, d, h in ((lox, ldx, o.size_x / 2), (loy, ldy, o.size_y / 2)):
            if abs(d) < 1e-12:
                if abs(lo) > h:
                    ok = False
            else:
                a, b = (-h - lo) / d, (h - lo) / d
                t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if ok and t0 <= t1 and t0 > 0:
            best = min(best, t0)
    for cx, cy, r in [(sc.fire.x, sc.fire.y, 0.15)] + [(v.x, v.y, 0.05) for v in sc.victims]:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0 and -b - math.sqrt(disc) > 0:
            best = min(best, -b - math.sqrt(disc))
    return best


def dist_true(px, py):
    hx, hy = sc.arena.size_x / 2, sc.arena.size_y / 2
    d = [abs(abs(px) - hx), abs(abs(py) - hy)]
    for o in sc.obstacles:
        c, s = math.cos(-o.yaw), math.sin(-o.yaw)
        lx, ly = (px - o.x) * c - (py - o.y) * s, (px - o.x) * s + (py - o.y) * c
        d.append(math.hypot(max(abs(lx) - o.size_x / 2, 0), max(abs(ly) - o.size_y / 2, 0)))
    d.append(max(math.hypot(px - sc.fire.x, py - sc.fire.y) - 0.15, 0))
    for v in sc.victims:
        d.append(max(math.hypot(px - v.x, py - v.y) - 0.056, 0))
    return min(d)


def own_group(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def stop_group(p, wait=8):
    try:
        os.killpg(p.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    t0 = time.time()
    while time.time() - t0 < wait and subprocess.run(['pgrep', '-g', str(p.pid)], capture_output=True).returncode == 0:
        time.sleep(0.3)
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def cpu_seconds():
    out = subprocess.run(['ps', '-eo', 'cputimes=,args='], capture_output=True, text=True).stdout
    agg = {}
    for ln in out.splitlines():
        parts = ln.strip().split(None, 1)
        if len(parts) < 2:
            continue
        for key in ('async_slam_toolbox', 'gz sim', 'depthimage_to_laserscan', 'parameter_bridge'):
            if key in parts[1]:
                agg[key] = agg.get(key, 0) + int(parts[0])
    return agg


def rtf():
    r = subprocess.run(['gz', 'topic', '-e', '-t', '/stats', '-n', '3'], capture_output=True, text=True, timeout=20).stdout
    v = [float(ln.split(':')[1]) for ln in r.splitlines() if 'real_time_factor' in ln]
    return sum(v) / len(v) if v else float('nan')


BASE = yaml.safe_load(open('/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'))


def cfg(**kw):
    d = yaml.safe_load(yaml.safe_dump(BASE))
    p = d['slam_toolbox']['ros__parameters']
    p.update(base_frame='base_link', max_laser_range=8.0, scan_buffer_maximum_scan_distance=8.0,
             map_update_interval=1.0, enable_interactive_mode=False, use_sim_time=True,
             minimum_travel_distance=0.0, minimum_travel_heading=0.05, minimum_time_interval=0.1,
             do_loop_closing=False)
    p.update(kw)
    return d


TIGHT = dict(coarse_search_angle_offset=0.06, fine_search_angle_offset=0.005,
             correlation_search_space_dimension=0.2, distance_variance_penalty=0.2, angle_variance_penalty=0.2)
if MODE == 'lap':
    G = dict(minimum_travel_distance=0.1, minimum_travel_heading=3.1)
    CONFIGS = {"L0 current preset": cfg(**G, **TIGHT),
               "L1 travel 0.3 m": cfg(**{**G, 'minimum_travel_distance': 0.3}, **TIGHT),
               "L2 tighter matcher": cfg(**G, **{**TIGHT, 'correlation_search_space_dimension': 0.1, 'distance_variance_penalty': 0.1, 'angle_variance_penalty': 0.1}),
               "L3 small scan buffer": cfg(**G, **TIGHT, scan_buffer_size=3),
               "L4 travel 0.2 + no barycenter": cfg(**{**G, 'minimum_travel_distance': 0.2}, **TIGHT, use_scan_barycenter=False)}
elif MODE == 'lap2':
    G = dict(minimum_travel_heading=3.1)
    CONFIGS = {"L1 travel 0.3 m (repeat)": cfg(**G, minimum_travel_distance=0.3, **TIGHT),
               "L5 travel 0.5 m": cfg(**G, minimum_travel_distance=0.5, **TIGHT),
               "L6 travel 0.4 m": cfg(**G, minimum_travel_distance=0.4, **TIGHT),
               "L1 travel 0.3 m (repeat 2)": cfg(**G, minimum_travel_distance=0.3, **TIGHT)}
elif MODE == 'gated':
    CONFIGS = {"E translation-gated, default matcher": cfg(minimum_travel_distance=0.1, minimum_travel_heading=3.1),
               "F translation-gated, tight matcher": cfg(minimum_travel_distance=0.1, minimum_travel_heading=3.1, **TIGHT)}
else:
    CONFIGS = {"A default-style matcher": cfg(),
               "B sparse + tight matcher": cfg(minimum_travel_heading=0.3, minimum_time_interval=0.5, **TIGHT),
               "C default-style + loop closure": cfg(do_loop_closing=True),
               "D matching OFF (odom only)": cfg(use_scan_matching=False)}

result = {"fov_deg": round(math.degrees(FOV), 1), "mode": MODE, "configs": {}}


def save():
    OUT.write_text(json.dumps(result, indent=1))


rclpy.init()
with run_sim([f'camera_hfov:={FOV}']):
    # scan_height 4: at wide FOV each image row spans more angle, and a 10-row band would start
    # seeing the FLOOR at ~6 m and invent returns. The ray-cast check below would expose it.
    d2l = own_group(['ros2', 'run', 'depthimage_to_laserscan', 'depthimage_to_laserscan_node', '--ros-args',
                     '-r', 'depth:=/camera/depth/image_raw', '-r', 'depth_camera_info:=/camera/depth/camera_info',
                     '-r', 'scan:=/scan', '-p', 'output_frame:=camera_link', '-p', 'scan_height:=4',
                     '-p', 'range_min:=0.2', '-p', 'range_max:=8.0', '-p', 'use_sim_time:=true'])
    bot = Bot(True)
    gt = GroundTruth()
    grids, scans = [], []
    bot.create_subscription(OccupancyGrid, '/map', lambda m: grids.append(m),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                       reliability=ReliabilityPolicy.RELIABLE))
    bot.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)
    try:
        bot.wait_for(lambda: bot.odom is not None and gt.get() is not None and len(scans) > 10, 90, 'sim + scan')
        bot.spin_wall(3)
        m, g = scans[-1], gt.get()
        r = np.array(m.ranges)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        cx, cy = g[0] + P['camera_x'] * math.cos(g[5]), g[1] + P['camera_x'] * math.sin(g[5])
        errs = []
        for i in range(0, len(r), 4):
            if np.isfinite(r[i]):
                e = ray_hit(cx, cy, math.cos(g[5] + ang[i]), math.sin(g[5] + ang[i]))
                if e < 8.0:
                    errs.append(abs(r[i] - e))
        result["scan"] = dict(beams=len(r), fov=round(math.degrees(m.angle_max - m.angle_min), 1),
                              valid_pct=round(100 * float(np.isfinite(r).mean())),
                              median_err_cm=round(float(np.median(errs)) * 100, 2),
                              p95_err_cm=round(float(np.percentile(errs, 95)) * 100, 2))
        print("scan:", result["scan"], flush=True)
        save()
        spawn = sc.robot_start

        def truth_world():
            g = gt.get()
            return (g[0], g[1], g[5])

        for name, d in CONFIGS.items():
            print(f"--- {name}", flush=True)
            subprocess.run(['gz', 'service', '-s', '/world/rescue_arena/set_pose', '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
                            f'name: "fire_resq" position: {{x: {spawn.x}, y: {spawn.y}, z: 0.05}} '
                            f'orientation: {{z: {math.sin(spawn.yaw / 2)}, w: {math.cos(spawn.yaw / 2)}}}'],
                           capture_output=True)
            bot.stop(0.5)
            bot.spin_wall(3)
            path = WORK / 'spike_cfg.yaml'
            yaml.safe_dump(d, open(path, 'w'))
            slam = own_group(['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                              f'slam_params_file:={path}', 'use_sim_time:=true'])
            try:
                grids.clear()
                bot.wait_for(lambda: len(grids) > 0, 60, 'first map')
                bot.spin_wall(2.0)
                o0, g0 = bot.pose(), truth_world()
                Twm = comp(g0, inv(o0))                       # map frame == odom frame at the first scan
                jumps, stop_s = [], threading.Event()

                def sampler():
                    last = None
                    while not stop_s.is_set():
                        try:
                            tr = bot.tfbuf.lookup_transform('map', 'odom', rclpy.time.Time())
                            t, q = tr.transform.translation, tr.transform.rotation
                            cur = (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
                            if last:
                                jumps.append((math.hypot(cur[0] - last[0], cur[1] - last[1]), abs(wrap(cur[2] - last[2]))))
                            last = cur
                        except Exception:
                            pass
                        time.sleep(0.1)

                threading.Thread(target=sampler, daemon=True).start()

                def stage(label):
                    bot.spin_wall(1.0)
                    tr = bot.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
                    t, q = tr.transform.translation, tr.transform.rotation
                    sp = comp(Twm, (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
                    g = truth_world()
                    e = (round(math.hypot(sp[0] - g[0], sp[1] - g[1]) * 100, 1), round(math.degrees(wrap(sp[2] - g[2])), 2))
                    print(f"    {label:20s} pos err {e[0]:6.1f} cm   yaw err {e[1]:+7.2f} deg", flush=True)
                    return e

                cpu0, w0 = cpu_seconds(), time.time()
                stages = {"start": stage("start")}
                if MODE in ('lap', 'lap2'):
                    Two = comp(g0, inv(o0))                      # odom -> world (after the teleport odom != spawn frame)
                    for i, (wx, wy) in enumerate([(0.7, -2.0), (2.0, -2.0), (2.0, 2.0), (-0.3, 2.0), (-0.3, -0.3), (-1.9, -1.9)]):
                        ox_, oy_, _ = comp(inv(Two), (wx, wy, 0.0))
                        bot.go_to(ox_, oy_)
                        stages[f"wp{i + 1}"] = stage(f"waypoint {i + 1}")
                    stages["spin 3"] = stages["wp6"]
                else:
                    bot.spin_sim(2 * math.pi / 0.4, wz=0.4)
                    bot.stop(1.5)
                    stages["spin 1"] = stage("after spin 1")
                    bot.drive_distance(2.55, 0.25)
                    stages["leg 1"] = stage("after 2.55 m")
                    bot.spin_sim(2 * math.pi / 0.4, wz=0.4)
                    bot.stop(1.5)
                    stages["spin 2"] = stage("after spin 2")
                    bot.turn_by(math.pi / 4, settle=0.7)
                    bot.drive_distance(1.1, 0.25)
                    stages["leg 2"] = stage("after turn + 1.1 m")
                    bot.spin_sim(2 * math.pi / 0.4, wz=0.4)
                    bot.stop(1.5)
                    stages["spin 3"] = stage("after spin 3")
                cpu1, w1 = cpu_seconds(), time.time()
                stop_s.set()
                bot.spin_wall(1.5)
                mp = grids[-1]
                a = np.array(mp.data).reshape(mp.info.height, mp.info.width)
                res = mp.info.resolution
                ys, xs = np.nonzero(a > 50)
                ox, oy = mp.info.origin.position.x, mp.info.origin.position.y
                de = np.array([dist_true(*comp(Twm, (ox + (x + 0.5) * res, oy + (y + 0.5) * res, 0.0))[:2])
                               for x, y in zip(xs, ys)]) if len(xs) else np.array([9.9])
                jt = np.array(jumps) if jumps else np.zeros((1, 2))
                result["configs"][name] = dict(
                    stages=stages, final_pos_err_cm=stages["spin 3"][0], final_yaw_err_deg=stages["spin 3"][1],
                    map=dict(occupied=int(len(de)), within5cm_pct=round(100 * float((de <= 0.05).mean()), 1),
                             within10cm_pct=round(100 * float((de <= 0.10).mean()), 1),
                             median_cm=round(float(np.median(de)) * 100, 1), p95_cm=round(float(np.percentile(de, 95)) * 100, 1),
                             known_m2=round(float((a >= 0).sum()) * res * res, 1)),
                    map_odom_max_jump=dict(cm=round(float(jt[:, 0].max()) * 100, 2), deg=round(math.degrees(float(jt[:, 1].max())), 2)),
                    cpu_cores={k: round((cpu1.get(k, 0) - cpu0.get(k, 0)) / (w1 - w0), 2) for k in cpu1},
                    rtf=round(rtf(), 3))
                print("   map:", result["configs"][name]["map"], "| max map->odom jump:",
                      result["configs"][name]["map_odom_max_jump"], "| RTF:", result["configs"][name]["rtf"], flush=True)
                save()
            finally:
                stop_group(slam)
    finally:
        gt.stop()
        bot.destroy_node()
        stop_group(d2l)
rclpy.shutdown()
save()
print("DONE", OUT)
