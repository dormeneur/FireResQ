"""Measure what the full Phase 4 stack costs: rates, map-update behaviour, CPU, real-time factor.

  python3 tests/sim/experiments/nav_performance.py [extra arena_nav launch args...]

Runs arena + depth->scan + SLAM (scan-matching) + Nav2, measures it idle, then while Nav2 drives a
goal. CPU is in cores (cputime delta / wall time) per process family.
"""
import subprocess
import sys
import time
from pathlib import Path

import rclpy

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from simlib import Bot, GroundTruth, run_sim  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402

FAMILIES = {'gazebo (server)': 'gz sim', 'ros_gz bridges': 'ros_gz_bridge', 'depth->scan': 'depthimage_to_laserscan',
            'slam_toolbox': 'async_slam_toolbox', 'controller_server': 'controller_server', 'planner_server': 'planner_server',
            'bt_navigator': 'bt_navigator', 'behavior_server': 'behavior_server', 'robot_state_pub': 'robot_state_publisher'}


def cpu():
    out = subprocess.run(['ps', '-eo', 'cputimes=,args='], capture_output=True, text=True).stdout
    agg = {k: 0 for k in FAMILIES}
    for ln in out.splitlines():
        parts = ln.strip().split(None, 1)
        if len(parts) == 2:
            for k, pat in FAMILIES.items():
                if pat in parts[1]:
                    agg[k] += int(parts[0])
    return agg


def rtf():
    r = subprocess.run(['gz', 'topic', '-e', '-t', '/stats', '-n', '4'], capture_output=True, text=True, timeout=20).stdout
    v = [float(ln.split(':')[1]) for ln in r.splitlines() if 'real_time_factor' in ln]
    return sum(v) / len(v)


sc = load_scenario(resolve_scenario('default'))
rclpy.init()
with run_sim(sys.argv[1:], launch_file='arena_nav.launch.py'):
    bot, gt = Bot(True, scan=True, nav=True), GroundTruth()
    bot.wait_for(lambda: bot.odom is not None and gt.get() is not None and bot.map_pose() is not None and bot.grids, 150, 'stack')
    map_stamps = []
    bot.create_subscription(type(bot.grids[-1]), '/map', lambda m: map_stamps.append(time.time()),
                            __import__('rclpy.qos', fromlist=['QoSProfile']).QoSProfile(
                                depth=1, durability=__import__('rclpy.qos', fromlist=['DurabilityPolicy']).DurabilityPolicy.TRANSIENT_LOCAL))
    bot.spin_wall(5)

    def window(label, seconds, action=None):
        bot.counts.clear(); map_stamps.clear()
        c0, w0, s0 = cpu(), time.time(), bot.simt
        if action:
            action()
        else:
            bot.spin_wall(seconds)
        c1, w1, s1 = cpu(), time.time(), bot.simt
        wall, sim = w1 - w0, s1 - s0
        print(f"\n--- {label}  ({wall:.0f} s wall, {sim:.0f} s sim)")
        print(f"  rates (per sim second): depth {bot.counts['depth'] / sim:5.1f} Hz | scan {bot.counts['scan'] / sim:5.1f} Hz | rgb {bot.counts['rgb'] / sim:5.1f} Hz | "
              f"odom {bot.counts['odom'] / sim:5.1f} Hz | joint_states {bot.counts['joint_states'] / sim:5.1f} Hz")
        gaps = [b - a for a, b in zip(map_stamps, map_stamps[1:])]
        print(f"  /map updates: {len(map_stamps)} in {wall:.0f} s" + (f" (every {sum(gaps) / len(gaps):.1f} s)" if gaps else ""))
        print(f"  real-time factor {rtf():.2f}")
        tot = 0.0
        row = []
        for k in FAMILIES:
            v = (c1[k] - c0[k]) / wall
            tot += v
            row.append(f"{k} {v:.2f}")
        print("  CPU (cores): " + " | ".join(row) + f" | TOTAL {tot:.2f}")

    window("idle (SLAM + Nav2 active, robot stationary)", 20)
    mx, my = sc.world_to_odom(0.6, -2.0)
    window("Nav2 driving a goal (G2, a detour)", 0, lambda: print(f"  goal result: {bot.navigate_to(mx, my, 0.0 - sc.robot_start.yaw, timeout=90)}"))
    gt.stop()
    bot.destroy_node()
rclpy.shutdown()
