"""Diagnostic: how does SLAM's pose error evolve while Nav2 drives, and what is the robot doing
(linear/angular velocity) when it grows?  Sends the same goals as tests/sim/test_nav2.py.

  python3 tests/sim/experiments/nav2_localization_diag.py [extra arena_nav launch args...]
"""
import math
import sys
from pathlib import Path

import rclpy

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from simlib import Bot, GroundTruth, entity_poses, run_sim, wrap  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402

sc = load_scenario(resolve_scenario('default'))
GOALS = [(-0.9, -0.9, math.pi / 4), (0.6, -2.0, 0.0), (-1.9, -1.9, math.pi / 4)]
rclpy.init()
with run_sim(sys.argv[1:], launch_file='arena_nav.launch.py'):
    bot, gt = Bot(True, scan=True, nav=True), GroundTruth()
    bot.wait_for(lambda: bot.odom is not None and gt.get() is not None and bot.map_pose() is not None and len(bot.grids) > 0, 120, 'stack')
    bot.spin_wall(3)
    for gi, goal in enumerate(GOALS, 1):
        rows, last = [], [0.0]

        def tick():
            if bot.simt - last[0] < 0.5 or bot.map_pose() is None:
                return
            last[0] = bot.simt
            est, g = bot.map_pose(), gt.get()
            ex, ey = sc.odom_to_world(est[0], est[1])
            v, w = bot.odom.twist.twist.linear.x, bot.odom.twist.twist.angular.z
            ox, oy, oyaw = bot.pose()
            owx, owy = sc.odom_to_world(ox, oy)
            rows.append((bot.simt, math.hypot(ex - g[0], ey - g[1]) * 100,
                         math.degrees(wrap(wrap(est[2] + sc.robot_start.yaw) - g[5])), v, w,
                         math.hypot(owx - g[0], owy - g[1]) * 100, math.degrees(wrap(wrap(oyaw + sc.robot_start.yaw) - g[5]))))

        mx, my = sc.world_to_odom(goal[0], goal[1])
        res = bot.navigate_to(mx, my, goal[2] - sc.robot_start.yaw, timeout=80, on_tick=tick)
        print(f"\n=== goal {gi} world({goal[0]},{goal[1]}): {res['status']} in {res['seconds']:.0f}s")
        print("   t(s)   slam pos err   slam yaw err   v(m/s)  w(rad/s)   ODOM pos err  ODOM yaw err")
        for r in rows:
            print(f"  {r[0]:6.1f}   {r[1]:8.1f} cm   {r[2]:+9.1f} deg   {r[3]:+6.2f}   {r[4]:+6.2f}   {r[5]:9.1f} cm   {r[6]:+8.1f} deg")
        moved = []
        now = entity_poses({v.id for v in sc.victims} | {o.name for o in sc.obstacles})
        for v in sc.victims:
            d = math.hypot(now[v.id][0] - v.x, now[v.id][1] - v.y)
            moved.append(f"{v.id} moved {d * 100:.1f} cm")
        print("   victims after this goal: " + "; ".join(moved))
        if res['status'] != 'succeeded':
            break
    gt.stop()
    bot.destroy_node()
rclpy.shutdown()
