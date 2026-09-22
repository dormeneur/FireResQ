"""Reproduce and root-cause an intermittent Nav2 goal failure (the Phase 4 detour goal), with a time series per goal.

  python3 tests/sim/experiments/nav_goal_diag.py [--trials N] [--reinit] [--map YAML] [--goals G1,G2,G3]
                                                  [--variants 'base;name=/controller_server:FollowPath.param=value,...'] [--out DIR]
  python3 tests/sim/experiments/nav_goal_diag.py --probe N [--map YAML]     # planner path END vs the requested goal

Maps the arena once (or reuses --map), then runs the SAME goal sequence as tests/sim/test_nav2.py (G1, G2 = the detour, G3 =
home) N times under AMCL + Nav2. --reinit re-initialises AMCL at the mission start pose before each trial (fresh-session
randomness, as the test fixture has). --variants runs the same trials under several controller-parameter settings, applied at
run time with `ros2 param set` and reset in between: that isolates WHICH mechanism causes a failure without editing any file.

For every goal it records, at ~10 Hz, Gazebo's TRUE pose, AMCL's estimate in the map frame (what Nav2 acts on), the robot's
velocity, the controller's /cmd_vel, the controller's own lookahead point and Nav2's feedback, as CSV in --out, and prints one
line per goal. Ground truth is used to JUDGE only.
"""
import argparse
import csv
import math
import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from routes import LAP_WORLD  # noqa: E402
from simlib import Bot, GroundTruth, entity_poses, lifecycle_state, run_sim, wrap  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402

GOALS = {'G1': (-0.9, -0.9, math.pi / 4), 'G2': (0.6, -2.0, 0.0), 'G3': (-1.9, -1.9, math.pi / 4)}   # WORLD frame, as test_nav2.py
# the shipped values of every parameter a variant may touch (restored before each variant)
RESET = [('/controller_server', 'FollowPath.use_collision_detection', 'true'),
         ('/controller_server', 'FollowPath.use_regulated_linear_velocity_scaling', 'true'),
         ('/controller_server', 'FollowPath.use_rotate_to_heading', 'true'),
         ('/controller_server', 'FollowPath.min_approach_linear_velocity', '0.05'),
         ('/controller_server', 'FollowPath.rotate_to_heading_min_angle', '0.6'),
         ('/controller_server', 'FollowPath.approach_velocity_scaling_dist', '0.6'),
         ('/controller_server', 'general_goal_checker.xy_goal_tolerance', '0.10'),
         ('/controller_server', 'general_goal_checker.yaw_goal_tolerance', '0.15'),
         ('/controller_server', 'progress_checker.movement_time_allowance', '15.0')]


def ros2_param_set(node, name, value):
    r = subprocess.run(['ros2', 'param', 'set', node, name, value], capture_output=True, text=True, timeout=30)
    return r.stdout.strip() or r.stderr.strip()


def make_map(sc):
    with run_sim(['navigation:=false'], launch_file='arena_nav.launch.py'):
        bot, gt = Bot(True, scan=True, nav=True), GroundTruth()
        bot.wait_for(lambda: bot.counts['odom'] > 5 and gt.get() is not None and len(bot.grids) > 0 and bot.map_pose() is not None, 150, 'SLAM')
        bot.spin_wall(3)
        for wx, wy in LAP_WORLD:
            bot.go_to(*sc.world_to_odom(wx, wy))
        bot.turn_to(0.0, settle=1.0)
        bot.spin_wall(3)
        out = Path(tempfile.mkdtemp(prefix='fire_resq_map_')) / 'arena'
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out), '--ros-args', '-p', 'save_map_timeout:=15000.0',
                        '-p', 'use_sim_time:=true'], capture_output=True, text=True, timeout=60)
        gt.stop()
        bot.destroy_node()
    return Path(str(out) + '.yaml')


class Stack:
    """AMCL + Nav2 on a saved map, a Bot, ground truth, and taps on the controller's own outputs."""

    def __init__(self, sc, map_yaml, sim):
        self.sc, self.sim = sc, sim
        self.bot, self.gt = Bot(True, scan=True, nav=True), GroundTruth()
        bot = self.bot
        bot.wait_for(lambda: lifecycle_state('/map_server') == 'active' and lifecycle_state('/amcl') == 'active', 120, 'amcl')
        self.init_pub = bot.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.reinit(first=True)
        bot.wait_for(lambda: bot.map_pose() is not None, 30, 'map->base_link')
        bot.wait_for(lambda: all(lifecycle_state(f'/{n}') == 'active' for n in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')), 120, 'nav2')
        bot.spin_wall(3)
        self.cmd, self.rpp = [0.0, 0.0], {}
        bot.create_subscription(Twist, '/cmd_vel', lambda t: self.cmd.__setitem__(slice(None), [t.linear.x, t.angular.z]), 10)
        bot.create_subscription(PointStamped, '/lookahead_point', lambda m: self.rpp.__setitem__('carrot', (m.point.x, m.point.y)), 10)   # ROBOT frame
        self.client = ActionClient(bot, NavigateToPose, 'navigate_to_pose')

    def reinit(self, first=False):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = m.pose.covariance[7] = m.pose.covariance[35] = 0.05
        for _ in range(8):
            m.header.stamp = self.bot.get_clock().now().to_msg()
            self.init_pub.publish(m)
            self.bot.spin_wall(0.4)
        if not first:
            self.bot.spin_wall(3.0)

    def displaced(self):
        """How far every victim and obstacle has moved from where the scenario put it (cm), from Gazebo's TRUE poses."""
        names = {v.id: (v.x, v.y) for v in self.sc.victims}
        names.update({o.name: (o.x, o.y) for o in self.sc.obstacles})
        names['fire'] = (self.sc.fire.x, self.sc.fire.y)
        now = entity_poses(set(names))
        return {n: math.hypot(now[n][0] - x, now[n][1] - y) * 100 for n, (x, y) in names.items() if n in now}

    def start_offset(self):
        e, g = self.bot.map_pose(), self.gt.get()
        gx, gy = self.sc.world_to_odom(g[0], g[1])
        return math.hypot(e[0] - gx, e[1] - gy) * 100

    def run_goal(self, name, tag, out):
        sc, bot, gt = self.sc, self.bot, self.gt
        goal = GOALS[name]
        mx, my = sc.world_to_odom(goal[0], goal[1])
        myaw = wrap(goal[2] - sc.robot_start.yaw)
        rows, fb, last = [], {}, [-1.0]

        def tick():
            if bot.simt - last[0] < 0.1 or bot.map_pose() is None:
                return
            last[0] = bot.simt
            est, g = bot.map_pose(), gt.get()
            gx, gy = sc.world_to_odom(g[0], g[1])
            gyaw = wrap(g[5] - sc.robot_start.yaw)
            c = self.rpp.get('carrot')
            rows.append(dict(t=bot.simt, gt_x=gx, gt_y=gy, gt_yaw=gyaw, amcl_x=est[0], amcl_y=est[1], amcl_yaw=est[2],
                             gt_dist=math.hypot(gx - mx, gy - my), amcl_dist=math.hypot(est[0] - mx, est[1] - my),
                             gt_yaw_err=wrap(gyaw - myaw), amcl_yaw_err=wrap(est[2] - myaw),
                             v=bot.odom.twist.twist.linear.x, w=bot.odom.twist.twist.angular.z, cmd_v=self.cmd[0], cmd_w=self.cmd[1],
                             carrot_dist=(math.hypot(*c) if c else float('nan')), carrot_x=(c[0] if c else float('nan')), carrot_y=(c[1] if c else float('nan')), remaining=fb.get('remaining', float('nan'))))

        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = mx, my
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
        t0 = time.time()
        send = self.client.send_goal_async(g, feedback_callback=lambda f: fb.__setitem__('remaining', f.feedback.distance_remaining))
        while not send.done() and time.time() - t0 < 20:
            rclpy.spin_once(bot, timeout_sec=0.05)
            tick()
        res = send.result().get_result_async()
        while not res.done() and time.time() - t0 < 120:
            rclpy.spin_once(bot, timeout_sec=0.05)
            tick()
        status = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(res.result().status, 'timeout') if res.done() else 'timeout'
        bot.send(0.0, 0.0)
        bot.spin_wall(0.5)
        tick()
        with open(out / f'{tag}_{name}_{status}.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        e = rows[-1]
        print(f"  {tag} {name}: {status:<9} {e['t'] - rows[0]['t']:5.1f}s | END truth {e['gt_dist'] * 100:5.1f} cm {math.degrees(e['gt_yaw_err']):+6.1f} deg"
              f" | amcl {e['amcl_dist'] * 100:5.1f} cm {math.degrees(e['amcl_yaw_err']):+6.1f} deg | controller carrot {e['carrot_dist'] * 100:5.1f} cm", flush=True)
        return status


def probe(stack, n):
    """Ask the planner for N paths and report how far each path END is from the requested goal."""
    rnd = random.Random(3)
    pc = ActionClient(stack.bot, ComputePathToPose, 'compute_path_to_pose')
    pc.wait_for_server(20)
    offs = []
    for _ in range(n):
        gx, gy, gyaw = rnd.uniform(-1.6, 1.6), rnd.uniform(-1.6, 1.6), rnd.uniform(-math.pi, math.pi)
        q = ComputePathToPose.Goal()
        q.use_start, q.planner_id = True, ''
        for pose, (x, y) in ((q.start, (0.0, 0.0)), (q.goal, (gx, gy))):
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        sf = pc.send_goal_async(q)
        stack.bot.wait_for(lambda: sf.done(), 10, 'goal accepted')
        rf = sf.result().get_result_async()
        stack.bot.wait_for(lambda: rf.done(), 10, 'path')
        poses = rf.result().result.path.poses
        if poses:
            offs.append(math.hypot(poses[-1].pose.position.x - gx, poses[-1].pose.position.y - gy) * 100)
    print(f'plan END is {min(offs):.2f}-{max(offs):.2f} cm (mean {sum(offs) / len(offs):.2f}) from the requested goal over {len(offs)} goals')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=8)
    ap.add_argument('--map', default='')
    ap.add_argument('--out', default='/tmp/nav_goal_diag')
    ap.add_argument('--goals', default='G1,G2,G3', help='run in this order every trial (end with G3: it brings the robot home)')
    ap.add_argument('--reinit', action='store_true', help='re-initialise AMCL at the mission start pose before each trial')
    ap.add_argument('--variants', default='base', help="'base;name=/controller_server:FollowPath.use_collision_detection=false,...'")
    ap.add_argument('--probe', type=int, default=0)
    args = ap.parse_args()
    sc = load_scenario(resolve_scenario('default'))
    rclpy.init()
    map_yaml = Path(args.map) if args.map else make_map(sc)
    print('map:', map_yaml)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    with run_sim(['localization:=amcl', f'map:={map_yaml}', 'navigation:=true'], launch_file='arena_nav.launch.py') as sim:
        stack = Stack(sc, map_yaml, sim)
        live = subprocess.run(['ros2', 'param', 'get', '/controller_server', 'FollowPath.stateful'], capture_output=True, text=True, timeout=30).stdout.strip()
        print('LIVE controller parameter FollowPath.stateful:', live, flush=True)
        if args.probe:
            probe(stack, args.probe)
        else:
            first = True
            for variant in [v for v in args.variants.split(';') if v]:
                vname, _, vsets = variant.partition('=')
                for node, name, val in RESET:
                    ros2_param_set(node, name, val)
                for item in [x for x in vsets.split(',') if x]:
                    node_param, _, val = item.rpartition('=')
                    node, _, name = node_param.partition(':')
                    print(f'  set {node} {name} = {val}: {ros2_param_set(node, name, val)[:70]}', flush=True)
                print(f'=== variant {vname}', flush=True)
                if not first:
                    stack.reinit()
                first = False
                for trial in range(1, args.trials + 1):
                    if args.reinit and trial > 1:
                        stack.reinit()
                    moved = stack.displaced()
                    print(f'--- {vname} trial {trial}: AMCL offset from truth at the start {stack.start_offset():.1f} cm | displaced from the scenario (cm): '
                          + ' '.join(f'{k} {v:.1f}' for k, v in moved.items() if k.startswith('victim') or k == 'fire'), flush=True)
                    for name in args.goals.split(','):
                        status = stack.run_goal(name, f'{vname}_{trial:02d}', out)
                        results.setdefault((vname, name), []).append(status)
                        if name == 'G3' and status != 'succeeded':
                            break
            print('\n=== SUMMARY: succeeded / total, per variant and goal')
            for (vname, name), st in results.items():
                print(f'  {vname:<16} {name}: {sum(1 for x in st if x == "succeeded")}/{len(st)}')
        stack.gt.stop()
        stack.bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
