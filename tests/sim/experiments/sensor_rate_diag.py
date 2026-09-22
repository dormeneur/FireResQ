"""Measure the depth and /scan rates per SIMULATION second, and how fast the test client itself can drain messages.

  python3 tests/sim/experiments/sensor_rate_diag.py [--windows 4]

Brings up the same arena_nav configuration as the `navsim` fixture (depth -> /scan only), then for a few 6-simulated-second
windows counts what a Bot receives: depth images, scans, and every other callback the Bot handles (joint states, TF, odom, RGB).
`test_scan_rate` counts what the test client RECEIVES; this shows how much of that is the pipeline and how much the client's own
callback throughput (rclpy `spin_once` handles one message per call, and the client also drains ~500 TF/joint-state messages/s).
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from simlib import Bot, run_sim  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--windows', type=int, default=4)
    args = ap.parse_args()
    rclpy.init()
    with run_sim(['localization:=none', 'navigation:=false'], launch_file='arena_nav.launch.py'):
        bot = Bot(depth=True, scan=True)
        bot.wait_for(lambda: all(bot.counts[k] > 5 for k in ('odom', 'rgb', 'joint_states', 'depth', 'scan')), 150, 'the sensors')
        t0 = bot.simt
        bot.wait_for(lambda: bot.simt - t0 > 3.0, 30, 'settle')
        for w in range(args.windows):
            bot.counts.clear()
            s0, w0, cb = bot.simt, time.time(), 0
            while bot.simt - s0 < 6.0:
                rclpy.spin_once(bot, timeout_sec=0.1)
                cb += 1
            sim_s, wall = bot.simt - s0, time.time() - w0
            c = dict(bot.counts)
            print(f'window {w + 1}: scan {c.get("scan", 0) / sim_s:5.1f} Hz  depth {c.get("depth", 0) / sim_s:5.1f} Hz  rgb {c.get("rgb", 0) / sim_s:5.1f} Hz  '
                  f'joint_states {c.get("joint_states", 0) / sim_s:6.1f} Hz  odom {c.get("odom", 0) / sim_s:5.1f} Hz | '
                  f'real-time factor {sim_s / wall:.2f}, client callbacks/s {cb / wall:.0f}', flush=True)
        bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
