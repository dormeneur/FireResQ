"""Measure perception accuracy against ground truth, per spatial estimator and per range.

  python3 tests/sim/experiments/perception_accuracy.py [--cam87] [--quick]

One simulation, several perception nodes side by side (each publishes on its own topic):
  depth        auto backend (depth)                      gated  (the shipped configuration)
  kh_top       RGB-only, known height, TOP edge anchor   gated
  kh_bottom    RGB-only, known height, BOTTOM anchor     gated  (the ground-plane method it replaced)
  ungated      depth with the motion gate OFF
  ungated_kh   known height with the motion gate OFF
The robot visits stations and looks all the way round from each, stopping at every heading (so the RGB/depth
offset does not apply); it then TURNS at 0.4 rad/s and DRIVES at 0.2 m/s. Errors are XY distance to the true
object axis, in `odom`, binned by camera-to-object range. Truth comes from the scenario: this is a test tool.
"""
import argparse
import math
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
import yaml
from fire_resq_interfaces.msg import DetectionArray

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'simulation'))
from simlib import Bot, run_sim, start_group, stop_group  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402

CONFIG = REPO / 'src' / 'fire_resq_perception' / 'config' / 'perception.yaml'
FLOOR_OFFSET = 0.0325
NODES = {                                        # name -> parameter overrides
    'depth': {},
    'kh_top': {'spatial_backend': 'known_height', 'known_height_anchor': 'top'},
    'kh_bottom': {'spatial_backend': 'known_height', 'known_height_anchor': 'bottom'},
    'ungated': {'max_angular_rate': 1e9, 'max_linear_speed': 1e9},
    'ungated_kh': {'max_angular_rate': 1e9, 'max_linear_speed': 1e9, 'spatial_backend': 'known_height'},
}
STATIONS_WORLD = [(-1.2, -1.0), (-0.4, -0.2), (0.4, 0.3), (-0.4, 1.3), (1.0, -0.4)]
BINS = [(0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 9.0)]


def start_node(name, overrides, target_frame='odom'):
    cfg = yaml.safe_load(CONFIG.read_text())
    params = cfg['perception_node']['ros__parameters']
    params.update({'target_frame': target_frame, 'floor_offset_m': FLOOR_OFFSET, 'use_sim_time': True,
                   'detections_topic': f'/exp/{name}', **overrides})
    f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({f'perc_{name}': {'ros__parameters': params}}, f)
    f.close()
    return start_group(['ros2', 'run', 'fire_resq_perception', 'perception_node', '--ros-args',
                        '-r', f'__node:=perc_{name}', '--params-file', f.name])


def bin_of(r):
    return next(f'{lo:g}-{hi:g} m' for lo, hi in BINS if lo <= r < hi)


class Rec:
    def __init__(self, bot, sc):
        self.bot, self.sc = bot, sc
        self.truth = {'fire': sc.world_to_odom(sc.fire.x, sc.fire.y)}
        self.truth.update({v.id: sc.world_to_odom(v.x, v.y) for v in sc.victims})
        self.rows = defaultdict(list)          # (phase, node) -> [(cls, entity, range, err, valid)]
        self.invalid = defaultdict(int)
        self.unmatched = defaultdict(int)
        self.phase = 'still'
        self.buf = {n: [] for n in NODES}
        for n in NODES:
            bot.create_subscription(DetectionArray, f'/exp/{n}', lambda m, n=n: self.buf[n].append(m), 10)

    def flush(self):
        for n, msgs in self.buf.items():
            for m in msgs:
                tf = self.bot.cam_to_odom(m.header.stamp)
                for d in m.detections:
                    if not d.position_valid:
                        b = d.bbox
                        cx, cy = b.center.position.x, b.center.position.y
                        edge = (cx - b.size_x / 2 <= 0.5 or cx + b.size_x / 2 >= 639.5
                                or cy - b.size_y / 2 <= 0.5 or cy + b.size_y / 2 >= 479.5)
                        self.invalid[(self.phase, n, d.class_id, 'border' if edge else 'other')] += 1
                        continue
                    if tf is None:
                        continue
                    px, py = d.position.x, d.position.y
                    best = min(((math.hypot(px - x, py - y), e) for e, (x, y) in self.truth.items()
                                if (e == 'fire') == (d.class_id == 'fire')), key=lambda t: t[0])
                    if best[0] > 0.6:
                        self.unmatched[(self.phase, n)] += 1
                        continue
                    tx, ty = self.truth[best[1]]
                    rng = math.hypot(tx - tf[1][0], ty - tf[1][1])
                    u = np.array([tx - tf[1][0], ty - tf[1][1]]) / max(rng, 1e-6)
                    dv = np.array([px - tx, py - ty])
                    radial, cross = float(dv @ u), float(dv @ np.array([-u[1], u[0]]))
                    self.rows[(self.phase, n)].append((d.class_id, best[1], rng, best[0], radial, cross))
            msgs.clear()

    def sample(self, seconds):
        for n in self.buf:
            self.buf[n].clear()
        self.bot.spin_sim(seconds)
        self.flush()


def goto(bot, x, y):
    px, py, _ = bot.pose()
    bot.turn_to(math.atan2(y - py, x - px), rate=1.0, settle=0.5)
    bot.drive_distance(math.hypot(x - px, y - py), speed=0.25)
    bot.stop(0.8)


def summarize(rec, phase):
    print(f'\n=== {phase} ===')
    print(f'{"node":<11}{"class":<8}{"range":<10}{"n":>5}{"median cm":>11}{"p90 cm":>9}{"max cm":>9}{"radial cm":>11}{"cross cm":>10}   (radial/cross = signed MEDIAN, +radial = too far)')
    for n in NODES:
        rows = rec.rows.get((phase, n), [])
        for cls in ('victim', 'fire'):
            for lo, hi in BINS:
                e = [r[3] for r in rows if r[0] == cls and lo <= r[2] < hi]
                if e:
                    print(f'{n:<11}{cls:<8}{bin_of(lo):<10}{len(e):>5}{100 * np.median(e):>11.1f}'
                          f'{100 * np.percentile(e, 90):>9.1f}{100 * max(e):>9.1f}'
                          f'{100 * np.median([r[4] for r in rows if r[0] == cls and lo <= r[2] < hi]):>11.1f}'
                          f'{100 * np.median([r[5] for r in rows if r[0] == cls and lo <= r[2] < hi]):>10.1f}')
        inv = {f'{c}/{why}': rec.invalid[(phase, n, c, why)] for c in ('victim', 'fire') for why in ('border', 'other')
               if rec.invalid[(phase, n, c, why)]}
        um = rec.unmatched[(phase, n)]
        print(f'{n:<11}withheld/invalid {inv or 0}   detections matching no object: {um}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam87', action='store_true', help='87 deg camera (arena_nav) instead of the 60 deg arena default')
    ap.add_argument('--quick', action='store_true', help='fewer stations/headings')
    a = ap.parse_args()
    sc = load_scenario(resolve_scenario('default'))
    args = ['camera_hfov:=1.518'] if a.cam87 else []
    procs = []
    rclpy.init()
    with run_sim(args):
        bot = Bot(depth=True)
        try:
            bot.wait_for(lambda: all(bot.counts[k] > 5 for k in ('odom', 'rgb', 'depth')) and 'rgb_info' in bot.msgs,
                         timeout=150, what='the simulation')
            t0 = bot.simt
            bot.wait_for(lambda: bot.simt - t0 > 3.0, timeout=30, what='settle')
            rec = Rec(bot, sc)
            procs = [start_node(n, o) for n, o in NODES.items()]
            bot.wait_for(lambda: all(rec.buf[n] for n in NODES), timeout=60, what='the perception nodes')

            stations = STATIONS_WORLD[:2] if a.quick else STATIONS_WORLD
            step = math.radians(60 if a.quick else 30)
            rec.phase = 'still'
            for target in [None] + [sc.world_to_odom(*s) for s in stations]:      # None = where the robot starts
                if target is not None:
                    goto(bot, *target)
                yaw0 = bot.pose()[2]
                for k in range(int(round(2 * math.pi / step))):
                    bot.turn_to(yaw0 + (k + 1) * step, rate=1.0, settle=0.8)
                    rec.sample(0.7)
            summarize(rec, 'still')

            # moving: from a station with objects in view, turn and drive
            goto(bot, *sc.world_to_odom(-0.4, -0.2))
            bot.turn_to(math.radians(20), settle=1.0)
            rec.phase = 'turning 0.4 rad/s'
            for n in rec.buf:
                rec.buf[n].clear()
            bot.spin_sim(9.0, wz=0.4, on_tick=rec.flush)
            rec.flush()
            bot.stop(1.0)
            summarize(rec, 'turning 0.4 rad/s')

            goto(bot, *sc.world_to_odom(-1.2, -1.0))
            bot.turn_to(math.atan2(sc.world_to_odom(0.4, 0.3)[1] - bot.pose()[1], sc.world_to_odom(0.4, 0.3)[0] - bot.pose()[0]),
                        settle=1.0)
            rec.phase = 'driving 0.2 m/s'
            for n in rec.buf:
                rec.buf[n].clear()
            bot.spin_sim(4.0, vx=0.2, on_tick=rec.flush)
            rec.flush()
            bot.stop(1.0)
            summarize(rec, 'driving 0.2 m/s')
        finally:
            for p in procs:
                stop_group(p)
            bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
