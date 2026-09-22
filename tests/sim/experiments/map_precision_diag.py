"""Is the intermittent map-precision failure map QUALITY, or the map's rigid offset from Gazebo's frame?

  python3 tests/sim/experiments/map_precision_diag.py --runs 8        # map the arena 8 times, then analyse every saved map
  python3 tests/sim/experiments/map_precision_diag.py --analyze       # analyse the maps already saved under /tmp/fire_resq_map_*

For each saved map it computes what tests/sim/test_nav_mapping.py::test_occupied_cells_lie_on_real_surfaces computes (share of
occupied cells within 5 / 10 cm of a TRUE surface), and again after the best rigid TRANSLATION of the map (search +-8 cm). A
constant offset between the map and Gazebo's world frame is irrelevant to navigation (AMCL localises against the same map), but
on a 5 cm grid it moves cells across the 5 cm line. Ground truth is used to JUDGE only.
"""
import argparse
import glob
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tests' / 'sim'))
sys.path.insert(0, str(REPO / 'tests' / 'sim' / 'experiments'))
sys.path.insert(0, str(REPO / 'simulation'))
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402
from fire_resq_simulation.scenario import distance_to_surface  # noqa: E402


def occupied_world(sc, yaml_path):
    t = open(yaml_path).read()
    res = float(re.search(r'resolution:\s*([\d.]+)', t)[1])
    org = [float(x) for x in re.search(r'origin:\s*\[([^\]]+)\]', t)[1].split(',')]
    parts = open(yaml_path.replace('.yaml', '.pgm'), 'rb').read().split(b'\n', 3)
    w, h = map(int, parts[1].split())
    data = np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w)
    ys, xs = np.where(data < 50)                     # black = occupied
    return np.array([sc.odom_to_world(org[0] + (c + 0.5) * res, org[1] + (h - 1 - r + 0.5) * res) for r, c in zip(ys, xs)])


def metric(sc, pts, dx=0.0, dy=0.0):
    d = np.array([distance_to_surface(sc, x + dx, y + dy) for x, y in pts])
    return (d <= 0.05).mean() * 100, (d <= 0.10).mean() * 100


def analyse(sc, maps):
    print(f'{"map":<26}{"cells":>6}{"raw<=5cm":>10}{"raw<=10cm":>11}   {"aligned<=5cm":>13}{"shift cm (x,y)":>16}{"aligned<=10cm":>15}')
    raw, aligned = [], []
    for y in maps:
        pts = occupied_world(sc, y)
        if len(pts) < 300:
            continue
        r5, r10 = metric(sc, pts)
        best = max(((metric(sc, pts, dx / 100, dy / 100)[0], dx, dy) for dx in range(-8, 9) for dy in range(-8, 9)), key=lambda t: t[0])
        raw.append(r5)
        aligned.append(best[0])
        print(f'{y.split("/")[2]:<26}{len(pts):>6}{r5:>9.1f}%{r10:>10.1f}%   {best[0]:>12.1f}%{f"({best[1]:+d},{best[2]:+d})":>16}{metric(sc, pts, best[1] / 100, best[2] / 100)[1]:>14.1f}%')
    if raw:
        print(f'\nwithin 5 cm over {len(raw)} maps: raw {min(raw):.1f}-{max(raw):.1f}% (mean {np.mean(raw):.1f}); after removing the rigid offset {min(aligned):.1f}-{max(aligned):.1f}% (mean {np.mean(aligned):.1f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=0)
    ap.add_argument('--analyze', action='store_true')
    args = ap.parse_args()
    sc = load_scenario(resolve_scenario('default'))
    if args.runs:
        import rclpy
        from nav_goal_diag import make_map
        rclpy.init()
        for i in range(args.runs):
            print(f'mapping run {i + 1}/{args.runs}: {make_map(sc)}', flush=True)
        rclpy.shutdown()
    analyse(sc, sorted(glob.glob('/tmp/fire_resq_map_*/arena.yaml')))


if __name__ == '__main__':
    main()
