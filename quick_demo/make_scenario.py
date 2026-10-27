#!/usr/bin/env python3
"""Random SMALL rescue arena for the quick demo (simulator side: this is ground truth, the robot never reads it).

3 m x 3 m, safe zone in the south-west corner, one fire, three victims at random distances from the fire, no obstacles.
A layout is only accepted if the quick controller can do it cleanly: every victim visible from the start, the straight
drive from the safe-zone gate to each victim clear of the fire and the other victims, and the fire distances distinct
enough that the rescue order is unambiguous to perception (+- a few cm).

  python3 quick_demo/make_scenario.py --seed 7 --out /tmp/s.yaml
"""
import argparse
import math
import random
import sys
from pathlib import Path

import yaml

from fire_resq_simulation.scenario import ascii_map, load_scenario

HALF = 1.5                       # arena inner half-size
ZONE = (-1.0, -1.0)              # safe-zone centre = robot start
GATE = (-0.4, -0.4)              # where the robot leaves/enters the zone (rescue.py: 0.85 m ahead of the start)
ROBOT_R = 0.16                   # circumscribed robot radius (chassis + magnet)
FIRE_R, VICTIM_R = 0.15, 0.056
MARGIN = 0.10


def seg_dist(p, a, b):
    ax, ay, bx, by = *a, *b
    t = max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))


def candidate(rng):
    d = math.dist
    fire = (rng.uniform(-0.9, 1.1), rng.uniform(-0.9, 1.1))
    if d(fire, ZONE) < 1.4 or d(fire, GATE) < 0.8:
        return None
    victims = []
    for _ in range(3):
        a, r = rng.uniform(0, 2 * math.pi), rng.uniform(0.6, 1.7)
        victims.append((fire[0] + r * math.cos(a), fire[1] + r * math.sin(a)))
    if any(abs(c) > HALF - 0.35 for v in victims for c in v):
        return None
    if any(d(v, ZONE) < 1.0 or d(v, GATE) < 0.6 for v in victims):
        return None
    if any(d(a, b) < 0.7 for i, a in enumerate(victims) for b in victims[i + 1:]):
        return None
    df = sorted(d(v, fire) for v in victims)
    if df[1] - df[0] < 0.15 or df[2] - df[1] < 0.15:
        return None
    for v in victims:
        if seg_dist(fire, GATE, v) < ROBOT_R + FIRE_R + MARGIN:            # the drive out and back
            return None
        if seg_dist(fire, ZONE, v) < FIRE_R + 0.05:                         # the first look from the start
            return None
        for o in victims:
            if o is not v and (seg_dist(o, GATE, v) < ROBOT_R + VICTIM_R + MARGIN or seg_dist(o, ZONE, v) < 0.15):
                return None
    return fire, victims


def make(seed):
    rng = random.Random(seed)
    for _ in range(200000):
        c = candidate(rng)
        if c:
            break
    else:
        sys.exit(f'no valid layout for seed {seed}')
    fire, victims = c
    return {
        'name': f'quick_{seed}',
        'description': f'quick_demo/make_scenario.py --seed {seed}: 3 x 3 m, one fire, three victims, no obstacles.',
        'arena': {'size_x': 2 * HALF, 'size_y': 2 * HALF, 'wall_height': 0.35, 'wall_thickness': 0.08},
        'robot_start': {'x': ZONE[0], 'y': ZONE[1], 'yaw': math.pi / 4},
        'safe_zone': {'x': ZONE[0], 'y': ZONE[1], 'size_x': 1.0, 'size_y': 1.0},
        'fire': {'model': 'fire', 'x': round(fire[0], 3), 'y': round(fire[1], 3)},
        'victims': [{'id': f'victim_{i + 1}', 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(rng.uniform(0, 6.28), 2)}
                    for i, (x, y) in enumerate(victims)],
        'obstacles': [],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    doc = make(a.seed)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(yaml.safe_dump(doc, sort_keys=False))
    sc = load_scenario(a.out)                      # the project's own validator: fails loudly on a bad layout
    print(ascii_map(sc))
    f = doc['fire']
    for v in sorted(doc['victims'], key=lambda v: math.hypot(v['x'] - f['x'], v['y'] - f['y'])):
        print(f"  {v['id']}: {math.hypot(v['x'] - f['x'], v['y'] - f['y']):.2f} m from the fire")


if __name__ == '__main__':
    main()
