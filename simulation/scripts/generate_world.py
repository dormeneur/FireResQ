#!/usr/bin/env python3
"""Generate a Gazebo world from a FireResQ scenario, or print its layout.

  ros2 run fire_resq_simulation generate_world.py --ascii
  ros2 run fire_resq_simulation generate_world.py --scenario default --output /tmp/arena.sdf
"""
import argparse
import sys
from pathlib import Path

try:
    import fire_resq_simulation  # noqa: F401
except ImportError:  # running from the source tree, not installed
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fire_resq_simulation import ScenarioError, load_scenario, resolve_scenario, write_world
from fire_resq_simulation.scenario import ascii_map


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--scenario', default='default', help='name under config/scenarios or a path')
    ap.add_argument('--output', help='write the world SDF here')
    ap.add_argument('--overview-camera', action='store_true',
                    help='DEBUG: add a top-down camera publishing overview/image_raw')
    ap.add_argument('--ascii', action='store_true', help='print a top-down text map')
    a = ap.parse_args()
    try:
        sc = load_scenario(resolve_scenario(a.scenario))
    except ScenarioError as e:
        print(e, file=sys.stderr)
        return 2
    if a.ascii or not a.output:
        print(f'scenario "{sc.name}"   # wall, . safe zone, X obstacle, F fire, 1-3 victims, R robot')
        print(ascii_map(sc))
    if a.output:
        print(f'wrote {write_world(sc, a.output, overview_camera=a.overview_camera)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
