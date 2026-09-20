"""Navigation Step 3: localize against a SAVED map with AMCL.

The map comes from a SLAM run (the session fixture `arena_map_yaml`: lap under scan-matching SLAM,
saved with map_saver_cli). A FRESH simulation then runs AMCL against it - this is the "localize
itself against the generated map" requirement, and where the single-publisher rule matters: SLAM
Toolbox and AMCL both publish map -> odom, so exactly one may exist at a time.

The start pose given to AMCL is the mission START pose (the map origin, because the map was built
from the start) - not ground truth.
"""
import math
import subprocess

import numpy as np

from routes import ROUTE_WORLD
from simlib import lifecycle_state, wrap
from test_nav_scan import tf_publishers


def nodes():
    return subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True).stdout.split()


def error_vs_truth(env, sc):
    est, g = env.bot.map_pose(), env.gt.get()
    ex, ey = sc.odom_to_world(est[0], est[1])
    return math.hypot(ex - g[0], ey - g[1]), abs(math.degrees(wrap(wrap(est[2] + sc.robot_start.yaw) - g[5])))


# ---------------------------------------------------------------- the saved map
def test_the_map_was_saved(arena_map_yaml):
    assert arena_map_yaml.exists() and arena_map_yaml.with_suffix('.pgm').exists()
    text = arena_map_yaml.read_text()
    assert 'resolution: 0.05' in text and 'image: arena.pgm' in text


# ---------------------------------------------------------------- AMCL against it
def test_map_server_and_amcl_become_active(amclsim):
    assert lifecycle_state('/map_server') == 'active' and lifecycle_state('/amcl') == 'active'


def test_exactly_one_map_odom_publisher_and_it_is_amcl(amclsim):
    assert tf_publishers() == ['amcl', 'bridge_base', 'robot_state_publisher']
    assert '/slam_toolbox' not in nodes(), 'SLAM Toolbox is running: two map->odom publishers'


def test_amcl_localizes_from_the_mission_start_pose(amclsim, scenario):
    pos, yaw = error_vs_truth(amclsim, scenario)
    print(f'MEASURED amcl error at the mission start pose: {pos * 100:.1f} cm, {yaw:.1f} deg')
    assert pos <= 0.15 and yaw <= 5.0, f'{pos * 100:.0f} cm, {yaw:.1f} deg'


def test_amcl_keeps_localizing_on_a_different_route_with_spins(amclsim, scenario):
    """A route that was not the mapping lap, with in-place spins: the SLAM preset skips rotation-only
    scans, AMCL must cope with them."""
    bot = amclsim.bot
    errs = []

    def sample(label):
        bot.spin_wall(1.0)
        errs.append((label,) + error_vs_truth(amclsim, scenario))

    for i, (wx, wy) in enumerate(ROUTE_WORLD):
        bot.go_to(*scenario.world_to_odom(wx, wy))
        sample(f'waypoint {i + 1}')
        if i in (1, 2):
            bot.spin_sim(2 * math.pi / 0.6, wz=0.6)
            bot.stop(1.0)
            sample(f'spin after waypoint {i + 1}')
    detail = ', '.join(f'{l}: {p * 100:.1f} cm/{y:.1f} deg' for l, p, y in errs)
    pos = np.array([e[1] for e in errs])
    print(f'MEASURED amcl error on a different route with spins: {detail}; median {np.median(pos) * 100:.1f} cm')
    assert np.median(pos) <= 0.10, f'median {np.median(pos) * 100:.0f} cm ({detail})'
    assert pos.max() <= 0.35, f'worst {pos.max() * 100:.0f} cm ({detail})'
    assert errs[-1][1] <= 0.20, f'final {errs[-1][1] * 100:.0f} cm ({detail})'
