"""Navigation Step 2: SLAM Toolbox builds a map and localizes while the robot drives.

The route is supplied BY THE TEST (the robot has no exploration behaviour yet - that belongs to
the later rescue loop). Everything is judged against ground truth: the scenario geometry for the
map, Gazebo's pose for localization.

The scan-matching preset skips rotation-only scans (see its header), so the test documents that
limitation explicitly instead of hiding it.
"""
import math
import subprocess

import numpy as np
import pytest

from fire_resq_simulation.scenario import distance_to_surface
from routes import LAP_WORLD
from simlib import wrap
from test_nav_scan import tf_publishers



def known_area(grid):
    a = np.array(grid.data)
    return float((a >= 0).sum()) * grid.info.resolution ** 2


def occupied_world_points(grid, sc):
    a = np.array(grid.data).reshape(grid.info.height, grid.info.width)
    ys, xs = np.nonzero(a > 50)
    r, ox, oy = grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y
    return [sc.odom_to_world(ox + (x + 0.5) * r, oy + (y + 0.5) * r) for x, y in zip(xs, ys)]     # map == odom at start


# ---------------------------------------------------------------- before the robot moves
def test_slam_toolbox_lifecycle_is_active(mapsim):
    out = subprocess.run(['ros2', 'lifecycle', 'get', '/slam_toolbox'], capture_output=True, text=True).stdout
    assert 'active' in out, out


def test_map_frame_exists_with_a_single_publisher(mapsim):
    bot = mapsim.bot
    bot.tf_dyn.clear()
    bot.spin_wall(4.0)
    assert ('map', 'odom') in bot.tf_dyn, 'no map->odom transform'
    ts = bot.tf_dyn[('map', 'odom')]
    assert len(ts) / (max(ts) - min(ts)) >= 10
    assert tf_publishers() == ['bridge_base', 'robot_state_publisher', 'slam_toolbox']
    parents = {}
    for p, c in list(bot.tf_dyn) + list(bot.tf_static):
        parents.setdefault(c, set()).add(p)
    assert all(len(v) == 1 for v in parents.values()), 'a frame has two parents'
    assert parents['odom'] == {'map'} and parents['base_link'] == {'odom'}


def test_first_scan_produces_a_map(mapsim):
    g = mapsim.bot.grids[-1]
    assert g.header.frame_id == 'map' and g.info.resolution == pytest.approx(0.05)
    a = np.array(g.data)
    assert (a > 50).sum() > 20 and ((a >= 0) & (a <= 50)).sum() > 500


def test_known_limitation_in_place_rotation_adds_nothing_to_the_map(mapsim):
    """KNOWN LIMITATION, asserted so it cannot be forgotten: the scan-matching preset skips
    rotation-only scans (matching them diverged), so spinning in place does not map. If a future
    configuration integrates rotation safely this test should fail - and the docs be updated."""
    bot = mapsim.bot
    before = known_area(bot.grids[-1])
    bot.spin_sim(2 * math.pi / 0.6, wz=0.6)
    bot.stop(1.0)
    bot.spin_wall(3.0)
    assert known_area(bot.grids[-1]) - before < 0.5


# ---------------------------------------------------------------- one lap, many judgements
@pytest.fixture(scope='module')
def lap(mapsim, scenario):
    bot, gt = mapsim.bot, mapsim.gt
    start_odom = bot.pose()
    samples = []

    def sample(label):
        bot.spin_wall(0.6)
        est, g = bot.map_pose(), gt.get()
        ex, ey = scenario.odom_to_world(est[0], est[1])
        eyaw = wrap(est[2] + scenario.robot_start.yaw)
        samples.append((label, math.hypot(ex - g[0], ey - g[1]), abs(math.degrees(wrap(eyaw - g[5])))))

    n0 = len(bot.map_odom)
    for i, (wx, wy) in enumerate(LAP_WORLD):
        bot.go_to(*scenario.world_to_odom(wx, wy))
        sample(f'waypoint {i + 1}')
    bot.spin_wall(3.0)                                    # let the last map update publish
    return dict(samples=samples, grid=bot.grids[-1], map_odom=bot.map_odom[n0:], start_odom=start_odom)


def test_localization_stays_close_to_the_truth_along_the_whole_lap(lap):
    worst = max(s[1] for s in lap['samples'])
    final = lap['samples'][-1]
    detail = ', '.join(f'{s[0]}: {s[1] * 100:.1f} cm/{s[2]:.1f} deg' for s in lap['samples'])
    print(f'MEASURED slam pose error along the lap: {detail}')
    # Measured over three runs of this lap: worst waypoint 2.6-4.8 cm, final 0.3-2.3 cm, yaw <= 1.5 deg.
    # Bounds are ~2x the worst measured, so a real regression (the 0.1 m gate gave 20 cm / 6 deg) fails.
    assert worst <= 0.10, f'worst position error {worst * 100:.0f} cm ({detail})'
    assert final[1] <= 0.08 and final[2] <= 4.0, f'final {final[1] * 100:.0f} cm, {final[2]:.1f} deg ({detail})'


def test_map_to_odom_correction_is_not_erratic(lap):
    m = np.array(lap['map_odom'])
    jumps = np.hypot(np.diff(m[:, 1]), np.diff(m[:, 2]))
    yaw_jumps = np.abs([wrap(d) for d in np.diff(m[:, 3])])
    print(f'MEASURED map->odom max step: {jumps.max() * 100:.1f} cm, {math.degrees(yaw_jumps.max()):.2f} deg; samples {len(m)}')
    assert jumps.max() <= 0.15, f'map->odom jumped {jumps.max() * 100:.0f} cm in one step'   # measured ~8 cm
    assert math.degrees(yaw_jumps.max()) <= 3.0, f'map->odom turned {math.degrees(yaw_jumps.max()):.1f} deg in one step'


def test_occupied_cells_lie_on_real_surfaces(lap, scenario):
    """Precision: an occupied cell far from every real surface is a wrong map."""
    d = np.array([distance_to_surface(scenario, x, y) for x, y in occupied_world_points(lap['grid'], scenario)])
    assert len(d) > 300
    print(f'MEASURED map precision: {100 * (d <= 0.05).mean():.1f}% within 5 cm, {100 * (d <= 0.10).mean():.1f}% within 10 cm, median {np.median(d) * 100:.1f} cm, {len(d)} occupied cells')
    assert (d <= 0.10).mean() >= 0.95, f'only {100 * (d <= 0.10).mean():.0f}% of occupied cells are within 10 cm of a real surface'
    assert (d <= 0.05).mean() >= 0.70, f'only {100 * (d <= 0.05).mean():.0f}% within 5 cm'


def test_the_lap_mapped_the_arena_boundary(lap, scenario):
    """Recall: sample the TRUE inner wall faces every 5 cm; most must have a mapped cell nearby."""
    pts = np.array(occupied_world_points(lap['grid'], scenario))
    h = scenario.arena.size_x / 2
    truth = [(x, y) for x in np.arange(-h, h, 0.05) for y in (-h, h)] + [(x, y) for y in np.arange(-h, h, 0.05) for x in (-h, h)]
    hit = np.mean([np.min(np.hypot(pts[:, 0] - x, pts[:, 1] - y)) <= 0.10 for x, y in truth])
    print(f'MEASURED wall recall: {100 * hit:.1f}%; known area {known_area(lap["grid"]):.1f} m2 of {scenario.arena.size_x * scenario.arena.size_y:.0f}')
    assert hit >= 0.85, f'only {100 * hit:.0f}% of the true walls were mapped'
    assert known_area(lap['grid']) >= 0.6 * scenario.arena.size_x * scenario.arena.size_y
