"""The rescue arena: everything is where the scenario says, perceivable, and driveable.

Object placement is ground truth from the scenario YAML. The robot must not know it - these
tests judge what the robot's own camera + depth + TF can recover.
"""
import math

import numpy as np
import pytest

from simlib import (WHEEL_R, class_masks, entity_poses, match_entities, survey, wrap)


@pytest.fixture(scope='module')
def poses(arena, scenario):
    return entity_poses(set(scenario.entities()) | {'fire_resq', 'wall_north', 'wall_south',
                                                    'wall_east', 'wall_west'})


# ---------------------------------------------------------------- world content
def test_arena_loads_without_gazebo_errors(arena):
    errors = [ln for ln in arena.sim.log().splitlines() if '[Err]' in ln]
    assert not errors, 'Gazebo reported errors:\n' + '\n'.join(errors[:8])


def test_every_scenario_entity_exists_at_its_scenario_pose(poses, scenario):
    for name, (kind, x, y) in scenario.entities().items():
        assert name in poses, f'{kind} "{name}" is missing from the world'
        assert poses[name][0] == pytest.approx(x, abs=2e-3), name
        assert poses[name][1] == pytest.approx(y, abs=2e-3), name


def test_arena_is_enclosed_and_five_metres_across(poses, scenario):
    a = scenario.arena
    assert poses['wall_east'][0] - poses['wall_west'][0] == pytest.approx(a.size_x + a.wall_thickness, abs=2e-3)
    assert poses['wall_north'][1] - poses['wall_south'][1] == pytest.approx(a.size_y + a.wall_thickness, abs=2e-3)
    assert a.size_x == pytest.approx(5.0) and a.size_y == pytest.approx(5.0)


def test_robot_spawns_in_the_safe_zone_at_the_scenario_start(poses, scenario):
    x, y, z, roll, pitch, yaw = poses['fire_resq']
    s, zn = scenario.robot_start, scenario.safe_zone
    assert (x, y) == pytest.approx((s.x, s.y), abs=0.01)
    assert wrap(yaw - s.yaw) == pytest.approx(0.0, abs=0.01)
    assert z == pytest.approx(WHEEL_R, abs=1e-3)
    assert abs(x - zn.x) <= zn.size_x / 2 and abs(y - zn.y) <= zn.size_y / 2


def test_victims_are_stable_upright_and_stay_put(arena, scenario, poses):
    """Dynamic bodies (so the magnet can lift them in Phase 9) must not drift, sink or topple."""
    arena.bot.stop(3.0)
    later = entity_poses({v.id for v in scenario.victims})
    for v in scenario.victims:
        x, y, z, roll, pitch, yaw = later[v.id]
        assert (x, y) == pytest.approx((v.x, v.y), abs=1e-3), f'{v.id} drifted'
        assert abs(z) < 1e-3, f'{v.id} is not resting on the floor (z={z})'
        assert max(abs(roll), abs(pitch)) < math.radians(1.0), f'{v.id} is tilted'


def test_fire_is_static(arena, scenario):
    a = entity_poses({'fire'})['fire']
    assert (a[0], a[1]) == pytest.approx((scenario.fire.x, scenario.fire.y), abs=1e-6)


# ---------------------------------------------------------------- what the robot can see
def test_safe_zone_is_visible_and_localises_correctly(arena, scenario):
    """Green floor pixels + depth -> world points must fall inside the safe-zone rectangle."""
    bot = arena.bot
    bot.spin_wall(1.0)
    pair = bot.wait_pair()
    assert pair, 'no aligned RGB/depth pair within 4 s'
    rgb, dep = bot.rgb_array(pair[0]), bot.depth_array(pair[1])
    mask = class_masks(rgb)['safe_zone']
    assert mask.mean() > 0.05, 'safe zone should fill a visible part of the view from inside it'
    K = bot.intrinsics()
    R, T = bot.cam_to_odom(pair[0].header.stamp)
    zn = scenario.safe_zone
    vs, us = np.nonzero(mask)
    idx = np.random.default_rng(0).choice(len(us), size=min(400, len(us)), replace=False)
    inside = 0
    for u, v in zip(us[idx], vs[idx]):
        Z = dep[v, u]
        if not np.isfinite(Z):
            continue
        p = R @ np.array([(u - K[0, 2]) * Z / K[0, 0], (v - K[1, 2]) * Z / K[1, 1], Z]) + T
        wx, wy = scenario.odom_to_world(p[0], p[1])
        inside += abs(wx - zn.x) <= zn.size_x / 2 + 0.05 and abs(wy - zn.y) <= zn.size_y / 2 + 0.05
        assert p[2] == pytest.approx(-WHEEL_R, abs=0.02), 'a safe-zone pixel is not on the floor'
    assert inside / len(idx) > 0.95, f'only {inside}/{len(idx)} safe-zone points landed in the zone'


def test_survey_from_the_start_sees_fire_victim_1_and_victim_2_only(arena, scenario):
    """One full turn in place. victim_3 is behind the partition and must stay hidden;
    every detection must sit on a real object (no phantoms)."""
    dets = survey(arena.bot, scenario)
    seen, unmatched = match_entities(scenario, dets)
    assert seen == {'fire', 'victim_1', 'victim_2'}, f'saw {sorted(seen)}'
    assert not unmatched, f'detections not on any real object: {unmatched}'


def test_robot_did_not_move_during_the_spin(arena, scenario):
    """An in-place survey leaves base_link where it was (axle-midpoint regression, in the arena)."""
    g = arena.gt.get()
    assert math.hypot(g[0] - scenario.robot_start.x, g[1] - scenario.robot_start.y) < 0.02


def test_drive_through_the_arena_and_find_victim_3(arena, scenario):
    """Leave the safe zone, go round the east end of the partition, look back at victim_3.
    Open-loop route chosen by the TEST (never by the robot); it doubles as a check that the
    arena is driveable and odometry stays valid over a real trip."""
    bot, gt = arena.bot, arena.gt
    s = scenario.robot_start
    g0, o0, m = gt.get(), bot.pose(), gt.mark()

    bot.turn_to(0.0)                                  # odom yaw 0 == the start heading (pi/4 in the world)
    bot.drive_distance(2.55, speed=0.25)
    bot.turn_to(math.pi / 4)                          # face north
    bot.drive_distance(1.1, speed=0.25)

    g1, o1 = gt.get(), bot.pose()
    ex, ey = s.x + 2.55 * math.cos(s.yaw), s.y + 2.55 * math.sin(s.yaw)
    ex, ey = ex, ey + 1.1
    assert math.hypot(g1[0] - ex, g1[1] - ey) < 0.10, f'ended at ({g1[0]:.2f},{g1[1]:.2f}), planned ({ex:.2f},{ey:.2f})'
    hist = gt.since(m)
    assert max(max(abs(math.degrees(h[3])), abs(math.degrees(h[4]))) for h in hist) < 1.0, 'robot tilted: it hit something'
    assert max(h[2] for h in hist) <= WHEEL_R + 1e-4

    d_gt = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
    d_od = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    assert d_od == pytest.approx(d_gt, rel=0.03, abs=0.02), 'odometry disagrees with truth over the trip'

    dets = survey(bot, scenario)
    seen, unmatched = match_entities(scenario, dets)
    assert 'victim_3' in seen, f'victim_3 not found after exploring; saw {sorted(seen)}'
    assert 'fire' in seen
    assert not unmatched, f'detections not on any real object: {unmatched}'
