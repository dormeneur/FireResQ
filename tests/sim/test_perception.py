"""Phase 5 perception in the real simulation, judged against the scenario's ground truth.

The scenario is read HERE, by the test, to know where things really are; the perception node never sees it
(a unit-test guard enforces that). Detections come from /fire_resq/detections, in `odom` (this launch has no
SLAM, so there is no `map`). The robot looks only while stationary or slow - that is what the node is built
to do, because RGB and depth are separate sensors offset by ~40 ms (docs/Implementation_Plan.md, Phase 3).
"""
import math
import subprocess
import time

import numpy as np

from perceptionlib import FIRE_TOL_M, VICTIM_TOL_M, collect, face_arena, nearest, seen, valid


# ------------------------------------------------------------------------------ interface
def test_detections_are_published_in_odom_at_camera_rate_with_fresh_stamps(percsim):
    bot = percsim.bot
    msgs = collect(bot, 6.0)
    rate = len(msgs) / 6.0
    assert rate >= 15.0, f'{rate:.1f} detection messages per sim second'
    assert {m.header.frame_id for m in msgs} == {'odom'}
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]
    assert stamps == sorted(stamps), 'stamps go backwards'
    lags = [t_rx - s for (t_rx, _), s in zip(list(bot.detections)[-len(msgs):], stamps)]
    assert np.median(lags) < 0.25, f'detections arrive {np.median(lags):.2f} s after the image they describe'
    for m in msgs[-5:]:
        for d in m.detections:
            assert d.header.frame_id == 'odom' and d.header.stamp == m.header.stamp


def cpu_seconds(pid):
    fields = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / 100.0            # utime + stime, 100 ticks/s


def test_perception_costs_well_under_one_core(percsim):
    """It is its own process, so it cannot stall the simulator; the budget is what SLAM and Nav2 must share."""
    pid = subprocess.run(['pgrep', '-f', 'perception_node'], capture_output=True, text=True).stdout.split()
    assert pid, 'perception_node is not running'
    c0 = sum(cpu_seconds(p) for p in pid)
    t0 = time.time()
    percsim.bot.spin_wall(8.0)
    cores = (sum(cpu_seconds(p) for p in pid) - c0) / (time.time() - t0)
    print(f'\nperception CPU: {cores:.2f} cores')
    assert cores < 0.8, f'{cores:.2f} cores'


# ------------------------------------------------------------------------------ accuracy (depth backend)
def test_from_the_start_pose_the_visible_objects_are_found_where_they_are(percsim, truth):
    face_arena(percsim.bot)
    dets = valid(collect(percsim.bot, 3.0))
    assert dets, 'nothing was localised'
    errs = seen(truth, dets)                                   # also fails on any false positive
    assert {'fire', 'victim_1', 'victim_2'} <= set(errs), f'expected fire, victim_1, victim_2 in view; got {sorted(errs)}'
    depth_share = np.mean([d.source_backend == 'depth' for d in dets])      # `auto` may fall back to known-height per blob
    assert depth_share >= 0.8, f'only {100 * depth_share:.0f}% of positions came from depth'
    for e, v in errs.items():
        assert v <= (FIRE_TOL_M if e == 'fire' else VICTIM_TOL_M), f'{e}: median error {100 * v:.1f} cm'
    print('\nstart pose (median error, cm):', {e: round(100 * v, 1) for e, v in sorted(errs.items())})


def test_a_victim_behind_the_partition_is_not_reported_until_it_is_seen(percsim, truth):
    """victim_3 is hidden from the start pose: perception must not invent it (no prior knowledge of the scenario)."""
    face_arena(percsim.bot)
    dets = valid(collect(percsim.bot, 2.0))
    assert all(nearest(truth, d)[0] != 'victim_3' for d in dets), 'victim_3 was reported without being visible'


# ------------------------------------------------------------------------------ the RGB/depth timing offset
def test_positions_are_withheld_while_the_camera_turns_and_come_back_when_it_stops(percsim, truth):
    """The measured ~40 ms RGB/depth offset: while turning, 2D detections keep flowing but NO position is claimed."""
    bot = percsim.bot
    face_arena(bot)
    assert valid(collect(bot, 1.0)), 'no positions while stationary - the test cannot show anything'
    turning = collect(bot, 2.5, wz=0.4)[6:]                    # skip the ramp-up; 1 rad, so the objects stay in view
    assert turning, 'no detections while turning'
    assert any(m.detections for m in turning), 'the 2D detections stopped too'
    assert not valid(turning), 'a position was published while the camera was turning at 0.4 rad/s'
    bot.stop(0.5)
    face_arena(bot)
    after = collect(bot, 2.0)
    assert valid(after[-10:]), 'positions did not resume after the robot stopped'


def test_positions_are_published_while_driving_slowly(percsim, truth):
    """Translation costs ~1 cm at 0.2 m/s (measured), so it is allowed: the robot need not stop to see."""
    bot = percsim.bot
    face_arena(bot)
    msgs = collect(bot, 2.5, vx=0.15)
    bot.stop(1.0)
    dets = valid(msgs)
    assert dets, 'no positions while driving at 0.15 m/s'
    x0, y0, _ = bot.pose()
    for d in dets:                                             # judge against truth in the SAME frame they were sent in
        e, dist = nearest(truth, d)
        assert dist <= (FIRE_TOL_M if e == 'fire' else VICTIM_TOL_M) * 1.5, f'{e}: {100 * dist:.1f} cm while driving'


# ------------------------------------------------------------------------------ what is seen, and only that
def test_looking_all_the_way_round_finds_every_visible_object_and_nothing_else(percsim, truth):
    bot = percsim.bot
    face_arena(bot)
    yaw0 = bot.pose()[2]
    step, dets = math.radians(30), []
    for k in range(1, 13):
        bot.turn_to(yaw0 + k * step, rate=1.0, settle=0.8)
        dets.extend(valid(collect(bot, 0.6)))
    errs = seen(truth, dets)                                   # fails on any false positive: floor, walls, obstacles
    assert {'fire', 'victim_1', 'victim_2'} <= set(errs), sorted(errs)
    print('\nfull turn (median error, cm):', {e: round(100 * v, 1) for e, v in sorted(errs.items())})


def test_the_hidden_victim_is_found_once_the_robot_has_a_line_of_sight(percsim, truth, scenario):
    """victim_3 is behind the partition: not found from the start pose (previous tests), found after moving.
    The route is chosen by the test; perception is told nothing about where to look."""
    bot = percsim.bot
    for wx, wy in [(-1.2, -1.0), (-0.4, -0.2), (-0.4, 1.3)]:
        x, y = scenario.world_to_odom(wx, wy)
        px, py, _ = bot.pose()
        bot.turn_to(math.atan2(y - py, x - px), rate=1.0, settle=0.5)
        bot.drive_distance(math.hypot(x - px, y - py), speed=0.25)
        bot.stop(0.8)
    tx, ty = truth['victim_3']
    px, py, _ = bot.pose()
    bot.turn_to(math.atan2(ty - py, tx - px), rate=1.0, settle=1.0)
    dets = [d for d in valid(collect(bot, 2.0)) if nearest(truth, d)[0] == 'victim_3']
    assert dets, 'victim_3 was not detected with a clear view'
    assert np.median([nearest(truth, d)[1] for d in dets]) <= VICTIM_TOL_M
