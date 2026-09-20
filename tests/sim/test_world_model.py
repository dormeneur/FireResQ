"""Phase 6 world model in the real simulation, judged against the scenario's ground truth.

The stack is arena_nav (87 deg camera) + scan-matching SLAM + perception + world model, so beliefs are expressed in a REAL
`map` frame whose `map -> odom` comes from SLAM (Phase 5 only verified `odom`). `map` starts at the robot's start pose
exactly as `odom` does, so the scenario's positions in `map` are `scenario.world_to_odom(...)` up to SLAM's error.

The scenario is read HERE, by the test, to know where things really are; the world model never sees it (unit guards).
The tests share one simulation and run in file order: each builds on the belief the previous ones left behind.
"""
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest
import yaml
from fire_resq_interfaces.msg import VictimState

from simlib import wrap

TOL_M = 0.12            # SLAM's own error (a few cm, up to ~10 cm) plus perception's; a wrong frame or estimator is metres off
CFG = Path(__file__).resolve().parents[2] / 'src' / 'fire_resq_world_model' / 'config' / 'world_model.yaml'


@pytest.fixture(scope='module')
def truth(scenario):
    t = {'fire': scenario.world_to_odom(scenario.fire.x, scenario.fire.y)}
    t.update({v.id: scenario.world_to_odom(v.x, v.y) for v in scenario.victims})
    return t


def look(bot, sim_s):
    bot.spin_sim(sim_s)
    return bot.world


def face_arena(bot):
    bot.turn_to(0.0, rate=1.0, settle=1.0)


def believed(ws):
    return {v.id: (v.position.x, v.position.y) for v in ws.victims}


def nearest_truth(truth, x, y):
    return min(((e, math.hypot(x - tx, y - ty)) for e, (tx, ty) in truth.items() if e != 'fire'), key=lambda t: t[1])


def wait_world(bot, cond, sim_s=20.0, what='the belief to reach the expected state'):
    t0 = bot.simt
    while bot.simt - t0 < sim_s:
        bot.spin_sim(0.5)
        if bot.world is not None and cond(bot.world):
            return bot.world
    raise AssertionError(f'timed out after {sim_s}s of simulation waiting for {what}; last belief: '
                         f'{[(v.id, v.status, round(v.confidence, 2)) for v in bot.world.victims] if bot.world else None}')


# ------------------------------------------------------------------------------ the belief exists and is well-formed
def test_world_state_is_published_in_map_with_the_robot_pose_and_the_configured_safe_zone(worldsim):
    bot = worldsim.bot
    ws = look(bot, 2.0)
    assert ws.header.frame_id == 'map'
    mx, my, myaw = bot.map_pose()
    assert (ws.robot_pose.position.x, ws.robot_pose.position.y) == pytest.approx((mx, my), abs=0.05)
    cfg = yaml.safe_load(CFG.read_text())['world_model_node']['ros__parameters']
    assert (ws.safe_zone_pose.position.x, ws.safe_zone_pose.position.y) == pytest.approx((cfg['safe_zone.x'], cfg['safe_zone.y']))
    # Per SIMULATION second: the node's timer follows the sim clock, so a wall-clock rate would just measure how fast the
    # host happens to run the simulator (a slowed host once gave 1.5 Hz here: exactly 0.3 x 5 at a real-time factor of 0.3).
    n0, t0 = bot.counts['world_state'], bot.simt
    bot.spin_sim(6.0)
    rate = (bot.counts['world_state'] - n0) / (bot.simt - t0)
    print(f'\nWorldState rate: {rate:.1f} Hz per simulated second (configured 5)')
    assert 3.5 <= rate <= 6.5


def test_from_the_start_pose_the_visible_victims_and_the_fire_are_believed_where_they_are(worldsim, truth):
    bot = worldsim.bot
    face_arena(bot)
    ws = wait_world(bot, lambda w: len(w.victims) >= 2 and w.fire_known
                    and all(v.status == VictimState.DETECTED for v in w.victims), what='two confirmed victims and the fire')
    assert len(ws.victims) == 2, f'victim_3 is hidden, so exactly two are believed: {believed(ws)}'
    matched = {}
    for v in ws.victims:
        e, d = nearest_truth(truth, v.position.x, v.position.y)
        assert d <= TOL_M, f'{v.id} is {100 * d:.1f} cm from the nearest real victim'
        matched[e] = d
        assert v.confidence > 0.25, f'{v.id} confidence {v.confidence:.2f}'    # pixel-count proxy: ~0.3 at ~5 m, ~1 near
    assert set(matched) == {'victim_1', 'victim_2'}, 'each real victim must be believed exactly once'
    fx, fy = truth['fire']
    assert math.hypot(ws.fire_position.x - fx, ws.fire_position.y - fy) <= TOL_M
    assert ws.fire_confidence > 0.5 and ws.current_target_id == ''
    assert max(v.confidence for v in ws.victims) > 0.8, 'the near victim should be seen with high confidence'
    print('\nstart pose, error vs truth (cm):', {e: round(100 * d, 1) for e, d in matched.items()},
          'fire', round(100 * math.hypot(ws.fire_position.x - fx, ws.fire_position.y - fy), 1))


def test_looking_all_the_way_round_creates_no_duplicates_and_moves_nobody(worldsim, truth):
    bot = worldsim.bot
    face_arena(bot)
    before = believed(look(bot, 1.0))
    yaw0, step = bot.pose()[2], math.radians(30)
    for k in range(1, 13):
        bot.turn_to(yaw0 + k * step, rate=1.0, settle=0.8)
        bot.spin_sim(0.6)
    after = believed(bot.world)
    assert set(after) == set(before), f'identities changed: {sorted(before)} -> {sorted(after)}'
    for i, (x, y) in before.items():
        assert math.hypot(after[i][0] - x, after[i][1] - y) < 0.06, f'{i} moved {100 * math.hypot(after[i][0] - x, after[i][1] - y):.1f} cm'


def test_confidence_decays_while_nothing_is_seen_and_recovers_when_the_victims_are_seen_again(worldsim):
    bot = worldsim.bot
    face_arena(bot)
    seen = {v.id: v.confidence for v in look(bot, 1.0).victims}
    bot.turn_to(math.pi, rate=1.0, settle=1.0)                                 # facing the wall behind: nothing in view
    ws = look(bot, 30.0)                                                        # one decay time constant (30 s)
    assert {v.id for v in ws.victims} == set(seen), 'unseen victims must not be deleted'
    for v in ws.victims:
        assert v.confidence < 0.6 * seen[v.id], f'{v.id}: {seen[v.id]:.2f} -> {v.confidence:.2f}'
        assert v.status == VictimState.DETECTED
    face_arena(bot)
    ws = wait_world(bot, lambda w: all(v.confidence > 0.8 * seen[v.id] for v in w.victims), what='confidence to recover')
    assert set(believed(ws)) == set(seen)


# ------------------------------------------------------------------------------ a victim nobody knew about
def test_the_hidden_victim_becomes_a_third_entity_once_seen_and_the_others_keep_their_ids(worldsim, truth, scenario):
    bot = worldsim.bot
    face_arena(bot)
    before = believed(look(bot, 1.0))
    assert len(before) == 2
    for wx, wy in [(-1.2, -1.0), (-0.4, -0.2), (-0.4, 1.3)]:                   # the test picks the route; the model is told nothing
        x, y = scenario.world_to_odom(wx, wy)
        px, py, _ = bot.pose()
        bot.turn_to(math.atan2(y - py, x - px), rate=1.0, settle=0.5)
        bot.drive_distance(math.hypot(x - px, y - py), speed=0.25)
        bot.stop(0.8)
    tx, ty = truth['victim_3']
    px, py, _ = bot.pose()
    bot.turn_to(math.atan2(ty - py, tx - px), rate=1.0, settle=1.0)
    ws = wait_world(bot, lambda w: len(w.victims) == 3 and all(v.status == VictimState.DETECTED for v in w.victims),
                    what='the third victim to be confirmed')
    after = believed(ws)
    assert set(before) < set(after), f'the original identities must survive: {sorted(before)} vs {sorted(after)}'
    assert len(after) == 3
    new = [i for i in after if i not in before][0]
    e, d = nearest_truth(truth, *after[new])
    assert e == 'victim_3' and d <= TOL_M, f'the new entity is {100 * d:.1f} cm from {e}'
    for i, (x, y) in before.items():                                            # SLAM moved the frame a little; nobody teleported
        assert math.hypot(after[i][0] - x, after[i][1] - y) < 0.15
    print(f'\nhidden victim found: error {100 * d:.1f} cm; ids {sorted(after)}')


# ------------------------------------------------------------------------------ rescue status
def test_a_rescued_victim_stays_rescued_and_is_not_duplicated_when_seen_again(worldsim, truth):
    bot = worldsim.bot
    ws = bot.world
    vic = {nearest_truth(truth, v.position.x, v.position.y)[0]: v.id for v in ws.victims}
    vid = vic['victim_3']                                                       # in view right now
    other = [i for i in believed(ws) if i != vid]
    assert not bot.set_status(vid, VictimState.CARRIED).success, 'cannot carry what was never targeted'
    for s in (VictimState.TARGETED, VictimState.CARRIED, VictimState.RESCUED):
        r = bot.set_status(vid, s)
        assert r.success, r.message
        if s == VictimState.TARGETED:
            assert wait_world(bot, lambda w: w.current_target_id == vid, what='the target to be published').current_target_id == vid
    bot.spin_sim(1.0)
    r = bot.set_status(other[0], VictimState.TARGETED)                          # another target is fine now: none is active
    assert r.success and bot.set_status(other[0], VictimState.DETECTED).success
    bot.spin_sim(8.0)                                                           # the victim is still in front of the camera
    ws = bot.world
    status = {v.id: v.status for v in ws.victims}
    assert len(ws.victims) == 3, f'seeing a rescued victim created a new entity: {[v.id for v in ws.victims]}'
    assert status[vid] == VictimState.RESCUED and ws.current_target_id == ''
    assert all(status[i] == VictimState.DETECTED for i in other)
    r = bot.set_status(vid, VictimState.DETECTED)
    assert not r.success and 'terminal' in r.message


def test_the_world_model_costs_little(worldsim):
    pid = subprocess.run(['pgrep', '-f', 'lib/fire_resq_world_model/world_model_node'], capture_output=True, text=True).stdout.split()
    assert pid, 'world_model_node is not running'

    def cpu(p):
        f = open(f'/proc/{p}/stat').read().rsplit(')', 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')
    c0, t0 = sum(cpu(p) for p in pid), time.time()
    worldsim.bot.spin_wall(8.0)
    cores = (sum(cpu(p) for p in pid) - c0) / (time.time() - t0)
    print(f'\nworld model CPU: {cores:.2f} cores')
    assert cores < 0.5
    assert np.isfinite(cores) and wrap(0.0) == 0.0
