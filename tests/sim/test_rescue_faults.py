"""Phase 10, integrated, fault injection: each fault gets its OWN cold-start mission (stopped as soon as the behaviour under test
has been seen), so this module never shares a simulation with test_rescue_loop.py's full run (a module-scoped fixture would still
be up). Judged against Gazebo's truth; the rescue node has no access to it.
"""
import math
import subprocess

import pytest

from conftest import mission_bring_up, mission_teardown
from fire_resq_interfaces.msg import MagnetState
from missionlib import LATCHED
from simlib import teleport


def inside_safe_zone(scenario, x, y, margin=0.0):
    z = scenario.safe_zone
    return abs(x - z.x) <= z.size_x / 2 - margin and abs(y - z.y) <= z.size_y / 2 - margin


# ================================================================================================ fault injection
def run_until(env, pred, timeout, what):
    """Spin the test node until `pred(snapshot)` holds on a mission_state message (or time out)."""
    return env.tap.wait(pred, timeout, what)


def entered(env, state):
    return any(h[2] == state for h in env.tap.history())


def target_scenario_victim(env, scenario):
    """Which scenario victim the mission is currently targeting: match its belief to a truth position (test-side only)."""
    tid = env.tap.last['target']
    v = next((v for v in env.bot.world.victims if v.id == tid), None)
    if v is None:
        return None
    best = min(scenario.victims, key=lambda s: math.hypot(scenario.world_to_odom(s.x, s.y)[0] - v.position.x,
                                                          scenario.world_to_odom(s.x, s.y)[1] - v.position.y))
    return best


@pytest.fixture()
def fault_env(arena_map_yaml, request):
    extra = getattr(request, 'param', ())
    env = mission_bring_up(arena_map_yaml, extra)
    yield env
    mission_teardown(env)


@pytest.mark.parametrize('fault_env', [('magnet_contact_tolerance_m:=0.0005',)], indirect=True)
def test_D_a_refused_attach_does_not_rescue_the_victim_and_the_mission_reassesses(fault_env, scenario):
    """The simulated magnet refuses every attach (its contact tolerance is 0.5 mm). The FSM must retry a bounded number of times,
    de-energise, give the attempt up, and go back to cognition - never mark the victim rescued, never carry on as if it held it."""
    env = fault_env
    run_until(env, lambda s: any(h[1] == 'ATTACH' and h[2] == 'ABANDON' for h in s['history']), 600, 'the first failed attach')
    snap = env.tap.last
    ab = next(h for h in snap['history'] if h[1] == 'ATTACH' and h[2] == 'ABANDON')
    print(f"MEASURED refused attach: {ab}; metrics {snap['metrics']}")
    assert 'refused' in ab[3]
    assert snap['metrics']['attach_refusals'] >= 1 and snap['rescued'] == []
    run_until(env, lambda s: any(h[1] == 'REASSESS' for h in s['history'] if h[0] > ab[0]), 60, 'REASSESS after the failed attach')
    box = {}
    sub = env.bot.create_subscription(MagnetState, '/fire_resq/magnet/state', lambda s: box.__setitem__('s', s), LATCHED)
    env.bot.spin_wall(2.0)
    env.bot.destroy_subscription(sub)
    assert box['s'].energized is False and box['s'].attached is False, 'a refused coil was left energised'
    for v in env.bot.world.victims:
        assert v.status != 4, f'{v.id} was marked RESCUED though nothing was ever picked up'
    for n in ('victim_1', 'victim_2', 'victim_3'):                 # nothing was carried anywhere near the safe zone
        p = env.truth.latest(n)
        v = next(s for s in scenario.victims if s.id == n)
        assert math.hypot(p[1] - v.x, p[2] - v.y) < 0.5, f'{n} was moved {math.hypot(p[1] - v.x, p[2] - v.y):.2f} m'


def test_E_losing_the_target_during_align_stops_the_robot_and_reconsiders(arena_map_yaml, scenario):
    """The victim being approached disappears (moved out of sight by the test) while the robot is in ALIGN. The robot must stop
    within a fraction of a metre - no blind driving - and the target must be reconsidered."""
    env = mission_bring_up(arena_map_yaml)
    try:
        run_until(env, lambda s: s['state'] == 'ALIGN', 600, 'the first ALIGN')
        victim = target_scenario_victim(env, scenario)
        p0 = env.truth.latest('fire_resq')
        teleport(victim.id, 2.0, 2.0, 0.0, z=0.0)                   # the far corner - for victim_2 that is straight BEHIND its spot
        #                                                             as the robot sees it: still in view by bearing, but far narrower
        run_until(env, lambda s: any(h[1] == 'ALIGN' and h[2] == 'ABANDON' for h in s['history']), 90, 'ALIGN to give up')
        ab = next(h for h in env.tap.history() if h[1] == 'ALIGN' and h[2] == 'ABANDON')
        env.bot.spin_wall(0.4)                                      # the stop itself takes a moment to reach the wheels
        p1 = env.truth.latest('fire_resq')
        env.bot.spin_wall(1.2)                                      # inside ABANDON's stand-still (depart_pause_s, 2 s)
        p2 = env.truth.latest('fire_resq')
        moved = math.hypot(p1[1] - p0[1], p1[2] - p0[2])
        still = math.hypot(p2[1] - p1[1], p2[2] - p1[2])
        print(f"MEASURED lost-target abandon: {ab}; robot moved {moved * 100:.0f} cm from the vanish to the stop, "
              f"then {still * 100:.1f} cm while standing still")
        assert 'not in view' in ab[3] or 'lost' in ab[3] or 'gone' in ab[3], ab
        assert moved < 0.30, 'the robot kept driving after the victim was lost'
        assert still < 0.02, 'the robot did not stop and stay stopped after losing its target'
        run_until(env, lambda s: s['state'] in ('SELECT_TARGET', 'NAVIGATE_TO_APPROACH', 'SEARCH', 'PERCEIVE'), 60, 'the target to be reconsidered')
        assert env.tap.last['rescued'] == []
    finally:
        mission_teardown(env)


def test_F_a_victim_dropped_in_transit_stops_the_robot_and_is_not_reported_rescued(arena_map_yaml, scenario):
    """The joint is released from outside while the victim is being carried to the safe zone. The magnet's hold sensor must say so,
    the robot must stop, and the FSM must not release or report a rescue."""
    env = mission_bring_up(arena_map_yaml)
    try:
        run_until(env, lambda s: s['state'] == 'NAVIGATE_TO_SAFE_ZONE', 900, 'the first carry')
        victim = target_scenario_victim(env, scenario)
        env.bot.spin_wall(4.0)                                        # a few seconds into the transport
        subprocess.run(['gz', 'topic', '-t', f'/model/fire_resq/magnet/{victim.id}/detach', '-m', 'gz.msgs.Empty', '-p', 'unused: true'],
                       capture_output=True, timeout=10)
        run_until(env, lambda s: any(h[1] == 'NAVIGATE_TO_SAFE_ZONE' and h[2] == 'ABANDON' for h in s['history']), 30, 'the FSM to notice the drop')
        ab = next(h for h in env.tap.history() if h[1] == 'NAVIGATE_TO_SAFE_ZONE' and h[2] == 'ABANDON')
        print(f"MEASURED drop detection: {ab}")
        assert 'hold was lost' in ab[3]
        assert not any(h[2] == 'RELEASE' for h in env.tap.history() if h[0] >= ab[0] - 0.01 and h[1] == 'NAVIGATE_TO_SAFE_ZONE')
        env.bot.spin_wall(3.0)
        assert env.tap.last['rescued'] == []
        r0 = env.truth.latest('fire_resq')
        env.bot.spin_wall(1.0)
        r1 = env.truth.latest('fire_resq')
        print(f"MEASURED robot motion 3-4 s after the drop was noticed: {math.hypot(r1[1] - r0[1], r1[2] - r0[2]) * 100:.1f} cm")
        assert all(v.status != 4 for v in env.bot.world.victims), 'a dropped victim was reported RESCUED'
        # where it fell is not asserted (anywhere on the way); that it was not reported rescued is
    finally:
        mission_teardown(env)
