"""Phase 10, integrated: the WHOLE rescue mission on real physics, from a cold start, judged against Gazebo's truth.

One full mission is run once (the `mission` fixture): arena + AMCL on the saved map + Nav2 + perception + world model +
cognition + the magnet + the rescue node, autostarted, with nobody touching anything. Tests A, B, C, G and H read that one run.
Fault injection (D, E, F) needs its own run each: tests/sim/test_rescue_faults.py.

What is asserted is what happened in the WORLD (Gazebo's poses), never only what the mission REPORTED: a mission that says
"rescued" while the victim lies outside the safe zone must fail here. Ground truth is used to judge only; the rescue node has no
access to it (fire_resq_planning imports no simulator code; tests/unit/test_architecture_guards.py).
"""
import math
import time

import pytest

from conftest import mission_bring_up, mission_teardown
from fire_resq_interfaces.msg import MagnetState
from missionlib import LATCHED, TERMINAL_STATES, cpu_seconds, victim_id_by_position

MISSION_WALL_BUDGET_S = 1500.0
HAPPY_ORDER = ['INIT', 'PERCEIVE', 'UPDATE_WORLD', 'SELECT_TARGET', 'NAVIGATE_TO_APPROACH', 'ALIGN', 'ATTACH', 'VERIFY_CARRY',
               'NAVIGATE_TO_SAFE_ZONE', 'RELEASE', 'VERIFY_RELEASE', 'REASSESS']


def safe_zone_world(scenario):
    z = scenario.safe_zone
    return z.x, z.y, z.size_x, z.size_y


def inside_safe_zone(scenario, x, y, margin=0.0):
    zx, zy, sx, sy = safe_zone_world(scenario)
    return abs(x - zx) <= sx / 2 - margin and abs(y - zy) <= sy / 2 - margin


@pytest.fixture(scope='module')
def mission(arena_map_yaml, scenario):
    """Run the whole mission once and return what happened."""
    env = mission_bring_up(arena_map_yaml)
    out = {'env': env}
    try:
        bot, tap = env.bot, env.tap
        cpu0 = {k: cpu_seconds(k) for k in ('rescue_node', 'prioritizer_node', 'world_model_node', 'perception_node')}
        t0 = time.time()
        sim0 = bot.simt
        # what the world model believed before the mission moved: is the hidden victim known in advance?
        tap.wait(lambda s: s['state'] != 'INIT', 300, 'the mission to start')
        first_beliefs = None
        deadline = time.time() + MISSION_WALL_BUDGET_S
        while time.time() < deadline and tap.state not in TERMINAL_STATES:
            bot.spin_wall(0.5)
            if first_beliefs is None and tap.state in ('SELECT_TARGET', 'NAVIGATE_TO_APPROACH'):
                first_beliefs = [(v.id, v.position.x, v.position.y) for v in bot.world.victims]
        out.update(final=tap.last, wall_s=time.time() - t0, sim_s=bot.simt - sim0, first_beliefs=first_beliefs,
                   cpu={k: cpu_seconds(k) - cpu0[k] for k in cpu0}, world=bot.world)
        # let the last physics settle, then snapshot the truth of every victim
        bot.spin_wall(3.0)
        out['truth'] = {n: env.truth.latest(n) for n in ('victim_1', 'victim_2', 'victim_3', 'fire_resq')}
        yield out
    finally:
        mission_teardown(env)


def victim_names_to_ids(mission, scenario):
    """Scenario victim -> the world model's id, matched by POSITION (the ids are the world model's, in creation order)."""
    w = mission['world']
    return {v.id: victim_id_by_position(w, scenario.world_to_odom(v.x, v.y)) for v in scenario.victims}


# ------------------------------------------------------------------------------------------------ A, H
def test_the_mission_runs_from_a_cold_start_to_completion(mission):
    f = mission['final']
    print(f"\nMEASURED mission: {f['state']} in {mission['sim_s']:.0f} s simulated ({mission['wall_s']:.0f} s wall); rescued {f['rescued']}, "
          f"given up {f['given_up']}, attempts {f['attempts']}, metrics {f['metrics']}, coverage {f['coverage']}")
    assert f['state'] == 'MISSION_COMPLETE', f"{f['state']}: {f['reason']}"
    assert not f['given_up'], f"the mission gave up on {f['given_up']}"


def test_every_victim_is_rescued_and_lies_inside_the_safe_zone(mission, scenario):
    f = mission['final']
    assert len(f['rescued']) == len(scenario.victims), f"rescued {f['rescued']}"
    for v in scenario.victims:
        t = mission['truth'][v.id]
        assert t is not None
        print(f"MEASURED {v.id} ends at ({t[1]:.2f}, {t[2]:.2f}); safe zone centre ({scenario.safe_zone.x:.2f}, {scenario.safe_zone.y:.2f})")
        assert inside_safe_zone(scenario, t[1], t[2], margin=0.03), f'{v.id} is not inside the safe zone: ({t[1]:.2f}, {t[2]:.2f})'
    for v in mission['world'].victims:
        assert v.status == 4, f'{v.id} is {v.status}, not RESCUED, in the world model'


def test_the_first_rescue_follows_the_declared_state_sequence(mission):
    path = [mission['final']['history'][0][1]] + [h[2] for h in mission['final']['history']]
    assert path[:len(HAPPY_ORDER)] == HAPPY_ORDER, path[:len(HAPPY_ORDER) + 3]


def test_the_mission_never_reversed(mission):
    """The robot is blind behind itself: neither the FSM nor Nav2 may ever command a negative linear velocity."""
    cmds = mission['env'].tap.cmd
    assert cmds
    assert min(c[1] for c in cmds) >= -1e-6, f'a reverse command was issued: {min(c[1] for c in cmds)}'
    assert max(abs(c[1]) for c in cmds) <= 0.30 + 1e-6 and max(abs(c[2]) for c in cmds) <= 1.5 + 1e-6, 'a velocity limit was exceeded'


# ------------------------------------------------------------------------------------------------ B
def test_cognition_not_a_fixed_order_chose_who_to_rescue_and_it_is_not_nearest_first(mission, scenario):
    hist = mission['final']['history']
    ids = victim_names_to_ids(mission, scenario)
    by_id = {v: k for k, v in ids.items()}
    order = [by_id.get(h[4]) for h in hist if h[1] == 'SELECT_TARGET' and h[2] == 'NAVIGATE_TO_APPROACH']
    firsts = []
    for x in order:                                          # first selection of each victim
        if x not in firsts:
            firsts.append(x)
    r = scenario.robot_start
    nearest = min(scenario.victims, key=lambda v: math.hypot(v.x - r.x, v.y - r.y)).id
    print(f'MEASURED order of first selection: {firsts}; nearest to the start: {nearest}')
    assert firsts[0] != nearest, 'the first rescue was the nearest victim: cognition made no difference'
    assert sorted(firsts) == sorted(v.id for v in scenario.victims)


def test_a_rescued_victim_is_never_selected_again(mission):
    hist = mission['final']['history']
    rescued_at = {}
    for h in hist:
        if h[1] == 'VERIFY_RELEASE' and h[2] == 'REASSESS':
            rescued_at[h[4]] = h[0]
    for h in hist:
        if h[1] == 'SELECT_TARGET' and h[2] == 'NAVIGATE_TO_APPROACH' and h[4] in rescued_at:
            assert h[0] < rescued_at[h[4]], f'{h[4]} was selected again after it was rescued'


# ------------------------------------------------------------------------------------------------ C
def test_the_hidden_victim_is_not_known_in_advance_but_is_found_and_rescued(mission, scenario):
    hidden = next(v for v in scenario.victims if v.id == 'victim_3')
    hxy = scenario.world_to_odom(hidden.x, hidden.y)
    early = mission['first_beliefs']
    assert early is not None
    print(f'MEASURED beliefs at the first selection: {early}')
    assert not any(math.hypot(x - hxy[0], y - hxy[1]) < 0.6 for _, x, y in early), \
        'victim_3 was already believed at the first selection: it is supposed to be hidden from the start'
    ids = victim_names_to_ids(mission, scenario)
    assert ids['victim_3'] is not None and ids['victim_3'] in mission['final']['rescued']


# ------------------------------------------------------------------------------------------------ G (and the release, physically)
def test_the_release_put_each_victim_down_inside_the_zone_not_on_top_of_another(mission, scenario):
    pts = [mission['truth'][v.id] for v in scenario.victims]
    for i, a in enumerate(pts):
        for b in pts[i + 1:]:
            assert math.hypot(a[1] - b[1], a[2] - b[2]) > 0.12, 'two victims ended on top of each other'
    for v in scenario.victims:
        t = mission['truth'][v.id]
        assert abs(t[3]) < 0.02, f'{v.id} is not resting on the floor (z {t[3]:.3f}): still lifted or toppled'


def test_the_magnet_was_left_off_and_holding_nothing(mission):
    env = mission['env']
    m = mission['final']
    assert m['metrics']['magnet_requests'] >= 2 * len(m['rescued'])
    box = {}                                                  # the last reported state, read through the driver's own topic
    sub = env.bot.create_subscription(MagnetState, '/fire_resq/magnet/state', lambda s: box.setdefault('s', s), LATCHED)
    env.bot.wait_for(lambda: 's' in box, 10, 'a MagnetState')
    env.bot.destroy_subscription(sub)
    assert box['s'].energized is False and box['s'].attached is False


# ------------------------------------------------------------------------------------------------ performance (reported, not gated)
def test_the_mission_costs_are_reported(mission):
    f = mission['final']
    print(f"MEASURED performance: mission {mission['sim_s']:.0f} s simulated; state transitions {f['metrics']['transitions']}; "
          f"SelectTarget calls {f['metrics']['select_calls']}; Nav2 goals {f['metrics']['nav_goals']} ({f['metrics']['nav_failures']} failed); "
          f"magnet requests {f['metrics']['magnet_requests']} ({f['metrics']['attach_refusals']} refused); abandoned attempts "
          f"{f['metrics']['abandons']}; perception messages seen {f.get('detections_seen')}; "
          f"CPU seconds over the mission: { {k: round(v, 1) for k, v in mission['cpu'].items()} } "
          f"(= {', '.join(f'{k} {v / max(mission['sim_s'], 1):.2f} cores' for k, v in mission['cpu'].items())})")
    assert f['metrics']['transitions'] >= 12 * len(f['rescued'])
    assert mission['cpu']['rescue_node'] / max(mission['sim_s'], 1) < 1.0, 'the rescue node alone should cost well under a core'
