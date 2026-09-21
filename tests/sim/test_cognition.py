"""Phase 7 cognition in the real simulation: the real world model, the real Nav2 planner, the real decision nodes.

The whole stack runs (AMCL on the saved map, Nav2, perception, world model) with TWO cognition nodes: the MVP weighted
utility and the nearest baseline, each asking Nav2's `compute_path_to_pose` for path lengths. The tests judge the cognitive
layer, not the robot: does it consume what the world model believes, ask the planner, decide, explain itself, and re-decide
when the world changes. The scenario is read HERE only to name which real victim a believed one is (the world model uses
its own ids, `V1..`, and the decision code sees nothing but positions).

The tests share one simulation and run in file order: each builds on the belief the previous ones left behind.
"""
import math
import os
import subprocess
import time

import pytest
from fire_resq_cognition.config import WEIGHT_NAMES
from fire_resq_cognition.prioritizer import make_prioritizer
from fire_resq_cognition.types import PathInfo, PlanningContext, VictimView, WorldView
from fire_resq_interfaces.msg import VictimState

TOL_APPROACH = 0.02                  # replay uses the report's own world, so the approach points reproduce exactly
LIVE_TOL_M = 0.10                    # the planner's grid and the AMCL pose: the path to a point is never shorter than straight


@pytest.fixture(scope='module')
def truth(scenario):
    t = {v.id: scenario.world_to_odom(v.x, v.y) for v in scenario.victims}
    return t


def which(truth, x, y):
    """Ground-truth name of the real victim a believed position belongs to (evaluation only)."""
    return min(((e, math.hypot(x - tx, y - ty)) for e, (tx, ty) in truth.items()), key=lambda t: t[1])[0]


def face_arena(bot):
    bot.turn_to(0.0, rate=1.0, settle=1.0)


def wait_for(bot, cond, sim_s=40.0, what='the condition'):
    t0 = bot.simt
    while bot.simt - t0 < sim_s:
        bot.spin_sim(0.5)
        if cond():
            return
    raise AssertionError(f'timed out after {sim_s}s of simulation waiting for {what}')


def believed(bot, status=VictimState.DETECTED):
    return {v.id: (v.position.x, v.position.y) for v in bot.world.victims if v.status == status}


def settle_decisions(bot, n_victims, sim_s=90.0):
    """Wait until BOTH models have selected someone on a world of n confirmed victims, each decision made ON such a world.
    Not merely "a report exists": a decision made while Nav2 is still starting selects nothing (and is retried), and a report
    made a moment earlier, when a victim was still tentative, would describe a different world than the one we now read."""
    def decided_on_n_confirmed(rep):
        vs = rep['world']['victims']
        return len(vs) == n_victims and all(v['status'] == VictimState.DETECTED for v in vs) and len(rep['candidates']) == n_victims

    def ready():
        w = bot.world
        if w is None or sum(1 for v in w.victims if v.status == VictimState.DETECTED) != n_victims:
            return False
        return all(tag in bot.reports and bot.reports[tag]['selected'] and bot.targets[tag].victim_id
                   and decided_on_n_confirmed(bot.reports[tag]) for tag in ('mvp', 'nearest'))
    wait_for(bot, ready, sim_s, f'both models to select a victim on {n_victims} confirmed victims')


def replay(report_candidates, world_view, model):
    """Re-run the pure library on the world the node saw, answering path queries with the lengths the node's planner gave."""
    lengths = {}
    for c in report_candidates:
        if c['eligible']:
            lengths[tuple(round(v, 2) for v in c['approach_xy'])] = (c['factors']['robot_path_m'], c['factors']['return_path_m'])

    def query(a, b):
        for approach, (out, back) in lengths.items():
            if math.hypot(b[0] - approach[0], b[1] - approach[1]) < TOL_APPROACH:
                return PathInfo.reachable(out)                    # robot -> approach point
            if math.hypot(a[0] - approach[0], a[1] - approach[1]) < TOL_APPROACH:
                return PathInfo.reachable(back)                   # approach point -> safe zone
        return PathInfo.unreachable('not a leg the node queried')
    return make_prioritizer(model).select(world_view, PlanningContext(query))


def world_view(report):
    """The WorldView a decision was made on, taken from ITS OWN report (the world keeps moving after it is published)."""
    w = report['world']
    return WorldView(w['stamp'], tuple(VictimView(v['id'], v['x'], v['y'], v['confidence'], v['status']) for v in w['victims']),
                     w['fire_known'], tuple(w['fire']), tuple(w['safe_zone']), tuple(w['robot']))


# ------------------------------------------------------------------------------ consuming the world model
def test_cognition_consumes_the_world_model_asks_nav2_and_publishes_an_explained_rescue_target(cogsim, truth):
    bot = cogsim.bot
    face_arena(bot)
    settle_decisions(bot, 2)
    t, rep = bot.targets['mvp'], bot.reports['mvp']
    ids = believed(bot)
    assert t.victim_id in ids, f'the target {t.victim_id!r} is not something the world model believes in: {sorted(ids)}'
    assert t.header.frame_id == 'map' and rep['model'] == 'weighted_utility' and rep['selected'] == t.victim_id
    assert {c['victim_id'] for c in rep['candidates']} == set(ids)
    for c in rep['candidates']:
        assert c['eligible'], c['exclusion']
        f = c['factors']
        assert f['robot_path_m'] >= f['straight_distance_m'] - LIVE_TOL_M, 'a planned path cannot be shorter than the straight line'
        assert f['rescue_cost_m'] == pytest.approx(f['robot_path_m'] + f['return_path_m'])
        assert sum(c['contributions'].values()) == pytest.approx(c['utility'])
        assert set(c['contributions']) == set(WEIGHT_NAMES)
    assert list(t.breakdown.weight_names) == list(WEIGHT_NAMES) and t.breakdown.total == pytest.approx(t.utility)
    assert t.rationale.startswith('weighted_utility: selected') and all(v in t.rationale for v in ids)
    print('\nlive MVP decision on the live world:', t.victim_id, {v: which(truth, *p) for v, p in ids.items()})


def test_the_node_decides_exactly_what_the_library_decides_from_the_same_world_and_planner_answers(cogsim):
    """Guards the ROS conversion: replay the pure prioritizer on the world the node saw, with the planner's own numbers."""
    bot = cogsim.bot
    settle_decisions(bot, 2)
    for tag, model in (('mvp', 'weighted_utility'), ('nearest', 'nearest')):
        rep = bot.reports[tag]
        d = replay(rep['candidates'], world_view(rep), model)
        assert d.selected is not None and d.selected.victim_id == rep['selected'], f'{model}: the node and the library disagree'
        for c in rep['candidates']:
            mine = next(s for s in d.ranked if s.victim_id == c['victim_id'])
            assert mine.utility == pytest.approx(c['utility'], abs=1e-6), f"{model}: {c['victim_id']}"


def test_the_mvp_and_the_nearest_baseline_decide_differently_on_the_same_live_world(cogsim, truth):
    bot = cogsim.bot
    settle_decisions(bot, 2)
    mvp, near = bot.reports['mvp'], bot.reports['nearest']
    real = {v: which(truth, *p) for v, p in believed(bot).items()}
    print('\nlive, two victims known:  MVP ->', real[mvp['selected']], '  nearest ->', real[near['selected']])
    why = {t: [(c['victim_id'], c['eligible'], c['exclusion'], c['utility']) for c in r['candidates']] for t, r in (('mvp', mvp), ('nearest', near))}
    assert {real[mvp['selected']], real[near['selected']]} == {'victim_1', 'victim_2'}, why
    assert real[near['selected']] == 'victim_1' and real[mvp['selected']] == 'victim_2', why
    # The two nodes decide independently, a moment apart, but on the same world model and the same planner: their tables must
    # describe the same victims at (nearly) the same places with (nearly) the same numbers. The exact check is the replay.
    wa, wb = {v['id']: v for v in mvp['world']['victims']}, {v['id']: v for v in near['world']['victims']}
    assert set(wa) == set(wb)
    for vid in wa:
        assert math.hypot(wa[vid]['x'] - wb[vid]['x'], wa[vid]['y'] - wb[vid]['y']) < 0.15
    a = {c['victim_id']: c['factors'] for c in mvp['candidates']}
    b = {c['victim_id']: c['factors'] for c in near['candidates']}
    for vid in a:
        assert a[vid]['fire_risk'] == pytest.approx(b[vid]['fire_risk'], abs=0.15)
        assert a[vid]['robot_path_m'] == pytest.approx(b[vid]['robot_path_m'], abs=0.5)


# ------------------------------------------------------------------------------ a victim nobody knew about
def test_a_newly_found_victim_joins_the_decision_with_its_detour_measured_by_the_real_planner(cogsim, truth, scenario):
    bot = cogsim.bot
    for wx, wy in [(-1.2, -1.0), (-0.4, -0.2), (-0.4, 1.3)]:                   # the test picks the route; cognition is told nothing
        bot.go_to(*scenario.world_to_odom(wx, wy))
    tx, ty = truth['victim_3']
    px, py, _ = bot.pose()
    bot.turn_to(math.atan2(ty - py, tx - px), rate=1.0, settle=1.0)
    wait_for(bot, lambda: len(believed(bot)) == 3, 40.0, 'the third victim to be confirmed')
    for wx, wy in [(-0.4, -0.2), (-1.2, -1.0), (-1.9, -1.9)]:                  # ...and back to the start, facing the arena
        bot.go_to(*scenario.world_to_odom(wx, wy))
    face_arena(bot)
    settle_decisions(bot, 3, 90.0)
    real = {v: which(truth, *p) for v, p in believed(bot).items()}
    assert sorted(real.values()) == ['victim_1', 'victim_2', 'victim_3']
    rep = bot.reports['mvp']
    by_real = {real[c['victim_id']]: c for c in rep['candidates']}
    v3, v1 = by_real['victim_3']['factors'], by_real['victim_1']['factors']
    assert v3['robot_path_m'] > v3['straight_distance_m'] + 0.3, 'the partition should force a detour'
    assert v3['accessibility'] < 0.9 < v1['accessibility']
    print('\nlive, all three known: directness victim_3 %.2f vs victim_1 %.2f; paths %.2f / %.2f m'
          % (v3['accessibility'], v1['accessibility'], v3['robot_path_m'], v1['robot_path_m']))


def test_live_evaluation_all_three_victims_known(cogsim, truth):
    """The live counterpart of tests/sim/experiments/prioritization_eval.py: the real world model and the real planner.
    Reported, and checked against the offline decisions (which used ground truth and confidence 1.0)."""
    bot = cogsim.bot
    settle_decisions(bot, 3, 60.0)
    real = {v: which(truth, *p) for v, p in believed(bot).items()}
    print('\n=== LIVE evaluation, three victims known, robot at the start pose ===')
    picks = {}
    for tag in ('mvp', 'nearest'):
        rep = bot.reports[tag]
        picks[tag] = real[rep['selected']]
        print(f"  {rep['model']:<17} selects {picks[tag]}   weights {rep['weights']}")
        for c in rep['candidates']:
            f = c['factors']
            print(f"    {real[c['victim_id']]:<9} ({c['victim_id']})  U={c['utility']:+.3f}  d_fire {f['distance_to_fire_m']:.2f}  risk {f['fire_risk']:.2f}  "
                  f"trip {f['robot_path_m']:.2f}  cost {f['rescue_cost_m']:.2f}  direct {f['accessibility']:.2f}  conf {f['confidence']:.2f}")
    assert picks == {'mvp': 'victim_2', 'nearest': 'victim_1'}


# ------------------------------------------------------------------------------ reassessment and SelectTarget
def test_when_the_world_model_changes_cognition_reassesses_and_the_rescued_victim_is_never_chosen_again(cogsim, truth):
    bot = cogsim.bot
    settle_decisions(bot, 3)
    first = bot.targets['mvp'].victim_id
    n0 = len(bot.world.victims)
    for status in (VictimState.TARGETED, VictimState.CARRIED, VictimState.RESCUED):     # what the rescue side will do
        r = bot.set_status(first, status)
        assert r.success, r.message
    wait_for(bot, lambda: bot.targets['mvp'].victim_id not in ('', first), 40.0, 'the MVP to re-decide after the rescue')
    wait_for(bot, lambda: bot.targets['nearest'].victim_id not in ('', first), 40.0, 'the baseline to re-decide after the rescue')
    assert len(bot.world.victims) == n0
    for tag in ('mvp', 'nearest'):
        rep = bot.reports[tag]
        ex = {c['victim_id']: c for c in rep['candidates'] if not c['eligible']}
        assert first in ex and 'already rescued' in ex[first]['exclusion'] and rep['selected'] != first
    real = {v: which(truth, *p) for v, p in believed(bot).items()}
    rescued = next(v for v in bot.world.victims if v.id == first)
    print('\nafter rescuing', which(truth, rescued.position.x, rescued.position.y),
          ': MVP ->', real[bot.reports['mvp']['selected']], ' nearest ->', real[bot.reports['nearest']['selected']])


def test_select_target_answers_from_the_current_belief_and_costs_little(cogsim):
    bot = cogsim.bot
    r = bot.select_target('mvp')
    assert r.found and r.reason == '' and r.target.victim_id != ''
    live = {v.id: v.status for v in bot.world.victims}
    assert live[r.target.victim_id] == VictimState.DETECTED, 'SelectTarget chose something that is not a candidate'
    assert bot.world.current_target_id == ''                                             # cognition SELECTS; it never marks TARGETED
    pids = subprocess.run(['pgrep', '-f', 'lib/fire_resq_cognition/prioritizer_node'], capture_output=True, text=True).stdout.split()
    assert len(pids) >= 2, 'both cognition nodes should be running'

    def cpu(p):
        f = open(f'/proc/{p}/stat').read().rsplit(')', 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')
    marker = cogsim.sim.log().count('decision #')
    c0, t0 = sum(cpu(p) for p in pids), time.time()
    bot.spin_wall(8.0)
    cores = (sum(cpu(p) for p in pids) - c0) / (time.time() - t0)
    print(f'\ncognition CPU (both nodes, idle world): {cores:.2f} cores')
    lines = [ln for ln in cogsim.sim.log().splitlines() if 'decision #' in ln][marker:]
    assert len(lines) <= 1, 'an idle world must not keep triggering decisions; it decided again because:\n' + '\n'.join(lines)
    assert cores < 1.0, f'{cores:.2f} cores for two nodes'          # measured ~0.3 per node: mostly rclpy handling the 250 Hz sim clock
