"""The deterministic prioritisation evaluation on the project's scenario, pinned as tests.

The WORLD is the scenario's ground truth and the PATHS come from an evaluation-only oracle over its true geometry
(tests/scenario_paths.py); see tests/sim/experiments/prioritization_eval.py, which prints the full tables. These tests pin
what was MEASURED there - the decisions and the factor values behind them - and the causal claims (the difference is the
fire; changing the world changes the decision). They read the scenario because this is evaluation, which robot code may not.
Victim ids appear only here, to name ground truth in the report; the decision code never sees them as anything but labels.
"""
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sim' / 'experiments'))
from fire_resq_cognition.config import DEFAULT_WEIGHTS, PrioritizerConfig  # noqa: E402
from fire_resq_cognition.prioritizer import make_prioritizer  # noqa: E402
from fire_resq_cognition.types import RESCUED, PlanningContext, VictimView  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402
from prioritization_eval import evaluate, world_from_scenario  # noqa: E402
from scenario_paths import ScenarioPathOracle  # noqa: E402


@pytest.fixture(scope='module')
def sc():
    return load_scenario(resolve_scenario('default'))


@pytest.fixture(scope='module')
def oracle(sc):
    return ScenarioPathOracle(sc)


@pytest.fixture(scope='module')
def result(sc, oracle):
    decisions, world, _ = evaluate(sc, oracle=oracle)
    return decisions, world


def by_id(decision):
    return {s.victim_id: s for s in decision.ranked}


# ------------------------------------------------------------------------------ the path oracle itself
def test_the_oracle_gives_sane_lengths_and_finds_the_detour_round_the_partition(sc, oracle):
    d, w = None, world_from_scenario(sc)
    ctx = PlanningContext(oracle)
    s = {x.victim_id: x for x in make_prioritizer('weighted_utility').score(w, ctx)}
    for v in s.values():
        assert v.factors['robot_path_m'] >= v.factors['straight_distance_m'] - 0.06           # a path is never shorter than straight
    assert s['victim_3'].factors['accessibility'] < 0.9                                        # the partition forces a detour
    assert s['victim_1'].factors['accessibility'] > 0.95 and s['victim_2'].factors['accessibility'] > 0.95
    assert d is None


def test_the_oracle_reports_a_goal_inside_a_solid_as_unreachable_and_never_as_a_length(sc, oracle):
    o = sc.obstacles[0]
    assert oracle((sc.robot_start.x, sc.robot_start.y), (o.x, o.y)).status == 'unreachable'
    assert oracle((sc.robot_start.x, sc.robot_start.y), (sc.fire.x, sc.fire.y)).status == 'unreachable'
    assert oracle((sc.robot_start.x, sc.robot_start.y), (99.0, 99.0)).status == 'unreachable'


# ------------------------------------------------------------------------------ the measured decisions
def test_nearest_first_and_the_weighted_utility_choose_different_victims_in_the_scenario(result, sc):
    decisions, _ = result
    weighted, nearest = decisions['weighted_utility'].selected, decisions['nearest'].selected
    assert nearest.victim_id == 'victim_1' and weighted.victim_id == 'victim_2'
    # the scenario's design intent, restated as measurements: the nearest is safe and cheap, the chosen one is endangered
    near, far = by_id(decisions['weighted_utility'])['victim_1'], by_id(decisions['weighted_utility'])['victim_2']
    assert near.factors['robot_path_m'] < far.factors['robot_path_m']
    assert far.factors['distance_to_fire_m'] < near.factors['distance_to_fire_m']
    assert far.factors['fire_risk'] > near.factors['fire_risk']


def test_both_models_report_every_victim_with_the_same_factor_values(result):
    decisions, _ = result
    a, b = by_id(decisions['weighted_utility']), by_id(decisions['nearest'])
    assert set(a) == set(b) == {'victim_1', 'victim_2', 'victim_3'}
    for vid in a:                                                                              # comparable tables: same numbers
        for k in ('distance_to_fire_m', 'fire_risk', 'robot_path_m', 'rescue_cost_m', 'accessibility'):
            assert a[vid].factors[k] == pytest.approx(b[vid].factors[k])


def test_the_evaluation_is_deterministic(sc, result):
    again, _, _ = evaluate(sc)
    for m in ('weighted_utility', 'nearest'):
        assert [(s.victim_id, s.utility) for s in again[m].ranked] == [(s.victim_id, s.utility) for s in result[0][m].ranked]


def test_the_selected_utilities_are_explained_by_their_contributions(result):
    for d in result[0].values():
        for s in d.ranked:
            assert sum(s.contributions.values()) == pytest.approx(s.utility)


# ------------------------------------------------------------------------------ causes, and the world changing the decision
def test_the_difference_between_the_models_is_the_fire(sc, oracle):
    """Take the fire away and the weighted model agrees with nearest-first: the fire terms are what separate them."""
    w = world_from_scenario(sc)
    ctx = PlanningContext(oracle)
    no_fire = replace(w, fire_known=False)
    assert make_prioritizer('weighted_utility').select(no_fire, ctx).selected.victim_id == 'victim_1'
    assert make_prioritizer('nearest').select(no_fire, ctx).selected.victim_id == 'victim_1'


def test_rescuing_the_chosen_victim_changes_what_each_model_chooses_next(sc, oracle):
    w = world_from_scenario(sc)
    ctx = PlanningContext(oracle)
    after = replace(w, victims=tuple(VictimView(v.id, v.x, v.y, v.confidence, RESCUED if v.id == 'victim_2' else v.status)
                                     for v in w.victims))
    # MEASURED: with victim_2 rescued, the two that remain are equally far from the fire, so the cheaper one is next
    assert make_prioritizer('weighted_utility').select(after, ctx).selected.victim_id == 'victim_1'
    assert make_prioritizer('nearest').select(after, ctx).selected.victim_id == 'victim_1'
    after1 = replace(w, victims=tuple(VictimView(v.id, v.x, v.y, v.confidence, RESCUED if v.id == 'victim_1' else v.status)
                                      for v in w.victims))
    assert make_prioritizer('weighted_utility').select(after1, ctx).selected.victim_id == 'victim_2'
    assert make_prioritizer('nearest').select(after1, ctx).selected.victim_id == 'victim_3'


def test_the_mvp_choice_depends_on_the_risk_weight_and_the_default_is_well_inside_the_flip_point(sc, oracle):
    """Reported, not judged: the choice flips to the nearest victim only when the risk weight falls to about 1.1, against a
    default of 3.0. If someone changes the defaults, this says how close to the boundary they have moved."""
    w = world_from_scenario(sc)
    ctx = PlanningContext(oracle)
    pick = lambda r: make_prioritizer('weighted_utility', PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'risk': r})).select(w, ctx).selected.victim_id  # noqa: E731
    assert pick(0.0) == pick(1.0) == 'victim_1' and pick(1.5) == pick(3.0) == pick(12.0) == 'victim_2'
    assert DEFAULT_WEIGHTS['risk'] > 2 * 1.1


def test_a_doubtful_nearest_victim_still_wins_when_only_confidence_argues_against_it(sc, oracle):
    """Confidence is a tie-break-tier term (weight 0.5 of a possible 1): even a nearly worthless confidence on the nearest
    victim does not overturn its large distance advantage. Measured, and worth knowing before Phase 8 tuning."""
    w = world_from_scenario(sc)
    ctx = PlanningContext(oracle)
    no_fire = replace(w, fire_known=False, victims=tuple(VictimView(v.id, v.x, v.y, 0.05 if v.id == 'victim_1' else 1.0, v.status)
                                                          for v in w.victims))
    assert make_prioritizer('weighted_utility').select(no_fire, ctx).selected.victim_id == 'victim_1'
