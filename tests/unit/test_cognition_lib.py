"""The decision layer as a pure library: factors, weighted utility, nearest baseline, eligibility, explanation.

No ROS, no simulator: the planner is a fake callable. Victim ids in these scenes are arbitrary strings - nothing in
cognition may depend on them - and every scene is built from positions, so the rules are pinned independently of the
project's scenario.
"""
import json
import math
import random
import re
from pathlib import Path

import pytest
from fire_resq_cognition import factors as F
from fire_resq_cognition.config import DEFAULT_WEIGHTS, WEIGHT_NAMES, PrioritizerConfig
from fire_resq_cognition.prioritizer import NearestVictimPrioritizer, WeightedUtilityPrioritizer, make_prioritizer
from fire_resq_cognition.types import (CARRIED, DETECTED, RESCUED, STATUS_NAMES, TARGETED, UNKNOWN, UNREACHABLE, PathInfo,
                                       PlanningContext, VictimView, WorldView)

REPO = Path(__file__).resolve().parents[2]
CFG = PrioritizerConfig()


# ------------------------------------------------------------------------------ scene builders
def straight_query(detour=None, blocked=None):
    """A fake planner: straight-line lengths, optionally lengthened by `detour(start, goal)`, optionally blocked."""
    def q(a, b):
        if blocked and blocked(a, b):
            return PathInfo.unreachable('blocked in the test')
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        return PathInfo.reachable(d * (detour(a, b) if detour else 1.0))
    return q


def scene(victims, robot=(0.0, 0.0), fire=(6.0, 0.0), safe=(0.0, 0.0), stamp=1.0):
    vs = tuple(VictimView(*v) if not isinstance(v, VictimView) else v for v in victims)
    return WorldView(stamp, vs, fire is not None, fire or (0.0, 0.0), safe, robot)


def V(vid, x, y, conf=0.9, status=DETECTED):
    return VictimView(vid, x, y, conf, status)


def ctx(**kw):
    return PlanningContext(straight_query(**kw))


# ------------------------------------------------------------------------------ factors
def test_fire_risk_is_one_inside_the_hazard_radius_and_decays_beyond_it():
    assert F.fire_risk(0.0, CFG) == F.fire_risk(CFG.fire_hazard_radius_m, CFG) == 1.0
    a, b, c = (F.fire_risk(CFG.fire_hazard_radius_m + d, CFG) for d in (0.5, 1.0, 3.0))
    assert 1.0 > a > b > c > 0.0
    assert b == pytest.approx(math.exp(-1.0 / CFG.fire_risk_decay_m))


def test_proximity_terms_are_linear_and_bounded():
    assert F.fire_proximity(0.0, CFG) == 1.0 and F.fire_proximity(CFG.extent_m, CFG) == 0.0
    assert F.fire_proximity(10 * CFG.extent_m, CFG) == 0.0                       # clamped, never negative
    assert F.robot_proximity(CFG.extent_m / 2, CFG) == pytest.approx(0.5)
    assert F.rescue_cost_norm(2 * CFG.extent_m, CFG) == 1.0 and F.rescue_cost_norm(0.0, CFG) == 0.0


def test_accessibility_is_the_directness_of_the_route():
    assert F.accessibility(3.0, 3.0) == 1.0
    assert F.accessibility(3.0, 6.0) == pytest.approx(0.5)
    assert F.accessibility(3.0, 2.0) == 1.0                                       # a path cannot be more direct than straight
    assert F.accessibility(0.0, 0.0) == 1.0


def test_approach_point_is_a_standoff_short_of_the_victim_on_the_line_from_the_robot():
    p = F.approach_point((0.0, 0.0), (3.0, 4.0), 1.0)
    assert math.hypot(p[0] - 3.0, p[1] - 4.0) == pytest.approx(1.0)
    assert p[0] / p[1] == pytest.approx(3.0 / 4.0)                                # on the robot->victim line
    assert F.approach_point((1.0, 1.0), (1.2, 1.0), 0.4) == (1.0, 1.0)            # already within the standoff: stay


# ------------------------------------------------------------------------------ config
def test_default_weights_are_exactly_the_documented_set_and_all_positive():
    assert set(DEFAULT_WEIGHTS) == set(WEIGHT_NAMES) == set(PrioritizerConfig().weights)
    assert all(w > 0 for w in DEFAULT_WEIGHTS.values())


def test_config_rejects_bad_values_and_unknown_keys():
    for bad in ({'weights': {'risk': 1.0}}, {'weights': {**DEFAULT_WEIGHTS, 'risk': -1.0}},
                {'weights': {k: 0.0 for k in WEIGHT_NAMES}}, {'weights': {**DEFAULT_WEIGHTS, 'luck': 1.0}},
                {'extent_m': 0.0}, {'fire_hazard_radius_m': -1.0}, {'approach_standoff_m': -0.1}):
        with pytest.raises(ValueError):
            PrioritizerConfig(**bad)
    with pytest.raises(ValueError, match='unknown'):
        PrioritizerConfig.from_dict({'weights.rsk': 1.0})
    with pytest.raises(ValueError, match='unknown'):
        PrioritizerConfig.from_dict({'extent': 7.0})


def test_config_from_dotted_and_nested_parameters_agree_and_keep_defaults():
    a = PrioritizerConfig.from_dict({'weights.risk': 9.0, 'extent_m': 5.0})
    b = PrioritizerConfig.from_dict({'weights': {'risk': 9.0}, 'extent_m': 5.0})
    assert a == b and a.weights['risk'] == 9.0 and a.weights['cost'] == DEFAULT_WEIGHTS['cost'] and a.extent_m == 5.0


# ------------------------------------------------------------------------------ the weighted utility
def test_utility_is_the_weighted_sum_of_documented_terms_worked_by_hand():
    cfg = PrioritizerConfig()
    d = WeightedUtilityPrioritizer(cfg).select(scene([V('x', 2.0, 0.0)]), ctx())
    s = d.selected
    f, w = s.factors, cfg.weights
    d_fire = 4.0                                                                  # fire at (6, 0)
    assert f['distance_to_fire_m'] == pytest.approx(d_fire)
    assert f['fire_risk'] == pytest.approx(math.exp(-(d_fire - 1.0) / 1.0))
    assert f['fire_proximity'] == pytest.approx(1 - d_fire / 7.07)
    assert f['robot_path_m'] == pytest.approx(1.6) and f['return_path_m'] == pytest.approx(1.6)     # approach 0.4 m short
    expected = (w['risk'] * f['fire_risk'] + w['fire_proximity'] * f['fire_proximity']
                + w['reach'] * (1 - 1.6 / 7.07) + w['accessibility'] * 1.0 + w['confidence'] * 0.9
                - w['cost'] * (3.2 / (2 * 7.07)))
    assert s.utility == pytest.approx(expected, abs=1e-9)


def test_the_contributions_sum_to_the_utility_and_are_signed_by_meaning():
    d = WeightedUtilityPrioritizer().select(scene([V('a', 2, 1), V('b', 4, -2, conf=0.4)]), ctx())
    for s in d.ranked:
        assert sum(s.contributions.values()) == pytest.approx(s.utility, abs=1e-9)
        assert set(s.contributions) == set(WEIGHT_NAMES)
        assert s.contributions['cost'] <= 0 and all(s.contributions[k] >= 0 for k in WEIGHT_NAMES if k != 'cost')


def test_a_distant_endangered_victim_beats_a_near_safe_one_and_the_nearest_baseline_disagrees():
    """The plan's constructed scene: nearest-but-safe vs distant-but-endangered. The two models provably differ."""
    w = scene([V('near-safe', 2.0, 0.0), V('far-endangered', 5.0, 0.0)])          # fire at (6, 0): the far one is 1 m from it
    assert WeightedUtilityPrioritizer().select(w, ctx()).selected.victim_id == 'far-endangered'
    assert NearestVictimPrioritizer().select(w, ctx()).selected.victim_id == 'near-safe'


def test_fire_risk_weight_decides_it_zero_makes_the_near_victim_win():
    w = scene([V('near-safe', 2.0, 0.0), V('far-endangered', 5.0, 0.0)])
    no_risk = PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'risk': 0.0, 'fire_proximity': 0.0})
    assert WeightedUtilityPrioritizer(no_risk).select(w, ctx()).selected.victim_id == 'near-safe'
    assert WeightedUtilityPrioritizer().select(w, ctx()).selected.victim_id == 'far-endangered'


def test_sign_discipline_nearer_the_fire_raises_priority_and_farther_from_the_robot_lowers_it():
    """With the fire far from both, the only pull is distance from the robot, so the nearer victim wins; putting the
    fire beside the farther victim reverses it. The two terms must pull in opposite directions, as documented."""
    victims = [V('a', 1.0, 0.0), V('b', 3.0, 0.0)]
    far_fire = scene(victims, fire=(-6.0, 0.0))
    assert WeightedUtilityPrioritizer().select(far_fire, ctx()).selected.victim_id == 'a'
    hot_b = scene(victims, fire=(3.5, 0.0))
    assert WeightedUtilityPrioritizer().select(hot_b, ctx()).selected.victim_id == 'b'
    s = {x.victim_id: x for x in WeightedUtilityPrioritizer().score(hot_b, ctx())}
    assert s['b'].contributions['risk'] > s['a'].contributions['risk']            # closer to the fire: more urgent
    assert s['b'].contributions['reach'] < s['a'].contributions['reach']          # farther from the robot: costlier


def test_a_victim_behind_an_obstacle_is_less_accessible_and_scores_lower_than_an_identical_one_in_the_open():
    def detour(a, b):                                                            # anything ending on the y>0 side needs 60 % more path
        return 1.6 if b[1] > 0.5 else 1.0
    w = scene([V('open', 2.0, -2.0), V('behind', 2.0, 2.0)], fire=None)
    s = {x.victim_id: x for x in WeightedUtilityPrioritizer().score(w, ctx(detour=detour))}
    assert s['open'].factors['accessibility'] == pytest.approx(1.0)
    assert s['behind'].factors['accessibility'] == pytest.approx(1 / 1.6, rel=1e-6)
    assert s['behind'].utility < s['open'].utility
    assert WeightedUtilityPrioritizer().select(w, ctx(detour=detour)).selected.victim_id == 'open'


def test_a_more_confident_victim_scores_higher_all_else_equal_and_the_confidence_weight_controls_it():
    w = scene([V('sure', 2.0, 1.0, conf=0.95), V('doubtful', 2.0, -1.0, conf=0.3)], fire=None)
    d = WeightedUtilityPrioritizer().select(w, ctx())
    assert d.selected.victim_id == 'sure'
    gap = d.ranked[0].contributions['confidence'] - d.ranked[1].contributions['confidence']
    assert gap == pytest.approx(DEFAULT_WEIGHTS['confidence'] * 0.65)
    # with the confidence weight at zero the two are identical apart from the tie-break, so the decision is by id, not by luck
    tied = WeightedUtilityPrioritizer(PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'confidence': 0.0})).select(w, ctx())
    assert tied.ranked[0].utility == pytest.approx(tied.ranked[1].utility)


def test_a_cheaper_rescue_wins_when_cost_is_the_only_thing_that_counts():
    only_cost = PrioritizerConfig(weights={k: (1.0 if k == 'cost' else 0.0) for k in WEIGHT_NAMES})
    w = scene([V('close-to-home', 1.0, 0.0), V('far-from-home', 4.0, 3.0)], fire=(0.0, 5.0))
    assert WeightedUtilityPrioritizer(only_cost).select(w, ctx()).selected.victim_id == 'close-to-home'


def test_distance_matters_when_reach_dominates_and_a_path_longer_than_the_extent_is_not_negative():
    only_reach = PrioritizerConfig(weights={k: (1.0 if k == 'reach' else 0.0) for k in WEIGHT_NAMES})
    w = scene([V('a', 1.0, 0.0), V('b', 1.0, 3.0)], fire=None)
    assert WeightedUtilityPrioritizer(only_reach).select(w, ctx()).selected.victim_id == 'a'
    huge = scene([V('a', 500.0, 0.0)], fire=None)
    assert WeightedUtilityPrioritizer().select(huge, ctx()).selected.factors['robot_proximity'] == 0.0


def test_scaling_every_weight_leaves_the_decision_unchanged():
    w = scene([V('a', 2.0, 0.0), V('b', 5.0, 0.0), V('c', 3.0, 3.0, conf=0.5)])
    base = WeightedUtilityPrioritizer().select(w, ctx())
    scaled = WeightedUtilityPrioritizer(PrioritizerConfig(weights={k: 7.0 * v for k, v in DEFAULT_WEIGHTS.items()})).select(w, ctx())
    assert [s.victim_id for s in base.ranked] == [s.victim_id for s in scaled.ranked]


# ------------------------------------------------------------------------------ no fixed ids, no fixed order
def test_the_decision_does_not_depend_on_victim_ids_or_their_order():
    positions = [(2.0, 0.0), (5.0, 0.0), (3.0, 3.0), (-2.0, 1.0)]
    reference = None
    rng = random.Random(7)
    for trial in range(12):
        ids = [f'{rng.choice("abcxyz")}{rng.randrange(1000)}-{i}' for i in range(4)]
        order = list(range(4))
        rng.shuffle(order)
        w = scene([V(ids[i], *positions[i]) for i in order])
        chosen = WeightedUtilityPrioritizer().select(w, ctx()).selected
        pos = chosen.position
        assert reference is None or pos == reference, 'the same world must give the same choice under any labelling'
        reference = pos


def test_rescue_order_is_recomputed_from_the_world_each_time_not_stored():
    p = WeightedUtilityPrioritizer()
    victims = [V('a', 2.0, 0.0), V('b', 5.0, 0.0), V('c', 3.0, 3.0)]
    order = []
    for _ in range(3):
        d = p.select(scene(victims), ctx())
        order.append(d.selected.victim_id)
        victims = [v if v.id != d.selected.victim_id else VictimView(v.id, v.x, v.y, v.confidence, RESCUED) for v in victims]
    assert sorted(order) == ['a', 'b', 'c'] and len(set(order)) == 3
    assert p.select(scene(victims), ctx()).selected is None


# ------------------------------------------------------------------------------ eligibility
@pytest.mark.parametrize('status', [RESCUED, CARRIED, UNKNOWN, UNREACHABLE])
@pytest.mark.parametrize('model', ['weighted_utility', 'nearest'])
def test_only_detected_or_targeted_victims_can_be_selected_and_the_others_say_why(model, status):
    w = scene([V('other', 5.0, 0.0), V('closer-but-ineligible', 1.0, 0.0, status=status)])
    d = make_prioritizer(model).select(w, ctx())
    assert d.selected.victim_id == 'other'
    ex = [s for s in d.ranked if not s.eligible]
    assert len(ex) == 1 and ex[0].victim_id == 'closer-but-ineligible' and ex[0].utility is None and ex[0].exclusion


def test_the_current_target_stays_a_candidate_so_a_redecision_can_confirm_it():
    w = scene([V('current', 2.0, 0.0, status=TARGETED), V('other', 5.0, 3.0)], fire=None)
    d = WeightedUtilityPrioritizer().select(w, ctx())
    assert d.selected.victim_id == 'current' and all(s.eligible for s in d.ranked)
    beside_fire = scene([V('current', 2.0, 0.0, status=TARGETED), V('other', 5.0, 0.0)])          # a re-decision may switch
    assert WeightedUtilityPrioritizer().select(beside_fire, ctx()).selected.victim_id == 'other'


def test_an_unreachable_candidate_is_excluded_explicitly_even_if_it_would_have_won():
    def blocked(a, b):
        return b[0] > 3.5                                                        # the far victim's approach is walled off
    w = scene([V('near-safe', 2.0, 0.0), V('far-endangered', 5.0, 0.0)])
    d = WeightedUtilityPrioritizer().select(w, ctx(blocked=blocked))
    assert d.selected.victim_id == 'near-safe'
    ex = {s.victim_id: s for s in d.ranked if not s.eligible}
    assert 'far-endangered' in ex and 'no path' in ex['far-endangered'].exclusion and ex['far-endangered'].utility is None
    assert d.ranked[-1].victim_id == 'far-endangered', 'excluded candidates are listed after the eligible ones'


def test_an_exclusion_records_whether_the_planner_said_no_or_could_not_be_asked():
    """The node retries only the second kind: 'unknown' is transient (Nav2 starting), 'unreachable' is a verdict."""
    no_path = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)]), ctx(blocked=lambda a, b: True)).ranked[0]
    assert no_path.path_status == 'unreachable'
    silent = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)]), PlanningContext(lambda a, b: PathInfo.unknown('x'))).ranked[0]
    assert silent.path_status == 'unknown'
    wrong_status = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 1.0, status=RESCUED)]), ctx()).ranked[0]
    assert wrong_status.path_status == '' and not wrong_status.eligible                    # excluded for its status, not the planner
    assert json.loads(json.dumps(WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)]), ctx()).to_dict()))['candidates'][0]['path_status'] == ''


def test_when_nothing_is_reachable_nothing_is_selected_and_the_reason_says_so():
    d = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0), V('b', 3.0, 1.0)]), ctx(blocked=lambda a, b: True))
    assert d.selected is None and 'no path' in d.reason and 'a:' in d.reason and 'b:' in d.reason


def test_no_way_back_to_the_safe_zone_makes_a_victim_ineligible():
    d = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)]), ctx(blocked=lambda a, b: b == (0.0, 0.0)))
    assert d.selected is None and 'back to the safe zone' in d.ranked[0].exclusion


@pytest.mark.parametrize('failure', ['unknown', 'raises', 'garbage'])
def test_a_planner_that_cannot_answer_is_not_treated_as_reachable(failure):
    def q(a, b):
        if failure == 'raises':
            raise RuntimeError('server gone')
        return PathInfo.unknown('timeout') if failure == 'unknown' else 'nonsense'
    d = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)]), PlanningContext(q))
    assert d.selected is None and 'not assumed reachable' in d.ranked[0].exclusion


def test_no_victims_and_all_rescued_give_no_selection_with_a_reason():
    assert WeightedUtilityPrioritizer().select(scene([]), ctx()).reason == 'no victims are known'
    d = WeightedUtilityPrioritizer().select(scene([V('a', 1, 0, status=RESCUED)]), ctx())
    assert d.selected is None and 'already rescued' in d.reason


def test_an_unknown_fire_zeroes_the_fire_terms_and_says_so():
    d = WeightedUtilityPrioritizer().select(scene([V('a', 2.0, 0.0)], fire=None), ctx())
    s = d.selected
    assert s.contributions['risk'] == 0.0 and s.contributions['fire_proximity'] == 0.0
    assert s.factors['distance_to_fire_m'] == -1.0 and 'fire position unknown' in s.rationale


# ------------------------------------------------------------------------------ the world changing changes the decision
def test_changing_the_world_state_changes_the_decision():
    p = WeightedUtilityPrioritizer()
    base = [V('a', 2.0, 0.0), V('b', 5.0, 0.0)]
    assert p.select(scene(base), ctx()).selected.victim_id == 'b'                       # b is beside the fire
    assert p.select(scene(base, fire=(-4.0, 0.0)), ctx()).selected.victim_id == 'a'     # the fire moved away from b
    assert p.select(scene([base[0], V('b', 5.0, 0.0, status=RESCUED)]), ctx()).selected.victim_id == 'a'
    assert p.select(scene(base, robot=(6.0, 0.0)), ctx()).selected.victim_id == 'b'     # robot now stands by b
    assert p.select(scene([base[0]]), ctx()).selected.victim_id == 'a'
    assert p.select(scene([base[0], V('c', 5.0, 0.0)]), ctx()).selected.victim_id == 'c'   # a newly found victim beside the fire
    assert p.select(scene([V('a', 2.0, 0.0), V('b', 5.0, 0.0, conf=0.01)]), ctx()).selected.victim_id == 'b'   # still risk-led
    low = PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'risk': 0.5})
    assert WeightedUtilityPrioritizer(low).select(scene([V('a', 2.0, 0.0), V('b', 5.0, 0.0, conf=0.01)]), ctx()).selected.victim_id == 'a'


def test_ties_are_broken_deterministically_by_cost_then_id():
    w = scene([V('b', 0.0, 2.0), V('a', 0.0, -2.0)], fire=None)
    d = WeightedUtilityPrioritizer().select(w, ctx())
    assert d.ranked[0].utility == pytest.approx(d.ranked[1].utility)
    assert d.selected.victim_id == 'a'
    assert WeightedUtilityPrioritizer().select(scene(list(reversed(w.victims)), fire=None), ctx()).selected.victim_id == 'a'


# ------------------------------------------------------------------------------ the explanation
def test_the_decision_explains_itself_completely_and_is_json_serialisable():
    w = scene([V('a', 2.0, 0.0), V('b', 5.0, 0.0), V('gone', 1.0, 1.0, status=RESCUED)])
    d = WeightedUtilityPrioritizer().select(w, ctx())
    j = json.loads(json.dumps(d.to_dict()))
    assert j['model'] == 'weighted_utility' and j['selected'] == d.selected.victim_id and set(j['weights']) == set(WEIGHT_NAMES)
    assert [c['victim_id'] for c in j['candidates']] == [s.victim_id for s in d.ranked]
    for c in j['candidates']:
        if c['eligible']:
            assert sum(c['contributions'].values()) == pytest.approx(c['utility'])
            assert c['rationale'].startswith(c['victim_id']) and 'U=' in c['rationale']
            assert {'fire_risk', 'robot_path_m', 'rescue_cost_m', 'accessibility', 'confidence'} <= set(c['factors'])
        else:
            assert c['exclusion'] and c['utility'] is None


# ------------------------------------------------------------------------------ the nearest baseline, on its own
def test_nearest_selects_by_planned_path_not_by_straight_line():
    def detour(a, b):
        return 3.0 if b[1] < -0.5 else 1.0                                       # the straight-line-nearest victim is walled in
    w = scene([V('straight-nearest', 1.0, -2.0), V('path-nearest', 3.0, 1.0)], fire=None)
    d = NearestVictimPrioritizer().select(w, ctx(detour=detour))
    assert d.selected.victim_id == 'path-nearest'


def test_nearest_ignores_fire_confidence_and_the_return_leg_but_reports_them():
    a = scene([V('close-doomed', 2.0, 0.0, conf=0.2), V('far-sure', 4.0, 0.0, conf=0.99)], fire=(2.2, 0.0))
    d = NearestVictimPrioritizer().select(a, ctx())
    assert d.selected.victim_id == 'close-doomed'
    assert d.selected.factors['fire_risk'] == 1.0 and d.selected.factors['confidence'] == 0.2       # reported ...
    assert d.weights == {k: (1.0 if k == 'reach' else 0.0) for k in WEIGHT_NAMES}                   # ... but weightless
    assert NearestVictimPrioritizer().select(scene([V('x', 2.0, 0.0)], fire=None), ctx()).selected.rationale.startswith('x:')


def test_nearest_ties_break_by_id_and_utility_is_minus_the_distance():
    d = NearestVictimPrioritizer().select(scene([V('b', 0.0, 2.0), V('a', 0.0, -2.0)], fire=None), ctx())
    assert d.selected.victim_id == 'a' and d.selected.utility == pytest.approx(-1.6)
    assert sum(d.selected.contributions.values()) == pytest.approx(d.selected.utility)


def test_nearest_never_selects_what_the_weighted_model_may_not():
    blocked = lambda a, b: b[0] > 3.5  # noqa: E731
    w = scene([V('reachable', 6.0 - 4.0, 0.0), V('walled', 5.0, 0.0), V('rescued', 0.5, 0.0, status=RESCUED)])
    for model in ('weighted_utility', 'nearest'):
        d = make_prioritizer(model).select(w, ctx(blocked=blocked))
        assert d.selected.victim_id == 'reachable'
        assert {s.victim_id for s in d.ranked if not s.eligible} == {'walled', 'rescued'}


# ------------------------------------------------------------------------------ factory and interface agreement
def test_the_factory_builds_the_named_model_and_rejects_unknown_ones():
    assert isinstance(make_prioritizer('weighted_utility'), WeightedUtilityPrioritizer)
    assert isinstance(make_prioritizer('nearest'), NearestVictimPrioritizer)
    with pytest.raises(ValueError, match='decision_model'):
        make_prioritizer('lucky_dip')


def test_status_constants_match_the_victim_state_message():
    text = (REPO / 'src/fire_resq_interfaces/msg/VictimState.msg').read_text()
    msg = {name: int(val) for name, val in re.findall(r'^uint8 (\w+)=(\d+)', text, re.M)}
    assert msg == {name: code for code, name in STATUS_NAMES.items()}


def test_prioritizers_are_interchangeable_behind_one_interface():
    w = scene([V('a', 2.0, 0.0), V('b', 5.0, 0.0)])
    for model in ('weighted_utility', 'nearest'):
        p = make_prioritizer(model)
        assert [type(s).__name__ for s in p.score(w, ctx())] == ['ScoredVictim'] * 2
        assert p.select(w, ctx()).model == model
