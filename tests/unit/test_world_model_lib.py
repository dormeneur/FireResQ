"""The world model as a pure library: creation, association, identity, confidence, lifecycle, safe zone, markers.

No ROS, no simulator. Time is explicit, so every rule is deterministic.
"""
import math
import re
from pathlib import Path

import pytest
from fire_resq_world_model import entities as E
from fire_resq_world_model.config import SafeZone, WorldModelConfig
from fire_resq_world_model.entities import Observation
from fire_resq_world_model.markers import STATUS_RGB, marker_specs
from fire_resq_world_model.world_model import WorldModel

REPO = Path(__file__).resolve().parents[2]


def V(x, y, c=0.9, z=0.13):
    return Observation('victim', x, y, z, c)


def F(x, y, c=0.9):
    return Observation('fire', x, y, 0.05, c)


def model(**kw):
    kw.setdefault('safe_zone', SafeZone(-3.0, -3.0, 0.0, 1.0, 1.0))     # out of the way unless a test wants it
    return WorldModel(WorldModelConfig(**kw))


def confirmed(m, *pts, t=0.0):
    """Observe each point twice (confirm_hits default 2)."""
    for k in range(2):
        m.observe([V(*p) for p in pts], t + 0.1 * k)
    return m


# ------------------------------------------------------------------------------ creation
def test_a_new_victim_becomes_a_tentative_track_then_is_confirmed_by_a_second_observation():
    m = model()
    r = m.observe([V(1.0, 1.0)], 0.0)
    assert r.created == ['V1']
    assert m.snapshot(0.0).victims[0].status == E.UNKNOWN
    r = m.observe([V(1.0, 1.0)], 0.1)
    assert r.confirmed == ['V1'] and r.created == []
    assert m.snapshot(0.1).victims[0].status == E.DETECTED


def test_confirm_hits_one_confirms_immediately():
    m = model(confirm_hits=1)
    m.observe([V(1, 1)], 0.0)
    assert m.snapshot(0.0).victims[0].status == E.DETECTED


def test_several_blobs_in_one_message_are_one_hit_not_several():
    """A victim split by an occluder gives two blobs in one message: that is one observation of one victim."""
    m = model()
    m.observe([V(1.0, 1.0), V(1.05, 1.0, c=0.5)], 0.0)
    v = m.snapshot(0.0).victims
    assert len(v) == 1 and v[0].hits == 1 and v[0].status == E.UNKNOWN


def test_a_tentative_ghost_is_dropped_but_a_confirmed_victim_never_is():
    m = model(tentative_timeout_s=5.0)
    m.observe([V(1, 1)], 0.0)                                         # ghost: seen once
    confirmed(m, (2.0, 2.0), t=0.0)                                   # real: seen twice
    ids = {v.id for v in m.snapshot(4.0).victims}
    assert ids == {'V1', 'V2'}
    assert {v.id for v in m.snapshot(6.0).victims} == {'V2'}          # the ghost has expired
    assert {v.id for v in m.snapshot(1e6).victims} == {'V2'}          # far in the future, still there


def test_ids_are_creation_order_and_never_reused():
    m = model(tentative_timeout_s=1.0)
    m.observe([V(1, 1)], 0.0)
    assert m.snapshot(5.0).victims == ()                              # V1 expired
    m.observe([V(2, 2)], 6.0)
    assert [v.id for v in m.snapshot(6.0).victims] == ['V2']


# ------------------------------------------------------------------------------ association / identity
def test_observations_within_the_gate_update_the_same_victim_and_beyond_it_create_another():
    m = confirmed(model(victim_gate_m=0.3), (1.0, 1.0))
    m.observe([V(1.2, 1.0)], 1.0)                                     # 0.2 m away: same victim
    assert [v.id for v in m.snapshot(1.0).victims] == ['V1']
    m.observe([V(1.6, 1.0)], 1.1)                                     # 0.4+ m from V1: a different victim
    assert [v.id for v in m.snapshot(1.1).victims] == ['V1', 'V2']


def test_the_scenario_victim_spacing_exceeds_the_association_gate():
    """The scenario validator promises victims are further apart than the gate; keep the two in step."""
    from fire_resq_simulation.scenario import VICTIM_SPACING
    assert WorldModelConfig().victim_gate_m < VICTIM_SPACING


def test_each_victim_keeps_its_identity_while_observed_with_noise():
    m = confirmed(model(), (0.0, 0.0), (1.0, 0.0), (0.0, 1.0))
    ids = {v.id: (v.x, v.y) for v in m.snapshot(0.1).victims}
    assert len(ids) == 3
    for k in range(50):                                               # wobble +-4 cm, alternating
        e = 0.04 if k % 2 else -0.04
        m.observe([V(0.0 + e, 0.0), V(1.0 - e, 0.0 + e), V(0.0 + e, 1.0 - e)], 1.0 + 0.1 * k)
    after = {v.id: (v.x, v.y) for v in m.snapshot(6.0).victims}
    assert set(after) == set(ids)
    for i, (x, y) in ids.items():
        assert math.hypot(after[i][0] - x, after[i][1] - y) < 0.05, i


def test_a_victim_that_leaves_view_and_returns_is_the_same_victim():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    first = {v.id: (v.x, v.y) for v in m.snapshot(0.1).victims}
    m.observe([V(2.0, 0.0)], 50.0)                                    # only one of them seen for a long while
    m.observe([V(1.02, 1.01)], 120.0)                                 # the other comes back
    now = {v.id: (v.x, v.y) for v in m.snapshot(120.0).victims}
    assert set(now) == set(first), 'a returning victim must not become a new entity'


def test_a_previously_unseen_victim_appearing_later_is_added_without_disturbing_the_others():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    before = {v.id: v.status for v in m.snapshot(0.1).victims}
    m.observe([V(-1.0, 1.4)], 30.0)
    m.observe([V(-1.0, 1.4)], 30.1)
    snap = m.snapshot(30.1)
    assert [v.id for v in snap.victims] == ['V1', 'V2', 'V3']
    assert snap.victims[2].status == E.DETECTED
    assert {v.id: v.status for v in snap.victims[:2]} == before


def test_two_detections_never_claim_two_tracks_for_one_victim_in_a_message():
    m = confirmed(model(), (1.0, 1.0))
    m.observe([V(1.02, 1.0), V(1.0, 1.03, c=0.6)], 1.0)
    assert len(m.snapshot(1.0).victims) == 1


# ------------------------------------------------------------------------------ smoothing
def test_position_is_an_ema_of_the_observations():
    m = model(position_alpha=0.5, confirm_hits=1)
    m.observe([V(0.0, 0.0)], 0.0)
    m.observe([V(0.2, 0.0)], 0.1)
    assert m.snapshot(0.1).victims[0].x == pytest.approx(0.1)
    for k in range(40):
        m.observe([V(0.2, 0.0)], 1.0 + k)
    assert m.snapshot(41.0).victims[0].x == pytest.approx(0.2, abs=1e-3)


def test_alpha_one_follows_the_latest_observation():
    m = model(position_alpha=1.0, confirm_hits=1)
    m.observe([V(0.0, 0.0)], 0.0)
    m.observe([V(0.15, -0.1)], 0.1)
    v = m.snapshot(0.1).victims[0]
    assert (v.x, v.y) == pytest.approx((0.15, -0.1))


# ------------------------------------------------------------------------------ confidence / validity
def test_low_confidence_and_nonfinite_detections_are_ignored():
    m = model(min_confidence=0.3)
    r = m.observe([V(1, 1, c=0.25), V(2, 2, c=float('nan')), V(float('inf'), 0)], 0.0)
    assert m.snapshot(0.0).victims == ()
    assert r.ignored == {'low confidence': 1, 'non-finite': 2}


def test_unhandled_classes_are_ignored_not_crashed_on():
    m = model()
    r = m.observe([Observation('obstacle', 1, 1, 0, 0.9)], 0.0)
    assert m.snapshot(0.0).victims == () and 'unhandled class' in list(r.ignored)[0]


def test_confidence_is_the_latest_and_decays_with_time_since_seen():
    m = confirmed(model(confidence_decay_tau_s=10.0), (1.0, 1.0))
    m.observe([V(1.0, 1.0, c=0.8)], 1.0)
    assert m.snapshot(1.0).victims[0].confidence == pytest.approx(0.8)
    assert m.snapshot(11.0).victims[0].confidence == pytest.approx(0.8 * math.exp(-1.0), rel=1e-6)
    m.observe([V(1.0, 1.0, c=0.6)], 11.0)                              # seen again: the latest, undecayed
    assert m.snapshot(11.0).victims[0].confidence == pytest.approx(0.6)
    assert m.snapshot(1e4).victims[0].confidence < 1e-6                # decayed to ~0 but the victim is not deleted
    assert len(m.snapshot(1e4).victims) == 1


# ------------------------------------------------------------------------------ fire
def test_fire_is_created_confirmed_smoothed_and_tracked_as_one_entity():
    m = model(position_alpha=0.5)
    assert not m.snapshot(0.0).fire.known
    m.observe([F(3.0, 0.0)], 0.0)
    assert not m.snapshot(0.0).fire.known                             # tentative
    m.observe([F(3.2, 0.0)], 0.1)
    f = m.snapshot(0.1).fire
    assert f.known and f.x == pytest.approx(3.1)


def test_a_far_second_fire_is_ignored_and_reported():
    m = model(fire_gate_m=0.5, confirm_hits=1)
    m.observe([F(3.0, 0.0)], 0.0)
    r = m.observe([F(0.0, 0.0)], 0.1)
    assert m.snapshot(0.1).fire.x == pytest.approx(3.0) and 'second fire (one fire is tracked)' in r.ignored


def test_an_unconfirmed_fire_ghost_expires():
    m = model(tentative_timeout_s=2.0)
    m.observe([F(3.0, 0.0)], 0.0)
    assert not m.snapshot(3.0).fire.known and m.snapshot(3.0).fire.confidence == 0.0


# ------------------------------------------------------------------------------ status lifecycle
def test_the_normal_rescue_sequence_and_the_current_target():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    assert m.current_target_id() == ''
    for status in (E.TARGETED, E.CARRIED):
        assert m.set_status('V1', status)[0]
        assert m.current_target_id() == 'V1'
    assert m.set_status('V1', E.RESCUED)[0]
    assert m.current_target_id() == ''
    assert m.snapshot(1.0).victims[0].status == E.RESCUED


def test_rescued_is_terminal():
    m = confirmed(model(), (1.0, 1.0))
    for s in (E.TARGETED, E.CARRIED, E.RESCUED):
        m.set_status('V1', s)
    for s in (E.DETECTED, E.TARGETED, E.CARRIED, E.UNREACHABLE, E.UNKNOWN):
        ok, msg = m.set_status('V1', s)
        assert not ok and 'terminal' in msg
    assert m.set_status('V1', E.RESCUED)[0]                            # idempotent, still fine


def test_illegal_transitions_and_unknown_ids_are_refused_with_a_reason():
    m = confirmed(model(), (1.0, 1.0))
    assert not m.set_status('V1', E.CARRIED)[0]                        # cannot carry what was never targeted
    assert not m.set_status('V1', E.RESCUED)[0]
    assert not m.set_status('V1', E.UNKNOWN)[0]
    assert not m.set_status('V1', 42)[0]
    ok, msg = m.set_status('V9', E.TARGETED)
    assert not ok and 'unknown victim' in msg


def test_a_tentative_track_cannot_be_targeted():
    m = model()
    m.observe([V(1, 1)], 0.0)
    assert not m.set_status('V1', E.TARGETED)[0]


def test_one_active_target_at_a_time():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    assert m.set_status('V1', E.TARGETED)[0]
    ok, msg = m.set_status('V2', E.TARGETED)
    assert not ok and 'one target at a time' in msg
    assert m.set_status('V1', E.DETECTED)[0]                           # abandoned: V1 is free again
    assert m.set_status('V2', E.TARGETED)[0]


def test_unreachable_can_be_retried_explicitly():
    m = confirmed(model(), (1.0, 1.0))
    assert m.set_status('V1', E.UNREACHABLE)[0]
    assert m.set_status('V1', E.DETECTED)[0]


def test_a_rescued_victim_stays_rescued_and_cannot_be_resurrected_by_seeing_it_again():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    for s in (E.TARGETED, E.CARRIED, E.RESCUED):
        m.set_status('V1', s)
    for k in range(20):
        m.observe([V(1.0, 1.0)], 5.0 + k)                              # still seen where it was (or a look-alike)
    snap = m.snapshot(30.0)
    assert [v.status for v in snap.victims] == [E.RESCUED, E.DETECTED]
    assert len(snap.victims) == 2, 'seeing a rescued victim must not create a new one'


def test_a_carried_victims_track_is_frozen_and_absorbs_detections():
    m = confirmed(model(), (1.0, 1.0))
    m.set_status('V1', E.TARGETED)
    m.set_status('V1', E.CARRIED)
    m.observe([V(1.2, 1.0)], 5.0)                                      # a moving blob near the old spot
    v = m.snapshot(5.0).victims
    assert len(v) == 1 and (v[0].x, v[0].y) == pytest.approx((1.0, 1.0))     # TODO(phase-9): should follow the robot


# ------------------------------------------------------------------------------ safe zone
def test_a_victim_detected_inside_the_safe_zone_never_creates_a_new_victim():
    """A released victim is seen in the safe zone: it must not appear as an extra candidate."""
    m = model(safe_zone=SafeZone(0.0, 0.0, 0.0, 1.0, 1.0))
    confirmed(m, (2.0, 2.0))
    r = m.observe([V(0.1, -0.2)], 1.0)
    r = m.observe([V(0.1, -0.2)], 1.1)
    assert len(m.snapshot(1.1).victims) == 1 and 'victim inside the safe zone' in r.ignored


def test_the_safe_zone_respects_its_rotation():
    z = SafeZone(0.0, 0.0, math.pi / 4, 1.0, 0.2)
    assert z.contains(0.3, 0.3) and not z.contains(0.3, -0.3)


def test_the_safe_zone_is_reported_in_the_snapshot():
    m = model(safe_zone=SafeZone(1.0, 2.0, 0.5, 1.0, 1.0))
    z = m.snapshot(0.0).safe_zone
    assert (z.x, z.y, z.yaw) == (1.0, 2.0, 0.5)


# ------------------------------------------------------------------------------ config
def test_config_rejects_bad_values_and_unknown_keys():
    for bad in ({'victim_gate_m': 0.0}, {'position_alpha': 0.0}, {'position_alpha': 1.5}, {'confirm_hits': 0},
                {'tentative_timeout_s': -1.0}, {'min_confidence': 2.0}):
        with pytest.raises(ValueError):
            WorldModelConfig(**bad)
    with pytest.raises(ValueError, match='unknown'):
        WorldModelConfig.from_dict({'victim_gate': 0.3})
    with pytest.raises(ValueError, match='safe_zone'):
        WorldModelConfig.from_dict({'safe_zone.wdith': 1.0})


def test_config_from_flat_and_nested_parameters_agree():
    a = WorldModelConfig.from_dict({'confirm_hits': 3, 'safe_zone.x': 1.0, 'safe_zone.size_x': 2.0})
    b = WorldModelConfig.from_dict({'confirm_hits': 3, 'safe_zone': {'x': 1.0, 'size_x': 2.0}})
    assert a == b and a.confirm_hits == 3 and a.safe_zone.size_x == 2.0


# ------------------------------------------------------------------------------ interface agreement
def test_status_constants_match_the_victim_state_message():
    text = (REPO / 'src/fire_resq_interfaces/msg/VictimState.msg').read_text()
    msg = {name: int(val) for name, val in re.findall(r'^uint8 (\w+)=(\d+)', text, re.M)}
    assert msg == {name: code for code, name in E.STATUS_NAMES.items()}


def test_every_status_has_a_transition_entry_and_only_to_real_statuses():
    assert set(E.TRANSITIONS) == set(E.STATUS_NAMES)
    assert all(t in E.STATUS_NAMES for ts in E.TRANSITIONS.values() for t in ts)


# ------------------------------------------------------------------------------ markers
def test_markers_colour_victims_by_status_and_label_them():
    m = confirmed(model(), (1.0, 1.0), (2.0, 0.0))
    m.set_status('V1', E.TARGETED)
    specs = marker_specs(m.snapshot(0.1))
    spheres = {s.id: s for s in specs if s.ns == 'victim'}
    assert spheres[0].rgba[:3] == STATUS_RGB[E.TARGETED] and spheres[1].rgba[:3] == STATUS_RGB[E.DETECTED]
    labels = [s.text for s in specs if s.ns == 'victim_label']
    assert labels[0].startswith('V1 TARGETED') and labels[1].startswith('V2 DETECTED')
    assert any(s.ns == 'safe_zone' for s in specs)


def test_the_fire_marker_appears_only_once_the_fire_is_known():
    m = model()
    assert not any(s.ns == 'fire' for s in marker_specs(m.snapshot(0.0)))
    m.observe([F(3, 0)], 0.0)
    m.observe([F(3, 0)], 0.1)
    fire = [s for s in marker_specs(m.snapshot(0.1)) if s.ns == 'fire']
    assert len(fire) == 1 and (fire[0].x, fire[0].y) == pytest.approx((3.0, 0.0))
