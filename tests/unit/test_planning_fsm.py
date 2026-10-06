"""The rescue FSM against a closed-loop stand-in world (tests/unit/fsm_harness.py): no ROS, no simulator.

These pin the PROCEDURE: the states and their order, the failure handling, the safety rules. That the procedure works on
real physics is tests/sim/test_rescue_loop.py's job.
"""
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tests' / 'unit'))
for pkg in ('fire_resq_planning', 'fire_resq_world_model', 'fire_resq_description'):
    sys.path.insert(0, str(REPO / 'src' / pkg))

from fsm_harness import SimEnv, arena_grid  # noqa: E402

from fire_resq_planning.config import PlanningConfig  # noqa: E402
from fire_resq_planning.fsm import ALLOWED, IllegalTransition, RescueFSM  # noqa: E402
from fire_resq_planning.geometry import Pose2, Zone, dist  # noqa: E402
from fire_resq_planning.types import (DETECTED, RESCUED, TARGETED, UNREACHABLE, Drive, Observation, State)  # noqa: E402

ZONE = Zone(0.8, 0.8, 0.0, 1.0, 1.0)
START = Pose2(0.8, 0.8, 0.7)
CFG = PlanningConfig()
HAPPY_ORDER = ['INIT', 'PERCEIVE', 'UPDATE_WORLD', 'SELECT_TARGET', 'NAVIGATE_TO_APPROACH', 'ALIGN', 'ATTACH', 'VERIFY_CARRY',
               'NAVIGATE_TO_SAFE_ZONE', 'RELEASE', 'VERIFY_RELEASE', 'REASSESS']


def env_with(victims, **kw):
    return SimEnv(CFG, victims, ZONE, pose=START, **kw), RescueFSM(CFG)


def path_of(fsm):
    return [fsm.history[0].frm] + [t.to for t in fsm.history]


def test_one_visible_victim_is_rescued_and_the_mission_then_completes():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.run(fsm, 1500)
    assert fsm.state == State.MISSION_COMPLETE, (fsm.state, fsm.reason, fsm.history[-3:])
    assert fsm.m.rescued == ['V1'] and not fsm.m.given_up
    assert path_of(fsm)[:len(HAPPY_ORDER)] == HAPPY_ORDER
    tv = env.tv['a']
    assert ZONE.contains((tv.x, tv.y), margin=0.05), 'the victim is not inside the safe zone'
    assert not tv.held and env.belief['V1'].status == RESCUED
    assert not env.energized, 'the coil was left energised'
    for t in fsm.history:                                    # every transition was declared
        assert State(t.to) in ALLOWED[State(t.frm)] or t.to == 'MISSION_ABORTED'


def test_the_fsm_is_deterministic():
    runs = []
    for _ in range(2):
        env, fsm = env_with({'a': (2.6, 1.4)}, seed=5)
        seq = []
        env.run(fsm, 600, on_step=lambda o, fx: seq.append(tuple(type(e).__name__ for e in fx)))
        runs.append((seq, [(t.t, t.frm, t.to, t.reason) for t in fsm.history]))
    assert runs[0] == runs[1]


# --------------------------------------------------------------------------------------------- scenario helpers
def grid_with_pocket():
    """The test arena with a wall that hides the north-west from everywhere except the east end."""
    return arena_grid(block=True)


def run_mission(victims, seconds=3000, **kw):
    env, fsm = env_with(victims, **kw)
    env.run(fsm, seconds)
    return env, fsm


def drives(log):
    return [e for o, fx in log for e in fx if isinstance(e, Drive)]


def collect(env, fsm, seconds, **kw):
    log = []
    env.run(fsm, seconds, on_step=lambda o, fx: log.append((o, fx)), **kw)
    return log


# ------------------------------------------------------------------------------------ selection and ranking
def test_the_fsm_follows_whatever_order_cognition_gives_it_and_has_none_of_its_own():
    victims = {'near': (2.2, 1.0), 'far': (4.2, 1.4)}
    nearest_first, _ = run_mission(victims)
    farthest_first, fsm = run_mission(victims, rank=lambda pos, robot: sorted(pos, key=lambda k: -dist(pos[k], robot.xy)))

    def order(env, fsm):
        names = {v: k for k, v in env.ids.items()}
        return [names[t.target] for t in fsm.history if t.frm == 'SELECT_TARGET' and t.to == 'NAVIGATE_TO_APPROACH']
    e1, f1 = run_mission(victims)
    assert order(e1, f1)[0] == 'near'
    assert order(farthest_first, fsm)[0] == 'far', 'the FSM must rescue in the order cognition chose, not its own'
    assert f1.state == fsm.state == State.MISSION_COMPLETE


def test_a_rescued_victim_is_never_selected_again():
    env, fsm = run_mission({'a': (2.2, 1.0), 'b': (4.2, 1.4)})
    targets = [t.target for t in fsm.history if t.frm == 'SELECT_TARGET' and t.to == 'NAVIGATE_TO_APPROACH']
    assert sorted(targets) == ['V1', 'V2'] and len(set(targets)) == 2


def test_a_hidden_victim_is_found_by_searching_not_known_in_advance():
    """Victim c sits behind a long wall: no line of sight to it from where the robot starts, from the other victims, or from the
    safe zone. Only a SEARCH viewpoint at the far east end looks along the wall's north side."""
    grid = arena_grid(block=False)
    r0, r1, c1 = grid.cell(0.1, 2.45)[0], grid.cell(0.1, 2.55)[0], grid.cell(3.6, 0)[1]
    grid.data[r0:r1 + 1, 3:c1] = 100
    victims = {'a': (2.6, 1.4), 'b': (4.2, 1.0), 'c': (1.2, 3.8)}
    env = SimEnv(CFG, victims, ZONE, pose=START, grid=grid)
    fsm = RescueFSM(CFG)
    assert not env._visible('c')
    env.run(fsm, 4000)
    assert fsm.state == State.MISSION_COMPLETE, (fsm.state, fsm.reason)
    assert sorted(fsm.m.rescued) == ['V1', 'V2', 'V3'], fsm.m.rescued
    assert any(t.frm == 'SEARCH' and t.to == 'PERCEIVE' for t in fsm.history), 'c can only have been found by searching'
    v3_selected = next(t for t in fsm.history if t.target == 'V3')
    assert any(t.frm == 'SEARCH' and t.t < v3_selected.t for t in fsm.history)


def test_no_target_alone_does_not_end_the_mission_without_search_evidence():
    env, fsm = env_with({})
    env.run(fsm, 1200)
    assert fsm.state == State.MISSION_COMPLETE
    assert any(t.frm == 'SELECT_TARGET' and t.to == 'SEARCH' for t in fsm.history)
    assert fsm.planner.coverage() >= CFG.coverage_goal or 'exhausted' in fsm.reason
    assert 'search exhausted' in fsm.history[-1].reason


# ----------------------------------------------------------------------------------------- failure handling
def test_a_failed_navigation_is_not_a_failed_rescue_the_target_is_reconsidered():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.fail_nav = 1
    env.run(fsm, 2500)
    assert fsm.state == State.MISSION_COMPLETE and fsm.m.rescued == ['V1'] and not fsm.m.given_up
    kinds = [(t.frm, t.to) for t in fsm.history]
    assert ('NAVIGATE_TO_APPROACH', 'ABANDON') in kinds and ('ABANDON', 'REASSESS') in kinds
    assert fsm.m.attempts == {'V1': 1}


def test_a_victim_that_keeps_failing_is_given_up_as_unreachable_after_the_attempt_budget():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.fail_nav = 5
    env.run(fsm, 2500)
    assert fsm.state == State.MISSION_COMPLETE
    assert fsm.m.given_up == ['V1'] and fsm.m.rescued == []
    assert env.belief['V1'].status == UNREACHABLE


def test_a_refused_attach_never_marks_the_victim_rescued_and_leaves_the_coil_off():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.refuse_attach = True
    env.run(fsm, 2500)
    assert fsm.m.rescued == []
    assert env.belief['V1'].status in (DETECTED, UNREACHABLE)
    assert not env.energized, 'a refused coil must not stay live'
    assert fsm.metrics['attach_refusals'] >= 1
    assert any(t.frm == 'ATTACH' and t.to == 'ABANDON' for t in fsm.history)
    assert not env.tv['a'].held


def test_losing_the_target_during_align_stops_the_robot_and_reconsiders():
    env, fsm = env_with({'a': (2.6, 1.4), 'b': (4.2, 1.0)})
    env.vanish_on = (State.ALIGN, 'a')
    log = collect(env, fsm, 2500)
    t_align = next(t for t in fsm.history if t.to == 'ALIGN').t
    ab = next(t for t in fsm.history if t.frm == 'ALIGN' and t.to == 'ABANDON')
    assert 'not in view' in ab.reason or 'gone' in ab.reason or 'lost' in ab.reason, ab.reason
    # after the abandon no Drive was issued until the next state that legitimately drives
    after = [(o, fx) for o, fx in log if ab.t <= o.t < ab.t + 1.0]
    assert not any(isinstance(e, Drive) for o, fx in after for e in fx if o.t > ab.t)
    assert ab.t - t_align < CFG.align_lost_timeout_s + CFG.align_timeout_s / 2
    assert 'V1' in fsm.m.attempts


def test_a_victim_moved_away_behind_its_own_spot_is_not_mistaken_for_the_target_in_align():
    """Measured live: the target moved 0.8 m further off along the same bearing stayed "in view" by bearing alone, and ALIGN
    crept onto empty floor. A blob far narrower than a victim at the stored range is something else: the target is lost."""
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.move_on = (State.ALIGN, 'a', 0.8)
    env.run(fsm, 2500, until=lambda: any(t.to == 'ABANDON' for t in fsm.history))
    ab = next(t for t in fsm.history if t.to == 'ABANDON')
    assert ab.frm == 'ALIGN', ab                              # lost from view, or its track moved: either way, never ATTACH
    assert not any(t.to == 'ATTACH' for t in fsm.history)


def test_align_ignores_a_blob_at_the_right_bearing_that_is_too_narrow_for_a_victim_at_the_stored_range():
    from fire_resq_planning.types import Camera, Sighting
    fsm = RescueFSM(CFG)
    cam = Camera(fx=337.0, cx=320.0, hfov=CFG.camera_hfov_rad)
    wide = Sighting(u=320.0, width=337.0 * 0.10 / 0.40, stamp=10.0)          # a victim 0.4 m ahead
    narrow = Sighting(u=320.0, width=337.0 * 0.10 / 1.20, stamp=10.0)        # the same bearing, 1.2 m away
    assert fsm._best_sighting(Observation(t=10.0, camera=cam, sightings=(wide,)), 0.0, 0.40) is not None
    assert fsm._best_sighting(Observation(t=10.0, camera=cam, sightings=(narrow,)), 0.0, 0.40) is None


def test_an_undeclared_transition_is_a_programming_error():
    fsm = RescueFSM(CFG)
    with pytest.raises(IllegalTransition):
        fsm._go(State.ALIGN, 'skipping ahead')


# ------------------------------------------------------------------------------------------- carrying
def test_a_hold_lost_while_verifying_the_carry_is_detected_before_moving_off():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.drop_after = 1.0                                     # the victim falls off one second after the pick-up (once)
    env.run(fsm, 2500)
    ab = next(t for t in fsm.history if t.to == 'ABANDON')
    assert ab.frm == 'VERIFY_CARRY', ab
    assert 'hold was lost' in ab.reason
    first_release = next((t.t for t in fsm.history if t.to == 'RELEASE'), None)
    assert first_release is None or first_release > ab.t, 'the FSM released a victim it had already lost'
    assert fsm.m.attempts == {'V1': 1}                        # counted as a failed attempt; the retry (no drop this time) succeeds
    assert fsm.m.rescued == ['V1']


def test_a_victim_dropped_in_transit_stops_the_robot_and_is_not_reported_rescued():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.drop_after = 22.0
    collect(env, fsm, 2500)
    assert fsm.m.rescued == []
    ab = next(t for t in fsm.history if t.to == 'ABANDON')
    assert ab.frm == 'NAVIGATE_TO_SAFE_ZONE', ab
    assert 'hold was lost during transport' in ab.reason
    assert env.belief['V1'].status in (DETECTED, UNREACHABLE)
    assert not any(t.to == 'RELEASE' for t in fsm.history if t.t <= ab.t + 0.01 and t.frm == 'NAVIGATE_TO_SAFE_ZONE')


def test_release_only_when_the_carried_victim_would_land_inside_the_safe_zone():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.run(fsm, 1200, until=lambda: fsm.state == State.RELEASE)
    assert fsm.state == State.RELEASE
    assert ZONE.contains((env.tv['a'].x, env.tv['a'].y), margin=CFG.release_margin_m * 0.8), 'released outside the zone margin'


def test_the_release_is_verified_before_the_world_model_hears_rescued():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.run(fsm, 1500)
    i_release = next(i for i, t in enumerate(fsm.history) if t.frm == 'RELEASE')
    i_verify = next(i for i, t in enumerate(fsm.history) if t.frm == 'VERIFY_RELEASE')
    assert i_release < i_verify
    assert fsm.history[i_verify].t - fsm.history[i_release].t >= CFG.verify_release_s


# --------------------------------------------------------------------------------------------- safety
def test_a_magnet_that_contradicts_the_fsm_aborts_the_mission_and_locks_motion():
    env, fsm = env_with({'a': (2.6, 1.4)})
    env.run(fsm, 60, until=lambda: fsm.state == State.NAVIGATE_TO_APPROACH)
    env.holding, env.energized = 'a', True                       # the magnet suddenly claims a victim is held
    env.tv['a'].held = True
    log = collect(env, fsm, 30)
    assert fsm.state == State.MISSION_ABORTED and 'held' in fsm.reason
    assert not any(isinstance(e, Drive) for o, fx in log for e in fx), 'no motion command after a fatal inconsistency'


def test_the_fsm_never_drives_backwards_or_past_the_robots_limits():
    env, fsm = env_with({'a': (2.6, 1.4), 'b': (4.2, 1.0)})
    log = collect(env, fsm, 3000)
    ds = drives(log)
    assert ds, 'the FSM should have driven at least the sweep and the align'
    assert all(d.v >= 0.0 for d in ds), 'a Drive with negative linear velocity: the robot is blind behind itself'
    assert max(abs(d.v) for d in ds) <= CFG.align_speed_mps + 1e-9
    assert max(abs(d.w) for d in ds) <= CFG.max_angular_rps


def test_a_navigation_that_never_finishes_times_out_and_the_attempt_is_abandoned():
    cfg = PlanningConfig(nav_timeout_s=20.0)
    env = SimEnv(cfg, {'a': (2.6, 1.4)}, ZONE, pose=START, nav_speed=0.005)
    fsm = RescueFSM(cfg)
    env.run(fsm, 400, until=lambda: any(t.to == 'ABANDON' for t in fsm.history))
    ab = next(t for t in fsm.history if t.to == 'ABANDON')
    assert 'timed out' in ab.reason and ab.frm == 'NAVIGATE_TO_APPROACH'


def test_the_mission_time_budget_aborts():
    cfg = PlanningConfig(mission_timeout_s=30.0)
    env = SimEnv(cfg, {'a': (2.6, 1.4)}, ZONE, pose=START)
    fsm = RescueFSM(cfg)
    env.run(fsm, 100)
    assert fsm.state == State.MISSION_ABORTED and 'budget' in fsm.reason


def test_a_target_invalidated_from_outside_is_dropped_without_counting_a_failed_attempt():
    env, fsm = env_with({'a': (2.6, 1.4), 'b': (4.2, 1.0)})
    env.run(fsm, 200, until=lambda: fsm.state == State.NAVIGATE_TO_APPROACH)
    tgt = fsm.m.target
    b = env.belief[tgt]
    from fire_resq_planning.types import VictimBelief
    env.belief[tgt] = VictimBelief(b.id, b.x, b.y, b.confidence, UNREACHABLE, b.last_seen)     # someone else gave up on it
    env.run(fsm, 60, until=lambda: fsm.state == State.SELECT_TARGET and any(t.frm == 'ABANDON' for t in fsm.history))
    ab = next(t for t in fsm.history if t.to == 'ABANDON')
    assert 'UNREACHABLE' in ab.reason
    assert tgt not in fsm.m.attempts, 'an externally invalidated target is not a failed attempt'


def test_an_identity_mismatch_at_the_magnet_releases_it_and_does_not_carry_on():
    """Two victims side by side; the belief the FSM holds for the target is 30 cm off, so what the magnet takes is not it."""
    env, fsm = env_with({'a': (2.6, 1.4), 'b': (2.6, 1.7)}, belief_error_m=0.0)
    env.run(fsm, 300, until=lambda: fsm.state == State.ALIGN)
    assert fsm.state == State.ALIGN
    b = env.belief[fsm.m.target]
    from fire_resq_planning.types import VictimBelief
    # the FSM's belief about its target slides 0.3 m; it will stop where IT thinks the victim is
    env.belief[fsm.m.target] = VictimBelief(b.id, b.x, b.y - 0.28 if b.y > 1.5 else b.y + 0.28, b.confidence, b.status, b.last_seen)
    env.run(fsm, 600, until=lambda: any(t.frm in ('ATTACH', 'ALIGN') and t.to == 'ABANDON' for t in fsm.history))
    assert any(t.to == 'ABANDON' for t in fsm.history)
    assert fsm.m.rescued == []


def _world_with_target(conf):
    from fire_resq_planning.types import VictimBelief, World
    return World((VictimBelief('V1', 2.0, 1.0, conf, TARGETED, 0.0),), ZONE, stamp=10.0)


def test_a_target_already_faint_when_chosen_is_pursued_not_dropped_at_once():
    """Measured live: cognition chose a victim last seen ~100 s earlier (confidence 0.03); dropping it for being below the
    floor the moment navigation began made cognition choose it again - 157 abandons in a row. Driving towards it is how it
    gets seen again; only a MATERIAL decay since the choice invalidates it."""
    fsm = RescueFSM(CFG)
    fsm.m.target, fsm.m.targeted_t, fsm.m.target_xy = 'V1', 0.0, (2.0, 1.0)
    fsm.m.target_conf = 0.03
    assert fsm._target_invalid(Observation(t=10.0, world=_world_with_target(0.03))) is None
    fsm.m.target_conf = 0.8
    why = fsm._target_invalid(Observation(t=10.0, world=_world_with_target(0.05)))
    assert why is not None and 'confidence' in why
