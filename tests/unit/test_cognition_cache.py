"""The path-query cache and the approach pose: what may be remembered, for how long, and what the standoff must respect."""
import math
import re
from pathlib import Path

import pytest
from fire_resq_cognition import factors as F
from fire_resq_cognition.config import PrioritizerConfig
from fire_resq_cognition.path_cache import CachedPathQuery
from fire_resq_cognition.prioritizer import make_prioritizer
from fire_resq_cognition.types import DETECTED, PathInfo, PlanningContext, VictimView, WorldView

REPO = Path(__file__).resolve().parents[2]


class Planner:
    """A counting fake planner; `answer` decides what it says."""

    def __init__(self, answer=None):
        self.calls, self.answer = [], answer or (lambda a, b: PathInfo.reachable(math.hypot(a[0] - b[0], a[1] - b[1])))

    def __call__(self, a, b):
        self.calls.append((a, b))
        return self.answer(a, b)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


# ------------------------------------------------------------------------------ the cache
def test_a_repeated_question_is_answered_from_memory_and_the_planner_is_asked_once():
    p, c = Planner(), Clock()
    q = CachedPathQuery(p, ttl_s=10.0, clock=c)
    first = q((0.0, 0.0), (3.0, 4.0))
    assert first.ok and first.length_m == pytest.approx(5.0)
    assert q((0.0, 0.0), (3.0, 4.0)) is first and q((0.0, 0.0), (3.0, 4.0)) is first
    assert len(p.calls) == 1 and q.asked == 1 and q.hits == 2


def test_an_answer_expires_after_the_ttl_so_a_costmap_change_is_eventually_heard():
    p, c = Planner(), Clock()
    q = CachedPathQuery(p, ttl_s=10.0, clock=c)
    q((0.0, 0.0), (1.0, 0.0))
    c.t += 9.9
    q((0.0, 0.0), (1.0, 0.0))
    assert len(p.calls) == 1
    c.t += 0.2
    q((0.0, 0.0), (1.0, 0.0))
    assert len(p.calls) == 2


def test_only_reachable_answers_are_remembered():
    """Nav2 says 'goal outside the map' while its costmap starts, and 'unknown' when it cannot be asked: both are exactly the
    answers a retry exists to change, so they are never cached."""
    for answer in (PathInfo.unreachable('goal outside the map'), PathInfo.unknown('timeout')):
        p = Planner(lambda a, b, answer=answer: answer)
        q = CachedPathQuery(p, ttl_s=10.0, clock=Clock())
        q((0.0, 0.0), (1.0, 0.0))
        q((0.0, 0.0), (1.0, 0.0))
        assert len(p.calls) == 2, answer.status


def test_a_planner_that_changes_its_mind_is_heard_at_once_for_a_negative_answer():
    state = {'up': False}
    p = Planner(lambda a, b: PathInfo.reachable(1.0) if state['up'] else PathInfo.unreachable('starting'))
    q = CachedPathQuery(p, ttl_s=10.0, clock=Clock())
    assert q((0.0, 0.0), (1.0, 0.0)).status == 'unreachable'
    state['up'] = True
    assert q((0.0, 0.0), (1.0, 0.0)).ok


def test_start_and_goal_are_matched_at_the_costmap_resolution():
    p = Planner()
    q = CachedPathQuery(p, ttl_s=10.0, quantum_m=0.05, clock=Clock())
    q((0.0, 0.0), (2.0, 0.0))
    q((0.01, -0.01), (2.01, 0.01))                     # within a cell: the same question
    assert len(p.calls) == 1
    q((0.0, 0.0), (2.20, 0.0))                         # a different goal
    q((0.30, 0.0), (2.0, 0.0))                         # a different start (the robot moved)
    assert len(p.calls) == 3


def test_invalidate_forgets_everything_and_a_zero_ttl_disables_the_cache():
    p, c = Planner(), Clock()
    q = CachedPathQuery(p, ttl_s=10.0, clock=c)
    q((0.0, 0.0), (1.0, 0.0))
    q.invalidate()
    q((0.0, 0.0), (1.0, 0.0))
    assert len(p.calls) == 2
    off = CachedPathQuery(p, ttl_s=0.0, clock=c)
    off((0.0, 0.0), (1.0, 0.0))
    off((0.0, 0.0), (1.0, 0.0))
    assert len(p.calls) == 4 and off.hits == 0


def test_a_bad_quantum_is_rejected_and_destroy_reaches_the_wrapped_query():
    with pytest.raises(ValueError):
        CachedPathQuery(Planner(), quantum_m=0.0)
    seen = []
    inner = Planner()
    inner.destroy = lambda: seen.append('destroyed')
    CachedPathQuery(inner).destroy()
    assert seen == ['destroyed']


def test_a_second_decision_on_the_same_world_costs_the_planner_nothing():
    """Two questions per candidate on the first decision, none on the second: what SelectTarget right after an automatic decision saves."""
    p = Planner()
    ctx = PlanningContext(CachedPathQuery(p, ttl_s=10.0, clock=Clock()))
    world = WorldView(1.0, tuple(VictimView(f'v{i}', 1.0 + i, 0.5 * i, 0.9, DETECTED) for i in range(3)), True, (6.0, 0.0), (0.0, 0.0), (0.0, 0.0))
    prio = make_prioritizer('weighted_utility')
    first = prio.select(world, ctx)
    assert len(p.calls) == 6
    second = prio.select(world, ctx)
    assert len(p.calls) == 6 and [s.victim_id for s in first.ranked] == [s.victim_id for s in second.ranked]
    assert [s.utility for s in first.ranked] == [s.utility for s in second.ranked]


# ------------------------------------------------------------------------------ the approach pose
def test_the_approach_pose_is_the_standoff_point_facing_the_victim():
    x, y, yaw = F.approach_pose((0.0, 0.0), (3.0, 4.0), 1.0)
    assert math.hypot(x - 3.0, y - 4.0) == pytest.approx(1.0)
    assert yaw == pytest.approx(math.atan2(4.0, 3.0))
    x, y, yaw = F.approach_pose((5.0, 5.0), (1.0, 5.0), 0.4)                       # coming from the east, facing west
    assert (x, y) == pytest.approx((1.4, 5.0)) and abs(yaw) == pytest.approx(math.pi)


def test_the_approach_pose_faces_the_victim_even_when_the_robot_is_already_inside_the_standoff():
    x, y, yaw = F.approach_pose((1.0, 1.0), (1.2, 1.0), 0.4)
    assert (x, y) == (1.0, 1.0) and yaw == pytest.approx(0.0)


# ------------------------------------------------------------------------------ the standoff must be physically sensible
def _props():
    text = (REPO / 'src/fire_resq_description/urdf/parameters.xacro').read_text()
    return {n: float(v) for n, v in re.findall(r'<xacro:property\s+name="(\w+)"\s+value="([-\d.]+)"', text)}


def test_the_standoff_is_reachable_for_nav2_and_leaves_a_creep_for_the_magnet():
    """Derived from the robot's own description and the victim model, so it cannot silently rot when either changes:
      * NAV2 side: the planner sees the robot as a circle of the circumscribed radius, so it can take the robot no closer to a
        victim's centre than radius + the victim's ring + one 5 cm cell. The standoff must be at least that.
      * MAGNET side: contact is base_link -> magnet front face -> the ring, so the standoff must exceed that contact distance,
        leaving a creep (Phase 10's open-loop ALIGN, not Nav2's) between the approach pose and pickup."""
    import sys
    sys.path.insert(0, str(REPO / 'src' / 'fire_resq_navigation'))
    from fire_resq_navigation.robot_params import load_robot_params
    robot = load_robot_params()
    ring = float(re.search(r'<cylinder><radius>([\d.]+)</radius><length>0.055</length>', (REPO / 'simulation/models/victim/model.sdf').read_text())[1])
    props = _props()
    contact = props['magnet_x'] + props['magnet_length'] / 2 + ring
    nav2_floor = robot.circumscribed_radius + ring + 0.05
    standoff = PrioritizerConfig().approach_standoff_m
    assert standoff >= nav2_floor + 0.05, f'standoff {standoff} m is inside what Nav2 can plan to ({nav2_floor:.3f} m + margin)'
    assert standoff > contact + 0.05, f'no creep left between the standoff {standoff} m and magnet contact {contact:.3f} m'
    print(f'\nstandoff {standoff} m; Nav2 floor {nav2_floor:.3f} m; magnet contact {contact:.3f} m; ALIGN creep {standoff - contact:.3f} m')
