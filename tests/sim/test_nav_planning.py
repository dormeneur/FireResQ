"""Navigation reliability for the rescue loop: what the planner tells cognition, and what Nav2 then does.

Runs on the mapped arena (AMCL on the saved map + Nav2). Two things are judged, both against Gazebo's TRUE geometry:

  * the PATH QUERY cognition uses (`Nav2PathQuery`): valid lengths for reachable goals, `unreachable` (the planner said no) kept
    apart from `unknown` (it could not be asked), and the detour round the partition showing up as a longer, less direct path;
  * NAVIGATION to what the rescue loop will ask for: the approach pose of every victim (the point `RescueTarget.approach_pose`
    carries), without touching the victim, and home again - plus a regression test for the goal-tolerance deadlock found in
    plan Phase 7 (the controller latched "arrived" and froze outside the goal checker's tolerance; `FollowPath.stateful: false`).

Victim positions appear here only to say WHERE THE REAL victims are, for judging; the query goals are computed like cognition does.
"""
import math

import pytest
from fire_resq_cognition import factors as F
from fire_resq_cognition.nav2_path_query import Nav2PathQuery
from simlib import entity_poses
from test_nav2 import G1, G3, go, true_dist

STANDOFF = 0.4


@pytest.fixture(scope='module')
def planner():
    q = Nav2PathQuery('/compute_path_to_pose', 'map', '', 5.0)
    yield q
    q.destroy()


def to_map(sc, wx, wy):
    return sc.world_to_odom(wx, wy)


def approach_for(sc, victim, robot_map=(0.0, 0.0)):
    return F.approach_pose(robot_map, to_map(sc, victim.x, victim.y), STANDOFF)


# ---------------------------------------------------------------- the path query
def test_reachable_goals_get_a_valid_length_no_shorter_than_the_straight_line(navamcl, scenario, planner):
    for wx, wy in [(-0.9, -0.9), (0.6, -2.0), (0.4, 0.3), (-0.4, 1.3)]:
        gx, gy = to_map(scenario, wx, wy)
        info = planner((0.0, 0.0), (gx, gy))
        assert info.status == 'reachable' and info.ok, (wx, wy, info)
        assert info.length_m >= math.hypot(gx, gy) - 0.06, f'a path of {info.length_m:.2f} m to a goal {math.hypot(gx, gy):.2f} m away'
        assert info.length_m <= 3.0 * math.hypot(gx, gy) + 0.5, 'a path this long in an open arena is a planner fault'


def test_a_goal_inside_a_known_solid_or_off_the_map_is_UNREACHABLE_not_unknown(navamcl, scenario, planner):
    """The distinction cognition depends on: 'the planner said no' is a verdict, 'could not ask' is not. Covers what the map KNOWS:
    the partition (its faces were seen from both sides), the isolated fire cone, and a point far off the map. (A goal in the interior
    of an obstacle whose faces were only partly seen is a documented limitation: see the xfail below.)"""
    part = next(o for o in scenario.obstacles if abs(o.size_x - 1.4) < 1e-6)                     # the partition
    for name, (wx, wy) in {'inside the partition': (part.x, part.y), 'inside the fire': (scenario.fire.x, scenario.fire.y),
                           'off the map': (40.0, 40.0)}.items():
        gx, gy = to_map(scenario, wx, wy)
        info = planner((0.0, 0.0), (gx, gy))
        assert info.status == 'unreachable', f'{name}: {info}'
        assert info.reason, f'{name}: an unreachable verdict must say why'


@pytest.mark.xfail(reason='KNOWN LIMITATION: the saved map holds only the SURFACES the depth wedge saw, so the interior of a block is UNKNOWN '
                          'space, and Nav2 plans through unknown space (planner `allow_unknown: true`, a documented TODO). A goal at the '
                          'centre of a 0.5 m block therefore comes back reachable with a detour into the unmapped interior. Cognition is '
                          'unaffected (approach points are 0.4 m outside a victim, never inside an obstacle); tighten allow_unknown once '
                          'the robot maps before it moves. Will XPASS when fixed.', strict=False)
def test_known_limitation_a_goal_inside_a_block_is_reported_reachable(navamcl, scenario, planner):
    part = next(o for o in scenario.obstacles if abs(o.size_x - 1.4) < 1e-6)
    block = next(o for o in scenario.obstacles if o is not part)
    info = planner((0.0, 0.0), to_map(scenario, block.x, block.y))
    assert info.status == 'unreachable', f'a goal at the centre of a solid block: {info}'


def test_the_partition_makes_the_hidden_victims_route_a_detour_and_the_others_direct(navamcl, scenario, planner):
    direct = {}
    for v in scenario.victims:
        ax, ay, _ = approach_for(scenario, v)
        info = planner((0.0, 0.0), (ax, ay))
        assert info.ok, (v.id, info)
        direct[v.id] = F.accessibility(math.hypot(ax, ay), info.length_m)
    print('\nroute directness (1 = straight):', {k: round(x, 2) for k, x in direct.items()})
    assert direct['victim_3'] < 0.8, 'victim_3 is behind the partition: its route must be a visible detour'
    assert direct['victim_1'] > 0.9 and direct['victim_2'] > 0.9


def test_every_victims_approach_point_is_reachable_out_and_home_to_the_safe_zone(navamcl, scenario, planner):
    for v in scenario.victims:
        ax, ay, _ = approach_for(scenario, v)
        out, back = planner((0.0, 0.0), (ax, ay)), planner((ax, ay), (0.0, 0.0))
        assert out.ok and back.ok, (v.id, out, back)
        assert abs(out.length_m - back.length_m) < 0.5, f'{v.id}: out {out.length_m:.2f} m, back {back.length_m:.2f} m'


def test_answering_the_same_question_twice_gives_the_same_length(navamcl, scenario, planner):
    """A planner that gave different lengths for one question would make decisions flip for no reason."""
    ax, ay, _ = approach_for(scenario, scenario.victims[1])
    a, b = planner((0.0, 0.0), (ax, ay)), planner((0.0, 0.0), (ax, ay))
    assert a.length_m == pytest.approx(b.length_m, abs=0.01)


# ---------------------------------------------------------------- navigating to what the rescue loop will ask for
@pytest.mark.parametrize('victim_index', [0, 1, 2], ids=['victim_1', 'victim_2', 'victim_3'])
def test_nav2_reaches_the_approach_pose_of_each_victim_without_touching_it_and_comes_home(navamcl, scenario, victim_index):
    v = scenario.victims[victim_index]
    before = entity_poses({v.id})[v.id]
    bx, by, byaw = navamcl.bot.map_pose()
    ax, ay, ayaw = approach_for(scenario, v, (bx, by))
    world_goal = scenario.odom_to_world(ax, ay)
    res, track = go(navamcl, scenario, (world_goal[0], world_goal[1], wrap_world(ayaw + scenario.robot_start.yaw)))
    print(f"MEASURED approach to {v.id}: {res['status']} in {res['seconds']:.0f} s; true distance to the approach point "
          f"{true_dist(navamcl, *world_goal) * 100:.1f} cm; to the victim {math.hypot(navamcl.gt.get()[0] - v.x, navamcl.gt.get()[1] - v.y) * 100:.1f} cm")
    assert res['status'] == 'succeeded', res
    assert true_dist(navamcl, *world_goal) <= 0.25
    gap = min(math.hypot(g[0] - v.x, g[1] - v.y) for g in track)
    assert gap >= 0.25, f'the robot came within {gap * 100:.0f} cm of {v.id}'                    # Nav2's own floor is ~0.27 m
    after = entity_poses({v.id})[v.id]
    assert math.hypot(after[0] - before[0], after[1] - before[1]) < 0.01, f'{v.id} was moved by {100 * math.hypot(after[0] - before[0], after[1] - before[1]):.1f} cm'
    home, _ = go(navamcl, scenario, G3)
    assert home['status'] == 'succeeded', home                                               # and back to the safe zone


def wrap_world(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- the regression for the tolerance deadlock
def test_repeated_goals_at_the_tolerance_edge_never_deadlock(navamcl, scenario):
    """Plan Phase 7 root cause: RPP's `stateful` latch froze the robot in rotate-only mode outside the goal checker's tolerance, so a goal
    that ends at the 10 cm edge failed about one time in three. Five out-and-back rounds of the goal that failed most often: with
    the latch (`FollowPath.stateful: true`) the chance that all ten succeed is a few per cent."""
    results = []
    for _ in range(5):
        for goal in (G1, G3):
            res, _ = go(navamcl, scenario, goal)
            results.append(res['status'])
            assert res['status'] == 'succeeded', f'{goal}: {res} (after {results})'
