"""Navigation Step 4: Nav2 drives the robot to goals it is GIVEN.

Goals are chosen by the test (the robot has no exploration or decision logic yet). What is judged:
Nav2's own state, and - independently - Gazebo's TRUE pose, because Nav2 can only be as right as
the localization under it.

Nav2 is run in the MAPPED-ARENA mode: the arena is mapped first (SLAM, translation-based routes),
then Nav2 navigates under AMCL on the saved map. Running Nav2 while scan-matching SLAM is still
localizing is a KNOWN LIMITATION (see the xfail at the bottom): SLAM Toolbox can inject a heading
jump at the first scan after a large in-place rotation, which Nav2's rotate-to-heading produces.
"""
import math
import subprocess

import pytest

from simlib import lifecycle_state

NAV_SERVERS = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

# WORLD-frame goals. G2 forces a detour round obstacle_2; G3 brings the robot home.
G1 = (-0.9, -0.9, math.pi / 4)
G2 = (0.6, -2.0, 0.0)
G3 = (-1.9, -1.9, math.pi / 4)


def true_dist(env, wx, wy):
    g = env.gt.get()
    return math.hypot(g[0] - wx, g[1] - wy)


def nav2_problems(env, since_line):
    """Nav2's own warnings/errors since a point in the launch log - so a failed goal explains itself."""
    keep = ('planner_server', 'controller_server', 'bt_navigator', 'behavior_server', 'costmap')
    lines = env.sim.log().splitlines()[since_line:]
    bad = [ln for ln in lines if any(k in ln for k in keep) and ('[WARN]' in ln or '[ERROR]' in ln)
           and 'Message Filter dropping' not in ln]
    return bad[-12:]


def go(env, sc, goal, timeout=120.0):
    """Send a WORLD goal (converted to the map frame = the robot's start frame) and record the true track."""
    track = []
    mx, my = sc.world_to_odom(goal[0], goal[1])
    myaw = goal[2] - sc.robot_start.yaw
    mark = len(env.sim.log().splitlines())
    res = env.bot.navigate_to(mx, my, myaw, timeout=timeout, on_tick=lambda: track.append(env.gt.get()))
    if res['status'] != 'succeeded':
        res['nav2'] = nav2_problems(env, mark)
        print('NAV2 LOG for the failed goal:\n  ' + '\n  '.join(l[-230:] for l in res['nav2']))
    return res, track


# ---------------------------------------------------------------- bring-up
def test_nav2_lifecycle_nodes_become_active(navamcl):
    for n in NAV_SERVERS:
        assert lifecycle_state(f'/{n}') == 'active', n


def test_navigation_action_server_is_available(navamcl):
    assert navamcl.bot._nav_client.wait_for_server(timeout_sec=20.0)


def test_nav2_uses_our_footprint_and_controller_not_turtlebot_defaults(navamcl):
    def param(node, name):
        return subprocess.run(['ros2', 'param', 'get', node, name], capture_output=True, text=True).stdout
    assert 'RegulatedPurePursuitController' in param('/controller_server', 'FollowPath.plugin')
    assert 'NavfnPlanner' in param('/planner_server', 'GridBased.plugin')
    r = float(param('/local_costmap/local_costmap', 'robot_radius').split()[-1])
    assert 0.15 < r < 0.20, r                      # derived from parameters.xacro, not the TurtleBot 0.22 m
    assert 'base_footprint' not in param('/bt_navigator', 'robot_base_frame')


def test_only_the_expected_nodes_publish_tf_with_nav2_running(navamcl):
    from test_nav_scan import tf_publishers
    assert tf_publishers() == ['amcl', 'bridge_base', 'robot_state_publisher']


# ---------------------------------------------------------------- navigation
def test_goal_is_accepted_and_the_robot_moves_toward_it(navamcl, scenario):
    d0 = true_dist(navamcl, *G1[:2])
    res, track = go(navamcl, scenario, G1)
    print(f"MEASURED goal G1: {res}; start {d0:.2f} m away, closest {min(math.hypot(g[0] - G1[0], g[1] - G1[1]) for g in track):.2f} m")
    assert res['accepted'], res
    d_min = min(math.hypot(g[0] - G1[0], g[1] - G1[1]) for g in track)
    assert d_min < d0 - 0.5, f'never got closer: {d0:.2f} m -> best {d_min:.2f} m'


def test_navigation_completes_within_tolerance(navamcl, scenario):
    """Nav2's SUCCEEDED means the goal was reached in the MAP frame; the truth check says it was
    reached in the real world too (allowing for the localization error underneath)."""
    res, _ = go(navamcl, scenario, G1)                          # already there or re-issued: must succeed
    print(f"MEASURED G1 re-issue: {res}; true distance to goal {true_dist(navamcl, *G1[:2]) * 100:.1f} cm")
    assert res['status'] == 'succeeded', res
    assert true_dist(navamcl, *G1[:2]) <= 0.25, f'{true_dist(navamcl, *G1[:2]) * 100:.0f} cm from the goal in the real world'


def test_navigation_avoids_an_obstacle_it_must_go_round(navamcl, scenario):
    res, track = go(navamcl, scenario, G2)
    o2m = next(o for o in scenario.obstacles if o.name == 'obstacle_2')
    print(f"MEASURED goal G2 (detour): {res}; true distance to goal {true_dist(navamcl, *G2[:2]) * 100:.1f} cm; closest approach to obstacle_2 {min(max(abs(g[0] - o2m.x) - o2m.size_x / 2, abs(g[1] - o2m.y) - o2m.size_y / 2) for g in track) * 100:.0f} cm")
    assert res['status'] == 'succeeded', res
    assert true_dist(navamcl, *G2[:2]) <= 0.25
    o2 = next(o for o in scenario.obstacles if o.name == 'obstacle_2')
    for g in track:                                            # never inside the obstacle plus the robot's own radius
        gap = max(abs(g[0] - o2.x) - o2.size_x / 2, abs(g[1] - o2.y) - o2.size_y / 2)
        assert gap > 0.08, f'passed {gap * 100:.0f} cm from obstacle_2'


def test_robot_stays_upright_and_on_its_wheels_throughout(navamcl):
    import math as _m
    from simlib import WHEEL_R
    g = navamcl.gt.get()
    assert g[2] <= WHEEL_R + 1e-4 and max(abs(g[3]), abs(g[4])) < _m.radians(1.5)


def test_navigation_home_again(navamcl, scenario):
    res, _ = go(navamcl, scenario, G3)
    print(f"MEASURED goal G3 (home): {res}; true distance to goal {true_dist(navamcl, *G3[:2]) * 100:.1f} cm")
    assert res['status'] == 'succeeded', res
    assert true_dist(navamcl, *G3[:2]) <= 0.25


# ---------------------------------------------------------------- known limitation
@pytest.mark.xfail(reason='KNOWN LIMITATION: scan-matching SLAM (SLAM Toolbox) running concurrently with Nav2 can inject a '
                          'heading jump (~44 deg, measured) at the first scan after a large in-place rotation; odometry '
                          'and truth agree throughout, so it is SLAM\'s correction. With odometry-only mapping the same '
                          'goals all succeed. Localize with AMCL on a saved map while navigating.', strict=False)
def test_known_limitation_nav2_under_scan_matching_slam(navnav, scenario):
    """Documented, not hidden. If this ever XPASSes reliably, the limitation is fixed: update the docs."""
    for goal in (G1, G2, G3):
        res, _ = go(navnav, scenario, goal)
        assert res['status'] == 'succeeded', res
        assert true_dist(navnav, *goal[:2]) <= 0.25, f'localization drifted: {true_dist(navnav, *goal[:2]) * 100:.0f} cm from the goal'
