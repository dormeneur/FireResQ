"""The rendered Nav2 parameters follow the robot description, and stay inside its limits."""
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src' / 'fire_resq_navigation'))
from fire_resq_navigation.nav2_params import render  # noqa: E402
from fire_resq_navigation.robot_params import load_robot_params  # noqa: E402

TEMPLATE = REPO / 'src' / 'fire_resq_navigation' / 'config' / 'nav2_params.yaml'
XACRO = REPO / 'src' / 'fire_resq_description' / 'urdf' / 'parameters.xacro'


@pytest.fixture(scope='module')
def rp():
    return load_robot_params(XACRO)


@pytest.fixture(scope='module')
def d(rp):
    return render(TEMPLATE, rp, bt_xml='/x/tree.xml')


def test_costmaps_use_a_circle_at_the_robots_circumscribed_radius(d, rp):
    """Planner (point) and controller (footprint) must agree; a polygon whose wheels protrude past
    its inscribed radius made them disagree ('collision ahead' aborts). See nav2_params.render."""
    for name in ('local_costmap', 'global_costmap'):
        p = d[name][name]['ros__parameters']
        assert 'footprint' not in p
        assert p['robot_radius'] == pytest.approx(rp.circumscribed_radius + 0.01, abs=2e-3)
        assert p['robot_radius'] > max(y for _, y in rp.footprint), 'the circle must cover the wheels'


def test_inflation_clears_the_robot_but_does_not_fill_the_arena(d, rp):
    r = d['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius']
    assert rp.circumscribed_radius < r < 0.4          # the stock 0.7 m would swallow the partition gap
    assert 2 * d['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] < 0.5, 'must fit the 0.5 m partition gap'


def test_controller_speeds_are_inside_the_drive_limits(d, rp):
    c = d['controller_server']['ros__parameters']['FollowPath']
    assert 0 < c['desired_linear_vel'] <= rp.max_linear_velocity
    assert 0 < c['rotate_to_heading_angular_vel'] <= rp.max_angular_velocity
    assert c['max_angular_accel'] == rp.max_angular_acceleration
    assert c['allow_reversing'] is False              # nothing looks behind the robot
    b = d['behavior_server']['ros__parameters']
    assert b['max_rotational_vel'] <= rp.max_angular_velocity


def test_no_backup_or_stock_turtlebot_baggage(d):
    b = d['behavior_server']['ros__parameters']
    assert b['behavior_plugins'] == ['spin', 'wait']
    text = yaml.safe_dump(d)
    for banned in ('backup', 'BackUp', 'MPPI', 'base_footprint', 'route_server', 'docking'):
        assert banned not in text, banned


def test_the_controller_and_planner_are_the_ones_we_chose(d):
    assert d['controller_server']['ros__parameters']['FollowPath']['plugin'].endswith('RegulatedPurePursuitController')
    assert d['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'


def test_only_the_frames_this_robot_has(d):
    text = yaml.safe_dump(d)
    assert 'base_footprint' not in text
    assert d['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_link'
    assert d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == '/x/tree.xml'


def test_global_costmap_is_a_rolling_window_that_always_contains_the_robot(d):
    """A map-sized global costmap failed every plan at the start: the SLAM map is only what has
    been SEEN, and the robot's own position was just outside it."""
    g = d['global_costmap']['global_costmap']['ros__parameters']
    assert g['rolling_window'] is True
    assert min(g['width'], g['height']) >= 8, 'must cover the 5 m arena diagonal (7.1 m) from any position'
    assert 'static_layer' in g['plugins'], 'the SLAM / saved map must still feed the costmap'
