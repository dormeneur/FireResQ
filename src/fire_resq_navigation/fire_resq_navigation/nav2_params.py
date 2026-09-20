"""Render the Nav2 parameter file: the template plus values derived from the robot description.

The footprint, speed and acceleration limits and the inflation radius are COMPUTED from
parameters.xacro (see robot_params.py) and written into a temporary copy of nav2_params.yaml, so
they are never restated by hand. Pure Python: unit-tested without ROS.

TODO(future): recovery from failed navigation (backup/clear-costmap) - deliberately absent because
              the robot cannot see behind itself; needs a sensing answer first.
TODO(hardware): tune the controller and costmaps on the physical robot (Hardware.md TODO).
"""
from __future__ import annotations

import copy
import tempfile
from pathlib import Path
from typing import Optional

import yaml

from .robot_params import RobotParams, load_robot_params

FOOTPRINT_PADDING = 0.01         # metres added to the circumscribed radius
INFLATION_MARGIN = 0.10          # metres beyond the robot radius
CRUISE_FRACTION = 0.8            # cruise at 80% of the drive's maximum linear speed
BT_FILE = 'navigate_w_replanning_only_if_path_becomes_invalid.xml'


def _bt_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory('nav2_bt_navigator')) / 'behavior_trees' / BT_FILE)
    except Exception:
        return str(Path('/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees') / BT_FILE)


def render(template: Path, robot: Optional[RobotParams] = None, bt_xml: Optional[str] = None) -> dict:
    robot = robot or load_robot_params()
    d = copy.deepcopy(yaml.safe_load(Path(template).read_text()))
    # A CIRCLE at the circumscribed radius, not the footprint polygon. The planner (NavFn) plans for
    # a point and the controller checks the real footprint; with a polygon whose wheels stick out
    # further (9.8 cm) than its inscribed radius (7.5 cm) they DISAGREE: the planner routed the
    # centre ~20 cm from a victim and the controller then aborted with "collision ahead" (measured
    # in Phase 4). A circle makes both agree by construction. It is slightly conservative and the
    # narrowest gap in the arena (0.5 m) is well over its diameter (0.32 m).
    # TODO(future): a tighter polygon may be worth it for the close victim approach (Phase 8-10).
    radius = round(robot.circumscribed_radius + FOOTPRINT_PADDING, 3)
    inflation = round(radius + INFLATION_MARGIN, 3)
    for name in ('local_costmap', 'global_costmap'):
        p = d[name][name]['ros__parameters']
        p.pop('footprint', None)
        p.pop('footprint_padding', None)
        p['robot_radius'] = radius
        p['inflation_layer']['inflation_radius'] = inflation
    fp_ctrl = d['controller_server']['ros__parameters']['FollowPath']
    fp_ctrl['desired_linear_vel'] = round(CRUISE_FRACTION * robot.max_linear_velocity, 3)
    fp_ctrl['rotate_to_heading_angular_vel'] = round(CRUISE_FRACTION * robot.max_angular_velocity, 3)
    fp_ctrl['max_angular_accel'] = robot.max_angular_acceleration
    b = d['behavior_server']['ros__parameters']
    b['max_rotational_vel'] = round(CRUISE_FRACTION * robot.max_angular_velocity, 3)
    b['rotational_acc_lim'] = robot.max_angular_acceleration
    d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml or _bt_path()
    return d


def render_to_file(template: Path, use_sim_time: bool, out_dir: Optional[Path] = None) -> Path:
    d = render(template)
    for node in ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server'):
        d[node]['ros__parameters']['use_sim_time'] = use_sim_time
    for cm in ('local_costmap', 'global_costmap'):
        d[cm][cm]['ros__parameters']['use_sim_time'] = use_sim_time
    out = Path(out_dir or tempfile.gettempdir()) / 'fire_resq_nav2_params.yaml'
    out.write_text(yaml.safe_dump(d, sort_keys=False))
    return out
