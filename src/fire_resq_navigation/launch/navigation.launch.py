"""Nav2: planner + controller + behaviours + BT navigator + lifecycle manager. Nothing else.

Needs map -> odom (from mapping.launch.py or localization.launch.py), /odom, /scan and /map.
Goals arrive on the standard interface: the /navigate_to_pose action, or a PoseStamped on
/goal_pose (RViz "2D Goal Pose"). Velocity commands go out on /cmd_vel, the same topic the
simulated drive and the ESP32 bridge consume.

The Nav2 parameter file is RENDERED at launch: footprint, speed limits and inflation are computed
from the robot description (fire_resq_navigation/nav2_params.py), never restated here.
"""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fire_resq_navigation.nav2_params import render_to_file

SERVERS = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
           ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    template = Path(LaunchConfiguration('nav2_params').perform(context))
    params = render_to_file(template, use_sim_time, Path(tempfile.gettempdir()))
    nodes = [Node(package=pkg, executable=exe, name=exe, output='screen', parameters=[str(params)])
             for pkg, exe in SERVERS]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 4.0,
                     'node_names': [exe for _, exe in SERVERS]}]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=str(share / 'config' / 'nav2_params.yaml')),
        OpaqueFunction(function=_setup),
    ])
