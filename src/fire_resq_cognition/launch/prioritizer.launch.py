"""Cognition: /fire_resq/world_state -> /fire_resq/rescue_target (+ SelectTarget), asking Nav2's planner for path lengths.

  ros2 launch fire_resq_cognition prioritizer.launch.py
  ros2 launch fire_resq_cognition prioritizer.launch.py decision_model:=nearest use_sim_time:=true

Runs unchanged on the simulated and the real robot: it needs only the world state and a Nav2 planner.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
              'decision_model': LaunchConfiguration('decision_model').perform(context)}
    return [Node(package='fire_resq_cognition', executable='prioritizer_node', name='prioritizer_node', output='screen',
                 parameters=[LaunchConfiguration('prioritizer_params').perform(context), params])]


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_cognition'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('decision_model', default_value='weighted_utility', description='weighted_utility | nearest'),
        DeclareLaunchArgument('prioritizer_params', default_value=str(share / 'config' / 'prioritizer.yaml')),
        OpaqueFunction(function=_setup),
    ])
