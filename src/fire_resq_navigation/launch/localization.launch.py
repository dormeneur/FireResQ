"""Localise against a saved map: nav2_map_server + AMCL (scan-to-map). Publishes map -> odom.

  ros2 launch fire_resq_navigation localization.launch.py map:=/path/to/arena.yaml
  # then give the start pose:  RViz "2D Pose Estimate", or publish /initialpose

Do NOT run this together with mapping.launch.py: both publish map -> odom.
Save a map from a SLAM run with:
  ros2 run nav2_map_server map_saver_cli -f /path/to/arena
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fire_resq_navigation')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Path to the map YAML written by map_saver_cli.'),
        DeclareLaunchArgument('amcl_params', default_value=PathJoinSubstitution([pkg, 'config', 'amcl.yaml'])),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}, sim_time]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[LaunchConfiguration('amcl_params'), sim_time]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'], 'bond_timeout': 4.0}, sim_time]),
    ])
