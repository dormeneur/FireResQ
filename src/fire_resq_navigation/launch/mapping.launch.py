"""SLAM Toolbox (online, asynchronous) as the mapping backend.

Publishes /map and the map -> odom transform - the interface the rest of the system relies on, so
the backend can be swapped (AMCL against a saved map, a different SLAM) without touching cognition.

  slam_preset:=scan_matching   genuine scan-matching SLAM (default)        - see the preset's header
  slam_preset:=odom_only       odometry-only mapping. NOT SLAM: a labelled control/baseline.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fire_resq_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_preset', default_value='scan_matching',
                              description='scan_matching (SLAM) | odom_only (NOT SLAM, baseline)'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([pkg, 'config', ['slam_toolbox_', LaunchConfiguration('slam_preset'), '.yaml']]),
            description='Overrides the preset with an explicit SLAM Toolbox parameter file.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
            }.items()),
    ])
