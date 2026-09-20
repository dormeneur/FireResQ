"""The navigation stack: depth -> /scan, then (added step by step) SLAM / localization and Nav2.

Runs unchanged on the simulated robot and on hardware: it only needs the depth image, odom and TF.
Simulation composes it with the arena in simulation/launch/arena_nav.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


VALID_LOCALIZATION = ('slam', 'amcl', 'none')


def _check_localization(context, *args, **kwargs):
    """Exactly ONE node may publish map -> odom. Reject anything that could start two."""
    value = LaunchConfiguration('localization').perform(context)
    if value not in VALID_LOCALIZATION:
        raise RuntimeError(f"localization:='{value}' is invalid; choose one of {VALID_LOCALIZATION}")
    if value == 'amcl' and not LaunchConfiguration('map').perform(context):
        raise RuntimeError("localization:=amcl needs map:=<path to the saved map yaml>")
    return []


def generate_launch_description():
    pkg = FindPackageShare('fire_resq_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when driven by a simulator clock.'),
        DeclareLaunchArgument('localization', default_value='slam',
                              description='Who publishes map -> odom: slam (SLAM Toolbox) | amcl (saved map) | '
                                          'none. Exactly one publisher may exist.'),
        DeclareLaunchArgument('map', default_value='', description='Saved map yaml (localization:=amcl).'),
        OpaqueFunction(function=_check_localization),
        DeclareLaunchArgument('navigation', default_value='true', description='Start Nav2 (needs map -> odom).'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='RViz with the navigation view.'),
        DeclareLaunchArgument('slam_preset', default_value='scan_matching',
                              description='scan_matching (SLAM) | odom_only (NOT SLAM, baseline)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'scan.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'slam_preset': LaunchConfiguration('slam_preset')}.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'slam'"]))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'navigation.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'localization.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'map': LaunchConfiguration('map')}.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'amcl'"]))),
    ])
