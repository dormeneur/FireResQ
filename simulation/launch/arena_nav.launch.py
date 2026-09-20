"""Rescue arena + navigation stack: the Phase 4 simulation bringup.

  ros2 launch fire_resq_simulation arena_nav.launch.py [gui:=false] [scenario:=...]

arena.launch.py (unchanged) provides the world, the robot and the ROS bridge; nav_stack.launch.py
(shared with hardware) provides depth -> /scan and, as they are added, SLAM / localization / Nav2.

The camera defaults to 87 deg here, not the 60 deg placeholder of the plain arena launch. Phase 4
Step 0 measured that scan-matching SLAM is only good on the wider, RealSense-like wedge
(docs/Implementation_Plan.md, Phase 4). The Phase 2-3 launches keep 60 deg so their behaviour and
tests are unchanged.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    sim = Path(get_package_share_directory('fire_resq_simulation')) / 'launch'
    nav = Path(get_package_share_directory('fire_resq_navigation')) / 'launch'
    perc = Path(get_package_share_directory('fire_resq_perception')) / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('localization', default_value='slam',
                              description='slam | amcl | none  (who publishes map -> odom)'),
        DeclareLaunchArgument('map', default_value='', description='Saved map yaml for localization:=amcl.'),
        DeclareLaunchArgument('navigation', default_value='true', description='Start Nav2.'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='RViz navigation view.'),
        DeclareLaunchArgument('slam_preset', default_value='scan_matching',
                              description='scan_matching (SLAM) | odom_only (NOT SLAM, baseline)'),
        DeclareLaunchArgument('perception', default_value='false', description='Also run the perception node.'),
        DeclareLaunchArgument('perception_frame', default_value='map',
                              description='Frame perception publishes positions in (map needs localization != none).'),
        DeclareLaunchArgument('spatial_backend', default_value='auto', description='auto | depth | known_height'),
        DeclareLaunchArgument('camera_hfov', default_value='1.518',
                              description='Depth/RGB horizontal FOV, radians (1.518 = 87 deg).'),
        # SCOPED, because launch arguments are GLOBAL across included launch files: the values set for
        # the arena include would otherwise overwrite this file's own arguments for everything that
        # comes after it (measured: `use_rviz:=false` for the arena silently disabled the
        # navigation RViz; before that, the user's `use_rviz:=true` leaked into the arena and
        # opened a second RViz).
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sim / 'arena.launch.py')),
                launch_arguments={
                    'scenario': LaunchConfiguration('scenario'),
                    'gui': LaunchConfiguration('gui'),
                    'use_depth': 'true',                   # the navigation stack needs depth
                    'use_rviz': 'false',                   # RViz comes from the navigation stack
                    'perception': 'false',                 # ...and so does perception (in `map`); else it would start twice
                    'camera_hfov': LaunchConfiguration('camera_hfov'),
                }.items()),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / 'nav_stack.launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'localization': LaunchConfiguration('localization'),
                              'map': LaunchConfiguration('map'),
                              'navigation': LaunchConfiguration('navigation'),
                              'use_rviz': LaunchConfiguration('use_rviz'),
                              'slam_preset': LaunchConfiguration('slam_preset')}.items()),
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration('perception')), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(perc / 'perception.launch.py')),
                launch_arguments={'use_sim_time': 'true',
                                  'target_frame': LaunchConfiguration('perception_frame'),
                                  'spatial_backend': LaunchConfiguration('spatial_backend')}.items()),
        ]),
    ])
