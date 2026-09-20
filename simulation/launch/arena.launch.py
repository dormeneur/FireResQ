"""FireResQ rescue arena: generate the world from a scenario, then launch the simulation.

  ros2 launch fire_resq_simulation arena.launch.py                     # default scenario, GUI
  ros2 launch fire_resq_simulation arena.launch.py gui:=false          # headless
  ros2 launch fire_resq_simulation arena.launch.py use_depth:=false    # RGB-only robot
  ros2 launch fire_resq_simulation arena.launch.py scenario:=/path/to/other.yaml

The world is regenerated from the scenario YAML on every launch, and generation is
deterministic (same YAML -> byte-identical SDF), so the same command always starts the same
arena with the robot at the same pose. The robot, sensors, bridge and TF are exactly those of
sim.launch.py, which this file wraps - it does not fork or duplicate any of them.
"""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fire_resq_simulation import load_scenario, resolve_scenario, write_world


def _setup(context, *args, **kwargs):
    share = Path(get_package_share_directory('fire_resq_simulation'))
    scenario = load_scenario(resolve_scenario(LaunchConfiguration('scenario').perform(context)))
    overview = LaunchConfiguration('overview_camera').perform(context).lower() == 'true'

    world = write_world(
        scenario, Path(tempfile.gettempdir()) / 'fire_resq_worlds' / f'{scenario.name}.sdf',
        overview_camera=overview)

    s = scenario.robot_start
    actions = [
        # Lets <include><uri>model://victim</uri> and model://fire resolve.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share / 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'sim.launch.py')),
            launch_arguments={
                'world': str(world),
                'x': str(s.x), 'y': str(s.y), 'yaw': str(s.yaw),
                'gui': LaunchConfiguration('gui').perform(context),
                'use_rviz': LaunchConfiguration('use_rviz').perform(context),
                'use_depth': LaunchConfiguration('use_depth').perform(context),
                'camera_hfov': LaunchConfiguration('camera_hfov').perform(context),
            }.items()),
    ]
    if overview:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='bridge_overview',
            arguments=['/overview/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default',
                              description='Scenario name (config/scenarios/<name>.yaml) or path.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI. false = server only (headless).'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Also launch RViz2.'),
        DeclareLaunchArgument('use_depth', default_value='true',
                              description='Simulate the depth camera (false = RGB-only robot).'),
        DeclareLaunchArgument('camera_hfov', default_value='1.047',
                              description='Camera horizontal FOV, radians (1.047 = 60 deg, 1.518 = 87 deg).'),
        DeclareLaunchArgument('overview_camera', default_value='false',
                              description='DEBUG: add a top-down camera on /overview/image_raw.'),
        OpaqueFunction(function=_setup),
    ])
