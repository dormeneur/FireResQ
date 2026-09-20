"""FireResQ simulation bringup: Gazebo + robot + ROS bridge.

Phase 2 scope: the robot is controllable (cmd_vel), reports odometry and wheel joint
states, and publishes RGB (and optionally depth) images. No SLAM, Nav2, perception or
cognition is started here - those attach to these topics in later phases.

Nothing in this file is hardware-specific or cognitive. On the physical robot the same
ROS-level topics come from the ESP32 bridge and a camera driver instead of this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = FindPackageShare('fire_resq_simulation')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_sim, 'worlds', 'empty_world.sdf']),
            description='World file to load.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo GUI. Set false for headless runs.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', description='Also launch RViz2.'),
        DeclareLaunchArgument(
            'use_depth', default_value='true',
            description='Simulate the depth camera. false = RGB-only robot, and no '
                        '/camera/depth/* topics exist.'),
        DeclareLaunchArgument(
            'camera_hfov', default_value='1.047',
            description='Camera horizontal FOV in radians (1.047 = 60 deg, 1.518 = 87 deg RealSense-like).'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y (m).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad).'),
    ]

    # The SIMULATION wrapper, not the bare description: it adds Gazebo-only content.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_sim, 'urdf', 'fire_resq_gazebo.urdf.xacro']),
            ' use_depth:=', LaunchConfiguration('use_depth'),
            ' camera_hfov:=', LaunchConfiguration('camera_hfov'),
        ]),
        value_type=str,
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                LaunchConfiguration('world'),
                # -r: start running. -s: server only, i.e. headless (gui:=false).
                ' -r -v 2',
                PythonExpression(
                    ["' -s' if '", LaunchConfiguration('gui'), "' == 'false' else ''"]),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fire_resq',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.08',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge_base = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_base',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_sim, 'config', 'bridge_base.yaml']),
            'use_sim_time': True,
        }],
    )

    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_sim, 'config', 'bridge_depth.yaml']),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_depth')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_sim, 'config', 'sim.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [gz, rsp, spawn, bridge_base, bridge_depth, rviz])
