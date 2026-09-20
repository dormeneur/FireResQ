"""Inspect the robot description on its own - no Gazebo, no simulation.

This launch file exists to prove the description stands alone, which is what makes it
reusable on the physical robot later.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fire_resq_description')

    model = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([pkg, 'urdf', 'fire_resq.urdf.xacro']),
        description='Absolute path to the robot xacro.',
    )
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2.')
    use_jsp = DeclareLaunchArgument(
        'use_joint_state_publisher', default_value='true',
        description='Publish zeroed wheel joint states so TF is complete without a simulator.')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        model, use_rviz, use_jsp,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # The two drive joints are continuous. Without a simulator nothing moves them,
        # so this fills in zeroed joint states to complete the TF tree.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'description.rviz'])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
