"""STATION SIDE (PC): what the real robot needs from the PC today - decoding, the TF tree, perception.

  ros2 launch fire_resq_hardware station.launch.py
  ros2 launch fire_resq_hardware station.launch.py perception:=false

The robot must be running launch/robot.launch.py. Positions are published in `odom` (dead reckoning, see
drive.py) until the robot has a localisation source that provides `map`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('target_frame', default_value='odom'),
        Node(package='fire_resq_hardware', executable='image_decoder_node', name='image_decoder_node', output='screen'),
        # robot_state_publisher: base_link -> camera frames. Zeroed wheel joints complete the tree (no encoders).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('fire_resq_description'), 'launch', 'display.launch.py'])),
            launch_arguments={'use_rviz': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('fire_resq_perception'), 'launch', 'perception.launch.py'])),
            launch_arguments={'target_frame': LaunchConfiguration('target_frame'),
                              'spatial_backend': 'known_height'}.items(),
            condition=IfCondition(LaunchConfiguration('perception'))),
    ])
