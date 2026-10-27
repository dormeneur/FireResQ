"""ROBOT SIDE (Raspberry Pi): the drivers, and only the drivers.

  ros2 launch fire_resq_hardware robot.launch.py
  ros2 launch fire_resq_hardware robot.launch.py base:=false          # camera only (e.g. while running motor_check)

Publishes /camera/image_raw/compressed, /camera/camera_info, /odom, TF odom->base_link, /fire_resq/ir_left,
/fire_resq/ir_right; consumes /cmd_vel. The wheel separation comes from the ROBOT DESCRIPTION, never restated here.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fire_resq_description.geometry import read_properties


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_hardware'))
    params = LaunchConfiguration('robot_params')
    return LaunchDescription([
        DeclareLaunchArgument('robot_params', default_value=str(share / 'config' / 'robot.yaml')),
        DeclareLaunchArgument('calibration_file', default_value=str(share / 'config' / 'camera_c270.yaml')),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('ir', default_value='true'),
        Node(package='fire_resq_hardware', executable='camera_node', name='camera_node', output='screen',
             parameters=[params, {'calibration_file': LaunchConfiguration('calibration_file')}],
             condition=IfCondition(LaunchConfiguration('camera'))),
        Node(package='fire_resq_hardware', executable='base_node', name='base_node', output='screen',
             parameters=[params, {'wheel_separation': float(read_properties()['wheel_separation'])}],
             condition=IfCondition(LaunchConfiguration('base'))),
        Node(package='fire_resq_hardware', executable='ir_node', name='ir_node', output='screen',
             parameters=[params], condition=IfCondition(LaunchConfiguration('ir'))),
    ])
