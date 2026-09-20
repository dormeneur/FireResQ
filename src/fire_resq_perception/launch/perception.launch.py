"""Perception: camera stream -> /fire_resq/detections.

  ros2 launch fire_resq_perception perception.launch.py
  ros2 launch fire_resq_perception perception.launch.py target_frame:=odom spatial_backend:=known_height

Runs unchanged on the simulated robot and on hardware: it needs only the camera topics and TF.
The floor height (how far below base_link the floor is) is read from the ROBOT DESCRIPTION here,
so it is never restated in a perception file.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fire_resq_description.geometry import read_properties


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params = {
        'use_sim_time': use_sim_time,
        'target_frame': LaunchConfiguration('target_frame').perform(context),
        'spatial_backend': LaunchConfiguration('spatial_backend').perform(context),
        'known_height_anchor': LaunchConfiguration('known_height_anchor').perform(context),
        'floor_offset_m': float(read_properties()['wheel_radius']),      # the wheel radius: base_link is the axle
    }
    return [Node(package='fire_resq_perception', executable='perception_node', name='perception_node',
                 output='screen', parameters=[LaunchConfiguration('perception_params').perform(context), params])]


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target_frame', default_value='map', description='Frame the positions are published in.'),
        DeclareLaunchArgument('spatial_backend', default_value='auto', description='auto | depth | known_height'),
        DeclareLaunchArgument('known_height_anchor', default_value='top', description='top | bottom'),
        DeclareLaunchArgument('perception_params', default_value=str(share / 'config' / 'perception.yaml')),
        OpaqueFunction(function=_setup),
    ])
