"""The magnet abstraction alone: /fire_resq/set_magnet (SetMagnet) -> a backend -> /fire_resq/magnet/state.

  ros2 launch fire_resq_control magnet.launch.py
  ros2 launch fire_resq_control magnet.launch.py use_sim_time:=true

Runs unchanged on the simulated and the real robot - it talks only to the two standard topics
magnet_backend.py names; which driver answers them (the simulation bridge today, the ESP32
bridge on hardware, Phase 12) is not this node's concern.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    return [Node(package='fire_resq_control', executable='magnet_node', name='magnet_node',
                 output='screen', parameters=[LaunchConfiguration('magnet_params').perform(context), params])]


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_control'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('magnet_params', default_value=str(share / 'config' / 'magnet.yaml')),
        OpaqueFunction(function=_setup),
    ])
