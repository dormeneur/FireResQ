"""The rescue mission alone: ros2 launch fire_resq_planning rescue.launch.py [use_sim_time:=true] [autostart:=true]

It needs the rest of the stack to be running (Nav2 under AMCL on a saved map, perception, the world model, cognition and the
magnet): `ros2 launch fire_resq_simulation arena_nav.launch.py ... rescue:=true` starts all of it. Runs unchanged on the
simulated and the real robot - it speaks only stable ROS interfaces.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
              'autostart': LaunchConfiguration('autostart').perform(context).lower() == 'true'}
    return [Node(package='fire_resq_planning', executable='rescue_node', name='rescue_node', output='screen',
                 parameters=[LaunchConfiguration('rescue_params').perform(context), params])]


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true', description='Start the mission at launch (false = wait for ExecuteRescue).'),
        DeclareLaunchArgument('rescue_params', default_value=str(share / 'config' / 'rescue.yaml')),
        OpaqueFunction(function=_setup),
    ])
