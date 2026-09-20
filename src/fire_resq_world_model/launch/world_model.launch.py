"""World model: /fire_resq/detections -> /fire_resq/world_state (+ RViz markers, + the UpdateVictimStatus service).

  ros2 launch fire_resq_world_model world_model.launch.py
  ros2 launch fire_resq_world_model world_model.launch.py use_sim_time:=true world_frame:=odom

Runs unchanged on the simulated and the real robot: it needs only detections and TF.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
              'world_frame': LaunchConfiguration('world_frame').perform(context)}
    return [Node(package='fire_resq_world_model', executable='world_model_node', name='world_model_node',
                 output='screen', parameters=[LaunchConfiguration('world_model_params').perform(context), params])]


def generate_launch_description():
    share = Path(get_package_share_directory('fire_resq_world_model'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world_frame', default_value='map', description='Frame WorldState is expressed in.'),
        DeclareLaunchArgument('world_model_params', default_value=str(share / 'config' / 'world_model.yaml')),
        OpaqueFunction(function=_setup),
    ])
