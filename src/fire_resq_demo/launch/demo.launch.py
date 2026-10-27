"""The bottle demo. Use ./demo.sh (repository root), which also picks the right ROS graph:

  ros2 launch fire_resq_demo demo.launch.py                 real robot (the Pi must be running its drivers)
  ros2 launch fire_resq_demo demo.launch.py drive:=false    dry run: sees and plans, never commands the motors
  ros2 launch fire_resq_demo demo.launch.py rehearsal:=true a pretend robot and room. ONLY in the simulation graph
                                                            (FIRE_RESQ_SIM=1 docker/station.sh): see fake_robot_node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='true'),
        DeclareLaunchArgument('rehearsal', default_value='false'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('settings_file', default_value='', description='where the dashboard keeps its tuning'),
        Node(package='fire_resq_demo', executable='demo_node', name='bottle_demo', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('fire_resq_demo'), 'config', 'demo.yaml']),
                         {'drive': ParameterValue(LaunchConfiguration('drive'), value_type=bool),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          'settings_file': ParameterValue(LaunchConfiguration('settings_file'), value_type=str)}]),
        Node(package='fire_resq_demo', executable='fake_robot_node', name='fake_robot', output='screen',
             condition=IfCondition(LaunchConfiguration('rehearsal'))),
    ])
