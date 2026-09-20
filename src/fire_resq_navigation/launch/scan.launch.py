"""depth image -> /scan.  Sensor-agnostic: any depth source that publishes a depth image and its
CameraInfo (the Gazebo depth camera, a RealSense driver) feeds it through remappable topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('fire_resq_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('depth_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_params', default_value=PathJoinSubstitution([pkg, 'config', 'depth_to_scan.yaml'])),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[LaunchConfiguration('scan_params'),
                        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            remappings=[('depth', LaunchConfiguration('depth_topic')),
                        ('depth_camera_info', LaunchConfiguration('depth_info_topic')),
                        ('scan', LaunchConfiguration('scan_topic'))],
        ),
    ])
