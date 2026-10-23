from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([FindPackageShare('vins'), 'config', 'realsense', 'm3dgr.yaml'])),
        DeclareLaunchArgument('output_path', default_value=[EnvironmentVariable('HOME'), '/.ros/elevator_slam/vio']),
        DeclareLaunchArgument('image_sync_tolerance', default_value='0.003'),
        DeclareLaunchArgument('detector_wait_timeout', default_value='0.5'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='vins', executable='vins_node', name='vins_estimator', output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'output_path': LaunchConfiguration('output_path'),
                'image_sync_tolerance': LaunchConfiguration('image_sync_tolerance'),
                'detector_wait_timeout': LaunchConfiguration('detector_wait_timeout'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
