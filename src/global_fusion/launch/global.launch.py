from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odometry_topic = LaunchConfiguration('odometry_topic')
    position_topic = LaunchConfiguration('position_topic')
    return LaunchDescription([
        DeclareLaunchArgument('odometry_topic', default_value='/fusion/odometry'),
        DeclareLaunchArgument('position_topic', default_value='/tag_Odometry'),
        DeclareLaunchArgument('global_frame', default_value='global_world'),
        DeclareLaunchArgument('position_sigma', default_value='0.1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='global_fusion',
            executable='global_fusion_node',
            name='global_fusion',
            output='screen',
            parameters=[{
                'global_frame': LaunchConfiguration('global_frame'),
                'position_sigma': LaunchConfiguration('position_sigma'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('/fusion/odometry', odometry_topic), ('/tag_Odometry', position_topic)],
        ),
    ])
