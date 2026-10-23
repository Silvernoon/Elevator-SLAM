from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import yaml


def _extrinsics_parameters(context):
    filename = LaunchConfiguration('extrinsics_file').perform(context)
    if not filename:
        return {}
    with open(filename, 'r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)
    if not isinstance(parameters, dict):
        raise RuntimeError('extrinsics_file must contain a YAML mapping of node parameters')

    def normalize(value):
        if isinstance(value, dict):
            return {key: normalize(item) for key, item in value.items()}
        if isinstance(value, list):
            if all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
                return [float(item) for item in value]
            return [normalize(item) for item in value]
        return value

    return normalize(parameters)


def _launch_node(context):
    return [Node(
        package='elevator_fusion', executable='trajectory_fusion',
        name='trajectory_fusion', output='screen',
        parameters=[LaunchConfiguration('config'), _extrinsics_parameters(context), {
            'enable_vio': LaunchConfiguration('enable_vio'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('elevator_fusion'), 'config', 'fusion.yaml'])),
        DeclareLaunchArgument('extrinsics_file', default_value=''),
        DeclareLaunchArgument('enable_vio', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_node),
    ])


