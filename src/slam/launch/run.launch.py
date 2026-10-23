"""Compose the native Humble estimation and mapping pipeline."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def config(name):
        return LaunchConfiguration(name)

    def resource(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    def include(package, filename, arguments, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(resource(package, "launch", filename)),
            launch_arguments=arguments.items(), condition=condition,
        )

    defaults = {
        "lio_config": "root_config.yaml",
        "vio_config": resource("vins", "config", "realsense", "m3dgr.yaml"),
        "fusion_config": resource("elevator_fusion", "config", "fusion.yaml"),
        "mesh_config": resource("im_mesh", "config", "mapping.yaml"),
        "extrinsics_file": "",
        "enable_vio": "true",
        "enable_mesh": "true",
        "enable_mesh_color": "true",
        "enable_dense_map": "false",
        "enable_global_fusion": "false",
        "decompress_images": "true",
        "rviz": "false",
        "show_mesh_gui": "false",
        "use_sim_time": "true",
    }
    clock = {"use_sim_time": ParameterValue(config("use_sim_time"), value_type=bool)}
    lio = Node(
        package="lio", executable="lio", name="lio", output="screen",
        parameters=[clock, {"config_path": config("lio_config")}],
    )
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    actions += [
        RegisterEventHandler(OnProcessExit(
            target_action=lio,
            on_exit=[EmitEvent(event=Shutdown(reason="LIO exited; stop the estimation chain"))],
        )),
        lio,
    ]
    for name, transport, topic in (
        ("color_decompress", "compressed", "/camera/color/image_raw"),
        ("depth_decompress", "compressedDepth", "/camera/aligned_depth_to_color/image_raw"),
    ):
        actions.append(Node(
            package="image_transport", executable="republish", name=name, output="screen",
            arguments=[transport, "raw"], parameters=[clock],
            remappings=[("in", topic), ("out", topic)],
            condition=IfCondition(config("decompress_images")),
        ))
    actions += [
        include("vins", "vio.launch.py", {
            "config_file": config("vio_config"), "use_sim_time": config("use_sim_time"),
        }, IfCondition(config("enable_vio"))),
        include("elevator_fusion", "fusion.launch.py", {
            "config": config("fusion_config"), "extrinsics_file": config("extrinsics_file"),
            "enable_vio": config("enable_vio"), "use_sim_time": config("use_sim_time"),
        }),
        include("im_mesh", "mapping.launch.py", {
            "config_file": config("mesh_config"), "extrinsics_file": config("extrinsics_file"),
            "show_gui": config("show_mesh_gui"), "color_enabled": config("enable_mesh_color"),
            "use_sim_time": config("use_sim_time"),
        }, IfCondition(config("enable_mesh"))),
        include("dense_map", "mapping.launch.py", {
            "config_file": config("vio_config"), "extrinsics_file": config("extrinsics_file"),
            "use_sim_time": config("use_sim_time"),
        }, IfCondition(config("enable_dense_map"))),
        include("global_fusion", "global.launch.py", {
            "use_sim_time": config("use_sim_time"),
        }, IfCondition(config("enable_global_fusion"))),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            parameters=[clock], arguments=["-d", resource("elevator_slam", "config", "fusion.rviz")],
            condition=IfCondition(config("rviz")),
        ),
    ]
    return LaunchDescription(actions)
