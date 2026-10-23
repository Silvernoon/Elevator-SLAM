from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flatten(values, prefix=""):
    flattened = {}
    for key, value in values.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flattened.update(_flatten(value, name))
        else:
            flattened[name] = value
    return flattened


def _load_parameters(path):
    values = yaml.safe_load(path.read_text()) or {}
    if "/**" in values:
        values = values["/**"].get("ros__parameters", {})
    return _flatten(values)


def _launch_detector(context):
    package_share = Path(get_package_share_directory("vins"))
    network_config = Path(LaunchConfiguration("network_param_file").perform(context))
    topic_config = Path(LaunchConfiguration("topic_param_file").perform(context))
    parameters = _load_parameters(network_config)
    parameters.update(_load_parameters(topic_config))
    parameters.update({
        "weights_path": LaunchConfiguration("weights_path").perform(context),
        "config_path": LaunchConfiguration("config_path").perform(context),
        "yolo_model.weight_file.name": LaunchConfiguration("weights_file").perform(context),
        "yolo_model.config_file.name": LaunchConfiguration("config_file").perform(context),
        "subscribers.camera_reading.topic": LaunchConfiguration("image_topic").perform(context),
        "image_view.enable_opencv": LaunchConfiguration("view_image").perform(context).lower() == "true",
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true",
    })
    return [Node(
        package="vins",
        executable="darknet_ros",
        name="darknet_ros",
        output="screen",
        parameters=[parameters],
        emulate_tty=True,
    )]


def generate_launch_description():
    share = Path(get_package_share_directory("vins"))
    return LaunchDescription([
        DeclareLaunchArgument("weights_path", description="Directory containing the required trained YOLO weights"),
        DeclareLaunchArgument("weights_file", default_value="yolov3.weights"),
        DeclareLaunchArgument("config_file", default_value="yolov3.cfg"),
        DeclareLaunchArgument("config_path", default_value=str(share / "yolo_network_config" / "cfg")),
        DeclareLaunchArgument("network_param_file", default_value=str(share / "config" / "yolov3.yaml")),
        DeclareLaunchArgument("topic_param_file", default_value=str(share / "config" / "ros.yaml")),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("view_image", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_launch_detector),
    ])
