from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _make_node(context):
    parameters = [LaunchConfiguration("config_file")]
    extrinsics_file = context.perform_substitution(LaunchConfiguration("extrinsics_file"))
    if extrinsics_file:
        with open(os.path.expanduser(extrinsics_file), "r", encoding="utf-8") as stream:
            extrinsics = yaml.safe_load(stream) or {}
        if not isinstance(extrinsics, dict):
            raise RuntimeError("extrinsics_file must contain a plain YAML mapping")
        for name in ("imu_lio_T_imu_vio", "imu_t_camera", "camera_k", "camera_distortion"):
            if name in extrinsics:
                extrinsics[name] = [float(value) for value in extrinsics[name]]
        parameters.append(extrinsics)
    camera_transform = yaml.safe_load(context.perform_substitution(LaunchConfiguration("imu_t_camera")))
    if camera_transform:
        parameters.append({"imu_t_camera": [float(value) for value in camera_transform]})
    parameters.append({
        "show_gui": LaunchConfiguration("show_gui"),
        "color_enabled": LaunchConfiguration("color_enabled"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "cloud_topic": LaunchConfiguration("cloud_topic"),
        "odom_topic": LaunchConfiguration("odom_topic"),
        "image_topic": LaunchConfiguration("image_topic"),
        "output_mesh": LaunchConfiguration("output_mesh"),
    })
    return [Node(package="im_mesh", executable="external_mapping", name="meshing", output="screen", parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(get_package_share_directory("im_mesh"), "config", "mapping.yaml")),
        DeclareLaunchArgument("extrinsics_file", default_value=""),
        DeclareLaunchArgument("imu_t_camera", default_value="[]"),
        DeclareLaunchArgument("color_enabled", default_value="true"),
        DeclareLaunchArgument("show_gui", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/fusion/cloud_world"),
        DeclareLaunchArgument("odom_topic", default_value="/fusion/odometry"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("output_mesh", default_value=os.path.join(os.path.expanduser("~"), ".ros", "fusion_mesh.ply")),
        OpaqueFunction(function=_make_node),
    ])
