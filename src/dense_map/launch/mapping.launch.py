from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _normalize_calibration(value):
    """Keep calibration arrays compatible with ROS 2's homogeneous parameter type."""
    if isinstance(value, dict):
        return {key: _normalize_calibration(item) for key, item in value.items()}
    if isinstance(value, list):
        return [float(item) if isinstance(item, (int, float)) and not isinstance(item, bool)
                else _normalize_calibration(item) for item in value]
    return value


def _make_node(context):
    parameters = []
    config_file = context.perform_substitution(LaunchConfiguration("config_file"))
    parameters.append({"config_file": os.path.expanduser(config_file)})

    extrinsics_file = context.perform_substitution(LaunchConfiguration("extrinsics_file"))
    if extrinsics_file:
        with open(os.path.expanduser(extrinsics_file), "r", encoding="utf-8") as stream:
            extrinsics = yaml.safe_load(stream) or {}
        if not isinstance(extrinsics, dict):
            raise RuntimeError("extrinsics_file must contain a plain YAML mapping")
        parameters.append(_normalize_calibration(extrinsics))

    parameters.append({
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        "pose_topic": LaunchConfiguration("pose_topic"),
        "image_topic": LaunchConfiguration("image_topic"),
        "depth_topic": LaunchConfiguration("depth_topic"),
        "keyframe_pose_topic": LaunchConfiguration("keyframe_pose_topic"),
        "keyframe_point_topic": LaunchConfiguration("keyframe_point_topic"),
        "extrinsic_topic": ParameterValue(LaunchConfiguration("extrinsic_topic"), value_type=str),
        "output_path": LaunchConfiguration("output_path"),
        "pose_graph_save_path": LaunchConfiguration("pose_graph_save_path"),
        "cloud_save_path": LaunchConfiguration("cloud_save_path"),
        "load_previous_pose_graph": ParameterValue(LaunchConfiguration("load_previous_pose_graph"), value_type=bool),
        "save_on_shutdown": ParameterValue(LaunchConfiguration("save_on_shutdown"), value_type=bool),
        "frame_id": LaunchConfiguration("frame_id"),
        "fusion_child_frame": LaunchConfiguration("fusion_child_frame"),
        "sync_tolerance": ParameterValue(LaunchConfiguration("sync_tolerance"), value_type=float),
        "max_pose_gap": ParameterValue(LaunchConfiguration("max_pose_gap"), value_type=float),
        "buffer_duration": ParameterValue(LaunchConfiguration("buffer_duration"), value_type=float),
        "buffer_size": ParameterValue(LaunchConfiguration("buffer_size"), value_type=int),
        "depth_enabled": ParameterValue(LaunchConfiguration("depth_enabled"), value_type=bool),
        "depth_scale": ParameterValue(LaunchConfiguration("depth_scale"), value_type=float),
        "depth_stride": ParameterValue(LaunchConfiguration("depth_stride"), value_type=int),
        "depth_boundary": ParameterValue(LaunchConfiguration("depth_boundary"), value_type=int),
        "min_depth": ParameterValue(LaunchConfiguration("min_depth"), value_type=float),
        "max_depth": ParameterValue(LaunchConfiguration("max_depth"), value_type=float),
        "skip_distance": ParameterValue(LaunchConfiguration("skip_distance"), value_type=float),
        "skip_frames": ParameterValue(LaunchConfiguration("skip_frames"), value_type=int),
        "save_image": ParameterValue(LaunchConfiguration("save_image"), value_type=int),
        "equalize": ParameterValue(LaunchConfiguration("equalize"), value_type=bool),
        "use_imu": ParameterValue(LaunchConfiguration("use_imu"), value_type=bool),
        "vocabulary_file": LaunchConfiguration("vocabulary_file"),
        "brief_pattern_file": LaunchConfiguration("brief_pattern_file"),
    })

    imu_t_camera = context.perform_substitution(LaunchConfiguration("imu_t_camera"))
    if imu_t_camera and imu_t_camera != "[]":
        values = yaml.safe_load(imu_t_camera)
        if not isinstance(values, list):
            raise RuntimeError("imu_t_camera must be a YAML list")
        parameters.append({"imu_t_camera": _normalize_calibration(values)})

    return [Node(
        package="dense_map",
        executable="dense_map_node",
        name="dense_map",
        output="screen",
        parameters=parameters,
    )]


def generate_launch_description():
    home = os.path.join(os.path.expanduser("~"), ".ros", "dense_map")
    resources = os.path.join(get_package_share_directory("dense_map"), "resources")
    return LaunchDescription([
        DeclareLaunchArgument("config_file"),
        DeclareLaunchArgument("extrinsics_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("pose_topic", default_value="/fusion/odometry"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("keyframe_pose_topic", default_value="/vins_estimator/keyframe_pose"),
        DeclareLaunchArgument("keyframe_point_topic", default_value="/vins_estimator/keyframe_point"),
        DeclareLaunchArgument("extrinsic_topic", default_value="/vins_estimator/extrinsic"),
        DeclareLaunchArgument("imu_t_camera", default_value="[]"),
        DeclareLaunchArgument("output_path", default_value=home),
        DeclareLaunchArgument("pose_graph_save_path", default_value=[LaunchConfiguration("output_path"), "/pose_graph"]),
        DeclareLaunchArgument("cloud_save_path", default_value=[LaunchConfiguration("output_path"), "/map.pcd"]),
        DeclareLaunchArgument("vocabulary_file", default_value=os.path.join(resources, "brief_k10L6.bin")),
        DeclareLaunchArgument("brief_pattern_file", default_value=os.path.join(resources, "brief_pattern.yml")), 
        DeclareLaunchArgument("load_previous_pose_graph", default_value="false"),
        DeclareLaunchArgument("save_on_shutdown", default_value="true"),
        DeclareLaunchArgument("frame_id", default_value="fusion_world"),
        DeclareLaunchArgument("fusion_child_frame", default_value="fusion_imu"),
        DeclareLaunchArgument("sync_tolerance", default_value="0.01"),
        DeclareLaunchArgument("max_pose_gap", default_value="0.10"),
        DeclareLaunchArgument("buffer_duration", default_value="5.0"),
        DeclareLaunchArgument("buffer_size", default_value="300"),
        DeclareLaunchArgument("depth_enabled", default_value="true"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument("depth_stride", default_value="10"),
        DeclareLaunchArgument("depth_boundary", default_value="10"),
        DeclareLaunchArgument("min_depth", default_value="0.1"),
        DeclareLaunchArgument("max_depth", default_value="7.0"),
        DeclareLaunchArgument("skip_distance", default_value="0.0"),
        DeclareLaunchArgument("skip_frames", default_value="0"),
        DeclareLaunchArgument("save_image", default_value="0"),
        DeclareLaunchArgument("equalize", default_value="false"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        OpaqueFunction(function=_make_node),
    ])
