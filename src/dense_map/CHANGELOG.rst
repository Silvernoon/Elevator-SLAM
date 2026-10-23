dense_map integration history
=============================

0.1.0
-----

Imported the RGB-D keyframe mapper, BRIEF/DBoW loop detector, 4-DoF/6-DoF
Ceres pose graph and graph/descriptor/dense-point persistence from Ground-Fusion++
revision aa1c8dc80b05df7a536f5759d2f7725ad12015a6. The actual binary
brief_k10L6.bin vocabulary and its brief_pattern.yml are installed in resources/.
The existing camera_models ament package supplies camodocal; no second camera
implementation or LIO source import is required.

Launch contract
~~~~~~~~~~~~~~~

ros2 launch dense_map mapping.launch.py config_file:=/absolute/path/to/vio_config.yaml
extrinsics_file:=/absolute/path/to/extrinsics.yaml

The executable is dense_map_node and the node/private namespace is /dense_map.
config_file is required and has no guessed calibration default. cam0_calib is
resolved relative to that VIO YAML, or may be replaced with private camera_file.
The shared extrinsics_file is optional as a launch argument, but calibrated
extrinsic parameters are mandatory. It is loaded into the node's private
namespace before optional explicit imu_t_camera launch override.
The launch accepts use_sim_time (default false), loads plain shared calibration
YAML as a parameter dictionary, and normalizes numeric calibration arrays to
double precision. OpenCV algorithm YAML is passed as config_file, not interpreted
as a ROS parameter file. Odometry is reliable; camera, depth and feature streams
use SensorDataQoS. A single-threaded executor preserves callback serialization.

Shared calibration parameter conventions:

* imu_lio_T_imu_vio: [x, y, z, qx, qy, qz, qw], mapping VIO IMU coordinates
  into physical LIO IMU coordinates.
* imu_t_camera: 16 row-major numbers giving physical LIO_IMU_T_camera, NOT
  the original VIO body_T_cam0.

The original body_T_cam0 remains VIO_IMU_T_camera. If only the inter-IMU
transform is supplied, it is composed with body_T_cam0. If only an explicit
physical imu_t_camera is supplied, body_T_cam0 is also required, and the
inter-IMU transform is derived as T_lio_camera * inverse(T_vio_camera).
With neither route available the node fails at startup, rather than silently
assuming the two IMUs are identical. If static camera calibration is unavailable,
an explicit inter-IMU transform plus extrinsic_topic may supply it at runtime.

Default input topics (all configurable via same-named private parameters and
launch arguments):

* pose_topic: /fusion/odometry, nav_msgs/Odometry; fusion_world -> fusion_imu.
* image_topic: /camera/color/image_raw, sensor_msgs/Image.
* depth_topic: /camera/aligned_depth_to_color/image_raw, sensor_msgs/Image.
* keyframe_pose_topic: /vins_estimator/keyframe_pose, original VIO IMU pose.
* keyframe_point_topic: /vins_estimator/keyframe_point, original VIO world
  landmarks and original [nx, ny, u, v, id] per-point channels.
* extrinsic_topic: /vins_estimator/extrinsic, original VIO_IMU_T_camera.
  Set to an empty string to use only static calibration. Timestamped estimates
  are composed with the fixed inter-IMU transform, never interpreted as LIO axes.

The graph body frame is the physical LIO IMU. At keyframe timestamp t, fusion
translation is interpolated linearly and orientation by quaternion slerp.
The graph receives T_fusionWorld_lio(t); BEFORE insertion each VIO landmark is
transformed by C(t) = T_fusionWorld_lio(t) * T_lio_vio * inverse(T_vioWorld_vio(t)).
The corresponding camera extrinsic is T_lio_vio * T_vio_camera. Camera pixel and
normalized coordinates remain unchanged. Each keyframe retains its own camera
extrinsic for PnP, dense map rebuilding, save and reload; changing an estimated
extrinsic does not move older RGB-D samples.

Synchronization uses bounded timestamp-indexed buffers, not FIFO pairing.
Images, depth and feature points must match the keyframe within sync_tolerance
(default 0.01 seconds). Fusion poses must exactly match or bracket the timestamp
with a gap no greater than max_pose_gap (default 0.10 seconds); no extrapolation.
Extrinsic estimates use the newest sample no later than the keyframe timestamp,
with static calibrated fallback where available. buffer_duration (5 seconds)
and buffer_size (300 entries per stream) bound retention, including pending
keyframes. new_sequence clears all pending streams for a new timeline.

Outputs and persistence
~~~~~~~~~~~~~~~~~~~~~~~

Private output topics retain the upstream graph interface: octree (colored
PointCloud2), pose_graph_path, base_path, path_1 through path_9, pose_graph
markers, match_image (when save_image is enabled), and point_cloud_loop_rect
(timestamp- and loop-corrected fusion-world landmarks). Outputs use frame_id
(default fusion_world); these mapping outputs include graph loop correction.
The node publishes no TF and never replaces fusion's raw odometry or its TF.
Fusion input additionally checks fusion_child_frame (fusion_imu).
Graph loop closure remains enabled for every admitted keyframe. use_imu=true
selects gravity-aligned 4-DoF graph optimization; false selects 6-DoF.

No stdin or GUI interaction is required. Call /dense_map/save_map (std_srvs/Trigger)
to save graph state and the colored PCD, or /dense_map/new_sequence to start a
new sequence (at most 9). save_on_shutdown defaults to true. Optimization is
joined before shutdown persistence; save operations serialize with graph updates.
load_previous_pose_graph defaults to false. The graph saves descriptors,
normalized keypoints, optional images, RGB-D camera-frame points and a required
per-keyframe _extrinsic.txt sidecar; reload consumes these calibration sidecars.
Old upstream graph directories without sidecars are not silently interpreted
under a different physical IMU calibration.

output_path defaults to ~/.ros/dense_map in mapping.launch.py. pose_graph_save_path
defaults to output_path/pose_graph; cloud_save_path defaults to output_path/map.pcd;
the loop-corrected trajectory is output_path/fusion_loop.txt. vocabulary_file and
brief_pattern_file default to package-installed resources and can be overridden
as private parameters. ROS remapping can rename any private output topic.

Depth accepts registered 16UC1 (depth_scale=0.001 meters/unit) or 32FC1 (meters).
The calibration and RGB/depth dimensions must agree. Other private processing
parameters are depth_enabled=true, depth_stride=10, depth_boundary=10,
min_depth=0.1, max_depth=7.0, skip_distance=0, skip_frames=0, equalize=false,
and save_image=0. Depth sampling unprojects through the camera model and divides
by ray.z(), rather than assuming unit-z output from every camera model.

Dependencies and licensing
~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 Humble dependencies: ament_index_cpp, rclcpp, std_msgs, std_srvs,
geometry_msgs, nav_msgs, sensor_msgs, visualization_msgs, camera_models, cv_bridge,
pcl_conversions. System dependencies: OpenCV 4 (core/imgproc/imgcodecs/features2d/
calib3d), Ceres, Eigen3, Boost filesystem/system, PCL common/io/octree/filters,
and Threads. CMake flags, include paths and linkage are target scoped.
No octomap dependency is needed: this mapper uses PCL's octree.

The imported VINS code and resources retain the upstream GPLv3 LICENSE and
source notices. DUtils and DVision headers explicitly retain LGPLv3-or-later
notices. The older bundled DBoW2.h explicitly retains CC-BY-NC-SA-3.0 notices,
including its noncommercial restriction; do not substitute the license of a
newer, different DBoW2 release. License texts are included beside each bundled
library and installed under the package's licenses directory.
