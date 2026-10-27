# Elevator-SLAM

将 [Ground-Fusion2 / Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2) 的视觉、融合与建图职责拆开，用 Elevator-LIO 替换上游内置 LIO。
上游源码基线为 `aa1c8dc80b05df7a536f5759d2f7725ad12015a6`。除已有 `src/lio` 外，模块都是本仓库直接管理的源码，**没有新增 submodule**。

## 模块边界

| 目录 | ROS 包 / 职责 |
| --- | --- |
| `src/lio` | `lio`；独立 LiDAR/IMU 估计、电梯状态与升降位移，保留已有 submodule 管理方式 |
| `src/vio` | `vins`；视觉/IMU/深度/轮速/线特征/GNSS 估计；检测器是独立构建与启动单元 |
| `src/camera_models` | 相机模型与标定支持 |
| `src/gnss_comm` | GNSS 消息、星历、坐标和时间工具 |
| `src/darknet_ros_msgs` | 检测消息与 action 定义 |
| `src/fusion` | `elevator_fusion`；几何退化判定、扫描时刻插值、轨迹源切换、融合点云坐标转换 |
| `src/meshing` | `im_mesh`；消费融合轨迹和世界点云的在线着色/三角网格，不再运行内置 LIO |
| `src/dense_map` | RGB-D 稠密建图、回环和位姿图；用同时间融合位姿校正 VIO 关键帧及路标 |
| `src/global_fusion` | 可选绝对位置位姿图；融合轨迹与外部 AprilTag 定位结果，不是第二套 LIO |
| `src/slam` | `elevator_slam`；系统编排、模块开关与 RViz 配置，不包含估计数学 |

默认链路：

```text
LiDAR + IMU -> lio -> /LIO/frame ---------> trajectory_fusion -> /fusion/odometry
                    /LIO/clouds_lidar --->                  -> /fusion/cloud_world
camera + IMU -> vins -> odometry -------->                  -> /fusion/cloud_body
                                                             |
                                                  mesh / RGB-D map
```

`elevator_fusion_core` 不依赖 ROS、PCL 或 VIO 私有状态。正常场景使用 LIO 的相邻刚体增量；几何退化时使用同步 VIO 的相邻刚体增量，避免把两个世界系的位置偏移与旋转混用。进入/退出切换不重新设置融合世界原点。

电梯模式保留 Elevator-LIO 的专用位移估计，不因轿厢点面几何退化而覆盖为相机 IMU 估计。VIO 缺失、超时、重置后尚未获得一对同步位姿时，继续使用 LIO，并在 `/fusion/status` 明确标记 `vio_unavailable` 或 `vio_unprimed`；不复用过期 VIO，不伪造协方差。

自有新增/重写代码采用 Google C++ 命名：类型 `UpperCamelCase`、函数 `PascalCase`、变量 `snake_case`、私有成员后缀 `_`、枚举值 `kPrefix`；文件名使用小写加下划线，并保持仓库的 `.cpp` 后缀。导入的上游估计器和第三方算法接口保留原名，避免无关重命名。

## 构建

系统入口目标为 **ROS 2 Foxy / Ubuntu 20.04**：10 个包使用 `ament_cmake` / `rosidl`，通过 Python launch 编排。`src/lio` 保留其已有 ROS 1/ROS 2 双版本实现；其余系统包不再提供 catkin 构建或 ROS 1 launch。Foxy 是本次选择的 ROS 2 目标；Noetic 需要把其余系统包改写为 ROS 1，本次不做该迁移。

```bash
# 已检出本仓库时，仅初始化已有 LIO 子模块。
git submodule update --init --recursive src/lio
source /opt/ros/foxy/setup.bash
# 先安装标准模板版 Sophus 和 Livox SDK2 / livox_ros_driver2，并 source 驱动工作空间。
# Docker 环境已包含这些依赖，见下方。
rosdep install --from-paths src --ignore-src --rosdistro foxy --skip-keys "sophus livox_ros_driver2" -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

依赖包含 Eigen 3.4、OpenCV 4（含 contrib 线特征接口）、PCL 1.12、Ceres 2.0、标准模板版 Sophus（`Sophus::Sophus` / `SO3d`，不是 no-template 分支）、fmt，以及 LIO 使用的 yaml-cpp、TBB、Qt5、nlohmann-json。网格模块还需要 CGAL、Boost；默认 headless，不需要 OpenGL 上下文。需要网格窗口时用 `colcon build --packages-select im_mesh --cmake-args -DIM_MESH_BUILD_GUI=ON` 构建额外的 PCL visualization/VTK 路径；YOLO 可用 `-DVINS_BUILD_YOLO=OFF` 从构建中关闭，CUDA 仅在显式 `-DVINS_YOLO_CUDA=ON` 时启用。CUDA 路径需要宿主机/容器另行提供匹配的 CUDA 工具链；CPU 镜像不包含 CUDA。

LIO 的 ROS 2 消息链接使用 Foxy 提供的 `rosidl_target_interfaces()`，同时连接节点、仿真节点和 RViz 插件。若构建报 `Unknown CMake command "rosidl_get_typesupport_target"`，请同步 `src/lio/CMakeLists.txt` 的对应修改；这不是 Livox 安装路径错误。在 Humble 上该接口会产生弃用警告，但不阻止构建。该兼容修改已通过 Humble 容器中的 LIO 构建、安装及 `lio/msg/LioFrame` 接口查询；不代表已完成 Foxy 实机构建验证。

**LIO 子模块保存注意：**本次新增扫描消息和质量输出位于 `src/lio` 内。父仓库的 gitlink 只能保存子仓库 commit，不能保存未提交的子仓库工作区修改。发布这些改动时，需要先在 LIO 仓库提交、推送，再在父仓库更新指针；否则新克隆缺少 `LioFrame.msg`，无法构建融合节点。不要用当前尚未更新的旧 gitlink 重新检出覆盖这些工作区改动。

### Docker 环境

`docker/Dockerfile.foxy` 安装 Foxy 系统依赖，并固定 Sophus `1.22.10`、Livox SDK2 `v1.3.1` 和驱动 `1.2.6`。Foxy 已停止维护；镜像面向 Ubuntu 20.04 / Focal，运行兼容性需按下方实际构建验证。镜像不包含项目源码、数据集或 YOLO 权重；源码挂载后在容器内构建。

```bash
docker build -f docker/Dockerfile.foxy -t elevator-slam:foxy .
docker run --rm -it --network host \
  --user "$(id -u):$(id -g)" -e HOME=/ws/.docker-home \
  -v "$PWD:/ws" elevator-slam:foxy bash
# 容器内：
source /opt/ros/foxy/setup.bash
source /opt/livox/setup.bash
mkdir -p "$HOME"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/elevator_fusion --output-on-failure
ctest --test-dir build/global_fusion --output-on-failure
```

网络受限时可传 `--build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`；代理使用 Docker 标准 `HTTP_PROXY` / `HTTPS_PROXY` build args，不写入镜像。Foxy 已 EOL，旧软件源或依赖可能不可用；需以 Docker 实际构建结果确认，不可沿用 Humble 构建验证结论。

运行真实数据时另挂载 bag、标定和设备配置目录；launch 参数中的路径必须是容器内路径。图形窗口需要显式传入 `DISPLAY` 并挂载 X11 socket；默认无 GUI，不要求 `--privileged`。

## 标定与启动

### 标定契约

不能用上游硬编码的重力方向旋转替代两个 IMU 之间的刚体标定。开启 VIO 融合时，必须提供实际传感器的 `imu_lio_T_imu_vio`；没有隐式单位阵默认值。

通过 `extrinsics_file:=/绝对路径/rig.yaml` 加载到各消费者的私有参数空间：

- `imu_lio_T_imu_vio`：7 元素 `[tx, ty, tz, qx, qy, qz, qw]`，把 VIO IMU 坐标变到 LIO IMU 坐标。单位米，四元数必须归一化。
- `imu_t_camera`：16 元素、row-major 的 `T_LIO_IMU_camera`，供相机着色和 RGB-D 建图。它不是单独的 VIO `body_T_cam0`；若标定只给出后者，需要先合成 `T_LIO_IMU_VIO_IMU * T_VIO_IMU_camera`。
- `camera_k`：着色网格需要的 9 元素、row-major 针孔内参矩阵；可放在共享标定 YAML 或 `mesh_config` 中。`camera_distortion` 可选；已矫正图像可省略。

不提供虚构的通用外参数值。上游相机/数据集 YAML 是配置参考，不能替代当前设备标定。

`extrinsics_file` 仍为普通 YAML 映射，不加 ROS 参数包装；launch 将其标定数组转为浮点参数。`fusion_config` / `mesh_config` 则使用 ROS 2 参数文件格式 `/**: {ros__parameters: ...}`。VIO 和 LIO 算法 YAML 保持原有 OpenCV / 自定义格式，不作为 ROS 参数文件直接加载。

### 完整系统

```bash
ros2 launch elevator_slam run.launch.py \
  extrinsics_file:=/绝对路径/rig.yaml \
  lio_config:=root_config.yaml \
  vio_config:=/绝对路径/camera_sensor.yaml \
  rviz:=true
# 另一个已 source 的终端：
ros2 bag play /绝对路径/sequence_ros2 --clock
```

`lio_config` 按现有 LIO 配置机制，相对 `lio/yaml/`；修改其根配置选择对应传感器、运行时和日志 YAML。雷达时间单位、IMU/LiDAR 外参、相机内参、深度编码与单位、话题名都必须与数据一致。默认 `root_config.yaml` 是现有 Livox 配置，不是对所有 M3DGR/其他设备通用的配置。

系统开关：

- `enable_vio:=false`：只用 Elevator-LIO 的扫描状态与同一融合输出接口；不要求跨 IMU 标定。
- `enable_mesh:=false`：不启动 LiDAR 网格。
- `enable_mesh_color:=false`：仅用融合世界点云生成几何网格，不需要相机标定或图像。
- `mesh_config:=/绝对路径/mapping.yaml`：选择网格参数及实际相机内参配置。
- `enable_dense_map:=true`：另启 RGB-D 地图/回环，默认关闭。
- `enable_global_fusion:=true`：另启绝对位置位姿图，默认关闭。
- `show_mesh_gui:=true`：开启网格交互显示（需构建时启用 `IM_MESH_BUILD_GUI`），默认 headless。窗口支持空格暂停、`w` 切换线框、`s` 保存 PLY；无窗口时可调用 `/meshing/save_mesh` 保存。
- `decompress_images:=false`：相机已经发布原始图像时关闭解压节点。
- `use_sim_time:=false`：连接实时传感器而非 `ros2 bag play --clock` 时使用；该参数显式传给每个节点。

仅运行 LIO 与融合输出、不启用相机和建图：

```bash
ros2 launch elevator_slam run.launch.py enable_vio:=false enable_mesh:=false \
  enable_dense_map:=false decompress_images:=false
```

单独运行 VIO：`ros2 launch vins vio.launch.py config_file:=/绝对路径/camera_sensor.yaml`。
YOLO 是独立可选节点：`ros2 launch vins yolo.launch.py weights_path:=/绝对路径/weights`；需真实网络权重且 VIO YAML 开启 `use_yolo`，不会在构建或启动时静默下载权重。`/darknet_ros/check_for_objects` 使用原生 ROS 2 action；并发目标拒绝，取消作用于对应目标。检测消息字段从 ROS 1 的 `Class` 改为 `class_id`；GNSS 消息的 `A/OMG0/M0/OMG_dot/A_dot/CN0/LLI` 分别改为 `a/omg0/m0/omg_dot/a_dot/cn0/lli`，外部发布器也需更新。

GNSS 在线对时按 `gnss_comm` 的 `T_SYS_*` 时钟编号处理（GPS 为 `0`，不是卫星系统位掩码 `SYS_*`），保留消息已序列化的绝对周历，再转换 UTC/BDT/GLONASS 时标；每条新鲜的下一 PPS 信息只与一个本地传感器触发配对。上游自动 `failureDetection()` 未启用的状态没有被包装成健康检测能力；显式重置使用 `/vins_estimator/restart` 的 `std_msgs/msg/Bool(true)`，并通过 reliable / transient-local 的 `restart_epoch` 通知消费者。

可选绝对位置图：`ros2 launch global_fusion global.launch.py`。输入 `/tag_Odometry` 的位置必须是配置的 `global_frame` 中的 **IMU 原点位置**，不是未经外参转换的标签位置；时间戳取真实测量时刻。异步测量通过相邻轨迹位置插值关联，不要求浮点时间戳恰好相等。该模块没有把未实现的 GPS 订阅当作 GNSS 支持；原始 GNSS 融合保留在 VIO 模块。单个位置锚点不约束全局旋转，不能将其视为完整全局姿态标定。

## 输出与时间语义

| 输出 | 契约 |
| --- | --- |
| `/LIO/frame` | 每个完成扫描的 scan-end stamp；位姿、最终匹配法向量 `sum(n*n^T)`、匹配数量、电梯标志是同一扫描的原子快照 |
| `/LIO/frame.imu_pose` | `world` 中的 IMU 位姿，电梯时包含 `state.z` |
| `/LIO/frame.cloud_imu_pose` | 生成当前输入世界点云时使用的原始 `q/p`，不额外叠加电梯子状态 |
| `/fusion/odometry` | `fusion_world -> fusion_imu`；位姿选择输出，不声称估计了额外的速度或协方差 |
| `/fusion/cloud_world` | 同一扫描点云转换到 `fusion_world`，保留原消息的其他点字段 |
| `/fusion/cloud_body` | 点云转换到 `fusion_imu`，**不是 LiDAR 坐标系** |
| `/fusion/status` | 实际轨迹源、退化/缺失原因、法向量奇异值、有效点数、电梯标志 |
| `/vins_estimator/restart_epoch` | VIO 重置代数；切断历史增量链，不把世界原点重置当作机器人运动 |

输入点云只与 **完全相同 ROS scan-end 时间戳** 的快照配对，允许点云先到或独立降频。VIO 只在两端存在且间隔不超过 `max_vio_gap` 时做线性位置 / 四元数球面插值，不外推。队列与位姿历史有界；超时/溢出会明确报告并丢弃无法配对的消息。

退化阈值默认沿用上游：有效法向量至少 11 个，最小奇异值至少 7、平均奇异值至少 10。由固定 3×3 Gram 矩阵求特征值，避免每帧分配 N×3 的 SVD 输入；这些量是匹配几何信息，**不是里程计协方差**。阈值仍需随雷达覆盖、采样率和有效匹配数量调整。

倒放/循环播放造成时间戳回退时，重启整条估计链；不能只让某个消费者继续使用旧世界系历史。

## 原生核心验证

不安装 ROS 也能构建、运行融合和可选位置图的行为回归：

```bash
cmake -S src/fusion -B build-core -DELEVATOR_FUSION_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build build-core -j2
ctest --test-dir build-core --output-on-failure

cmake -S src/global_fusion -B build-global -DGLOBAL_FUSION_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build build-global -j2
ctest --test-dir build-global --output-on-failure
```

融合回归覆盖几何秩亏、历史环形缓存/插值边界、旋转世界系下的 LIO/VIO 切换、电梯点云高度、VIO 中断和重置。位置图回归覆盖异步绝对位置关联、未来观测及相对运动保持。

此前 Humble 环境的构建与运行验证不构成 Foxy 验证。切换 Foxy 后，必须在 Foxy/Focal 环境重新构建并执行本节列出的运行检查；当前没有将旧版验证结果标记为 Foxy 通过。

这些合成输入 smoke 不等于实际传感器精度评估；尚未完成真实电梯数据集回放、GNSS 接收机在线融合或 CUDA 推理验证。ROS 1 bag 不能直接由此系统作为原生 ROS 2 消息消费，需先转换 bag 与自定义消息。

## 许可证

保留导入源码的原作者声明与各模块 LICENSE。上游仓库根许可证不能覆盖内部 VINS/Ground-Fusion 的 GPLv3 或 ImMesh/其他第三方源码声明；不将这些模块重新标为 MIT。使用与分发前应按各模块原始许可检查，尤其是第三方网格/图形组件。

额外限制：导入的 DBoW2 头文件声明 `CC-BY-NC-SA-3.0`，DUtils/DVision 声明 `LGPL-3.0-or-later`；ImMesh 原声明还包含个人/学术使用及商业用途需联系作者的条款。不能把整个工作空间视为统一 MIT/BSD 或默认允许商业使用。
