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

当前系统入口面向 **ROS 1 Noetic / Ubuntu 20.04**。`lio` 自身保留 ROS 2 支持，但这里的 VIO、融合和建图尚未移植到 ROS 2，不能将整个工作空间直接用 colcon 当作 ROS 2 系统构建。

```bash
# 已检出本仓库时，仅初始化已有 LIO 子模块。
git submodule update --init --recursive src/lio
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

依赖包含 Eigen、OpenCV 4（含 contrib 线特征接口）、PCL、Ceres（上游基线 1.14）、标准模板版 Sophus（`Sophus::Sophus` / `SO3d`，不是 no-template 分支）、fmt，以及 LIO 使用的 yaml-cpp、TBB、Qt5、nlohmann-json。网格模块还需要 CGAL、Boost；默认 headless，不需要 OpenGL 上下文。需要网格窗口时用 `catkin_make -DIM_MESH_BUILD_GUI=ON` 构建额外的 PCL visualization/VTK 路径；YOLO 可用 `-DVINS_BUILD_YOLO=OFF` 从构建中关闭，CUDA 仅在显式 `-DVINS_YOLO_CUDA=ON` 时启用。`rosdep` 无法安装的第三方库需先安装到 CMake 可发现的前缀，不再依赖开发机 `/usr/local/lib/*.so` 绝对路径。

**LIO 子模块保存注意：**本次新增扫描消息和质量输出位于 `src/lio` 内。父仓库的 gitlink 只能保存子仓库 commit，不能保存未提交的子仓库工作区修改。发布这些改动时，需要先在 LIO 仓库提交、推送，再在父仓库更新指针；否则新克隆缺少 `LioFrame.msg`，无法构建融合节点。不要用当前尚未更新的旧 gitlink 重新检出覆盖这些工作区改动。

## 标定与启动

### 标定契约

不能用上游硬编码的重力方向旋转替代两个 IMU 之间的刚体标定。开启 VIO 融合时，必须提供实际传感器的 `imu_lio_T_imu_vio`；没有隐式单位阵默认值。

通过 `extrinsics_file:=/绝对路径/rig.yaml` 加载到各消费者的私有参数空间：

- `imu_lio_T_imu_vio`：7 元素 `[tx, ty, tz, qx, qy, qz, qw]`，把 VIO IMU 坐标变到 LIO IMU 坐标。单位米，四元数必须归一化。
- `imu_t_camera`：16 元素、row-major 的 `T_LIO_IMU_camera`，供相机着色和 RGB-D 建图。它不是单独的 VIO `body_T_cam0`；若标定只给出后者，需要先合成 `T_LIO_IMU_VIO_IMU * T_VIO_IMU_camera`。
- `camera_k`：着色网格需要的 9 元素、row-major 针孔内参矩阵；可放在共享标定 YAML 或 `mesh_config` 中。`camera_distortion` 可选；已矫正图像可省略。

不提供虚构的通用外参数值。上游相机/数据集 YAML 是配置参考，不能替代当前设备标定。

### 完整系统

```bash
roslaunch elevator_slam run.launch \
  extrinsics_file:=/绝对路径/rig.yaml \
  lio_config:=root_config.yaml \
  vio_config:=/绝对路径/camera_sensor.yaml \
  rviz:=true
# 另一个已 source 的终端：
rosbag play --clock /绝对路径/sequence.bag
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
- `use_sim_time:=false`：连接实时传感器而非 `rosbag --clock` 时使用。

仅运行 LIO 与融合输出、不启用相机和建图：

```bash
roslaunch elevator_slam run.launch enable_vio:=false enable_mesh:=false \
  enable_dense_map:=false decompress_images:=false
```

单独运行 VIO：`roslaunch vins vio.launch config_file:=/绝对路径/camera_sensor.yaml`。
YOLO 是独立可选节点，需真实网络权重且 VIO YAML 开启 `use_yolo`；不会在构建或启动时静默下载权重。

GNSS 在线对时按 `gnss_comm` 的 `T_SYS_*` 时钟编号处理（GPS 为 `0`，不是卫星系统位掩码 `SYS_*`），保留消息已序列化的绝对周历，再转换 UTC/BDT/GLONASS 时标；每条新鲜的下一 PPS 信息只与一个本地传感器触发配对。上游自动 `failureDetection()` 未启用的状态没有被包装成健康检测能力；显式重置使用 `/vins_estimator/restart` 的 `std_msgs/Bool(true)`，并通过 `restart_epoch` 通知消费者。

可选绝对位置图：`roslaunch global_fusion global.launch`。输入 `/tag_Odometry` 的位置必须是配置的 `global_frame` 中的 **IMU 原点位置**，不是未经外参转换的标签位置；时间戳取真实测量时刻。异步测量通过相邻轨迹位置插值关联，不要求浮点时间戳恰好相等。该模块没有把未实现的 GPS 订阅当作 GNSS 支持；原始 GNSS 融合保留在 VIO 模块。

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

本次集成已在非 ROS 环境用 Eigen 3.3.7、Ceres 1.14 实际编译两个核心并通过上述行为回归；另用逐扫描数据流消费者运行了 LIO→VIO→电梯→退出提交→VIO 重置/恢复，观察到融合位置连续、电梯点云高度只加一次、外部位置图继承正确偏移。当前环境没有 ROS，**尚未验证整个 catkin 工作空间编译、rosbag 实跑或 RViz/网格图形显示**；原生核心 smoke 不能替代这些系统验证。

命名迁移后已重新编译并通过两组核心回归；另外完成 10 个 ROS 包的清单/依赖排序与 8 个 launch 的 XML/本地资源解析。GNSS 时钟转换也已用真实 `gnss_comm` 数学实现独立运行，覆盖 GPS 编号 `0`、Galileo 周历保持、BDT/UTC/GLONASS 时差与未知时标拒绝；这不代表已验证 GNSS 接收机和 ROS 触发链路。

## 许可证

保留导入源码的原作者声明与各模块 LICENSE。上游仓库根许可证不能覆盖内部 VINS/Ground-Fusion 的 GPLv3 或 ImMesh/其他第三方源码声明；不将这些模块重新标为 MIT。使用与分发前应按各模块原始许可检查，尤其是第三方网格/图形组件。

额外限制：导入的 DBoW2 头文件声明 `CC-BY-NC-SA-3.0`，DUtils/DVision 声明 `LGPL-3.0-or-later`；ImMesh 原声明还包含个人/学术使用及商业用途需联系作者的条款。不能把整个工作空间视为统一 MIT/BSD 或默认允许商业使用。
