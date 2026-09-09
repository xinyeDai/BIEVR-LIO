<p align="center">
  <img width=400 src="doc/bievr_final.svg">
</p>


# BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps

<p align="center">
<a href="https://patripfr.github.io/bievr-lio/"><img src="https://shieldcn.dev/badge/Project-Page-gray?size=xs" alt="Project Page" /></a>
<a href="https://arxiv.org/abs/2604.14421"><img src="https://shieldcn.dev/badge/arXiv-2604.14421-b31b1b?logo=arxiv&size=xs" alt="arXiv" /></a>
<a href="https://arxiv.org/pdf/2604.14421"><img src="https://shieldcn.dev/badge/Paper-PDF-black?size=xs" alt="Paper PDF" /></a>
<a href="LICENSE"><img src="https://shieldcn.dev/badge/License-BSD--3--Clause-green?size=xs" alt="License: BSD-3-Clause" /></a>
<a href="https://youtu.be/TsDJOdthhNk"><img src="https://shieldcn.dev/badge/YouTube-red?logo=youtube&size=xs" alt="YouTube" /></a>
</p>

<p align="center">
<a href="https://github.com/ethz-asl/BIEVR-LIO/actions/workflows/build_20_04.yaml"><img src="https://shieldcn.dev/github/ethz-asl/BIEVR-LIO/ci.svg?workflow=build_20_04.yaml&label=ROS1%20Noetic&size=xs&variant=outline&mode=light" alt="Ubuntu 20.04 + ROS Noetic Build" /></a>
</p>

<p align="center">
  <img width='100%' src="doc/tunnel_detail.png">
</p>

BIEVR-LIO is a robust LiDAR-Inertial Odometry framework that uses a high-resolution,
voxel-wise oriented height image map to exploit subtle geometric variations in
challenging, information-sparse environments.

<details>
<summary><b>Abstract</b></summary>
<br>
Reliable odometry is essential for mobile robots as they increasingly enter more challenging environments, which often contain little information to constrain point cloud registration, resulting in degraded LiDAR–Inertial Odometry (LIO) accuracy or even divergence. To address this, we present BIEVR-LIO, a novel approach designed specifically to exploit subtle variations in the available geometry for improved robustness. We propose a high-resolution map representation that stores surfaces as voxel-wise oriented height images. This representation can directly be used for registration without the calculation of intermediate geometric primitives while still supporting efficient updates. Since informative geometry is often sparsely distributed in the environment, we further propose a map-informed point sampling strategy to focus registration on geometrically informative regions, improving robustness in uninformative environments while reducing computational cost compared to global high-resolution sampling. Experiments across multiple sensors, platforms, and environments demonstrate state-of-the-art performance in well-constrained scenes and substantial improvements in challenging scenarios where baseline methods diverge. Additionally, we demonstrate that the fine-grained geometry captured by BIEVR-LIO can be used for downstream tasks such as elevation mapping for robot locomotion.
</details>

# Setup

The core estimator (`bievr_lio`) is a self-contained, ROS-independent library.
This repository provides a **ROS1 Noetic** interface (`bievr_lio_ros`) under
`interfaces/ros1`.

## Installation

### Dependencies

BIEVR-LIO is intentionally light on dependencies: the core estimator only needs
**[Eigen](https://eigen.tuxfamily.org)** and **[Ceres](http://ceres-solver.org)**.


### Build

Requires [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) and
`python3-catkin-tools` (`sudo apt install python3-catkin-tools`).

Create a catkin workspace and clone BIEVR-LIO into it:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/BIEVR-LIO.git BIEVR-LIO
```

Make sure a compatible Ceres installation is available to CMake before building.

(Optional) **Livox support.** The Livox `CustomMsg` branches are only compiled if
the corresponding driver is found in the workspace at build time. Otherwise
BIEVR-LIO builds fine without them. If you need to process Livox data, clone and
build the matching driver into `~/catkin_ws/src` *before* building BIEVR-LIO
(each driver also needs its Livox-SDK installed system-wide):

- Livox gen1 (`livox_ros_driver`, enables `BIEVR_WITH_LIVOX`):
  [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) +
  [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
- Livox gen2 (`livox_ros_driver2`, enables `BIEVR_WITH_LIVOX2`):
  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) +
  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

Build and source it:

```bash
cd ~/catkin_ws
catkin build bievr_lio_ros
source devel/setup.bash
```

## Run data

BIEVR-LIO provides two ROS1 entry points:

- **`process_topics`** runs online: it subscribes to the LiDAR and IMU topics and
  processes messages as they arrive. Use it with a live sensor or alongside
  `rosbag play`.
- **`process_bag`** reads a recorded bag directly and pushes its messages through
  the pipeline as fast as they can be processed (no real-time playback). This is the preferred choice for offline evaluation and reproducing results.

### Quick start（中文）

每次打开新终端后，先加载 ROS 和工作空间环境（请将路径替换为自己的
catkin 工作空间）：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

如果刚修改过代码，需要先重新编译：

```bash
cd ~/catkin_ws
catkin build bievr_lio_ros
source devel/setup.bash
```

离线处理 rosbag（推荐）前，直接编辑
`interfaces/ros1/launch/process_bag.launch` 顶部的 `sensor_config` 和
`rosbag` 默认值，例如：

```xml
<arg name="sensor_config" default="ncd"/>
<arg name="rosbag" default="/absolute/path/to/data.bag"/>
```

然后启动时不需要再从终端传入配置：

```bash
roslaunch bievr_lio_ros process_bag.launch
```

程序会直接读取 bag，不需要另外执行 `rosbag play`。`sensor_config` 可选
`enwide`、`ncd`、`gamma`、`mars`、`grandtour`、`xiangyang`、`tzb`，也可以传入自定义 YAML
配置文件的绝对路径。

在线处理雷达和 IMU 话题前，编辑
`interfaces/ros1/launch/process_topics.launch` 顶部的 `sensor_config` 默认值，
然后直接启动：

```bash
roslaunch bievr_lio_ros process_topics.launch
```

如果数据来自 rosbag，请在另一个已经 source 工作空间的终端中播放：

```bash
rosbag play /absolute/path/to/data.bag
```

需要 RViz 时，在上述 `roslaunch` 命令末尾添加 `rviz:=true`。

在线模式使用 `Ctrl+C` 正常结束；离线模式会在 bag 处理完成后自动结束。
结束时程序会等待尚未累计完成的点云，并创建 `Log` 目录，输出：

| File | Description |
|------|-------------|
| `Log/map.pcd` | 体素降采样后的累计点云地图，包含 `x y z intensity`；每个体素保留首个点及其强度 |
| `Log/trajectory.obj` | OBJ 折线轨迹，可由 MeshLab、Blender 等软件打开 |
| `Log/odometry_tum.txt` | TUM 格式里程计，字段为 `timestamp tx ty tz qx qy qz qw` |
| `Log/localization_tum.txt` | Localization 模式输出的地图坐标系定位轨迹；只有成功定位后才生成 |
| `Log/poles/landmarks.txt` | 去重后的支柱坐标、柱轴、观测次数、拟合半径和 RMS |
| `Log/poles/detections.txt` | 每次检测的柱轴、半径、RMS、内点率、角度覆盖率等拟合质量指标 |
| `Log/poles/landmarks.pcd` | 去重后的支柱中心点云，`intensity` 表示观测次数 |
| `Log/poles/rail_vector_map.txt` | 左右轨顶三维控制点、线路切向、轨距和提取质量 |
| `Log/poles/keyframe_XXXXXX_poles.pcd` | 每个支柱关键帧中通过细过滤的竖直支柱点 |
| `Log/poles_localization/localization_corrections.txt` | 二次运行中通过验收的 `POLE/RAIL` 原始修正、LIO 累计里程、匹配数和 RMS |
| `Log/pole_localization/localization_tum.txt` | 不使用稠密地图时，由支柱修正与每帧 LIO 组合得到的完整轨迹 |
| `Log/pole_localization/filtered_corrections.txt` | 纯支柱定位每次滤波更新的原始/滤波修正、创新、马氏距离和协方差 |
| `Log/pole_localization/summary.yaml` | 稀疏地图定位状态、支柱/轨道接受数、轨迹帧数和末次创新 |

当前 `config/params.yaml` 给出了项目目录下的绝对路径默认值；使用覆盖参数时，
输出目录由最后一层 YAML 决定：

```bash
ls -lh /home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log
```

保存点云要求 `config/params.yaml` 中的 `map_save.accumulate` 保持为
`True`。如果程序被 `kill -9` 强制终止，则退出保存逻辑无法执行。

### 铁路支柱提取

`config/params.yaml` 的 `pole_extraction` 段控制在线支柱提取。模块直接接收
SLAM 内部同一时刻的去畸变点云和位姿。行驶距离达到
`keyframe_distance_m` 且至少积累 `min_frames_per_keyframe` 帧时生成支柱关键帧；
低速或停车时由 `frames_per_keyframe` 限制最大帧数，相邻关键帧复用
`keyframe_overlap_frames` 帧以减少窗口边界漏检。后台提取先把体素点投影到该
关键帧的轨迹折线，按局部切向建立铁路走廊 ROI，再执行二维栅格高度连续性、
形态学连通域过滤和欧式聚类。闭运算补出的栅格只用于连通判断，不会把桥接
栅格中的杂点重新带入三维候选。

粗筛后参考 SE-LIO，通过 PCA 估计柱轴，将候选点投影到柱轴法平面，再用
RANSAC 和最小二乘拟合截面圆；轴倾角、拟合半径、全点径向 RMS、内点率和
角度覆盖率共同决定候选是否保留。PCA 与圆拟合及其验收门限保持不变。单次
观测的支柱坐标是拟合柱轴与 `trajectory_z + center_z_offset_m` 高程平面的
交点；跨关键帧合并时使用拟合质量加权融合柱轴，最后再计算支柱基点，并限制
同一关键帧中的观测与地标一对一匹配。它不会通过 ROS 重新同步点云和里程计。

运行时可在 RViz 中订阅：

- `/bievr_lio/poles/centers`：当前支柱关键帧的中心；
- `/bievr_lio/poles/points`：当前关键帧通过细过滤的支柱点；
- `/bievr_lio/poles/map`：跨关键帧合并后的支柱中心地图。
- `/bievr_lio/poles/markers`：累计支柱的拟合柱体、柱轴、基点和文字 ID。
- `/bievr_lio/poles/prior_map`：二次定位所加载的历史支柱地图（洋红色）。
- `/bievr_lio/poles/correction`：支柱估计的原始 `map <- odom` 修正坐标轴。
- `/bievr_lio/poles/correction_filtered`：纯支柱定位滤波后的正式修正。

随包提供的 RViz 配置已经启用支柱显示：青色点是实际被提取的点，半透明绿色
圆柱是拟合模型，黄色箭头是柱轴，红色球是输出坐标，文字包含支柱 ID、观测
次数、半径和 RMS。在线运行或希望边处理边看时使用：

```bash
roslaunch bievr_lio_ros process_topics.launch rviz:=true
```

离线 `process_bag` 通常很快结束，处理完成后推荐直接查看保存结果：

```bash
python3 scripts/visualize_poles.py
```

查看器会加载 `Log/map.pcd` 和 `Log/poles/`。也可以导出一个带颜色的点云，供
CloudCompare 或 MeshLab 打开：

```bash
python3 scripts/visualize_poles.py --no-map --no-gui \
  --export Log/poles/poles_visualization.ply \
  --overview Log/poles/poles_overview.png
```

襄阳铁路 bag 可直接使用 `xiangyang` 传感器配置，例如：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  rosbag:="/mnt/e/Data/襄阳铁路数据-第二次数据/2026-01-13_襄阳金鹰重工_焦柳线_郜营-余家湖/20260113/bag20260113_1.bag"
```

默认参数以约 10 Hz 的 Livox 数据为基准：正常行驶每 30 m 生成关键帧，至少
积累 20 帧；低速时最多积累 50 帧，并保留 5 帧重叠。其他帧率应相应修改
`min_frames_per_keyframe`、`frames_per_keyframe` 和 `keyframe_overlap_frames`；
提取跟不上时会丢弃最老的待处理支柱关键帧，但不会阻塞里程计，最终统计会
报告丢弃数。

论文面向树干使用的 `0.05 m` 拟合误差在铁路格构柱、方柱和局部遮挡条件下
偏严；襄阳数据默认使用 `cylinder_inlier_threshold_m: 0.10`、
`cylinder_max_rms_m: 0.20` 和 `cylinder_min_inlier_ratio: 0.35`。铁路刚性支柱
默认 `cylinder_max_piecewise_depth: 0`；只有需要拟合弯曲柱体时才建议设为
`1`。跨关键帧地图会在共同高度比较两条拟合轴线，并同时检查轴线距离、柱轴
夹角和半径差；匹配后按点数、内点率、角度覆盖率和拟合残差进行质量加权，
减少斜柱基点外推误差和相邻目标误合并。

将保存支柱用于第二次运行时，`PoleExtractor` 会对最近 7 个关键帧做质量加权
融合，以轴线、轴角和半径门限生成候选，再用 Hungarian 完成一对一关联和 Huber
`SE(2)` 优化。验收后的 `x/y/yaw` 修正直接进入纯支柱定位滤波器，不再传给
GICP。参数、初值要求和完整设计参见
[`doc/pole_landmark_localization.md`](doc/pole_landmark_localization.md)。离线验证器
`scripts/validate_pole_localization.py` 可对两次提取结果注入可控位置/航向误差。

bag 含 `sensor_msgs/NavSatFix` 时，可先用 GTSAM 后端把 LIO 轨迹优化到局部 ENU：

```bash
rosrun bievr_lio_ros gnss_optimize --bag data.bag \
  --lio_trajectory Log/odometry_tum.txt --output_directory Log/gnss_optimization
```

该工具是离线批优化：先读取完整 LIO 轨迹和 bag 内 GNSS，再执行 GTSAM LM；不是
随每帧增量更新的实时 iSAM2 节点。GNSS 按时间插值约束相邻 LIO 位姿，并使用
`p_antenna = p_body + R_body * lever_arm` 杆臂模型。可在
`interfaces/ros1/launch/gnss_optimize.launch` 配置杆臂、协方差缩放/轴向标准差覆盖、
Huber/Cauchy 鲁棒核以及第一轮优化后的卡方异常值剔除。铁路配置将
`odom_neighbor_span` 设为 `3`，以加入参考 PGO 的多邻接 LIO 边；这些组合边与相邻
边相关，如果数据噪声较大或希望更保守，可改回 `1`。

为避免 GNSS 因子逐帧拉动 LIO 位姿并破坏原有点云配准结构，图中默认每 `0.5 s`
选取一个控制位姿；优化后对 `map<-odom` 修正做平滑插值，再作用到全部原始 LIO
位姿。将 `control_pose_interval_s` 设为 `0` 可恢复逐帧优化。

节点还会读取 `pointcloud_topic`（默认 `/livox/lidar`），按 GNSS 优化后的 LIO
位姿重新累积点云，并输出带强度的 `gnss_optimized_map.pcd`。地图采用体素降采样，
默认分辨率为 `0.15 m`，并根据 Livox 点内时间在扫描首尾优化位姿之间做 32 段
运动补偿；可通过 `output_map`、`map_voxel_resolution_m` 和距离门限参数调整。
当前襄阳传感器配置的 LiDAR→IMU 外参为单位阵；其他设备需要在地图重建代码中补入
对应的 `T_I_L` 外参，否则点云会有固定外参偏移。

首次支柱建图把输出的 `gnss_optimized_tum.txt` 填入
`pole_extraction.mapping_trajectory_path`；第二次支柱定位必须清空该项，运行时
只使用 LIO 和 `landmarks.txt`。节点还输出含完整初始四元数的
`initial_correction.yaml`，完整流程和实测结果见上述设计文档。

可选的 `rail_extraction` 会复用同一批支柱关键帧，按横断面显著性、软轨距和
连续性提取左右轨顶，将稀疏三维折线保存为 `rail_vector_map.txt`。第二次运行加载
该矢量地图，以鲁棒点到轨面残差补充 `z/pitch` 约束；支柱继续负责 `x/y/yaw`，
查询阶段仍不加载 GNSS 或稠密点云。组合模式使用：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_rail_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

轨道观测和地图分别发布到 `/bievr_lio/rails/observed` 与
`/bievr_lio/rails/map`。本数据的同源验证中，垂直 RMSE 从纯支柱的 `0.960 m`
降至 `0.076 m`，pitch RMSE 从 `0.246 deg` 降至 `0.039 deg`；详细限制和指标见
设计文档。

#### 推荐的铁路数据完整流程

以下命令以襄阳铁路数据为例。路径和输出目录可以替换，但建图阶段的
`mapping_trajectory_path` 必须填写 GNSS 优化后的 TUM 轨迹，重定位阶段必须清空。

```bash
source /opt/ros/noetic/setup.bash
source /home/zsq/slam/bievr-slam/devel_isolated/setup.bash

BAG="/mnt/e/Data/襄阳铁路数据-第二次数据/2026-01-13_襄阳金鹰重工_焦柳线_郜营-余家湖/20260113/bag20260113_1.bag"

# 终端 1：保持 ROS master 运行
roscore

# 终端 2：继续执行下面的命令
# 1. 先运行 LIO，得到 Log/odometry_tum.txt
rosrun bievr_lio_ros process_bag \
  --sensor_config_file config/sensor_configs/xiangyang.yaml \
  --params_file config/params.yaml --bag "$BAG"

# 2. 使用 /gnss 建立 GTSAM 优化轨迹和初始 map<-odom 变换
rosrun bievr_lio_ros gnss_optimize --bag "$BAG" \
  --lio_trajectory Log/odometry_tum.txt \
  --output_directory Log/gnss_optimization
```

首次提取地图时，在 `config/params.yaml` 中设置：

```yaml
pole_extraction:
  enable: true
  localization_enable: false
  mapping_trajectory_path: "/absolute/path/to/Log/gnss_optimization/gnss_optimized_tum.txt"

rail_extraction:
  enable: true
```

然后再次运行 `process_bag`。输出目录中会同时生成
`landmarks.txt` 和 `rail_vector_map.txt`。建图阶段可以使用累计点云，也可以将
`map_save.accumulate` 设为 `false`；轨道/支柱地图本身不依赖稠密 PCD。

第二次运行前，编辑
`config/params_pole_rail_localization.yaml` 中的 `localization_map_path`、
`rail_extraction.map_path` 和输出目录，然后运行：

```bash
roslaunch bievr_lio_ros process_bag.launch \
  sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_rail_localization \
  rosbag:="$BAG" rviz:=true
```

查询阶段只读取 `/livox/lidar` 和 `/imu/data`（或传感器配置中的 LiDAR/IMU 话题），
不读取 `/gnss`，不加载 `map.pcd`，也不运行 GICP。若从中途开始运行，需要将
`gnss_optimize` 输出的 `initial_correction.yaml` 内容作为最后一层参数，或者通过
RViz `/initialpose` 提供初始位姿。

GNSS 后端使用 GTSAM 优化 LIO 轨迹；轨道在线定位使用鲁棒小量最小二乘和独立的
`z/roll/pitch` 滤波器，支柱在线定位使用 Hungarian 关联和 Huber `SE(2)`。轨道并
不是稠密点云匹配的替代品，而是稀疏支柱地图模式中的连续高程/坡度约束。

#### 评估和可视化

相对于参考 TUM 轨迹计算水平、垂直、三维、yaw、roll 和 pitch 误差：

```bash
python3 scripts/evaluate_tum_against_reference.py \
  --estimate Log/pole_rail_localization/localization_tum.txt \
  --reference Log/gnss_optimization/gnss_optimized_tum.txt \
  --output Log/pole_rail_localization/metrics.json
```

查看支柱和轨道矢量地图，并导出 CloudCompare/MeshLab 可读的 PLY：

```bash
python3 scripts/visualize_poles.py --no-map --no-gui \
  --poles Log/poles \
  --overview Log/poles/rail_overview.png \
  --export Log/poles/rail_map_colored.ply
```

`visualize_poles.py` 检测到 `rail_vector_map.txt` 后会自动绘制左右轨；没有该文件
时仍可只查看支柱地图。

稠密点云重定位叠加 `params_localization.yaml`：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

该模式关闭全部支柱模块，只加载稠密地图并运行 GICP。完全不加载 `map.pcd`、
只使用 LIO 和 `landmarks.txt` 时，使用独立的纯支柱定位模式：

```bash
roslaunch bievr_lio_ros pole_localization.launch \
  sensor_config:=xiangyang rviz:=true
```

离线复放使用：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

纯支柱模式发布与稠密定位相同的 `/bievr_lio/loc/odom`、`/bievr_lio/loc/path`、
`map -> odom` TF 和 `/bievr_lio/loc/status`。默认等待 `/initialpose`；已知两次 LIO
共原点时也可临时启用 `pole_extraction.localization_use_initial_guess`。支柱失配后
状态按行驶距离从 `TRACKING` 变为 `DEGRADED/LOST`，但最后有效修正会保留，LIO
局部轨迹不会被打断。后台提取得到的修正按关键帧传感器时间和累计里程进入滤波，
离线高速回放时会回填修正对应的轨迹尾段，不受后台线程完成时刻影响。

launch 文件只负责选择配置文件、数据和 RViz，不会覆盖算法参数。运行模式
统一在参数文件中设置；两个重定位模式严格互斥，误将它们同时开启会直接退出：

- 纯里程计：`loop_closure.enable: False`，`localization.enable: False`
- 回环建图：`loop_closure.enable: True`，`localization.enable: False`；地图包
  默认输出到 `Log/bievr_map_bundle`
- 稠密点云定位：`loop_closure.enable: False`，`localization.enable: True`，
  `pole_extraction.enable: False`，`pole_localization.enable: False`；默认加载
  `Log/bievr_map_bundle`
- 纯支柱定位：`loop_closure.enable: False`，`localization.enable: False`，
  `pole_extraction.localization_enable: True`，`pole_localization.enable: True`
- 支柱 + 轨道矢量定位：在纯支柱模式基础上设置 `rail_extraction.enable: True`，
  或叠加 `params_pole_rail_localization.yaml`

The launch files contain editable defaults for the sensor config, parameter file,
bag path and RViz switch. Change those values at the top of the relevant launch
file, then use the commands below without additional terminal arguments.

Algorithm settings are not overridden by launch files. Use `config/params.yaml`
as the base and select exactly one localization overlay when relocalizing:

- Odometry: `loop_closure.enable: False`, `localization.enable: False`
- Mapping: `loop_closure.enable: True`, `localization.enable: False`; the default
  bundle output is `Log/bievr_map_bundle`
- Dense-map localization (`config/params_localization.yaml`): `localization.enable: True`,
  `pole_extraction.enable: False`, `pole_localization.enable: False`, and load
  `Log/bievr_map_bundle` through `localization.map_path`
- Pole-map localization (`config/params_pole_localization.yaml`): `localization.enable: False`,
  `pole_extraction.localization_enable: True`, `pole_localization.enable: True`

Process live topics:

```bash
roslaunch bievr_lio_ros process_topics.launch
```

Replay a rosbag:

```bash
roslaunch bievr_lio_ros process_bag.launch
```

## Configuration

The configuration is split in two files:

- **`config/params.yaml`**: Algorithm parameters (map resolution, sampling,
  optimization, IMU window, ...). These are dataset-independent and **typically do
  not need to be adjusted**: the defaults have been validated across a wide range
  of sensors, platforms, and environments.
- **`config/sensor_configs/<name>.yaml`**: Per-dataset / per-sensor settings:
  the LiDAR and IMU topic names, the LiDAR→IMU extrinsic calibration, and the
  LiDAR min/max range.

Set the `sensor_config` default near the top of the launch file. A short name such
as `ncd` resolves to `config/sensor_configs/ncd.yaml`; an absolute path is used
verbatim, so configs may also live outside the package. The `params` default works
the same way and selects a file under `config/` (the default is `params.yaml`).

<details>
<summary><b>Provided datasets</b></summary>
<br>

We provide ready-to-use sensor configs for the following public datasets:

| Config | Dataset |
|--------|---------|
| `enwide` | [ENWIDE](https://projects.asl.ethz.ch/datasets/enwide/) |
| `ncd` | [Newer College Dataset](https://drive.google.com/drive/u/0/folders/1uR476FzjN3PfAiCknVKtuZi3_QfVvSdA) |
| `gamma` | [GEODE](https://thisparticle.github.io/geode) |
| `mars` | [MARS-LVIG](https://mars.hku.hk/dataset.html) |
| `grandtour` | [GrandTour](https://grand-tour.leggedrobotics.com/) |
</details>

<details>
<summary><b>Running on your own data</b></summary>
<br>

To run BIEVR-LIO on a new sensor or dataset, copy one of the provided sensor
configs to `config/sensor_configs/<your_name>.yaml` and adjust:

- `topics.pointcloud` / `topics.imu` : The topic names in your data.
- `calibration` : the `T_IMU_LIDAR` extrinsic (LiDAR → IMU) rotation and
  translation for your setup.
- `lidar.min_range_m` / `lidar.max_range_m` : the usable range of your LiDAR.

The algorithm parameters in `params.yaml` can usually be left at their defaults.
</details>

# Acknowledgements
For a full SLAM integration including loop-closure detection and pose-graph optimization check out [BIEVR-LIO-SLAM](https://github.com/S0UL4/BIEVR-LIO-SLAM), contributed by [SOUL4](https://github.com/S0UL4)

We thank the authors of [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry), [Wavemap](https://github.com/ethz-asl/wavemap) and [UGPM](https://github.com/UTS-RI/ugpm) for open-sourcing their works that served as an inspiration for us.
We used [ascii-image-converter](https://github.com/TheZoraiz/ascii-image-converter) for our ascii art.

# Citation

Please cite our work if you are using BIEVR-LIO in your research.
  ```bibtex
@article{pfreundschuh2026bievr,
  title        = {BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps},
  author       = {Pfreundschuh, Patrick and Tuna, Turcan and {Le Gentil}, Cedric and Siegwart, Roland and Cadena, Cesar and Oleynikova, Helen},
  year         = 2026,
  journal      = {Robotics: Science and Systems},
}
  ```
