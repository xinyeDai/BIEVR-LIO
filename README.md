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
`enwide`、`ncd`、`gamma`、`mars`、`grandtour`，也可以传入自定义 YAML
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

当前 `config/params.yaml` 使用绝对路径，所有结果固定保存到项目目录：

```bash
ls -lh /home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log
```

保存点云要求 `config/params.yaml` 中的 `map_save.accumulate` 保持为
`True`。如果程序被 `kill -9` 强制终止，则退出保存逻辑无法执行。

launch 文件只负责选择配置文件、数据和 RViz，不会覆盖算法参数。运行模式
统一在 `config/params.yaml` 中设置：

- 纯里程计：`loop_closure.enable: False`，`localization.enable: False`
- 回环建图：`loop_closure.enable: True`，`localization.enable: False`；地图包
  默认输出到 `Log/bievr_map_bundle`
- 地图定位：`loop_closure.enable: False`，`localization.enable: True`；默认加载
  `Log/bievr_map_bundle`

The launch files contain editable defaults for the sensor config, parameter file,
bag path and RViz switch. Change those values at the top of the relevant launch
file, then use the commands below without additional terminal arguments.

Algorithm settings are not overridden by launch files. Configure each mode only
in `config/params.yaml`:

- Odometry: `loop_closure.enable: False`, `localization.enable: False`
- Mapping: `loop_closure.enable: True`, `localization.enable: False`; the default
  bundle output is `Log/bievr_map_bundle`
- Localization: `loop_closure.enable: False`, `localization.enable: True`, and
  load `Log/bievr_map_bundle` through `localization.map_path`

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
