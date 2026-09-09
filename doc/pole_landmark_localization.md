# 支柱地图辅助定位设计

## 定位边界

支柱是稀疏、稳定的几何地标，适合修正局部 `x、y、yaw`。轨道矢量地图作为同一
稀疏地图模式的可选组成，连续修正 `z、pitch`；当前数据的 LIO 横滚优于稀疏轨顶
估计，因此 `roll` 默认保持 LIO，可通过 `rail_extraction.update_roll` 显式开启。
铁路支柱结构重复，不应只凭一根支柱完成全局重定位。

系统只保留两种互斥的重定位模式：稠密点云模式只运行 Localizer/GICP，支柱模式
只运行 PoleExtractor/PoleLocalization，并可加载支柱点地图和轨道矢量地图。稠密
地图不会与稀疏地图在同一次重定位中加载或融合。

## 保存格式

数学上，已知对应 ID 时只保存一个二维位置就能参与 `SE(2)` 优化；但这
不足以支持自动匹配。每根支柱的工程最小集为：

| 字段 | 用途 |
|---|---|
| `id` | 地图内稳定且唯一的标识 |
| `axis_point[3]` | 已知高程上的柱轴点；当前 `center X/Y/Z` 可直接作为该点 |
| `axis[3]` | 单位柱轴方向，用于共同高度比较和姿态一致性检查 |
| `radius` | 关联特征，不直接当作位姿残差 |
| `cov_xy[3]` | `CovXX、CovXY、CovYY`，决定该地标对优化的权重 |
| `observations` | 重复观测次数，用于拒绝未确认的单次地标 |
| `fit_rms` | 无协方差时的质量权重退化项 |

推荐额外保存 `axial_min/max、radius_std、axis_std_deg、total_points、track_s、side`。
`track_s` 是沿线累计里程，`side` 是线路左右侧；两者对拒绝周期性误匹非常有效。

每张支柱地图还必须有一份地图级元数据，不应在每根支柱中重复保存：

- `format_version`、`frame_id`、米/弧度单位和坐标轴定义；
- 局部原点或 GNSS/UTM 与局部地图的变换；
- 地图创建时间、版本和支柱提取参数哈希；
- `axis_point` 的高程基准定义。

当前 `landmarks.txt` 已有 `ID、center、axis、axial extent、observations、radius、RMS`，
可以用于原型验证。生产地图格式应再增加 `cov_xy`和上述地图级元数据。

## 在线数据流

```text
LIO 去畸变帧 + T_odom_body
              |
              +---- 稠密模式 ----> 点云地图 ----> Localizer / GICP
              |                                      |
              |                              六自由度 T_map_odom
              |
              +---- 支柱模式 ----> PoleExtractor ----> 局部支柱子图(odom)
                                                      |
已保存支柱地图(map) ----------------------------+
                                                      v
                                         门限 + 一对一关联
                                                      |
                                           Huber SE(2) 优化
                                                      |
                                            PoleLocalization
                                                      |
                                        x/y/yaw T_map_odom
```

1. 积累最近 5–7 个支柱关键帧，先在 odom 系内去重，形成局部支柱子图。
2. 用当前 `T_map_odom` 预测位置，以轴线距离、轴角、半径、线路侧别和
   `track_s` 给出候选。关联门限建议从 `4 m / 5 deg / 0.30 m` 开始。
3. 对候选成本矩阵使用 Hungarian 一对一匹配，禁止多个当前观测共用同一
   历史地标。
4. 在共同高度取两条柱轴点，优化

   `r_i = sqrt_information_i * (project_xy(T_map_odom * L_odom_i) - L_map_i)`。

   使用 Huber 核，信息矩阵由地图协方差和当前拟合质量联合给出。
5. 3 对支柱是数学验收下限，且沿线空间基线必须大于 10 m。工程上建议
   至少 5 对、匹配比例大于 60%、残差 RMS 小于 0.5 m，否则不更新。
6. 支柱解通过创新和马氏距离门限后更新 `T_map_odom`；否则保持上次有效修正。
   首次定位和丢失恢复都要求连续多窗口确认，避免 TF 跳变。

## BIEVR-LIO 中的实现

实现没有把支柱残差写入 LIO 前端。`PoleExtractor` 在后台维护最近
`localization_window_keyframes` 个关键帧的局部支柱，以轴线距离、轴角和半径门限
生成候选，再用 Hungarian 做一对一关联。通过匹配数、匹配率、空间基线、RMS
和单次修正幅度验收后，输出 `T_map_odom` 的 `x、y、yaw` 修正。

支柱模式下，`PoleExtractor::setCorrectionProvider(...)` 只读取
`PoleLocalization` 的上次滤波解作为关联初值，`setCorrectionObserver(...)` 只把
新支柱解送回该滤波器。稠密 `Localizer` 不再提供或接收任何支柱接口。程序会在
加载地图前检查配置；若稠密定位与支柱定位同时开启，或稠密模式仍开启支柱提取，
会报错退出。

不使用稠密地图时，`PoleLocalization` 直接接收支柱修正，维护
`[T_map_odom.x、T_map_odom.y、yaw]` 三维滤波状态。过程协方差随 LIO 行驶距离
增长；支柱测量协方差由匹配 RMS、数量、比例和空间基线自适应计算。更新前还会
检查平移/航向创新和马氏距离。它不修改 LIO 状态，而是在每个 LIO 帧计算
`T_map_body = T_map_odom * T_odom_body`。

支柱提取在后台线程执行，但每个修正携带关键帧时间戳和该时刻的 LIO 累计里程。
滤波预测按这个传感器时间轴推进；若离线高速回放时结果延迟返回，则从该时间戳
重算已经缓存的轨迹尾段。因此调度只影响修正何时发布，不影响最终保存轨迹。

纯支柱定位状态为：

```text
UNINITIALIZED -> SEARCHING -> TRACKING -> DEGRADED -> LOST
                         ^          |          |
                         +----------+----------+
                              连续支柱确认恢复
```

`SEARCHING` 默认需要两个有效窗口才发布正式定位；行驶一定距离没有新支柱更新后
依次进入 `DEGRADED` 和 `LOST`。两个状态都保留最后的 `T_map_odom`，因此输出由
LIO 连续推算而不会突然中断，同时 `/loc/status` 明确报告可信度下降。

定位相关输出为：

- `/bievr_lio/poles/prior_map`：首次运行保存的支柱地图，位于 `map` 坐标系；
- `/bievr_lio/poles/correction`：支柱估计的 `map <- odom` 修正；
- `/bievr_lio/poles/correction_filtered`：纯支柱模式滤波后的正式修正；
- `poles_localization/localization_corrections.txt`：每个有效窗口的时间戳、位姿、
  LIO 累计里程、匹配数、匹配率和 RMS；
- `pole_localization/filtered_corrections.txt`：每次纯支柱滤波更新的原始/滤波解、
  创新、马氏距离和协方差；
- `pole_localization/localization_tum.txt`：纯支柱模式的地图坐标系轨迹。

当前文本地图会一次性载入内存；超长线路的生产版本仍建议增加 `track_s/side` 和
地图 tile 索引，只加载当前位置附近的支柱。

## GNSS 因子图建图

新增的 `gnss_optimize` 节点将 LIO TUM 轨迹作为 `Pose3` 变量，相邻帧加入 LIO
`BetweenFactor<Pose3>`，`/gnss` 的 WGS84 坐标转换到首个有效定位建立的局部 ENU。
GNSS 时间通常位于两个激光帧之间，因此使用二元插值位置因子约束
`(1-alpha) * p_i + alpha * p_(i+1)`，不把定位结果强行吸附到最近激光帧。GNSS
因子按消息协方差加权，并带 Huber 核和初始残差异常值门限。

```bash
rosrun bievr_lio_ros gnss_optimize \
  --bag /absolute/path/to/data.bag \
  --lio_trajectory /absolute/path/to/odometry_tum.txt \
  --output_directory /absolute/path/to/gnss_result
```

输出包括 `gnss_optimized_tum.txt`、`summary.yaml` 和
`initial_correction.yaml`。随后首次支柱建图在 `pole_extraction` 中配置：

```yaml
localization_enable: false
mapping_trajectory_path: /absolute/path/to/gnss_optimized_tum.txt
mapping_trajectory_max_time_gap_s: 0.20
```

提取器会用优化轨迹变换每帧点云并保存 ENU/map 系下的支柱。若定位模式开启时
仍填写 `mapping_trajectory_path`，程序会拒绝配置。第二次运行必须清空该路径，
关闭 GNSS 与稠密地图，只加载 `landmarks.txt`。`initial_correction.yaml` 还保存
完整四元数；中途启动或坡道场景用它初始化固定的 roll/pitch，后续支柱仍只优化
`x、y、yaw`。

`initial_correction.yaml` 本身是可叠加的参数文件，可在第二次离线运行中作为最后
一个 `--params_file` 传入；在线系统也可改由 `/initialpose` 提供同等初值。

2026-09-09 使用 bag1 的 160--340 s 做了同源闭环验证：179.7 s、约 1.109 km，
支柱查询 bag 只包含 `/livox/lidar` 与 `/imu/data`。GNSS 因子图使用 719 个定位
约束，水平观测残差 RMSE 从 `0.527 m` 降到 `0.0067 m`；这只是拟合残差，不是
独立真值误差。优化轨迹建立了 41 根支柱地图，第二次无 GNSS 运行的 45/45 个
支柱窗口均通过滤波，输出 1704 帧并保持 `TRACKING`。相对 GNSS 优化参考：

- 原始 LIO 经首帧 SE(3) 初始化后，水平 RMSE `1.112 m`、P95 `2.624 m`、末端
  `2.978 m`；
- LIO + 支柱地图不再二次对齐，水平 RMSE `0.198 m`、P95 `0.266 m`、最大
  `0.292 m`、末端 `0.199 m`，航向 RMSE `0.067 deg`；
- 支柱当前不更新 z，三维 RMSE `0.980 m` 主要来自 LIO 高程漂移，不能将水平
  精度当成六自由度精度。

同一组数据建图和定位会高估重复检测能力；正式验收应使用独立往返采集、独立
GNSS 真值，并标定 GNSS 天线到 IMU 的杆臂。

## 轨道矢量地图与垂直优化

`rail_extraction` 直接复用支柱模块已经形成的关键帧和 GNSS 优化建图轨迹，不重复
缓存点云。算法沿关键帧轨迹按 `station_resolution_m` 划横断面，在每个断面构建
横向高度栅格，然后联合应用轨顶显著性、软轨距、左右轨高差、横向/垂向连续性和
最短有效段门限。通过的断面保存左右轨顶控制点、切线、轨距和质量：

```text
Segment Index Timestamp Station
LeftX LeftY LeftZ RightX RightY RightZ
TangentX TangentY TangentZ Gauge Quality
```

定位时先用支柱滤波结果预测轨道位置，将当前成对轨样本和地图控制点按中心距离、
切线方向及左右侧组合一对一关联。在当前关键帧中心附近建立小角度线性残差：

`r_z = z_map - z(T_map_odom * p_rail)`

其雅可比为 `[dy, -dx, 1]`，对应 `roll、pitch、z`。采用 Huber IRLS 求出局部修正，
通过匹配数、沿轨基线、垂直 RMS 和单步幅度验收后送入独立垂直滤波状态。支柱更新
`x/y/yaw`，轨道默认更新 `z/pitch`，所以直线轨道不可观的沿轨位移不会污染结果。

建图输出 `rail_vector_map.txt`，ROS 可视化话题为：

- `/bievr_lio/rails/observed`：当前关键帧提取的左右轨控制点；
- `/bievr_lio/rails/map`：建图累计或定位加载的轨道矢量地图。

使用 `config/params_pole_rail_localization.yaml` 可启动 LIO + 支柱 + 轨道模式。查询
阶段必须清空 `mapping_trajectory_path`，只读取 LiDAR/IMU、`landmarks.txt` 和
`rail_vector_map.txt`，不读取 GNSS 或稠密点云。

2026-09-09 对 bag1 160--340 s 的同源闭环测试提取了 2544 个成对轨道控制点；
39 个有足够运动基线的查询关键帧中，37 个形成几何校正且 37 个全部通过垂直滤波。
相对同段 GTSAM GNSS 优化轨迹：

- 支柱模式垂直 RMSE `0.960 m`、P95 `2.470 m`、末端 `2.940 m`；
- 支柱 + 轨道模式垂直 RMSE `0.076 m`、P95 `0.157 m`、末端 `0.067 m`；
- pitch RMSE 从 `0.246 deg` 降到 `0.039 deg`，三维 RMSE 从 `0.980 m` 降到
  `0.207 m`；水平 RMSE 为 `0.193 m`。

这些结果是同一数据的重复提取一致性，不是独立真值精度。轨道高程继承 GNSS 建图
轨迹的高程基准，正式验收仍需独立复测和更可靠的 GNSS/INS 高程真值。

## 两次运行方法

首次建图保留 `pole_extraction.localization_enable: False`，运行结束会保存
`Log/poles/landmarks.txt`。支柱模式只加载 `landmarks.txt`。稠密点云在线定位使用：

```bash
roslaunch bievr_lio_ros localization.launch sensor_config:=xiangyang rviz:=true
```

这是纯稠密 GICP 模式，不提取或加载支柱。完全不加载稠密地图时使用：

```bash
roslaunch bievr_lio_ros pole_localization.launch sensor_config:=xiangyang rviz:=true
```

离线复放 bag 时使用基础参数和定位覆盖层：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

纯支柱离线模式将覆盖层改为 `params_pole_localization`。离线 bag 无法交互点击
`/initialpose` 时，需要在覆盖参数中显式配置已知初值。

稠密模式若地图包含 Scan Context 数据库，可以自动提供全局初值。纯支柱模式当前
没有实现无先验的全线支柱序列搜索：在线先在 RViz 发布 `/initialpose`，离线需由
GNSS/已知起点提供初值。只有明确知道两次 odom 共原点时，才开启
`localization_use_initial_guess` 并填写手工初值。

## 离线验证

`scripts/validate_pole_localization.py` 使用首次运行的 `landmarks.txt` 和另一组
`detections.txt`，注入局部位置/航向误差，再运行一对一匹配与质量加权
Huber `SE(2)` 优化：

```bash
python3 scripts/validate_pole_localization.py \
  --map first_run/poles/landmarks.txt \
  --detections second_run/poles/detections.txt \
  --window-keyframes 7 --trials 1000 \
  --max-position-error-m 3.0 --max-yaw-error-deg 2.0
```

当前数据集没有在同一区段的独立第二次采集。系统集成验证用 bag1 的
160--340 s 同一片段生成先验，再复放该片段并注入 `2 m、-1 m、1 deg` 初始误差。
它可验证在线数据流、局部捕获和支柱关联，但同源复放不能代替真实
往返复跑验收。

纯支柱模式在同一片段中完全关闭 `localization`，未加载 `map.pcd`，结果为：

- 45 个包含新支柱证据的窗口产生有效原始解，滤波确定性接受 45 个；
- 输出 1704/1798 个 LIO 帧，缺少的前 94 帧用于首次两个窗口确认；
- 相对同源建图轨迹，XY 平均误差 `0.072 m`、P95 `0.140 m`，航向平均
  `0.043 deg`；同一 bag 连续回放两次，轨迹和滤波诊断文件逐字节一致；
- 将支柱地图截断在线路约一半处后，状态按预期进入 `DEGRADED`、`LOST`，仍连续
  输出到最后一帧，末端 XY 偏差 `0.194 m`，没有关联到地图外的错误区段。
