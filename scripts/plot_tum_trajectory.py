#!/usr/bin/env python3
"""Compare an estimated TUM trajectory with ground truth and draw error plots.

TUM trajectory format (one pose per line):
    timestamp tx ty tz qx qy qz qw

Only timestamps and translations are used. Ground-truth positions are linearly
interpolated at the estimate timestamps before alignment and error calculation.
"""

# =============================================================================
# 用户配置区：只需要修改这里
# =============================================================================

# 估计轨迹和真值轨迹，均为 TUM 格式：
# timestamp tx ty tz qx qy qz qw
ESTIMATE_PATH = "Log/odometry_tum.txt"
GROUND_TRUTH_PATH = "/home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log/bievr_map_bundle/poses_tum.txt"

# 图片输出设置
OUTPUT_DIR = "trajectory_plots"
OUTPUT_PREFIX = "bievr"
ESTIMATE_LABEL = "BIEVR-LIO"
IMAGE_FORMAT = "png"  # 可选：png、pdf、svg
IMAGE_DPI = 200
SHOW_FIGURES = False  # True：保存后弹出绘图窗口；False：只保存图片

# 空间对齐方式：
# "none"   不对齐，适用于两条轨迹已经处于完全相同的坐标系
# "origin" 仅平移估计轨迹，使两条轨迹起点重合
# "se3"    旋转和平移对齐，适用于 LIO/SLAM 轨迹（推荐）
# "sim3"   旋转、平移和尺度对齐，适用于尺度不确定的轨迹
ALIGNMENT = "se3"

# 时间同步设置
# 该偏移量会加到估计轨迹时间戳上，单位为秒。
TIME_OFFSET = 0.0

# 估计时刻与最近真值采样之间允许的最大距离，单位为秒。
# 设置为 0 表示不限制，只要处于真值的起止时间范围内就进行线性插值。
MAX_TIME_GAP = 0.0

# =============================================================================
# 配置区结束，以下代码通常不需要修改
# =============================================================================

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np


@dataclass
class Trajectory:
    timestamps: np.ndarray
    positions: np.ndarray


@dataclass
class Alignment:
    scale: float
    rotation: np.ndarray
    translation: np.ndarray

    def apply(self, positions: np.ndarray) -> np.ndarray:
        return self.scale * (positions @ self.rotation.T) + self.translation


def load_tum(path: Path) -> Trajectory:
    """Load and validate a whitespace-separated TUM trajectory file."""
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 8:
                raise ValueError(
                    "{}:{} 不是有效的 TUM 数据（至少需要 8 列，实际为 {} 列）".format(
                        path, line_number, len(fields)
                    )
                )
            try:
                values = [float(value) for value in fields[:8]]
            except ValueError as exc:
                raise ValueError("{}:{} 含有非数字字段".format(path, line_number)) from exc
            if not np.all(np.isfinite(values)):
                raise ValueError("{}:{} 含有 NaN 或 Inf".format(path, line_number))
            rows.append(values)

    if not rows:
        raise ValueError("TUM 文件为空或只有注释：{}".format(path))

    data = np.asarray(rows, dtype=np.float64)
    order = np.argsort(data[:, 0], kind="stable")
    data = data[order]

    # np.interp requires strictly increasing sample timestamps. Keep the last
    # pose when a producer has emitted the same timestamp more than once.
    reversed_data = data[::-1]
    _, reversed_unique_indices = np.unique(reversed_data[:, 0], return_index=True)
    data = reversed_data[np.sort(reversed_unique_indices)][::-1]
    data = data[np.argsort(data[:, 0], kind="stable")]

    return Trajectory(timestamps=data[:, 0], positions=data[:, 1:4])


def interpolate_ground_truth(
    estimate: Trajectory,
    ground_truth: Trajectory,
    time_offset: float,
    max_time_gap: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return estimate timestamps/positions and interpolated GT positions."""
    query_timestamps = estimate.timestamps + time_offset
    valid = (query_timestamps >= ground_truth.timestamps[0]) & (
        query_timestamps <= ground_truth.timestamps[-1]
    )

    if max_time_gap > 0.0:
        insertion = np.searchsorted(ground_truth.timestamps, query_timestamps)
        left = np.clip(insertion - 1, 0, len(ground_truth.timestamps) - 1)
        right = np.clip(insertion, 0, len(ground_truth.timestamps) - 1)
        nearest_gap = np.minimum(
            np.abs(query_timestamps - ground_truth.timestamps[left]),
            np.abs(query_timestamps - ground_truth.timestamps[right]),
        )
        valid &= nearest_gap <= max_time_gap

    if np.count_nonzero(valid) < 3:
        raise ValueError(
            "共同时间范围内不足 3 个有效位姿；请检查时间戳、--time-offset "
            "和 --max-time-gap"
        )

    matched_timestamps = query_timestamps[valid]
    estimate_positions = estimate.positions[valid]

    # Subtracting the first timestamp improves interpolation precision when
    # Unix epoch timestamps are around 1e9 seconds.
    time_origin = ground_truth.timestamps[0]
    gt_relative_timestamps = ground_truth.timestamps - time_origin
    query_relative_timestamps = matched_timestamps - time_origin
    ground_truth_positions = np.column_stack(
        [
            np.interp(
                query_relative_timestamps,
                gt_relative_timestamps,
                ground_truth.positions[:, axis],
            )
            for axis in range(3)
        ]
    )
    return matched_timestamps, estimate_positions, ground_truth_positions


def calculate_alignment(
    source: np.ndarray, target: np.ndarray, method: str
) -> Alignment:
    """Calculate a transform mapping source points into the target frame."""
    identity = np.eye(3, dtype=np.float64)
    zero = np.zeros(3, dtype=np.float64)

    if method == "none":
        return Alignment(1.0, identity, zero)
    if method == "origin":
        return Alignment(1.0, identity, target[0] - source[0])

    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    source_variance = np.mean(np.sum(source_centered**2, axis=1))
    if source_variance <= np.finfo(np.float64).eps:
        raise ValueError("估计轨迹几乎没有运动，无法进行 {} 对齐".format(method))

    covariance = target_centered.T @ source_centered / len(source)
    u_matrix, singular_values, vt_matrix = np.linalg.svd(covariance)
    signs = np.ones(3, dtype=np.float64)
    if np.linalg.det(u_matrix @ vt_matrix) < 0.0:
        signs[-1] = -1.0
    rotation = u_matrix @ np.diag(signs) @ vt_matrix

    scale = 1.0
    if method == "sim3":
        scale = float(np.sum(singular_values * signs) / source_variance)
        if scale <= 0.0:
            raise ValueError("Sim(3) 对齐得到非正尺度，无法继续")

    translation = target_mean - scale * (rotation @ source_mean)
    return Alignment(scale, rotation, translation)


def configure_plot_style() -> None:
    import matplotlib
    from matplotlib import font_manager

    # Some Matplotlib builds do not discover TrueType Collection (.ttc) fonts
    # automatically even though fontconfig can see them. Register common CJK
    # collections explicitly so Chinese labels also work in headless runs.
    cjk_font_files = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
        "/System/Library/Fonts/PingFang.ttc",
        "C:/Windows/Fonts/msyh.ttc",
    )
    for font_path in cjk_font_files:
        if Path(font_path).is_file():
            try:
                font_manager.fontManager.addfont(font_path)
            except (OSError, RuntimeError):
                pass

    installed_fonts = {font.name for font in font_manager.fontManager.ttflist}
    preferred_fonts = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Microsoft YaHei",
        "SimHei",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    ]
    selected_font = next(
        (font for font in preferred_fonts if font in installed_fonts), "DejaVu Sans"
    )
    matplotlib.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [selected_font, "DejaVu Sans"],
            "axes.unicode_minus": False,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "legend.fontsize": 10,
            "grid.alpha": 0.28,
            "grid.linestyle": "--",
            "lines.linewidth": 1.7,
        }
    )


def set_3d_equal_scale(axis, point_sets: Sequence[np.ndarray]) -> None:
    points = np.vstack(point_sets)
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    center = (lower + upper) / 2.0
    radius = max(float(np.max(upper - lower)) / 2.0, 1e-6)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1.0, 1.0, 1.0))


def draw_trajectory_figure(
    pyplot,
    estimate_positions: np.ndarray,
    ground_truth_positions: np.ndarray,
    estimate_label: str,
    alignment_method: str,
):
    figure = pyplot.figure(figsize=(13.5, 5.8), constrained_layout=True)
    figure.suptitle("轨迹对比（空间对齐：{}）".format(alignment_method.upper()), fontsize=16)

    axis_xy = figure.add_subplot(1, 2, 1)
    axis_xy.plot(
        ground_truth_positions[:, 0],
        ground_truth_positions[:, 1],
        color="black",
        linewidth=2.2,
        label="真值",
    )
    axis_xy.plot(
        estimate_positions[:, 0],
        estimate_positions[:, 1],
        color="#d62728",
        label=estimate_label,
    )
    axis_xy.scatter(
        ground_truth_positions[0, 0],
        ground_truth_positions[0, 1],
        color="#2ca02c",
        marker="o",
        s=42,
        zorder=4,
        label="起点",
    )
    axis_xy.set_title("XY 平面轨迹")
    axis_xy.set_xlabel("X (m)")
    axis_xy.set_ylabel("Y (m)")
    axis_xy.set_aspect("equal", adjustable="datalim")
    axis_xy.grid(True)
    axis_xy.legend()

    axis_3d = figure.add_subplot(1, 2, 2, projection="3d")
    axis_3d.plot(
        ground_truth_positions[:, 0],
        ground_truth_positions[:, 1],
        ground_truth_positions[:, 2],
        color="black",
        linewidth=2.2,
        label="真值",
    )
    axis_3d.plot(
        estimate_positions[:, 0],
        estimate_positions[:, 1],
        estimate_positions[:, 2],
        color="#d62728",
        label=estimate_label,
    )
    axis_3d.scatter(
        ground_truth_positions[0, 0],
        ground_truth_positions[0, 1],
        ground_truth_positions[0, 2],
        color="#2ca02c",
        marker="o",
        s=36,
        label="起点",
    )
    axis_3d.set_title("三维轨迹")
    axis_3d.set_xlabel("X (m)")
    axis_3d.set_ylabel("Y (m)")
    axis_3d.set_zlabel("Z (m)")
    axis_3d.grid(True)
    axis_3d.legend()
    set_3d_equal_scale(axis_3d, (estimate_positions, ground_truth_positions))
    return figure


def draw_error_figure(
    pyplot, relative_time: np.ndarray, errors: np.ndarray, estimate_label: str
):
    error_norm = np.linalg.norm(errors, axis=1)
    figure, axes = pyplot.subplots(
        2, 2, figsize=(13.5, 9.0), sharex=True, constrained_layout=True
    )
    figure.suptitle("相对于真值的位置误差分析", fontsize=16)

    components = (("X", 0), ("Y", 1), ("Z", 2))
    for axis, (component, index) in zip(axes.flat[:3], components):
        axis.plot(relative_time, errors[:, index], color="#d62728", label=estimate_label)
        axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.55)
        axis.set_title("{} 方向误差".format(component))
        axis.set_ylabel(r"$\Delta {}$ (m)".format(component))
        axis.grid(True)
        axis.legend()

    norm_axis = axes.flat[3]
    norm_axis.plot(relative_time, error_norm, color="#d62728", label=estimate_label)
    norm_axis.set_title("三维位置误差范数")
    norm_axis.set_ylabel(r"$\|\Delta \mathbf{p}\|$ (m)")
    norm_axis.grid(True)
    norm_axis.legend()
    norm_axis.text(
        0.98,
        0.96,
        "RMSE = {:.3f} m\nMAE = {:.3f} m\nMAX = {:.3f} m".format(
            np.sqrt(np.mean(error_norm**2)),
            np.mean(error_norm),
            np.max(error_norm),
        ),
        transform=norm_axis.transAxes,
        ha="right",
        va="top",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.82},
    )

    for axis in axes[-1, :]:
        axis.set_xlabel("时间 (s)")
    return figure


def print_statistics(errors: np.ndarray, sample_count: int, alignment: Alignment) -> None:
    error_norm = np.linalg.norm(errors, axis=1)
    component_rmse = np.sqrt(np.mean(errors**2, axis=0))
    print("有效匹配位姿数: {}".format(sample_count))
    print("对齐尺度: {:.9f}".format(alignment.scale))
    print(
        "分轴 RMSE [m]: X={:.6f}, Y={:.6f}, Z={:.6f}".format(*component_rmse)
    )
    print("三维位置 RMSE [m]: {:.6f}".format(np.sqrt(np.mean(error_norm**2))))
    print("三维位置 MAE  [m]: {:.6f}".format(np.mean(error_norm)))
    print("三维位置 MAX  [m]: {:.6f}".format(np.max(error_norm)))


def validate_config() -> None:
    if ALIGNMENT not in ("none", "origin", "se3", "sim3"):
        raise ValueError("ALIGNMENT 必须是 none、origin、se3 或 sim3")
    if MAX_TIME_GAP < 0.0:
        raise ValueError("MAX_TIME_GAP 不能为负数")
    if IMAGE_DPI <= 0:
        raise ValueError("IMAGE_DPI 必须为正数")
    if IMAGE_FORMAT not in ("png", "pdf", "svg"):
        raise ValueError("IMAGE_FORMAT 必须是 png、pdf 或 svg")
    if not OUTPUT_PREFIX:
        raise ValueError("OUTPUT_PREFIX 不能为空")


def main() -> int:
    validate_config()

    if not SHOW_FIGURES:
        import matplotlib

        matplotlib.use("Agg")
    from matplotlib import pyplot

    configure_plot_style()

    estimate_path = Path(ESTIMATE_PATH).expanduser()
    ground_truth_path = Path(GROUND_TRUTH_PATH).expanduser()
    estimate = load_tum(estimate_path)
    ground_truth = load_tum(ground_truth_path)

    timestamps, estimate_positions, ground_truth_positions = interpolate_ground_truth(
        estimate, ground_truth, TIME_OFFSET, MAX_TIME_GAP
    )
    alignment = calculate_alignment(estimate_positions, ground_truth_positions, ALIGNMENT)
    aligned_estimate_positions = alignment.apply(estimate_positions)
    errors = aligned_estimate_positions - ground_truth_positions
    relative_time = timestamps - timestamps[0]

    output_dir = Path(OUTPUT_DIR).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_path = output_dir / "{}_trajectory.{}".format(
        OUTPUT_PREFIX, IMAGE_FORMAT
    )
    error_path = output_dir / "{}_position_error.{}".format(
        OUTPUT_PREFIX, IMAGE_FORMAT
    )

    trajectory_figure = draw_trajectory_figure(
        pyplot,
        aligned_estimate_positions,
        ground_truth_positions,
        ESTIMATE_LABEL,
        ALIGNMENT,
    )
    error_figure = draw_error_figure(pyplot, relative_time, errors, ESTIMATE_LABEL)
    trajectory_figure.savefig(trajectory_path, dpi=IMAGE_DPI, bbox_inches="tight")
    error_figure.savefig(error_path, dpi=IMAGE_DPI, bbox_inches="tight")

    print_statistics(errors, len(timestamps), alignment)
    print("轨迹图: {}".format(trajectory_path.resolve()))
    print("误差图: {}".format(error_path.resolve()))

    if SHOW_FIGURES:
        pyplot.show()
    else:
        pyplot.close(trajectory_figure)
        pyplot.close(error_figure)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("错误: {}".format(error), file=sys.stderr)
        sys.exit(2)
