#!/usr/bin/env python3
"""Visualize saved BIEVR-LIO pole extraction results with Open3D."""

import argparse
from pathlib import Path
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError as error:
    raise SystemExit("缺少 open3d，请先安装：pip3 install open3d") from error


ROOT = Path(__file__).resolve().parents[1]


def read_table(path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"找不到结果文件：{path}")
    with path.open("r", encoding="utf-8") as stream:
        header = stream.readline().split()
        rows = []
        for line in stream:
            values = line.split()
            if values:
                rows.append(dict(zip(header, values)))
    return rows


def load_cloud(path: Path, color, voxel_size=0.0):
    cloud = o3d.io.read_point_cloud(str(path))
    if not cloud.has_points():
        raise RuntimeError(f"点云为空或无法读取：{path}")
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    cloud.paint_uniform_color(color)
    return cloud


def rotation_from_z(axis):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    z_axis = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z_axis, axis)
    sine = np.linalg.norm(cross)
    cosine = float(np.dot(z_axis, axis))
    if sine < 1e-10:
        return np.eye(3) if cosine >= 0.0 else np.diag([1.0, -1.0, -1.0])
    skew = np.array(
        [[0.0, -cross[2], cross[1]],
         [cross[2], 0.0, -cross[0]],
         [-cross[1], cross[0], 0.0]]
    )
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / (sine * sine))


def landmark_geometries(row):
    center = np.array([float(row["X"]), float(row["Y"]), float(row["Z"])])
    axis = np.array(
        [float(row.get("AxisX", 0.0)),
         float(row.get("AxisY", 0.0)),
         float(row.get("AxisZ", 1.0))]
    )
    axis /= max(np.linalg.norm(axis), 1e-12)
    radius = max(0.05, float(row.get("Radius", 0.20)))
    axial_min = float(row.get("AxialMin", 0.0))
    axial_max = float(row.get("AxialMax", 10.0))
    height = max(0.20, axial_max - axial_min)
    lower = center + axial_min * axis
    upper = center + axial_max * axis
    middle = 0.5 * (lower + upper)

    cylinder = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=height, resolution=24, split=4
    )
    cylinder.compute_vertex_normals()
    cylinder.rotate(rotation_from_z(axis), center=(0.0, 0.0, 0.0))
    cylinder.translate(middle)
    cylinder.paint_uniform_color([0.10, 0.95, 0.25])

    base = o3d.geometry.TriangleMesh.create_sphere(radius=0.22, resolution=16)
    base.compute_vertex_normals()
    base.translate(center)
    base.paint_uniform_color([1.0, 0.18, 0.03])

    axis_line = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([lower, upper]),
        lines=o3d.utility.Vector2iVector([[0, 1]]),
    )
    axis_line.colors = o3d.utility.Vector3dVector([[1.0, 0.82, 0.05]])
    return cylinder, base, axis_line


def rail_geometries(rows):
    geometries = []
    for segment in sorted({int(row["Segment"]) for row in rows}):
        samples = sorted(
            (row for row in rows if int(row["Segment"]) == segment),
            key=lambda row: int(row["Index"]),
        )
        if len(samples) < 2:
            continue
        left = np.array([[float(row["LeftX"]), float(row["LeftY"]),
                          float(row["LeftZ"])] for row in samples])
        right = np.array([[float(row["RightX"]), float(row["RightY"]),
                           float(row["RightZ"])] for row in samples])
        points = np.vstack([left, right])
        count = len(samples)
        lines = []
        colors = []
        for index in range(count - 1):
            lines.extend([[index, index + 1], [count + index, count + index + 1]])
            colors.extend([[1.0, 0.15, 0.05], [0.05, 0.35, 1.0]])
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        geometries.append(line_set)
    return geometries


def combine_extracted_clouds(paths, voxel_size):
    cloud = o3d.geometry.PointCloud()
    for path in paths:
        part = o3d.io.read_point_cloud(str(path))
        if part.has_points():
            cloud += part
    if not cloud.has_points():
        return None
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    cloud.paint_uniform_color([0.0, 1.0, 1.0])
    return cloud


def export_colored_cloud(path, geometries):
    output = o3d.geometry.PointCloud()
    for geometry in geometries:
        if isinstance(geometry, o3d.geometry.PointCloud):
            output += geometry
        elif isinstance(geometry, o3d.geometry.TriangleMesh):
            output += geometry.sample_points_uniformly(number_of_points=800)
        elif isinstance(geometry, o3d.geometry.LineSet):
            points = np.asarray(geometry.points)
            lines = np.asarray(geometry.lines)
            colors = np.asarray(geometry.colors)
            for index, (first, second) in enumerate(lines):
                samples = np.linspace(points[first], points[second], 20)
                line_cloud = o3d.geometry.PointCloud(
                    o3d.utility.Vector3dVector(samples)
                )
                color = colors[index] if index < len(colors) else [1.0, 0.82, 0.05]
                line_cloud.paint_uniform_color(color)
                output += line_cloud
    if output.has_colors():
        colors = np.asarray(output.colors)
        output.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
    if not o3d.io.write_point_cloud(str(path), output):
        raise RuntimeError(f"无法写出可视化点云：{path}")


def save_overview(path, map_cloud, extracted, landmarks, rails):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("输出总览图需要 matplotlib") from error

    map_points = np.asarray(map_cloud.points) if map_cloud is not None else np.empty((0, 3))
    pole_points = np.asarray(extracted.points) if extracted is not None else np.empty((0, 3))
    centers = np.array(
        [[float(row["X"]), float(row["Y"]), float(row["Z"])] for row in landmarks]
    )
    figure, (top, side) = plt.subplots(2, 1, figsize=(16, 9), constrained_layout=True)
    if len(map_points):
        top.scatter(map_points[:, 0], map_points[:, 1], s=0.15, c="#777777",
                    alpha=0.25, rasterized=True, label="SLAM map")
        side.scatter(map_points[:, 0], map_points[:, 2], s=0.15, c="#777777",
                     alpha=0.20, rasterized=True)
    if len(pole_points):
        top.scatter(pole_points[:, 0], pole_points[:, 1], s=2, c="#00d8ff",
                    alpha=0.9, rasterized=True, label="extracted pole points")
        side.scatter(pole_points[:, 0], pole_points[:, 2], s=2, c="#00d8ff",
                     alpha=0.9, rasterized=True)
    if len(centers):
        top.scatter(centers[:, 0], centers[:, 1], s=80, c="#ff2b16",
                    edgecolors="white", linewidths=1.2, zorder=5,
                    label="pole landmark")
    for segment in sorted({int(row["Segment"]) for row in rails}):
        samples = sorted(
            (row for row in rails if int(row["Segment"]) == segment),
            key=lambda row: int(row["Index"]),
        )
        left = np.array([[float(row["LeftX"]), float(row["LeftY"]),
                          float(row["LeftZ"])] for row in samples])
        right = np.array([[float(row["RightX"]), float(row["RightY"]),
                           float(row["RightZ"])] for row in samples])
        top.plot(left[:, 0], left[:, 1], color="#ff2918", linewidth=1.0)
        top.plot(right[:, 0], right[:, 1], color="#1769ff", linewidth=1.0)
        side.plot(left[:, 0], left[:, 2], color="#ff2918", linewidth=0.8)
        side.plot(right[:, 0], right[:, 2], color="#1769ff", linewidth=0.8)
    for index, (row, center) in enumerate(zip(landmarks, centers)):
        pole_id = int(row.get("ID", index))
        # Stagger compact IDs so dense groups remain legible in a long corridor.
        label_level = index % 4
        top.annotate(f"P{pole_id}", center[:2], xytext=(3, 4 + 9 * label_level),
                     textcoords="offset points", color="#b00000", weight="bold",
                     fontsize=8)
        axis = np.array([float(row.get("AxisX", 0.0)),
                         float(row.get("AxisY", 0.0)),
                         float(row.get("AxisZ", 1.0))])
        axis /= max(np.linalg.norm(axis), 1e-12)
        lower = center + float(row.get("AxialMin", 0.0)) * axis
        upper = center + float(row.get("AxialMax", 10.0)) * axis
        side.plot([lower[0], upper[0]], [lower[2], upper[2]], color="#00d8ff",
                  linewidth=3)
        side.text(upper[0], upper[2] + 0.4 + 1.2 * label_level, f"P{pole_id}",
                  ha="center", color="#b00000", weight="bold", fontsize=8)
    top.set_title("Railway pole + rail-vector map — top view")
    top.set_xlabel("X [m]")
    top.set_ylabel("Y [m]")
    top.axis("equal")
    top.grid(alpha=0.2)
    if len(centers):
        top.legend(loc="best", markerscale=2)
    side.set_title("Side view")
    side.set_xlabel("X [m]")
    side.set_ylabel("Z [m]")
    side.grid(alpha=0.2)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="显示灰色 SLAM 地图、青色支柱点、绿色拟合柱体和红色基点。"
    )
    parser.add_argument("--map", type=Path, default=ROOT / "Log/map.pcd")
    parser.add_argument("--poles", type=Path, default=ROOT / "Log/poles")
    parser.add_argument("--map-voxel", type=float, default=0.20)
    parser.add_argument("--pole-voxel", type=float, default=0.04)
    parser.add_argument("--no-map", action="store_true")
    parser.add_argument("--no-gui", action="store_true", help="只检查和打印结果")
    parser.add_argument("--export", type=Path, help="导出一个带颜色的 PLY/PCD 文件")
    parser.add_argument("--overview", type=Path, help="保存带支柱 ID 的俯视/侧视 PNG")
    args = parser.parse_args()

    landmarks = read_table(args.poles / "landmarks.txt")
    rail_path = args.poles / "rail_vector_map.txt"
    rails = read_table(rail_path) if rail_path.is_file() else []
    keyframe_paths = sorted(args.poles.glob("keyframe_*_poles.pcd"))
    geometries = []
    map_cloud = None
    if not args.no_map:
        map_cloud = load_cloud(args.map, [0.48, 0.48, 0.48], args.map_voxel)
        geometries.append(map_cloud)
    extracted = combine_extracted_clouds(keyframe_paths, args.pole_voxel)
    if extracted is not None:
        geometries.append(extracted)

    geometries.extend(rail_geometries(rails))
    print(f"支柱地标：{len(landmarks)}，轨道控制点：{len(rails)}，"
          f"含支柱点的关键帧：{len(keyframe_paths)}")
    print("ID       X        Y        Z      半径    RMS   观测")
    for index, row in enumerate(landmarks):
        pole_id = int(row.get("ID", index))
        print(
            f"{pole_id:2d} {float(row['X']):8.3f} {float(row['Y']):8.3f} "
            f"{float(row['Z']):8.3f} {float(row.get('Radius', 0.0)):7.3f} "
            f"{float(row.get('FitRMS', 0.0)):6.3f} "
            f"{int(row.get('Observations', 0)):4d}"
        )
        geometries.extend(landmark_geometries(row))

    if args.export:
        args.export.parent.mkdir(parents=True, exist_ok=True)
        export_colored_cloud(args.export, geometries)
        print(f"已导出：{args.export}")
    if args.overview:
        save_overview(args.overview, map_cloud, extracted, landmarks, rails)
        print(f"已保存总览图：{args.overview}")
    if not args.no_gui:
        o3d.visualization.draw_geometries(
            geometries,
            window_name="BIEVR-LIO Railway Pole Extraction",
            width=1440,
            height=900,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
