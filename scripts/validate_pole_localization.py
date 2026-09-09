#!/usr/bin/env python3
"""Offline repeatability and pose-correction test for saved pole landmarks.

The saved landmark table is treated as the mapping run. Detections from another
extraction run are fused in short keyframe windows, a controlled local SE(2)
error is injected into the initial map<-odom correction, and robust one-to-one
pole matching tries to remove it. Ground-truth associations are used only for
scoring, never by the optimizer.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError as error:
    raise SystemExit("验证脚本需要 scipy：pip3 install scipy") from error


def read_table(path):
    if not path.is_file():
        raise FileNotFoundError(f"找不到数据表：{path}")
    table = np.genfromtxt(path, names=True, dtype=float, encoding="utf-8")
    if table.size == 0:
        raise RuntimeError(f"数据表为空：{path}")
    return np.atleast_1d(table)


def field(table, name, fallback):
    return table[name] if name in table.dtype.names else np.full(len(table), fallback)


def map_landmarks(table):
    points = np.column_stack((table["X"], table["Y"]))
    radii = field(table, "Radius", 0.25)
    observations = field(table, "Observations", 1.0)
    rms = field(table, "FitRMS", 0.10)
    quality = np.maximum(observations, 1.0) / np.maximum(rms * rms, 0.0025)
    return points, radii, quality


def detection_quality(row):
    points = min(float(row["TotalPoints"]), 2000.0)
    support = math.sqrt(max(points, 1.0) / 100.0)
    inlier = float(row["InlierRatio"])
    coverage = float(row["AngularCoverage"])
    rms = float(row["FitRMS"])
    return max(0.01, support * inlier * coverage / max(rms * rms, 0.0025))


def fuse_window(rows, merge_radius, radius_tolerance):
    """Fuse repeated observations inside a local window in odom coordinates."""
    clusters = []
    for row in rows:
        point = np.array([row["X"], row["Y"]], dtype=float)
        radius = float(row["CylinderRadius"])
        weight = detection_quality(row)
        best = None
        best_distance = merge_radius
        for index, cluster in enumerate(clusters):
            distance = np.linalg.norm(point - cluster["point"])
            if (distance <= best_distance and
                    abs(radius - cluster["radius"]) <= radius_tolerance):
                best = index
                best_distance = distance
        if best is None:
            clusters.append({"point": point, "radius": radius, "weight": weight})
            continue
        cluster = clusters[best]
        total = cluster["weight"] + weight
        cluster["point"] = (cluster["weight"] * cluster["point"] + weight * point) / total
        cluster["radius"] = (
            cluster["weight"] * cluster["radius"] + weight * radius
        ) / total
        cluster["weight"] = total
    points = np.array([cluster["point"] for cluster in clusters], dtype=float)
    radii = np.array([cluster["radius"] for cluster in clusters], dtype=float)
    quality = np.array([cluster["weight"] for cluster in clusters], dtype=float)
    return points, radii, quality


def transform_points(points, yaw, translation):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + translation


def assign_one_to_one(query, query_radii, target, target_radii,
                      distance_gate, radius_gate):
    if len(query) == 0 or len(target) == 0:
        return np.empty((0, 2), dtype=int)
    delta = query[:, None, :] - target[None, :, :]
    distance = np.linalg.norm(delta, axis=2)
    radius_delta = np.abs(query_radii[:, None] - target_radii[None, :])
    valid = (distance <= distance_gate) & (radius_delta <= radius_gate)
    # Radius is an association cue, not a pose residual. Keep its cost weaker
    # than a metre of position error so non-circular railway poles remain usable.
    cost = distance * distance + 0.25 * (radius_delta / radius_gate) ** 2
    cost[~valid] = 1.0e9
    query_index, target_index = linear_sum_assignment(cost)
    keep = cost[query_index, target_index] < 1.0e8
    return np.column_stack((query_index[keep], target_index[keep]))


def weighted_se2(query, target, weights):
    weights = np.asarray(weights, dtype=float)
    weights /= max(np.sum(weights), 1.0e-12)
    query_mean = np.sum(query * weights[:, None], axis=0)
    target_mean = np.sum(target * weights[:, None], axis=0)
    query_centered = query - query_mean
    target_centered = target - target_mean
    dot = np.sum(weights * np.sum(query_centered * target_centered, axis=1))
    cross = np.sum(
        weights * (query_centered[:, 0] * target_centered[:, 1] -
                   query_centered[:, 1] * target_centered[:, 0])
    )
    yaw = math.atan2(cross, dot)
    transformed_mean = transform_points(query_mean[None, :], yaw, np.zeros(2))[0]
    return yaw, target_mean - transformed_mean


def refine_pose(query, query_radii, query_quality, target, target_radii,
                target_quality, initial_yaw, initial_translation, distance_gate,
                radius_gate, huber_delta, iterations, min_inliers):
    yaw = initial_yaw
    translation = initial_translation.copy()
    pairs = np.empty((0, 2), dtype=int)
    for _ in range(iterations):
        predicted = transform_points(query, yaw, translation)
        pairs = assign_one_to_one(predicted, query_radii, target, target_radii,
                                  distance_gate, radius_gate)
        if len(pairs) < min_inliers:
            return None
        source = query[pairs[:, 0]]
        reference = target[pairs[:, 1]]
        residual = np.linalg.norm(predicted[pairs[:, 0]] - reference, axis=1)
        robust = np.minimum(1.0, huber_delta / np.maximum(residual, 1.0e-6))
        quality = np.sqrt(
            query_quality[pairs[:, 0]] * target_quality[pairs[:, 1]]
        )
        quality /= max(np.median(quality), 1.0e-12)
        updated_yaw, updated_translation = weighted_se2(source, reference,
                                                        robust * quality)
        if (abs(updated_yaw - yaw) < 1.0e-8 and
                np.linalg.norm(updated_translation - translation) < 1.0e-6):
            yaw, translation = updated_yaw, updated_translation
            break
        yaw, translation = updated_yaw, updated_translation

    predicted = transform_points(query, yaw, translation)
    pairs = assign_one_to_one(predicted, query_radii, target, target_radii,
                              distance_gate, radius_gate)
    if len(pairs) < min_inliers:
        return None
    baseline = np.ptp(query[pairs[:, 0]], axis=0)
    if np.linalg.norm(baseline) < 10.0:
        return None
    residuals = np.linalg.norm(
        predicted[pairs[:, 0]] - target[pairs[:, 1]], axis=1
    )
    return {
        "yaw": yaw,
        "translation": translation,
        "pairs": pairs,
        "rms": float(np.sqrt(np.mean(residuals * residuals))),
        "max_residual": float(np.max(residuals)),
    }


def angle_error_degrees(angle):
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return abs(math.degrees(wrapped))


def percentile(values, q):
    return float(np.percentile(np.asarray(values), q)) if values else None


def summarize(values):
    return {
        "mean": float(np.mean(values)) if values else None,
        "median": percentile(values, 50),
        "p95": percentile(values, 95),
        "max": float(np.max(values)) if values else None,
    }


def validate(args):
    map_table = read_table(args.map)
    detection_table = read_table(args.detections)
    target, target_radii, target_quality = map_landmarks(map_table)
    first_keyframe = int(np.min(detection_table["Keyframe"]))
    last_keyframe = int(np.max(detection_table["Keyframe"]))

    windows = []
    for start in range(first_keyframe,
                       last_keyframe - args.window_keyframes + 2):
        selected = detection_table[
            (detection_table["Keyframe"] >= start) &
            (detection_table["Keyframe"] < start + args.window_keyframes)
        ]
        query, query_radii, query_quality = fuse_window(
            selected, args.local_merge_radius, args.radius_gate
        )
        truth = assign_one_to_one(query, query_radii, target, target_radii,
                                  args.truth_distance, args.radius_gate)
        if len(truth) < args.min_inliers:
            continue
        if np.linalg.norm(np.ptp(query[truth[:, 0]], axis=0)) < 10.0:
            continue
        windows.append((start, query, query_radii, query_quality, truth))
    if not windows:
        raise RuntimeError("没有找到包含足够可对应支柱的验证窗口")

    generator = np.random.default_rng(args.seed)
    before_rmse = []
    after_rmse = []
    position_error = []
    yaw_error = []
    precision = []
    recall = []
    inlier_counts = []
    solved = 0
    successful = 0

    for _ in range(args.trials):
        _, query, query_radii, query_quality, truth = windows[
            generator.integers(0, len(windows))
        ]
        centre = np.mean(query[truth[:, 0]], axis=0)
        error_yaw = math.radians(generator.uniform(-args.max_yaw_error_deg,
                                                    args.max_yaw_error_deg))
        error_distance = args.max_position_error_m * math.sqrt(generator.uniform())
        error_direction = generator.uniform(-math.pi, math.pi)
        error_translation = error_distance * np.array(
            [math.cos(error_direction), math.sin(error_direction)]
        )
        cosine = math.cos(error_yaw)
        sine = math.sin(error_yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        # Apply the initial yaw error around the current local-map centre. This
        # describes tracking error at the vehicle, independent of map origin.
        initial_translation = centre + error_translation - rotation @ centre
        initial = transform_points(query, error_yaw, initial_translation)
        truth_query = truth[:, 0]
        truth_target = truth[:, 1]
        before_residual = initial[truth_query] - target[truth_target]
        before_rmse.append(float(np.sqrt(np.mean(np.sum(before_residual ** 2, axis=1)))))

        result = refine_pose(
            query, query_radii, query_quality, target, target_radii, target_quality,
            error_yaw, initial_translation, args.match_distance, args.radius_gate,
            args.huber_delta, args.iterations, args.min_inliers
        )
        if result is None:
            continue
        solved += 1
        predicted = transform_points(query, result["yaw"], result["translation"])
        after_residual = predicted[truth_query] - target[truth_target]
        current_after_rmse = float(
            np.sqrt(np.mean(np.sum(after_residual ** 2, axis=1)))
        )
        after_rmse.append(current_after_rmse)
        corrected_centre = transform_points(
            centre[None, :], result["yaw"], result["translation"]
        )[0]
        position_error.append(float(np.linalg.norm(corrected_centre - centre)))
        yaw_error.append(angle_error_degrees(result["yaw"]))

        truth_set = {tuple(pair) for pair in truth.tolist()}
        estimated_set = {tuple(pair) for pair in result["pairs"].tolist()}
        correct = len(truth_set & estimated_set)
        current_precision = correct / max(len(estimated_set), 1)
        current_recall = correct / len(truth_set)
        precision.append(current_precision)
        recall.append(current_recall)
        inlier_counts.append(len(result["pairs"]))
        if (current_precision >= 0.90 and current_recall >= 0.80 and
                current_after_rmse <= 0.50 and
                position_error[-1] <= 0.50 and yaw_error[-1] <= 0.50):
            successful += 1

    summary = {
        "map_landmarks": int(len(target)),
        "eligible_windows": int(len(windows)),
        "window_keyframes": args.window_keyframes,
        "trials": args.trials,
        "solved": solved,
        "solve_rate": solved / args.trials,
        "strict_successes": successful,
        "strict_success_rate": successful / args.trials,
        "injected_max_position_error_m": args.max_position_error_m,
        "injected_max_yaw_error_deg": args.max_yaw_error_deg,
        "before_truth_rmse_m": summarize(before_rmse),
        "after_truth_rmse_m": summarize(after_rmse),
        "local_position_error_m": summarize(position_error),
        "yaw_error_deg": summarize(yaw_error),
        "association_precision": summarize(precision),
        "association_recall": summarize(recall),
        "accepted_matches": summarize(inlier_counts),
    }
    return summary


def main():
    parser = argparse.ArgumentParser(
        description="向已保存支柱地图注入定位误差，验证局部支柱匹配与 SE(2) 纠偏。"
    )
    parser.add_argument("--map", type=Path, required=True,
                        help="首次建图保存的 landmarks.txt")
    parser.add_argument("--detections", type=Path, required=True,
                        help="第二次运行的 detections.txt")
    parser.add_argument("--window-keyframes", type=int, default=7)
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--seed", type=int, default=20260903)
    parser.add_argument("--max-position-error-m", type=float, default=2.0)
    parser.add_argument("--max-yaw-error-deg", type=float, default=1.0)
    parser.add_argument("--match-distance", type=float, default=4.0)
    parser.add_argument("--truth-distance", type=float, default=1.0)
    parser.add_argument("--radius-gate", type=float, default=0.30)
    parser.add_argument("--local-merge-radius", type=float, default=1.0)
    parser.add_argument("--huber-delta", type=float, default=0.50)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--min-inliers", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.window_keyframes < 1 or args.trials < 1 or args.min_inliers < 2:
        parser.error("窗口、试验次数和最小内点数必须为正数")
    summary = validate(args)
    encoded = json.dumps(summary, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
        print(f"已保存：{args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
