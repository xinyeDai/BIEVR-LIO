#!/usr/bin/env python3
"""Evaluate a TUM trajectory directly in an existing reference/map frame."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_tum(path):
    values = np.loadtxt(path, ndmin=2)
    values = values[np.all(np.isfinite(values[:, :8]), axis=1)]
    values = values[np.argsort(values[:, 0], kind="stable")]
    _, indices = np.unique(values[:, 0], return_index=True)
    return values[indices]


def summary(values):
    values = np.asarray(values)
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def evaluate(estimate, reference, max_time_gap, align_first):
    right = np.searchsorted(reference[:, 0], estimate[:, 0])
    right = np.clip(right, 1, len(reference) - 1)
    left = right - 1
    interval = reference[right, 0] - reference[left, 0]
    alpha = (estimate[:, 0] - reference[left, 0]) / interval
    valid = (estimate[:, 0] >= reference[0, 0]) & (estimate[:, 0] <= reference[-1, 0])
    valid &= np.minimum(np.abs(estimate[:, 0] - reference[left, 0]),
                        np.abs(reference[right, 0] - estimate[:, 0])) <= max_time_gap
    estimate = estimate[valid]
    left, right, alpha = left[valid], right[valid], alpha[valid]
    target = ((1.0 - alpha[:, None]) * reference[left, 1:4] +
              alpha[:, None] * reference[right, 1:4])
    positions = estimate[:, 1:4].copy()

    # Orientation scoring uses the nearest reference sample. Both trajectories
    # normally originate from the same LIO frame stamps, making this exact.
    nearest = np.where(alpha <= 0.5, left, right)
    estimate_rotations = Rotation.from_quat(estimate[:, 4:8])
    reference_rotations = Rotation.from_quat(reference[nearest, 4:8])
    if align_first:
        correction = reference_rotations[0] * estimate_rotations[0].inv()
        positions = correction.apply(positions) + target[0] - correction.apply(positions[0])
        estimate_rotations = correction * estimate_rotations
    residual = positions - target
    rotation_error = reference_rotations.inv() * estimate_rotations
    rotation_components = rotation_error.as_euler("zyx", degrees=True)
    yaw_error = np.abs(rotation_components[:, 0])
    yaw_error = np.minimum(yaw_error, 360.0 - yaw_error)
    pitch_error = np.abs(rotation_components[:, 1])
    roll_error = np.abs(rotation_components[:, 2])
    tilt_error = np.linalg.norm(rotation_components[:, 1:3], axis=1)
    distance = np.sum(np.linalg.norm(np.diff(target[:, :2], axis=0), axis=1))
    return {
        "matched_poses": int(len(estimate)),
        "duration_s": float(estimate[-1, 0] - estimate[0, 0]),
        "reference_horizontal_distance_m": float(distance),
        "alignment": "first_pose_se3" if align_first else "none",
        "horizontal_error_m": summary(np.linalg.norm(residual[:, :2], axis=1)),
        "three_dimensional_error_m": summary(np.linalg.norm(residual, axis=1)),
        "vertical_abs_error_m": summary(np.abs(residual[:, 2])),
        "yaw_abs_error_deg": summary(yaw_error),
        "roll_abs_error_deg": summary(roll_error),
        "pitch_abs_error_deg": summary(pitch_error),
        "tilt_error_deg": summary(tilt_error),
        "endpoint_horizontal_error_m": float(np.linalg.norm(residual[-1, :2])),
        "endpoint_vertical_abs_error_m": float(np.abs(residual[-1, 2])),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-time-gap", type=float, default=0.11)
    parser.add_argument("--align-first", action="store_true",
                        help="Apply only the first-pose SE(3) transform (no trajectory fit).")
    args = parser.parse_args()
    result = evaluate(read_tum(args.estimate), read_tum(args.reference),
                      args.max_time_gap, args.align_first)
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
