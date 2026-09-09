#!/usr/bin/env python3
"""Extract Xiangyang GNSS references and evaluate metric LIO positions.

Run under the ROS1 Python environment. No clock-offset or scale fitting is used.
NavSatFix is the primary reference; gnss_pose is an auxiliary receiver solution.
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np


def enu(lla, origin):
    def ecef(values):
        lat, lon = np.deg2rad(values[:, 0]), np.deg2rad(values[:, 1])
        h = values[:, 2]
        e2 = 6.6943799901413165e-3
        n = 6378137.0 / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        return np.column_stack(((n+h)*np.cos(lat)*np.cos(lon),
                                (n+h)*np.cos(lat)*np.sin(lon),
                                (n*(1-e2)+h)*np.sin(lat)))
    lat, lon = np.deg2rad(origin[:2])
    rotation = np.array([[-np.sin(lon), np.cos(lon), 0],
                         [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
                         [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]])
    return (ecef(lla)-ecef(np.asarray(origin)[None, :])) @ rotation.T


def extract(bag_path, directory):
    import rosbag
    directory.mkdir(parents=True, exist_ok=True)
    fixes, poses, offsets = [], [], collections.defaultdict(list)
    with rosbag.Bag(str(bag_path)) as bag:
        info = {'bag': str(bag_path), 'start': bag.get_start_time(),
                'end': bag.get_end_time(), 'topics': {k: v._asdict() for k, v in
                                                    bag.get_type_and_topic_info().topics.items()}}
        for topic, msg, stamp in bag.read_messages(topics=['/gnss', '/gnss_pose']):
            t = msg.header.stamp.to_sec()
            offsets[topic].append(stamp.to_sec()-t)
            if topic == '/gnss':
                fixes.append([t, msg.latitude, msg.longitude, msg.altitude, msg.status.status,
                              msg.position_covariance[0], msg.position_covariance[4],
                              msg.position_covariance[8], msg.position_covariance_type])
            else:
                p = msg.pose.position
                poses.append([t, p.x, p.y, p.z])
    fixes = np.asarray(fixes)
    if not len(fixes):
        raise ValueError('No /gnss fixes in '+str(bag_path))
    valid = np.all(np.isfinite(fixes[:, :4]), axis=1) & (fixes[:, 4] >= 0)
    valid &= (np.abs(fixes[:, 1]) <= 90) & (np.abs(fixes[:, 2]) <= 180)
    good = fixes[valid]
    if len(good) < 3:
        raise ValueError('Insufficient valid GNSS fixes')
    origin = good[0, 1:4]
    np.savetxt(directory/'gnss_raw.txt', fixes, header='stamp lat lon altitude status var_e var_n var_u covariance_type')
    for name, data in [('gnss', good[:, :4]), ('gnss_pose', np.asarray(poses))]:
        if not len(data):
            continue
        data = data[np.all(np.isfinite(data), axis=1)]
        positions = enu(data[:, 1:4], origin)
        tum = np.column_stack((data[:, 0], positions, np.zeros((len(data), 3)), np.ones(len(data))))
        np.savetxt(directory/(name+'_enu_tum.txt'), tum, fmt='%.9f',
                   header='timestamp east north up qx qy qz qw; identity quaternion is a placeholder')
    info.update(origin_lla=origin.tolist(), fixes=len(fixes), valid_fixes=len(good),
                status_counts=dict(collections.Counter(str(int(s)) for s in fixes[:, 4])),
                median_reported_sigma_m=np.sqrt(np.maximum(0, np.median(good[:, 5:8], axis=0))).tolist(),
                receipt_minus_header_seconds={k: {'median': float(np.median(v)),
                                                  'min': float(np.min(v)), 'max': float(np.max(v))}
                                              for k, v in offsets.items()})
    (directory/'metadata.json').write_text(json.dumps(info, indent=2, ensure_ascii=False))
    return info


def unique_sorted(data):
    data = data[np.all(np.isfinite(data), axis=1)]
    data = data[np.argsort(data[:, 0], kind='stable')]
    _, indices = np.unique(data[:, 0], return_index=True)
    return data[indices]


def stats(values):
    return dict(rmse=float(np.sqrt(np.mean(values**2))), mean=float(np.mean(values)),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values)))


def align(source, target, yaw_only=False):
    a, b = source.mean(axis=0), target.mean(axis=0)
    rotation = np.eye(3)
    dimension = 2 if yaw_only else 3
    u, _, vt = np.linalg.svd((target-b)[:, :dimension].T @ (source-a)[:, :dimension])
    signs = np.ones(dimension)
    signs[-1] = np.linalg.det(u@vt)
    rotation[:dimension, :dimension] = u @ np.diag(signs) @ vt
    return rotation, b-rotation@a


def evaluate(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    estimate = unique_sorted(np.loadtxt(directory/'odometry_tum.txt', ndmin=2))
    result = {'estimate_frames': len(estimate), 'scale': 1.0, 'time_offset_s': 0.0,
              'reference': '/gnss NavSatFix; WGS84 ECEF to local ENU',
              'limitations': ['No antenna lever arm', 'Identity lidar-IMU extrinsic',
                              'Receiver fix status does not identify RTK fixed explicitly',
                              'Each bag initializes the estimator independently']}
    for name in ['gnss', 'gnss_pose']:
        path = directory/(name+'_enu_tum.txt')
        if not path.exists():
            continue
        ref = unique_sorted(np.loadtxt(path, ndmin=2))
        # Evaluate at reference epochs, avoiding artificial replication of 4Hz fixes.
        q = ref[:, 0]
        right = np.searchsorted(estimate[:, 0], q)
        left = np.clip(right-1, 0, len(estimate)-1)
        right = np.clip(right, 0, len(estimate)-1)
        mask = (q >= estimate[0, 0]) & (q <= estimate[-1, 0])
        mask &= (estimate[right, 0]-estimate[left, 0]) <= 0.3
        q, target = q[mask], ref[mask, 1:4]
        if len(q) < 3:
            result[name] = {'error': 'Insufficient matched positions'}
            continue
        source = np.column_stack([np.interp(q-estimate[0, 0], estimate[:, 0]-estimate[0, 0],
                                           estimate[:, i]) for i in range(1, 4)])
        distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(target, axis=0), axis=1))]
        speeds = np.linalg.norm(np.diff(target, axis=0), axis=1)/np.diff(q)
        metrics = {'matched': len(q), 'duration_s': float(q[-1]-q[0]),
                   'reference_distance_m': float(distance[-1]),
                   'reference_speed_mps': stats(speeds),
                   'initial_2s_median_speed_mps': float(np.median(speeds[(q[1:]-q[0]) <= 2])),
                   'reference_max_gap_s': float(np.max(np.diff(q))),
                   'reference_position_singular_values': np.linalg.svd(target-target.mean(axis=0),
                                                                         compute_uv=False).tolist()}
        for method in ['se3', 'yaw_translation']:
            r, t = align(source, target, method == 'yaw_translation')
            aligned = source@r.T+t
            errors = aligned-target
            metrics[method] = {'horizontal_m': stats(np.linalg.norm(errors[:, :2], axis=1)),
                               'three_dimensional_m': stats(np.linalg.norm(errors, axis=1)),
                               'vertical_abs_m': stats(np.abs(errors[:, 2])),
                               'endpoint_error_m': float(np.linalg.norm(errors[-1])),
                               'rotation': r.tolist(), 'translation': t.tolist()}
            moving = np.r_[speeds[0], speeds] > 0.5
            if np.any(moving):
                metrics[method]['moving_only'] = {
                    'samples': int(moving.sum()),
                    'horizontal_m': stats(np.linalg.norm(errors[moving, :2], axis=1)),
                    'three_dimensional_m': stats(np.linalg.norm(errors[moving], axis=1))}
            # Relative displacement error over fixed traveled distances; no extra alignment.
            relative = {}
            for length in [100, 500, 1000]:
                starts = np.arange(0, len(q), max(1, int(len(q)/200)))
                ends = np.searchsorted(distance, distance[starts]+length)
                ok = ends < len(q)
                a, b = starts[ok], ends[ok]
                if len(a):
                    e = (aligned[b]-aligned[a])-(target[b]-target[a])
                    relative[str(length)] = {'segments': len(a), 'horizontal_m': stats(np.linalg.norm(e[:, :2], axis=1)),
                                             'three_dimensional_m': stats(np.linalg.norm(e, axis=1))}
            metrics[method]['relative_displacement'] = relative
            if name == 'gnss':
                np.savetxt(directory/(method+'_errors.csv'),
                           np.column_stack((q, aligned, target, errors)), delimiter=',',
                           header='stamp,est_e,est_n,est_u,ref_e,ref_n,ref_u,error_e,error_n,error_u')
                fig, axes = plt.subplots(1, 2, figsize=(13, 5))
                axes[0].plot(target[:, 0], target[:, 1], label='GNSS', linewidth=1.5)
                axes[0].plot(aligned[:, 0], aligned[:, 1], label='LIO', linewidth=1)
                axes[0].axis('equal')
                axes[0].set(xlabel='East [m]', ylabel='North [m]', title=directory.name+' / '+method)
                axes[1].plot(q-q[0], np.linalg.norm(errors[:, :2], axis=1), label='Horizontal')
                axes[1].plot(q-q[0], errors[:, 2], label='Vertical (signed)')
                axes[1].set(xlabel='Time [s]', ylabel='Error [m]')
                for axis in axes:
                    axis.grid(alpha=.3)
                    axis.legend()
                fig.tight_layout()
                fig.savefig(directory/(method+'_comparison.png'), dpi=170)
                plt.close(fig)
        result[name] = metrics
    if (directory/'gnss_pose_enu_tum.txt').exists():
        raw = unique_sorted(np.loadtxt(directory/'gnss_enu_tum.txt', ndmin=2))
        fused = unique_sorted(np.loadtxt(directory/'gnss_pose_enu_tum.txt', ndmin=2))
        mask = (raw[:, 0] >= fused[0, 0]) & (raw[:, 0] <= fused[-1, 0])
        q = raw[mask, 0]-fused[0, 0]
        interp = np.column_stack([np.interp(q, fused[:, 0]-fused[0, 0], fused[:, i]) for i in range(1, 4)])
        result['gnss_pose_vs_gnss_without_alignment'] = stats(np.linalg.norm(interp-raw[mask, 1:4], axis=1))
    (directory/'metrics.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--extract-only', action='store_true')
    args = parser.parse_args()
    if args.bag:
        print(json.dumps(extract(args.bag, args.output), ensure_ascii=False), flush=True)
    if not args.extract_only:
        print(json.dumps(evaluate(args.output)), flush=True)


if __name__ == '__main__':
    main()
