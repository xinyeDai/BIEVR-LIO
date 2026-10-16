#!/usr/bin/env python3
"""Integration smoke test; source devel_isolated/setup.bash first.

Starts an isolated ROS master, publishes synthetic fixes, verifies live files,
then checks archive-based map reconstruction and trajectory-only optimization.
"""
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time


def rows(path):
    return [line.split() for line in path.read_text().splitlines()
            if line.strip() and not line.startswith('#')]


def wait_until(predicate, process=None):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if predicate():
            return
        if process is not None and process.poll() is not None:
            raise AssertionError('Process exited before becoming ready')
        time.sleep(0.05)
    raise AssertionError('Timed out')


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    processes = []
    with tempfile.TemporaryDirectory(prefix='bievr-gnss-test-') as temp:
        directory = Path(temp)
        with (directory / 'ros.log').open('w') as log:
            try:
                master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log)
                processes.append(master)
                import rosgraph
                wait_until(lambda: rosgraph.is_master_online(), master)
                import rospy
                from sensor_msgs.msg import NavSatFix, NavSatStatus
                from geometry_msgs.msg import PoseStamped
                rospy.init_node('gnss_recording_smoke', disable_signals=True)
                publisher = rospy.Publisher('/test/fix', NavSatFix, queue_size=100)
                pose_publisher = rospy.Publisher('/gnss_pose', PoseStamped, queue_size=100)
                disabled_directory = directory / 'disabled'
                disabled = subprocess.run(
                    ['rosrun', 'bievr_lio_ros', 'gnss_recorder',
                     '_gnss_recorder/enable:=false',
                     '_output_directory:=' + str(disabled_directory)],
                    stdout=log, stderr=log, timeout=10)
                assert disabled.returncode == 0
                assert not disabled_directory.exists()
                rospy.delete_param('/bievr_gnss_recorder/gnss_recorder/enable')
                recorder_cmd = ['rosrun', 'bievr_lio_ros', 'gnss_recorder',
                                '_gnss_recorder/gnss_topic:=/test/fix', '_output_directory:=' + temp]
                recorder = subprocess.Popen(recorder_cmd, stdout=log, stderr=log)
                processes.append(recorder)
                wait_until(lambda: publisher.get_num_connections() > 0, recorder)
                fixes = []
                for index in range(22):
                    fix = NavSatFix()
                    fix.header.stamp = rospy.Time(1700000000 + index, 123456789)
                    fix.latitude = 0
                    fix.longitude = index * 0.00001
                    fix.altitude = 10
                    fix.status.status = NavSatStatus.STATUS_FIX
                    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    fix.position_covariance = [0.04, 0, 0, 0, 0.09, 0, 0, 0, 0.16]
                    if index == 0:
                        fix.status.status = NavSatStatus.STATUS_NO_FIX
                    if index == 1:
                        fix.latitude = float('nan')
                    if index == 2:
                        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    if index == 3:
                        fix.position_covariance[0] = -1
                    fixes.append(fix)
                    publisher.publish(fix)
                    time.sleep(0.02)
                wait_until(lambda: (directory / 'gnss_raw.txt').exists() and
                           len(rows(directory / 'gnss_raw.txt')) == 22, recorder)
                wait_until(lambda: len(rows(directory / 'gnss_enu.txt')) == 20, recorder)
                enu = rows(directory / 'gnss_enu.txt')
                assert enu[0][0] == '1700000002.123456789'
                assert [float(v) for v in enu[0][1:4]] == [0, 0, 0]
                assert [float(v) for v in enu[0][4:]] == [1, 1, 4]
                assert [float(v) for v in enu[1][4:]] == [1, 1, 4]
                assert [float(v) for v in enu[2][4:]] == [0.04, 0.09, 0.16]
                assert 1.11 < float(enu[1][1]) < 1.12
                assert len(rows(directory / 'gnss_enu_tum.txt')) == 20
                assert '# origin_lla ' in (directory / 'gnss_enu.txt').read_text()
                wait_until(lambda: pose_publisher.get_num_connections() > 0, recorder)
                pose = PoseStamped()
                pose.header.stamp = rospy.Time(1700000003, 987654321)
                pose.header.frame_id = 'imu_link'
                pose.pose.position.x = 0
                pose.pose.position.y = .00003
                pose.pose.position.z = 10
                pose.pose.orientation.z = .6
                pose.pose.orientation.w = .8
                pose_publisher.publish(pose)
                wait_until(lambda: len(rows(directory / 'gnss_pose_enu.txt')) == 1, recorder)
                pose_enu = rows(directory / 'gnss_pose_enu.txt')[0]
                assert pose_enu[0] == '1700000003.987654321'
                assert 1.11 < float(pose_enu[1]) < 1.12  # shares /gnss origin, not its own
                assert [float(v) for v in pose_enu[4:]] == [1, 1, 4]
                assert [float(v) for v in rows(directory / 'gnss_pose_raw.txt')[0][4:]] == [0, 0, .6, .8]
                assert rows(directory / 'gnss_pose_enu_tum.txt')[0][4:] == ['0', '0', '0', '1']
                for name in ['gnss_enu.txt', 'gnss_enu_tum.txt',
                             'gnss_pose_enu.txt', 'gnss_pose_enu_tum.txt']:
                    origin = next(line for line in (directory / name).read_text().splitlines()
                                  if line.startswith('# origin_lla '))
                    assert origin == '# origin_lla 0.000000000000 0.000020000000 10.000000000000'
                pose.pose.position.x = 91  # invalid latitude: raw only
                pose_publisher.publish(pose)
                wait_until(lambda: len(rows(directory / 'gnss_pose_raw.txt')) == 2, recorder)
                assert len(rows(directory / 'gnss_pose_enu.txt')) == 1
                pose.pose.position.x = 0
                recorder.send_signal(signal.SIGINT)
                assert recorder.wait(timeout=10) == 0
                refused = subprocess.run(recorder_cmd, stdout=log, stderr=log, timeout=10)
                assert refused.returncode != 0
                assert rows(directory / 'gnss_enu.txt') == enu
                # A configured origin is retained even when the first fix is elsewhere.
                wait_until(lambda: publisher.get_num_connections() == 0)
                fixed_directory = directory / 'fixed'
                rospy.set_param('/bievr_gnss_recorder/origin_lla', [0.0, 0.0, 10.0])
                fixed = subprocess.Popen(
                    ['rosrun', 'bievr_lio_ros', 'gnss_recorder', '_gnss_recorder/gnss_topic:=/test/fix',
                     '_output_directory:=' + str(fixed_directory)], stdout=log, stderr=log)
                processes.append(fixed)
                wait_until(lambda: publisher.get_num_connections() > 0, fixed)
                publisher.publish(fixes[2])
                wait_until(lambda: (fixed_directory / 'gnss_enu.txt').exists() and
                           len(rows(fixed_directory / 'gnss_enu.txt')) == 1, fixed)
                assert 2.22 < float(rows(fixed_directory / 'gnss_enu.txt')[0][1]) < 2.23
                fixed.send_signal(signal.SIGINT)
                assert fixed.wait(timeout=10) == 0
                rospy.delete_param('/bievr_gnss_recorder/origin_lla')
                # PoseStamped alone must establish an origin without waiting for NavSatFix.
                wait_until(lambda: pose_publisher.get_num_connections() == 0)
                pose_only_directory = directory / 'pose_only'
                pose_only = subprocess.Popen(
                    ['rosrun', 'bievr_lio_ros', 'gnss_recorder',
                     '_gnss_recorder/gnss_topic:=/unused/fix',
                     '_output_directory:=' + str(pose_only_directory)], stdout=log, stderr=log)
                processes.append(pose_only)
                wait_until(lambda: pose_publisher.get_num_connections() > 0, pose_only)
                pose_publisher.publish(pose)
                wait_until(lambda: (pose_only_directory / 'gnss_pose_enu.txt').exists() and
                           len(rows(pose_only_directory / 'gnss_pose_enu.txt')) == 1, pose_only)
                assert [float(v) for v in rows(pose_only_directory / 'gnss_pose_enu.txt')[0][1:4]] == [0, 0, 0]
                assert rows(pose_only_directory / 'gnss_enu.txt') == []
                pose_only.send_signal(signal.SIGINT)
                assert pose_only.wait(timeout=10) == 0

                with (directory / 'lio.txt').open('w') as lio:
                    for row in enu:
                        lio.write('%s %.12f %s %s 0 0 0 1\n' %
                                  (row[0], float(row[1]) + 10, row[2], row[3]))
                with (directory / 'scans.bin').open('wb') as archive:
                    archive.write(b'BIEVRS1\0')
                    for row in enu:
                        sec, nsec = row[0].split('.')
                        archive.write(struct.pack('<QQ', int(sec) * 10**9 + int(nsec), 1))
                        archive.write(struct.pack('<ffff', 0, 2, 0, 42))
                base = ['rosrun', 'bievr_lio_ros', 'gnss_optimize',
                        '--lio_trajectory', str(directory / 'lio.txt'),
                        '--gnss_enu', str(directory / 'gnss_enu.txt')]
                for kind, args in [('map', []), ('trajectory', ['--skip-output-map'])]:
                    run = subprocess.run(base + args + ['--output_directory', str(directory / kind)],
                                         stdout=log, stderr=log, timeout=60)
                    assert run.returncode == 0, (directory / 'ros.log').read_text()
                trajectory = rows(directory / 'map/gnss_optimized_tum.txt')
                assert len(trajectory) == 20
                assert rows(directory / 'trajectory/gnss_optimized_tum.txt') == trajectory
                map_bytes = (directory / 'map/gnss_optimized_map.pcd').read_bytes()
                header, data = map_bytes.split(b'DATA binary\n', 1)
                assert b'FIELDS x y z intensity' in header
                points = sorted(struct.iter_unpack('<ffff', data))
                assert len(points) == 20
                # Body point [0, 2, 0] must follow the optimized pose, not raw odometry (+10m).
                assert abs(points[0][0] - float(trajectory[0][1])) < 1e-4
                assert abs(points[0][1] - (float(trajectory[0][2]) + 2)) < 1e-4
                assert all(point[3] == 42 for point in points)
                import yaml
                summary = yaml.safe_load((directory / 'map/summary.yaml').read_text())
                assert summary['optimized_map'].endswith('gnss_optimized_map.pcd')
                assert summary['scans_already_deskewed'] is True
                assert summary['origin_lla'] == [0, 0.00002, 10]
                assert summary['used_measurements'] > 0
                import json
                evaluation = json.loads((directory / 'map/evaluation_metrics.json').read_text())
                assert evaluation['matched_samples'] == 20
                assert evaluation['lio']['three_dimensional']['rmse_m'] < 1e-5
                assert (directory / 'map/trajectory_xyz.png').stat().st_size > 1000
                assert (directory / 'map/position_error_xyz.png').stat().st_size > 1000
                # Removed bag input must be rejected explicitly.
                assert subprocess.run(base + ['--bag', 'unused.bag'], stdout=log,
                                      stderr=log, timeout=10).returncode == 2
                archive_path = directory / 'scans.bin'
                valid_archive = archive_path.read_bytes()
                archive_path.write_bytes(valid_archive[:-1])
                assert subprocess.run(base + ['--output_directory', str(directory / 'rejected')],
                                      stdout=log, stderr=log, timeout=60).returncode != 0
                archive_path.unlink()
                assert subprocess.run(base + ['--output_directory', str(directory / 'rejected')],
                                      stdout=log, stderr=log, timeout=10).returncode != 0
                # Reject truncated rows and missing origin metadata before optimization.
                for invalid in ['# origin_lla 0 0 10\n1 2 3\n',
                                '\n'.join(' '.join(row) for row in enu)]:
                    (directory / 'bad.txt').write_text(invalid)
                    run = subprocess.run(['rosrun', 'bievr_lio_ros', 'gnss_optimize', '--lio_trajectory',
                                                 str(directory / 'lio.txt'), '--gnss_enu',
                                                 str(directory / 'bad.txt'), '--skip-output-map'],
                                         stdout=log, stderr=log, timeout=10)
                    assert run.returncode != 0
                print('PASS: live recording, validity/covariance, precise stamps, overwrite protection, '
                      'PoseStamped LLA/raw/ENU/TUM/shared origin and pose-only recording, '
                      'scan map geometry/intensity, trajectory-only mode, truncated/missing archive rejection')
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()


if __name__ == '__main__':
    main()
