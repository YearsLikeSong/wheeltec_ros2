#!/usr/bin/env python3
"""Read-only ROS sampling of scan, IMU, odometry and TF; sends no commands."""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', default='log/v550_tf_diagnostic.json')
    parser.add_argument('--record', help='Save complete odometry, TF and scans to a .json.gz file')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('v550_readonly_diagnostics')
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
    series = defaultdict(list)
    sources = defaultdict(set)
    scans = []

    def measurement(msg, topic):
        q = msg.orientation if isinstance(msg, Imu) else msg.pose.pose.orientation
        omega = msg.angular_velocity if isinstance(msg, Imu) else msg.twist.twist.angular
        item = {'t': time.monotonic(), 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame': msg.header.frame_id, 'yaw': yaw(q), 'wz': omega.z,
                'qnorm': math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)}
        if isinstance(msg, Odometry):
            item.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                        vx=msg.twist.twist.linear.x, vy=msg.twist.twist.linear.y,
                        yaw_cov=msg.pose.covariance[35], wz_cov=msg.twist.covariance[35])
        else:
            item.update(yaw_cov=msg.orientation_covariance[8], wz_cov=msg.angular_velocity_covariance[8])
        series[topic].append(item)

    def transforms(msg):
        gid = 'unavailable_in_this_rclpy_version'
        for tf in msg.transforms:
            key = f'{tf.header.frame_id} -> {tf.child_frame_id}'
            sources[tf.child_frame_id].add((tf.header.frame_id, gid))
            series[key].append({'t': time.monotonic(), 'yaw': yaw(tf.transform.rotation),
                                'stamp': tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9,
                                'x': tf.transform.translation.x, 'y': tf.transform.translation.y})

    def scan(msg):
        item = {'t': time.monotonic(), 'frame': msg.header.frame_id,
                'size': len(msg.ranges), 'increment': msg.angle_increment,
                'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
                'time_increment': msg.time_increment, 'scan_time': msg.scan_time,
                'age': node.get_clock().now().nanoseconds * 1e-9 - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9}
        item['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if args.record:
            item.update(ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],
                        range_min=msg.range_min, range_max=msg.range_max)
        raw = np.asarray(msg.ranges)
        raw = np.clip(np.where(np.isfinite(raw), raw, 10.0), 0.0, 10.0)
        angles = msg.angle_min + np.arange(len(raw))*msg.angle_increment
        normalized = np.interp(np.arange(720)*2*math.pi/720, angles, raw, period=2*math.pi)
        if scans:
            correlation = np.fft.irfft(np.fft.rfft(scans[1])*np.conj(np.fft.rfft(normalized)), n=720)
            offsets = np.arange(-180,181)
            item['scan_alignment_deg'] = float(offsets[np.argmax(correlation[offsets % 720])])*0.5
            scans[1] = normalized
        else:
            item['scan_alignment_deg'] = 0.0
            scans.extend([normalized, normalized])
        series['/scan'].append(item)

    for topic in ('/odom', '/odom_combined'):
        node.create_subscription(Odometry, topic, lambda m, t=topic: measurement(m, t), qos)
    for topic in ('/imu/data_raw', '/imu/data'):
        node.create_subscription(Imu, topic, lambda m, t=topic: measurement(m, t), qos)
    node.create_subscription(TFMessage, '/tf', transforms, qos)
    node.create_subscription(TFMessage, '/tf_static', transforms, static_qos)
    node.create_subscription(LaserScan, '/scan', scan, qos)
    node.create_subscription(LaserScan, '/scan_slam', lambda m: series['/scan_slam'].append({
        't': time.monotonic(), 'frame': m.header.frame_id, 'size': len(m.ranges),
        'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        'increment': m.angle_increment, 'angle_min': m.angle_min, 'angle_max': m.angle_max}), qos)
    node.create_subscription(PoseWithCovarianceStamped, '/pose', lambda m: series['/pose'].append({
        't': time.monotonic(), 'frame': m.header.frame_id,
        'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y,
        'yaw': yaw(m.pose.pose.orientation)}), qos)
    node.create_subscription(OccupancyGrid, '/map', lambda m: series['/map'].append({
        't': time.monotonic(), 'frame': m.header.frame_id, 'width': m.info.width,
        'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        'height': m.info.height, 'resolution': m.info.resolution,
        'origin_x': m.info.origin.position.x, 'origin_y': m.info.origin.position.y,
        'content_hash': hashlib.sha256(m.data.tobytes()).hexdigest()}), qos)
    node.create_subscription(Twist, '/cmd_vel', lambda m: series['/cmd_vel'].append({
        't': time.monotonic(), 'vx': m.linear.x, 'vy': m.linear.y, 'wz': m.angular.z}), qos)
    start = time.monotonic()
    progress_at = start
    print('Read-only capture started; Ctrl-C saves the captured data.', flush=True)
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - progress_at >= 5:
                progress_at = time.monotonic()
                status = {}
                for topic in ('/odom', '/odom_combined'):
                    if series[topic]:
                        last = series[topic][-1]
                        status[topic] = {key: round(last[key], 4) for key in ('x', 'y', 'yaw', 'vx', 'vy', 'wz')}
                print(f'{progress_at-start:.0f}s: {status}', flush=True)
    except KeyboardInterrupt:
        pass
    report = {'nodes': node.get_node_names_and_namespaces(), 'topics': {}, 'tf_sources': {}, 'publishers': {}}
    for key, values in series.items():
        if not values:
            continue
        elapsed = values[-1]['t'] - values[0]['t']
        stats = {'count': len(values), 'hz': round((len(values)-1)/elapsed, 2) if elapsed else 0,
                 'first': {k: v for k, v in values[0].items() if k != 'ranges'},
                 'last': {k: v for k, v in values[-1].items() if k != 'ranges'}}
        if 'x' in values[0]:
            xy = np.array([[v['x'], v['y']] for v in values])
            stats.update(x_span_m=float(np.ptp(xy[:, 0])), y_span_m=float(np.ptp(xy[:, 1])),
                         displacement_m=float(np.linalg.norm(xy[-1]-xy[0])))
        if 'content_hash' in values[0]:
            stats['content_versions'] = len({(v['width'], v['height'], v['origin_x'], v['origin_y'], v['content_hash']) for v in values})
        if 'size' in values[0]:
            stats['ray_counts'] = sorted({v['size'] for v in values})
        if 'yaw' in values[0]:
            angles = np.unwrap([v['yaw'] for v in values])
            stats.update(yaw_change_deg=float(np.rad2deg(angles[-1]-angles[0])),
                         yaw_span_deg=float(np.rad2deg(np.ptp(angles))))
        for field in ('wz', 'vx', 'vy', 'qnorm', 'age', 'scan_alignment_deg'):
            if field in values[0]:
                a = np.array([v[field] for v in values])
                stats[field] = {'min': float(a.min()), 'max': float(a.max()), 'mean': float(a.mean())}
        report['topics'][key] = stats
    for child, authorities in sources.items():
        report['tf_sources'][child] = sorted(authorities)
    for topic in ('/tf', '/tf_static', '/scan', '/scan_slam', '/pose', '/odom', '/odom_combined', '/imu/data_raw', '/cmd_vel', '/robot_description', '/map'):
        report['publishers'][topic] = [{'name': x.node_name, 'namespace': x.node_namespace,
                                       'gid': bytes(x.endpoint_gid).hex()}
                                      for x in node.get_publishers_info_by_topic(topic)]
    report['ros_environment'] = {key: os.environ.get(key, '(unset)')
                                 for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')}
    report['multiple_publishers'] = {
        topic: report['publishers'][topic]
        for topic in ('/scan', '/odom', '/odom_combined', '/robot_description', '/map')
        if len(report['publishers'].get(topic, [])) > 1}
    if report['multiple_publishers']:
        print('WARNING: multiple publishers on single-robot topics:',
              ', '.join(report['multiple_publishers']), flush=True)
    if len(scans) == 2 and len(scans[0]) == len(scans[1]):
        a, b = [np.where(np.isfinite(s), s, np.nan) for s in scans]
        limit = min(len(a)//4, 180)
        errors = [(float(np.nanmedian(abs(a - np.roll(b, k)))), k) for k in range(-limit, limit+1)]
        error, shift = min(errors)
        report['scan_first_last'] = {'best_shift_bins': shift, 'median_range_error_m': error,
                                    'zero_shift_median_error_m': float(np.nanmedian(abs(a-b)))}
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(report, indent=2))
    if args.record:
        Path(args.record).parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(args.record, 'wt') as stream:
            json.dump({'report': report, 'series': series}, stream)
        print('Full recording:', args.record)
    for key in ('/scan', '/scan_slam', '/pose', '/odom', '/imu/data_raw', '/odom_combined', 'odom_combined -> base_footprint', 'map -> odom_combined', '/map'):
        stats = report['topics'].get(key, {})
        print(key, {name: stats[name] for name in ('count', 'hz', 'ray_counts', 'content_versions', 'x_span_m', 'y_span_m', 'displacement_m', 'yaw_change_deg', 'yaw_span_deg', 'scan_alignment_deg') if name in stats})
    print('Full report:', args.output)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
