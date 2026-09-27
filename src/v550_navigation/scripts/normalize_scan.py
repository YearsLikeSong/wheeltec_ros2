#!/usr/bin/env python3
"""Give SLAM a fixed angular grid without changing the original lidar topic."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_scan(scan, count=1440):
    """Bin a full revolution, keeping the closest valid return in each bin.

    The V550 M10_P reports a slightly different number of rays per revolution.
    Karto caches the first scan geometry and rejects other lengths. Preserve
    timestamps, frame and blind sectors; never interpolate across missing rays.
    """
    if count < 4:
        raise ValueError('scan_points must be at least 4')
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0 or not math.isfinite(scan.angle_min)):
        raise ValueError('Invalid scan angle metadata')
    coverage = len(scan.ranges) * scan.angle_increment
    if abs(coverage - 2 * math.pi) > max(0.02, 2 * scan.angle_increment):
        raise ValueError('This V550 adapter requires a full 360 degree scan')
    result = LaserScan()
    result.header = scan.header
    result.angle_min = 0.0
    result.angle_increment = 2 * math.pi / count
    result.angle_max = (count - 1) * result.angle_increment
    result.scan_time = scan.scan_time
    # Rebinning changes the ray order at the circular boundary. A single precise
    # acquisition time per output bin is unavailable; retain the frame timestamp.
    result.time_increment = 0.0
    result.range_min = scan.range_min
    result.range_max = scan.range_max
    ranges = [float('inf')] * count
    has_intensity = len(scan.intensities) == len(scan.ranges)
    intensities = [0.0] * count if has_intensity else []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        target = int(math.floor(angle / result.angle_increment + 0.5)) % count
        if distance < ranges[target]:
            ranges[target] = distance
            if has_intensity:
                intensities[target] = scan.intensities[index]
    result.ranges = ranges
    result.intensities = intensities
    return result


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__('v550_scan_normalizer')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')
        self.declare_parameter('scan_points', 1440)
        self.count = self.get_parameter('scan_points').value
        if self.count < 4:
            raise ValueError('scan_points must be at least 4')
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value,
                                 self.receive, qos_profile_sensor_data)

    def receive(self, scan):
        try:
            self.publisher.publish(normalize_scan(scan, self.count))
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
