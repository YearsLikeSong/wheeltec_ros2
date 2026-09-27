#!/usr/bin/env python3
"""Regression: drifting IMU yaw must not rotate a stationary V550 estimate.

Also checks that genuine turn-rate and mecanum lateral-velocity inputs are kept.
Only synthetic sensor messages are published, on private domain 175.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

assert os.environ.get('ROS_DOMAIN_ID') == '175'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Sensors(Node):
    def __init__(self):
        super().__init__('v550_ekf_regression')
        self.start = time.monotonic()
        self.odom = self.create_publisher(Odometry, '/test/input/odom', 10)
        self.imu = self.create_publisher(Imu, '/test/input/imu', 10)
        self.samples = []
        self.create_subscription(Odometry, '/test/output/odom', self.receive, 100)
        self.static = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'gyro_link'
        transform.transform.rotation.w = 1.0
        self.static.sendTransform(transform)
        self.create_timer(0.05, self.publish)

    def receive(self, msg):
        self.samples.append((time.monotonic()-self.start, yaw(msg.pose.pose.orientation),
                             msg.twist.twist.angular.z, msg.twist.twist.linear.y))

    def publish(self):
        elapsed = time.monotonic()-self.start
        moving = elapsed >= 10.0
        rate = 0.3 if moving else 0.0
        angle = max(0.0, elapsed-10.0)*0.3
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.z = math.sin(angle/2)
        odom.pose.pose.orientation.w = math.cos(angle/2)
        odom.twist.twist.angular.z = rate
        odom.twist.twist.linear.y = 0.1 if moving else 0.0
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[7] = 0.001
        odom.twist.covariance[35] = 1000.0 if moving else 1e-9
        self.odom.publish(odom)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'gyro_link'
        # Drift in the integrated yaw, independent of the physically stationary wheels.
        imu.orientation.z = math.sin((angle+0.005*elapsed)/2)
        imu.orientation.w = math.cos((angle+0.005*elapsed)/2)
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = 1e6
        imu.orientation_covariance[8] = 1e-6
        imu.angular_velocity.z = rate + 0.001
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = 1e6
        imu.angular_velocity_covariance[8] = 1e-6
        imu.linear_acceleration.z = 9.81
        self.imu.publish(imu)


def main():
    root = Path(__file__).resolve().parents[1]
    rclpy.init()
    node = Sensors()
    with (root/'log/v550_ekf_regression.log').open('w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
            '-r', '__node:=ekf_filter_node', '--params-file',
            str(root/'install/turn_on_wheeltec_robot/share/turn_on_wheeltec_robot/config/ekf.yaml'),
            '-r', 'odom:=/test/input/odom', '-r', '/imu/data_raw:=/test/input/imu',
            '-r', '/odometry/filtered:=/test/output/odom'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic()-node.start < 14.1:
                assert process.poll() is None, 'EKF exited'
                rclpy.spin_once(node, timeout_sec=0.05)
            still = [x for x in node.samples if 2 < x[0] < 9.9]
            turning = [x for x in node.samples if x[0] > 13.5]
            assert still and turning, 'No EKF output'
            drift = abs(still[-1][1]-still[0][1])
            turn = turning[-1][1]-still[-1][1]
            expected = (turning[-1][0]-10.0)*0.3
            assert drift < 0.003, f'Stationary yaw drift {math.degrees(drift):.4f} deg'
            assert abs(turn-expected) < 0.1, (turn, expected)
            assert abs(turning[-1][2]-0.3) < 0.02, turning[-1]
            assert abs(turning[-1][3]-0.1) < 0.01, turning[-1]
            print(f'PASS: stationary yaw drift={math.degrees(drift):.5f} deg; '
                  f'turn={turn:.3f} rad, expected={expected:.3f}; lateral velocity preserved')
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
