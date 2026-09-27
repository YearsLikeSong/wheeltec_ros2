#!/usr/bin/env python3
"""Hardware-free smoke test: synthetic laser/odometry -> SLAM -> Nav2 goal.

Run after sourcing install/local_setup.bash. A private ROS domain is mandatory.
No serial ports or real chassis processes are opened.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '173', 'Run with ROS_DOMAIN_ID=173'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Run with ROS_LOCALHOST_ONLY=1'

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TestRobot(Node):
    def __init__(self):
        super().__init__('v550_synthetic_robot')
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.w = 0.0
        self.command_time = time.monotonic()
        self.last_time = time.monotonic()
        self.max_vy = 0.0
        self.got_map = False
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom_combined', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_received, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        sensor = TransformStamped()
        sensor.header.frame_id = 'base_footprint'
        sensor.child_frame_id = 'laser'
        sensor.transform.rotation.w = 1.0
        self.static_tf.sendTransform(sensor)
        self.create_timer(0.05, self.tick)

    def command(self, msg):
        self.vx, self.vy, self.w = msg.linear.x, msg.linear.y, msg.angular.z
        self.max_vy = max(self.max_vy, abs(self.vy))
        self.command_time = time.monotonic()

    def map_received(self, msg):
        self.got_map = bool(msg.data)

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now
        if now - self.command_time > 0.5:
            self.vx = self.vy = self.w = 0.0
        self.x += (math.cos(self.yaw) * self.vx - math.sin(self.yaw) * self.vy) * dt
        self.y += (math.sin(self.yaw) * self.vx + math.cos(self.yaw) * self.vy) * dt
        self.yaw += self.w * dt
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom_combined'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.vx, self.vy
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max = 0.1, 8.0
        scan.scan_time = 0.05
        # Rays intersect a 6 x 6 metre room; every scan matches the integrated pose.
        for i in range(360):
            angle = self.yaw + scan.angle_min + i * scan.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            tx = ((3.0 if dx > 0 else -3.0) - self.x) / dx if abs(dx) > 1e-9 else 100.0
            ty = ((3.0 if dy > 0 else -3.0) - self.y) / dy if abs(dy) > 1e-9 else 100.0
            scan.ranges.append(min(tx, ty))
        self.scan_pub.publish(scan)


def main():
    root = Path(__file__).resolve().parents[1]
    log_path = root / 'log/v550_navigation_smoke.log'
    goal_log_path = root / 'log/v550_goal_smoke.log'
    rclpy.init()
    robot = TestRobot()
    process = goal = None
    try:
        with log_path.open('w') as log, goal_log_path.open('w') as goal_log:
            process = subprocess.Popen([
                'ros2', 'launch', 'v550_navigation', 'unknown_navigation.launch.py',
                'bringup_hardware:=false', 'use_rviz:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            start = time.monotonic()
            while time.monotonic() - start < 90:
                assert process.poll() is None, 'Navigation launch exited; read the smoke log'
                rclpy.spin_once(robot, timeout_sec=0.05)
                if robot.got_map and time.monotonic() - start > 10 and goal is None:
                    goal = subprocess.Popen([
                        'ros2', 'run', 'v550_navigation', 'send_goal',
                        '0.7', '0.5', '--timeout', '55'],
                        stdout=goal_log, stderr=subprocess.STDOUT, start_new_session=True)
                if goal is not None and goal.poll() is not None:
                    assert goal.returncode == 0, f'Goal failed; see {goal_log_path}'
                    error = math.hypot(robot.x - 0.7, robot.y - 0.5)
                    assert error < 0.25, f'Synthetic robot missed the goal by {error:.3f} m'
                    assert robot.max_vy > 0.01, 'No lateral command was produced'
                    # Check cancellation through the public algorithm example.
                    cancel_test = subprocess.Popen([
                        'ros2', 'run', 'v550_navigation', 'send_goal',
                        '1.5', '0.5', '--timeout', '0.1'],
                        stdout=goal_log, stderr=subprocess.STDOUT, start_new_session=True)
                    cancel_deadline = time.monotonic() + 15
                    while cancel_test.poll() is None and time.monotonic() < cancel_deadline:
                        rclpy.spin_once(robot, timeout_sec=0.05)
                    if cancel_test.poll() is None:
                        cancel_test.kill()
                        cancel_test.wait()
                        raise AssertionError('Cancellation example timed out')
                    assert cancel_test.returncode == 1, 'Timed-out goal should return failure'
                    # Save the running SLAM map without writing into the user map directory.
                    with tempfile.TemporaryDirectory(prefix='v550_map_test_') as folder:
                        saver = subprocess.Popen([
                            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                            '-f', folder + '/test_map', '--ros-args',
                            '-p', 'save_map_timeout:=10.0'],
                            stdout=goal_log, stderr=subprocess.STDOUT)
                        save_deadline = time.monotonic() + 20
                        while saver.poll() is None and time.monotonic() < save_deadline:
                            rclpy.spin_once(robot, timeout_sec=0.05)
                        if saver.poll() is None:
                            saver.kill()
                            saver.wait()
                            raise AssertionError('Map save timed out')
                        assert saver.returncode == 0
                        assert Path(folder, 'test_map.yaml').is_file()
                        assert Path(folder, 'test_map.pgm').is_file()
                    assert 'Cancellation not acknowledged' not in goal_log_path.read_text()
                    print('PASS: goal timeout/cancellation and map saving')
                    print(f'PASS: live SLAM map, accepted Nav2 goal, reached goal (error={error:.3f} m), lateral speed={robot.max_vy:.3f} m/s')
                    return
            raise AssertionError('Timed out waiting for map, Nav2 or goal completion')
    finally:
        # The bundled Nav2 must clean up plugin instances before process exit.
        if process is not None and process.poll() is None:
            manager = robot.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            if manager.wait_for_service(timeout_sec=2.0):
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.SHUTDOWN
                shutdown = manager.call_async(request)
                rclpy.spin_until_future_complete(robot, shutdown, timeout_sec=8.0)
                if shutdown.done() and shutdown.exception() is None and shutdown.result().success:
                    print('PASS: Nav2 lifecycle shutdown')
            robot.destroy_client(manager)
        for child in (goal, process):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
