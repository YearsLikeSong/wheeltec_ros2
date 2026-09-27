#!/usr/bin/env python3
"""Hardware-free smoke test: synthetic laser/odometry -> SLAM -> Nav2 goal.

Run after sourcing install/local_setup.bash. A private ROS domain is mandatory.
No serial ports or real chassis processes are opened.
"""
import argparse
import json
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
    def __init__(self, near_wall=False, deadband=0.0):
        super().__init__('v550_synthetic_robot')
        self.wall_x = 1.25 if near_wall else 3.0
        self.deadband = deadband
        self.commands = []
        self.min_wall_clearance = float("inf")
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
        self.commands.append((time.monotonic(), self.vx, self.vy, self.w))
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
        vx, vy = self.vx, self.vy
        if math.hypot(vx, vy) < self.deadband:
            vx = vy = 0.0
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * dt
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * dt
        self.yaw += self.w * dt
        extent = abs(math.cos(self.yaw)) * 0.17 + abs(math.sin(self.yaw)) * 0.15
        self.min_wall_clearance = min(self.min_wall_clearance, self.wall_x-self.x-extent)
        assert self.min_wall_clearance > 0, 'Synthetic footprint collided with the wall'
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
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
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
        # Rays intersect the configured room walls and match the integrated pose.
        for i in range(360):
            angle = self.yaw + scan.angle_min + i * scan.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            tx = ((self.wall_x if dx > 0 else -3.0) - self.x) / dx if abs(dx) > 1e-9 else 100.0
            ty = ((3.0 if dy > 0 else -3.0) - self.y) / dy if abs(dy) > 1e-9 else 100.0
            scan.ranges.append(min(tx, ty))
        self.scan_pub.publish(scan)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--near-wall', action='store_true', help='Goal near a wall, with footprint clearance checked')
    parser.add_argument('--deadband', type=float, default=0.0, help='Synthetic planar drive deadband in m/s; not a measured chassis value')
    parser.add_argument('--params-file')
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--label', default='smoke')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    goal_xy = (0.9, 0.0) if args.near_wall else (0.7, 0.5)
    log_path = root / f'log/v550_navigation_{args.label}.log'
    goal_log_path = root / f'log/v550_goal_{args.label}.log'
    rclpy.init()
    robot = TestRobot(args.near_wall, args.deadband)
    process = goal = None
    try:
        with log_path.open('w') as log, goal_log_path.open('w') as goal_log:
            process = subprocess.Popen([
                'ros2', 'launch', 'v550_navigation', 'unknown_navigation.launch.py',
                'bringup_hardware:=false', 'use_rviz:=false'] +
                ([f'params_file:={Path(args.params_file).resolve()}'] if args.params_file else []),
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            start = time.monotonic()
            while time.monotonic() - start < 90:
                assert process.poll() is None, 'Navigation launch exited; read the smoke log'
                rclpy.spin_once(robot, timeout_sec=0.05)
                if robot.got_map and time.monotonic() - start > 10 and goal is None:
                    goal = subprocess.Popen([
                        'ros2', 'run', 'v550_navigation', 'send_goal',
                        str(goal_xy[0]), str(goal_xy[1]), '--yaw', str(args.goal_yaw), '--timeout', '55'],
                        stdout=goal_log, stderr=subprocess.STDOUT, start_new_session=True)
                if goal is not None and goal.poll() is not None:
                    assert goal.returncode == 0, f'Goal failed; see {goal_log_path}'
                    error = math.hypot(robot.x - goal_xy[0], robot.y - goal_xy[1])
                    assert error < 0.25, f'Synthetic robot missed the goal by {error:.3f} m'
                    yaw_error = abs(math.atan2(math.sin(robot.yaw-args.goal_yaw), math.cos(robot.yaw-args.goal_yaw)))
                    assert yaw_error < 0.35, f'Goal yaw error {yaw_error:.3f} rad'
                    if not args.near_wall:
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
        (root / f'log/v550_navigation_{args.label}.json').write_text(json.dumps({
            'near_wall': args.near_wall, 'synthetic_deadband': args.deadband,
            'goal': goal_xy, 'goal_yaw': args.goal_yaw, 'position': [robot.x, robot.y, robot.yaw],
            'position_error': math.hypot(robot.x-goal_xy[0], robot.y-goal_xy[1]),
            'min_wall_clearance': robot.min_wall_clearance,
            'commands': robot.commands,
        }, indent=2))
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
