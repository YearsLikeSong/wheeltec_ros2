#!/usr/bin/env python3
"""Minimal executable example for connecting your own algorithm to Nav2.

Only an explicit x/y command sends a goal. Coordinates use the map frame.
"""
import argparse
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('--yaw', type=float, default=0.0, help='Heading in radians')
    parser.add_argument('--timeout', type=float, default=180.0, help='Goal timeout in seconds')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw, args.timeout)) or args.timeout <= 0:
        parser.error('Coordinates must be finite and timeout must be positive')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('v550_goal_sender')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    handle = None
    pending = None
    finished = False
    code = 1
    try:
        if not client.wait_for_server(timeout_sec=20.0):
            node.get_logger().error('Nav2 is unavailable; start navigation and check lifecycle/TF/scan.')
            return 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        pending = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, pending, timeout_sec=10.0)
        if not pending.done():
            node.get_logger().error('Goal acknowledgement timed out.')
            return 1
        handle = pending.result()
        if not handle.accepted:
            node.get_logger().error('Nav2 rejected the goal.')
            return 1
        node.get_logger().info(f'Navigating to map ({args.x}, {args.y}), yaw={args.yaw}')
        result = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if result.done():
            finished = True
            status = result.result().status
            code = 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
            node.get_logger().info(f'Navigation result status: {status} (4 means succeeded)')
        else:
            node.get_logger().error('Navigation timed out; cancelling this goal.')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; cancelling this goal.')
        code = 130
    finally:
        # Resolve a late acknowledgement before cancelling, including Ctrl-C during send.
        if handle is None and pending is not None:
            rclpy.spin_until_future_complete(node, pending, timeout_sec=2.0)
            if pending.done() and pending.exception() is None:
                handle = pending.result()
        if handle is not None and handle.accepted and not finished:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
            if not cancel.done() or cancel.exception() or not cancel.result().goals_canceling:
                node.get_logger().error('Cancellation not acknowledged; check Nav2 or use the chassis stop control.')
        elif pending is not None and not pending.done():
            node.get_logger().error('No goal acknowledgement; verify robot state in RViz.')
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
