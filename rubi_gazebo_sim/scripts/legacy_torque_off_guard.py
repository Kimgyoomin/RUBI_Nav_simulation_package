#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--service',
        default='/rubi_gazebo_legacy/legacy_policy_controller/torque_off',
    )
    parser.add_argument('--timeout', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node('legacy_torque_off_guard')
    client = node.create_client(Trigger, args.service)
    elapsed = 0.0
    while rclpy.ok() and elapsed < args.timeout:
        if client.wait_for_service(timeout_sec=0.25):
            break
        elapsed += 0.25

    if not client.service_is_ready():
        node.get_logger().error(
            f'SAFE_START_GUARD=FAIL service_timeout service={args.service}')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None or not response.success:
        message = 'no response' if response is None else response.message
        node.get_logger().error(
            f'SAFE_START_GUARD=FAIL torque_off_rejected message={message}')
        node.destroy_node()
        rclpy.shutdown()
        return 3

    node.get_logger().info(
        f'SAFE_START_GUARD=PASS torque_off_queued service={args.service}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
