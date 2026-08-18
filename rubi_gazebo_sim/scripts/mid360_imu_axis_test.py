#!/usr/bin/env python3

import math
import sys
import time

from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import ApplyLinkWrench
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty


class AxisTest(Node):

    def __init__(self):
        super().__init__('mid360_imu_axis_test_runner')
        self.samples = []
        self.create_subscription(
            Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data)
        self.reset_client = self.create_client(Empty, '/reset_world')
        self.wrench_client = self.create_client(
            ApplyLinkWrench, '/apply_link_wrench')

    def on_imu(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.samples.append((
            stamp,
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
        ))

    def spin_until(self, predicate, timeout=10.0):
        start = time.monotonic()
        while rclpy.ok() and not predicate():
            if time.monotonic() - start > timeout:
                raise RuntimeError('timeout while waiting for simulation data')
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        self.spin_until(future.done)
        response = future.result()
        if response is None:
            raise RuntimeError(f'service failed: {client.srv_name}')
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(
                f'service rejected: {client.srv_name} '
                f'{response.status_message}')

    def settle_after_reset(self):
        self.call(self.reset_client, Empty.Request())
        start_count = len(self.samples)
        self.spin_until(lambda: len(self.samples) >= start_count + 40)

    def excite(self, axis, rotational):
        self.settle_after_reset()
        request = ApplyLinkWrench.Request()
        request.link_name = 'mid360_imu_axis_test::BODY'
        request.reference_frame = 'world'
        request.duration = Duration(sec=0, nanosec=100_000_000)
        start_time = self.samples[-1][0] + 0.05
        request.start_time.sec = int(math.floor(start_time))
        request.start_time.nanosec = int(
            round((start_time - request.start_time.sec) * 1e9))

        # The adapter maps raw FRD data back into BODY-aligned FLU axes.
        world_sign = 1.0
        if rotational:
            values = [0.0, 0.0, 0.0]
            values[axis] = world_sign * 0.08
            request.wrench.torque.x = values[0]
            request.wrench.torque.y = values[1]
            request.wrench.torque.z = values[2]
        else:
            values = [0.0, 0.0, 0.0]
            values[axis] = world_sign * 2.0
            request.wrench.force.x = values[0]
            request.wrench.force.y = values[1]
            request.wrench.force.z = values[2]

        start_count = len(self.samples)
        start_stamp = self.samples[-1][0]
        self.call(self.wrench_client, request)
        self.spin_until(
            lambda: self.samples and self.samples[-1][0] >= start_stamp + 0.25)
        response = self.samples[start_count:]
        field = 1 if rotational else 2
        component = [sample[field][axis] for sample in response]
        peak = max(component)
        cross_peak = max(
            abs(sample[field][other])
            for sample in response
            for other in range(3) if other != axis
        )
        return peak, cross_peak


def main():
    rclpy.init()
    node = AxisTest()
    passed = True
    labels = ['x', 'y', 'z']
    try:
        node.spin_until(lambda: len(node.samples) >= 40)
        for rotational in (True, False):
            kind = 'gyro' if rotational else 'accel'
            threshold = 0.05 if rotational else 0.5
            for axis, label in enumerate(labels):
                peak, cross_peak = node.excite(axis, rotational)
                axis_pass = math.isfinite(peak) and peak > threshold
                passed = passed and axis_pass
                print(
                    f'{kind}_{label}_positive_peak={peak:.9f} '
                    f'cross_axis_peak={cross_peak:.9f} '
                    f'result={"PASS" if axis_pass else "FAIL"}')
    except RuntimeError as error:
        print(f'AXIS_TEST=FAIL reason={error}')
        passed = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'AXIS_TEST={"PASS" if passed else "FAIL"}')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
