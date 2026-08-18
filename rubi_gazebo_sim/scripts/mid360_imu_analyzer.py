#!/usr/bin/env python3

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def format_triplet(values):
    return ','.join(f'{value:.9f}' for value in values)


class ImuCollector(Node):

    def __init__(self, topic):
        super().__init__('mid360_imu_analyzer')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4000,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.latest_clock_ns = None
        self.samples = []
        self.create_subscription(Clock, '/clock', self.on_clock, qos)
        self.create_subscription(Imu, topic, self.on_imu, qos)

    def on_clock(self, message):
        self.latest_clock_ns = (
            message.clock.sec * 1_000_000_000 + message.clock.nanosec)

    def on_imu(self, message):
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000 +
            message.header.stamp.nanosec)
        angular = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        linear = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        orientation = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        self.samples.append((
            stamp_ns,
            time.monotonic_ns(),
            self.latest_clock_ns,
            message.header.frame_id,
            angular,
            linear,
            orientation,
        ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/livox/imu')
    parser.add_argument('--count', type=int, default=2000)
    parser.add_argument('--discard', type=int, default=200)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--stationary-check', action='store_true')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ImuCollector(args.topic)
    start = time.monotonic()
    while rclpy.ok() and len(node.samples) < args.count:
        if time.monotonic() - start > args.timeout:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()
    if len(samples) < 2:
        print(f'ANALYSIS=FAIL reason=insufficient_samples count={len(samples)}')
        return 2

    stamps = [sample[0] for sample in samples]
    wall_receipts = [sample[1] for sample in samples]
    deltas = [
        (current - previous) / 1e9
        for previous, current in zip(stamps, stamps[1:])
    ]
    clock_diffs = [
        (sample[2] - sample[0]) / 1e9
        for sample in samples if sample[2] is not None
    ]
    finite = all(
        math.isfinite(value)
        for sample in samples
        for vector in (sample[4], sample[5], sample[6])
        for value in vector
    )
    frames = sorted({sample[3] for sample in samples})
    negative = sum(delta < 0 for delta in deltas)
    duplicate = sum(delta == 0 for delta in deltas)
    sim_duration = (stamps[-1] - stamps[0]) / 1e9
    wall_duration = (wall_receipts[-1] - wall_receipts[0]) / 1e9
    sim_rate = (len(samples) - 1) / sim_duration
    wall_rate = (len(samples) - 1) / wall_duration
    rtf = sim_duration / wall_duration

    steady = samples[min(args.discard, len(samples) - 1):]
    gyro_mean = tuple(
        statistics.fmean(sample[4][index] for sample in steady)
        for index in range(3)
    )
    accel_mean = tuple(
        statistics.fmean(sample[5][index] for sample in steady)
        for index in range(3)
    )
    gyro_norm_mean = statistics.fmean(
        vector_norm(sample[4]) for sample in steady)
    accel_norm_mean = statistics.fmean(
        vector_norm(sample[5]) for sample in steady)
    accel_variance = tuple(
        statistics.pvariance(sample[5][index] for sample in steady)
        for index in range(3)
    )

    print(f'message_count={len(samples)}')
    print(f'frame_ids={frames}')
    print(f'finite_fields={str(finite).lower()}')
    print(f'first_stamp_ns={stamps[0]}')
    print(f'last_stamp_ns={stamps[-1]}')
    print(f'negative_delta_count={negative}')
    print(f'duplicate_delta_count={duplicate}')
    print(f'delta_mean_sec={statistics.fmean(deltas):.9f}')
    print(f'delta_median_sec={statistics.median(deltas):.9f}')
    print(f'delta_min_sec={min(deltas):.9f}')
    print(f'delta_max_sec={max(deltas):.9f}')
    print(f'simulation_rate_hz={sim_rate:.6f}')
    print(f'wall_rate_hz={wall_rate:.6f}')
    print(f'real_time_factor={rtf:.6f}')
    if clock_diffs:
        print(f'clock_minus_stamp_min_sec={min(clock_diffs):.9f}')
        print(f'clock_minus_stamp_max_sec={max(clock_diffs):.9f}')
    print(f'gyro_mean_rad_s={format_triplet(gyro_mean)}')
    print(f'gyro_norm_mean_rad_s={gyro_norm_mean:.9f}')
    print(f'accel_mean_m_s2={format_triplet(accel_mean)}')
    print(f'accel_norm_mean_m_s2={accel_norm_mean:.9f}')
    print(f'accel_variance={format_triplet(accel_variance)}')

    passed = (
        len(samples) >= args.count and
        frames == ['livox_frame'] and
        finite and negative == 0 and duplicate == 0 and
        abs(statistics.fmean(deltas) - 0.005) <= 0.0001
    )
    if args.stationary_check:
        passed = passed and (
            gyro_norm_mean < 0.05 and
            abs(accel_norm_mean - 9.81) < 0.25 and
            abs(accel_mean[0]) < 0.25 and
            abs(accel_mean[1]) < 0.25 and
            abs(accel_mean[2] - 9.81) < 0.25
        )
    print(f'ANALYSIS={"PASS" if passed else "FAIL"}')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
