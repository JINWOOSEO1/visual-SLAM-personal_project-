#!/usr/bin/env python3
"""
latency_subscriber.py — Measure Pi -> host transport latency of /odometry/filtered.

Subscribes to /odometry/filtered and computes (host_now - msg.header.stamp)
for every received message. Prints rolling avg / p50 / p95 / max periodically.

Pair this with latency_replayer.py on the publisher side. Clocks must be
NTP-synced; otherwise the absolute latency is offset by the clock skew.

Prerequisites:
    source /opt/ros/humble/setup.bash

Usage:
    python3 latency_subscriber.py
    python3 latency_subscriber.py --topic /odometry/filtered --window 500
"""

import argparse
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LatencySubscriber(Node):

    def __init__(self, topic: str, window: int, report_period: float):
        super().__init__('latency_subscriber')
        self._lat_ms = deque(maxlen=window)
        self._count_total = 0
        self.create_subscription(Odometry, topic, self._cb, 10)
        self.create_timer(report_period, self._report)
        self.get_logger().info(
            f'Listening on {topic}, window={window}, report every {report_period:.1f}s')

    def _cb(self, msg: Odometry):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            return
        self._lat_ms.append((now_ns - stamp_ns) / 1e6)
        self._count_total += 1

    def _report(self):
        if not self._lat_ms:
            self.get_logger().info('No messages received yet.')
            return
        a = np.asarray(self._lat_ms)
        self.get_logger().info(
            f'n={self._count_total} window={len(a)} | '
            f'avg={a.mean():6.2f}ms  p50={np.percentile(a, 50):6.2f}  '
            f'p95={np.percentile(a, 95):6.2f}  max={a.max():6.2f}  '
            f'min={a.min():6.2f}'
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/odometry/filtered')
    parser.add_argument('--window', type=int, default=500,
                        help='Rolling window size for stats')
    parser.add_argument('--report-period', type=float, default=1.0,
                        help='How often to print stats (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = LatencySubscriber(args.topic, args.window, args.report_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
