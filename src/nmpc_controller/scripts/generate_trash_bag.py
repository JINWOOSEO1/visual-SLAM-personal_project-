#!/usr/bin/env python3
"""
generate_trash_bag.py — Create a rosbag2 of dummy /odometry/filtered messages.

The bag has the exact same message structure as the real /odometry/filtered
topic (nav_msgs/msg/Odometry, including 6x6 covariances) but is filled with
arbitrary numbers. The payload size is therefore realistic so that the bag
can be used to measure Pi -> host network transport latency.

Note:
    `ros2 bag play` does NOT rewrite header.stamp on playback. If you want
    the latency = (now - header.stamp) to mean "transport delay only", use
    latency_replayer.py instead of `ros2 bag play`. The replayer restamps
    header.stamp with the current time at publish time.

Prerequisites:
    source /opt/ros/humble/setup.bash

Usage:
    python3 generate_trash_bag.py \\
        --output /tmp/trash_odom_bag \\
        --duration 30.0 \\
        --rate 50.0
"""

import argparse
import os
import shutil

import numpy as np
import rclpy.serialization
import rosbag2_py
from nav_msgs.msg import Odometry


def make_trash_message(seq: int, t_ns: int) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec     = t_ns // 1_000_000_000
    msg.header.stamp.nanosec = t_ns % 1_000_000_000
    msg.header.frame_id      = 'odom'
    msg.child_frame_id       = 'base_link'

    # Trash pose / twist — deterministic so the bag content is reproducible
    rng = np.random.default_rng(seed=seq)
    msg.pose.pose.position.x = float(rng.uniform(-5.0, 5.0))
    msg.pose.pose.position.y = float(rng.uniform(-5.0, 5.0))
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = float(rng.uniform(-1.0, 1.0))
    msg.pose.pose.orientation.w = float(rng.uniform(-1.0, 1.0))
    msg.pose.covariance = [float(v) for v in rng.uniform(0.0, 0.1, size=36)]

    msg.twist.twist.linear.x  = float(rng.uniform(-0.3, 0.3))
    msg.twist.twist.linear.y  = 0.0
    msg.twist.twist.linear.z  = 0.0
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = float(rng.uniform(-1.5, 1.5))
    msg.twist.covariance = [float(v) for v in rng.uniform(0.0, 0.1, size=36)]

    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/trash_odom_bag',
                        help='Output bag directory (will be overwritten)')
    parser.add_argument('--topic', default='/odometry/filtered',
                        help='Topic name to write')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Total duration of fake data in seconds')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='Message publish rate in Hz')
    args = parser.parse_args()

    if os.path.exists(args.output):
        shutil.rmtree(args.output)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.topic,
        type='nav_msgs/msg/Odometry',
        serialization_format='cdr',
    ))

    n_msgs   = int(args.duration * args.rate)
    period_ns = int(1e9 / args.rate)
    base_t_ns = 0  # bag-time starts at 0; absolute stamp does not matter here

    for i in range(n_msgs):
        t_ns = base_t_ns + i * period_ns
        msg = make_trash_message(i, t_ns)
        writer.write(args.topic, rclpy.serialization.serialize_message(msg), t_ns)

    del writer
    print(f'Wrote {n_msgs} messages to {args.output} '
          f'({args.duration:.1f}s @ {args.rate:.1f} Hz)')


if __name__ == '__main__':
    main()
