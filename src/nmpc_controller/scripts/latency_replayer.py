#!/usr/bin/env python3
"""
latency_replayer.py — Publish messages from a rosbag2 with header.stamp = now().

Why not `ros2 bag play`?
    `ros2 bag play` keeps the original `header.stamp` of each message. For
    transport-latency measurement we need the stamp to be the *publish time*
    on this host, so the subscriber can compute (now - stamp) = transport
    delay only.

Run this on the Pi side. On the host side, subscribe to the same topic and
measure (host_now - msg.header.stamp).

Prerequisites:
    source /opt/ros/humble/setup.bash
    # NTP/chrony must keep Pi and host clocks in sync.

Usage:
    python3 latency_replayer.py --bag /tmp/trash_odom_bag --rate 50.0
"""

import argparse

import rclpy
import rclpy.serialization
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.node import Node


def load_bag(bag_path: str, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    msgs = []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != topic:
            continue
        msgs.append(rclpy.serialization.deserialize_message(data, Odometry))
    if not msgs:
        raise RuntimeError(f'No messages on topic {topic} in bag {bag_path}')
    return msgs


class LatencyReplayer(Node):

    def __init__(self, msgs, topic: str, rate_hz: float):
        super().__init__('latency_replayer')
        self._msgs = msgs
        self._idx  = 0
        self._pub  = self.create_publisher(Odometry, topic, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Replaying {len(msgs)} cached messages on {topic} @ {rate_hz:.1f} Hz')

    def _tick(self):
        msg = self._msgs[self._idx]
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        self._pub.publish(msg)
        self._idx = (self._idx + 1) % len(self._msgs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', default='/tmp/trash_odom_bag',
                        help='Bag directory created by generate_trash_bag.py')
    parser.add_argument('--topic', default='/odometry/filtered',
                        help='Topic to publish on')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='Publish rate (Hz)')
    args = parser.parse_args()

    msgs = load_bag(args.bag, args.topic)

    rclpy.init()
    node = LatencyReplayer(msgs, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
