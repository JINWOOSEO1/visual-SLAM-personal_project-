#!/usr/bin/env python3
"""
encoder_node.py
GPIO 인터럽트로 좌/우 엔코더 펄스를 카운트하여 퍼블리시

Published topics:
  /encoder/left_ticks  (std_msgs/Int32) : 왼쪽 누적 펄스 수
  /encoder/right_ticks (std_msgs/Int32) : 오른쪽 누적 펄스 수

Parameters:
  left_pin  (int, default=17)
  right_pin (int, default=16)
  edge_mode (str, default='rising')  : 'rising' | 'both'
  publish_rate (float, default=50.0) : Hz
"""

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        # ─── 파라미터 ────────────────────────────────────────────
        self.declare_parameter('left_pin',    17)
        self.declare_parameter('right_pin',   16)
        self.declare_parameter('edge_mode',   'rising')   # 'rising' or 'both'
        self.declare_parameter('publish_rate', 50.0)

        self.left_pin    = self.get_parameter('left_pin').value
        self.right_pin   = self.get_parameter('right_pin').value
        edge_mode        = self.get_parameter('edge_mode').value
        publish_rate     = self.get_parameter('publish_rate').value

        # ─── 카운터 ──────────────────────────────────────────────
        self._left_ticks  = 0
        self._right_ticks = 0

        # ─── GPIO 설정 ───────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        edge = GPIO.BOTH if edge_mode == 'both' else GPIO.RISING

        GPIO.setup(self.left_pin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.left_pin,  edge, callback=self._cb_left,  bouncetime=1)
        GPIO.add_event_detect(self.right_pin, edge, callback=self._cb_right, bouncetime=1)

        self.get_logger().info(
            f'EncoderNode 시작: left=GPIO{self.left_pin}, '
            f'right=GPIO{self.right_pin}, edge={edge_mode}'
        )

        # ─── 퍼블리셔 ────────────────────────────────────────────
        self._pub_left  = self.create_publisher(Int32, '/encoder/left_ticks',  10)
        self._pub_right = self.create_publisher(Int32, '/encoder/right_ticks', 10)

        # ─── 타이머 ──────────────────────────────────────────────
        period = 1.0 / publish_rate
        self.create_timer(period, self._publish)

    # ─── 인터럽트 콜백 ───────────────────────────────────────────
    def _cb_left(self, channel):
        self._left_ticks += 1

    def _cb_right(self, channel):
        self._right_ticks += 1

    # ─── 퍼블리시 ─────────────────────────────────────────────────
    def _publish(self):
        msg_l = Int32()
        msg_r = Int32()
        msg_l.data = self._left_ticks
        msg_r.data = self._right_ticks
        self._pub_left.publish(msg_l)
        self._pub_right.publish(msg_r)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
