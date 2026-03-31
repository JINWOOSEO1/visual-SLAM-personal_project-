#!/usr/bin/env python3
"""
odometry_node.py
좌/우 엔코더 틱 차이로 차동구동 오도메트리 계산 후 퍼블리시

Subscribed topics:
  /encoder/left_ticks  (std_msgs/Int32)
  /encoder/right_ticks (std_msgs/Int32)

Published topics:
  /odom (nav_msgs/Odometry)

TF broadcast:
  odom → base_link

Parameters:
  pulses_per_rev (float, default=20.0) : 엔코더 해상도 (검증 후 수정)
  wheel_diameter (float, default=0.065): 바퀴 직경 [m]
  wheel_base     (float, default=0.150): 휠베이스 [m]
  odom_frame     (str,   default='odom')
  base_frame     (str,   default='base_link')
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ─── 파라미터 ────────────────────────────────────────────
        self.declare_parameter('pulses_per_rev', 20.0)   # ← 검증 후 수정
        self.declare_parameter('wheel_diameter', 0.065)  # m
        self.declare_parameter('wheel_base',     0.150)  # m
        self.declare_parameter('odom_frame',     'odom')
        self.declare_parameter('base_frame',     'base_link')

        ppr         = self.get_parameter('pulses_per_rev').value
        wheel_d     = self.get_parameter('wheel_diameter').value
        self.wb     = self.get_parameter('wheel_base').value
        self.odom_f = self.get_parameter('odom_frame').value
        self.base_f = self.get_parameter('base_frame').value

        # 1 펄스당 이동 거리 [m]
        self.dist_per_tick = (math.pi * wheel_d) / ppr

        self.get_logger().info(
            f'OdometryNode 시작: PPR={ppr}, '
            f'wheel_d={wheel_d*1000:.0f}mm, base={self.wb*1000:.0f}mm, '
            f'dist/tick={self.dist_per_tick*1000:.3f}mm'
        )

        # ─── 상태 변수 ───────────────────────────────────────────
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0

        self._prev_left  = None   # 이전 틱 (첫 수신 전 None)
        self._prev_right = None

        self._last_left  = 0
        self._last_right = 0

        # ─── 퍼블리셔 / TF ───────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ─── 서브스크라이버 ──────────────────────────────────────
        self.create_subscription(Int32, '/encoder/left_ticks',  self._cb_left,  10)
        self.create_subscription(Int32, '/encoder/right_ticks', self._cb_right, 10)

    # ─── 틱 수신 ─────────────────────────────────────────────────
    def _cb_left(self, msg: Int32):
        self._last_left = msg.data
        self._update()

    def _cb_right(self, msg: Int32):
        self._last_right = msg.data
        self._update()

    # ─── 오도메트리 계산 ──────────────────────────────────────────
    def _update(self):
        # 첫 수신 시 기준값 설정
        if self._prev_left is None:
            self._prev_left  = self._last_left
            self._prev_right = self._last_right
            return

        # 구간 틱 차이
        d_left  = (self._last_left  - self._prev_left)  * self.dist_per_tick
        d_right = (self._last_right - self._prev_right) * self.dist_per_tick

        self._prev_left  = self._last_left
        self._prev_right = self._last_right

        if d_left == 0.0 and d_right == 0.0:
            return

        # 차동구동 기구학
        d_center = (d_right + d_left) / 2.0
        d_yaw    = (d_right - d_left) / self.wb

        self._yaw += d_yaw
        self._x   += d_center * math.cos(self._yaw)
        self._y   += d_center * math.sin(self._yaw)

        now = self.get_clock().now().to_msg()

        # ─── /odom 퍼블리시 ──────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_f
        odom.child_frame_id  = self.base_f

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y

        # yaw → quaternion (z축 회전만)
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)

        self._odom_pub.publish(odom)

        # ─── TF 브로드캐스트 ─────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.odom_f
        tf.child_frame_id  = self.base_f
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z    = math.sin(self._yaw / 2.0)
        tf.transform.rotation.w    = math.cos(self._yaw / 2.0)
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
