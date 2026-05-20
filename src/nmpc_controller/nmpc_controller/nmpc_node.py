#!/usr/bin/env python3
"""
nmpc_node.py

Subscribes to /odometry/filtered, runs NMPC at 10 Hz, publishes /cmd_vel.

Reference index tracking (monotonic):
  - First call: a = 0
  - Each step: compare dist(current, ref[a]) vs dist(current, ref[a+1])
               advance a if ref[a+1] is closer (strictly monotonic, no back-snap)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from nmpc_controller.nmpc_solver import NMPCSolver


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        # Parameters
        self.declare_parameter('traj_file',    '')
        self.declare_parameter('N',            20)
        self.declare_parameter('dt',           0.1)
        self.declare_parameter('v_max',        0.3)
        self.declare_parameter('w_max',        1.5)
        self.declare_parameter('Q_x',          2.0)
        self.declare_parameter('Q_y',          2.0)
        self.declare_parameter('Q_theta',      0.5)
        self.declare_parameter('R_v',          0.1)
        self.declare_parameter('R_w',          0.05)
        self.declare_parameter('QN_x',         4.0)
        self.declare_parameter('QN_y',         4.0)
        self.declare_parameter('QN_theta',     1.0)
        self.declare_parameter('odom_timeout', 1.0)

        traj_file        = self.get_parameter('traj_file').value
        N                = self.get_parameter('N').value
        dt               = self.get_parameter('dt').value
        v_max            = self.get_parameter('v_max').value
        w_max            = self.get_parameter('w_max').value
        self._odom_timeout = self.get_parameter('odom_timeout').value

        Q   = [self.get_parameter('Q_x').value,
               self.get_parameter('Q_y').value,
               self.get_parameter('Q_theta').value]
        R   = [self.get_parameter('R_v').value,
               self.get_parameter('R_w').value]
        Q_N = [self.get_parameter('QN_x').value,
               self.get_parameter('QN_y').value,
               self.get_parameter('QN_theta').value]

        # Load reference trajectory
        if not traj_file:
            self.get_logger().error('traj_file parameter is empty — cannot start')
            raise RuntimeError('traj_file is empty')

        try:
            self._ref_traj = np.load(traj_file)
            if self._ref_traj.ndim != 2 or self._ref_traj.shape[1] != 3:
                raise ValueError(f'Expected shape (T, 3), got {self._ref_traj.shape}')
            self.get_logger().info(
                f'Loaded trajectory: {traj_file}  shape={self._ref_traj.shape}')
        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory: {e}')
            raise

        self._T = len(self._ref_traj)
        self._N = N

        # Build NMPC solver (CasADi NLP compiled once here)
        self.get_logger().info('Building NMPC solver (CasADi NLP) ...')
        self._solver = NMPCSolver(
            N=N, dt=dt, Q=Q, R=R, Q_N=Q_N,
            v_max=v_max, w_max=w_max,
        )
        self.get_logger().info('NMPC solver ready')

        # State
        self._current_state  = None   # np.ndarray (3,) or None
        self._last_odom_time = None   # rclpy.time.Time
        self._ref_idx        = 0      # monotonic reference index a
        self._finished       = False

        # ROS interfaces
        self.create_subscription(Odometry, '/odometry/filtered', self._cb_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'NMPCNode started: N={N} dt={dt}s v_max={v_max} w_max={w_max} '
            f'traj_len={self._T}')

    # ------------------------------------------------------------------
    def _cb_odom(self, msg: Odometry):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)

        self._current_state  = np.array([x, y, theta])
        self._last_odom_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def _advance_ref_idx(self, pos: np.ndarray):
        """
        Advance reference index a by comparing distance to ref[a] and ref[a+1].
        Strictly monotonic — never goes backward.
        """
        a = self._ref_idx
        if a >= self._T - 1:
            return

        d_current = float(np.linalg.norm(pos - self._ref_traj[a, :2]))
        d_next    = float(np.linalg.norm(pos - self._ref_traj[a + 1, :2]))

        if d_next < d_current:
            self._ref_idx = a + 1

    # ------------------------------------------------------------------
    def _get_ref_segment(self) -> np.ndarray:
        """
        Extract N+1 reference states starting at _ref_idx.
        Pad with the last trajectory point when the end is reached.
        Returns shape (N+1, 3).
        """
        a = self._ref_idx
        N = self._N
        indices = np.arange(a, a + N + 1)
        indices = np.clip(indices, 0, self._T - 1)
        return self._ref_traj[indices]

    # ------------------------------------------------------------------
    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self._finished:
            self._publish_zero()
            return

        if self._current_state is None:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return

        elapsed = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        if elapsed > self._odom_timeout:
            self.get_logger().warn(
                f'Odometry timeout ({elapsed:.2f}s) — stopping',
                throttle_duration_sec=1.0)
            self._publish_zero()
            return

        pos = self._current_state[:2]

        # Advance reference index
        self._advance_ref_idx(pos)

        # Check trajectory completion
        dist_to_end = float(np.linalg.norm(pos - self._ref_traj[-1, :2]))
        if self._ref_idx >= self._T - 1 and dist_to_end < 0.15:
            self.get_logger().info('Trajectory complete — stopping')
            self._finished = True
            self._publish_zero()
            return

        # Extract reference segment
        ref_seg = self._get_ref_segment()   # (N+1, 3)

        # Solve NMPC
        u_opt, s_opt, success = self._solver.solve(self._current_state, ref_seg)

        if not success:
            self.get_logger().warn(
                'NMPC solve failed — applying fallback input',
                throttle_duration_sec=1.0)

        # Publish first input (receding horizon)
        twist = Twist()
        twist.linear.x  = float(u_opt[0, 0])
        twist.angular.z = float(u_opt[0, 1])
        self._cmd_pub.publish(twist)

        self.get_logger().info(
            f'[NMPC] idx={self._ref_idx}/{self._T} '
            f'pos=[{pos[0]:.2f},{pos[1]:.2f}] '
            f'theta={math.degrees(self._current_state[2]):.1f}deg '
            f'cmd=[v={twist.linear.x:.3f} w={twist.angular.z:.3f}] '
            f'ok={success}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
