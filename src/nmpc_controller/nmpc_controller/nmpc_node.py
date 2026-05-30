#!/usr/bin/env python3
"""
nmpc_node.py

Subscribes to /odometry/filtered, runs NMPC at 10 Hz, publishes /cmd_vel.

Reference index tracking (monotonic):
  - First call: a = 0
  - Each step: search the window [a, a + W - 1] and pick the waypoint that
               minimizes position distance + heading_weight * |heading error|
               (strictly monotonic, no back-snap)
"""

import math
import time
from collections import deque

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
        self.declare_parameter('v_min',       -0.3)   # set to 0.0 to forbid reverse
        self.declare_parameter('v_abs_min',    0.0)   # set >0.0 to constrain abs(v)
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
        self.declare_parameter('ref_search_window', 10)
        self.declare_parameter('ref_heading_weight', 0.5)

        traj_file        = self.get_parameter('traj_file').value
        N                = self.get_parameter('N').value
        dt               = self.get_parameter('dt').value
        v_max            = self.get_parameter('v_max').value
        v_min            = self.get_parameter('v_min').value
        v_abs_min        = self.get_parameter('v_abs_min').value
        w_max            = self.get_parameter('w_max').value
        self._odom_timeout = self.get_parameter('odom_timeout').value
        self._ref_search_window  = self.get_parameter('ref_search_window').value
        self._ref_heading_weight = self.get_parameter('ref_heading_weight').value

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
            v_max=v_max, v_min=v_min, v_abs_min=v_abs_min, w_max=w_max,
        )
        self.get_logger().info('NMPC solver ready')

        # State
        self._current_state  = None   # np.ndarray (3,) or None
        self._last_odom_time = None   # rclpy.time.Time
        self._ref_idx        = 0      # monotonic reference index a
        self._finished       = False

        # Diagnostics — rolling windows for NMPC solve time and odom latency
        self._solve_ms_window  = deque(maxlen=200)
        self._odom_lat_ms_window = deque(maxlen=200)

        # ROS interfaces
        self.create_subscription(Odometry, '/odometry/filtered', self._cb_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'NMPCNode started: N={N} dt={dt}s v_min={v_min} v_max={v_max} '
            f'v_abs_min={v_abs_min} w_max={w_max} '
            f'traj_len={self._T}')

    # ------------------------------------------------------------------
    def _cb_odom(self, msg: Odometry):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)

        now = self.get_clock().now()
        self._current_state  = np.array([x, y, theta])
        self._last_odom_time = now

        # Latency = (receive time on this host) - (header.stamp set by publisher)
        # Meaningful only if clocks of publisher and this host are NTP-synced.
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns > 0:
            lat_ms = (now.nanoseconds - stamp_ns) / 1e6
            self._odom_lat_ms_window.append(lat_ms)

    # ------------------------------------------------------------------
    def _advance_ref_idx(self, state: np.ndarray):
        """
        Select reference index a by searching the window [a, a + W - 1]
        (W = ref_search_window, clipped to the trajectory end) and picking the
        waypoint whose position AND heading best match the current state.
        Combined cost = position distance + heading_weight * |wrapped heading error|.
        Strictly monotonic — the window starts at the current index, so a never
        goes backward.
        """
        a = self._ref_idx
        if a >= self._T - 1:
            return

        pos   = state[:2]
        theta = state[2]

        hi = min(a + self._ref_search_window, self._T)
        ref = self._ref_traj[a:hi]

        d_pos = np.linalg.norm(ref[:, :2] - pos, axis=1)

        dtheta = ref[:, 2] - theta
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))   # wrap to [-pi, pi]
        d_head = np.abs(dtheta)

        cost = d_pos + self._ref_heading_weight * d_head
        self._ref_idx = a + int(np.argmin(cost))

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
        self._advance_ref_idx(self._current_state)

        # Check trajectory completion
        dist_to_end = float(np.linalg.norm(pos - self._ref_traj[-1, :2]))
        if self._ref_idx >= self._T - 1 and dist_to_end < 0.15:
            self.get_logger().info('Trajectory complete — stopping')
            self._finished = True
            self._publish_zero()
            return

        # Extract reference segment
        ref_seg = self._get_ref_segment()   # (N+1, 3)

        # Solve NMPC — measure wall-clock time for one optimization step
        t_solve_start = time.perf_counter()
        u_opt, s_opt, success = self._solver.solve(self._current_state, ref_seg)
        solve_ms = (time.perf_counter() - t_solve_start) * 1000.0
        self._solve_ms_window.append(solve_ms)

        if not success:
            self.get_logger().warn(
                'NMPC solve failed — applying fallback input',
                throttle_duration_sec=1.0)

        # Publish first input (receding horizon)
        twist = Twist()
        twist.linear.x  = float(u_opt[0, 0])
        twist.angular.z = float(u_opt[0, 1])
        self._cmd_pub.publish(twist)

        # Rolling stats for diagnostics
        sw = np.asarray(self._solve_ms_window)
        solve_stat = (f'solve {solve_ms:6.1f}ms '
                      f'(avg {sw.mean():5.1f} p95 {np.percentile(sw, 95):5.1f} '
                      f'max {sw.max():5.1f})')
        if len(self._odom_lat_ms_window) > 0:
            lw = np.asarray(self._odom_lat_ms_window)
            odom_stat = (f'odom_lat avg {lw.mean():5.1f}ms p95 {np.percentile(lw, 95):5.1f} '
                         f'max {lw.max():5.1f}')
        else:
            odom_stat = 'odom_lat n/a'

        self.get_logger().info(
            f'[NMPC] idx={self._ref_idx}/{self._T} '
            f'pos=[{pos[0]:.2f},{pos[1]:.2f}] '
            f'theta={math.degrees(self._current_state[2]):.1f}deg '
            f'cmd=[v={twist.linear.x:.3f} w={twist.angular.z:.3f}] '
            f'ok={success} | {solve_stat} | {odom_stat}',
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
