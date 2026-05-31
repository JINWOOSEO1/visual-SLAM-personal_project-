#!/usr/bin/env python3
"""
nmpc_node.py

Subscribes to /odom and /imu/data, fuses yaw at 50 Hz, runs NMPC, publishes /cmd_vel.

Reference index tracking (monotonic):
  - First call: a = 0
  - Each step: search the window [a, a + W - 1] and pick the waypoint that
               minimizes position distance + heading_weight * |heading error|
               (strictly monotonic, no back-snap)
"""

import math
from pathlib import Path
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from nmpc_controller.nmpc_solver import NMPCSolver


def default_real_traj_file(traj_file: str) -> Path:
    traj_path = Path(traj_file).resolve()
    parts = traj_path.parts

    if 'install' in parts:
        install_idx = parts.index('install')
        workspace_root = Path(*parts[:install_idx])
        src_scripts = workspace_root / 'src' / 'nmpc_controller' / 'scripts'
        if src_scripts.is_dir():
            return src_scripts / 'real_traj.npy'

    return traj_path.with_name('real_traj.npy')


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        # Parameters
        self.declare_parameter('traj_file',    '')
        self.declare_parameter('real_traj_file', '')
        self.declare_parameter('N',            20)
        self.declare_parameter('dt',           0.1)
        self.declare_parameter('wheel_base',   0.150)
        self.declare_parameter('wheel_v_min',  0.1)
        self.declare_parameter('wheel_v_max',  0.64)
        self.declare_parameter('wheel_delta_max', 0.0)
        self.declare_parameter('Q_x',          2.0)
        self.declare_parameter('Q_y',          2.0)
        self.declare_parameter('Q_theta',      0.5)
        self.declare_parameter('R_v',          0.1)
        self.declare_parameter('R_w',          0.05)
        self.declare_parameter('QN_x',         4.0)
        self.declare_parameter('QN_y',         4.0)
        self.declare_parameter('QN_theta',     1.0)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('imu_timeout',  1.0)
        self.declare_parameter('fusion_rate',  50.0)
        self.declare_parameter('yaw_fusion_alpha', 0.5)
        self.declare_parameter('ref_search_window', 10)
        self.declare_parameter('ref_heading_weight', 0.5)

        traj_file        = self.get_parameter('traj_file').value
        real_traj_file   = self.get_parameter('real_traj_file').value
        N                = self.get_parameter('N').value
        dt               = self.get_parameter('dt').value
        wheel_base       = self.get_parameter('wheel_base').value
        wheel_v_min      = self.get_parameter('wheel_v_min').value
        wheel_v_max      = self.get_parameter('wheel_v_max').value
        wheel_delta_max  = self.get_parameter('wheel_delta_max').value
        self._odom_timeout = self.get_parameter('odom_timeout').value
        self._imu_timeout  = self.get_parameter('imu_timeout').value
        fusion_rate        = float(self.get_parameter('fusion_rate').value)
        self._yaw_fusion_alpha = float(self.get_parameter('yaw_fusion_alpha').value)
        self._ref_search_window  = self.get_parameter('ref_search_window').value
        self._ref_heading_weight = self.get_parameter('ref_heading_weight').value

        if fusion_rate <= 0.0:
            raise ValueError('fusion_rate must be positive')
        if not 0.0 <= self._yaw_fusion_alpha <= 1.0:
            raise ValueError('yaw_fusion_alpha must be in [0, 1]')
        self._fusion_period = 1.0 / fusion_rate

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
        self._real_traj_file = Path(real_traj_file) if real_traj_file else default_real_traj_file(traj_file)
        self._real_traj_file.parent.mkdir(parents=True, exist_ok=True)
        self._odom_origin = None
        self._real_traj = []
        self._real_save_stride = 10
        self._save_real_traj()

        # Build NMPC solver (CasADi NLP compiled once here)
        self.get_logger().info('Building NMPC solver (CasADi NLP) ...')
        self._solver = NMPCSolver(
            N=N, dt=dt, Q=Q, R=R, Q_N=Q_N,
            wheel_base=wheel_base,
            wheel_v_min=wheel_v_min, wheel_v_max=wheel_v_max,
            wheel_delta_max=wheel_delta_max,
        )
        self.get_logger().info('NMPC solver ready')

        # State
        self._current_state  = None   # np.ndarray (3,) or None
        self._last_odom_time = None   # rclpy.time.Time of the last fused sensor sample
        self._latest_odom_msg = None
        self._latest_odom_receive_time = None
        self._latest_imu_msg = None
        self._latest_imu_receive_time = None
        self._prev_odom_pose = None
        self._last_fusion_time = None
        self._fused_state = None
        self._ref_idx        = 0      # monotonic reference index a
        self._finished       = False

        # Diagnostics — rolling windows for NMPC solve time and odom latency
        self._solve_ms_window  = deque(maxlen=200)
        self._odom_lat_ms_window = deque(maxlen=200)
        self._imu_lat_ms_window = deque(maxlen=200)

        # ROS interfaces
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self._fusion_period, self._fusion_loop)
        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'NMPCNode started: N={N} dt={dt}s '
            f'fusion_rate={fusion_rate:.1f}Hz yaw_fusion_alpha={self._yaw_fusion_alpha:.2f} '
            f'wheel_base={wheel_base} wheel_v_min={wheel_v_min} '
            f'wheel_v_max={wheel_v_max} wheel_delta_max={wheel_delta_max} '
            f'traj_len={self._T} real_traj_file={self._real_traj_file}')

    # ------------------------------------------------------------------
    def _cb_odom(self, msg: Odometry):
        self._latest_odom_msg = msg
        self._latest_odom_receive_time = self.get_clock().now()

        # Latency = (receive time on this host) - (header.stamp set by publisher)
        # Meaningful only if clocks of publisher and this host are NTP-synced.
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns > 0:
            lat_ms = (self._latest_odom_receive_time.nanoseconds - stamp_ns) / 1e6
            self._odom_lat_ms_window.append(lat_ms)

    # ------------------------------------------------------------------
    def _cb_imu(self, msg: Imu):
        self._latest_imu_msg = msg
        self._latest_imu_receive_time = self.get_clock().now()

        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if stamp_ns > 0:
            lat_ms = (self._latest_imu_receive_time.nanoseconds - stamp_ns) / 1e6
            self._imu_lat_ms_window.append(lat_ms)

    # ------------------------------------------------------------------
    def _fusion_loop(self):
        if self._latest_odom_msg is None or self._latest_imu_msg is None:
            self.get_logger().warn(
                'Waiting for /odom and /imu/data...',
                throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        odom_age = (now - self._latest_odom_receive_time).nanoseconds / 1e9
        imu_age = (now - self._latest_imu_receive_time).nanoseconds / 1e9
        if odom_age > self._odom_timeout or imu_age > self._imu_timeout:
            self.get_logger().warn(
                f'Sensor timeout: odom_age={odom_age:.2f}s imu_age={imu_age:.2f}s',
                throttle_duration_sec=1.0)
            return

        odom_pose = self._odom_pose(self._latest_odom_msg)
        if self._fused_state is None:
            self._fused_state = odom_pose.copy()
            self._prev_odom_pose = odom_pose
            self._last_fusion_time = now
            self._current_state = self._fused_state.copy()
            self._last_odom_time = now
            self._record_real_traj_sample(self._current_state)
            return

        dt = (now - self._last_fusion_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            dt = self._fusion_period

        dx_odom = odom_pose[0] - self._prev_odom_pose[0]
        dy_odom = odom_pose[1] - self._prev_odom_pose[1]
        d_center = dx_odom * math.cos(odom_pose[2]) + dy_odom * math.sin(odom_pose[2])
        d_yaw_enc = self._wrap_angle(odom_pose[2] - self._prev_odom_pose[2])
        d_yaw_imu = self._latest_imu_msg.angular_velocity.z * dt
        alpha = self._yaw_fusion_alpha
        d_yaw_fused = (1.0 - alpha) * d_yaw_enc + alpha * d_yaw_imu

        yaw = self._wrap_angle(self._fused_state[2] + d_yaw_fused)
        x = self._fused_state[0] + d_center * math.cos(yaw)
        y = self._fused_state[1] + d_center * math.sin(yaw)

        self._fused_state = np.array([x, y, yaw])
        self._prev_odom_pose = odom_pose
        self._last_fusion_time = now
        self._current_state = self._fused_state.copy()
        self._last_odom_time = now
        self._record_real_traj_sample(self._current_state)

    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    @classmethod
    def _odom_pose(cls, msg: Odometry) -> np.ndarray:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        ])

    # ------------------------------------------------------------------
    def _record_real_traj_sample(self, state: np.ndarray):
        if self._odom_origin is None:
            self._odom_origin = state.copy()

        x0, y0, theta0 = self._odom_origin
        dx = state[0] - x0
        dy = state[1] - y0
        c = math.cos(-theta0)
        s = math.sin(-theta0)
        x_rel = c * dx - s * dy
        y_rel = s * dx + c * dy
        yaw_rel = math.atan2(
            math.sin(state[2] - theta0),
            math.cos(state[2] - theta0),
        )

        self._real_traj.append([x_rel, y_rel, yaw_rel])
        if len(self._real_traj) == 1 or len(self._real_traj) % self._real_save_stride == 0:
            self._save_real_traj()

    # ------------------------------------------------------------------
    def _save_real_traj(self):
        traj = np.asarray(self._real_traj, dtype=float).reshape(-1, 3)
        np.save(self._real_traj_file, traj)

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
            self.get_logger().warn('Waiting for fused odometry...', throttle_duration_sec=2.0)
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
            self._save_real_traj()
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
        node._save_real_traj()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
