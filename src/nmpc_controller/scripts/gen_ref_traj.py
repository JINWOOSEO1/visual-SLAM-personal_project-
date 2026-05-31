#!/usr/bin/env python3
"""
gen_ref_traj.py — Reference trajectory generator for nmpc_controller.

Supported shapes:
    circle  : constant-speed circular arc
    figure8 : lemniscate (infinity symbol), arc-length resampled for uniform speed
    line    : straight line

Output: .npy file, shape (T, 3), each row = [x, y, theta]
theta is the tangent direction of the trajectory at each point.
The generated trajectory is normalized so the initial pose is always (0, 0, 0).

Usage examples:
    python3 gen_ref_traj.py --shape circle  --radius 1.0 --speed 0.2 --dt 0.1
    python3 gen_ref_traj.py --shape figure8 --radius 0.8 --speed 0.15 --dt 0.1
    python3 gen_ref_traj.py --shape line    --length 3.0 --speed 0.2  --dt 0.1
    python3 gen_ref_traj.py --shape circle  --radius 1.0 --speed 0.2  --dt 0.1 \\
                             --n_laps 2 --output traj_circle_2lap.npy
"""

import argparse
import math
from pathlib import Path
import numpy as np


def normalize_initial_pose(traj: np.ndarray) -> np.ndarray:
    """Transform trajectory into the initial pose frame: first row becomes (0, 0, 0)."""
    x0, y0, theta0 = traj[0]
    c = math.cos(-theta0)
    s = math.sin(-theta0)

    xy = traj[:, :2] - np.array([x0, y0])
    x = c * xy[:, 0] - s * xy[:, 1]
    y = s * xy[:, 0] + c * xy[:, 1]
    theta = np.arctan2(np.sin(traj[:, 2] - theta0), np.cos(traj[:, 2] - theta0))

    return np.column_stack([x, y, theta])


def gen_circle(radius: float, speed: float, dt: float, n_laps: float = 1.0) -> np.ndarray:
    """
    Constant-speed circular trajectory (counter-clockwise).
    Start point: (radius, 0).  theta = tangent direction = angle + pi/2.
    """
    circumference = 2 * math.pi * radius
    total_time    = n_laps * circumference / speed
    T             = int(total_time / dt) + 1

    angles = np.linspace(0.0, n_laps * 2 * math.pi, T)
    x      = radius * np.cos(angles)
    y      = radius * np.sin(angles)
    theta  = angles + math.pi / 2.0

    return np.column_stack([x, y, theta])


def gen_figure8(radius: float, speed: float, dt: float, n_laps: float = 1.0) -> np.ndarray:
    """
    Lemniscate (figure-8) trajectory.
    Parametric: x = A*sin(t),  y = (A/2)*sin(2t)
    Arc-length resampled so that consecutive points are separated by speed*dt.
    theta computed from finite differences of the resampled path.
    """
    resolution = 20000
    t_fine = np.linspace(0.0, n_laps * 2 * math.pi, resolution)

    x_fine = radius * np.sin(t_fine)
    y_fine = (radius / 2.0) * np.sin(2.0 * t_fine)

    # Arc-length cumulation
    ds          = np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2)
    arc_lengths = np.concatenate([[0.0], np.cumsum(ds)])
    total_len   = arc_lengths[-1]

    # Uniform arc-length resampling
    T         = int(total_len / (speed * dt)) + 1
    s_uniform = np.linspace(0.0, total_len, T)
    x         = np.interp(s_uniform, arc_lengths, x_fine)
    y         = np.interp(s_uniform, arc_lengths, y_fine)

    # Tangent direction via central differences
    dx    = np.gradient(x)
    dy    = np.gradient(y)
    theta = np.arctan2(dy, dx)

    return np.column_stack([x, y, theta])


def gen_line(length: float, speed: float, dt: float, direction: float = 0.0) -> np.ndarray:
    """Straight-line trajectory in the given direction [rad]."""
    total_time = length / speed
    T          = int(total_time / dt) + 1

    t_arr = np.linspace(0.0, total_time, T)
    x     = speed * math.cos(direction) * t_arr
    y     = speed * math.sin(direction) * t_arr
    theta = np.full(T, direction)

    return np.column_stack([x, y, theta])


def main():
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description='Reference trajectory generator')
    parser.add_argument('--shape',     choices=['circle', 'figure8', 'line'],
                        default='circle')
    parser.add_argument('--radius',    type=float, default=1.0,
                        help='Radius for circle/figure8 [m]')
    parser.add_argument('--length',    type=float, default=3.0,
                        help='Length for line trajectory [m]')
    parser.add_argument('--speed',     type=float, default=0.3,
                        help='Reference speed [m/s] — keep within NMPC speed constraints')
    parser.add_argument('--dt',        type=float, default=0.1,
                        help='Sample interval [s] — must match NMPC dt parameter')
    parser.add_argument('--n_laps',    type=float, default=1.0,
                        help='Number of laps for circle/figure8')
    parser.add_argument('--direction', type=float, default=0.0,
                        help='Direction for line [rad]')
    parser.add_argument('--output',    type=str,   default='ref_traj.npy',
                        help='Output .npy file path; relative paths are saved under this scripts directory')
    args = parser.parse_args()

    if args.shape == 'circle':
        traj = gen_circle(args.radius, args.speed, args.dt, args.n_laps)
    elif args.shape == 'figure8':
        traj = gen_figure8(args.radius, args.speed, args.dt, args.n_laps)
    else:
        traj = gen_line(args.length, args.speed, args.dt, args.direction)

    traj = normalize_initial_pose(traj)

    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = script_dir / output_path

    np.save(output_path, traj)

    print(f'Saved: {output_path}  shape={traj.shape}')
    print(f'  Start : x={traj[0,0]:.3f}  y={traj[0,1]:.3f}  '
          f'theta={math.degrees(traj[0,2]):.1f} deg')
    print(f'  End   : x={traj[-1,0]:.3f}  y={traj[-1,1]:.3f}  '
          f'theta={math.degrees(traj[-1,2]):.1f} deg')
    print(f'  Points: {traj.shape[0]}  Duration: {traj.shape[0] * args.dt:.1f} s')


if __name__ == '__main__':
    main()
