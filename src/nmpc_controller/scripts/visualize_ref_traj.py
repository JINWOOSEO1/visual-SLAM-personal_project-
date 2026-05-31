#!/usr/bin/env python3
"""
Visualize NMPC reference and measured trajectories on the x-y plane.

Trajectory files must be .npy arrays with shape (T, 3):
each row is [x, y, theta]. Every Nth waypoint is drawn as an
oriented rectangle to show the RC car pose and heading.
"""

import argparse
import math
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", tempfile.gettempdir())

import matplotlib.pyplot as plt
import numpy as np


def car_corners(x: float, y: float, theta: float, length: float, width: float) -> np.ndarray:
    """Return the four corners of a rectangle centered at (x, y)."""
    half_l = length / 2.0
    half_w = width / 2.0
    local = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
        ]
    )
    rot = np.array(
        [
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)],
        ]
    )
    return local @ rot.T + np.array([x, y])


def draw_car(
    ax,
    x: float,
    y: float,
    theta: float,
    length: float,
    width: float,
    color: str,
    alpha: float,
) -> None:
    corners = car_corners(x, y, theta, length, width)
    closed = np.vstack([corners, corners[0]])

    ax.plot(closed[:, 0], closed[:, 1], color=color, linewidth=1.4)
    ax.fill(corners[:, 0], corners[:, 1], color=color, alpha=alpha)

    nose = np.array(
        [
            x + math.cos(theta) * length * 0.5,
            y + math.sin(theta) * length * 0.5,
        ]
    )
    ax.arrow(
        x,
        y,
        nose[0] - x,
        nose[1] - y,
        width=0.003,
        head_width=0.04,
        head_length=0.05,
        length_includes_head=True,
        color=color,
    )


def load_trajectory(path: Path) -> np.ndarray:
    traj = np.load(path)
    if traj.ndim != 2 or traj.shape[1] != 3:
        raise ValueError(f"Expected trajectory shape (T, 3), got {traj.shape}")
    return traj


def resolve_path(path: Path, script_dir: Path) -> Path:
    return path if path.is_absolute() else script_dir / path


def draw_pose_markers(
    ax,
    traj: np.ndarray,
    pose_interval: int,
    car_length: float,
    car_width: float,
    color: str,
    alpha: float,
    text_prefix: str,
) -> None:
    pose_indices = list(range(0, len(traj), pose_interval))
    if pose_indices[-1] != len(traj) - 1:
        pose_indices.append(len(traj) - 1)

    for idx in pose_indices:
        x, y, theta = traj[idx]
        draw_car(ax, x, y, theta, car_length, car_width, color, alpha)
        ax.text(
            x,
            y,
            f"{text_prefix}{idx}",
            fontsize=8,
            color=color,
            ha="center",
            va="center",
        )


def main() -> None:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Visualize an RC car reference trajectory")
    parser.add_argument(
        "--traj",
        type=Path,
        default=script_dir / "ref_traj.npy",
        help="Path to .npy trajectory file with rows [x, y, theta]",
    )
    parser.add_argument(
        "--real-traj",
        type=Path,
        default=script_dir / "real_traj.npy",
        help="Path to measured odometry .npy trajectory. Skipped if the file does not exist.",
    )
    parser.add_argument(
        "--pose-interval",
        type=int,
        default=10,
        help="Draw one reference RC car rectangle every this many waypoints",
    )
    parser.add_argument(
        "--real-pose-interval",
        type=int,
        default=100,
        help="Draw one measured RC car rectangle every this many odometry samples",
    )
    parser.add_argument("--car-length", type=float, default=0.30, help="RC car length [m]")
    parser.add_argument("--car-width", type=float, default=0.18, help="RC car width [m]")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional image output path. If omitted, show an interactive window.",
    )
    args = parser.parse_args()

    ref_path = resolve_path(args.traj, script_dir)
    ref_traj = load_trajectory(ref_path)
    real_path = resolve_path(args.real_traj, script_dir)
    real_traj = load_trajectory(real_path) if real_path.exists() else None

    if args.pose_interval <= 0:
        raise ValueError("--pose-interval must be positive")
    if args.real_pose_interval <= 0:
        raise ValueError("--real-pose-interval must be positive")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(ref_traj[:, 0], ref_traj[:, 1], color="tab:blue", linewidth=2.0, label="ref_traj")
    ax.scatter(ref_traj[:, 0], ref_traj[:, 1], s=16, color="tab:blue", alpha=0.45, label="ref waypoints")
    draw_pose_markers(
        ax,
        ref_traj,
        args.pose_interval,
        args.car_length,
        args.car_width,
        "tab:red",
        0.10,
        "r",
    )

    title = f"RC Car Trajectory: {ref_path.name}"
    if real_traj is not None:
        ax.plot(real_traj[:, 0], real_traj[:, 1], color="tab:green", linewidth=2.0, label="real_traj")
        ax.scatter(
            real_traj[:, 0],
            real_traj[:, 1],
            s=12,
            color="tab:green",
            alpha=0.45,
            label="real waypoints",
        )
        draw_pose_markers(
            ax,
            real_traj,
            args.real_pose_interval,
            args.car_length,
            args.car_width,
            "darkorange",
            0.12,
            "a",
        )
        title += f" vs {real_path.name}"
    else:
        print(f"Measured trajectory not found, plotting reference only: {real_path}")

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.5)
    ax.legend(loc="best")
    fig.tight_layout()

    if args.output is None:
        plt.show()
    else:
        output_path = args.output if args.output.is_absolute() else Path.cwd() / args.output
        fig.savefig(output_path, dpi=180)
        print(f"Saved visualization: {output_path}")


if __name__ == "__main__":
    main()
