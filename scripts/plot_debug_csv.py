#!/usr/bin/env python3
"""
PX4 Backtrack Debug 可视化工具

用法:
  # 指定文件
  python3 plot_debug_csv.py waypoints.csv trajectory.csv

  # 自动选取目录下最新的一组文件
  python3 plot_debug_csv.py --dir ~/.ros/backtrack_debug/ --latest
"""

import argparse
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_csv(path):
    """加载 CSV（跳过 # 注释行）"""
    return np.loadtxt(path, delimiter=",", comments="#")


def find_latest_pair(directory):
    """在目录中找到最新的 waypoints/trajectory CSV 对"""
    wp_files = sorted(glob.glob(os.path.join(directory, "waypoints_*.csv")))
    tr_files = sorted(glob.glob(os.path.join(directory, "trajectory_*.csv")))
    if not wp_files:
        sys.exit(f"错误: 在 {directory} 中未找到 waypoints_*.csv")
    if not tr_files:
        sys.exit(f"错误: 在 {directory} 中未找到 trajectory_*.csv")
    return wp_files[-1], tr_files[-1]


def plot(wp_path, tr_path):
    """绘制 2x2 子图: 3D对比、俯视图、高度剖面、偏航剖面"""
    wp = load_csv(wp_path)  # columns: timestamp, x, y, z, yaw
    tr = load_csv(tr_path)  # columns: t, x, y, z, vx, vy, vz, yaw, yaw_rate

    wp_t, wp_x, wp_y, wp_z, wp_yaw = wp[:, 0], wp[:, 1], wp[:, 2], wp[:, 3], wp[:, 4]
    tr_t, tr_x, tr_y, tr_z = tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3]
    tr_yaw = tr[:, 7]

    fig = plt.figure(figsize=(14, 11))
    fig.suptitle("PX4 Backtrack Debug 可视化", fontsize=14, fontweight="bold")

    # ---- 1) 3D 飞行路径对比 ----
    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.plot(wp_x, wp_y, wp_z, "o-", color="green", markersize=2, linewidth=0.8,
             alpha=0.6, label="录制航点")
    ax1.plot(tr_x, tr_y, tr_z, "-", color="royalblue", linewidth=1.5,
             label="返航轨迹")
    # 起止标记
    ax1.scatter(*wp[:1, 1:4].T, color="red", s=80, zorder=5, label="录制起点")
    ax1.scatter(*wp[-1:, 1:4].T, color="orange", s=80, zorder=5, label="录制终点")
    ax1.scatter(*tr[:1, 1:4].T, color="blue", s=80, marker="^", zorder=5, label="轨迹起点")
    ax1.scatter(*tr[-1:, 1:4].T, color="purple", s=80, marker="v", zorder=5, label="轨迹终点")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title("3D 飞行路径")
    ax1.legend(fontsize=7, loc="upper left")

    # ---- 2) 俯视图 (XY) ----
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(wp_x, wp_y, "o-", color="green", markersize=2, linewidth=0.8,
             alpha=0.6, label="录制航点")
    ax2.plot(tr_x, tr_y, "-", color="royalblue", linewidth=1.5, label="返航轨迹")
    # 方向箭头（每隔 N 个点标注）
    n_arrows = min(15, len(tr_x) // 2)
    if n_arrows > 0:
        step = max(1, len(tr_x) // n_arrows)
        for i in range(0, len(tr_x) - 1, step):
            dx = tr_x[min(i + step, len(tr_x) - 1)] - tr_x[i]
            dy = tr_y[min(i + step, len(tr_y) - 1)] - tr_y[i]
            if abs(dx) + abs(dy) > 1e-6:
                ax2.annotate("", xy=(tr_x[i] + dx * 0.5, tr_y[i] + dy * 0.5),
                             xytext=(tr_x[i], tr_y[i]),
                             arrowprops=dict(arrowstyle="->", color="royalblue",
                                             lw=1.2))
    ax2.scatter(wp_x[0], wp_y[0], color="red", s=80, zorder=5)
    ax2.scatter(wp_x[-1], wp_y[-1], color="orange", s=80, zorder=5)
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title("俯视图 (XY)")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ---- 3) 高度剖面 (t vs Z) ----
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(wp_t, wp_z, "o-", color="green", markersize=2, linewidth=0.8,
             alpha=0.6, label="录制航点 Z")
    ax3.plot(tr_t, tr_z, "-", color="royalblue", linewidth=1.5, label="轨迹 Z")
    ax3.set_xlabel("时间 (s)")
    ax3.set_ylabel("高度 Z (m)")
    ax3.set_title("高度剖面")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ---- 4) 偏航剖面 (t vs yaw) ----
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(wp_t, np.degrees(wp_yaw), "o-", color="green", markersize=2,
             linewidth=0.8, alpha=0.6, label="录制航点 Yaw")
    ax4.plot(tr_t, np.degrees(tr_yaw), "-", color="royalblue", linewidth=1.5,
             label="轨迹 Yaw")
    ax4.set_xlabel("时间 (s)")
    ax4.set_ylabel("Yaw (deg)")
    ax4.set_title("偏航角剖面")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="PX4 Backtrack Debug 可视化 — 对比录制航点与返航轨迹")
    parser.add_argument("waypoints", nargs="?", help="waypoints CSV 文件路径")
    parser.add_argument("trajectory", nargs="?", help="trajectory CSV 文件路径")
    parser.add_argument("--dir", default="", help="CSV 输出目录 (配合 --latest 使用)")
    parser.add_argument("--latest", action="store_true",
                        help="自动选取目录下最新的一组文件")
    args = parser.parse_args()

    if args.latest or args.dir:
        d = args.dir if args.dir else os.path.expanduser("~/.ros/backtrack_debug")
        wp_path, tr_path = find_latest_pair(d)
        print(f"Waypoints: {wp_path}")
        print(f"Trajectory: {tr_path}")
    elif args.waypoints and args.trajectory:
        wp_path, tr_path = args.waypoints, args.trajectory
    else:
        parser.print_help()
        sys.exit(1)

    plot(wp_path, tr_path)


if __name__ == "__main__":
    main()
