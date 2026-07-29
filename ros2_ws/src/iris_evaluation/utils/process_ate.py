#!/usr/bin/env python3
"""
ATE processing script — runs inside ~/evo_env Python interpreter.
Called by run_ate.py, or directly for testing:
    ~/evo_env/bin/python3 process_ate.py --bag <path_to_ate_bag> --out <output_dir>
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")   # non-interactive backend — no GUI needed
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from evo.tools import file_interface
from evo.core  import metrics, sync
import plotly.graph_objects as go

GT_TOPIC   = '/ground_truth/pose'
SLAM_TOPIC = '/mavros/vision_pose/pose'


def main():
    parser = argparse.ArgumentParser(description='ATE processing script')
    parser.add_argument('--bag', required=True, help='Path to ate_bag directory')
    parser.add_argument('--out', required=True, help='Output directory for results')
    args = parser.parse_args()

    bag_path = os.path.expanduser(args.bag)
    run_dir  = os.path.expanduser(args.out)

    print(f'Bag     : {bag_path}')
    print(f'Out dir : {run_dir}')

    # ── load trajectories via rosbags reader ─────────────────────────────────
    with Reader(bag_path) as reader:
        traj_ref = file_interface.read_bag_trajectory(reader, GT_TOPIC)
        traj_est = file_interface.read_bag_trajectory(reader, SLAM_TOPIC)

    print(f'GT poses  : {len(traj_ref.positions_xyz)}')
    print(f'EST poses : {len(traj_est.positions_xyz)}')

    # ── associate & align ─────────────────────────────────────────────────────
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.5)
    traj_est.align(traj_ref, correct_scale=False)
    traj_est_aligned = traj_est

    # ── compute ATE ───────────────────────────────────────────────────────────
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref, traj_est_aligned))
    stats = ape_metric.get_all_statistics()

    # ── print & save metrics ──────────────────────────────────────────────────
    lines = [
        f"ATE Results — {os.path.basename(bag_path)}",
        f"Compared: {GT_TOPIC}  vs  {SLAM_TOPIC}",
        f"Poses   : {len(traj_ref.positions_xyz)}",
        "",
        f"RMSE   : {stats['rmse']:.4f} m",
        f"Mean   : {stats['mean']:.4f} m",
        f"Median : {stats['median']:.4f} m",
        f"Std    : {stats['std']:.4f} m",
        f"Min    : {stats['min']:.4f} m",
        f"Max    : {stats['max']:.4f} m",
    ]
    text = "\n".join(lines)
    print("\n" + text)

    metrics_txt = os.path.join(run_dir, 'ate_metrics.txt')
    with open(metrics_txt, 'w') as f:
        f.write(text + "\n")
    print(f'\nMetrics saved to {metrics_txt}')

    # ── error-over-time plot ──────────────────────────────────────────────────
    errors     = ape_metric.error
    timestamps = traj_est_aligned.timestamps - traj_est_aligned.timestamps[0]

    fig_err, ax_err = plt.subplots(figsize=(10, 4))
    ax_err.plot(timestamps, errors, color="crimson", linewidth=1.0)
    ax_err.axhline(stats["rmse"], color="navy",   linestyle="--", linewidth=1.0,
                   label=f"RMSE {stats['rmse']:.4f} m")
    ax_err.axhline(stats["mean"], color="orange", linestyle=":",  linewidth=1.0,
                   label=f"Mean {stats['mean']:.4f} m")
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("APE (m)")
    ax_err.set_title("ATE — Error over Time")
    ax_err.legend()
    ax_err.grid(True)
    fig_err.tight_layout()
    err_png = os.path.join(run_dir, 'ate_error.png')
    fig_err.savefig(err_png, dpi=150)
    plt.close(fig_err)
    print(f'Error plot saved to {err_png}')

    # ── static 3D trajectory plot ─────────────────────────────────────────────
    # fig_3d = plt.figure(figsize=(10, 7))
    # ax_3d  = fig_3d.add_subplot(111, projection='3d')
    # ax_3d.plot(traj_ref.positions_xyz[:, 0],
    #            traj_ref.positions_xyz[:, 1],
    #            traj_ref.positions_xyz[:, 2],
    #            label="Ground Truth", color="green", linewidth=2.0, linestyle='--')
    # ax_3d.plot(traj_est_aligned.positions_xyz[:, 0],
    #            traj_est_aligned.positions_xyz[:, 1],
    #            traj_est_aligned.positions_xyz[:, 2],
    #            label="SLAM Estimate", color="blue", linewidth=1.5)
    # ax_3d.set_xlabel("X (m)")
    # ax_3d.set_ylabel("Y (m)")
    # ax_3d.set_zlabel("Z (m)")
    # ax_3d.set_title("ATE — 3D Trajectory Comparison")
    # ax_3d.legend()
    # fig_3d.tight_layout()
    # traj_3d_png = os.path.join(run_dir, 'ate_traj_3d.png')
    # fig_3d.savefig(traj_3d_png, dpi=150)
    # plt.close(fig_3d)
    # print(f'3D trajectory plot saved to {traj_3d_png}')

    # ── interactive 3D trajectory (rotatable HTML) ────────────────────────────
    fig_int = go.Figure()
    fig_int.add_trace(go.Scatter3d(
        x=traj_ref.positions_xyz[:, 0],
        y=traj_ref.positions_xyz[:, 1],
        z=traj_ref.positions_xyz[:, 2],
        mode='lines',
        name='Ground Truth',
        line=dict(color='rgb(25,255,0)', width=4),
    ))
    fig_int.add_trace(go.Scatter3d(
        x=traj_est_aligned.positions_xyz[:, 0],
        y=traj_est_aligned.positions_xyz[:, 1],
        z=traj_est_aligned.positions_xyz[:, 2],
        mode='lines',
        name='SLAM Estimate',
        line=dict(color='rgb(25,0,255)', width=4),
    ))
    fig_int.update_layout(
        title='ATE — 3D Trajectory Comparison (interactive)',
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data',
        ),
        legend=dict(x=0.01, y=0.99),
    )
    html_path = os.path.join(run_dir, 'ate_traj_3d.html')
    fig_int.write_html(html_path)
    print(f'Interactive 3D plot saved to {html_path}')


if __name__ == '__main__':
    main()
