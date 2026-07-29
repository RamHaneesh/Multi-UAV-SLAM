#!/usr/bin/env python3
"""
ATE Evaluation Orchestrator
Runs: ground_truth_publisher → bag record → mission_planner → evo_ape (via API)
Usage (direct):  python3 run_ate.py --mission <n>
Usage (launch):  ros2 launch iris_evaluation ate.launch.py m:=<n>
"""

import argparse
import os
import signal
import subprocess
import time
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

# ── paths ────────────────────────────────────────────────────────────────────
EVAL_DIR = os.path.expanduser(
    '~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval'
)
EVO_PYTHON = os.path.expanduser('~/evo_env/bin/python3')

GT_TOPIC   = '/ground_truth/pose'
SLAM_TOPIC = '/mavros/vision_pose/pose'
# ─────────────────────────────────────────────────────────────────────────────


def log(msg: str):
    print(f'[ate] {msg}', flush=True)


def run_evo(bag_path: str, run_dir: str):
    script = os.path.join(
        get_package_share_directory('iris_evaluation'),
        'utils', 'process_ate.py'
    )
    ret = subprocess.call([EVO_PYTHON, script, '--bag', bag_path, '--out', run_dir])
    if ret != 0:
        log(f'WARNING: process_ate.py exited with code {ret}')
    else:
        log(f'Results saved in {run_dir}')


def main():
    parser = argparse.ArgumentParser(description='ATE Evaluation Orchestrator')
    parser.add_argument('--mission', required=True, help='Mission name (no .json)')
    args = parser.parse_args()

    mission   = args.mission
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir   = os.path.join(EVAL_DIR, f'{mission}_{timestamp}')
    bag_path  = os.path.join(run_dir, 'ate_bag')
    os.makedirs(run_dir, exist_ok=True)

    log(f'Mission : {mission}')
    log(f'Output  : {run_dir}')

    # ── 1. ground_truth_publisher ────────────────────────────────────────────
    log('Starting ground_truth_publisher...')
    env_gt = os.environ.copy()
    env_gt['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

    proc_gt = subprocess.Popen(
        ['ros2', 'run', 'iris_evaluation', 'ground_truth_publisher',
         '--ros-args', '-p', 'use_sim_time:=true'],
        env=env_gt,
    )
    time.sleep(2.0)

    # ── 2. bag recording ─────────────────────────────────────────────────────
    log('Starting bag recording...')
    proc_bag = subprocess.Popen(
        ['ros2', 'bag', 'record', GT_TOPIC, SLAM_TOPIC, '-o', bag_path],
    )
    time.sleep(1.0)

    # ── 3. mission_planner (blocking) ────────────────────────────────────────
    log(f'Running mission: {mission}')
    ret = subprocess.call(
        ['ros2', 'run', 'iris_control', 'mission_planner',
         '--ros-args', '-p', f'm:={mission}'],
    )
    if ret != 0:
        log(f'WARNING: mission_planner exited with code {ret}')

    # ── 4. stop bag ───────────────────────────────────────────────────────────
    log('Stopping bag recording...')
    proc_bag.send_signal(signal.SIGINT)
    try:
        proc_bag.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc_bag.kill()
    time.sleep(1.5)   # let bag flush

    # ── 5. stop ground truth publisher ──────────────────────────────────────
    proc_gt.send_signal(signal.SIGINT)
    try:
        proc_gt.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc_gt.kill()

    # ── 6. ATE ───────────────────────────────────────────────────────────────
    log('Running ATE evaluation...')
    run_evo(bag_path, run_dir)

    log('Done.')
    log(f'Results: {run_dir}')


if __name__ == '__main__':
    main()
