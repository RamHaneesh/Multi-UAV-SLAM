**[⬅️ Back to README](../README.md)**

## **ATE Evaluation**

This document explains how to evaluate the Absolute Trajectory Error (ATE) of the ORB-SLAM3 VSLAM pipeline against Gazebo ground truth using the `evo` library.

---

### **1. Why ATE?**

ATE measures the absolute difference between the SLAM estimated trajectory and the true trajectory after global alignment. It quantifies how accurately the SLAM system tracks the UAV position over the full mission.

**What we compare:**

| Topic | Source | Description |
|---|---|---|
| `/ground_truth/pose` | Gazebo world pose of `iris` model | True UAV position from simulator |
| `/mavros/vision_pose/pose` | ORB-SLAM3 output via `slam_bridge.py` | SLAM estimated position in ROS ENU frame |

The ground truth is the Gazebo world pose of the `iris` model published by `ground_truth_publisher` in the `iris_evaluation` package. This is the closest to true ground truth available in simulation — it is the simulator's internal state, unaffected by sensor noise or filter drift.

> **Note:** `/mavros/local_position/odom` (EKF3 output) is **not** used as ground truth. In Gazebo SITL, EKF3 receives near-perfect IMU data so it appears accurate, but it is still a filtered estimate — not the simulator's true state.

---

### **2. Prerequisites**

#### **2.1 Install evo in a virtual environment**

`evo` requires a newer version of matplotlib than the system matplotlib used by ROS2. Installing `evo` directly into the system Python environment will cause a conflict that breaks ROS2 visualization tools. A dedicated virtual environment is required.

```bash
python3 -m venv ~/evo_env
source ~/evo_env/bin/activate
pip install evo matplotlib PyQt6 rosbags plotly
deactivate
```

Add a convenience alias to `~/.bashrc`:

```bash
echo "alias evo_env='source ~/evo_env/bin/activate'" >> ~/.bashrc
source ~/.bashrc
```

To activate the evo environment in any terminal: `evo_env`

> **Why a separate venv?** The system matplotlib (installed via `apt` as a ROS2 dependency) is version 3.5.x. `evo` requires matplotlib ≥ 3.6.0. Installing via `pip` into the system environment causes `mpl_toolkits` import errors that crash both `evo` plots and ROS2 tools like `rqt_plot`. The virtual environment isolates `evo`'s dependencies completely.

> **Why `rosbags`?** Newer versions of `evo` no longer accept a bag path string directly. Instead they require a `rosbags.rosbag2.Reader` object to be passed to `file_interface.read_bag_trajectory`. `rosbags` is the correct ROS2 bag reader for this.

#### **2.2 iris_evaluation package**

The `iris_evaluation` package contains `ground_truth_publisher.py` which subscribes to the Gazebo world pose topic and publishes to `/ground_truth/pose`.

It requires the `gz.transport13` and `gz.msgs10` Python bindings which are installed with Gazebo Harmonic. Due to a protobuf version conflict, the node must be launched with an environment variable:

```bash
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python ros2 run iris_evaluation ground_truth_publisher --ros-args -p use_sim_time:=true
```

> **Path configuration:** The paths in `run_ate.py` and the commands in Section 4 assume the repository is cloned to `~/Desktop/multi_uav_slam`. If you have cloned it elsewhere, you must update one constant near the top of `iris_evaluation/utils/run_ate.py`:
> ```python
> EVAL_DIR = os.path.expanduser('<your_path>/ros2_ws/src/iris_evaluation/eval')
> ```
> For example, if cloned to `~/projects/multi_uav_slam`:
> ```python
> EVAL_DIR = os.path.expanduser('~/projects/multi_uav_slam/ros2_ws/src/iris_evaluation/eval')
> ```
> This is the only file that needs to be changed. All paths in `process_ate.py` are passed as arguments at runtime and do not need to be edited.

---

### **3. Approach A — Automated ATE (recommended)**

The `ate.launch.py` launch file automates the full pipeline in a single command. It starts the ground truth publisher, begins bag recording, runs the mission, stops recording on mission completion, and runs `process_ate.py` to produce all results automatically.

> **Prerequisites:** The full VSLAM stack must already be running (Terminals 1–4 from [Section 9](../README.md#9-single-uav-visual-slam-️-️)) and VSLAM must have initialised, and wait for `pre-arm good` in mavproxy console before proceeding.

```bash
ros2 launch iris_evaluation ate.launch.py m:=<mission_name>
```

**Example:**
```bash
ros2 launch iris_evaluation ate.launch.py m:=lcm
```

Results are saved automatically to:
```
eval/<mission_name>_<timestamp>/
├── ate_bag/            ← ROS2 bag
├── ate_metrics.txt     ← RMSE, mean, median, std, min, max
├── ate_error.png       ← APE over time plot
├── ate_traj_3d.png     ← static 3D trajectory comparison
└── ate_traj_3d.html    ← interactive rotatable 3D trajectory (open in browser)
```

---

### **4. Approach B — Manual ATE**

Use this approach to re-evaluate an existing bag, or when you want to control each step individually.

#### **Step 1 — Run the full VSLAM stack**

Follow [Section 9](../README.md#9-single-uav-visual-slam-️-️). Wait for `pre-arm good` in mavproxy console before proceeding.

#### **Step 2 — Start ground truth publisher (Terminal A)**

```bash
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python ros2 run iris_evaluation ground_truth_publisher --ros-args -p use_sim_time:=true
```

#### **Step 3 — Start bag recording (Terminal B)**

> Replace `~/Desktop/multi_uav_slam` with your actual repository root path if different.

```bash
mkdir -p ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval
cd ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval
ros2 bag record /ground_truth/pose /mavros/vision_pose/pose -o ate_bag
```

#### **Step 4 — Run mission (Terminal C)**

```bash
ros2 run iris_control mission_planner --ros-args -p m:=<mission_name>
```

Wait for the full mission to complete and UAV to land, then `Ctrl+C` on the bag recording.

> **Use `world:=straight_path_with_pillars`** for ATE evaluation. This world provides a clear 36m straight corridor with textured pillars, giving a clean trajectory for drift measurement.

#### **Step 5 — Run process_ate.py**

> Replace `~/Desktop/multi_uav_slam` with your actual repository root path if different. The `--bag` argument must point to the `ate_bag` directory inside your run folder, and `--out` to the run folder itself.

```bash
~/evo_env/bin/python3 ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/utils/process_ate.py \
    --bag ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval/ate_bag \
    --out ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval
```

If re-evaluating an existing run from the automated folder structure, point `--bag` and `--out` to the existing `<mission>_<timestamp>/` folder:

```bash
~/evo_env/bin/python3 ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/utils/process_ate.py \
    --bag ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval/<mission>_<timestamp>/ate_bag \
    --out ~/Desktop/multi_uav_slam/ros2_ws/src/iris_evaluation/eval/<mission>_<timestamp>
```

---

### **5. Interpreting Results**

`process_ate.py` outputs the following metrics (all values in metres):

| Metric | Description |
|---|---|
| `rmse` | Root Mean Square Error — primary ATE metric |
| `mean` | Mean absolute error |
| `median` | Median absolute error (robust to outliers) |
| `std` | Standard deviation of the error |
| `min` | Minimum error across all poses |
| `max` | Maximum error at any point in the trajectory |

**Output files:**

| File | Description |
|---|---|
| `ate_metrics.txt` | All metrics saved as plain text |
| `ate_error.png` | APE plotted over time with RMSE and mean reference lines |
| `ate_traj_3d.png` | Static 3D plot — ground truth (green dashed) vs SLAM estimate (blue) |
| `ate_traj_3d.html` | Interactive 3D plot — open in any browser to rotate, zoom, and pan |

---

### **6. Notes on Result Interpretation**

- Results using `ate.launch.py` or `process_ate.py` use Umeyama SE(3) alignment (`traj_est.align(traj_ref, correct_scale=False)`) which handles the origin difference between the SLAM map frame and the Gazebo world frame. This is equivalent to `--align` in the `evo_ape` CLI.
- The `lcm` mission (±4m perimeter with loop closure) will produce lower ATE than the straight path mission because loop closure actively corrects accumulated drift.
- For a conservative drift measurement, use `world:=straight_path_with_pillars` with `m:=mission` (no loop closure).

---
---