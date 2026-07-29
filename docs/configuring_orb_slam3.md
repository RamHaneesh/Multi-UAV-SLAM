## Configuring ORB-SLAM3 for Iris UAV

**[⬅️ Back to README](../README.md)**

This document explains the `orbslam3_config.yaml` file used by ORB-SLAM3 **stereo mode** for the Iris UAV simulation.

> **Note on Stereo-Inertial Mode:** ORB-SLAM3 was initially configured in stereo-inertial mode (`IMU_STEREO`). The UAV took off and appeared stable for a few seconds before crashing. The Pangolin map viewer showed the map periodically flipping between forward and backward orientations, a known symptom of an incorrect `Tbc` matrix. A `Tbc` rotation error causes IMU preintegration to diverge, making visual and inertial estimates contradict each other and destabilizing the flight controller. Pure stereo mode (`STEREO`) was adopted instead, which requires no `Tbc` calibration and is not affected by IMU preintegration errors. The key enabler for pure stereo was ensuring sufficient visual features in the camera view: the camera image format was changed from `L_INT8` to `R8G8B8` (better contrast), and worlds with textured pillars were used. Pure stereo SLAM initializes on the ground from visual features alone, before takeoff, without any motion required. ArduPilot's EKF3 handles IMU fusion independently at 1000 Hz through its own Gazebo SITL plugin, making a separate ROS IMU pipeline unnecessary. The `Tbc` and IMU parameter sections are retained below for reference in case stereo-inertial mode is revisited.

---

### **File Location**

```
~/multi_uav_slam/ros2_ws/src/iris_vslam/config/orbslam3_config.yaml
```

The launch file (`vslam.launch.py`) passes the installed path to the ORB-SLAM3 `stereo` executable at runtime.

---

### **If parameters change in `models/iris_with_standoffs/model.sdf`**

| What changed in SDF | Files to update |
|---|---|
| `horizontal_fov` | Recompute `fx`, `fy`, `Camera.bf`, `LEFT.K`, `LEFT.P`, `RIGHT.K`, `RIGHT.P` in `orbslam3_config.yaml` |
| `width` or `height` | Update `Camera.width`, `Camera.height`, `cx`, `cy`, `LEFT.width`, `LEFT.height`, `RIGHT.width`, `RIGHT.height`, and recompute `fx`, `fy` in `orbslam3_config.yaml` |
| Camera `y` position (baseline) | Update `Camera.bf`, `RIGHT.P[3]` in `orbslam3_config.yaml` and update `camera_base_tf_broadcaster.py` positions |
| Camera `x`, `z` position or `pitch` (tilt) | Update `camera_base_tf_broadcaster.py` positions and tilt correction in `slam_bridge.py` (camera_optical→base_link transform)|
| `update_rate` (camera fps) | Update `Camera.fps` in `orbslam3_config.yaml` |

> **If stereo-inertial mode is revisited:** IMU SDF changes additionally require updating `IMU.NoiseGyro`, `IMU.NoiseAcc`, `IMU.GyroWalk`, `IMU.AccWalk`, `IMU.Frequency`, and recomputing the full `Tbc` matrix in `orbslam3_config.yaml`.

---

### **Section 1: Camera Intrinsics**

```yaml
Camera.type: "PinHole"
Camera.fx: 381.37
Camera.fy: 381.37
Camera.cx: 320.0
Camera.cy: 240.0
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 15.0
Camera.bf: 45.764
Camera.RGB: 1
ThDepth: 40.0
```

<strong><u>What they mean</u></strong>

| Parameter | Meaning |
|---|---|
| `Camera.type` | Camera projection model. `PinHole` is the standard pinhole model used by Gazebo's camera plugin. |
| `Camera.fx`, `Camera.fy` | Focal lengths in pixels (x and y). Equal for a symmetric lens. |
| `Camera.cx`, `Camera.cy` | Principal point — the optical center of the image in pixels. Ideally the image center. |
| `Camera.k1`, `Camera.k2`, `Camera.p1`, `Camera.p2` | Lens distortion coefficients (radial + tangential). All zero because Gazebo's pinhole model is ideal with no distortion. |
| `Camera.width`, `Camera.height` | Image resolution in pixels. |
| `Camera.fps` | Camera frame rate. Must match the actual observed publish rate of the camera topic. |
| `Camera.bf` | Stereo baseline × focal length (`b × fx`). Used internally by ORB-SLAM3 to compute depth from disparity. |
| `Camera.RGB` | Color channel order. Ignored for grayscale images — set to 1 as a placeholder. |
| `ThDepth` | Close/far depth threshold in units of baseline. Points beyond `ThDepth × baseline` are treated as far points and handled differently in the map. |

<strong><u>How they were decided</u></strong>

**`fx`, `fy`** — Computed from the HFOV specified in the SDF (`horizontal_fov: 1.3962634` rad) and the image width (640 px):

```
fx = (width / 2) / tan(HFOV / 2)
fx = 320 / tan(0.6981317)
fx = 320 / 0.8391
fx = 381.37
```

**`cx`, `cy`** — Set to the image center (`width/2 = 320`, `height/2 = 240`), which is correct for an ideal simulated camera with no optical axis offset.

**`Camera.fps`** — Set to `15.0` to match the actual observed camera topic rate (`~15 Hz` measured via `ros2 topic hz`). The SDF specifies `update_rate: 20` but Gazebo delivers approximately 15 Hz on this hardware (Ryzen 7 5700U) due to simulation load. ORB-SLAM3 uses this value for timing assumptions in the stereo synchronizer.

**Baseline and disparity selection rationale** — The stereo baseline was set to `0.12 m` (left camera at `y=+0.06`, right camera at `y=-0.06` in the SDF). This was chosen based on the practical stereo depth formula:

```
practical max depth = Camera.bf / min_disparity
                    = 45.764 / 3
                    ≈ 15 m
```

The original baseline of `0.06 m` gave only ~7.6 m practical depth, which was insufficient for 2–5 m flight with margin. The `0.12 m` baseline doubles depth accuracy while remaining within the stereo matching comfort zone for 1–10 m indoor scenes. Going wider risks large disparity at close range causing feature matching failure.

**`Camera.bf`** — The stereo baseline is `0.12 m` (left camera at `y=+0.06`, right camera at `y=-0.06` in the SDF). So:
```
Camera.bf = baseline × fx = 0.12 × 381.37 = 45.764
```

**`ThDepth`** — Set to `40.0` (lower than EuRoC's `60.0`) because our operating depth range is 2–5 m flight altitude. This ensures points at the edge of reliable stereo depth are correctly classified as far points.

---

### **Section 2: Tbc (Camera-to-IMU Transform) — Retained for Reference**

> **Not used in current stereo-only configuration.** This section is retained for reference in case stereo-inertial mode is revisited. In the current setup, the `Tbc` block is commented out in `orbslam3_config.yaml`.

```yaml
# Tbc: !!opencv-matrix
#    rows: 4
#    cols: 4
#    dt: f
#    data: [ 0.000000, -0.258820,  0.965926,  0.120000,
#            1.000000,  0.000000,  0.000000, -0.060000,
#            0.000000,  0.965926,  0.258820, -0.020000,
#            0.000000,  0.000000,  0.000000,  1.000000]
```

**What it means**

`Tbc` is the **rigid body transform from the left camera optical frame to the IMU frame**. It tells ORB-SLAM3 where the camera is relative to the IMU so it can fuse visual and inertial measurements correctly in stereo-inertial mode. This is the most critical calibration parameter for stereo-inertial mode — an incorrect `Tbc` will cause IMU preintegration to diverge.

> **Key Issue:** During stereo-inertial testing, the map was observed to periodically flip between forward and backward orientations. This is a known symptom of a `Tbc` rotation error — if the rotation part has a sign or convention error, the IMU integrates motion in the wrong direction, causing visual and inertial estimates to contradict each other. The `Tbc` derivation involves three chained transforms (see below), and any error in convention or sign at any step produces a plausible-looking but incorrect matrix.

**How it was computed**

`Tbc` was computed analytically from three transforms chained together:

```
Tbc = T_imu_base⁻¹  ×  T_base_cam_physical  ×  T_cam_physical_optical
```

**Step 1 — `T_base_imu`**: From the SDF, `imu_link` is at pose `(0, 0, 0, roll=180°, pitch=0, yaw=0)` relative to `base_link`. The 180° roll around X matches the ArduPilot NED convention (Z pointing down in the body frame).

**Step 2 — `T_base_cam_physical`**: From the SDF, the left camera sensor is at pose `(x=0.12, y=0.06, z=0.02, pitch=0.2618 rad)` relative to `base_link`. The `pitch=0.2618 rad` is the 15° downward tilt.

**Step 3 — `T_cam_physical_optical`**: From `camera_base_tf_broadcaster.py`, the optical frame correction is `euler(-π/2, 0, -π/2)`. This is the standard ROS convention to convert from the physical camera frame (X-forward, Z-up) to the OpenCV optical frame (X-right, Y-down, Z-forward), which ORB-SLAM3 expects.

The full computation was done in Python using NumPy rotation matrices. The translation part `(0.12, -0.06, -0.02)` is the position of the left camera optical origin expressed in the IMU frame.

---

### **Section 3: IMU Parameters — Retained for Reference**

> **Not used in current stereo-only configuration.** This section is retained for reference in case stereo-inertial mode is revisited. In the current setup, all IMU parameters are commented out in `orbslam3_config.yaml`.

```yaml
# IMU.NoiseGyro: 1.9393e-05
# IMU.NoiseAcc: 1.7e-03
# IMU.GyroWalk: 1.0e-06
# IMU.AccWalk: 1.0e-04
# IMU.Frequency: 200
```

<strong><u>What they mean</u></strong>

| Parameter | Meaning |
|---|---|
| `IMU.NoiseGyro` | Gyroscope white noise density (rad/s/√Hz). Models the random measurement noise on angular velocity. |
| `IMU.NoiseAcc` | Accelerometer white noise density (m/s²/√Hz). Models the random measurement noise on linear acceleration. |
| `IMU.GyroWalk` | Gyroscope bias random walk (rad/s²/√Hz). Models how the gyro bias drifts over time. |
| `IMU.AccWalk` | Accelerometer bias random walk (m/s³/√Hz). Models how the accel bias drifts over time. |
| `IMU.Frequency` | IMU update rate in Hz. Must match `update_rate` in the SDF sensor definition. |

<strong><u>How they were decided</u></strong>

All values are taken directly from the SDF `imu_sensor` noise model:

| SDF field | Value | Maps to |
|---|---|---|
| `angular_velocity stddev` | `1.9393e-05` | `IMU.NoiseGyro` |
| `linear_acceleration stddev` | `1.7e-03` | `IMU.NoiseAcc` |
| `angular_velocity bias_stddev` | `1.0e-06` | `IMU.GyroWalk` |
| `linear_acceleration bias_stddev` | `1.0e-04` | `IMU.AccWalk` |
| `update_rate` | `200` | `IMU.Frequency` |

> **Note:** ArduPilot receives IMU data at 1000 Hz directly from Gazebo through its SITL plugin — not through ROS. The ROS-facing IMU sensor (`/imu/data_raw`, `/imu/data`) was only needed to feed ORB-SLAM3's inertial pipeline and has been disabled. See `ekf3_vision_config.md` for details on how ArduPilot's EKF3 handles IMU fusion independently.

---

### **Section 4: Stereo Rectification**

```yaml
LEFT.height: 480
LEFT.width: 640
LEFT.D: [0.0, 0.0, 0.0, 0.0, 0.0]
LEFT.K: [381.37, 0.0, 320.0, 0.0, 381.37, 240.0, 0.0, 0.0, 1.0]
LEFT.R: [identity]
LEFT.Rf: [identity]
LEFT.P: [381.37, 0.0, 320.0, 0.0, ...]
RIGHT.P: [381.37, 0.0, 320.0, -45.764, ...]
```

<strong><u>What they mean</u></strong>

These parameters describe the raw (unrectified) camera geometry and the rectification transforms. ORB-SLAM3 uses them when `do_rectify=true` is passed to the executable to pre-rectify images before feature extraction.

| Parameter | Meaning |
|---|---|
| `LEFT/RIGHT.D` | Distortion coefficients of the raw camera (5-element vector for `[k1, k2, p1, p2, k3]`). |
| `LEFT/RIGHT.K` | Raw camera intrinsic matrix (3×3). |
| `LEFT/RIGHT.R` | Rectification rotation matrix (3×3). Rotates the raw camera so both cameras are co-planar after rectification. |
| `LEFT/RIGHT.Rf` | Same as `.R` but in float32 (`dt: f`) — required by some OpenCV functions internally. |
| `LEFT/RIGHT.P` | Projection matrix after rectification (3×4). For the left camera, the last column is `[0,0,0]`. For the right camera, the `(0,3)` element is `-Camera.bf = -45.764`, encoding the baseline. |

<strong><u>How they were decided</u></strong>

Since Gazebo's pinhole camera model is **ideal and already rectified** (both cameras share the same intrinsics, no distortion, parallel optical axes), rectification is a no-op:

- `D` = all zeros (no distortion)
- `K` = same intrinsics as `Camera.fx/fy/cx/cy`
- `R` = identity (no rotation needed — cameras are already co-planar)
- `LEFT.P` = `[K | 0]` (standard projection, no offset)
- `RIGHT.P` = `[K | -bf]` (right camera offset encodes the baseline)

> **In practice**, `do_rectify=false` is passed in the launch file since rectification is unnecessary for ideal Gazebo cameras. These parameters are included for completeness and in case rectification is ever enabled.

---

### **Section 5: ORB Feature Extraction**

```yaml
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

<strong><u>What they mean</u></strong>

| Parameter | Meaning |
|---|---|
| `nFeatures` | Maximum number of ORB features extracted per image frame. More features = more accurate tracking but higher CPU cost. |
| `scaleFactor` | Scale factor between pyramid levels. Each level is downscaled by this factor. |
| `nLevels` | Number of levels in the image scale pyramid. More levels = better handling of scale changes. |
| `iniThFAST` | Initial FAST corner detection threshold. Higher = only strong corners are detected. |
| `minThFAST` | Fallback FAST threshold used when a grid cell has no corners at `iniThFAST`. |

<strong><u>How they were decided</u></strong>

- **`nFeatures: 1000`** — Reduced from the EuRoC default of 1200 for CPU performance on the Ryzen 7 5700U (no GPU). 1000 features is sufficient for 640×480 indoor scenes with rich visual content (textured pillars, poster images).
- **`scaleFactor`, `nLevels`, `iniThFAST`, `minThFAST`** — Left at EuRoC defaults, which are well-tuned general values for indoor environments at this resolution.

> **Visual features requirement:** ORB-SLAM3 requires sufficient texture in the camera view to detect keypoints. The simulation world (`straight_path_with_pillars.sdf`) was specifically designed with textured pillars (poster images) to ensure enough features are visible at 2.5 m flight altitude. The camera image format was also changed from `L_INT8` (grayscale, poor contrast) to `R8G8B8` (color, full contrast) in `model.sdf` to improve feature detection quality.

---

### **Section 6: Viewer**

```yaml
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

These are purely visual parameters for the Pangolin 3D viewer window — they control the size of rendered keyframe frustums, map points, and the initial camera viewpoint. They have no effect on SLAM accuracy and are copied from the EuRoC defaults.

> **Note:** The Pangolin viewer is disabled by default in `vslam.launch.py` (`bUseViewer=false` in `slam_node.cpp`). It can be temporarily enabled for debugging by setting `bUseViewer=true` and rebuilding `iris_vslam`. When enabled, it opens a Pangolin window showing live keypoints on the current frame and the 3D map. For normal operation, all visualization is done through RViz2: `slam_bridge.py` publishes `/slam/map_points` (PointCloud2), `/slam/path` (Path), and `/mavros/vision_pose/pose` (PoseStamped), all viewable in the `map` frame.