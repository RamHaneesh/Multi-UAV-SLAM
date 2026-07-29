**[⬅️ Back to README](../README.md)**

## **Selecting the Visual SLAM Algorithm**

### **1. Why ORB-SLAM3?**

Several Visual SLAM algorithms were considered. The table below compares the main candidates against the requirements of this project.

**Requirements:**
- Stereo-inertial sensor fusion (stereo camera + IMU)
- GPS-denied indoor flight
- Real-time on CPU only (Ryzen 7 5700U, no GPU)
- ROS2 support
- Extensible to multi-UAV in the future

| Algorithm | Sensor Support | IMU Fusion | Multi-Map / Multi-Agent | ROS2 Support | CPU-only Viable | Notes |
|---|---|---|---|---|---|---|
| **ORB-SLAM3** | Mono / Stereo / RGB-D | ✅ Tightly coupled | ✅ Atlas multi-map system — built-in multi-agent foundation | ✅ (via wrapper) | ✅ | Chosen. Only system with native multi-map architecture suitable for future multi-UAV extension. |
| VINS-Fusion | Mono / Stereo | ✅ Tightly coupled | ❌ Single map only | ✅ (ROS1 primary, ROS2 ports exist) | ✅ | Strong accuracy but no multi-agent path. |
| RTAB-Map | Stereo / RGB-D / LiDAR | ✅ Loose coupling | ⚠️ Multi-session only, not multi-agent | ✅ Native | ✅ | Good for mapping; no true multi-agent SLAM. |
| OpenVINS | Mono / Stereo | ✅ Tightly coupled | ❌ Single map only | ✅ Native | ✅ | Excellent VIO accuracy; no mapping or loop closure. |
| Kimera | Stereo | ✅ Tightly coupled | ⚠️ Kimera-Multi exists but is research-grade | ⚠️ ROS1 primary | ⚠️ Computationally heavy | Kimera-Multi promising but immature for practical use. |
| SVO2 | Mono / Stereo | ⚠️ Loosely coupled | ❌ | ❌ (proprietary ROS pkg) | ✅ | Fast but no loop closure; not fully open source. |
| DSO / LDSO | Mono only | ❌ | ❌ | ❌ No native ROS2 | ✅ | Direct method; no stereo-inertial support. |

**Why ORB-SLAM3 wins:**

- It is the only algorithm that natively supports stereo-inertial mode with a tightly coupled IMU matching the exact sensor configuration available (stereo camera + IMU from Gazebo).
- Its **Atlas** system maintains multiple independent maps and supports merging them. This serves two purposes: (1) single UAV robustness, when tracking is lost, Atlas creates a new map and merges it back when a previously mapped area is revisited, enabling graceful recovery; (2) multi-UAV collaborative mapping, each UAV builds its own map and Atlas merges them into a globally consistent map when overlapping regions are detected. Atlas is confirmed present in the `zang09` fork. In the current single-UAV implementation, Atlas manages map creation and reset internally, multi-map merging for collaborative mapping is reserved for the multi-UAV extension.
- It performs loop closure and map reuse, which is essential for ATE evaluation in the `straight_path_with_markers` and `indoor20x20` worlds.
- It runs in real time on CPU at our camera resolution (640×480, 20 Hz), confirmed by the original paper on hardware comparable to ours.

**Multi-UAV foresight:** The centralized client-server architecture for collaborative SLAM where each agent runs lightweight VO onboard and offloads bundle adjustment, loop closure, and map merging to a ground server is well established in literature (Schmuck & Chli, ICRA 2017; CVI-SLAM, RA-L 2018; CCM-SLAM, J. Field Robotics 2019). ORB-SLAM3's Atlas multi-map system is the natural single-library realization of this pattern, making it the only algorithm choice that does not require building a separate collaborative layer from scratch for the multi-UAV phase.

---

### **2. Why `zang09/ORB_SLAM3_ROS2`?**

Once ORB-SLAM3 was selected as the algorithm, several ROS2 wrapper implementations were evaluated.

| Implementation | Stereo-Inertial | Ubuntu 22.04 / Humble | Native Install | TF / Launch Files | Active (2024–25) | Notes |
|---|---|---|---|---|---|---|
| **`zang09/ORB_SLAM3_ROS2`** | ✅ | ✅ Confirmed working | ✅ | ✅ | ✅ | **Chosen.** Paired with `zang09/ORB-SLAM3-STEREO-FIXED` core. Known `opencv_calib3d` linker issue has a simple CMakeLists fix. Originally tested on Foxy/20.04 but confirmed working on Humble/22.04. |
| `suchetanrs/ORB-SLAM3-ROS2-Docker` | ⚠️ RGB-D focus | ✅ | ❌ Docker-based by design | ✅ | ✅ | Docker adds unnecessary complexity for a native simulation stack. RGB-D default mode does not match our stereo-inertial sensor setup. |
| `Mechazo11/ros2_orb_slam3` | ❌ Mono only | ✅ Humble native, AMD Ryzen tested | ✅ | ❌ Explicitly excludes TF, launch files by design | ⚠️ Limited | Author states it is a "bare-bones" starting point. No stereo-inertial support — hard blocker for this project. |
| `gjcliff/ORB_SLAM3_ROS2` | ❌ Mono / mono-inertial only | ✅ | ✅ | ✅ | ⚠️ | Stereo support announced as future work (as of Dec 2024). Not usable yet. |
| Official `UZ-SLAMLab/ORB_SLAM3` ROS examples | ✅ (ROS1 only) | ❌ ROS1 Melodic only | ✅ | ❌ No ROS2 | N/A | The upstream repo has no ROS2 wrapper. All ROS2 wrappers above are community forks. |

**Core library pairing:** `zang09/ORB-SLAM3-STEREO-FIXED` is used instead of the official `UZ-SLAMLab/ORB_SLAM3` because it fixes known stereo initialization bugs and is the tested base for the ROS2 wrapper.

**Subscribes to (matches our existing topics exactly):**
- `/camera/left/image_raw`
- `/camera/right/image_raw`
- `/imu/data` 
  - *(stereo-inertial mode only — republished by `imu_republisher` in `iris_sensors` with correct covariances, derived from `/imu/data_raw` bridged via `ros_gz_bridge`. Currently disabled, see [[vslam_working.md](vslam_working.md)])*

**Publishes:**
- Pose output → consumed by `iris_vslam` bridge node → `/mavros/vision_pose/pose`