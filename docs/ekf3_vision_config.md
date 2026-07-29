**[⬅️ Back to README](../README.md)**

## **EKF3 Vision Configuration**

This document explains the ArduPilot parameter changes made in `copter_vslam.parm` to enable GPS-denied flight using visual odometry from ORB-SLAM3 fed through MAVROS.

The file `copter_vslam.parm` is identical to `copter.parm` except for the VSLAM-specific block below.

---

### **Modified Parameters**

**Disable GPS**

```parm
GPS1_TYPE 0
```

Setting to `0` tells ArduPilot no GPS hardware exists, freeing EKF3 to use ExternalNav (vision) as the sole horizontal position source.

---

**Force EKF3**

```parm
AHRS_EKF_TYPE 3
EK3_ENABLE 1
EK2_ENABLE 0
```

EKF3 introduced the flexible `EK3_SRC` source-selection system that supports ExternalNav input. EKF2 does not have this. Disabling EKF2 prevents ArduPilot from running both filters in parallel and switching between them unpredictably.

---

**EKF3 Sensor Sources**

```parm
EK3_SRC1_POSXY 6
EK3_SRC1_VELXY 6
EK3_SRC1_POSZ 1     # Baro for Z position
EK3_SRC1_VELZ 0     # Let EKF3 derive Z velocity from baro internally
EK3_SRC1_YAW 6
```

`SRC1` is the primary sensor source for EKF3. The source values are:

| Value | Source |
|---|---|
| 0 | None |
| 1 | Barometer |
| 2 | RangeFinder |
| 3 | GPS |
| 6 | ExternalNav (vision/SLAM) |

| Parameter | Value | Reasoning |
|---|---|---|
| `EK3_SRC1_POSXY` | 6 | SLAM provides horizontal position (replaces GPS). |
| `EK3_SRC1_VELXY` | 6 | SLAM provides horizontal velocity (from successive pose estimates). Gives EKF3 two independent measurements — position and velocity — for faster convergence. Switch to `0` if horizontal oscillations are observed during tuning. |
| `EK3_SRC1_POSZ` | 1 | Barometer for altitude. This matches ArduPilot's default even when GPS is present — GPS vertical accuracy is poor (vertical DOP ~2x horizontal), so baro is always preferred for Z. |
| `EK3_SRC1_VELZ` | 0 | Stereo camera Z velocity is noisier than LiDAR — derived from triangulation of visual features, affected by camera tilt and pitch/roll motion. Baro handles Z position reliably (`POSZ 1`), and EKF3 derives Z velocity internally from baro. Note: official cartographer SLAM docs set this to `6` but that applies to LiDAR SLAM which has accurate Z estimates. |
| `EK3_SRC1_YAW` | 6 | SLAM provides heading/yaw (replaces compass+GPS fusion). |

---

**Skip GPS Pre-arm Checks**

```parm
ARMING_SKIPCHK 8
```

`ARMING_SKIPCHK 8` disables only the GPS-related pre-arm check (bit 3). Since pure stereo SLAM initializes on the ground from visual features before takeoff, EKF3 receives a valid position estimate and GUIDED mode arming works directly without requiring any workaround. All other safety checks remain active.

> **Note on RTL:** RTL is not used since it requires a GPS home position. The mission returns to the origin via planned waypoints and lands using `LAND` mode instead.

---

**Bypass EKF3 GPS Preflight Checks**

```parm
EK3_GPS_CHECK 0
```

Disables all GPS preflight checks inside EKF3 (satellite count, HDOP, speed error, etc.). Required since GPS is disabled and EKF3 would otherwise block arming waiting for GPS health.

---

**Magnetometer Calibration Mode**

```parm
EK3_MAG_CAL 3
```

Value `3` is the ArduPilot default for copters: heading fusion on the ground, 3-axis fusion after the first in-air yaw reset. This is the correct setting for indoor GPS-denied flight with visual yaw from SLAM.

---

**Enable Visual Odometry Input**

```parm
VISO_TYPE 1
```

Activates ArduPilot's visual odometry intake pipeline. Without this, ArduPilot silently ignores anything published on `/mavros/vision_pose/pose` even if EKF3 sources are correctly configured. When enabled, MAVROS forwards pose messages from `/mavros/vision_pose/pose` as `VISION_POSITION_ESTIMATE` MAVLink messages to ArduPilot.

---


### **Tuning Notes**

- If **horizontal position oscillates**: try `EK3_SRC1_VELXY 0` to let EKF3 derive velocity internally.
- If **altitude is unstable**: check baro is not being disturbed by motor wash in simulation (`SIM_BARO_RND 0` should already be set in `copter_vslam.parm`).
- If **yaw drifts**: verify ORB-SLAM3 is publishing orientation with a valid quaternion and that the frame conventions match (see `iris_vslam` package documentation).
- If **flight path has dips or oscillations**: verify the Gazebo IMU sensor `update_rate` is set to `1000` in `model.sdf`. Setting it lower (e.g. `200`) reduces EKF3 propagation rate and causes visible position dips between vision corrections.