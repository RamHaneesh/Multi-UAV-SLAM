**[⬅️ Back to README](../README.md)**

## **ORB-SLAM3 Installation and ROS2 Wrapper Setup**

**System:** Ubuntu 22.04, ROS2 Humble

> **Prerequisites:** Complete [ORB-SLAM3 Dependencies](orb_slam3_dependencies.md) before proceeding.

---

### **Notation**

Replace `/home/<your-username>` with your actual home directory path throughout this document. You can find yours by running:

```bash
echo $HOME
```

---

### **Part 1 — Build ORB-SLAM3 Core**

We use `zang09/ORB-SLAM3-STEREO-FIXED` instead of the official `UZ-SLAMLab/ORB_SLAM3` because it fixes known stereo initialization bugs and is the tested base for the ROS2 wrapper.

```bash
cd ~
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will take 5-10 minutes. The build produces warnings about deprecated Eigen `AlignedBit`, unused parameters, and constructor reordering — all are harmless and expected.

**Verify:**
```bash
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so
ls ~/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

Both files must be present before proceeding.

---

### **Part 2 — Install Sophus**

Sophus is a Lie group math library bundled inside ORB-SLAM3. It must be installed system-wide so the ROS2 wrapper can find it.

```bash
cd ~/ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

**Verify:**
```bash
ls /usr/local/include/sophus/se3.hpp
```

---

### **Part 3 — Build the ROS2 Wrapper**

We use a separate ROS2 workspace (`~/orbslam3_ws`) to keep the third-party wrapper isolated from the main project workspace.

#### 3.1 Clone the wrapper

```bash
mkdir -p ~/orbslam3_ws/src
cd ~/orbslam3_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

#### 3.2 Fix CMakeLists.txt

Open `~/orbslam3_ws/src/orbslam3_ros2/CMakeLists.txt` and apply the following changes:

**Change 1 — Add ORB_SLAM3_ROOT_DIR and fix Python path** (top of file):
```cmake
# FROM:
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

# TO:
set(ORB_SLAM3_ROOT_DIR "/home/<your-username>/ORB_SLAM3")
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
```

> Replace `/home/<your-username>` with your actual home directory path.

**Change 2 — Add opencv_calib3d to stereo, rgbd, and stereo-inertial targets.**

After each `ament_target_dependencies(...)` call for `rgbd`, `stereo`, and `stereo-inertial`, add:
```cmake
target_link_libraries(<target_name>
  ${OpenCV_LIBS}
  opencv_calib3d
)
```

> This fixes the known `undefined reference to initUndistortRectifyMap` linker error caused by `opencv_calib3d` not being linked on Ubuntu 22.04 with OpenCV 4.5.

Also add `message_filters` to the `stereo-inertial` dependencies:
```cmake
# FROM:
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

# TO:
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
```

The final `CMakeLists.txt` should look like this:

```cmake
cmake_minimum_required(VERSION 3.5)
project(orbslam3)
# You should set the PYTHONPATH to your own python site-packages path
set(ORB_SLAM3_ROOT_DIR "/home/<your-username>/ORB_SLAM3")
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)
link_directories(
  include
)
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd
  ${OpenCV_LIBS}
  opencv_calib3d
)
add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(stereo
  ${OpenCV_LIBS}
  opencv_calib3d
)
add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(stereo-inertial
  ${OpenCV_LIBS}
  opencv_calib3d
)
install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})
ament_package()
```

#### 3.3 Fix FindORB_SLAM3.cmake

This is a **separate file** from `CMakeLists.txt`. It must also be fixed because it unconditionally overwrites `ORB_SLAM3_ROOT_DIR` when `find_package(ORB_SLAM3 REQUIRED)` is called, negating the value set in `CMakeLists.txt`.

Open `~/orbslam3_ws/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake` and fix the hardcoded path:

```cmake
# FROM:
set(ORB_SLAM3_ROOT_DIR "~/Install/ORB_SLAM/ORB_SLAM3")

# TO:
set(ORB_SLAM3_ROOT_DIR "/home/<your-username>/ORB_SLAM3")
```

> **Note:** `~` does not expand in CMake — the absolute path is required.

#### 3.4 Build

```bash
cd ~/orbslam3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select orbslam3
```

Expected output:
```
Starting >>> orbslam3
Finished <<< orbslam3 [~30s]
Summary: 1 package finished
```

---

### **Part 4 — Source the Wrapper**

Add to `~/.bashrc` so it is sourced in every terminal:

```bash
echo 'source ~/orbslam3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

### **Verify**

```bash
ls ~/orbslam3_ws/install/orbslam3/lib/orbslam3/
```

Expected output:
```
mono  rgbd  stereo  stereo-inertial
```

```bash
ros2 run orbslam3 stereo --help
```

Expected output:
```
Usage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]
```

Exiting with failure 1 here is expected — it printed usage because no arguments were passed. The node is working correctly.

---

### **Known Issues and Fixes Applied**

| Issue | Cause | Fix |
|---|---|---|
| `Could NOT find ORB_SLAM3` | `FindORB_SLAM3.cmake` had wrong hardcoded path | Set correct absolute path in both `CMakeLists.txt` and `FindORB_SLAM3.cmake` |
| `undefined reference to initUndistortRectifyMap` | `opencv_calib3d` not linked on Ubuntu 22.04 | Added `target_link_libraries(...opencv_calib3d)` for `stereo`, `rgbd`, `stereo-inertial` |
| `PYTHONPATH` pointing to Foxy/Python 3.8 | Wrapper originally written for ROS2 Foxy | Updated to `humble/python3.10` |
| `message_filters` missing from `stereo-inertial` | Not included in original wrapper | Added to `ament_target_dependencies` |
| `~` not expanding in CMake | CMake does not expand shell shortcuts | Used absolute path `/home/<your-username>/ORB_SLAM3` |