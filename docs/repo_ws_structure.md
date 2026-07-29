### Repository & Workspace Structure
```
multi_uav_slam/                               ← project root
|
└── ros2_ws/                                  ← main ROS2 workspace
    └── src/
        ├── ardupilot_sitl/         
        │   ├── ardusim/
        │   │   ├── arducopter                ← SITL binary
        │   │   └── parameters/
        │   │       ├── copter.parm           ← default (GPS mode)
        │   │       ├── copter_vslam.parm     ← VSLAM mode (GPS off, EKF3 vision)
        │   │       └── gazebo-iris.parm
        │   ├── config/
        │   │   ├── apm_config.yaml           ← MAVROS config (timesync disabled)
        │   │   └── apm_pluginlists.yaml      ← fake_gps denylisted
        │   ├── launch/
        │   │   └── sitl.launch.py            ← ArduPilot SITL + Mavproxy + MAVROS launcher
        │   ├── resource/ardupilot_sitl
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        │
        ├── iris_control/                     ← keyboard control + mission planner
        │   ├── config/
        │   │   ├── mission.json              ← default straight path mission
        │   │   ├── lcm.json                  ← loop closure mission (±4m perimeter, 2.8m altitude)
        │   │   ├── haltm.json                ← high altitude mission (±4m perimeter, 4.0m altitude)
        │   │   └── em.json                   ← extended mission (±7m perimeter, 2.8m altitude)
        │   ├── iris_control/
        │   │   ├── __init__.py
        │   │   ├── keyboard_control.py
        │   │   ├── mission_planner.py        ← supports -p m:=<mission_name> parameter
        │   │   └── utils/
        │   │       ├── __init__.py
        │   │       ├── base.py               ← MAVROS service clients, state callbacks
        │   │       └── key_listener.py
        │   ├── resource/iris_control
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        │
        ├── iris_description/                 ← UAV SDF model, worlds, ros_gz_bridge
        │   ├── config/
        │   │   └── ros_gz_bridge.yaml
        │   ├── gazebo_plugins/
        │   │   └── libArduPilotPlugin.so     ← ArduPilot Gazebo Harmonic plugin
        │   ├── launch/
        │   │   └── gazebo.launch.py
        │   ├── materials/
        │   │   ├── media/
        │   │   │   └── wall1-36.png          ← poster textures for pillars
        │   │   └── textures/
        │   │       ├── plank_walls.jpg
        │   │       ├── plaster_bricks.jpg
        │   │       └── runway.png
        │   ├── models/
        │   │   ├── iris_uav/
        │   │   │   ├── model.config
        │   │   │   └── model.sdf             
        │   │   └── iris_with_standoffs/
        │   │       ├── meshes/               ← iris.dae, props
        │   │       ├── model.config
        │   │       └── model.sdf             ← stereo cameras + IMU, R8G8B8 format, 0.12m baseline
        │   ├── worlds/
        │   │   ├── empty.sdf
        │   │   ├── indoor20x20.sdf                 ← loop closure + mapping experiments
        │   │   ├── straight_path.sdf
        │   │   ├── straight_path_with_markers.sdf
        │   │   ├── straight_path_with_pillars.sdf  ← ATE evaluation (40m corridor)
        │   │   ├── straight_path_best.sdf
        │   │   └── temp.sdf
        │   ├── resource/iris_description
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        │
        ├── iris_transforms/                      ← TF broadcasters
        │   ├── iris_transforms/
        │   │   ├── __init__.py
        │   │   ├── odom_base_tf_broadcaster.py   ← odom → base_link (EKF3, w<0 normalization applied)
        │   │   ├── camera_base_tf_broadcaster.py ← base_link → camera_*_optical_link (15° tilt + 
        |   |   |                                   OpenCV correction)
        │   │   ├── map_odom_tf_broadcaster.py    ← map → odom (differential: T_map_base[SLAM] × 
        |   |   |                                   T_odom_base[EKF3]⁻¹)
        │   │   ├── ekf3_path_publisher.py        ← subscribes to /mavros/local_position/pose,
        │   │   │                                   publishes /ekf3/path for RViz2 (only when 
        |   |   |                                   vslam:=true)
        │   ├── launch/
        │   │   └── transforms.launch.py          ← vslam:=true launches map_odom_tf_broadcaster
        │   │                                       and ekf3_path_publisher
        │   ├── rviz2_config/
        │   │   ├── default_config.rviz
        │   │   └── vslam_config.rviz             ← map frame, /slam/path, /slam/map_points
        │   ├── resource/iris_transforms
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        │
        ├── iris_vslam/                     ← ORB-SLAM3 ROS2 integration (pure stereo mode) 
        │   ├── config/
        │   │   └── orb_slam3_config.yaml   ← camera intrinsics, ORB params (IMU section commented out)
        │   ├── iris_vslam/
        │   │   ├── __init__.py
        │   │   └── slam_bridge.py          ← ORB-SLAM3 frame → ROS ENU frame conversion + 15° tilt
        │   │                                 correction, publishes /mavros/vision_pose/pose 
        │   │                                 (ArduPilot) and /slam/path (RViz2)
        │   ├── launch/
        │   │   └── vslam.launch.py         ← uses gazebo_voc.txt
        │   ├── src/
        │   │   └── slam_node.cpp           ← stereo tracking, publishes /orbslam3/pose
        │   │                                 and /slam/map_points (/slam/path moved to
        │   │                                 slam_bridge.py for frame consistency)
        │   ├── resource/iris_vslam
        │   ├── CMakeLists.txt
        │   └── package.xml
        │
        └── iris_evaluation/                ← ATE evaluation package 
            ├── iris_evaluation/
            │   ├── __init__.py
            │   └── ground_truth_publisher.py  ← subscribes to Gazebo /world/default/pose/info,
            │                                    filters for 'iris' model, publishes 
            |                                    /ground_truth/pose requires: 
            |                                    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
            │                                    see docs/ate_eval.md
            ├── launch/
            │   └── ate.launch.py           ← automated ATE pipeline: ground_truth_publisher →
            │                                 bag record → mission_planner → process_ate.py
            |
            ├── utils/
            │   ├── run_ate.py        ← orchestrator: manages subprocesses for the full ATE
            │   │                       pipeline, called by ate.launch.py
            │   └── process_ate.py    ← evo processing script: loads bag via rosbags.rosbag2.Reader,
            │                           computes ATE, saves ate_metrics.txt, ate_error.png,
            │                           ate_traj_3d.png, ate_traj_3d.html (interactive plotly)
            │                           can also be run standalone with --bag and --out args
            │                           must be run via ~/evo_env/bin/python3
            ├── eval/                       ← ATE run outputs (gitignored)
            │   └── <mission>_<timestamp>/  ← one folder per run
            │       ├── ate_bag/            ← ROS2 bag (/ground_truth/pose + /mavros/vision_pose/pose)
            │       ├── ate_metrics.txt     ← RMSE, mean, median, std, min, max
            │       ├── ate_error.png       ← APE over time plot
            │       └── ate_traj_3d.html    ← interactive rotatable 3D trajectory (plotly)
            ├── resource/iris_evaluation
            ├── package.xml
            ├── setup.cfg
            └── setup.py

----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------

~/evo_env/   ← Python virtual environment for evo trajectory evaluation
               isolated from system Python to avoid matplotlib conflict with ROS2
               activate with: source ~/evo_env/bin/activate  OR  evo_run (alias)
               contains: evo 1.36.3, matplotlib, PyQt6, rosbags, plotly
               see docs/ate_eval.md for installation steps

----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------

~/ORB_SLAM3/                        ← core ORB-SLAM3 library (zang09/ORB-SLAM3-STEREO-FIXED)
├── lib/libORB_SLAM3.so             ← linked by iris_vslam/slam_node.cpp
├── Vocabulary/
|   └── ORBvoc.txt                  ← original real-world vocabulary
|
└── src/
    ├── KeyFrameDatabase.cc         ← unmodified (original ORB-SLAM3)
    └── LoopClosing.cc              ← unmodified (original ORB-SLAM3)
                                      see docs/loop_closure.md for experimental modifications

~/DBoW2/                            ← DBoW2 training tool (vocabulary retraining only)
├── build/                          ← compiled training binary
├── demo/
│   ├── demo.cpp                    ← modified for Gazebo vocabulary training (NIMAGES=6976, k=10, 
|   |                                 L=6)
│   └── gazebo_voc.txt              ← trained vocabulary (experimental, not used in active config)
└── include/DBoW2/
    └── TemplatedVocabulary.h       ← saveToTextFile() method added (avoids slow OpenCV FileStorage)

> Note: The zang09/ORB_SLAM3_ROS2 ROS2 wrapper was evaluated but not used at runtime.
> iris_vslam uses the ORB-SLAM3 C++ API directly via slam_node.cpp.
```