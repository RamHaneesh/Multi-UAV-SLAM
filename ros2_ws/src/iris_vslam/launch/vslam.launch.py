# main class for creating the launch description.
from launch import LaunchDescription

# for launch arguements
from launch.actions import DeclareLaunchArgument

# getting the launch arguement at runtime
from launch.substitutions import LaunchConfiguration


# to run ROS node
from launch_ros.actions import Node

# for getting the path of shared directory and to join
from ament_index_python.packages import get_package_share_directory

# os
import os

def generate_launch_description():

    # Package share directory
    pkg_iris_vslam = get_package_share_directory('iris_vslam')

    # Paths
    # vocab_path    = os.path.join(os.path.expanduser('~'), 'ORB_SLAM3', 'Vocabulary', 'gazebo_voc.txt')
    vocab_path    = os.path.join(os.path.expanduser('~'), 'ORB_SLAM3', 'Vocabulary', 'ORBvoc.txt')
    config_path = os.path.join(pkg_iris_vslam, 'config', 'orb_slam3_config.yaml')

    # launch arguements
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # reading the launch arguements at run-time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ROS2 nodes running

    # 1. slam_node (C++) — runs ORB-SLAM3, publishes /orbslam3/pose and /slam/map_points
    slam_node = Node(
        package='iris_vslam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        arguments=[vocab_path, config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. slam_bridge (Python) — converts ORB-SLAM3 pose to MAVROS frame, broadcasts map->odom TF
    slam_bridge = Node(
        package='iris_vslam',
        executable='slam_bridge.py',
        name='slam_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch Arguements:
        declare_use_sim_time,

        # Running ROS nodes
        slam_node,
        slam_bridge,
    ])