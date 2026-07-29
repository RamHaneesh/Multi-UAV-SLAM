# main class for creating the launch description.
from launch import LaunchDescription

# for launch arguements
from launch.actions import DeclareLaunchArgument

# getting the launch arguement at runtime
from launch.substitutions import LaunchConfiguration

# to create expression for file path
from launch.substitutions import PythonExpression

# for getting the path of shared directory and to join
from ament_index_python.packages import get_package_share_directory

# to run ROS node
from launch_ros.actions import Node

# to add delay
from launch.actions import TimerAction

# os
import os

# for conditional actions
from launch.conditions import IfCondition


def generate_launch_description():
    # paths
    pkg_path = get_package_share_directory('iris_transforms')

    # launch arguements

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    vslam_arg = DeclareLaunchArgument(
        "vslam",
        default_value="false",
        description="Launch map_odom_tf_broadcaster for VSLAM pipeline"
    )

    rviz2_config_arg = DeclareLaunchArgument(
        "rviz2_config",
        default_value = 'default_config',
        description = "config filename (without .rviz)"
    )

    # reading the launch arguements at run-time
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz2_config = LaunchConfiguration('rviz2_config')
    vslam = LaunchConfiguration('vslam')

    # ROS2 nodes running

    ### odom_tf_broadcaster
    odom_base_tf_broadcaster = Node(
        package = 'iris_transforms',
        executable = 'odom_base_tf_broadcaster',
        output = 'screen',
        parameters = [{
            'odom_topic': 'mavros/local_position/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': use_sim_time
        }]
    )

    ### camera_tf_broadcaster
    camera_base_tf_broadcaster = Node(
        package='iris_transforms',
        executable='camera_base_tf_broadcaster',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'left_camera_physical_frame': 'camera_left_link',
            'right_camera_physical_frame': 'camera_right_link',
            'left_camera_optical_frame': 'camera_left_optical_link',
            'right_camera_optical_frame': 'camera_right_optical_link',
            'use_sim_time': use_sim_time
        }]
    )

    ### map_odom_tf_broadcaster (only for VSLAM pipeline)
    map_odom_tf_broadcaster = Node(
        package='iris_transforms',
        executable='map_odom_tf_broadcaster',
        output='screen',
        condition=IfCondition(vslam),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    ### visualizing ekf3 path
    ekf3_path = Node(
        package='iris_transforms',
        executable='ekf3_path_publisher',
        name='ekf3_path_publisher',
        output='screen',
        condition=IfCondition(vslam),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            "-d",
            PythonExpression([
                "'", pkg_path, "/rviz2_config/",
                rviz2_config,
                ".rviz'"
            ])
        ]
    )


    # running RVIZ2 after delay to allow the /tfs to publish
    rviz2_delayed = TimerAction(
        period = 1.5,
        actions = [rviz2] 
    )

    return LaunchDescription([
        # launch arguements
        use_sim_time_arg,
        rviz2_config_arg,
        vslam_arg,

        # ROS2 nodes
        odom_base_tf_broadcaster,
        camera_base_tf_broadcaster,
        map_odom_tf_broadcaster,
        ekf3_path,
        rviz2_delayed
    ])