# main class for creating the launch description.
from launch import LaunchDescription

# for launch arguements & execute process
from launch.actions import DeclareLaunchArgument, ExecuteProcess

# getting the launch arguement at runtime
from launch.substitutions import LaunchConfiguration

# for getting the path of shared directory and to join
from ament_index_python.packages import get_package_share_directory

# os
import os

def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'm', default_value='mission',
        description='Mission file name (no .json extension)'
    )

    pkg_share = get_package_share_directory('iris_evaluation')
    script = os.path.join(pkg_share, 'utils/run_ate.py')

    orchestrator = ExecuteProcess(
        cmd=['python3', script, '--mission', LaunchConfiguration('m')],
        output='screen',
    )

    return LaunchDescription([mission_arg, 
                              orchestrator])