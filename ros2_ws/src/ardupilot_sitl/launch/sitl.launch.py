# main class for creating the launch description.
from launch import LaunchDescription

# to execute in terminal
from launch.actions import ExecuteProcess

# to get package shared directory
from ament_index_python.packages import get_package_share_directory

# for launch arguements
from launch.actions import DeclareLaunchArgument

# getting the launch arguement at runtime
from launch.substitutions import LaunchConfiguration

# to add delay
from launch.actions import TimerAction

# to launch ROS2 nodes
from launch_ros.actions import Node

# time
import time

# getting PythonExpression to get value of launch arguements
from launch.substitutions import PythonExpression

# os
import os


def generate_launch_description():
    # launch arguements
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    vslam_arg = DeclareLaunchArgument(
        "vslam",
        default_value="false",
        description="Use VSLAM parameters (GPS-denied, EKF3 vision) instead of default"
    )

    # reading the launch arguements at run-time
    use_sim_time = LaunchConfiguration('use_sim_time')
    vslam = LaunchConfiguration('vslam')

    # variables
    MAVROS_IN_PORT = 14550
    MAVROS_OUT_PORT = 14555
    PROXY_ADD_PORT = 14230

    # paths
    pkg_path = get_package_share_directory('ardupilot_sitl')
    ardusim_dir_path = os.path.join(pkg_path, "ardusim")
    arducopter_bin_path = os.path.join(ardusim_dir_path, "arducopter")
    params_path = os.path.join(ardusim_dir_path, "parameters")

    params = PythonExpression([
    f'"{os.path.join(params_path, "copter_vslam.parm")},{os.path.join(params_path, "gazebo-iris.parm")}"',
    ' if "', vslam, '" == "true" else ',
    f'"{os.path.join(params_path, "copter.parm")},{os.path.join(params_path, "gazebo-iris.parm")}"'
    ])

    # mavros config paths (now in our package)
    mavros_plugins = os.path.join(pkg_path, 'config', 'apm_pluginlists.yaml')
    mavros_config  = os.path.join(pkg_path, 'config', 'apm_config.yaml')

    # creating a log folder in the ardusim_dir_path, to log the runs
    logs_dir_path = os.path.join(ardusim_dir_path, "logs")
    if not os.path.isdir(logs_dir_path):
        try:
            os.makedirs(logs_dir_path, exist_ok=True)
        except Exception:
            pass

    # Terminal Processes

    ### running ardupilot SITL
    sitl = ExecuteProcess(
        cmd = [arducopter_bin_path,
            "-w", # clear previously set parameters
            "-I0", # instance - id
            "--model", "JSON", # use JSON model for communication between Gazebo and SITL
            "--speedup", "1", # simulation speed set to 1
            "--defaults", params, # setting parameters
            "--synthetic-clock",
            "--start-time", str(int(time.time())),
        ],
        cwd = ardusim_dir_path, # to ensure that logs and other contents at ardusim_dir_path
        output = "screen"
    )


    ### runnning mavproxy with "screen"-terminal multiplexer for debugging
    mavproxy = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"""
            cd {ardusim_dir_path} &&
            screen -S proxy -d -m bash -c '
            mavproxy.py \
            --master=tcp:127.0.0.1:5760 \
            --out=udp:127.0.0.1:{MAVROS_IN_PORT} \
            --out=udp:127.0.0.1:{PROXY_ADD_PORT} \
            --streamrate -1 \
            --console
            '
            """
        ],
        output="screen",
        shell = False
    )

    ### set mavlink stream rates after mavros connects
    ### 20 Hz matches camera rate and is sufficient for VSLAM pipeline
    set_stream_rate = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mav',
            '--mavros-ns', '/mavros',
            '--wait-fcu', '30.0',
            'sys', 'rate', '--all', '20'
        ],
        output='screen'
    )

    # launching ROS2 nodes.

    ### launching MAVROS directly as a node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',        # namespace, not name
        output='screen',
        parameters=[
            mavros_plugins,
            mavros_config,
            {
                'fcu_url': f'udp://:{MAVROS_IN_PORT}@127.0.0.1:{MAVROS_OUT_PORT}',
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'use_sim_time': use_sim_time,
                'conn/timesync_rate': 0.0,
                'conn/heartbeat_rate': 1.0,
                'conn/timeout': 30.0,
            }
        ],
    )


    # adding delay

    ### we need to start mavros after mavproxy has started
    mavros_delayed = TimerAction(
        period = 2.0,
        actions = [mavros]
    )

    set_stream_rate_delayed = TimerAction(
        period=3.0,
        actions=[set_stream_rate]
    )


    return LaunchDescription([
        # launch arguements
        use_sim_time_arg,
        vslam_arg,

        # terminal process
        sitl,
        mavproxy,

        # delayed launch file
        mavros_delayed,
        set_stream_rate_delayed,
    ])