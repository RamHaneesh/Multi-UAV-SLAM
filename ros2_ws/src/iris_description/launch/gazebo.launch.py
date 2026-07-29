# main class for creating the launch description.
from launch import LaunchDescription

# for launch arguements
from launch.actions import DeclareLaunchArgument

# getting the launch arguement at runtime
from launch.substitutions import LaunchConfiguration

# for getting the path of shared directory and to join
from ament_index_python.packages import get_package_share_directory

# to launch other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# to create expression for file path
from launch.substitutions import PythonExpression

# to run ROS node
from launch_ros.actions import Node

# to set environmental variables
from launch.actions import SetEnvironmentVariable

# to add delay
from launch.actions import TimerAction

# to execute process to clear all previous gazebo instances using terminal before starting the simulation.
from launch.actions import ExecuteProcess

# os 
import os


def generate_launch_description():

    # launch arguements
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    world_name_arg = DeclareLaunchArgument(
        "world",
        default_value="empty",
        description="World name (without .sdf)"
    )


    # reading the launch arguements at run-time
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')


    # Executing process
    ### cleaning all previous gazebo instances/cache before running the simulation
    cleanup_gz = ExecuteProcess(
    cmd=['pkill', '-f', 'gz'],
    output='screen'
    )

    cleanup_gui_config = ExecuteProcess(
        cmd=['rm', '-rf', os.path.expanduser('~/.gz/gui/default.config')],
        output='screen'
    )

    cleanup_sim_config = ExecuteProcess(
        cmd=['rm', '-rf', os.path.expanduser('~/.gz/sim/gui.config')],
        output='screen'
    )

    cleanup_cache = ExecuteProcess(
        cmd=['rm', '-rf', os.path.expanduser('~/.gz/sim/cache')],
        output='screen'
    )

    cleanup_sequence = TimerAction(
    period=0.5,
    actions=[
        cleanup_gz,
        cleanup_gui_config,
        cleanup_sim_config,
        cleanup_cache,
    ]
    )

    # defining paths
    pkg_path = get_package_share_directory('iris_description')

    world_path = PythonExpression([
        "'",
        pkg_path,
        "/worlds/",
        world_name,
        ".sdf'"
    ])

    model_path = os.path.join(
        pkg_path,
        "models",
        "iris_uav",
        "model.sdf"
    )

    config_path = os.path.join(
        pkg_path,
        "config",
        "ros_gz_bridge.yaml"
    )

    # setting environmental variable
    ### setting resource path for gazebo to use models and materials
    set_resource_path = SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH",
        value = os.path.join(pkg_path, "models") + ":" + pkg_path
    )

    set_plugin_path = SetEnvironmentVariable(
        name = "GZ_SIM_SYSTEM_PLUGIN_PATH",
        value = os.path.join(
        pkg_path,
        "gazebo_plugins"
        )
    )

    # launching other files
    ### launch gazebo harmonic
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'),
                        "launch",
                        "gz_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "gz_args": PythonExpression(["'-r -v 4 ",world_path,"'"])
                    }.items()
             )
    
    gazebo_delayed = TimerAction(
        period = 0.5,
        actions = [gazebo]
    )
    
    # ROS2 nodes running

    ### spawning UAV
    spawn_uav = Node(
        package="ros_gz_sim",
        executable="create",
        output = 'screen',
        arguments=[
            "-name","iris",
            "-file",model_path,
            "-x","0",
            "-y","0",
            "-z","0.1"
        ]
    )

    ### delaying the spawning of the UAV
    spawn_uav_delayed = TimerAction(
        period = 5.0,
        actions=[spawn_uav]
    )

    ### ros-gz-bridge
    bridge = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        name = 'ros_gz_bridge',
        output = 'screen',
        parameters = [{
            "config_file":config_path,
            "use_sim_time": True
        }]
    )

    ### delay bridge
    bridge_delayed = TimerAction(
        period = 3.0,
        actions = [bridge]
    )

    ### imu republisher
    # imu_republisher = Node(
    #     package='iris_sensors',
    #     executable='imu_republisher',
    #     name='imu_republisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # imu_republisher_delayed = TimerAction(
    # period=4.0,
    # actions=[imu_republisher]
    # )

    return LaunchDescription([
        # Launch Arguements:
        use_sim_time_arg,
        world_name_arg,

        # Setting environmental variables
        set_resource_path,
        set_plugin_path,

        # execute process to cleanup
        # cleanup_sequence,
        cleanup_cache,
        cleanup_gz,

        # Launch file
        gazebo_delayed,

        # Running ROS nodes
        spawn_uav_delayed,
        bridge_delayed,
        # imu_republisher_delayed,
    ])