from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])]
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    """
    Perseus Lite: hardware bringup + slam_toolbox mapping + Nav2.

    perseus_lite.launch.py provides the servos, lidar (+ /scan_filtered), IMU,
    EKF (odom -> base_link) and twist_mux; slam_toolbox provides map -> odom;
    Nav2's output enters twist_mux at the lowest priority, so any teleop
    input overrides it.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_params_file = LaunchConfiguration("nav_params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces (overrides hardware_plugin)",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="perseus_lite_hardware/ST3215SystemHardware",
            description="The hardware plugin to use for ros2_control",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for ST3215 servos",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="1000000",
            description="Baud rate for ST3215 servos",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "slam_toolbox_params.yaml"]
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
        DeclareLaunchArgument(
            "nav_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "perseus_nav_params.yaml"]
            ),
            description="Full path to the ROS2 parameters file for Nav2",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup SLAM and the nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn Nav2 nodes if they crash",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="Nav2 log level"
        ),
    ]

    bringup = _include(
        "perseus_lite",
        "perseus_lite.launch.py",
        {
            "use_sim_time": use_sim_time,
            "use_mock_hardware": use_mock_hardware,
            "hardware_plugin": hardware_plugin,
            "serial_port": serial_port,
            "baud_rate": baud_rate,
        },
    )
    slam = _include(
        "autonomy",
        "online_async_launch.py",
        {
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
            "autostart": autostart,
        },
    )
    nav2 = _include(
        "autonomy",
        "perseus_nav_bringup.launch.py",
        {
            "use_sim_time": use_sim_time,
            "params_file": nav_params_file,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "log_level": log_level,
        },
    )

    return LaunchDescription(arguments + [bringup, slam, nav2])
