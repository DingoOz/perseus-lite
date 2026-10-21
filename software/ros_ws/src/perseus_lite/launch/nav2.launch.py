import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    Perseus Lite: hardware bringup + Nav2 localising on a saved map (AMCL).

    Use perseus_lite_slam_and_nav2.launch.py instead to build a map while
    navigating; never run both (each publishes map -> odom).
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    map_yaml_file = LaunchConfiguration("map")
    nav_params_file = LaunchConfiguration("nav_params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

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
            "map",
            default_value="map.yaml",
            description="Full path to the saved map yaml (e.g. from slam_toolbox's save_map)",
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
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn Nav2 nodes if they crash",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="Nav2 log level"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="True", description="Start RViz"
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perseus_lite"), "rviz", "nav2.rviz"]
            ),
            description="Full path to the RViz config file to use",
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
    nav2 = _include(
        "autonomy",
        "perseus_nav_bringup.launch.py",
        {
            "use_sim_time": use_sim_time,
            "params_file": nav_params_file,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "use_localization": "True",
            "map": map_yaml_file,
        },
    )

    rviz_env = {}
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        # rviz2 (Qt) can crash in libfontconfig if it shares ~/.cache with
        # the system's fontconfig (different, ABI-incompatible version).
        rviz_env["XDG_CACHE_HOME"] = os.path.join(conda_prefix, "var", "cache")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        additional_env=rviz_env,
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(arguments + [bringup, nav2, rviz_node])
