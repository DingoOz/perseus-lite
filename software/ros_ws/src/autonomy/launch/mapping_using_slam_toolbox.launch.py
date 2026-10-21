import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Hardware bringup + slam_toolbox mapping + RViz (no Nav2)."""
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock ros2_control hardware instead of the ST3215 servos",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "slam_toolbox_params.yaml"]
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
    ]

    # Robot bringup (servos, lidar + scan filter, IMU, EKF, twist_mux)
    perseus_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "perseus_lite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    # slam_toolbox is a lifecycle node: online_async_launch.py configures and
    # activates it. It is also the only map -> odom publisher.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("autonomy"), "launch", "online_async_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "rviz", "perseus_slam.rviz"]
    )
    rviz_env = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_SCREEN_SCALE_FACTORS": "1",
        "ROS_NAMESPACE": "/",
        "RMW_QOS_POLICY_HISTORY": "keep_last",
        "RMW_QOS_POLICY_DEPTH": "100",
    }
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        # rviz2 (Qt) can crash in libfontconfig if it shares ~/.cache with
        # the system's fontconfig (different, ABI-incompatible version).
        rviz_env["XDG_CACHE_HOME"] = os.path.join(conda_prefix, "var", "cache")
    rviz = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env=rviz_env,
    )

    nodes = [
        perseus_bringup,
        slam_launch,
        rviz,
    ]

    return LaunchDescription(declare_args + nodes)
