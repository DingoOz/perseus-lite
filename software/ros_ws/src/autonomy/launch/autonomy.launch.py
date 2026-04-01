import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the complete autonomy stack including SLAM, navigation.

    This launch file combines:
    - online_async_launch.py: SLAM Toolbox for mapping/localization
    - perseus_nav_bringup.launch.py: Nav2 navigation stack
    """

    autonomy_dir = get_package_share_directory("autonomy")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation/Gazebo clock",
    )

    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu/corrected",
        description="IMU topic for robot_localization ekf (imu0)",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the autonomy stack",
    )
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        "ekf_config_file",
        default_value=os.path.join(autonomy_dir, "config", "ekf_config_fast_lio.yaml"),
        description="Full path to the ROS2 parameters file for EKF",
    )

    # TODO: Re-enable SLAM Toolbox (online_async_launch.py) and Nav2 Bringup
    # (perseus_nav_bringup.launch.py) when ready.

    # Crater Exit Node
    crater_exit_dir = get_package_share_directory("crater_exit")
    crater_exit_node = Node(
        package="crater_exit",
        executable="crater_exit_node",
        name="crater_exit_node",
        output="screen",
        parameters=[
            os.path.join(crater_exit_dir, "config", "crater_exit_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # EKF Node for sensor fusion and localization
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time, "imu0": LaunchConfiguration("imu_topic")},
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_ekf_config_file_cmd)

    # Include launch files and nodes
    ld.add_action(ekf_node)
    ld.add_action(crater_exit_node)

    return ld
