import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    SLAM + Nav2 on top of an already running robot (simulation by default).

    Combines online_async_launch.py (slam_toolbox, sole map->odom publisher)
    and perseus_nav_bringup.launch.py (Nav2 servers). The robot bringup —
    hardware (perseus_lite.launch.py) or simulation (perseus_sim.launch.py) —
    must already provide /scan_filtered, the EKF (odom->base_link) and
    twist_mux.
    """
    autonomy_dir = get_package_share_directory("autonomy")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_params_file = LaunchConfiguration("nav_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation/Gazebo clock",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the autonomy stack",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(
                autonomy_dir, "config", "slam_toolbox_params.yaml"
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
        DeclareLaunchArgument(
            "nav_params_file",
            default_value=os.path.join(
                autonomy_dir, "config", "perseus_nav_params.yaml"
            ),
            description="Full path to the ROS2 parameters file for Nav2",
        ),
    ]

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_dir, "launch", "perseus_nav_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav_params_file,
        }.items(),
    )

    return LaunchDescription(arguments + [slam_launch, nav_launch])
