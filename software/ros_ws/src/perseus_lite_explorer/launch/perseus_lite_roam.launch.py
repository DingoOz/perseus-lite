"""
Perseus Lite autonomous roam / mapping.

Brings up the full Perseus Lite stack (hardware, SLAM, Nav2) plus the
frontier_explorer node, which selects waypoints to drive the robot into
unknown space and progressively builds the map. Obstacle avoidance is
handled by Nav2's costmaps and collision_monitor.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    explorer_dir = get_package_share_directory("perseus_lite_explorer")

    explorer_params_file = LaunchConfiguration("explorer_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("explorer_start_delay")

    declare_explorer_params = DeclareLaunchArgument(
        "explorer_params_file",
        default_value=os.path.join(explorer_dir, "config", "explorer_params.yaml"),
        description="Full path to the parameters file for frontier_explorer",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock",
    )
    declare_start_delay = DeclareLaunchArgument(
        "explorer_start_delay",
        default_value="15.0",
        description="Seconds to wait after the stack starts before the explorer node launches",
    )

    # Bring up perseus_lite + SLAM + Nav2 (existing launch file does the heavy lifting).
    perseus_lite_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite"),
                    "launch",
                    "perseus_lite_slam_and_nav2.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    explorer_node = Node(
        package="perseus_lite_explorer",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[explorer_params_file, {"use_sim_time": use_sim_time}],
    )

    # Delay the explorer so SLAM has time to publish a first map and Nav2 has
    # finished its lifecycle activation.
    delayed_explorer = TimerAction(period=15.0, actions=[explorer_node])

    return LaunchDescription(
        [
            declare_explorer_params,
            declare_use_sim_time,
            declare_start_delay,
            perseus_lite_stack,
            delayed_explorer,
        ]
    )
