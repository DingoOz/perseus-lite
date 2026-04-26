import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perseus_vision_dir = get_package_share_directory("perseus_vision")
    config_file = os.path.join(
        perseus_vision_dir, "config", "cube_color_detector.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulated time",
            ),
            Node(
                package="perseus_vision",
                executable="cube_color_detector_node",
                name="cube_color_detector",
                parameters=[
                    config_file,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                output="screen",
            ),
        ]
    )
