import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("perseus_lite_voice")
    config = os.path.join(pkg_share, "config", "voice_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="perseus_lite_voice",
                executable="perseus_lite_voice",
                name="perseus_lite_voice",
                parameters=[config],
                output="screen",
            ),
        ]
    )
