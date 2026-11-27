"""
OAK-D Lite depth for Perseus Lite (runs in the `camera` Pixi env).

    pixi run -e camera oak

Starts depthai_ros_driver's Camera component (node /oak) with
config/oak_d_lite.yaml. Publishes /oak/stereo/image_raw (16UC1, mm) and
/oak/stereo/camera_info in oak_right_camera_optical_frame.

This file is deliberately self-contained: the `camera` env has only ROS base
and the depthai driver, not this colcon workspace, so it is run straight from
the source tree and finds its config relative to itself. It does NOT start the
driver's own robot_state_publisher; the robot's URDF owns the camera frames.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile

# realpath: under --symlink-install the installed copy is a symlink to this file.
_HERE = os.path.dirname(os.path.realpath(__file__))
_DEFAULT_PARAMS = os.path.normpath(
    os.path.join(_HERE, "..", "config", "oak_d_lite.yaml")
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=_DEFAULT_PARAMS,
            description="depthai_ros_driver parameter file",
        ),
    ]

    container = ComposableNodeContainer(
        name="oak_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name="oak",
                parameters=[ParameterFile(params_file, allow_substs=True)],
            )
        ],
        output="screen",
    )

    return LaunchDescription(arguments + [container])
