"""
Launch the Perseus Lite on-robot map screen.

Renders a top-down RViz-like view of the SLAM /map plus robot pose, /plan,
/goal_pose and /scan_filtered overlays at 1024x600 via Qt EGLFS on the
DisplayPort screen mounted on the robot.

By default the binary runs under Qt's EGLFS KMS platform (no X server). For
desktop development override QT_QPA_PLATFORM, e.g.:

    QT_QPA_PLATFORM=xcb ros2 launch perseus_lite_screen perseus_lite_screen.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("perseus_lite_screen")

    params_file = LaunchConfiguration("params_file")
    qpa_platform = LaunchConfiguration("qpa_platform")
    eglfs_integration = LaunchConfiguration("eglfs_integration")
    eglfs_kms_config = LaunchConfiguration("eglfs_kms_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declarations = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "screen_params.yaml"),
            description="Parameter file for perseus_lite_screen_node",
        ),
        DeclareLaunchArgument(
            "qpa_platform",
            default_value="eglfs",
            description="Qt platform plugin (eglfs for kiosk, xcb for desktop dev)",
        ),
        DeclareLaunchArgument(
            "eglfs_integration",
            default_value="eglfs_kms",
            description="EGLFS backend (eglfs_kms for DRM/KMS DisplayPort)",
        ),
        DeclareLaunchArgument(
            "eglfs_kms_config",
            default_value=os.path.join(pkg_share, "config", "eglfs_kms.json"),
            description="Path to Qt EGLFS KMS config (selects DRM device + connector)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation clock",
        ),
    ]

    # Setting env vars via SetEnvironmentVariable so they propagate to the Node
    # process. Each can be overridden by the user's shell environment because
    # SetEnvironmentVariable here writes into launch's child env, not the host.
    env_actions = [
        SetEnvironmentVariable("QT_QPA_PLATFORM", qpa_platform),
        SetEnvironmentVariable("QT_QPA_EGLFS_INTEGRATION", eglfs_integration),
        SetEnvironmentVariable("QT_QPA_EGLFS_KMS_CONFIG", eglfs_kms_config),
        # Hide the mouse cursor in kiosk mode.
        SetEnvironmentVariable("QT_QPA_EGLFS_HIDECURSOR", "1"),
    ]

    screen_node = Node(
        package="perseus_lite_screen",
        executable="perseus_lite_screen",
        name="perseus_lite_screen_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(declarations + env_actions + [screen_node])
