import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    SLAM + Nav2 on top of an already running robot (simulation by default).

    Combines online_async_launch.py (slam_toolbox, sole map->odom publisher)
    and perseus_nav_bringup.launch.py (Nav2 servers). The robot bringup —
    hardware (perseus_lite.launch.py) or simulation (perseus_sim.launch.py) —
    must already provide /scan_filtered, the EKF (odom->base_link) and
    twist_mux.

    terrain:=true adds the depth-camera terrain mapper (perseus_terrain) and
    the terrain-aware Nav2 overlay (config/terrain_nav_params.yaml). The mapper
    is started before Nav2 so its clear service exists when the terrain
    behaviour tree loads. monitoring:=true (default) adds the passive topic
    health and mobility monitors (perseus_monitoring).
    """
    autonomy_dir = get_package_share_directory("autonomy")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_params_file = LaunchConfiguration("nav_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    terrain = LaunchConfiguration("terrain")
    monitoring = LaunchConfiguration("monitoring")
    landmarks = LaunchConfiguration("landmarks")

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
        DeclareLaunchArgument(
            "terrain",
            default_value="false",
            description="Depth-camera terrain mapping + terrain-aware Nav2 overlay",
        ),
        DeclareLaunchArgument(
            "terrain_debug",
            default_value="false",
            description="terrain_mapper publishes /terrain/debug/* layers and its classified points",
        ),
        DeclareLaunchArgument(
            "monitoring",
            default_value="true",
            description="Topic health + mobility (stall/slip) monitors",
        ),
        DeclareLaunchArgument(
            "landmarks",
            default_value="false",
            description="AprilTag localisation (site -> map), tag search and destinations",
        ),
        DeclareLaunchArgument(
            "landmarks_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perseus_landmarks"), "config", "landmarks_sim.yaml"]
            ),
            description="Landmark map (default: the Gazebo terrain_test world's tags)",
        ),
        DeclareLaunchArgument(
            "search_on_start",
            default_value="false",
            description="With landmarks: turn in place at startup until a known tag is seen, then calibrate",
        ),
    ]

    nav_overlay = PythonExpression(
        [
            "'",
            os.path.join(autonomy_dir, "config", "terrain_nav_params.yaml"),
            "' if '",
            terrain,
            "'.lower() == 'true' else '",
            os.path.join(autonomy_dir, "config", "nav_overlay_none.yaml"),
            "'",
        ]
    )
    # Scoped groups: an unscoped include leaks its launch arguments into every
    # later include.
    terrain_launch = GroupAction(
        scoped=True,
        condition=IfCondition(terrain),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus_terrain"),
                            "launch",
                            "terrain.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_debug": LaunchConfiguration("terrain_debug"),
                }.items(),
            )
        ],
    )
    monitoring_launch = GroupAction(
        scoped=True,
        condition=IfCondition(monitoring),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus_monitoring"),
                            "launch",
                            "monitoring.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "terrain": terrain,
                    "landmarks": landmarks,
                }.items(),
            )
        ],
    )

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
            "overlay_params_file": nav_overlay,
        }.items(),
    )

    landmarks_launch = GroupAction(
        scoped=True,
        condition=IfCondition(landmarks),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus_landmarks"),
                            "launch",
                            "landmarks.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "landmarks_file": LaunchConfiguration("landmarks_file"),
                    "search_on_start": LaunchConfiguration("search_on_start"),
                    # Gazebo's rgbd camera has one camera_info for image and depth.
                    "camera_info_topic": "/oak/stereo/camera_info",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        arguments
        + [terrain_launch, monitoring_launch, landmarks_launch, slam_launch, nav_launch]
    )
