from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_lite"), "config", "aruco_detector.yaml"]
    )

    aruco_detector_node = Node(
        package="perseus_vision",
        executable="aruco_detector_node",
        name="aruco_detector",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulated time",
            ),
            aruco_detector_node,
        ]
    )
