"""Start the topic health and mobility monitors (both passive: they never command the robot)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    terrain = LaunchConfiguration("terrain")
    share = FindPackageShare("perseus_monitoring")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "terrain",
            default_value="false",
            description="Also watch the OAK-D Lite and terrain mapper topics",
        ),
        DeclareLaunchArgument(
            "landmarks",
            default_value="false",
            description="Also watch the AprilTag detector and localiser topics",
        ),
    ]

    health_config = PathJoinSubstitution(
        [
            share,
            "config",
            PythonExpression(
                [
                    "'topic_health_terrain.yaml' if '",
                    terrain,
                    "'.lower() == 'true' else 'topic_health.yaml'",
                ]
            ),
        ]
    )
    sim_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    def topic_health(context):
        extra = {}
        if LaunchConfiguration("landmarks").perform(context).lower() == "true":
            # tag_detector publishes every processed image (empty or not), so its rate is
            # also the tag camera's liveness; the localiser's status is 2 Hz.
            extra = {
                "extra_topics": ["/landmarks/detections", "/landmarks/localisation"],
                "extra_expected_rates_hz": [5.0, 2.0],
            }
        return [
            Node(
                package="perseus_monitoring",
                executable="topic_health_monitor",
                name="topic_health_monitor",
                output="screen",
                parameters=[health_config, sim_time, extra],
            )
        ]

    mobility = Node(
        package="perseus_monitoring",
        executable="mobility_monitor",
        name="mobility_monitor",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "mobility_monitor.yaml"]),
            sim_time,
        ],
    )

    return LaunchDescription(
        arguments + [OpaqueFunction(function=topic_health), mobility]
    )
