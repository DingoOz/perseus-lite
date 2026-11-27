"""AprilTag landmark localisation: tag_detector + landmark_localiser + landmark_navigator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("perseus_landmarks")
    use_sim_time = LaunchConfiguration("use_sim_time")
    landmarks_file = LaunchConfiguration("landmarks_file")
    params_file = LaunchConfiguration("landmarks_params_file")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "landmarks_file",
            default_value=PathJoinSubstitution([share, "config", "landmarks.yaml"]),
            description="Landmark map: tags, destinations, search points (site frame)",
        ),
        DeclareLaunchArgument(
            "landmarks_params_file",
            default_value=PathJoinSubstitution(
                [share, "config", "landmarks_params.yaml"]
            ),
            description="Node parameters",
        ),
        DeclareLaunchArgument(
            "search_on_start",
            default_value="false",
            description="Turn in place at startup until a known tag is seen, then calibrate (MOVES THE ROBOT)",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="",
            description="Override tag_detector's image_topic (empty: from the params file)",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="",
            description="Override tag_detector's camera_info_topic (empty: from the params file)",
        ),
        DeclareLaunchArgument(
            "landmarks_debug",
            default_value="false",
            description="tag_detector publishes /landmarks/debug/image",
        ),
    ]
    # Typed ParameterValues: a bare LaunchConfiguration is silently ignored (ERRORS.md).
    common = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "landmarks_file": ParameterValue(landmarks_file, value_type=str),
    }

    def detector(context):
        overrides = {
            "publish_debug_image": ParameterValue(
                LaunchConfiguration("landmarks_debug"), value_type=bool
            )
        }
        for key in ("image_topic", "camera_info_topic"):
            value = LaunchConfiguration(key).perform(context)
            if value:
                overrides[key] = value
        return [
            Node(
                package="perseus_landmarks",
                executable="tag_detector",
                name="tag_detector",
                output="screen",
                parameters=[params_file, common, overrides],
            )
        ]

    nodes = [
        OpaqueFunction(function=detector),
        Node(
            package="perseus_landmarks",
            executable="landmark_localiser",
            name="landmark_localiser",
            output="screen",
            parameters=[params_file, common],
        ),
        Node(
            package="perseus_landmarks",
            executable="landmark_navigator.py",
            name="landmark_navigator",
            output="screen",
            parameters=[
                params_file,
                common,
                {
                    "search_on_start": ParameterValue(
                        LaunchConfiguration("search_on_start"), value_type=bool
                    )
                },
            ],
        ),
    ]
    return LaunchDescription(arguments + nodes)
