"""Start terrain_mapper: OAK-D Lite depth in, /terrain/{local,global}_grid out."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("terrain_params_file")
    publish_debug = LaunchConfiguration("publish_debug")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "terrain_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perseus_terrain"), "config", "terrain_mapper.yaml"]
            ),
            description="terrain_mapper parameter file",
        ),
        DeclareLaunchArgument(
            "publish_debug",
            default_value="false",
            description="Publish /terrain/debug/* layers and the classified points",
        ),
    ]

    terrain_mapper = Node(
        package="perseus_terrain",
        executable="terrain_mapper",
        name="terrain_mapper",
        output="screen",
        # Typed ParameterValues: a bare LaunchConfiguration here is silently
        # ignored (see ERRORS.md).
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_debug": ParameterValue(publish_debug, value_type=bool),
            },
        ],
    )

    return LaunchDescription(arguments + [terrain_mapper])
