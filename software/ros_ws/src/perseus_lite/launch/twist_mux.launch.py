from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """twist_mux (all velocity sources -> /cmd_vel) plus the web UI stamper."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
    ]

    mux_config = PathJoinSubstitution(
        [FindPackageShare("perseus_lite"), "config", "twist_mux.yaml"]
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[mux_config, use_sim_time_param],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )
    # Foxglove / plain-Twist web clients publish /web_vel_unstamped
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="web_vel_stamper",
        output="screen",
        parameters=[use_sim_time_param],
        remappings=[
            ("cmd_vel_in", "/web_vel_unstamped"),
            ("cmd_vel_out", "/web_vel"),
        ],
    )

    return LaunchDescription(arguments + [twist_mux, twist_stamper])
