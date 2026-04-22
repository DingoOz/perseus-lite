from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    battery_topic = LaunchConfiguration("battery_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="HTTP bind address for the dashboard",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8080",
                description="HTTP port for the dashboard",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="5.0",
                description="SSE push rate (Hz) to connected browsers",
            ),
            DeclareLaunchArgument(
                "battery_topic", default_value="/power_monitor/battery_state"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            Node(
                package="perseus_lite_web",
                executable="web_node",
                name="perseus_lite_web",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "publish_rate_hz": publish_rate_hz,
                        "battery_topic": battery_topic,
                        "odom_topic": odom_topic,
                        "imu_topic": imu_topic,
                        "joint_states_topic": joint_states_topic,
                        "scan_topic": scan_topic,
                    }
                ],
            ),
        ]
    )
