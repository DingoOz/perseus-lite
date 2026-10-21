from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Map with a hand-held RPLidar C1 (no robot, no wheel odometry).

    odom -> base_link is a fixed identity, so slam_toolbox's scan matcher does
    all the motion estimation (walk slowly). base_link -> c1_lidar_frame is an
    identity too: hold the lidar upright.
    """
    lidar_port = LaunchConfiguration("lidar_port")
    scan_min_range = LaunchConfiguration("scan_min_range")

    arguments = [
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for the RPLidar C1",
        ),
        DeclareLaunchArgument(
            "scan_min_range",
            default_value="0.3",
            description="Drop readings closer than this (m), e.g. the operator's body",
        ),
    ]

    static_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handheld_odom_tf",
            arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handheld_lidar_tf",
            arguments=["--frame-id", "base_link", "--child-frame-id", "c1_lidar_frame"],
        ),
    ]

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 460800,
                "frame_id": "c1_lidar_frame",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
    )
    scan_range_filter = Node(
        package="perseus_lite",
        executable="scan_range_filter.py",
        name="scan_range_filter",
        output="screen",
        parameters=[{"min_range": ParameterValue(scan_min_range, value_type=float)}],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("autonomy"), "launch", "online_async_launch.py"]
                )
            ]
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        arguments + static_transforms + [rplidar_node, scan_range_filter, slam_launch]
    )
