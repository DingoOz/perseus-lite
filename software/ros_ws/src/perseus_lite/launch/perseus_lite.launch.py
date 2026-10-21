from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Perseus Lite hardware bringup.

    Starts the robot description, ros2_control (ST3215 servos), the RPLidar C1
    plus the self-occlusion scan filter (/scan -> /scan_filtered), the IMU, the
    EKF (sole publisher of odom -> base_link), twist_mux (every velocity source
    -> /cmd_vel) and rosbridge for the web UI. SLAM / Nav2 launches include
    this file rather than duplicating it.
    """
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    servo_max_rpm = LaunchConfiguration("servo_max_rpm")
    lidar_port = LaunchConfiguration("lidar_port")
    scan_min_range = LaunchConfiguration("scan_min_range")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces (overrides hardware_plugin)",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="perseus_lite_hardware/ST3215SystemHardware",
            choices=[
                "mock_components/GenericSystem",
                "gz_ros2_control/GazeboSimSystem",
                "perseus_lite_hardware/ST3215SystemHardware",
            ],
            description="The hardware plugin to use for ros2_control",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for ST3215 servos",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="1000000",
            description="Baud rate for ST3215 servos",
        ),
        DeclareLaunchArgument(
            "servo_max_rpm",
            default_value="485.1",
            description="Wheel RPM represented by the servo's full-scale speed value (calibrated on the robot)",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for the RPLidar C1",
        ),
        DeclareLaunchArgument(
            "scan_min_range",
            default_value="0.5",
            description="Lidar readings closer than this (m) are the robot's own body and are dropped",
        ),
        DeclareLaunchArgument(
            "use_rosbridge",
            default_value="True",
            description="Start the rosbridge websocket (port 9090) for the web UI",
        ),
    ]

    # IMPORTED LAUNCH FILES
    def robot_state_publisher(context):
        performed_use_mock_hardware = IfCondition(use_mock_hardware).evaluate(context)
        final_hardware_plugin = (
            "mock_components/GenericSystem"
            if performed_use_mock_hardware
            else hardware_plugin
        )
        rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus_lite"),
                            "launch",
                            "robot_state_publisher.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hardware_plugin": final_hardware_plugin,
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "servo_max_rpm": servo_max_rpm,
            }.items(),
        )
        return [rsp_launch]

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "twist_mux.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    i2c_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_sensors"),
                        "launch",
                        "i2c_imu.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_rosbridge),
    )

    # NODES
    # RPLidar C1 (vendored Slamtec ros2-branch driver; RoboStack's 2.1.0 has
    # no C1 support). The lidar is mounted upside down; that is expressed in
    # the URDF, so the driver's own `inverted` flag stays false.
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
        parameters=[
            {"min_range": ParameterValue(scan_min_range, value_type=float)},
            use_sim_time_param,
        ],
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("perseus_lite"), "config", "ekf.yaml"]
            ),
            use_sim_time_param,
        ],
    )

    launch_files = [
        OpaqueFunction(function=robot_state_publisher),
        controllers_launch,
        twist_mux_launch,
        rplidar_node,
        scan_range_filter,
        i2c_imu_launch,
        ekf_node,
        rosbridge_launch,
    ]

    return LaunchDescription(arguments + launch_files)
