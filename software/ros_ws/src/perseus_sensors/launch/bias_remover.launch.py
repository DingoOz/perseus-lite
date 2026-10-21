"""IMU gyro bias estimator + remover in one component container.

/imu/data -> imu_bias_estimator -> /imu/gyro_bias
/imu/data + /imu/gyro_bias -> imu_bias_remover -> /imu/data_corrected

Parameters (topics, stationary detection) live in config/imu_bias.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("perseus_sensors"), "config", "imu_bias.yaml"
    )

    config_file = LaunchConfiguration("config_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # mt is nice for multiple callbacks
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
            ),
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (/clock) time",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Parameter file for imu_bias_estimator / imu_bias_remover",
            ),
            container,
        ]
    )
