"""Launch the I2C IMU driver (publishes sensor_msgs/Imu on /imu/data).

Parameters come from ``config/i2c_imu_config.yaml`` (or ``config_file``) and
any launch argument given a non-empty value overrides the file. Leaving an
argument at its empty default means "use the value from the config file" (or
the node's built-in default when ``use_config_file:=false``).

Examples::

    ros2 launch perseus_sensors i2c_imu.launch.py
    ros2 launch perseus_sensors i2c_imu.launch.py device_type:=mpu6050 \\
        device_address:=0x68 i2c_bus:=/dev/i2c-1
    ros2 launch perseus_sensors i2c_imu.launch.py config_file:=/path/to/imu.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch argument -> (parameter type, description). An empty value means "not
# overridden" so the config file stays the single source of truth.
OVERRIDABLE_ARGS = {
    "i2c_bus": (str, "I2C bus device file (config default: /dev/i2c-7)"),
    "device_type": (
        str,
        "IMU type: lsm6dsox, mpu6050, mpu9250, lsm6ds3 (config default: lsm6dsox)",
    ),
    "device_address": (
        int,
        "7-bit I2C address, decimal or 0x-prefixed hex (config default: 0x6A = 106)",
    ),
    "update_rate": (float, "Publish rate in Hz, (0, 1000] (config default: 100.0)"),
    "frame_id": (str, "Frame ID for IMU data (config default: imu_link)"),
    "required": (
        bool,
        "Exit if the IMU cannot be initialised (config default: false)",
    ),
    "retry_count": (
        int,
        "Initialisation attempts, >= 1 (config default: 3)",
    ),
}


def _to_bool(value):
    lowered = value.strip().lower()
    if lowered in ("true", "1", "yes", "on"):
        return True
    if lowered in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"expected a boolean, got '{value}'")


def _convert(name, kind, value):
    try:
        if kind is bool:
            return _to_bool(value)
        if kind is int:
            return int(value, 0)  # accepts 106 and 0x6A
        if kind is float:
            return float(value)
        return value
    except ValueError as e:
        raise ValueError(f"i2c_imu.launch.py: invalid value for '{name}': {e}") from e


def _resolve_config_file(config_file):
    # A bare file name is looked up in this package's config directory; anything
    # containing a path separator is used as given.
    if os.sep in config_file:
        return config_file
    return os.path.join(
        get_package_share_directory("perseus_sensors"), "config", config_file
    )


def _launch_setup(context):
    parameters = []

    if _to_bool(LaunchConfiguration("use_config_file").perform(context)):
        config_path = _resolve_config_file(
            LaunchConfiguration("config_file").perform(context)
        )
        if not os.path.isfile(config_path):
            raise FileNotFoundError(
                f"i2c_imu.launch.py: config file not found: {config_path}"
            )
        parameters.append(config_path)

    overrides = {
        "use_sim_time": _to_bool(LaunchConfiguration("use_sim_time").perform(context))
    }
    for name, (kind, _) in OVERRIDABLE_ARGS.items():
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = _convert(name, kind, value)
    parameters.append(overrides)

    return [
        Node(
            package="perseus_sensors",
            executable="i2c_imu_node",
            name="i2c_imu_node",
            parameters=parameters,
            output="screen",
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, (_, description) in OVERRIDABLE_ARGS.items()
    ]
    arguments += [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (/clock) time for message stamps",
        ),
        DeclareLaunchArgument(
            "use_config_file",
            default_value="true",
            description="Load parameters from config_file before applying overrides",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="i2c_imu_config.yaml",
            description="Parameter file: a name in perseus_sensors/config or a full path",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
