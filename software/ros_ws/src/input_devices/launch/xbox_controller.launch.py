# Copyright (c) 2024 The Remote Offworld Autonomous Robotics Team
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Launch joy_node + the Xbox controller node, publishing /joy_vel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    # ARGUMENTS
    turn_speed = LaunchConfiguration('turn_speed')
    drive_speed = LaunchConfiguration('drive_speed')
    high_speed_multiplier = LaunchConfiguration('high_speed_multiplier')

    arguments = [
        DeclareLaunchArgument(
            'turn_speed',
            default_value='0.5',
            description='Turn speed in rad/s',
        ),
        DeclareLaunchArgument(
            'drive_speed',
            default_value='0.25',
            description='Drive speed in m/s',
        ),
        DeclareLaunchArgument(
            'high_speed_multiplier',
            default_value='2.0',
            description='High speed multiplier for drive and turn speeds',
        ),
    ]

    # NODES
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='both',
        emulate_tty=True,
        parameters=[
            {
                'device': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }
        ],
    )
    controller_node = Node(
        package='input_devices',
        executable='xbox_controller',
        name='xbox_controller',
        output='both',
        emulate_tty=True,
        parameters=[
            {
                'use_stamped_msg': True,
                'translation_scale': drive_speed,
                'rotation_scale': turn_speed,
                'high_speed_multiplier': high_speed_multiplier,
            }
        ],
        remappings=[('/input_devices/cmd_vel', '/joy_vel')],
    )
    nodes = [joy_node, controller_node]

    return LaunchDescription(arguments + nodes)
