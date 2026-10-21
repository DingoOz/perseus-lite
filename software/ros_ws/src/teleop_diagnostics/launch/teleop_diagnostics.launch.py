# Copyright (c) 2025 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Launch the teleop diagnostics TUI with the lite wheel/mux config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the teleop diagnostics TUI."""
    config = LaunchConfiguration('config')
    arguments = [
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('teleop_diagnostics'),
                    'config',
                    'teleop_diagnostics.yaml',
                ]
            ),
            description='Parameter file (wheel geometry, mux priorities)',
        ),
    ]
    teleop_diagnostics = Node(
        package='teleop_diagnostics',
        executable='teleop_tui',
        name='teleop_diagnostics',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription(arguments + [teleop_diagnostics])
