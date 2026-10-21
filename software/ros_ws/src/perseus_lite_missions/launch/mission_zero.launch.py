# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Mission Zero -- Selene Base Rescue launch.

Composes the perseus_lite simulation and the mission orchestrator + checker
nodes. The orchestrator starts slam_toolbox and Nav2 with deliberately
broken parameters and restarts each with its answer key as the viewer fixes
the challenges. The intentional faults (the wrong values in the two YAML
files) are the heart of the script; the viewer fixes them mid-mission.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mission_file(*parts):
    return PathJoinSubstitution(
        [FindPackageShare('perseus_lite_missions'), 'missions', 'm00-cold-open']
        + list(parts)
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    mission_yaml = LaunchConfiguration('mission_yaml')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock',
        ),
        DeclareLaunchArgument(
            'gz_world',
            default_value='selene_base.sdf',
            description='Gazebo world file (overrides perseus_sim default)',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Rover spawn X (open area west of the habitat)',
        ),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.3'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=_mission_file('configs', 'slam_params.broken.yaml'),
            description=(
                'SLAM toolbox params (defaults to the broken Challenge 1 '
                'starting state)'
            ),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=_mission_file('configs', 'nav2_params.broken.yaml'),
            description=(
                'Nav2 params (defaults to the broken Challenge 2 starting state)'
            ),
        ),
        DeclareLaunchArgument(
            'mission_yaml',
            default_value=_mission_file('mission.yaml'),
            description='Mission descriptor consumed by the orchestrator',
        ),
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('perseus_lite_simulation'),
                        'launch',
                        'perseus_sim.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gz_world': LaunchConfiguration('gz_world'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
        }.items(),
    )

    # slam_toolbox and Nav2 are deliberately NOT started here: the
    # orchestrator owns each as its own process group, so it can restart
    # them with the reference params without pkill or orphaned nodes.
    # There is also no static map -> odom publisher: slam_toolbox is the
    # only map -> odom source (a static one would fight its transform).
    orchestrator = Node(
        package='perseus_lite_missions',
        executable='orchestrator_node',
        name='mission_orchestrator',
        output='screen',
        parameters=[
            {
                'mission_yaml': mission_yaml,
                'slam_params_file': slam_params_file,
                'nav2_params_file': nav2_params_file,
                # Delay Nav2 until Gazebo, controllers and SLAM have had time
                # to come up (mirrors the controllers delay in
                # perseus_sim.launch.py).
                'nav2_start_delay_s': 12.0,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    map_quality_checker = Node(
        package='perseus_lite_missions',
        executable='map_quality_checker_node',
        name='map_quality_checker',
        output='screen',
        parameters=[
            {
                'slam_params_file': slam_params_file,
                'min_resolution_inverse': 10.0,  # resolution <= 0.1 m
                'min_max_laser_range': 8.0,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    nav2_params_checker = Node(
        package='perseus_lite_missions',
        executable='nav2_params_checker_node',
        name='nav2_params_checker',
        output='screen',
        parameters=[
            {
                'nav2_params_file': nav2_params_file,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    goal_reached_checker = Node(
        package='perseus_lite_missions',
        executable='goal_reached_checker_node',
        name='goal_reached_checker',
        output='screen',
        parameters=[
            {
                'mission_yaml': mission_yaml,
                'tolerance_m': 0.5,
                'robot_frame': 'base_link',
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        declare_args
        + [
            sim_launch,
            orchestrator,
            map_quality_checker,
            nav2_params_checker,
            goal_reached_checker,
        ]
    )
