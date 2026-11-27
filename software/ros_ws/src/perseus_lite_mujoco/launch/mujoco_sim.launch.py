"""
Perseus Lite in MuJoCo: the same stack as the Gazebo simulation, a different simulator.

    ros2 launch perseus_lite_mujoco mujoco_sim.launch.py            # viewer + RViz
    ros2 launch perseus_lite_mujoco mujoco_sim.launch.py headless:=true

Then, as with Gazebo, start autonomy separately:
    ros2 launch autonomy autonomy.launch.py [terrain:=true] [landmarks:=true]

Differences from perseus_sim.launch.py (Gazebo):
- ros2_control runs mock_components/GenericSystem (command mirrored to state, like a
  servo tracking its setpoint); mujoco_sim reads the commands from /joint_states and
  drives MuJoCo's wheels.
- diff_drive_base_controller gets a MuJoCo-calibrated effective wheel_separation
  (config/mujoco_controllers.yaml).
- Worlds come from perseus_lite_simulation/worlds/*.sdf; only simple static geometry
  (boxes, cylinders, spheres, planes, local model includes) is converted, so
  terrain_test.sdf works; mesh/heightmap worlds (e.g. the arc world) do not.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('perseus_lite_mujoco')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    sim_time = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value='terrain_test.sdf',
            description='World from perseus_lite_simulation/worlds (simple static geometry)',
        ),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.1'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='No MuJoCo viewer and no RViz',
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='Render the OAK-D Lite depth and image (for terrain:= and landmarks:=)',
        ),
    ]

    # The ros2_control description on MOCK hardware: command mirrored to state.
    robot_description = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('perseus_lite'), 'urdf', 'perseus_lite.urdf.xacro']
            ),
            ' hardware_plugin:=mock_components/GenericSystem',
        ]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            sim_time,
        ],
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('perseus_lite'),
                    'config',
                    'perseus_lite_controllers.yaml',
                ]
            ),
            # MuJoCo-calibrated effective wheel separation (later file wins).
            PathJoinSubstitution([share, 'config', 'mujoco_controllers.yaml']),
            sim_time,
        ],
    )
    # The spawners, from the robot's own controllers launch (controller_manager above).
    spawners = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('perseus_lite'),
                            'launch',
                            'controllers.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'launch_controller_manager': 'false',
                }.items(),
            )
        ],
    )

    def simulator(context):
        world = LaunchConfiguration('world').perform(context)
        world_path = (
            world
            if os.path.isabs(world)
            else os.path.join(
                FindPackageShare('perseus_lite_simulation').perform(context),
                'worlds',
                world,
            )
        )
        gui = LaunchConfiguration('headless').perform(context).lower() != 'true'
        env = {}
        conda_prefix = os.environ.get('CONDA_PREFIX')
        if conda_prefix and gui:
            # Same shared-fontconfig-cache crash guard as the Gazebo/RViz launches.
            env['XDG_CACHE_HOME'] = os.path.join(conda_prefix, 'var', 'cache')
        return [
            Node(
                package='perseus_lite_mujoco',
                executable='mujoco_sim',
                name='mujoco_sim',
                output='screen',
                arguments=['--gui'] if gui else [],
                additional_env=env,
                parameters=[
                    PathJoinSubstitution([share, 'config', 'mujoco_sim.yaml']),
                    {
                        # The plain URDF (no ros2_control specifics matter here).
                        'robot_description': ParameterValue(
                            robot_description, value_type=str
                        ),
                        'world_file': world_path,
                        'initial_pose_x': ParameterValue(
                            LaunchConfiguration('initial_pose_x'), value_type=float
                        ),
                        'initial_pose_y': ParameterValue(
                            LaunchConfiguration('initial_pose_y'), value_type=float
                        ),
                        'initial_pose_z': ParameterValue(
                            LaunchConfiguration('initial_pose_z'), value_type=float
                        ),
                        'initial_pose_yaw': ParameterValue(
                            LaunchConfiguration('initial_pose_yaw'), value_type=float
                        ),
                        'camera': ParameterValue(
                            LaunchConfiguration('camera'), value_type=bool
                        ),
                    },
                ],
            )
        ]

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('perseus_lite'), 'config', 'ekf.yaml']
            ),
            # Open the wheel-odometry outlier gates (see config/mujoco_ekf.yaml).
            PathJoinSubstitution([share, 'config', 'mujoco_ekf.yaml']),
            sim_time,
        ],
    )
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('perseus_lite'), 'launch', 'twist_mux.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    scan_range_filter = Node(
        package='perseus_lite',
        executable='scan_range_filter.py',
        name='scan_range_filter',
        output='screen',
        parameters=[{'min_range': 0.5}, sim_time],
    )
    rviz_env = {'QT_QPA_PLATFORM': 'xcb'}
    if os.environ.get('CONDA_PREFIX'):
        rviz_env['XDG_CACHE_HOME'] = os.path.join(
            os.environ['CONDA_PREFIX'], 'var', 'cache'
        )
    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('perseus_lite_simulation'), 'rviz', 'view.rviz']
            ),
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
        output='screen',
        additional_env=rviz_env,
        condition=UnlessCondition(headless),
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=simulator),
            robot_state_publisher,
            controller_manager,
            spawners,
            TimerAction(period=2.0, actions=[ekf]),
            twist_mux,
            scan_range_filter,
            rviz,
        ]
    )
