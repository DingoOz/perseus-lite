# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""perseus-lite launch profiles and developer tasks shown in the TUI."""

from perseus_lite_tui.registry import (
    BOOL,
    DRIVE,
    JOYSTICK,
    NAV2,
    Option,
    Profile,
    SLAM,
    STATE_PUBLISHER,
    Task,
)

_MOCK = Option('use_mock_hardware', 'Mock hardware', BOOL, 'False')

# Every profile that owns the drive goes through twist_mux, whose output is
# /cmd_vel (the diff_drive_base_controller input). Some launch files default
# cmd_vel_topic to /joy_vel, which would wire the controller straight to the
# gamepad and bypass the mux (keyboard/web/Nav2 ignored), so pin it.
_CMD_VEL = ('cmd_vel_topic:=/cmd_vel',)

# Resource sets (see registry.DRIVE etc.). The simulation also runs a
# controller_manager + robot_state_publisher on the same ROS domain, so it
# conflicts with hardware bringup exactly like a second hardware bringup would.
_ROBOT = (DRIVE, STATE_PUBLISHER)

# Isaac ROS container stack (Jetson; see software/docker/isaac-ros/). Tasks run
# with cwd=repo_root, so this repo-relative path resolves.
_ISAAC_COMPOSE = 'software/docker/isaac-ros/compose.yaml'

# Ordered as they appear in the Launch tab. Every launch_file is verified to
# exist on disk by test/test_config.py, so a rename upstream fails the suite.
PROFILES = (
    # --- Bringup -------------------------------------------------------------
    Profile(
        'bringup',
        'Robot bringup',
        'perseus_lite',
        'perseus_lite.launch.py',
        options=(_MOCK,),
        group='Bringup',
        resources=_ROBOT,
        fixed_args=_CMD_VEL,
    ),
    Profile(
        'controllers',
        'Controllers only',
        'perseus_lite',
        'controllers.launch.py',
        options=(_MOCK,),
        group='Bringup',
        resources=(DRIVE,),
        fixed_args=_CMD_VEL,
    ),
    Profile(
        'rsp',
        'Robot state publisher',
        'perseus_lite',
        'robot_state_publisher.launch.py',
        group='Bringup',
        resources=(STATE_PUBLISHER,),
    ),
    # --- Autonomy ------------------------------------------------------------
    Profile(
        'slam_nav2',
        'SLAM + Nav2',
        'perseus_lite',
        'perseus_lite_slam_and_nav2.launch.py',
        options=(_MOCK,),
        group='Autonomy',
        resources=_ROBOT + (SLAM, NAV2),
        fixed_args=_CMD_VEL,
    ),
    Profile(
        'nav2',
        'Nav2 only',
        'perseus_lite',
        'nav2.launch.py',
        group='Autonomy',
        resources=_ROBOT + (NAV2,),
        fixed_args=_CMD_VEL,
    ),
    Profile(
        'autonomy',
        'Autonomy stack',
        'autonomy',
        'autonomy.launch.py',
        group='Autonomy',
        resources=(SLAM, NAV2),
    ),
    Profile(
        'mapping',
        'SLAM Toolbox mapping',
        'autonomy',
        'mapping_using_slam_toolbox.launch.py',
        group='Autonomy',
        resources=_ROBOT + (SLAM,),
        fixed_args=_CMD_VEL,
    ),
    Profile(
        'handheld_mapping',
        'Handheld mapping',
        'autonomy',
        'handheld_mapping.launch.py',
        group='Autonomy',
        resources=(SLAM,),
    ),
    # --- Teleop --------------------------------------------------------------
    Profile(
        'input_controller',
        'Input router',
        'perseus_input',
        'controller.launch.py',
        group='Teleop',
        resources=(JOYSTICK,),
    ),
    Profile(
        'xbox',
        'Xbox controller',
        'input_devices',
        'xbox_controller.launch.py',
        group='Teleop',
        resources=(JOYSTICK,),
    ),
    Profile(
        'teleop_diagnostics',
        'Teleop diagnostics (TUI)',
        'teleop_diagnostics',
        'teleop_diagnostics.launch.py',
        group='Teleop',
        disabled=True,
        note='Nested full-screen TUI; run in a separate terminal, not as a job.',
    ),
    # --- Simulation (linux-64 only; needs the simulation Pixi env) -----------
    Profile(
        'sim',
        'Gazebo simulation',
        'perseus_lite_simulation',
        'perseus_sim.launch.py',
        env='simulation',
        group='Simulation',
        linux64_only=True,
        note='First launch may trigger a large simulation-env install.',
        resources=_ROBOT,
    ),
    Profile(
        'mission_zero',
        'Mission Zero (Selene)',
        'perseus_lite_missions',
        'mission_zero.launch.py',
        env='simulation',
        group='Simulation',
        linux64_only=True,
        resources=_ROBOT + (SLAM, NAV2),
    ),
)

# Developer tasks. Each argv is a complete pixi invocation; the Pixi task itself
# carries the right cwd and package-skip flags.
TASKS = (
    Task('build', 'Build (default env)', ('pixi', 'run', '-e', 'default', 'build')),
    Task(
        'build_test',
        'Build with tests',
        ('pixi', 'run', '-e', 'default', 'build-test'),
    ),
    Task('test', 'Run unit tests', ('pixi', 'run', '-e', 'default', 'test')),
    Task('clean', 'Clean workspace', ('pixi', 'run', '-e', 'default', 'clean')),
    Task(
        'sim_build',
        'Build (simulation env)',
        ('pixi', 'run', '-e', 'simulation', 'build'),
        group='Build',
        note='linux-64 only.',
    ),
    Task('fmt', 'Format (treefmt)', ('pixi', 'run', '-e', 'format', 'fmt')),
    # --- Isaac ROS (docker; Jetson only) -------------------------------------
    Task(
        'isaac_build',
        'Isaac image build (docker)',
        ('docker', 'compose', '-f', _ISAAC_COMPOSE, 'build'),
        group='Isaac',
        note='Jetson/GPU only; `docker login nvcr.io` may be required.',
    ),
    Task(
        'isaac_up',
        'Isaac ROS up (docker)',
        ('docker', 'compose', '-f', _ISAAC_COMPOSE, 'up', '-d'),
        group='Isaac',
    ),
    Task(
        'isaac_down',
        'Isaac ROS down (docker)',
        ('docker', 'compose', '-f', _ISAAC_COMPOSE, 'down'),
        group='Isaac',
    ),
    Task(
        'isaac_logs',
        'Isaac ROS logs (docker)',
        ('docker', 'compose', '-f', _ISAAC_COMPOSE, 'logs', '-f', '--tail=100'),
        group='Isaac',
    ),
    Task(
        'isaac_status',
        'Isaac ROS status (docker)',
        ('docker', 'compose', '-f', _ISAAC_COMPOSE, 'ps'),
        group='Isaac',
    ),
)
