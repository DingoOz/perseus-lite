# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for command building in the registry."""

import shlex

from perseus_lite_tui.registry import (
    BOOL,
    build_launch_command,
    build_task_command,
    DRIVE,
    normalize_bool,
    Option,
    option_args,
    Profile,
    resource_conflict_message,
    STRING,
    Task,
)

_BRINGUP = Profile(
    'bringup',
    'Bringup',
    'perseus_lite',
    'perseus_lite.launch.py',
    options=(Option('use_mock_hardware', 'Mock', BOOL, 'False'),),
)
_SIM = Profile(
    'sim',
    'Sim',
    'perseus_lite_simulation',
    'perseus_sim.launch.py',
    env='simulation',
)


def test_normalize_bool():
    assert normalize_bool('true') == 'True'
    assert normalize_bool('1') == 'True'
    assert normalize_bool('False') == 'False'
    assert normalize_bool('nonsense') == 'False'


def test_default_launch_emits_no_args():
    argv = build_launch_command(_BRINGUP, {}, '/ws')
    assert argv[:7] == ['pixi', 'run', '-e', 'default', '--', 'bash', '-c']
    inner = argv[-1]
    assert inner.endswith('ros2 launch perseus_lite perseus_lite.launch.py')
    assert '/ws/install/setup.bash' in inner


def test_changed_option_is_emitted():
    argv = build_launch_command(_BRINGUP, {'use_mock_hardware': 'true'}, '/ws')
    assert argv[-1].endswith('use_mock_hardware:=True')


def test_option_set_to_default_is_not_emitted():
    assert option_args(_BRINGUP, {'use_mock_hardware': 'False'}) == []


def test_env_routing_selects_simulation():
    argv = build_launch_command(_SIM, {}, '/ws')
    assert argv[:4] == ['pixi', 'run', '-e', 'simulation']


def test_build_task_command():
    task = Task('build', 'Build', ('pixi', 'run', '-e', 'default', 'build'))
    assert build_task_command(task) == ['pixi', 'run', '-e', 'default', 'build']


def test_option_values_are_shell_quoted():
    hostile = Profile(
        'x',
        'X',
        'perseus_lite',
        'perseus_lite.launch.py',
        options=(Option('map', 'Map', STRING, ''),),
    )
    argv = build_launch_command(hostile, {'map': 'a; touch /tmp/pwned'}, '/ws')
    inner = argv[-1]
    assert "'map:=a; touch /tmp/pwned'" in inner
    # The shell sees exactly one argument for the option (no injection).
    words = shlex.split(inner.split('&&', 1)[1])
    assert words[-1] == 'map:=a; touch /tmp/pwned'
    assert 'touch' not in words


def test_fixed_args_are_always_emitted():
    prof = Profile(
        'p',
        'P',
        'perseus_lite',
        'perseus_lite_slam_and_nav2.launch.py',
        fixed_args=('cmd_vel_topic:=/cmd_vel',),
    )
    inner = build_launch_command(prof, {}, '/ws')[-1]
    assert inner.endswith('slam_and_nav2.launch.py cmd_vel_topic:=/cmd_vel')


def test_resource_conflict_message_names_holder_and_resource():
    msg = resource_conflict_message('SLAM + Nav2', {'Robot bringup': [DRIVE]})
    assert 'SLAM + Nav2' in msg
    assert 'Robot bringup' in msg
    assert 'drive hardware' in msg
