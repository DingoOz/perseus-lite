# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Validate that the profile/task config matches the workspace on disk."""

from pathlib import Path

from perseus_lite_tui import config, pixi_env, registry


def _repo_root():
    root = pixi_env.find_repo_root(Path(__file__).resolve())
    assert root is not None, 'could not locate pixi.toml above the test file'
    return root


def test_profile_ids_are_unique():
    ids = [p.slug for p in config.PROFILES]
    assert len(ids) == len(set(ids))


def test_task_ids_are_unique():
    ids = [t.slug for t in config.TASKS]
    assert len(ids) == len(set(ids))


def test_every_profile_launch_file_exists():
    src = pixi_env.ros_ws_dir(_repo_root()) / 'src'
    missing = [
        str(src / p.package / 'launch' / p.launch_file)
        for p in config.PROFILES
        if not (src / p.package / 'launch' / p.launch_file).is_file()
    ]
    assert not missing, 'missing launch files: %s' % missing


def test_isaac_compose_files_exist():
    root = _repo_root()
    compose_paths = set()
    for task in config.TASKS:
        argv = task.argv
        if 'docker' in argv and '-f' in argv:
            compose_paths.add(argv[argv.index('-f') + 1])
    assert compose_paths, 'expected at least one docker compose task'
    missing = [p for p in compose_paths if not (root / p).is_file()]
    assert not missing, 'missing compose files: %s' % missing


def test_simulation_profiles_are_gated():
    sim = [p for p in config.PROFILES if p.env == 'simulation']
    assert sim, 'expected at least one simulation profile'
    assert all(p.linux64_only for p in sim)


def _by_slug():
    return {p.slug: p for p in config.PROFILES}


def test_drive_owners_are_mutually_exclusive():
    # Every profile that starts controller_manager + the ST3215 bus (or the
    # simulated equivalent on the same ROS domain) must claim DRIVE, so the TUI
    # refuses to run two at once.
    profiles = _by_slug()
    for slug in ('bringup', 'controllers', 'slam_nav2', 'nav2', 'mapping', 'sim'):
        assert registry.DRIVE in profiles[slug].resources, slug
    assert registry.DRIVE not in profiles['rsp'].resources
    assert registry.DRIVE not in profiles['autonomy'].resources


def test_no_profile_bypasses_twist_mux():
    # cmd_vel_topic, when passed, must be the mux output the controller reads.
    for prof in config.PROFILES:
        for arg in prof.fixed_args:
            key, _, value = arg.partition(':=')
            if key == 'cmd_vel_topic':
                assert value == '/cmd_vel', prof.slug
        for option in prof.options:
            assert option.key != 'cmd_vel_topic', prof.slug
    profiles = _by_slug()
    for slug in ('slam_nav2', 'nav2'):
        assert 'cmd_vel_topic:=/cmd_vel' in profiles[slug].fixed_args, slug


def test_resources_are_known():
    for prof in config.PROFILES:
        for resource in prof.resources:
            assert resource in registry.RESOURCE_LABELS, (prof.slug, resource)
