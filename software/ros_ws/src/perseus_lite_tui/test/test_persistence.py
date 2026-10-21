# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for selection persistence."""

from perseus_lite_tui import persistence


def test_config_dir_uses_xdg():
    env = {'XDG_CONFIG_HOME': '/xdg', 'HOME': '/home/u'}
    assert str(persistence.config_dir(env)) == '/xdg/perseus_lite_tui'


def test_config_dir_falls_back_to_home():
    env = {'HOME': '/home/u'}
    assert str(persistence.config_dir(env)) == '/home/u/.config/perseus_lite_tui'


def test_round_trip(tmp_path):
    path = tmp_path / 'selections.json'
    data = {'bringup': {'use_mock_hardware': 'True'}}
    assert persistence.save_selections(path, data) is True
    assert persistence.load_selections(path) == data


def test_load_missing_returns_empty(tmp_path):
    assert persistence.load_selections(tmp_path / 'nope.json') == {}


def test_load_corrupt_returns_empty(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{not valid json')
    assert persistence.load_selections(path) == {}


def test_save_creates_parent_dirs(tmp_path):
    path = tmp_path / 'nested' / 'dir' / 'selections.json'
    assert persistence.save_selections(path, {'a': {'b': 'c'}}) is True
    assert path.is_file()


def test_load_drops_malformed_entries(tmp_path):
    path = tmp_path / 'selections.json'
    path.write_text(
        '{"bringup": {"use_mock_hardware": true, "bad": [1], "n": 3},'
        ' "list_profile": [1, 2], "str_profile": "oops", "empty": {}}'
    )
    assert persistence.load_selections(path) == {
        'bringup': {'use_mock_hardware': 'True', 'n': '3'},
    }


def test_load_non_dict_top_level_returns_empty(tmp_path):
    path = tmp_path / 'selections.json'
    path.write_text('[1, 2, 3]')
    assert persistence.load_selections(path) == {}


def test_running_jobs_round_trip(tmp_path):
    path = tmp_path / 'running_jobs.json'
    jobs = {'bringup': {'pgid': 4242, 'label': 'Robot bringup', 'resources': ['drive']}}
    assert persistence.save_running_jobs(path, jobs, current_boot='boot-a')
    assert persistence.load_running_jobs(path, current_boot='boot-a') == jobs
    # pgids from another boot are meaningless and must be ignored.
    assert persistence.load_running_jobs(path, current_boot='boot-b') == {}
    # Saving an empty set removes the file.
    assert persistence.save_running_jobs(path, {}, current_boot='boot-a')
    assert not path.exists()


def test_running_jobs_rejects_bad_pgids(tmp_path):
    path = tmp_path / 'running_jobs.json'
    path.write_text(
        '{"boot_id": "b", "jobs": {"a": {"pgid": 1}, "b": {"pgid": "12"},'
        ' "c": {"pgid": true}, "d": [], "e": {"pgid": 99, "resources": "x"}}}'
    )
    assert persistence.load_running_jobs(path, current_boot='b') == {
        'e': {'pgid': 99, 'label': 'e', 'resources': []},
    }
