# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""App-level tests for launch refusal, start failures and signal shutdown."""

import os
from pathlib import Path
import signal
import time

from perseus_lite_tui import app as app_mod
from perseus_lite_tui import jobs, persistence, pixi_env
import pytest


def _repo_root():
    root = pixi_env.find_repo_root(Path(__file__).resolve())
    assert root is not None
    return root


@pytest.fixture
def tui(tmp_path, monkeypatch):
    """Build an App with tmp state, fake launches and no ROS monitor."""
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setattr(app_mod.pixi_env, 'install_setup_exists', lambda _root: True)
    monkeypatch.setattr(
        app_mod,
        'build_launch_command',
        lambda prof, sel, ws: ['bash', '-c', 'sleep 30 & wait'],
    )
    monkeypatch.setattr(jobs, 'EXIT_SIGTERM_AFTER_S', 0.5)
    monkeypatch.setattr(jobs, 'EXIT_SIGKILL_AFTER_S', 0.5)
    instance = app_mod.App(_repo_root())
    monkeypatch.setattr(instance.monitor, 'start', lambda: None)
    yield instance
    instance.jobs.stop_all(wait=True)


def _select(tui, slug):
    profiles = [row[2] for row in tui._profile_rows()]
    tui.launch_sel = [p.slug for p in profiles].index(slug)


def test_second_drive_owner_is_refused(tui):
    _select(tui, 'bringup')
    tui._launch_selected()
    assert tui.jobs.get('bringup').running
    _select(tui, 'slam_nav2')
    tui._launch_selected()
    assert tui.jobs.get('slam_nav2') is None
    assert tui.status.startswith('Refused SLAM + Nav2')
    assert 'Robot bringup' in tui.status
    # A non-conflicting add-on is still allowed alongside the bringup.
    _select(tui, 'autonomy')
    tui._launch_selected()
    assert tui.jobs.get('autonomy').running


def test_start_failure_reports_and_keeps_jobs(tui, monkeypatch):
    _select(tui, 'bringup')
    tui._launch_selected()
    bringup = tui.jobs.get('bringup')
    monkeypatch.setattr(
        app_mod, 'build_launch_command', lambda prof, sel, ws: ['/no/such/pixi-xyz']
    )
    _select(tui, 'xbox')
    tui._launch_selected()
    assert tui.status.startswith('Failed to start Xbox controller')
    assert bringup.running


def test_live_groups_recorded_for_orphan_detection(tui):
    _select(tui, 'bringup')
    tui._launch_selected()
    tui._record_live_groups()
    recorded = persistence.load_running_jobs(tui.jobs_file)
    assert recorded['bringup']['pgid'] == tui.jobs.get('bringup').pgid
    assert 'drive' in recorded['bringup']['resources']
    # A fresh TUI sees the still-running group as an orphan, warns, and
    # refuses to start a second drive owner while it lives.
    other = app_mod.App(_repo_root())
    assert 'bringup' in other.orphans
    assert 'previous session' in other.status
    _select(other, 'controllers')
    other._launch_selected()
    assert other.status.startswith('Refused Controllers only')
    assert other.jobs.get('controllers') is None


def test_sigterm_runs_orderly_stop(tui, monkeypatch):
    started = {}

    def fake_wrapper(_loop):
        _select(tui, 'bringup')
        tui._launch_selected()
        started['job'] = tui.jobs.get('bringup')
        os.kill(os.getpid(), signal.SIGTERM)
        time.sleep(5.0)  # interrupted by the handler's exception
        raise AssertionError('signal handler did not unwind the loop')

    monkeypatch.setattr(app_mod.curses, 'wrapper', fake_wrapper)
    before = signal.getsignal(signal.SIGTERM)
    rc = tui.run()
    assert rc == 128 + signal.SIGTERM
    job = started['job']
    assert not job.group_alive  # stopped and waited for, not orphaned
    assert not tui.jobs_file.exists()
    assert signal.getsignal(signal.SIGTERM) == before
