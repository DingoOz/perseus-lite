# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the subprocess job manager."""

import os
import time

from perseus_lite_tui import jobs
import pytest


def _wait_until(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    # A zombie still answers kill(0); treat it as gone.
    try:
        with open(f'/proc/{pid}/stat', 'r', encoding='ascii') as handle:
            return handle.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def _grandchild_pid(job, timeout=5.0):
    """Return the pid the job's shell echoed as `child=<pid>`."""
    found = {}

    def _scan():
        for line in job.snapshot_log():
            if line.startswith('child='):
                found['pid'] = int(line.split('=', 1)[1])
                return True
        return False

    assert _wait_until(_scan, timeout), job.snapshot_log()
    return found['pid']


@pytest.fixture
def fast_escalation(monkeypatch):
    """Shrink the interactive escalation windows so tests stay quick."""
    monkeypatch.setattr(jobs, 'SIGTERM_AFTER_S', 0.5)
    monkeypatch.setattr(jobs, 'SIGKILL_AFTER_S', 0.5)


def test_job_runs_and_captures_output():
    job = jobs.Job('echo', 'echo', ['bash', '-c', 'echo hello; echo world'])
    assert job.start() is True
    assert _wait_until(lambda: not job.running)
    log = job.snapshot_log()
    assert 'hello' in log
    assert 'world' in log
    assert job.returncode == 0
    assert job.status() == jobs.EXITED


def test_job_stop_terminates_process_group(fast_escalation):
    # `sleep 30 &` + `wait` keeps bash alive as the leader *and* leaves a
    # separate grandchild in the group, so this exercises killpg, not just a
    # signal to the leader (a bare `bash -c 'sleep 30'` execs sleep). A
    # non-interactive shell's background jobs ignore SIGINT, so the grandchild
    # outlives the leader and only goes once escalation reaches SIGTERM.
    argv = ['bash', '-c', 'sleep 30 & echo child=$!; wait']
    job = jobs.Job('group', 'group', argv)
    job.start()
    child = _grandchild_pid(job)
    assert child != job.pgid
    assert os.getpgid(child) == job.pgid
    job.stop()
    assert job.wait_stopped(10.0)
    assert not job.running
    assert job.returncode is not None
    assert _wait_until(lambda: not _pid_alive(child), 5.0)
    assert not job.group_alive


def test_stop_kills_group_even_after_leader_exits():
    # The leader exits on SIGINT but its child ignores SIGINT and survives;
    # escalation must keep going on the group, not stop at the leader.
    script = (
        "(trap '' INT; sleep 30) & echo child=$!; "
        "trap 'exit 0' INT; while true; do sleep 0.05; done"
    )
    job = jobs.Job('leader', 'leader', ['bash', '-c', script])
    job.start()
    child = _grandchild_pid(job)
    job.stop(sigterm_after=0.5, sigkill_after=0.5)
    assert job.wait_stopped(10.0)
    assert _wait_until(lambda: not _pid_alive(child), 5.0)


def test_stop_escalates_to_sigkill(fast_escalation):
    # Ignores both SIGINT and SIGTERM, so only SIGKILL can end it.
    script = "trap '' INT TERM; echo ready; while true; do sleep 0.05; done"
    job = jobs.Job('stubborn', 'stubborn', ['bash', '-c', script])
    job.start()
    assert _wait_until(lambda: 'ready' in job.snapshot_log())
    started = time.monotonic()
    job.stop()
    assert job.status() == jobs.STOPPING
    assert job.wait_stopped(10.0)
    elapsed = time.monotonic() - started
    assert job.returncode == -9  # killed by SIGKILL
    assert elapsed >= 0.9  # waited out both SIGINT and SIGTERM windows


def test_missing_executable_does_not_raise():
    job = jobs.Job('nope', 'nope', ['/definitely/not/a/real/binary-xyz'])
    assert job.start() is False
    assert job.error is not None
    assert 'FileNotFoundError' in job.error
    assert job.status() == jobs.FAILED
    assert job.status_line() == 'failed to start'
    assert not job.running
    assert not job.group_alive
    assert any('failed to start' in line for line in job.snapshot_log())
    job.stop()  # must be a harmless no-op


def test_manager_start_failure_keeps_other_jobs_alive():
    manager = jobs.JobManager()
    live, started = manager.start('live', 'Live', ['bash', '-c', 'sleep 30'], None)
    assert started
    bad, bad_started = manager.start('bad', 'Bad', ['/no/such/exe-xyz'], None)
    assert not bad_started
    assert bad.error is not None
    assert manager.get('bad') is bad  # listed, so its log explains the failure
    assert live.running
    manager.stop_all(wait=True)
    assert not live.running


def test_stop_all_wait_is_bounded_and_complete(monkeypatch):
    monkeypatch.setattr(jobs, 'EXIT_SIGTERM_AFTER_S', 0.3)
    monkeypatch.setattr(jobs, 'EXIT_SIGKILL_AFTER_S', 0.3)
    manager = jobs.JobManager()
    script = "trap '' INT TERM; sleep 30 & echo child=$!; wait"
    a, _ = manager.start('a', 'A', ['bash', '-c', script], None)
    b, _ = manager.start('b', 'B', ['bash', '-c', 'sleep 30 & wait'], None)
    child = _grandchild_pid(a)
    started = time.monotonic()
    survivors = manager.stop_all(wait=True, timeout=5.0)
    assert survivors == []
    assert time.monotonic() - started < 5.0
    assert not a.group_alive and not b.group_alive
    assert _wait_until(lambda: not _pid_alive(child), 2.0)


def test_log_buffer_is_bounded():
    argv = ['bash', '-c', 'for i in $(seq 1 200); do echo line$i; done']
    job = jobs.Job('spam', 'spam', argv, log_max=10)
    job.start()
    assert _wait_until(lambda: not job.running)
    assert len(job.snapshot_log()) <= 10


def test_noise_filter_drops_lines():
    argv = ['bash', '-c', 'echo keep; echo NOISE; echo keep2']
    job = jobs.Job('nf', 'nf', argv, noise_filter=lambda line: line == 'NOISE')
    job.start()
    assert _wait_until(lambda: not job.running)
    assert 'NOISE' not in job.snapshot_log()
    assert 'keep' in job.snapshot_log()


def test_manager_one_job_per_key():
    manager = jobs.JobManager()
    job1, started1 = manager.start('a', 'A', ['bash', '-c', 'sleep 30'], None)
    assert started1
    job2, started2 = manager.start('a', 'A', ['bash', '-c', 'sleep 30'], None)
    assert not started2
    assert job1 is job2
    manager.stop_all(wait=True)
    assert _wait_until(lambda: not manager.running_jobs())


def test_manager_conflicts_by_resource():
    manager = jobs.JobManager()
    hw, _ = manager.start(
        'bringup', 'Bringup', ['bash', '-c', 'sleep 30'], None, resources=('drive',)
    )
    try:
        clash = manager.conflicts(('drive', 'slam'), key='slam_nav2')
        assert clash == [hw]
        assert manager.conflicts(('slam',), key='mapping') == []
        assert manager.conflicts(('drive',), key='bringup') == []  # itself
        assert manager.conflicts((), key='x') == []
        groups = manager.live_groups()
        assert groups['bringup']['pgid'] == hw.pgid
        assert groups['bringup']['resources'] == ['drive']
    finally:
        manager.stop_all(wait=True)
    assert manager.conflicts(('drive',), key='slam_nav2') == []
    assert manager.live_groups() == {}


def test_group_alive_helper():
    assert jobs.group_alive(None) is False
    assert jobs.group_alive(0) is False
    assert jobs.group_alive(os.getpgid(0)) is True


def test_manager_clear_finished():
    manager = jobs.JobManager()
    job, _ = manager.start('done', 'done', ['bash', '-c', 'true'], None)
    assert _wait_until(lambda: not job.running)
    assert manager.clear_finished() == 1
    assert manager.get('done') is None
