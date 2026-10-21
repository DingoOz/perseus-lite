# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""ManagedProcess must stop a wrapper *and* everything it spawned."""

import os
import signal
import sys
import time

from perseus_lite_missions.process_group import (
    EXITED,
    group_alive,
    IDLE,
    ManagedProcess,
    RUNNING,
    STOPPING,
)
import pytest

pytestmark = pytest.mark.skipif(
    not sys.platform.startswith('linux'), reason='process groups / prctl are Linux'
)

# A "wrapper" that spawns a long-lived child, like `ros2 launch` spawning
# async_slam_toolbox_node. The child ignores SIGINT to force escalation.
WRAPPER = 'sleep 60 & wait'
STUBBORN = "trap '' INT; sleep 60 & wait"
STUBBORN_CHILD = "bash -c \"trap '' INT TERM; sleep 60\" & trap '' INT; wait"


def _wait_for(proc, state, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() == state:
            return True
        time.sleep(0.05)
    return False


def _children_pids(pgid):
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            if os.getpgid(int(entry)) == pgid:
                pids.append(int(entry))
        except ProcessLookupError:
            continue
    return pids


def test_idle_before_start():
    proc = ManagedProcess('t')
    assert proc.state == IDLE
    assert proc.poll() == IDLE
    proc.request_stop()  # no-op, must not raise
    assert proc.state == IDLE


def test_runs_in_its_own_group():
    proc = ManagedProcess('t')
    proc.start(['bash', '-c', WRAPPER])
    try:
        assert proc.poll() == RUNNING
        assert proc.pgid != os.getpgid(0)
        assert os.getpgid(proc._proc.pid) == proc.pgid
    finally:
        proc.stop_blocking(sigint_timeout=1.0, sigterm_timeout=1.0)


def test_stop_kills_grandchildren():
    proc = ManagedProcess('t', sigint_timeout=1.0, sigterm_timeout=1.0)
    proc.start(['bash', '-c', WRAPPER])
    time.sleep(0.3)
    pgid = proc.pgid
    assert len(_children_pids(pgid)) >= 2  # bash + sleep
    proc.request_stop()
    assert proc.poll() in (STOPPING, EXITED)
    assert _wait_for(proc, EXITED)
    assert not group_alive(pgid)
    assert proc.state == EXITED


def test_escalates_to_sigterm_then_sigkill():
    now = [0.0]
    proc = ManagedProcess(
        't', sigint_timeout=1.0, sigterm_timeout=1.0, clock=lambda: now[0]
    )
    proc.start(['bash', '-c', STUBBORN_CHILD])
    time.sleep(0.3)
    pgid = proc.pgid
    proc.request_stop()
    assert proc._sent == signal.SIGINT
    time.sleep(0.2)
    assert proc.poll() == STOPPING  # everything ignores SIGINT
    now[0] = 1.5
    proc.poll()
    assert proc._sent == signal.SIGTERM
    time.sleep(0.3)
    # The wrapper dies on SIGTERM; the grandchild ignores it and lingers.
    assert proc.poll() == STOPPING
    assert group_alive(pgid)
    now[0] = 3.0
    proc.poll()
    assert proc._sent == signal.SIGKILL
    assert _wait_for(proc, EXITED)
    assert not group_alive(pgid)


def test_stop_blocking_returns_when_gone():
    proc = ManagedProcess('t')
    proc.start(['bash', '-c', STUBBORN])
    time.sleep(0.2)
    pgid = proc.pgid
    start = time.monotonic()
    proc.stop_blocking(sigint_timeout=0.5, sigterm_timeout=0.5)
    assert time.monotonic() - start < 5.0
    assert not group_alive(pgid)
    assert proc.state == EXITED


def test_unexpected_exit_is_reported_and_restartable():
    proc = ManagedProcess('t')
    proc.start(['bash', '-c', 'exit 3'])
    assert _wait_for(proc, EXITED)
    assert proc.returncode == 3
    proc.start(['bash', '-c', WRAPPER])
    assert proc.poll() == RUNNING
    proc.stop_blocking(sigint_timeout=1.0, sigterm_timeout=1.0)


def test_cannot_double_start():
    proc = ManagedProcess('t')
    proc.start(['bash', '-c', WRAPPER])
    try:
        with pytest.raises(RuntimeError):
            proc.start(['true'])
    finally:
        proc.stop_blocking(sigint_timeout=1.0, sigterm_timeout=1.0)
