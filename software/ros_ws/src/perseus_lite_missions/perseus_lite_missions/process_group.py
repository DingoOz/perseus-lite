# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Own a ``ros2 launch`` subprocess *and every process it spawns* (no rclpy).

``ros2 launch`` / ``ros2 run`` are wrappers: signalling only the wrapper PID
leaves the real node processes running, which is how a relaunch used to
leave a duplicate ``/slam_toolbox`` behind. ``ManagedProcess`` starts the
command as the leader of a fresh process group (``start_new_session``) and
always signals the whole group, escalating SIGINT -> SIGTERM -> SIGKILL. It
never touches processes it did not start (no ``pkill``).

Stopping is non-blocking: ``request_stop()`` sends SIGINT and ``poll()``
(call it from a timer) escalates on schedule, so the ROS executor thread
never blocks. ``stop_blocking()`` is for node shutdown only.
"""

import ctypes
import ctypes.util
import os
import signal
import subprocess
import time

IDLE = 'idle'
RUNNING = 'running'
STOPPING = 'stopping'
EXITED = 'exited'

_PR_SET_PDEATHSIG = 1


def _load_prctl():
    """Resolve libc's prctl in the parent (no imports/dlopen after fork)."""
    try:
        libc = ctypes.CDLL(ctypes.util.find_library('c') or 'libc.so.6', use_errno=True)
        return libc.prctl
    except (OSError, AttributeError):
        return None


_PRCTL = _load_prctl()


def _child_setup():
    """Run in the child between fork and exec: die with the orchestrator."""
    # Backstop for an orchestrator that is SIGKILLed before it can stop its
    # groups: the launch wrapper gets SIGTERM and shuts its nodes down.
    if _PRCTL is not None:
        _PRCTL(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)


def group_alive(pgid: int) -> bool:
    """Return True while any process in the group still exists."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class ManagedProcess:
    """One command run as its own process group."""

    def __init__(
        self,
        name: str,
        sigint_timeout: float = 10.0,
        sigterm_timeout: float = 5.0,
        clock=time.monotonic,
    ):
        self.name = name
        self.sigint_timeout = sigint_timeout
        self.sigterm_timeout = sigterm_timeout
        self._clock = clock
        self._proc = None
        self._pgid = None
        self._stop_started = None
        self._sent = None
        self.argv = None
        self.returncode = None

    @property
    def pgid(self):
        """Return the process-group id of the running command, if any."""
        return self._pgid

    @property
    def state(self) -> str:
        """Return idle, running, stopping or exited (does not escalate)."""
        if self._proc is None:
            return IDLE if self.returncode is None else EXITED
        if self._stop_started is not None:
            return STOPPING
        return RUNNING

    def start(self, argv: list) -> None:
        """Start ``argv`` as the leader of a new process group."""
        if self._proc is not None:
            raise RuntimeError(f'{self.name} is already running')
        self.argv = list(argv)
        self.returncode = None
        self._stop_started = None
        self._sent = None
        self._proc = subprocess.Popen(
            self.argv,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            preexec_fn=_child_setup,
        )
        self._pgid = self._proc.pid

    def _signal_group(self, sig) -> None:
        try:
            os.killpg(self._pgid, sig)
        except ProcessLookupError:
            pass
        self._sent = sig

    def request_stop(self) -> None:
        """Begin stopping the whole group (SIGINT now, escalate in poll)."""
        if self._proc is None or self._stop_started is not None:
            return
        self._stop_started = self._clock()
        self._signal_group(signal.SIGINT)

    def poll(self) -> str:
        """
        Reap / escalate and return the current state.

        A group only counts as gone once the leader is reaped *and* no
        other member is left. A leader that exits on its own while not
        being stopped reports ``exited`` (with ``returncode``) even if
        stragglers remain, and the stragglers are then stopped too.
        """
        if self._proc is None:
            return self.state
        code = self._proc.poll()
        if self._stop_started is None:
            if code is None:
                return RUNNING
            # Unexpected exit: record it, then make sure nothing lingers.
            self.returncode = code
            self.request_stop()
        if not group_alive(self._pgid):
            self._proc.poll()
            self.returncode = self._proc.returncode
            self._proc = None
            self._pgid = None
            self._stop_started = None
            return EXITED
        elapsed = self._clock() - self._stop_started
        if self._sent == signal.SIGINT and elapsed >= self.sigint_timeout:
            self._signal_group(signal.SIGTERM)
        elif (
            self._sent == signal.SIGTERM
            and elapsed >= self.sigint_timeout + self.sigterm_timeout
        ):
            self._signal_group(signal.SIGKILL)
        return STOPPING

    def stop_blocking(
        self,
        sigint_timeout: float | None = None,
        sigterm_timeout: float | None = None,
        poll_period: float = 0.05,
    ) -> None:
        """
        Stop the group and wait until it is gone (shutdown path only).

        Optional shorter timeouts let a node shut down inside its parent
        launch's own SIGTERM/SIGKILL grace period.
        """
        if sigint_timeout is not None:
            self.sigint_timeout = sigint_timeout
        if sigterm_timeout is not None:
            self.sigterm_timeout = sigterm_timeout
        self.request_stop()
        # SIGKILL cannot be ignored, so this terminates; the cap is a guard
        # against a process stuck in uninterruptible sleep.
        deadline = self._clock() + self.sigint_timeout + self.sigterm_timeout + 2.0
        while self.poll() == STOPPING and self._clock() < deadline:
            time.sleep(poll_period)
