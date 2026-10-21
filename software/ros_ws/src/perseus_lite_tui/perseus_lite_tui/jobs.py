# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Subprocess job management: process groups, log ring, signal escalation."""

# Touches the OS (subprocess/signal/threading) but imports neither curses nor
# rclpy, so it stays inside the pure-layer import fence and is directly
# unit-testable with short-lived shell commands.
#
# Every job is started in its own session (start_new_session=True) so a Ctrl-C
# or SIGHUP aimed at the TUI's terminal never reaches a live robot bringup
# directly. The flip side is that the TUI itself is responsible for stopping
# the groups: escalation always targets the *whole process group* (pgid ==
# leader pid) and keeps going until no member is left, not merely until the
# leader (`pixi run`) has exited.

import collections
import os
import signal
from subprocess import PIPE, Popen, STDOUT
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

RUNNING = 'running'
STOPPING = 'stopping'
EXITED = 'exited'
FAILED = 'failed'

# Graceful-stop escalation windows for an interactive stop (`k` on a job).
SIGTERM_AFTER_S = 10.0
SIGKILL_AFTER_S = 3.0
# Tighter windows used when the TUI itself is exiting, so the whole shutdown
# (SIGINT -> SIGTERM -> SIGKILL -> reap) stays bounded at ~10 s.
EXIT_SIGTERM_AFTER_S = 6.0
EXIT_SIGKILL_AFTER_S = 3.0
EXIT_TOTAL_TIMEOUT_S = 10.0
_POLL_INTERVAL_S = 0.05


def group_alive(pgid: Optional[int]) -> bool:
    """Return whether any process in process group `pgid` still exists."""
    if pgid is None or pgid <= 0:
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # Exists, but owned by someone else (only possible after pgid reuse).
        return True
    return True


def _signal_group(pgid: int, sig: int) -> None:
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError):
        pass


class Job:
    """A single launched command with a bounded, thread-safe log buffer."""

    def __init__(
        self,
        key: str,
        label: str,
        argv: List[str],
        cwd: Optional[str] = None,
        log_max: int = 4000,
        noise_filter: Optional[Callable[[str], bool]] = None,
        resources: Sequence[str] = (),
    ):
        self.key = key
        self.label = label
        self.argv = list(argv)
        self.cwd = cwd
        self.resources = tuple(resources)
        self.error: Optional[str] = None
        self._log = collections.deque(maxlen=log_max)
        self._lock = threading.Lock()
        self._proc: Optional[Popen] = None
        self._pgid: Optional[int] = None
        self._stopping = False
        self._started_at: Optional[float] = None
        self._noise_filter = noise_filter
        self._escalator: Optional[threading.Thread] = None
        # Latched once the group is observed empty, so a later pgid reuse by
        # an unrelated process can never make this job look alive again (and
        # be signalled by stop()).
        self._group_gone = False

    @property
    def pgid(self) -> Optional[int]:
        """Return the job's process-group id (== leader pid), if started."""
        return self._pgid

    def start(self) -> bool:
        """
        Spawn the process in its own session; return False if it failed.

        A missing executable (or any other spawn-time OSError) is recorded in
        `error` and the log instead of propagating: an exception escaping here
        would unwind the curses loop and stop every *other* running job.
        """
        if self._proc is not None or self.error is not None:
            return self._proc is not None
        self._started_at = time.monotonic()
        try:
            self._proc = Popen(
                self.argv,
                cwd=self.cwd,
                stdout=PIPE,
                stderr=STDOUT,
                start_new_session=True,
                text=True,
                errors='replace',
                bufsize=1,
            )
        except OSError as exc:
            self.error = f'{type(exc).__name__}: {exc}'
            with self._lock:
                self._log.append(f'failed to start {self.argv[:1]}: {self.error}')
            return False
        # start_new_session makes the child a session and group leader.
        self._pgid = self._proc.pid
        reader = threading.Thread(target=self._drain, daemon=True)
        reader.start()
        return True

    def _drain(self) -> None:
        stream = self._proc.stdout if self._proc else None
        if stream is None:
            return
        for line in stream:
            text = line.rstrip('\n')
            if self._noise_filter is not None and self._noise_filter(text):
                continue
            with self._lock:
                self._log.append(text)
        try:
            stream.close()
        except OSError:
            pass

    def snapshot_log(self) -> List[str]:
        """Return a copy of the current log buffer."""
        with self._lock:
            return list(self._log)

    @property
    def running(self) -> bool:
        """Return whether the process is still alive."""
        return self._proc is not None and self._proc.poll() is None

    @property
    def group_alive(self) -> bool:
        """Return whether any member of the job's process group survives."""
        if self._group_gone or self._pgid is None:
            return False
        if self._proc is not None and self._proc.poll() is None:
            return True  # leader alive (polling also reaps it once it exits)
        if group_alive(self._pgid):
            return True
        self._group_gone = True
        return False

    @property
    def returncode(self) -> Optional[int]:
        """Return the process exit code, or None if unstarted/still running."""
        return None if self._proc is None else self._proc.poll()

    def status(self) -> str:
        """Return one of RUNNING / STOPPING / EXITED / FAILED."""
        if self.error is not None:
            return FAILED
        if self._proc is None or self._proc.poll() is not None:
            return EXITED
        return STOPPING if self._stopping else RUNNING

    def status_line(self) -> str:
        """Return a short human-readable status string."""
        if self.error is not None:
            return 'failed to start'
        if self._proc is None:
            return 'not started'
        rc = self._proc.poll()
        if rc is None:
            elapsed = time.monotonic() - (self._started_at or time.monotonic())
            state = 'stopping' if self._stopping else 'running'
            return f'{state} {elapsed:.0f}s'
        return f'exited ({rc})'

    def stop(
        self,
        sigterm_after: Optional[float] = None,
        sigkill_after: Optional[float] = None,
    ) -> None:
        """
        Send SIGINT to the process group, escalating on a worker thread.

        Escalation continues until the whole group is gone (not just the
        leader), so `ros2 launch` children that outlive `pixi run` are still
        terminated. Call wait_stopped() to block until it has finished.
        """
        if self._pgid is None or not self.group_alive:
            return
        if self._escalator is not None and self._escalator.is_alive():
            return
        if sigterm_after is None:
            sigterm_after = SIGTERM_AFTER_S
        if sigkill_after is None:
            sigkill_after = SIGKILL_AFTER_S
        self._stopping = True
        _signal_group(self._pgid, signal.SIGINT)
        self._escalator = threading.Thread(
            target=self._escalate,
            args=(self._pgid, sigterm_after, sigkill_after),
            daemon=True,
        )
        self._escalator.start()

    def wait_stopped(self, timeout: float) -> bool:
        """Block until escalation finished and the group is gone (bounded)."""
        deadline = time.monotonic() + max(0.0, timeout)
        escalator = self._escalator
        if escalator is not None:
            escalator.join(max(0.0, deadline - time.monotonic()))
        return self._wait_group_exit(max(0.0, deadline - time.monotonic()))

    def _escalate(self, pgid: int, sigterm_after: float, sigkill_after: float) -> None:
        if self._wait_group_exit(sigterm_after):
            return
        _signal_group(pgid, signal.SIGTERM)
        if self._wait_group_exit(sigkill_after):
            return
        _signal_group(pgid, signal.SIGKILL)
        self._wait_group_exit(1.0)

    def _wait_group_exit(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.group_alive:
                return True
            time.sleep(_POLL_INTERVAL_S)
        return not self.group_alive


class JobManager:
    """Owns the set of jobs; at most one live job per key."""

    def __init__(self, noise_filter: Optional[Callable[[str], bool]] = None):
        self._jobs: Dict[str, Job] = {}
        self._order: List[str] = []
        self._noise_filter = noise_filter

    def conflicts(
        self, resources: Iterable[str], key: Optional[str] = None
    ) -> List[Job]:
        """Return live jobs (other than `key`) holding any of `resources`."""
        wanted = set(resources)
        if not wanted:
            return []
        return [
            job
            for job in self.jobs()
            if job.key != key and job.group_alive and wanted & set(job.resources)
        ]

    def start(
        self,
        key: str,
        label: str,
        argv: List[str],
        cwd: Optional[str],
        resources: Sequence[str] = (),
    ) -> Tuple[Job, bool]:
        """
        Start a job for `key`; returns (job, started).

        started is False when the key is already live (the live job is
        returned) or when spawning failed (the returned job has `error` set and
        stays listed so its log explains why). Resource conflicts are the
        caller's responsibility; see conflicts().
        """
        existing = self._jobs.get(key)
        if existing is not None and existing.group_alive:
            return existing, False
        job = Job(
            key, label, argv, cwd, noise_filter=self._noise_filter, resources=resources
        )
        started = job.start()
        self._jobs[key] = job
        if key not in self._order:
            self._order.append(key)
        return job, started

    def get(self, key: str) -> Optional[Job]:
        """Return the job for `key`, if any."""
        return self._jobs.get(key)

    def jobs(self) -> List[Job]:
        """Return jobs in start order."""
        return [self._jobs[k] for k in self._order if k in self._jobs]

    def running_jobs(self) -> List[Job]:
        """Return only the jobs that are still alive."""
        return [job for job in self.jobs() if job.running]

    def live_groups(self) -> Dict[str, dict]:
        """Return {key: {pgid, label, resources}} for every surviving group."""
        return {
            job.key: {
                'pgid': job.pgid,
                'label': job.label,
                'resources': list(job.resources),
            }
            for job in self.jobs()
            if job.pgid is not None and job.group_alive
        }

    def stop(self, key: str) -> None:
        """Stop the job for `key`, if any."""
        job = self._jobs.get(key)
        if job is not None:
            job.stop()

    def stop_all(
        self, wait: bool = False, timeout: Optional[float] = None
    ) -> List[Job]:
        """
        Stop every managed job; with wait=True block (bounded) until gone.

        Used on exit: all groups get SIGINT at once, escalate in parallel on
        the exit windows, and this call only returns once every group is gone
        or `timeout` elapsed, so no escalator thread is abandoned mid-way when
        the interpreter exits. Returns the jobs whose group still survives.
        """
        for job in self.jobs():
            if wait:
                job.stop(EXIT_SIGTERM_AFTER_S, EXIT_SIGKILL_AFTER_S)
            else:
                job.stop()
        if not wait:
            return []
        if timeout is None:
            timeout = EXIT_TOTAL_TIMEOUT_S
        deadline = time.monotonic() + timeout
        for job in self.jobs():
            job.wait_stopped(max(0.0, deadline - time.monotonic()))
        return [job for job in self.jobs() if job.group_alive]

    def clear_finished(self) -> int:
        """Forget jobs that have exited; return how many were removed."""
        finished = [k for k in self._order if not self._jobs[k].group_alive]
        for key in finished:
            del self._jobs[key]
            self._order.remove(key)
        return len(finished)
