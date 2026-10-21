# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Curses front-end: tabs, key handling, and the render loop."""

# This is the only module that imports curses. All display logic funnels
# through `_addstr`, which clips to the window and swallows curses errors so a
# tiny or resizing terminal can never crash the loop.

import curses
from pathlib import Path
import signal
import sys

from perseus_lite_tui import (
    config,
    jobs,
    layout,
    persistence,
    pixi_env,
    ros_monitor,
    topic_stats,
)
from perseus_lite_tui.registry import (
    BOOL,
    build_launch_command,
    build_task_command,
    CHOICE,
    normalize_bool,
    resource_conflict_message,
)

TAB_LAUNCH = 0
TAB_TASKS = 1
TAB_MONITOR = 2
TAB_LOGS = 3

_TAB_TITLES = ['Launch', 'Tasks', 'Monitor', 'Logs']

_HINTS = {
    TAB_LAUNCH: '↑↓ pick  ←→ option  l/enter launch  r reset  1-4 tabs  q quit',
    TAB_TASKS: '↑↓ pick  l/enter run  1-4 tabs  q quit',
    TAB_MONITOR: '↑↓ scroll  1-4 tabs  q quit',
    TAB_LOGS: '[ ] job  ↑↓ scroll  k stop  x clear finished  1-4 tabs  q quit',
}

# Signals that trigger the same orderly stop path as `q`. SIGHUP matters most:
# closing the terminal / dropping SSH would otherwise kill the TUI while every
# job (each in its own session) keeps running, orphaned, holding the robot.
_SHUTDOWN_SIGNALS = (signal.SIGHUP, signal.SIGTERM)


class _Shutdown(BaseException):
    """
    Raised from a signal handler to unwind the curses loop.

    A BaseException (like KeyboardInterrupt) so the per-key `except Exception`
    guard in the loop can never swallow it.
    """

    def __init__(self, signum: int):
        super().__init__(signum)
        self.signum = signum


class App:
    """Owns TUI state and drives the curses render/input loop."""

    def __init__(self, repo_root: Path):
        self.repo_root = Path(repo_root)
        self.ros_ws = str(pixi_env.ros_ws_dir(self.repo_root))
        self.profiles = pixi_env.visible_profiles(config.PROFILES)
        self.tasks = list(config.TASKS)
        self.jobs = jobs.JobManager(noise_filter=None)
        self.state_file = persistence.state_path()
        self.selections = persistence.load_selections(self.state_file)
        self.jobs_file = persistence.jobs_path()
        self.orphans = self._find_orphans()
        self._recorded_groups = None

        self.active = TAB_LAUNCH
        self.launch_sel = 0
        self.opt_focus = 0
        self.task_sel = 0
        self.log_sel = 0
        self.log_scroll = 0
        self.mon_scroll = 0
        self.monitor = ros_monitor.RosMonitor()
        self.status = self._orphan_warning()
        self.colors = {}
        self.stdscr = None

    # -- lifecycle ------------------------------------------------------------

    def run(self) -> int:
        """Run the curses loop; on any exit stop every job (bounded wait)."""
        exit_code = 0
        previous = self._install_signal_handlers()
        try:
            curses.wrapper(self._loop)
        except _Shutdown as exc:
            exit_code = 128 + exc.signum
        except KeyboardInterrupt:
            exit_code = 130
        finally:
            self._shutdown()
            self._restore_signal_handlers(previous)
        return exit_code

    def _install_signal_handlers(self) -> dict:
        previous = {signal.SIGINT: signal.getsignal(signal.SIGINT)}
        for signum in _SHUTDOWN_SIGNALS:
            try:
                previous[signum] = signal.signal(signum, self._on_shutdown_signal)
            except (OSError, ValueError):
                pass
        return previous

    def _restore_signal_handlers(self, previous: dict) -> None:
        for signum, handler in previous.items():
            try:
                signal.signal(signum, handler)
            except (OSError, ValueError, TypeError):
                pass

    def _on_shutdown_signal(self, signum, _frame) -> None:
        # Further SIGHUP/SIGTERM/SIGINT must not interrupt the bounded stop
        # that follows, so ignore them from here on before unwinding.
        for other in (*_SHUTDOWN_SIGNALS, signal.SIGINT):
            try:
                signal.signal(other, signal.SIG_IGN)
            except (OSError, ValueError):
                pass
        raise _Shutdown(signum)

    def _shutdown(self) -> None:
        """Orderly stop: SIGINT -> SIGTERM -> SIGKILL every group, then wait."""
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except (OSError, ValueError):
            pass
        live = self.jobs.running_jobs()
        if live:
            self._notify(f'perseus_tui: stopping {len(live)} job(s)…')
        survivors = self.jobs.stop_all(wait=True)
        for job in survivors:
            self._notify(
                f'perseus_tui: {job.label} (pgid {job.pgid}) did not exit; '
                f'stop it with: kill -KILL -- -{job.pgid}'
            )
        self._record_live_groups(force=True)
        self.monitor.stop()
        persistence.save_selections(self.state_file, self.selections)

    @staticmethod
    def _notify(message: str) -> None:
        # The terminal may already be gone (SIGHUP): never let this raise.
        try:
            print(message, file=sys.stderr, flush=True)
        except (OSError, ValueError):
            pass

    # -- orphan tracking ------------------------------------------------------

    def _find_orphans(self) -> dict:
        recorded = persistence.load_running_jobs(self.jobs_file)
        return {
            key: entry
            for key, entry in recorded.items()
            if jobs.group_alive(entry['pgid'])
        }

    def _orphan_warning(self) -> str:
        if not self.orphans:
            return ''
        parts = ', '.join(
            f'{entry["label"]} (pgid {entry["pgid"]})'
            for entry in self.orphans.values()
        )
        first = next(iter(self.orphans.values()))['pgid']
        return (
            f'WARNING: still running from a previous session: {parts}. '
            f'Stop with e.g. kill -INT -- -{first}'
        )

    def _record_live_groups(self, force: bool = False) -> None:
        """Persist live job pgids (plus surviving orphans) when they change."""
        self.orphans = {
            key: entry
            for key, entry in self.orphans.items()
            if jobs.group_alive(entry['pgid'])
        }
        groups = dict(self.orphans)
        groups.update(self.jobs.live_groups())
        if force or groups != self._recorded_groups:
            persistence.save_running_jobs(self.jobs_file, groups)
            self._recorded_groups = groups

    def _resource_conflicts(self, key: str, resources) -> dict:
        """Return {holder label: [shared resources]} for live jobs/orphans."""
        wanted = set(resources)
        holders = {}
        for job in self.jobs.conflicts(wanted, key=key):
            holders[job.label] = sorted(wanted & set(job.resources))
        for entry in self.orphans.values():
            shared = wanted & set(entry['resources'])
            if shared and jobs.group_alive(entry['pgid']):
                label = f'{entry["label"]} [orphan pgid {entry["pgid"]}]'
                holders[label] = sorted(shared)
        return holders

    def _loop(self, stdscr) -> None:
        self.stdscr = stdscr
        curses.curs_set(0)
        stdscr.timeout(100)
        self._init_colors()
        self.monitor.start()
        while True:
            self._record_live_groups()
            self._render()
            key = stdscr.getch()
            if key == -1 or key == curses.KEY_RESIZE:
                continue
            try:
                if self._handle_key(key):
                    return
            except Exception as exc:  # noqa: BLE001
                # A bug in one action must not unwind the loop: that would run
                # the exit path and stop every live job (e.g. robot bringup).
                self.status = f'Error: {type(exc).__name__}: {exc}'

    def _init_colors(self) -> None:
        try:
            curses.start_color()
            curses.use_default_colors()
        except curses.error:
            return
        pairs = {
            'header': (curses.COLOR_WHITE, curses.COLOR_BLUE),
            'group': (curses.COLOR_CYAN, -1),
            'running': (curses.COLOR_GREEN, -1),
            'bad': (curses.COLOR_RED, -1),
            'dim': (curses.COLOR_WHITE, -1),
        }
        for index, (name, (fg, bg)) in enumerate(pairs.items(), start=1):
            try:
                curses.init_pair(index, fg, bg)
                self.colors[name] = curses.color_pair(index)
            except curses.error:
                self.colors[name] = 0
        self.colors['dim'] = self.colors.get('dim', 0) | curses.A_DIM

    def _color(self, name: str) -> int:
        return self.colors.get(name, 0)

    # -- drawing --------------------------------------------------------------

    def _addstr(self, y: int, x: int, text: str, attr: int = 0) -> None:
        maxy, maxx = self.stdscr.getmaxyx()
        if y < 0 or y >= maxy or x >= maxx or x < 0:
            return
        clipped = text[: max(0, maxx - x)]
        try:
            self.stdscr.addstr(y, x, clipped, attr)
        except curses.error:
            pass

    def _render(self) -> None:
        self.stdscr.erase()
        maxy, maxx = self.stdscr.getmaxyx()
        self._addstr(
            0,
            0,
            ' perseus-lite mission control '.ljust(maxx),
            self._color('header') | curses.A_BOLD,
        )
        self._draw_tabs(1)
        body_top, body_bottom = 3, maxy - 2
        if self.active == TAB_LAUNCH:
            self._draw_launch(body_top, body_bottom)
        elif self.active == TAB_TASKS:
            self._draw_tasks(body_top, body_bottom)
        elif self.active == TAB_MONITOR:
            self._draw_monitor(body_top, body_bottom)
        else:
            self._draw_logs(body_top, body_bottom)
        self._addstr(maxy - 1, 0, self.status[:maxx], curses.A_BOLD)
        hint = _HINTS.get(self.active, '')
        self._addstr(maxy - 1, max(0, maxx - len(hint) - 1), hint, curses.A_DIM)
        self.stdscr.refresh()

    def _draw_tabs(self, y: int) -> None:
        x = 0
        for text, is_active in layout.tab_bar(_TAB_TITLES, self.active):
            attr = curses.A_REVERSE if is_active else curses.A_DIM
            self._addstr(y, x, text, attr)
            x += len(text) + 1

    def _draw_launch(self, top: int, bottom: int) -> None:
        rows = self._launch_rows()
        detail_height = 4
        list_bottom = bottom - detail_height
        height = max(1, list_bottom - top)
        target = self._selected_row_index(rows)
        offset = target - height + 1 if target >= height else 0
        offset = layout.clamp_scroll(offset, len(rows), height)
        end = offset + height
        window = rows[offset:end]
        y = top
        for kind, text, ref in window:
            if kind == 'group':
                self._addstr(y, 0, text, self._color('group') | curses.A_BOLD)
            else:
                selected = ref is self._selected_profile()
                attr = curses.A_REVERSE if selected else 0
                if ref.disabled:
                    attr = curses.A_DIM
                self._addstr(y, 0, '  ' + text, attr)
                self._addstr(y, 2 + len(text) + 2, self._job_badge(ref.slug))
            y += 1
        self._draw_launch_detail(list_bottom + 1, bottom)

    def _draw_launch_detail(self, top: int, bottom: int) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        self._addstr(top, 0, '─' * 40, curses.A_DIM)
        if prof.disabled:
            self._addstr(top + 1, 0, prof.note or 'Disabled.', self._color('dim'))
            return
        if not prof.options:
            self._addstr(top + 1, 0, '(no options)', curses.A_DIM)
        x = 0
        for index, option in enumerate(prof.options):
            value = self._opt_value(prof, option)
            label = f'{option.label}: {value}'
            focused = index == self.opt_focus
            attr = curses.A_REVERSE if focused else 0
            self._addstr(top + 1, x, label, attr)
            x += len(label) + 3
        if prof.note:
            self._addstr(top + 2, 0, prof.note, curses.A_DIM)

    def _draw_tasks(self, top: int, bottom: int) -> None:
        y = top
        for index, task in enumerate(self.tasks):
            if y > bottom:
                break
            selected = index == self.task_sel
            attr = curses.A_REVERSE if selected else 0
            self._addstr(y, 0, '  ' + task.label, attr)
            self._addstr(y, 40, self._job_badge('task:' + task.slug))
            if task.note:
                self._addstr(y, 56, task.note, curses.A_DIM)
            y += 1

    def _draw_monitor(self, top: int, bottom: int) -> None:
        if not self.monitor.available:
            self._addstr(
                top,
                0,
                self.monitor.error or 'ROS monitoring unavailable.',
                self._color('dim'),
            )
            self._addstr(
                top + 1,
                0,
                'The rest of the TUI works without ROS.',
                curses.A_DIM,
            )
            return
        rows = self.monitor.snapshot()
        header = '{:<34}{:>8}  {:>6}  type'.format('topic', 'rate', 'age')
        self._addstr(top, 0, header, curses.A_BOLD)
        if not rows:
            self._addstr(top + 1, 0, 'Waiting for topics…', curses.A_DIM)
            return
        height = max(1, bottom - (top + 1))
        view, self.mon_scroll = layout.visible_slice(rows, self.mon_scroll, height)
        y = top + 1
        for row in view:
            topic = row['topic']
            age = row['age']
            rate = topic_stats.format_rate(row['rate'])
            stale = age is not None and topic_stats.is_stale(age)
            attr = self._color('bad') if stale else self._color('running')
            age_text = '     -' if age is None else f'{age:6.1f}'
            self._addstr(y, 0, f'{topic:<34}')
            self._addstr(y, 34, f'{rate:>8}', attr)
            self._addstr(y, 44, age_text, attr)
            self._addstr(y, 52, row['type'], curses.A_DIM)
            y += 1

    def _handle_monitor_key(self, key: int) -> None:
        if key == curses.KEY_UP:
            self.mon_scroll = max(0, self.mon_scroll - 1)
        elif key == curses.KEY_DOWN:
            self.mon_scroll += 1

    def _draw_logs(self, top: int, bottom: int) -> None:
        all_jobs = self.jobs.jobs()
        if not all_jobs:
            self._addstr(
                top,
                0,
                'No jobs yet. Launch something from the Launch tab.',
                curses.A_DIM,
            )
            return
        self.log_sel = max(0, min(self.log_sel, len(all_jobs) - 1))
        x = 0
        for index, job in enumerate(all_jobs):
            marker = '▸' if index == self.log_sel else ' '
            label = f'{marker}{job.label}'
            attr = curses.A_REVERSE if index == self.log_sel else self._job_attr(job)
            self._addstr(top, x, label, attr)
            x += len(label) + 2
        job = all_jobs[self.log_sel]
        self._addstr(
            top + 1,
            0,
            f'{job.label} — {job.status_line()}',
            self._job_attr(job) | curses.A_BOLD,
        )
        self._draw_log_body(job, top + 2, bottom)

    def _draw_log_body(self, job, top: int, bottom: int) -> None:
        height = max(1, bottom - top + 1)
        lines = job.snapshot_log()
        if self.log_scroll <= 0:
            view = layout.tail(lines, height)
        else:
            end = max(0, len(lines) - self.log_scroll)
            start = max(0, end - height)
            view = lines[start:end]
        for offset, line in enumerate(view):
            self._addstr(top + offset, 0, line)

    # -- row/selection helpers ------------------------------------------------

    def _launch_rows(self):
        rows = []
        last_group = None
        for prof in self.profiles:
            if prof.group != last_group:
                rows.append(('group', prof.group, None))
                last_group = prof.group
            rows.append(('profile', prof.label, prof))
        return rows

    def _profile_rows(self):
        return [r for r in self._launch_rows() if r[0] == 'profile']

    def _selected_profile(self):
        profiles = [r[2] for r in self._profile_rows()]
        if not profiles:
            return None
        self.launch_sel = max(0, min(self.launch_sel, len(profiles) - 1))
        return profiles[self.launch_sel]

    def _selected_row_index(self, rows) -> int:
        prof = self._selected_profile()
        for index, (kind, _text, ref) in enumerate(rows):
            if kind == 'profile' and ref is prof:
                return index
        return 0

    def _job_badge(self, key: str) -> str:
        job = self.jobs.get(key)
        return '' if job is None else f'[{job.status_line()}]'

    def _job_attr(self, job) -> int:
        if job.running:
            return self._color('running')
        if job.error is not None or job.returncode:
            return self._color('bad')
        return self._color('dim')

    def _opt_value(self, prof, option) -> str:
        options = self.selections.get(prof.slug)
        if not isinstance(options, dict):
            return option.default
        return str(options.get(option.key, option.default))

    def _set_opt(self, prof, option, value: str) -> None:
        self.selections.setdefault(prof.slug, {})[option.key] = value
        persistence.save_selections(self.state_file, self.selections)

    # -- actions --------------------------------------------------------------

    def _cycle_option(self, direction: int) -> None:
        prof = self._selected_profile()
        if prof is None or not prof.options:
            return
        self.opt_focus = max(0, min(self.opt_focus, len(prof.options) - 1))
        option = prof.options[self.opt_focus]
        value = self._opt_value(prof, option)
        if option.kind == BOOL:
            self._set_opt(
                prof, option, 'False' if normalize_bool(value) == 'True' else 'True'
            )
        elif option.kind == CHOICE and option.choices:
            current = option.choices.index(value) if value in option.choices else 0
            nxt = (current + direction) % len(option.choices)
            self._set_opt(prof, option, option.choices[nxt])

    def _reset_options(self) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        self.selections.pop(prof.slug, None)
        persistence.save_selections(self.state_file, self.selections)
        self.status = f'Reset options for {prof.label}'

    def _launch_selected(self) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        if prof.disabled:
            self.status = prof.note or 'This entry cannot be launched here.'
            return
        if not pixi_env.install_setup_exists(self.repo_root):
            self.status = 'Workspace not built — run Tasks → Build first.'
            self.active = TAB_TASKS
            return
        existing = self.jobs.get(prof.slug)
        if existing is not None and existing.group_alive:
            self.status = f'{prof.label} already running'
            self._focus_logs(prof.slug)
            return
        holders = self._resource_conflicts(prof.slug, prof.resources)
        if holders:
            self.status = resource_conflict_message(prof.label, holders)
            return
        selections = self.selections.get(prof.slug, {})
        argv = build_launch_command(prof, selections, self.ros_ws)
        job, started = self.jobs.start(
            prof.slug, prof.label, argv, str(self.repo_root), resources=prof.resources
        )
        self.status = self._start_status(job, started, 'Launched')
        self._focus_logs(prof.slug)

    def _run_task(self) -> None:
        if not self.tasks:
            return
        task = self.tasks[self.task_sel]
        argv = build_task_command(task)
        key = 'task:' + task.slug
        job, started = self.jobs.start(key, task.label, argv, str(self.repo_root))
        self.status = self._start_status(job, started, 'Running')
        self._focus_logs(key)

    @staticmethod
    def _start_status(job, started: bool, verb: str) -> str:
        if started:
            return f'{verb} {job.label}'
        if job.error is not None:
            return f'Failed to start {job.label}: {job.error}'
        return f'{job.label} already running'

    def _focus_logs(self, key: str) -> None:
        self.active = TAB_LOGS
        self.log_scroll = 0
        all_jobs = self.jobs.jobs()
        for index, job in enumerate(all_jobs):
            if job.key == key:
                self.log_sel = index

    # -- input ----------------------------------------------------------------

    def _handle_key(self, key: int) -> bool:
        if key in (ord('q'), 27):
            return True
        if key in (ord('1'), ord('2'), ord('3'), ord('4')):
            self.active = key - ord('1')
            return False
        if key == ord('\t'):
            self.active = (self.active + 1) % len(_TAB_TITLES)
            return False
        if key == curses.KEY_BTAB:
            self.active = (self.active - 1) % len(_TAB_TITLES)
            return False
        if self.active == TAB_LAUNCH:
            self._handle_launch_key(key)
        elif self.active == TAB_TASKS:
            self._handle_tasks_key(key)
        elif self.active == TAB_MONITOR:
            self._handle_monitor_key(key)
        else:
            self._handle_logs_key(key)
        return False

    def _handle_launch_key(self, key: int) -> None:
        profiles = self._profile_rows()
        if key == curses.KEY_UP:
            self.launch_sel = max(0, self.launch_sel - 1)
            self.opt_focus = 0
        elif key == curses.KEY_DOWN:
            self.launch_sel = min(len(profiles) - 1, self.launch_sel + 1)
            self.opt_focus = 0
        elif key == curses.KEY_LEFT:
            self._cycle_option(-1)
        elif key in (curses.KEY_RIGHT, ord(' ')):
            self._cycle_option(1)
        elif key in (ord('l'), curses.KEY_ENTER, 10, 13):
            self._launch_selected()
        elif key == ord('r'):
            self._reset_options()

    def _handle_tasks_key(self, key: int) -> None:
        if key == curses.KEY_UP:
            self.task_sel = max(0, self.task_sel - 1)
        elif key == curses.KEY_DOWN:
            self.task_sel = min(len(self.tasks) - 1, self.task_sel + 1)
        elif key in (ord('l'), curses.KEY_ENTER, 10, 13):
            self._run_task()

    def _handle_logs_key(self, key: int) -> None:
        all_jobs = self.jobs.jobs()
        if key == ord('[') and all_jobs:
            self.log_sel = max(0, self.log_sel - 1)
            self.log_scroll = 0
        elif key == ord(']') and all_jobs:
            self.log_sel = min(len(all_jobs) - 1, self.log_sel + 1)
            self.log_scroll = 0
        elif key == curses.KEY_UP:
            self.log_scroll += 1
        elif key == curses.KEY_DOWN:
            self.log_scroll = max(0, self.log_scroll - 1)
        elif key == ord('k') and all_jobs:
            all_jobs[self.log_sel].stop()
            self.status = 'Stopping job…'
        elif key == ord('x'):
            removed = self.jobs.clear_finished()
            self.status = f'Cleared {removed} finished job(s)'


def _resolve_repo_root() -> Path:
    for start in (Path.cwd(), Path(__file__).resolve()):
        root = pixi_env.find_repo_root(start)
        if root is not None:
            return root
    return None


def main() -> int:
    """Console-script entry point for the perseus_tui command."""
    root = _resolve_repo_root()
    if root is None:
        print('perseus_tui: could not find pixi.toml (run from the repo).')
        return 1
    return App(root).run()
