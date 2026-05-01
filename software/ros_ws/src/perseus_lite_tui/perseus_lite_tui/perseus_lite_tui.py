#!/usr/bin/env python3
"""Perseus Lite Roam TUI.

Curses front-end for the ``frontier_explorer`` node that ``perseus-lite-roam``
launches. Lets the operator inspect and adjust the runtime ROS 2 parameters
that drive autonomous exploration without restarting the stack.

Run via ``nix run .#perseus-lite-TUI`` while ``perseus-lite-roam`` is active.
"""

import argparse
import curses
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, List, Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


DEFAULT_TARGET_NODE = "/frontier_explorer"
SERVICE_TIMEOUT_SEC = 2.0
REFRESH_INTERVAL_SEC = 2.0
CURSES_TICK_MS = 150
ROAM_FLAKE_ATTR = ".#perseus-lite-roam"
ROAM_STOP_GRACE_SEC = 5.0


# Editable parameters exposed by the frontier_explorer node, in display order.
# Each tuple is (parameter_name, kind, help_text)
#   kind: "double" | "int" | "string"
@dataclass(frozen=True)
class ParamSpec:
    name: str
    kind: str
    help: str


PARAM_SPECS: List[ParamSpec] = [
    ParamSpec(
        "planning_period_sec",
        "double",
        "Seconds between frontier re-evaluations when idle.",
    ),
    ParamSpec(
        "min_frontier_size_cells",
        "int",
        "Reject frontier clusters smaller than N cells.",
    ),
    ParamSpec(
        "free_threshold", "int", "Occupancy <= this counts as free space (0..100)."
    ),
    ParamSpec(
        "occupied_threshold", "int", "Occupancy >= this counts as obstacle (0..100)."
    ),
    ParamSpec(
        "blacklist_radius_m",
        "double",
        "Skip a frontier within this radius of a failed goal.",
    ),
    ParamSpec(
        "goal_reached_radius_m",
        "double",
        "Treat goal as reached within this distance (m).",
    ),
    ParamSpec(
        "distance_weight",
        "double",
        "Score penalty per metre to a frontier (higher = prefer near).",
    ),
    ParamSpec(
        "size_weight",
        "double",
        "Score bonus per frontier cell (higher = prefer big frontiers).",
    ),
    ParamSpec(
        "goal_timeout_sec",
        "double",
        "Cancel + blacklist a goal if Nav2 has not finished within N s.",
    ),
    ParamSpec(
        "start_delay_sec",
        "double",
        "Initial settle period after the node starts before planning.",
    ),
    ParamSpec("map_topic", "string", "OccupancyGrid topic to subscribe to."),
    ParamSpec("global_frame", "string", "Frame the goals are sent in (typically map)."),
    ParamSpec(
        "robot_frame",
        "string",
        "Frame to look up the robot pose from (typically base_link).",
    ),
]


def _value_from_param_value(v: ParameterValue) -> Optional[Any]:
    """Convert a rcl_interfaces ParameterValue into a python primitive."""
    t = v.type
    if t == ParameterType.PARAMETER_BOOL:
        return bool(v.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(v.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(v.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(v.string_value)
    return None


def _build_param_value(kind: str, raw: str) -> ParameterValue:
    """Build a ParameterValue from a user-typed string. Raises ValueError on bad input."""
    pv = ParameterValue()
    if kind == "int":
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(raw)
    elif kind == "double":
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(raw)
    elif kind == "string":
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(raw)
    else:
        raise ValueError(f"Unsupported kind: {kind}")
    return pv


class ExplorerParamClient(Node):
    """Async client around frontier_explorer's parameter services.

    Spins in a background thread so the curses UI never blocks on rclpy. Callers
    fire ``get_values_async`` / ``set_value_async`` and receive the result via a
    callback that runs on the executor thread.
    """

    def __init__(self, target_node: str):
        super().__init__("perseus_lite_tui")
        target_node = target_node if target_node.startswith("/") else "/" + target_node
        self._target = target_node
        self._get_cli = self.create_client(
            GetParameters, f"{target_node}/get_parameters"
        )
        self._set_cli = self.create_client(
            SetParameters, f"{target_node}/set_parameters"
        )
        self._list_cli = self.create_client(
            ListParameters, f"{target_node}/list_parameters"
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="perseus_lite_tui_spin", daemon=True
        )
        self._spin_thread.start()

    @property
    def target(self) -> str:
        return self._target

    def services_ready(self) -> bool:
        return self._get_cli.service_is_ready() and self._set_cli.service_is_ready()

    def discovery_summary(self) -> str:
        """Short status string about what we currently see on the wire."""
        try:
            names_ns = self.get_node_names_and_namespaces()
        except Exception as e:
            return f"discovery error: {e}"
        total = len(names_ns)
        target_name = self._target.lstrip("/")
        target_visible = any(n == target_name for n, _ns in names_ns)
        return (
            f"{total} nodes visible; "
            f"{self._target} {'FOUND' if target_visible else 'NOT FOUND'}"
        )

    def get_values_async(self, names: List[str], callback) -> bool:
        """Fire a non-blocking get_parameters call.

        ``callback(values_or_None, error_message)`` is invoked on the executor
        thread when the response arrives (or immediately if the service is not
        yet discovered). Returns True if the request was queued.
        """
        if not self._get_cli.service_is_ready():
            callback(None, f"{self._target}/get_parameters not yet discovered")
            return False
        req = GetParameters.Request()
        req.names = names
        future = self._get_cli.call_async(req)

        def _done(fut):
            try:
                res = fut.result()
            except Exception as e:  # pragma: no cover - defensive
                callback(None, f"get_parameters raised: {e}")
                return
            if res is None:
                callback(None, "get_parameters returned no result")
                return
            callback([_value_from_param_value(v) for v in res.values], None)

        future.add_done_callback(_done)
        return True

    def set_value_async(self, name: str, kind: str, raw: str, callback) -> bool:
        """Fire a non-blocking set_parameters call.

        ``callback(ok: bool, message: str)`` runs on the executor thread.
        """
        if not self._set_cli.service_is_ready():
            callback(False, "set_parameters service not yet discovered")
            return False
        try:
            pv = _build_param_value(kind, raw)
        except ValueError as e:
            callback(False, f"bad value: {e}")
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = name
        p.value = pv
        req.parameters = [p]
        future = self._set_cli.call_async(req)

        def _done(fut):
            try:
                res = fut.result()
            except Exception as e:  # pragma: no cover - defensive
                callback(False, f"set_parameters raised: {e}")
                return
            if res is None or not res.results:
                callback(False, "no result")
                return
            r = res.results[0]
            callback(
                bool(r.successful), r.reason or ("ok" if r.successful else "rejected")
            )

        future.add_done_callback(_done)
        return True

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        self._spin_thread.join(timeout=1.0)


class TuiState:
    """Shared state between the ROS thread and the curses UI thread."""

    def __init__(self, specs: List[ParamSpec]):
        self.specs = specs
        self.values: List[Optional[Any]] = [None] * len(specs)
        self.last_refresh: float = 0.0
        self.connected: bool = False
        self.status: str = "Starting..."
        self.lock = threading.Lock()


def _find_flake_dir(start: Optional[str] = None) -> Optional[str]:
    """Walk up from ``start`` (or CWD) looking for a flake.nix."""
    cur = os.path.abspath(start or os.getcwd())
    while True:
        if os.path.isfile(os.path.join(cur, "flake.nix")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


class RoamLauncher:
    """Manages a ``nix run .#perseus-lite-roam`` subprocess on behalf of the TUI."""

    def __init__(self, project_dir: Optional[str], log_path: Optional[str] = None):
        self._project_dir = project_dir
        self._proc: Optional[subprocess.Popen] = None
        self._log_path: Optional[str] = log_path
        self._log_file = None
        self._lock = threading.Lock()

    @property
    def project_dir(self) -> Optional[str]:
        return self._project_dir

    @property
    def log_path(self) -> Optional[str]:
        return self._log_path

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def status_line(self) -> str:
        with self._lock:
            if self._proc is None:
                return "roam: not started"
            rc = self._proc.poll()
            if rc is None:
                return f"roam: running (pid {self._proc.pid}) → {self._log_path}"
            return f"roam: exited rc={rc} (log: {self._log_path})"

    def start(self) -> tuple:
        """Returns (ok, message)."""
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, f"roam already running (pid {self._proc.pid})"
            if shutil.which("nix") is None:
                return False, "nix executable not found in PATH"
            project = self._project_dir or _find_flake_dir()
            if project is None:
                return False, "no flake.nix found — pass --project DIR"
            self._project_dir = project
            ts = time.strftime("%Y%m%d-%H%M%S")
            self._log_path = self._log_path or f"/tmp/perseus-lite-roam-{ts}.log"
            try:
                self._log_file = open(self._log_path, "ab", buffering=0)
            except OSError as e:
                return False, f"could not open log {self._log_path}: {e}"
            try:
                # start_new_session=True puts the child in its own process
                # group so we can signal the whole launch tree on stop.
                self._proc = subprocess.Popen(
                    ["nix", "run", ROAM_FLAKE_ATTR],
                    cwd=project,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except OSError as e:
                self._log_file.close()
                self._log_file = None
                return False, f"failed to spawn nix run: {e}"
            return True, (f"started roam (pid {self._proc.pid}) → {self._log_path}")

    def stop(self) -> tuple:
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                return False, "roam is not running"
            pid = self._proc.pid
            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
            except ProcessLookupError:
                return False, "process already gone"
            except OSError as e:
                return False, f"SIGINT failed: {e}"
            try:
                self._proc.wait(timeout=ROAM_STOP_GRACE_SEC)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                    self._proc.wait(timeout=ROAM_STOP_GRACE_SEC)
                except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                    try:
                        os.killpg(os.getpgid(pid), signal.SIGKILL)
                    except OSError:
                        pass
            rc = self._proc.poll()
            if self._log_file is not None:
                try:
                    self._log_file.close()
                except OSError:
                    pass
                self._log_file = None
            return True, f"roam stopped (rc={rc})"

    def detach(self) -> Optional[int]:
        """Release file handles without killing the child. Returns pid if alive."""
        with self._lock:
            alive_pid: Optional[int] = None
            if self._proc is not None and self._proc.poll() is None:
                alive_pid = self._proc.pid
            if self._log_file is not None:
                try:
                    self._log_file.close()
                except OSError:
                    pass
                self._log_file = None
            return alive_pid


def request_refresh(client: ExplorerParamClient, state: TuiState) -> None:
    """Fire-and-forget refresh. State is updated when the callback runs."""
    names = [s.name for s in state.specs]

    def on_done(values, error):
        with state.lock:
            if error or values is None:
                state.connected = False
                state.status = (
                    f"No reply from {client.target} "
                    f"({error or 'unknown error'}) — {client.discovery_summary()}"
                )
                return
            state.values = values
            state.last_refresh = time.time()
            state.connected = True
            state.status = f"Connected to {client.target}"

    queued = client.get_values_async(names, on_done)
    if not queued:
        with state.lock:
            state.connected = False
            state.status = (
                f"Waiting for {client.target}/get_parameters "
                f"— {client.discovery_summary()}"
            )


def fmt_value(v: Any, kind: str) -> str:
    if v is None:
        return "—"
    if kind == "double":
        return f"{float(v):.4g}"
    return str(v)


# =============================================================================
# Curses UI
# =============================================================================


def _safe_addstr(win, y: int, x: int, text: str, attr: int = 0) -> None:
    try:
        win.addstr(y, x, text, attr)
    except curses.error:
        pass


def _prompt_value(
    stdscr, y: int, x: int, prompt: str, initial: str = ""
) -> Optional[str]:
    """Inline single-line text prompt. Returns None on Esc, "" allowed otherwise."""
    curses.curs_set(1)
    try:
        buf = list(initial)
        while True:
            line = prompt + "".join(buf)
            _safe_addstr(stdscr, y, x, " " * (curses.COLS - x - 1))
            _safe_addstr(stdscr, y, x, line[: curses.COLS - x - 1], curses.A_REVERSE)
            stdscr.move(y, x + len(line))
            stdscr.refresh()
            ch = stdscr.getch()
            if ch in (curses.KEY_ENTER, 10, 13):
                return "".join(buf)
            if ch == 27:  # Esc
                return None
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                if buf:
                    buf.pop()
                continue
            if 32 <= ch < 127:
                buf.append(chr(ch))
    finally:
        curses.curs_set(0)


def _draw(stdscr, state: TuiState, selected: int, launcher: "RoamLauncher") -> None:
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    title = " Perseus Lite Roam — Parameter TUI "
    _safe_addstr(
        stdscr,
        0,
        max(0, (w - len(title)) // 2),
        title,
        curses.A_BOLD | curses.A_REVERSE,
    )

    with state.lock:
        connected = state.connected
        values = list(state.values)
        status = state.status
        last = state.last_refresh

    conn_str = "● connected" if connected else "○ waiting"
    conn_attr = curses.color_pair(1) if connected else curses.color_pair(2)
    _safe_addstr(stdscr, 1, 1, conn_str, conn_attr | curses.A_BOLD)
    if last:
        _safe_addstr(
            stdscr,
            1,
            16,
            f"refreshed {time.strftime('%H:%M:%S', time.localtime(last))}",
        )

    roam_running = launcher.is_running()
    roam_attr = curses.color_pair(1) if roam_running else curses.color_pair(3)
    _safe_addstr(
        stdscr, 2, 1, launcher.status_line()[: w - 2], roam_attr | curses.A_BOLD
    )

    table_top = 4
    name_w = max(len(s.name) for s in state.specs) + 2
    val_w = 18
    _safe_addstr(stdscr, table_top, 1, "Parameter".ljust(name_w), curses.A_UNDERLINE)
    _safe_addstr(
        stdscr, table_top, 1 + name_w, "Value".ljust(val_w), curses.A_UNDERLINE
    )
    _safe_addstr(
        stdscr, table_top, 1 + name_w + val_w, "Description", curses.A_UNDERLINE
    )

    for i, spec in enumerate(state.specs):
        row = table_top + 1 + i
        if row >= h - 3:
            break
        attr = curses.A_REVERSE if i == selected else 0
        line = " " * (w - 2)
        _safe_addstr(stdscr, row, 1, line, attr)
        _safe_addstr(stdscr, row, 1, spec.name.ljust(name_w), attr)
        _safe_addstr(
            stdscr,
            row,
            1 + name_w,
            fmt_value(values[i], spec.kind).ljust(val_w),
            attr | curses.A_BOLD,
        )
        desc_x = 1 + name_w + val_w
        _safe_addstr(stdscr, row, desc_x, spec.help[: max(0, w - desc_x - 1)], attr)

    help_y = h - 2
    _safe_addstr(
        stdscr,
        help_y,
        1,
        "↑/↓ select  •  Enter edit  •  r refresh  •  l launch roam  •  x stop roam  •  q quit",
        curses.A_DIM,
    )
    status_y = h - 1
    _safe_addstr(stdscr, status_y, 1, status[: w - 2], curses.A_DIM)
    stdscr.refresh()


def run_tui(
    stdscr, client: ExplorerParamClient, state: TuiState, launcher: "RoamLauncher"
) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(CURSES_TICK_MS)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_RED, -1)

    selected = 0
    last_auto_refresh = 0.0

    while True:
        now = time.time()
        if now - last_auto_refresh > REFRESH_INTERVAL_SEC:
            request_refresh(client, state)
            last_auto_refresh = now

        _draw(stdscr, state, selected, launcher)

        ch = stdscr.getch()
        if ch == -1:
            continue
        if ch in (ord("q"), ord("Q")):
            return
        if ch in (curses.KEY_UP, ord("k")):
            selected = (selected - 1) % len(state.specs)
        elif ch in (curses.KEY_DOWN, ord("j")):
            selected = (selected + 1) % len(state.specs)
        elif ch in (ord("r"), ord("R")):
            with state.lock:
                state.status = "Refreshing..."
            request_refresh(client, state)
            last_auto_refresh = time.time()
        elif ch in (ord("l"), ord("L")):
            ok, msg = launcher.start()
            with state.lock:
                state.status = msg if ok else f"launch failed: {msg}"
        elif ch in (ord("x"), ord("X")):
            ok, msg = launcher.stop()
            with state.lock:
                state.status = msg
        elif ch in (curses.KEY_ENTER, 10, 13):
            spec = state.specs[selected]
            with state.lock:
                current = fmt_value(state.values[selected], spec.kind)
            prompt = f" set {spec.name} ({spec.kind}) = "
            entered = _prompt_value(
                stdscr,
                curses.LINES - 1,
                0,
                prompt,
                initial=current if current != "—" else "",
            )
            if entered is None or entered.strip() == "":
                with state.lock:
                    state.status = "Edit cancelled."
                continue
            value = entered.strip()
            with state.lock:
                state.status = f"setting {spec.name} = {value}..."

            def on_set(ok, msg, name=spec.name, value=value):
                with state.lock:
                    state.status = (
                        f"set {name} = {value} → {msg}"
                        if ok
                        else f"FAILED set {name}: {msg}"
                    )
                if ok:
                    request_refresh(client, state)

            client.set_value_async(spec.name, spec.kind, value, on_set)


# =============================================================================
# Entrypoint
# =============================================================================


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="perseus_lite_tui",
        description="TUI to tune the Perseus Lite roam (frontier_explorer) node.",
    )
    p.add_argument(
        "--node",
        default=DEFAULT_TARGET_NODE,
        help=f"Fully-qualified target node name (default: {DEFAULT_TARGET_NODE})",
    )
    p.add_argument(
        "--project",
        default=None,
        help="Path to the perseus-v2 flake (defaults to walking up "
        "from CWD until flake.nix is found).",
    )
    p.add_argument(
        "--roam-log",
        default=None,
        help="Override path for the roam stdout/stderr log file.",
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])

    if not sys.stdout.isatty():
        print(
            "perseus_lite_tui: requires a TTY (interactive terminal).", file=sys.stderr
        )
        return 1

    rclpy.init()
    client = ExplorerParamClient(args.node)
    state = TuiState(PARAM_SPECS)
    launcher = RoamLauncher(project_dir=args.project, log_path=args.roam_log)
    alive_pid: Optional[int] = None
    try:
        # Kick off an initial fetch — the executor thread is already spinning,
        # so this returns immediately and the result lands when discovered.
        request_refresh(client, state)
        # Workaround for some terminals that don't expose ESC properly.
        os.environ.setdefault("ESCDELAY", "25")
        curses.wrapper(run_tui, client, state, launcher)
    except KeyboardInterrupt:
        pass
    finally:
        alive_pid = launcher.detach()
        client.shutdown()
        client.destroy_node()
        rclpy.shutdown()
    if alive_pid is not None:
        print(
            f"perseus_lite_tui: roam still running (pid {alive_pid}, log "
            f"{launcher.log_path}). Use `kill -INT {alive_pid}` to stop it.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
