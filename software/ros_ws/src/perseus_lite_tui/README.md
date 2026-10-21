# perseus_lite_tui

A keyboard-driven `curses` mission-control TUI for the perseus-lite software
stack: launch bringup / SLAM+Nav2 / teleop / simulation, run developer tasks,
watch live topic rates, and tail per-job logs — from one screen, over SSH,
without a GUI.

## Run

```console
pixi run tui
# or, inside `pixi shell`:
ros2 run perseus_lite_tui perseus_tui
```

See `docs/source/home/perseus-lite-operation/mission-control-tui.md` for the tab
and key reference.

## Design

Standard-library `curses` only; `rclpy` is a soft dependency (the Monitor tab
self-disables if ROS isn't available). The code splits into a pure, testable
core and a thin shell:

- **Pure** (`registry`, `config`, `pixi_env`, `topic_stats`, `layout`,
  `persistence`, `jobs`) — no `curses`, no `rclpy` at import. An import-hygiene
  test enforces this.
- **Thin shell** — `app.py` (the only `curses` importer) and `ros_monitor.py`
  (the only `rclpy` importer, lazily).

Launches and tasks run through `pixi run -e <env>` so the right Pixi environment
activates first; simulation/mission profiles therefore work from the default-env
TUI. Each job is its own process group, stopped with SIGINT→SIGTERM→SIGKILL
escalation that keeps going until the whole group is gone (not just the
`pixi run` leader). Quitting, SIGHUP (terminal/SSH closed) and SIGTERM all run
the same orderly stop and wait (bounded, ~10 s) for it to finish. Live job
process groups are recorded in `$XDG_CONFIG_HOME/perseus_lite_tui/running_jobs.json`
so a relaunch warns about (and refuses to double-start over) orphans.

Profiles declare exclusive resources (`drive` = controller_manager + ST3215
bus or the simulated equivalent, `robot_state_publisher`, `slam`, `nav2`,
`joystick`); a launch whose resources overlap a live job is refused with a
status-line message — e.g. "SLAM + Nav2" while "Robot bringup" runs. A failed
spawn (e.g. missing executable) is shown in the status line and Logs tab and
never affects other running jobs.

## Test

```console
pixi run -e default test        # includes this package
# or directly:
cd software/ros_ws && colcon test --packages-select perseus_lite_tui
```
