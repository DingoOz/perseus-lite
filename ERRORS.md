# Error Log

Defects found and fixed in this codebase, recorded per the error-tracking
workflow. Review the **Prevention rules** before modifying the listed files.

### CI cppcheck failed on the vendored rplidar_ros sources — 2026-10-21

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `.github/workflows/all.yaml`, `perseus_lite_hardware/tests/st3215_system_test.cpp`
- **Pattern:** The PR's "Static analysis" job failed (`--error-exitcode=1`). All 14 findings were in `software/ros_ws/src/rplidar_ros/` (including Win32 HAL files that never build here). Local `colcon test` was green, because the package doesn't run ament_cppcheck.
- **Root cause:** The CI job runs plain `cppcheck` over all of `software/ros_ws/src`. Vendoring `rplidar_ros` was excluded from treefmt but not from this job. Separately, the newer cppcheck 2.21 flags an uninitialised `RegisterWrite::reg` in the fake-servo-bus test; CI's 2.13 doesn't.
- **Fix applied:** Added `-i software/ros_ws/src/rplidar_ros` to the CI cppcheck command. Gave `RegisterWrite::reg` a default initializer. A local run of the CI command (cppcheck 2.21) now exits 0.
- **Prevention rule:** When vendoring third-party code, exclude it from every repo-wide check: treefmt, the CI cppcheck path list, and any other linter that walks a whole directory. Run the CI cppcheck command locally (`pixi run -e default cppcheck ...`, copied from `all.yaml`) before pushing.

### Robot env could not spawn any controller (`filelock` missing) and lacked twist_mux/rosbridge — 2026-10-21

- **Severity:** Critical
- **Category:** Configuration
- **File(s):** `pixi.toml`, `pixi.lock`
- **Pattern:** `ros2 launch perseus_lite perseus_lite.launch.py` in the `default` env: both `spawner` processes die with `ModuleNotFoundError: No module named 'filelock'`, `ros2 control list_controllers` is empty and the robot never drives. The bringup's `twist_mux` and `rosbridge_server` were "package not found" in the same env.
- **Root cause:** RoboStack's `ros-jazzy-controller-manager` imports `filelock` without depending on it. `twist_mux` and `rosbridge-suite` were declared only under `[feature.simulation.dependencies]`, so they existed in sim (where launches were tried) but not in the robot env. Nothing ever launched the hardware bringup in `default`.
- **Fix applied:** Added `filelock`, `ros-jazzy-twist-mux`, `ros-jazzy-rosbridge-suite` and `cbor2` to the default `[dependencies]` (all exist for linux-aarch64). Verified with a mock-hardware bringup: both controllers active and joy → twist_mux → `/cmd_vel` → wheels → odom → EKF TF.
- **Prevention rule:** Any package a hardware launch file starts must be a default-env dependency, not a feature one. Before merging bringup changes, run `pixi run -e default bringup-mock` and confirm `ros2 control list_controllers` shows both controllers `active`. A green `colcon build` does not prove a launch can start.

### twist_mux output never reached the controller; docs told users to bypass it — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `perseus_lite/launch/{perseus_lite,twist_mux}.launch.py`, `perseus_lite/config/twist_mux.yaml`, `perseus_lite_simulation/launch/perseus_sim.launch.py`, `docs/source/home/perseus-lite-operation/basic-operation.md`
- **Pattern:** In sim, keyboard, web and Nav2 commands went nowhere. On hardware only one source worked at a time, chosen by `cmd_vel_topic:=/joy_vel`.
- **Root cause:** twist_mux publishes `cmd_vel_out`, which was never remapped to `/cmd_vel` (the controller input). The hardware bringup didn't start twist_mux at all, and the docs worked around that by pointing the controller straight at `/joy_vel`.
- **Fix applied:** One shared `perseus_lite/launch/twist_mux.launch.py` (joy 100 > key 90 > web 80 > nav 10, output remapped to `/cmd_vel`), used by both the hardware bringup and sim. The docs no longer pass `cmd_vel_topic`.
- **Prevention rule:** Exactly one node, twist_mux, publishes `/cmd_vel`; check with `ros2 topic info /cmd_vel -v`. Never remap the controller onto a single input topic. A high-priority input must stop publishing when idle (deadman), or it starves every lower one.

### Multiple publishers for the same TF edge (controller + EKF, AMCL + SLAM, static map→odom) — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `perseus_lite/config/perseus_lite_controllers.yaml`, `perseus_lite/launch/{nav2,perseus_lite_slam_and_nav2}.launch.py`, `autonomy/launch/mapping_using_slam_toolbox.launch.py`, `perseus_lite_missions/launch/mission_zero.launch.py`
- **Pattern:** The robot pose jumps or jitters in RViz and Nav2. The map drifts against the robot.
- **Root cause:** `enable_odom_tf: true` on the controller while an EKF also published odom→base_link. AMCL ran alongside slam_toolbox, and static map→odom publishers stood in for a SLAM node that never activated.
- **Fix applied:** TF contract: map→odom comes only from slam_toolbox (or AMCL in `use_localization` mode, never both), and odom→base_link only from the EKF (`perseus_lite/config/ekf.yaml`), which runs on hardware and in sim. The controller sets `enable_odom_tf: false`. Static map→odom publishers are removed.
- **Prevention rule:** Before adding any TF broadcaster, check who already owns that edge (`ros2 run tf2_ros tf2_monitor <parent> <child>`). Never paper over a missing transform with a static publisher.

### Lifecycle node (slam_toolbox) started as a plain `Node` — never mapped — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `autonomy/launch/mapping_using_slam_toolbox.launch.py`, `perseus_lite_missions` orchestrator
- **Pattern:** slam_toolbox runs but `/map` never appears, and a static map→odom hides the problem.
- **Root cause:** slam_toolbox ≥ 2.8 is a lifecycle node. Started as `Node` or with `ros2 run`, it stays unconfigured.
- **Fix applied:** Every SLAM start goes through `autonomy/launch/online_async_launch.py` (a LifecycleNode with configure/activate events).
- **Prevention rule:** Start lifecycle nodes with a LifecycleNode plus transition events or a lifecycle manager, and check with `ros2 lifecycle get /<node>`.

### Left-side wheel feedback not inverted (commands were) — odometry turned instead of driving straight — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** Driving straight produced wheel odometry that spun in place.
- **Root cause:** `apply_motor_direction` negated the commanded velocity for the inverted left-side servos (IDs 2, 3), but `read()` passed their feedback through unchanged.
- **Fix applied:** read() applies the same per-ID inversion to velocity and position, and a fake-servo-bus test covers both directions.
- **Prevention rule:** Any per-actuator sign or scale correction must be applied symmetrically to commands and state, with a test for each side.

### Write ACKs counted as servo feedback; stale data never detected — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** A servo that stopped answering status reads still looked healthy, with its last position and velocity frozen.
- **Root cause:** At status return level 1 every WRITE is ACKed, and any valid packet from a servo refreshed its "last update" time. The staleness check also mixed ROS clock types.
- **Fix applied:** Only a full status payload refreshes the timestamp. Staleness uses `steady_clock` (1 s). Input is flushed before each READ.
- **Prevention rule:** Liveness must come from the response type that carries the data, never from any packet. Use a monotonic clock for timeouts in hardware interfaces.

### Servos kept running after a fault or shutdown — 2026-10-21

- **Severity:** Critical
- **Category:** Logic Error
- **File(s):** `perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** After a controller crash, `on_error`, or Ctrl+C, the wheels kept turning at the last commanded speed.
- **Root cause:** ST3215s have no comms watchdog. on_error and on_shutdown weren't implemented, and cleanup didn't zero GOAL_SPEED.
- **Fix applied:** on_cleanup, on_shutdown and on_error send zero speed plus torque-off, then close the port. configure zeroes the speed before enabling torque.
- **Prevention rule:** For actuators without a watchdog, every lifecycle exit path must command zero, and a test must exercise it.

### Left wheel joint axis rolled the wheels backwards — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `perseus_lite_description/urdf/wheel.urdf.xacro`
- **Pattern:** In sim and RViz the left wheels spun the wrong way for forward motion.
- **Root cause:** The left motor link is yawed by π, but the joint axis was `0 0 -1` on both sides.
- **Fix applied:** Left axis `0 0 1`. A forward-kinematics check confirms all four axes are +Y in base_link.
- **Prevention rule:** After editing a wheel or joint origin, compute each wheel axis in base_link (FK at zero angles). diff_drive assumes +Y for every wheel.

### Robot-validated fixes stranded on an unmerged branch — 2026-10-21

- **Severity:** High
- **Category:** Configuration
- **File(s):** `perseus_lite*`, `autonomy/config/*`
- **Pattern:** main carried the pre-calibration drive (wheel_separation 0.40), the old servo scaling and the wrong lidar mount, while `origin/feat/perseus-lite` had the versions tuned on the robot.
- **Root cause:** Hardware work continued on a long-lived branch that was never merged after main was restructured.
- **Fix applied:** Ported the validated driver, calibration, EKF, SLAM and Nav2 tuning, and the lidar mount and range filter to main.
- **Prevention rule:** Before changing lite drive or nav config, run `git log --oneline main..origin/feat/perseus-lite -- <path>`. Merge hardware-validated changes to main promptly instead of keeping them on a side branch.

### RoboStack `rplidar_ros` 2.1.0 cannot drive the RPLidar C1 — 2026-10-21

- **Severity:** High
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/rplidar_ros/`, `pixi.toml`
- **Pattern:** The bringup includes `rplidar_c1_launch.py`, which doesn't exist in the installed package.
- **Root cause:** The Nix setup had built the Slamtec `ros2` branch from source, which has C1 support. The Pixi migration swapped in RoboStack's 2.1.0, which predates C1.
- **Fix applied:** Vendored upstream `ros2` @ 24cc9b6 in-tree (see `VENDORED.md`) and removed the conda package. The bringup starts `rplidar_node` directly.
- **Prevention rule:** When replacing a source-built (patched or branch) package with a distro binary during a toolchain migration, compare versions and check the specific hardware support you rely on.

### Collision-monitor zones placed inside the lidar's blind radius — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `autonomy/config/perseus_nav_params.yaml`, mission nav2 configs
- **Pattern:** The "stop" polygon could never trigger.
- **Root cause:** `/scan_filtered` drops everything within 0.5 m of the lidar (the robot's own rockers), and the whole stop zone was inside that radius.
- **Fix applied:** Protection now comes from FootprintApproach plus a slowdown zone that reaches past the blind radius.
- **Prevention rule:** Any safety zone must contain points the filtered sensor can actually report. Check it against `min_range` and the lidar's position in base_link.

### Waypoint BT reported SUCCESS when a waypoint failed — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `autonomy/behavior_trees/navigate_through_poses_sequential{,_v2}.xml`
- **Pattern:** NavigateThroughPoses "succeeded" after silently dropping unreachable waypoints. v1 didn't load at all (empty `<Sequence/>`).
- **Root cause:** The loop's FAILURE exit couldn't tell "queue empty" from "waypoint failed", and Fallback/AlwaysSuccess turned both into SUCCESS.
- **Fix applied:** ForceSuccess(loop) followed by a new `IsGoalQueueEmpty` condition, with tests using mocked Nav2 leaves.
- **Prevention rule:** A BT loop that ends on FAILURE must check afterwards why it ended, and the failure path needs a test.

### Async action callbacks clobbered newer goal state (autonomy bridge) — 2026-10-21

- **Severity:** Medium
- **Category:** Race Condition
- **File(s):** `perseus_autonomy_bridge`
- **Pattern:** Cancel sometimes did nothing. A late result from an old goal cleared a new one.
- **Root cause:** Goal-response and result callbacks changed shared state without checking which request they belonged to.
- **Fix applied:** A `GoalTracker` with one token per send. Callbacks change state only for the current token, and a cancel that arrives while a goal is pending is applied as soon as Nav2 accepts it.
- **Prevention rule:** Every async callback must capture its request's identity and drop itself if it is no longer current.

### IMU published zero-filled samples on I2C read failure — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `perseus_sensors/i2c_imu_driver/src/i2c_imu_node.cpp`
- **Pattern:** On a failing bus, the EKF and bias estimator received zero accel and gyro readings with valid timestamps.
- **Root cause:** Failed reads left default values in place and were published anyway.
- **Fix applied:** Reads return `std::optional`. Nothing is published on failure, and the device is re-initialised after repeated failures.
- **Prevention rule:** A failed sensor read must never reach a publisher. Return optional or throw.

### Gyro-bias estimator counted disabled sources as "stationary" — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `perseus_sensors/src/bias_remover/bias_estimator_component.cpp`
- **Pattern:** Bias was learned while turning.
- **Root cause:** A disabled or stale motion source voted "ok".
- **Fix applied:** Only enabled sources vote. Stale data means "not stationary", and with no enabled source there is no estimate.
- **Prevention rule:** In gating logic, a missing or disabled input means "unknown", never "ok".

### Socket connect fallback reused the first address — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `software/shared/simple-networking/src/client.cpp`
- **Pattern:** Connect failed with EAFNOSUPPORT when the first resolved address was unusable. Large sends were truncated, and a peer reset killed the process with SIGPIPE.
- **Root cause:** The getaddrinfo loop ignored its loop variable. send() had no partial-write loop and no MSG_NOSIGNAL.
- **Fix applied:** Use the current entry. Send loops over partial writes with MSG_NOSIGNAL and EINTR retry.
- **Prevention rule:** Inside a getaddrinfo loop, use only the current entry. Always handle partial sends and SIGPIPE.

### Arm teleop: refactor deleted the main loop; blocking reads; signal-handler I/O; wrong load decode — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `software/arm-teleop-direct/`
- **Pattern:** The program did nothing (main loop body gone since 3a05a33d). A missing servo hung both the program and Ctrl+C. Small negative loads tripped torque protection.
- **Root cause:** A mechanical refactor dropped the loop body and still compiled. Reads used VMIN=1 with no deadline. The signal handler wrote to serial. Load was decoded as two's complement, but Feetech uses sign-magnitude on bit 10.
- **Fix applied:** Restored the loop. Reads use async_read with a deadline. The handler only sets an atomic flag. Sign-magnitude decoding is unit-tested.
- **Prevention rule:** In refactors, review the whole diff for function bodies that shrank. Serial reads need deadlines. Signal handlers only set a lock-free flag. Test register decoding with the sign bit set.

### Sourced activation script changed the caller's shell options — 2026-10-21

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/scripts/pixi-env-sanitize.sh`
- **Pattern:** After `pixi shell` or direnv, shell completion and `source install/setup.bash` broke on unset variables.
- **Root cause:** `set -uo pipefail` in a script that is sourced (Pixi activation) leaked into the interactive shell. `pixi run` was unaffected, which hid it.
- **Fix applied:** Removed the `set` line and made the body nounset-safe.
- **Prevention rule:** Scripts that get sourced must never change shell options. Test them with `pixi shell-hook`/direnv, not only `pixi run`.

### Time-sync script set the clock from a failed fetch, then rebooted — 2026-10-21

- **Severity:** High
- **Category:** Logic Error
- **File(s):** `software/daemons/`
- **Pattern:** With no network, `date -s "$(curl …)"` set the clock to midnight and the unit reported success.
- **Root cause:** External input wasn't validated before acting on the system, and every failure was swallowed.
- **Fix applied:** `timesync-google.sh` validates the HTTPS Date header, refuses large backward steps, exits non-zero on failure, and never reboots.
- **Prevention rule:** Validate external data before acting on the system, and fail loudly (non-zero).

### Web UI: joystick kept driving after blur; gst-launch pipeline injection — 2026-10-21

- **Severity:** Critical
- **Category:** Security
- **File(s):** `software/web_ui/src/…/teleop`, `software/web_ui/pi-server/`
- **Pattern:** Switching tabs while driving left the rover moving. Camera request fields could add GStreamer elements, and a domain endpoint shelled out with user input.
- **Root cause:** Separate lock and active flags, with stop returning early when locked. gst-launch re-parses its arguments, so `!` or spaces inside one argument add elements.
- **Fix applied:** One `lock()` that always sends a zero twist on every exit path. Every client value is whitelisted before it reaches gst-launch. The shell endpoint is removed.
- **Prevention rule:** Every teleop exit path (blur, hidden, cancel, unmount) must send a zero twist. Whitelist all client input before it reaches a process spawn, even without a shell.

### Web UI: `$effect` subscriptions piled up; dead roslib 1 API; non-bindable prop — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `software/web_ui/src/lib/widgets/*`
- **Pattern:** Duplicate topic subscriptions grew over time. Start/Stop Autonomy threw at runtime. The custom ROS host was never saved.
- **Root cause:** Effects didn't return a cleanup. `ROSLIB.ServiceRequest` doesn't exist in roslib 2. `bind:value` on a non-`$bindable` prop only edits a local copy.
- **Fix applied:** Effects return cleanups, service calls pass plain objects, and the prop is `$bindable()`.
- **Prevention rule:** Every `$effect` that subscribes returns an unsubscribe. Treat roslib type errors as real bugs. Shared state needs `$bindable()`.

### TUI orphaned robot process groups on terminal close — 2026-10-21

- **Severity:** High
- **Category:** Resource Leak
- **File(s):** `perseus_lite_tui/`
- **Pattern:** Closing the terminal left the bringup (and the wheels' last command source) running.
- **Root cause:** Jobs run in new sessions, with no SIGHUP/SIGTERM handler, daemon escalator threads, and escalation that stopped at the `pixi run` leader.
- **Fix applied:** Signal handlers, a bounded SIGINT→SIGTERM→SIGKILL wait on the whole group, and a running-jobs registry that flags orphans.
- **Prevention rule:** Whoever detaches a process group owns its full stop path. Signal the group, never the leader alone, and never `pkill -f`.

### Bare `LaunchConfiguration` in `Node(parameters=[…])` silently ignored — 2026-10-21

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `perseus_input/launch/controller.launch.py`
- **Pattern:** A launch argument such as `timeout_enable` never reached the node.
- **Root cause:** launch_ros treats a bare substitution in `parameters` as a params-file path, and typed params need `ParameterValue`.
- **Fix applied:** `{"name": ParameterValue(sub, value_type=…)}`.
- **Prevention rule:** Pass launch args as `{"param": ParameterValue(sub, value_type=T)}` and check them with `ros2 param get` after launching.

### CI hid -Werror and ran no lint tests — 2026-10-21

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/colcon_defaults.yaml`, `pixi.toml`, `.github/workflows/*`
- **Pattern:** Warnings that fail Release builds passed CI. Lint-only packages "passed" with zero tests. Main-branch runs got cancelled.
- **Root cause:** The default build type was Debug, so the `Rel.*` -Werror never applied. `--ctest-args -R 'Test\.'` excluded every ament_lint test. A shared concurrency group cancelled main runs. yamlfmt mangled folded `if: >-` expressions.
- **Fix applied:** RelWithDebInfo by default. The test task excludes only the formatter-duplicating linters by label. Concurrency groups are per-ref only for PRs (pushes to main use the run id), and the Pages condition is kept on a single line so yamlfmt can't fold it.
- **Prevention rule:** Make CI build the same type the robot runs. After changing a test filter, check that the per-package test counts are non-zero.

### Mission checkers crashed on malformed live edits; `use_sim_time` re-declared — 2026-10-21

- **Severity:** Medium
- **Category:** Logic Error
- **File(s):** `perseus_lite_missions/`
- **Pattern:** A typo in an edited YAML killed the checker node. Nodes crashed at startup with ParameterAlreadyDeclared.
- **Root cause:** Only YAMLError was caught, so TypeError, AttributeError and OSError escaped the timers. rclpy already declares `use_sim_time`.
- **Fix applied:** Validation moved to a pure function that returns a status, and timers never raise. The re-declare is removed.
- **Prevention rule:** Never declare `use_sim_time`. Timer callbacks that read user-edited files must not raise.

### ament_python tests silently skipped under `colcon test` in the RoboStack env — 2026-07-07

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_tui/pytest.ini`, `software/ros_ws/src/perseus_lite_tui/setup.py`, `software/ros_ws/src/perseus_lite_tui/ruff.toml`
- **Pattern:** A new `ament_python` package's `pytest` suite does not actually run under `colcon test` in the Pixi/RoboStack Jazzy env, for two independent, compounding reasons that both fail _quietly_ (green build, no obvious error). (1) The env ships `launch_testing` / `launch_ros` pytest plugins whose hookimpls are incompatible with the env's pytest 9 (they declare a `path` arg / a `pytest_launch_collect_makemodule` hook that pytest 9 removed from the hookspec); loading either aborts **every** pytest invocation in the env with `PluginValidationError` before collection. (2) colcon's pytest test-step only _matches_ a package when its `setup.py` metadata declares a `test` dependency (`has_test_dependency` reads `tests_require` **or** `extras_require['test'/'tests'/'testing']`); with neither, colcon silently falls back to the `setup.py`/unittest runner, which finds nothing and exits 5 "NO TESTS RAN". Separately, `ruff format` (the repo treefmt formatter, double-quote default) and `ament_flake8` (enforces flake8-quotes Q000, single quotes) fight over every string literal in the package.
- **Root cause:** The env's pytest was upgraded past the pinned launch_testing plugins' hookspec; and the repo's own `tests_require=` prohibition (2026-07-03 entry) removes the very metadata colcon uses to pick its pytest runner, so "no `tests_require`" + "nothing else declaring test deps" = unittest fallback.
- **Fix applied:** Per-package `pytest.ini` with `addopts = -p no:launch_testing -p no:launch_ros` (these tests don't need either plugin), which colcon's `python -m pytest` subprocess honours via rootdir discovery. Declared the test dep as `extras_require={'test': ['pytest']}` (the non-deprecated form, so colcon selects its pytest step) rather than `tests_require=`. Added a package-local `ruff.toml` with `[format] quote-style = "single"` so the formatter and flake8 agree (docstrings stay triple-double, which flake8-quotes also wants). Verified: `colcon test --packages-select perseus_lite_tui` runs 45 pytest tests (incl. the ament lint trio) green, with `configfile: pytest.ini` and neither launch plugin loaded.
- **Prevention rule:** For a new `ament_python` package that should be tested in CI, don't trust a green `colcon test` summary — confirm the pytest count is non-zero (`colcon test-result --verbose`, or read `build/<pkg>/pytest.xml`). Declare the pytest dep via `extras_require['test']` (never `tests_require=`), and if you add pytest lint/marker tests in this env, disable the launch_testing/launch_ros plugins in the package's `pytest.ini`. When adding Python to this repo, add a package-local `ruff.toml` pinning single quotes if the package's tests run `ament_flake8`.

### Committed docker `.env` silently swallowed by the global `.gitignore` — 2026-07-07

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `.gitignore`, `software/docker/isaac-ros/.env`, `.github/workflows/all.yaml`
- **Pattern:** A config file that is meant to be committed but whose name matches a broad `.gitignore` rule (here `.env`, ignored globally) is silently skipped by `git add <dir>` — no error, and it stays present on the author's disk. Every local check that reads it passes, but a fresh checkout (CI, another clone, the robot) doesn't have it. Here `docker compose config` passed locally and failed **only in CI** with `The "ISAAC_ROS_IMAGE" variable is not set` warnings → `service "bridge-jazzy" has neither an image nor a build context specified`, because docker compose loads `.env` from the project directory and the file simply wasn't in the CI checkout.
- **Root cause:** The repo's `.gitignore` has a generic `.env` rule (Python-template origin); the new committed-on-purpose defaults file matched it, and `git add software/docker/` skipped it without complaint.
- **Fix applied:** Added a negation `!software/docker/isaac-ros/.env` immediately after the `.env` rule so this specific non-secret file (image pins, ROS domains, camera device) is tracked, and committed it. Verified `git ls-files` now lists it and `docker compose -f .../compose.yaml config` resolves all variables from repo root.
- **Prevention rule:** After adding a dotfile/config that must ship, confirm it is actually tracked (`git ls-files <path>` returns it, or `git status --ignored` shows it under "Ignored" if not) — a green local run of a tool that reads the file is NOT evidence it was committed. Watch specifically for `.env`, `.envrc`, `*.local`, and other names covered by broad ignore rules.

### Sphinx doctree cache published to Pages after switching to plain-builder invocation — 2026-07-07

- **Severity:** Low
- **Category:** Configuration
- **File(s):** `pixi.toml`
- **Pattern:** The Pixi `docs` task invoked Sphinx as `sphinx-build -b html source build/html` (plain builder mode) instead of the `-M html source build` "make mode" the old Nix build effectively used. In plain-builder mode, without an explicit `-d`, Sphinx writes its doctree cache to `<outdir>/.doctrees` — i.e. _inside_ `build/html` — so the Pages deploy step (`cp -RL docs/build/html/. _site/`) swept ~14 MB of build cache (incl. a ~6 MB `environment.pickle`) into the published site on every deploy. Harmless to page rendering but bloats the artifact and leaks internal build state.
- **Root cause:** Porting the docs build to a hand-written `sphinx-build` command line dropped make-mode's implicit doctree/output separation, and no one checked the contents of the published output tree.
- **Fix applied:** Added `-d build/doctrees` to the `docs` task command so the doctree cache lands in a sibling directory outside `build/html`. Verified: a clean `pixi run -e docs docs` produces `build/html` with no `.doctrees`/`*.pickle` and the cache in `build/doctrees` (14 MB) instead.
- **Prevention rule:** When a build product is published/deployed by copying an output directory wholesale (`cp -RL <dir>`), verify the contents of that directory — build tools frequently drop caches/intermediates (`.doctrees`, `.cache`, sourcemaps) alongside the real output. Prefer the tool's mode that separates intermediates (Sphinx `-M`/`-d`) over a bare builder invocation.

### cppcheck static-analysis findings went unnoticed because the job was advisory-only — 2026-07-06

- **Severity:** Low
- **Category:** Convention
- **File(s):** `.github/workflows/all.yaml`, `software/ros_ws/src/perseus_sensors/src/bias_remover/bias_estimator_component.cpp`, `software/ros_ws/src/perseus_sensors/src/bias_remover/bias_remover_component.cpp`, `software/ros_ws/src/perseus_vision/src/aruco_detector.cpp`, `software/ros_ws/src/perseus_autonomy_bridge/src/nav2_waypoints_bridge.cpp`
- **Pattern:** A `continue-on-error: true` CI job (`static-analysis`, ported from the old Nix CI's code-review-fixes PR) ran cppcheck on every push/PR but its findings never blocked a merge, so it silently stayed red across at least two PRs (the Pixi migration merge and whatever introduced the underlying issues) without anyone noticing. Findings: 8× `useInitializationList` (string parameters assigned in constructor bodies instead of the member-init list, in `BiasEstimator`/`BiasRemover`) and 2× `duplicateAssignExpression` (a `double min_x = corner[0].x, max_x = corner[0].x;` comma-declaration cppcheck reads as "the same expression assigned to two different variables," in `aruco_detector.cpp`'s bounding-box calculation). A related but separately-caught issue: an unused `req` parameter in `nav2_waypoints_bridge.cpp::on_cancel` produced a `-Wunused-parameter` build warning on every colcon build.
- **Root cause:** Advisory-only CI checks are easy to ignore since a red status never blocks anything — the same failure mode as a check nobody looks at.
- **Fix applied:** Moved the flagged `std::string` member assignments into the constructor member-initializer list (in declaration order, to avoid `-Wreorder`); split the `aruco_detector.cpp` comma-declarations into separate `min_x`/`max_x = min_x` (and `min_y`/`max_y`) statements — same seeding semantics, cppcheck no longer reads it as a duplicate expression; commented out the unused `req` parameter name in `on_cancel`. Removed `continue-on-error: true` from the `static-analysis` job now that the tree is clean, so future findings fail the PR. Verified: `cppcheck --enable=warning,performance,portability --std=c++20 --inline-suppr --error-exitcode=1 software/ros_ws/src software/shared software/arm-teleop-direct` exits 0; a clean (`rm -rf build install log`) `pixi run -e default build-test` produces zero stderr output (previously `perseus_autonomy_bridge` warned on every build).
- **Prevention rule:** Don't add a CI check as `continue-on-error`/advisory-only as a permanent state — either fix it up and make it blocking promptly, or it will silently accumulate findings that nobody acts on. If a check must start advisory (e.g., to avoid blocking on a large pre-existing backlog), track "flip to blocking" as a concrete, time-boxed follow-up rather than leaving it indefinitely soft.

### Docs env missing `graphviz`, only caught because no CI ever built docs on a PR — 2026-07-06

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `pixi.toml`, `.github/workflows/all.yaml`
- **Pattern:** The Pixi migration's `docs` feature ported `uv`/`doxygen` but not `graphviz`, even though several docs pages use `{graphviz}` directives (`sphinx_immaterial.graphviz`) that shell out to the `dot` binary at build time. This was a `documented gap, not implemented` note in `CLAUDE.md`, but nothing in CI actually built the Pixi docs env on a pull request — only `static.yml`'s push-to-`main` Pages deploy did, and that workflow still ran the _old_ Nix-based docs build (which had graphviz) right up until this same PR flipped it to Pixi. So the gap was invisible until a PR-time `Build docs` CI job was added as part of this same migration, at which point it failed outright with `FileNotFoundError: 'dot'`.
- **Root cause:** A capability documented as "not yet ported" had no CI signal forcing it to be revisited, and the one workflow that _would_ have caught it (`static.yml`) was, until this PR, still running the old toolchain that didn't have the gap.
- **Fix applied:** Added `graphviz = "*"` to `[feature.docs.dependencies]` in `pixi.toml`. Verified with `pixi run -e docs docs` locally (exit 0, 113 pre-existing/unrelated warnings) and via the new PR-time `Build docs` job in `all.yaml`.
- **Prevention rule:** When porting a toolchain/environment (Nix shell → Pixi env, etc.), diff the old environment's package list against the new one line-by-line rather than relying on "it built once" — and give every build product (including docs) a CI job that runs on pull requests, not just on push to the default branch, so gaps surface before merge.

### `perseus_lite_missions` unbuildable in envs that skip its `exec_depend` — 2026-07-06

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `pixi.toml`
- **Pattern:** `perseus_lite_missions` (`ament_python`) declares `<exec_depend>perseus_lite_simulation</exec_depend>`, but the `default` and `machine-learning` Pixi envs both `--packages-skip perseus_lite_simulation` in their build tasks. colcon's `ament_python` build task generates an environment hook that sources the exec_depend's install-space files, so building `perseus_lite_missions` in either env failed on a clean workspace with `Failed to find the following files: .../perseus_lite_simulation/share/perseus_lite_simulation/package.sh`. This passed locally the first time only because `perseus_lite_simulation` had already been built earlier in the same `software/ros_ws/install/` from prior work on the `simulation` env — a fresh CI runner caught it immediately.
- **Root cause:** Adding a new package with a cross-package `exec_depend` without checking whether every env that would build it also builds (or skips) that dependency consistently.
- **Fix applied:** Added `perseus_lite_missions` to the `--packages-skip` list alongside `perseus_lite_simulation` in both the `default` and `machine-learning` features' `build`/`build-test` tasks; it already builds correctly in `simulation`, where the dependency is present.
- **Prevention rule:** When a new package's `package.xml` declares an `exec_depend` on another in-tree package, check every Pixi/colcon env's skip list for consistency between the two — and verify with a **fully clean** rebuild (`rm -rf build install log`), not an incremental one, since a stale `install/` directory from earlier env-switching can mask a missing-dependency bug.

### `gz sim`/`rviz2` GUI segfaults from a shared fontconfig cache — 2026-07-05

- **Severity:** High
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_simulation/launch/gazebo.launch.py`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`, `software/ros_ws/src/autonomy/launch/mapping_using_slam_toolbox.launch.py`
- **Pattern:** `pixi run sim`'s GUI (Gazebo's Qt-based GUI, and separately RViz) reliably segfaulted a few seconds into startup, in `libfontconfig`'s `FcCharSetFindLeafForward`/`FcCharSetHasChar` (dereferencing a near-null pointer) during Qt text shaping. The system has fontconfig 2.15.0 installed; the Pixi `simulation` env bundles a separate, newer fontconfig 2.18.1 (`libfontconfig.so.1.17.0`). Neither installation overrides `XDG_CACHE_HOME`, so both share the same `~/.cache/fontconfig` directory — a binary charset/font cache written by whichever fontconfig ran most recently. Reading a cache built by one fontconfig version with a different, ABI-incompatible version corrupts an `FcCharSet`'s internal leaf-array traversal, producing the segfault. This is the same "two installations, one shared mutable resource" pattern as the `GZ_CONFIG_PATH` bug, just via a filesystem cache instead of an env var pointing at plugin libraries.
- **Root cause:** No isolation between the Pixi env's own library versions and whatever else on the machine (system packages, other conda envs) happens to write to the same default (`$HOME`-relative) cache/config locations that aren't Pixi/conda-prefix-scoped by default.
- **Fix applied:** Added `XDG_CACHE_HOME` (pointed at `$CONDA_PREFIX/var/cache`, a Pixi-env-private directory) to the `additional_env` of all three GUI-launching `ExecuteProcess` actions (Gazebo GUI, and both RViz launches). Verified with three separate clean runs (30s, 75s, 60s) of `pixi run sim` under `sg render` — zero crashes, and the Pixi env's own `var/cache/fontconfig` (plus `mesa_shader_cache`, `Gazebo`, Qt's shader cache) populated as expected, confirming it never touches `~/.cache` anymore. A prior diagnosis session (documented, since corrected, in `CLAUDE.md`) had left this as an open/unresolved gap.
- **Prevention rule:** When a Pixi/conda-packaged GUI/rendering tool crashes with no informative error (a bare segfault deep in a system library like fontconfig, Mesa, or similar), suspect a shared, version-sensitive cache or config directory under `$HOME` before assuming a code bug — check whether the tool respects an `XDG_*`-style override and, if so, point it at a env-private directory (`$CONDA_PREFIX/...`) rather than trying to reconcile versions.

### `arm-teleop-direct` silently broken by a Boost version bump — 2026-07-05

- **Severity:** Medium
- **Category:** Build
- **File(s):** `software/arm-teleop-direct/include/perseus-arm-teleop.hpp`, `software/arm-teleop-direct/src/perseus-arm-teleop.cpp`, `software/arm-teleop-direct/src/main.cpp`
- **Pattern:** `software/arm-teleop-direct` is a standalone CMake project (not part of the colcon workspace under `software/ros_ws/src/`), so it is never built by `pixi run build`/`build-test` or exercised by CI. `pixi.toml` pins `libboost-devel = "1.88.*"` for the colcon workspace's benefit (documented reason: Boost 1.90 drops the `boost_system` CMake component perseus_lite_hardware needs), but that pin also silently applies to this out-of-tree project, which still used `boost::asio::io_service` — removed in Boost ≥1.87 (renamed to `io_context`). Separately, `main.cpp`'s `display_progress_bar` had a parameter named `ncurses_win` shadowing the file-scope global `static WINDOW* ncurses_win`, which this project's `-Werror -Wshadow` build flags turn into a hard error. Neither was caught because nothing in `pixi run build`/CI ever compiles this package.
- **Root cause:** A package living outside the colcon workspace isn't covered by any of the repo's build/test tasks, so a toolchain-wide dependency bump (done for an unrelated in-tree package) broke it with no signal until someone builds it manually.
- **Fix applied:** Renamed `_io_service`/`io_service` to `_io_context`/`io_context` (3 sites; only ever used to construct `_serial_port`, no `io_service`-specific API in use). Renamed the shadowing parameter to `win` throughout `display_progress_bar`. Verified with a standalone `cmake && make` build — compiles clean.
- **Prevention rule:** Any package outside `software/ros_ws/src/` (not colcon-built) needs its own explicit build step in CI or a documented manual-verification note, or toolchain-wide pins (Boost, compiler, etc.) will silently break it with no test ever catching the regression.

### `gz_ros2_control` plugin tag never existed — sim was never end-to-end validated post-fork — 2026-07-04

- **Severity:** High
- **Category:** Logic
- **File(s):** `software/ros_ws/src/perseus_lite/urdf/perseus_lite.urdf.xacro`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`, `software/ros_ws/src/perseus_lite/launch/controllers.launch.py`
- **Pattern:** A sim launch graph passes `launch_controller_manager:=false` to the standalone controller-manager launch file, expecting a Gazebo plugin to host `controller_manager` instead — but the robot's URDF/xacro never actually declares that plugin. Nothing fails at build time (xacro expands fine, Gazebo spawns the robot fine); the only symptom is `ros2_control` spawners retrying forever because no `controller_manager` service ever exists. This is invisible unless someone actually runs the simulation for long enough to watch the spawners fail, which the Nix→Pixi migration's CI never did (build-only, no launch smoke test).
- **Root cause:** The `perseus_lite.urdf.xacro` top-level file was missing a `<gazebo><plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">` block gated on a `use_sim` xacro arg. Likely dropped or never carried over during the `perseus_simulation` → `perseus_lite_simulation` fork (Phase 3) or an earlier upstream rebase; no test ever asserted that `controller_manager` actually comes up in Gazebo.
- **Fix applied:** Added `<xacro:arg name="use_sim" default="false"/>` plus a `<xacro:if value="$(arg use_sim)">` block emitting the `gz_ros2_control` plugin tag (parameterized on `perseus_lite_controllers.yaml`) to `perseus_lite.urdf.xacro`. Verified with a standalone headless (`gz sim -s`, no GUI) run of the full launch graph: log shows `Loaded system [gz_ros2_control::GazeboSimROS2ControlPlugin] for entity [115]`, followed by `controller_manager` loading, configuring, and activating both `joint_state_broadcaster` and `diff_drive_base_controller` ("Configured and activated ...", "Successfully switched controllers!").
- **Prevention rule:** When a sim launch graph splits controller-manager hosting between "standalone node" and "Gazebo-plugin-hosted" modes via a boolean launch arg, add a smoke test (or at minimum a documented manual verification step) that actually launches the sim and asserts `ros2 control list_controllers` shows the expected controllers `active` — a clean build and a clean `--show-args` resolution are not sufficient evidence that a plugin-hosted controller_manager actually starts.

### Partial toolchain-migration cleanup missed a sibling launch file and a dead package include — 2026-07-04

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_description/launch/view_robot_nixgl.launch.py` (deleted), `software/ros_ws/src/autonomy/launch/mapping_using_slam_toolbox.launch.py`, `software/scripts/{update.sh,machine-setup.sh,autocomplete.sh}` (deleted), `software/scripts/_internal/secure_path_includes_nix` (deleted)
- **Pattern:** A prior cleanup pass (Nix→Pixi migration, or the Phase 1-4 perseus-lite hard-divergence) fixed the _primary_ instance of a stale reference (e.g. renamed `perseus` → `perseus_lite` in most launch files, replaced `nix run --impure nixGL` wrappers in the main sim launch chain) but missed a sibling file doing the same thing in a less-obvious location: a whole second RViz launch file (`view_robot_nixgl.launch.py`) still shelling out to a deleted `nixgl` wrapper alongside an already-fixed `view_robot.launch.py` twin, and `autonomy/launch/mapping_using_slam_toolbox.launch.py` still including `FindPackageShare("perseus")/launch/perseus.launch.py` — a package deleted in Phase 4 — because the SLAM launch file wasn't touched when other autonomy launch files were repointed at `perseus_lite`.
- **Root cause:** Migrations and package-deletions were grepped for the "main" usage pattern (the sim bringup chain, the primary launch file) but not swept exhaustively across every file referencing the same string, so parallel/rarely-run files (an alternate RViz launcher, a SLAM-only bringup path, one-off setup scripts) kept referencing deleted packages/tools with no build-time signal.
- **Fix applied:** Deleted `view_robot_nixgl.launch.py` (a non-functional twin of `view_robot.launch.py`, which already worked). Repointed `mapping_using_slam_toolbox.launch.py`'s `FindPackageShare("perseus")` / `perseus.launch.py` to `perseus_lite` / `perseus_lite.launch.py`. Deleted four fully-dead Nix-era scripts (`update.sh`, `machine-setup.sh`, `autocomplete.sh`, `_internal/secure_path_includes_nix`) that referenced `nix flake update`, `nix run home-manager`, and flake-provided helpers with no Pixi equivalent.
- **Prevention rule:** After a rename/deletion migration, `grep -rn` the _exact old identifier_ (package name, executable, wrapper command) across the **entire** repo — not just the directory/launch-chain you're actively working in — and enumerate every hit before declaring the migration done. Rarely-invoked sibling files (alternate entry points, one-off scripts) are exactly where stale references survive a "does it build" check, because they only fail at run time.

### Deprecated `tests_require=` in ament_python setup.py — 2026-07-03

- **Severity:** Low
- **Category:** Convention
- **File(s):** `software/ros_ws/src/input_devices/setup.py`, `software/ros_ws/src/teleop_diagnostics/setup.py`
- **Pattern:** Declaring test dependencies via `tests_require=[...]` in a `setup.py`'s `setup()` call. Setuptools deprecated and then dropped support for this argument; under setuptools >=68 it triggers `UserWarning: Unknown distribution option: 'tests_require'` on every build, which colcon surfaces as "N packages had stderr output" even though the build succeeds.
- **Root cause:** These `setup.py` files predate the setuptools deprecation and were never updated when test-dependency declaration moved to the ROS 2 package manifest.
- **Fix applied:** Removed `tests_require=["pytest"],` from both `setup.py` files. Test dependencies are already declared correctly via `<test_depend>pytest</test_depend>` (or equivalent) in each package's `package.xml`, which is the mechanism ament_python/colcon actually uses.
- **Prevention rule:** Never add `tests_require=` to a new ament_python `setup.py`. Declare test dependencies only via `<test_depend>` in `package.xml`. When adding a new Python package, check its `setup.py` doesn't carry this option before merging.

### Launch files still invoking `nix run` after the Nix→Pixi migration — 2026-07-03

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_simulation/launch/gazebo.launch.py`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`, `software/ros_ws/src/autonomy/launch/mapping_using_slam_toolbox.launch.py`
- **Pattern:** A `launch.actions.ExecuteProcess` wraps a GUI/rendering binary (Gazebo, RViz) in `nix run --impure github:nix-community/nixGL -- <real cmd>`. This wrapper only made sense under the old Nix dev shell, where Nix-built binaries couldn't see host OpenGL drivers. When a build system migration removes the underlying toolchain (Nix → Pixi/RoboStack here), commit `43c286c2` ("remove Nix build system") deleted `flake.nix` etc. but didn't grep launch files (or any non-toolchain file) for lingering invocations of the old toolchain's CLI.
- **Root cause:** Migration removed the tool the wrapper depended on but not the wrapper itself; nothing failed at build time because `ExecuteProcess` only fails at _launch_ time, so `pixi run -e simulation build` stayed green while `pixi run sim` was broken.
- **Fix applied:** Stripped the 5-token `nix run --impure github:nix-community/nixGL --` prefix and the now-meaningless `NIXPKGS_ALLOW_UNFREE` env var from all three `ExecuteProcess` calls; kept the other `additional_env` entries (`QT_QPA_PLATFORM`, `GZ_SIM_RESOURCE_PATH`, RMW QoS vars) which are still relevant under Pixi.
- **Prevention rule:** After removing a build system/toolchain, `grep -rn` the old tool's invocation string (e.g. `nix run`, `nix develop`) across the _entire_ repo, not just packaging files — launch files, scripts, and CI configs can all embed direct shell-outs to a toolchain that a migration otherwise fully replaces.

### `gz sim` segfaults from a contaminated `GZ_CONFIG_PATH` — 2026-07-03

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_simulation/launch/gazebo.launch.py`
- **Pattern:** `gz sim`'s core library and its plugins (rendering, physics, sensors) are resolved separately: the core binary comes from whatever's first on `PATH`/`LD_LIBRARY_PATH`, but which _plugin_ `.so` files it `dlopen`s is governed entirely by `GZ_CONFIG_PATH`. If a machine also has a native/system ROS 2 install (e.g. `/opt/ros/jazzy` sourced unconditionally from `~/.bashrc`), that install's own `setup.bash` sets `GZ_CONFIG_PATH` to its own Gazebo vendor packages' config dirs. Pixi's `simulation` env activation does not clear or override this var, so it leaks through from the parent shell. Result: the Pixi env's `gz-sim` core (v8.10.0 here) loads plugins built for the system install's different version (v8.11.0) — an ABI mismatch that segfaults (exit 139) immediately on rendering-engine plugin load, with **zero log output** (no Python traceback, no rclpy error, nothing — the crash is below the level anything in-process can catch or print).
- **Root cause:** Pixi/conda environment activation only adds/prepends env vars on top of whatever the parent shell already exported; it doesn't reset tool-specific vars like `GZ_CONFIG_PATH` that a _different_ toolchain's activation script (system ROS's `setup.bash`) had already set in that same shell.
- **First (wrong) diagnosis:** Initially misattributed this to the user's account not being in the `render` Unix group (needed for `/dev/dri/renderD128` GPU access). That group membership is real and worth having, but adding it did **not** fix the segfault — confirmed by reproducing the identical crash with `render` group active. The actual fix (below) was found by dumping the full env inside the activated Pixi shell and noticing `GZ_CONFIG_PATH` pointed entirely at `/opt/ros/jazzy`.
- **Fix applied:** `gazebo.launch.py`'s `additional_env` now explicitly sets `GZ_CONFIG_PATH` to `$CONDA_PREFIX/share/gz` (the Pixi env's own Gazebo config directory), overriding whatever the parent shell inherited. Confirmed fix by clearing/repointing the var manually first (`unset` and `$CONDA_PREFIX/share/gz` both avoided the crash; `""` did not — gz-sim treats an empty string as "no config" rather than falling back to a compiled-in default, so it must be pointed at a real directory, not just cleared).
- **Prevention rule:** When a Pixi/conda-activated tool crashes or misbehaves in a way that doesn't match its own version, dump the full environment (`env | grep -i <tool>`) inside the activated shell and check for the _other_ installation's paths (system package manager installs, a different environment manager, etc.) bleeding through — conda/Pixi activation is additive, not isolating, for vars it doesn't itself manage. Don't stop at the first plausible-sounding cause (like a permissions issue) without reproducing the fix in isolation first.

### Ineffective serial read timeout in ST3215 hardware interface — 2026-06-10

- **Severity:** Critical
- **Category:** API Misuse
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** Constructing a `boost::asio::steady_timer` next to a blocking I/O call without ever `async_wait`ing on it — the timer object does nothing unless it is wired into the same `io_context` run loop as an async operation.
- **Root cause:** The communication thread created a timer and then called the blocking `serial_port.read_some()`, assuming the timer's existence imposed a deadline. A wedged servo or unplugged USB adapter would block the thread forever.
- **Fix applied:** Added `read_with_timeout()` which pairs `async_read_some` with a deadline timer on the shared `io_context` and runs it to completion; timeout cancels the read and is reported as `operation_aborted`. The never-started `_io_thread` and unused async machinery were removed.
- **Prevention rule:** Never mix a constructed-but-unawaited asio timer with blocking I/O. If a timeout is required, both the I/O and the timer must be async operations on the same `io_context`, with each handler cancelling the other.

### Servo temperature state read the voltage register — 2026-06-10

- **Severity:** High
- **Category:** Logic
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`, `software/ros_ws/src/perseus_lite_hardware/include/st3215_protocol.hpp`
- **Pattern:** Hardcoded byte-offset constants for a wire format that drift from the device register map (off-by-one across a multi-byte field).
- **Root cause:** The status read covers registers 0x38–0x3F (position 2, speed 2, load 2, voltage 1, temperature 1). With the error byte at payload offset 0, temperature lands at offset 8, but `_TEMPERATURE_BYTE_INDEX` was 7 — the voltage register — so the exported temperature state reported bus voltage in 0.1 V units as °C.
- **Fix applied:** Packet parsing extracted into `protocol::extract_packets()` / `protocol::parse_status_payload()` with the correct offsets and a regression unit test (`ParseStatusPayloadReadsTemperatureNotVoltage`).
- **Prevention rule:** When parsing register-block reads, derive byte offsets from the register table (start register + sizes), write them next to a comment naming the registers, and pin them with a unit test using distinct sentinel values in adjacent fields.

### Serial request/response transaction not atomic — 2026-06-10

- **Severity:** High
- **Category:** Concurrency
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** Locking a shared bus only for the send half of a request/response exchange, allowing another thread's write to interleave between request and reply.
- **Root cause:** The communication thread released `_serial_mutex` after sending the status request and read the response unlocked, while the controller's `write()` could acquire the mutex and transmit velocity packets mid-exchange, corrupting the reply stream.
- **Fix applied:** The mutex is now held across the complete send → wait → read transaction.
- **Prevention rule:** On a half-duplex serial bus, the unit of mutual exclusion is the whole transaction (request plus response), never just the write.

### CI main-branch gate referenced the upstream repository — 2026-06-10

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `.github/workflows/all.yaml`, `README.md`
- **Pattern:** Fork retains `github.repository == '<upstream>/<repo>'` guards (and badges/URLs) from the parent project, silently disabling main-branch automation.
- **Root cause:** `is-main` checked for `ROAR-QUTRC/perseus-v2`, which is never true on `DingoOz/perseus-lite`, so jobs gated on it could never run; the README badge pointed at the upstream repo as well.
- **Fix applied:** Updated the repository check, gated the docs-website deploy on a `DOCS_DEPLOY_KEY` secret existing, and fixed the README title/badge.
- **Prevention rule:** After forking, grep workflows and docs for the upstream `owner/repo` string; every `github.repository ==` comparison must name the fork.

### Launch arguments declared but never forwarded to the included launch file — 2026-06-10

- **Severity:** High
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_missions/launch/mission_zero.launch.py`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`
- **Pattern:** `DeclareLaunchArgument` in a parent launch file without passing the value through `launch_arguments` of the `IncludeLaunchDescription` chain — the argument parses fine but has no effect.
- **Root cause:** `mission_zero.launch.py` declared `gz_world:=selene_base.sdf` and `initial_pose_*`, but `perseus_sim.launch.py` neither accepted nor forwarded them, so Mission Zero silently launched the default arc world at the default spawn pose.
- **Fix applied:** `perseus_sim.launch.py` now declares and forwards `gz_world` and `initial_pose_*` to `gazebo.launch.py`, and `mission_zero.launch.py` passes them down.
- **Prevention rule:** Every declared launch argument must appear either in a `Node`/process definition or in a child's `launch_arguments`; after adding one, verify it reaches the leaf with `ros2 launch <pkg> <file> --show-args` plus a grep of the include chain.
