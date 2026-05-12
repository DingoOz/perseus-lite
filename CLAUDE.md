# CLAUDE.md — perseus-lite

Project-specific guidance. Global rules from `~/.claude/CLAUDE.md` (auto-approved
commands, `ERRORS.md` workflow, `feature/<name>` branch workflow, no AI
attribution in git messages) apply here — this file does not restate them.

## 1. Project identity

`perseus-lite` is a fork of [`ROAR-QUTRC/perseus-v2`](https://github.com/ROAR-QUTRC/perseus-v2)
(`upstream` remote already configured; `origin` =
[`DingoOz/perseus-lite`](https://github.com/DingoOz/perseus-lite)).

The **lite robot** is a smaller, simpler subset of the full perseus-v2 platform:

- 50%-scale, 4-wheel skid-steer chassis with rocker suspension
- **Feetech ST3215 servos over serial** for wheel drive (one per wheel,
  IDs 1–4) — no CAN bus, no VESCs
- **Feetech-servo manipulator arm** (controlled by
  `software/arm-teleop-direct`)
- Designed for sandy environments
- Autonomy / SLAM / Nav2 / vision / Gazebo simulation are all in scope

**Explicitly NOT on the lite robot:**

- CAN bus, VESC motor controllers, mecanum drive
- Excavation bucket, elevator module, processing plant, light tower
- Bespoke perseus-v2 control PCBs (rover-control-board, smol-brain-board)

**Current cleanup state:** Phases 1, 2, and 3 complete on `main`. Phase 4
not started.

## 2. Repository layout

| Path                                                                      | Contents                                                       |
| ------------------------------------------------------------------------- | -------------------------------------------------------------- |
| `software/ros_ws/src/`                                                    | All ROS 2 packages (Jazzy). See §4 for KEEP/DISABLE.           |
| `software/arm-teleop-direct/`                                             | Standalone serial Feetech arm teleop (C++). Lite-relevant.     |
| `software/{daemons,scripts,utilities,web_ui,shared,native,home-manager}/` | General system infra — keep.                                   |
| `firmware/`                                                               | ESP32/MCU firmware. Subdirs split lite/v2 — see §4.            |
| `hardware/`                                                               | KiCAD PCB designs. Mostly v2-specific propulsion.              |
| `packages/`                                                               | Nix overlays for third-party deps (Groot2, Livox SDK, Open3D). |
| `docs/`                                                                   | Sphinx docs site (mostly perseus-v2 — see §4).                 |
| `nix/`, `flake.nix`, `default.nix`, `shell.nix`                           | Nix-based dev shell + build.                                   |
| `.clang-format`, `treefmt.nix`, `treefmt.toml`                            | Formatting config — run before commits.                        |

## 3. Build / run quick reference

```bash
# Dev shell (direnv loads automatically; otherwise:)
nix develop

# Build ROS workspace
cd software/ros_ws
colcon build --symlink-install
source install/setup.bash

# Primary lite bringup
ros2 launch perseus_lite perseus_lite.launch.py

# SLAM + Nav2
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py

# Just controllers / state publisher
ros2 launch perseus_lite controllers.launch.py
ros2 launch perseus_lite robot_state_publisher.launch.py

# Arm teleop (standalone, not a ROS package)
# Build per software/arm-teleop-direct/README.md
```

`ros2_control` requires stamped twist messages on Jazzy. To debug-publish:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.5}}}'
```

## 4. What's lite-relevant vs upstream-only

### KEEP — used directly by lite

| Package / dir                                                                              | Role                                                                                                     |
| ------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------- |
| `perseus_lite`                                                                             | Lite bringup: launch files, controllers, RViz config                                                     |
| `perseus_lite_hardware`                                                                    | `ros2_control` hardware interface for ST3215 servos over serial                                          |
| `perseus_lite_description`                                                                 | Lite URDF (4-wheel skid-steer, rocker, scaled meshes). All meshes now self-contained (Phase 2).          |
| `perseus_sensors`                                                                          | IMU + lidar drivers (RPLidar, Livox)                                                                     |
| `perseus_interfaces`                                                                       | Custom msg/srv definitions (shared)                                                                      |
| `input_devices`, `perseus_input`, `perseus_input_config`                                   | Gamepad/keyboard input + routing                                                                         |
| `teleop_diagnostics`                                                                       | TUI debug for teleop (shared)                                                                            |
| `autonomy`, `perseus_autonomy_bridge`, `perseus_bt_nodes`, `perseus_mapping`, `pcl_to_lsr` | Nav2 / SLAM / behavior trees / pointcloud→laserscan                                                      |
| `perseus_vision`                                                                           | ONNX detectors (cube, ArUco)                                                                             |
| `perseus_lite_simulation`                                                                  | Gazebo sim forked from `perseus_simulation`; spawns the lite URDF, vendors `twist_mux` config (Phase 3). |
| `software/arm-teleop-direct`                                                               | Serial Feetech arm teleop                                                                                |
| `firmware/battery-management-system`                                                       | BMS firmware (powers whole rover)                                                                        |
| `firmware/components`                                                                      | Shared firmware libs (board-support, hi-can, crc, type)                                                  |
| `software/shared`                                                                          | Shared C++ libs (hi-can, fd-wrapper, crc, ptr-wrapper, type-demangle)                                    |
| `software/{daemons,scripts,utilities,web_ui,home-manager,native}`                          | General infra                                                                                            |
| `packages/{groot2,livox-sdk2,open3d}`                                                      | Nix overlays for autonomy deps                                                                           |
| `hardware/libraries`, `hardware/templates`                                                 | Shared KiCAD libs                                                                                        |

### DISABLED — perseus-v2-only, ignored via `COLCON_IGNORE`

| Package / dir                | Phase | Reason it's v2-only                                                                               |
| ---------------------------- | ----- | ------------------------------------------------------------------------------------------------- |
| `perseus`                    | 1     | v2 bringup; assumes mecanum + CAN + VESCs. Lite uses `perseus_lite` instead.                      |
| `perseus_hardware`           | 1     | `ros2_control` hardware interface for VESCs over CAN. Lite uses `perseus_lite_hardware`.          |
| `perseus_can_if`             | 1     | CAN payload bridge. Lite has no CAN bus.                                                          |
| `perseus_payloads`           | 1     | VESC-driven arm/excavator/processing-plant drivers. Lite arm is Feetech (in `arm-teleop-direct`). |
| `perseus_simulation`         | 1/3   | Forked to `perseus_lite_simulation` in Phase 3; original kept disabled.                           |
| `perseus_description`        | 2     | Disabled after Phase 2 mesh migration. Lite no longer references it.                              |
| `firmware/excavation-bucket` | —     | Excavation arm MCU firmware (not in any default build target).                                    |
| `firmware/elevator-module`   | —     | Sample-elevator MCU firmware (not in any default build target).                                   |
| `firmware/light-tower`       | —     | Light-tower MCU firmware (not in any default build target).                                       |
| `firmware/processing-plant`  | —     | Processing-plant MCU firmware (not in any default build target).                                  |
| `hardware/dc-motor-driver`   | —     | KiCAD design for v2 DC-motor driver PCB; lite uses Feetech servos.                                |

### NEEDS WORK — has lite-relevant content but coupled to v2

| Package                                                                             | What's coupled                                                      | Resolution                                     |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------- | ---------------------------------------------- |
| `docs/source/challenge-breakdowns/{excavation-and-construction,space-resources}.md` | Describe v2 competition tasks lite doesn't run.                     | Delete in Phase 4; harmless until then.        |
| `teleop_diagnostics/config/teleop_diagnostics.yaml`                                 | Comment line points at `perseus_description/urdf/wheel.urdf.xacro`. | Update comment when convenient — non-blocking. |

### DELETE-LATER (Phase 4)

Everything on the DISABLED list above, **plus** `perseus_description` once
Phase 2 is done, **plus** the v2-specific docs files. Only after the team
commits to a hard divergence from upstream (see §6).

## 5. Phased cleanup playbook

The fork should currently support clean `git merge upstream/main` pulls. The
goal of this playbook is to **strip the v2-only code without losing that
property** until a deliberate decision to hard-diverge.

### Phase 1 — disable (safe, reversible) — DONE

`COLCON_IGNORE` markers are in place on:
`perseus`, `perseus_hardware`, `perseus_can_if`, `perseus_payloads`,
and `perseus_simulation` (cascaded — it depends on `perseus`).

Verified: `colcon list` shows 16 packages, `colcon build` succeeds clean.

Firmware modules (`excavation-bucket`, `elevator-module`, `light-tower`,
`processing-plant`) are **not** wired into `flake.nix` and aren't part
of any default build target — no Phase 1 action was needed. They stay
in-tree until Phase 4 deletion.

### Phase 2 — break the `perseus_description` mesh dependency — DONE

Six meshes (`chassis`, `flange_bearing`, `differential_bar`, `rocker_left`,
`rocker_right`, `gearbox`) copied into
`perseus_lite_description/meshes/`. Three xacro files
(`chassis.urdf.xacro`, `rocker.urdf.xacro`, `motor_wheel.urdf.xacro`)
repointed from `$(find perseus_description)` to
`$(find perseus_lite_description)`. `<depend>perseus_description</depend>`
removed from `perseus_lite_description/package.xml`.
`perseus_description` itself disabled via `COLCON_IGNORE`.

Verified: `xacro src/perseus_lite/urdf/perseus_lite.urdf.xacro` resolves
to a 644-line URDF with 18 mesh references, all under
`perseus_lite_description/`, zero stale `perseus_description` refs.
`colcon build` passes with 15 packages.

### Phase 3 — fork `perseus_simulation` → `perseus_lite_simulation` — DONE

Took **Option A** (full fork). `perseus_simulation/` was copied to
`perseus_lite_simulation/` (the original keeps its `COLCON_IGNORE`):

- `package.xml`: `<name>perseus_lite_simulation</name>`,
  `<exec_depend>perseus</exec_depend>` → `<exec_depend>perseus_lite</exec_depend>`,
  maintainer updated.
- `CMakeLists.txt`: `project(perseus_lite_simulation)`.
- `launch/perseus_sim.launch.py`: `FindPackageShare("perseus")` for
  `robot_state_publisher.launch.py` and `controllers.launch.py` swapped to
  `perseus_lite`.
- `launch/gazebo.launch.py`: Gazebo spawn-entity name `"perseus"` →
  `"perseus_lite"`.
- `launch/twist_mux.launch.py` + `config/twist_mux.yaml`: vendored from the
  v2 `perseus` package; the launch file now self-references
  `perseus_lite_simulation`.
- `tests/gz_bridge_config_test.cpp`: hardcoded path updated by the bulk
  rename.

Verified: `colcon build` passes 16 packages; `ROS2 launch --show-args
perseus_lite_simulation perseus_sim.launch.py` resolves the full launch
graph (rsp + controllers + gazebo + rosbridge + twist_mux + ekf + rviz)
without errors. Actually running Gazebo requires `nix develop .#simulation`
and a GPU — not validated in this batch.

### Phase 4 — hard-divergence delete (manual approval only)

Only after the team explicitly decides not to track upstream further:

- `git rm -r` everything on the DISABLED list and `perseus_description`.
- Delete `docs/source/challenge-breakdowns/{excavation-and-construction,space-resources}.md`.
- Rename remote: `git remote rename upstream upstream-archive`.

**Do not execute Phase 4 without an explicit user request.** It is
irreversible from upstream's perspective and changes the merge strategy
permanently.

## 6. Working with upstream

Until Phase 4:

```bash
git fetch upstream
git log --oneline HEAD..upstream/main      # what's incoming
git merge upstream/main                    # conflicts confined to lite-specific xacro + COLCON_IGNORE
```

After Phase 4, prefer `git cherry-pick` of specific upstream commits — full
merges will pull back the deleted v2 packages.

## 7. Conventions specific to this repo

- ROS 2 distribution: **Jazzy**. `diff_drive_controller` requires
  `TwistStamped` messages.
- Package licenses are MIT (see recent `chore: Update Nix packaging` and
  `chore: Changed ROS package.xml TODO licenses to MIT` commits).
- C++ formatting: `.clang-format` at repo root. Run `treefmt` before commit.
- Servo IDs are fixed by hardware: FL=1, FR=2, RL=3, RR=4. Don't renumber
  without coordinating with the firmware/wiring side.

## 8. First-time-on-this-repo checklist

1. Read this file end-to-end.
2. `git fetch upstream && git log --oneline HEAD..upstream/main` — see
   pending upstream drift.
3. `colcon list` in `software/ros_ws/` and cross-check against the KEEP /
   DISABLED tables in §4.
4. Check `ERRORS.md` (per global rules) for any prevention rules touching
   files you plan to edit. Create `ERRORS.md` and log new bugs as
   instructed in `~/.claude/CLAUDE.md`.
5. Create a `feature/<name>` branch before changes (per global rules).
