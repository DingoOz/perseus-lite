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

**Current cleanup state:** Phase 1 complete on branch
`feature/lite-cleanup-phase1`. Phases 2–4 not started.

## 2. Repository layout

| Path | Contents |
|---|---|
| `software/ros_ws/src/` | All ROS 2 packages (Jazzy). See §4 for KEEP/DISABLE. |
| `software/arm-teleop-direct/` | Standalone serial Feetech arm teleop (C++). Lite-relevant. |
| `software/{daemons,scripts,utilities,web_ui,shared,native,home-manager}/` | General system infra — keep. |
| `firmware/` | ESP32/MCU firmware. Subdirs split lite/v2 — see §4. |
| `hardware/` | KiCAD PCB designs. Mostly v2-specific propulsion. |
| `packages/` | Nix overlays for third-party deps (Groot2, Livox SDK, Open3D). |
| `docs/` | Sphinx docs site (mostly perseus-v2 — see §4). |
| `nix/`, `flake.nix`, `default.nix`, `shell.nix` | Nix-based dev shell + build. |
| `.clang-format`, `treefmt.nix`, `treefmt.toml` | Formatting config — run before commits. |

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

| Package / dir | Role |
|---|---|
| `perseus_lite` | Lite bringup: launch files, controllers, RViz config |
| `perseus_lite_hardware` | `ros2_control` hardware interface for ST3215 servos over serial |
| `perseus_lite_description` | Lite URDF (4-wheel skid-steer, rocker, scaled meshes). **Currently `<depend>perseus_description</depend>` — see Phase 2.** |
| `perseus_sensors` | IMU + lidar drivers (RPLidar, Livox) |
| `perseus_interfaces` | Custom msg/srv definitions (shared) |
| `input_devices`, `perseus_input`, `perseus_input_config` | Gamepad/keyboard input + routing |
| `teleop_diagnostics` | TUI debug for teleop (shared) |
| `autonomy`, `perseus_autonomy_bridge`, `perseus_bt_nodes`, `perseus_mapping`, `pcl_to_lsr` | Nav2 / SLAM / behavior trees / pointcloud→laserscan |
| `perseus_vision` | ONNX detectors (cube, ArUco) |
| `software/arm-teleop-direct` | Serial Feetech arm teleop |
| `firmware/battery-management-system` | BMS firmware (powers whole rover) |
| `firmware/components` | Shared firmware libs (board-support, hi-can, crc, type) |
| `software/shared` | Shared C++ libs (hi-can, fd-wrapper, crc, ptr-wrapper, type-demangle) |
| `software/{daemons,scripts,utilities,web_ui,home-manager,native}` | General infra |
| `packages/{groot2,livox-sdk2,open3d}` | Nix overlays for autonomy deps |
| `hardware/libraries`, `hardware/templates` | Shared KiCAD libs |

### DISABLED (Phase 1) — perseus-v2-only, ignored via `COLCON_IGNORE`

| Package / dir | Reason it's v2-only |
|---|---|
| `perseus` | v2 bringup; assumes mecanum + CAN + VESCs. Lite uses `perseus_lite` instead. |
| `perseus_hardware` | `ros2_control` hardware interface for VESCs over CAN. Lite uses `perseus_lite_hardware`. |
| `perseus_can_if` | CAN payload bridge. Lite has no CAN bus. |
| `perseus_payloads` | VESC-driven arm/excavator/processing-plant drivers. Lite arm is Feetech (in `arm-teleop-direct`). |
| `perseus_simulation` | Depends on `perseus`; cannot build with Phase 1 markers in place. Restored or replaced in Phase 3. |
| `firmware/excavation-bucket` | Excavation arm MCU firmware. |
| `firmware/elevator-module` | Sample-elevator MCU firmware. |
| `firmware/light-tower` | Light-tower MCU firmware. |
| `firmware/processing-plant` | Processing-plant MCU firmware. |
| `hardware/dc-motor-driver` | KiCAD design for v2 DC-motor driver PCB; lite uses Feetech servos. |

### NEEDS WORK — has lite-relevant content but coupled to v2

| Package | What's coupled | Resolution |
|---|---|---|
| `perseus_description` | `perseus_lite_description/urdf/*.xacro` reference its meshes (`chassis.dae`, `flange_bearing.dae`, `differential_bar.dae`, `rocker_{left,right}.dae`, `gearbox.dae`). | Phase 2 — copy meshes, repoint xacro, drop dependency. |
| `docs/source/challenge-breakdowns/{excavation-and-construction,space-resources}.md` | Describe v2 competition tasks lite doesn't run. | Delete in Phase 4; harmless until then. |

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

### Phase 2 — break the `perseus_description` mesh dependency

```bash
cd software/ros_ws/src
mkdir -p perseus_lite_description/meshes
for f in chassis flange_bearing differential_bar rocker_left rocker_right gearbox; do
  cp perseus_description/meshes/${f}.dae perseus_lite_description/meshes/
done
```

Then in `perseus_lite_description/urdf/*.xacro`, replace every
`$(find perseus_description)/meshes/...` with
`$(find perseus_lite_description)/meshes/...`. Remove
`<depend>perseus_description</depend>` from
`perseus_lite_description/package.xml`. Verify:

```bash
colcon build --packages-select perseus_lite_description
ros2 launch perseus_lite_description view_robot.launch.py   # if it exists,
                                                            # else use perseus_lite robot_state_publisher.launch.py
```

After Phase 2, `perseus_description` is unreferenced and can move to the
DISABLED list (add its `COLCON_IGNORE`).

### Phase 3 — repoint or fork `perseus_simulation`

The current package builds a Gazebo model of perseus-v2. Two options:

- **Option A (preferred long-term):** copy `perseus_simulation/` to
  `perseus_lite_simulation/`, swap `<exec_depend>perseus</exec_depend>` for
  `perseus_lite`, replace v2 URDF references with `perseus_lite_description`,
  drop mecanum/arm/excavation models, retain Gazebo plumbing. Then add
  `COLCON_IGNORE` to the original `perseus_simulation`.
- **Option B (interim, less work):** edit `perseus_simulation/launch/*` to
  reference `perseus_lite` instead of `perseus`. Cheaper, but invites merge
  conflicts when upstream changes the package.

State the choice here in CLAUDE.md when one is made.

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
