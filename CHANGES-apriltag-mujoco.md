# Summary of changes on `feature/apriltag-localisation`

Nothing is committed. The branch sits on top of the terrain feature (PR #49, commit 8fbf8a72).

## 1. AprilTag landmark localisation: new package `perseus_landmarks`

- **`tag_detector`**: OpenCV AprilTag detection with IPPE pose estimation. It rejects ambiguous poses using the IPPE ambiguity ratio.
- **`landmark_localiser`**:
  - Publishes `site -> map`, and only after the first fix. That frame is the surveyed frame of the tag and destination map.
  - It gates on range (4 m), view angle (60 deg) and reprojection error (2 px).
  - It gates on jumps (0.3 m, confirmed over 3 s) and refuses fixes while the robot is turning.
  - It counts rejections by cause, so a calibration timeout says why it failed.
- **`landmark_navigator`**: search and go-to-destination actions. It moves the robot only through Nav2.
- **Interfaces** (`perseus_interfaces`): `TagDetection`, `TagDetectionArray`, `LandmarkLocalisation`, the `CalibrateFromLandmarks` service and the action definitions.
- **Integration**: a `landmarks:=true` option in `autonomy.launch.py`, plus the Gazebo AprilTag model, the `terrain_test.sdf` world and the `gz_bridge.yaml` entries.

### Fixes made this session

| Problem | Fix |
| --- | --- |
| The search stopped on unusable sightings (5.5 m away, or 63 deg off-normal), so calibration then saw none. | The navigator applies the same gates as the localiser (`min_sightings` 3, `seen_timeout_s` 1.0, range, angle, reprojection). |
| A single tag seen cleanly at 59 deg (ratio 2.80) could never calibrate. | `min_ambiguity_ratio` lowered from 3.0 to 2.0, based on measured error against ratio. |
| Nav2's Spin overshot and carried tags out of view. | The spin stops at the first glimpse and turns back to that heading if it overshot. |
| Calibration failed once, then the search gave up. | `search(calibrate=True)` keeps turning until a fix is taken. |
| `go_to_destination` was rejected right after calibration because the `site` TF hadn't propagated. | It waits up to 3 s for the frame. |
| The error at the destination was measured against a stale estimate. | It takes a stationary calibration on arrival, then refines if needed. |

## 2. MuJoCo simulator: new package `perseus_lite_mujoco`

- **Run it**: `pixi run -e simulation sim-mujoco [headless:=true]`. `mujoco` and `pyopengl` were added to the `simulation` env in `pixi.toml` and `pixi.lock`.
- **Model**: `model_builder.py` builds the MJCF from the xacro URDF and the SDF world. It handles box, cylinder, sphere and plane geometry and local model includes, so tags and `terrain_test.sdf` work. Mesh and heightmap worlds are skipped.
- **Simulator node**: `sim_node.py` runs the physics in real time.
  - Wheel commands come from `/joint_states`, with ros2_control on mock hardware.
  - It publishes `/clock`, `/scan`, `/imu/data`, the OAK-D depth and RGB images, and `/mujoco/ground_truth`.
- **Rendering fix**: clip planes are pinned in metres (near 2 cm, far 60 m). Before this, nothing closer than about 0.5 m was visible.
- **Calibration**: effective `wheel_separation` is 2.693 (`calibrate_turning` finds it), and the robot mass is a realistic estimate.
- **EKF overlay** (`mujoco_ekf.yaml`):
  - It opens the wheel-odometry gates.
  - It fuses the gyro yaw rate. Before this, heading drifted 40-60 deg after a few turns; it now stays within 1 deg of truth.
- **Launch and tooling**:
  - There is a `mujoco_sim.launch.py` and a README.
  - The TUI got a "MuJoCo simulation" profile, with the matching test updated.
  - Package tests: 8.

## 3. Shared Nav2 config change (affects the real robot)

I removed the `PreferForward` critic and its two parameters from `autonomy/config/perseus_nav_params.yaml`. It scored 320 for a 1 rad/s turn against about 60 for all other critics combined, so DWB always drove straight. A goal 90 deg to the left failed with the rover driving at a rock. With the critic gone, the same goal succeeds. **This needs a hardware re-check, or a revert.**

## 4. Validation against ground truth (MuJoCo)

| Test | Result |
| --- | --- |
| Static calibration | 0.9-4.0 cm, 0.1-0.7 deg |
| Search, starting with the rover facing away from every tag | Found a tag and calibrated, 1.0-2.3 cm, 0.2-0.6 deg |
| Go to `start` (uncalibrated, tag behind) | Succeeded, 17 cm off, estimate within 3 cm |
| Go to `north_bay` | Succeeded, 6 cm |
| Go to `east_dock` | Succeeded, 8 cm; with `terrain:=true`, 12 cm and the tag was found |
| Steady turns at 0.3 / 0.6 / 1.0 rad/s | 0.28 / 0.62 / 1.11 rad/s achieved |
| Nav2 goal 90 deg to the left | Succeeded |

The camera is pitched down, so a tag closer than about 0.55 m leaves the top of the image. Put stop poses at least 0.8 m from the tag.

## 5. Documentation and process files

- **New docs**: `landmark-localisation.md`, `mujoco-simulation.md` and the operator guide `landmark-navigation.md`, plus a package README for the MuJoCo package.
- **Updated docs**: `terrain-autonomy.md` (Gazebo known issues now point at MuJoCo), `running-simulation.md` (new MuJoCo section) and `mission-control-tui.md`.
- **`CLAUDE.md`**: MuJoCo build and run commands, a KEEP-table row, and the package count now 23.
- **`ERRORS.md`**: four new entries (DWB `PreferForward`, MuJoCo clip planes, MuJoCo EKF heading, AprilTag search issues). It now has 51 entries.

## 6. Verification

- **Default env**: clean build, 19 packages, 380 tests, 0 failures.
- **Simulation env**: clean build, 22 packages, 201 tests, 0 failures (MuJoCo 8, TUI 70).
- **Other checks**: treefmt clean, cppcheck clean, docs build clean (the 11 warnings were already there and are unrelated).

## Things to know

- **Not changed, but worth looking at on the robot**:
  - The EKF's wheel-odometry gate of 1.0 probably rejects the start of fast turns.
  - The robot EKF fuses no gyro.
  - Both are written up in `mujoco-simulation.md`.
- **Build state**: the workspace is currently built in the simulation env. Run `pixi run clean` before building the default env.
- **PR #49**: it still contains the old bounce statements, which have since been superseded. Update it when committing this.
