# Mission Zero — Next Steps

Script (source of truth for the segment flow) is at
`software/assets/missions/rover-robotics-mission-zero-cold-open-script.md`.

## Current state

- Package `perseus_lite_missions` (ament_python) builds and tests in the
  `simulation` Pixi env only (it `exec_depend`s on `perseus_lite_simulation`,
  which the `default`/`machine-learning` envs skip — see `pixi.toml`).
- Selene Base world (`selene_base.sdf`) in `perseus_lite_simulation/worlds/`.
  Box-primitive habitat with a 0.8 m corridor (narrowest point ~0.72 m beside
  the entrance debris), collapsed panels, leak marker at (9.8, 0). Moon_DEM
  terrain. Launch defaults to this world with spawn at (0, 0, 0.3).
  `test/test_world_geometry.py` checks the corridor is passable with the
  answer key and sealed with the broken values.
- Broken + reference configs for SLAM and Nav2 are in place. Frames are
  `base_link`/`odom`/`map`; scans come from `/scan_filtered`. The Nav2 answer
  key uses the rover's real polygon footprint.
- The orchestrator **owns slam_toolbox and Nav2** as `ros2 launch` process
  groups (`process_group.ManagedProcess`), started with the broken params:
  - Challenge 1 passes (map_quality checker) → slam_toolbox is stopped
    (whole process group, SIGINT → SIGTERM → SIGKILL) and relaunched with
    `slam_params.reference.yaml`.
  - nav2_params checker reports the student's file fixed → Nav2 is stopped
    and relaunched with `nav2_params.reference.yaml`.
  - Challenge 2 passes when the goal_reached checker sees the rover at the
    leak under an executing/succeeded NavigateToPose goal.
  - Challenges are accepted in mission order only (`mission_logic.py`), so
    driving to the leak by hand before Challenge 1 does not count.
- Badges persist to `~/.config/rover-robotics/badges.json`. Mission state +
  badge events on `/mission/state` and `/mission/badge`; checker status
  (`pass`/`fail`/`invalid`/`missing` + reason) on
  `/mission/challenge_1/status` and `/mission/challenge_2/params_status`.

## Resume here

```bash
# Build (simulation env builds the whole workspace except perseus_vision)
pixi run -e simulation build

# Inspect the launch graph
pixi run -e simulation bash -c \
  'cd software/ros_ws && source install/setup.bash && \
   ros2 launch perseus_lite_missions mission_zero.launch.py --show-args'

# Actually launch (Gazebo opens — needs a GPU):
pixi run -e simulation bash -c \
  'cd software/ros_ws && source install/setup.bash && \
   ros2 launch perseus_lite_missions mission_zero.launch.py'

# Unit + lint tests (expect a non-zero pytest count)
pixi run -e simulation bash -c \
  'cd software/ros_ws && colcon test --packages-select perseus_lite_missions && \
   colcon test-result --verbose'
```

## Remaining work (priority order)

### 1. Web UI page

Create `software/web_ui/src/routes/mission-zero/+page.svelte`. Needs:

- Video player that walks segments from `mission.yaml`.
- Monaco/CodeMirror editor for the two YAML files, highlighting the
  relevant lines per challenge.
- rosbridge subscription to `/mission/state`,
  `/mission/challenge_1/passed`, `/mission/challenge_2/passed`,
  `/mission/challenge_1/status`, `/mission/challenge_2/params_status`,
  `/mission/badge`.
- Badge overlay on challenge pass.
- Save button that writes the edited YAML back to the orchestrator via
  rosbridge (or via a small REST shim if rosbridge file write is
  awkward — needs a 5-min spike to confirm).
- Segment advancement for `video`/`drive` segments (the orchestrator only
  advances on challenge passes today).

`rosbridge_server` is already launched by `perseus_sim.launch.py`
so the websocket wiring is free. Note `software/web_ui/` has no Pixi
toolchain yet (see CLAUDE.md).

### 2. `mission-zero` Pixi task

Add a task to `pixi.toml`'s `[feature.simulation.tasks]`, mirroring the
existing `sim` task:

```toml
mission-zero = { cmd = "bash -c 'source install/setup.bash && ros2 launch perseus_lite_missions mission_zero.launch.py'", cwd = "software/ros_ws", depends-on = ["build"] }
```

Then extend it (or a wrapper script) to serve the web_ui build and
`xdg-open http://localhost:5173/mission-zero`. Goal: a single
`pixi run -e simulation mission-zero` opens the experience end-to-end.

### 3. Docs landing page

`docs/source/tutorials/mission-zero.md`. Above-the-fold: the single
`pixi run -e simulation mission-zero` command, prerequisites (Pixi
installed, Linux x86_64, GPU). Below: lesson notes from the script
(commands + the two YAML diffs) for readers who prefer text. Link from
the docs landing page (commit `036159fe`).

### 4. CI coverage

Confirm a CI job builds and runs `colcon test --packages-select
perseus_lite_missions` in the `simulation` env (the `default` env skips
the package). Check the pytest count is non-zero in the job log.

### 5. End-to-end run

Walk the full script flow: cold launch, drive, see smeared map, edit
slam params, see clean map, send goal, see corridor blocked, edit nav2
params, watch Nav2 restart and the rover reach the leak. Time it — target
under 12 minutes on a warm build.

### 6. World geometry polish

The current Selene Base world uses box primitives. For the video
production, you may want:

- Better textures on the habitat walls (Gazebo material scripts).
- A partial roof over parts of the habitat.
- Lighting inside the corridor (point lights at the entrance and leak).
- Dust/particle effects near the collapsed section.
- Contact sensor on the chassis for a no-collisions gate on Challenge 2.

Re-run `test/test_world_geometry.py` after moving anything near the
corridor.

## Out of scope

- Video production (filming, voiceover, captions).
- Mission 1 onwards.
- Mac/Windows support (call out Linux-only in docs).
- Signed-in skill-tree backend (local badges are enough for Tier 0).
