# Terrain-aware autonomy

Perseus Lite's 2D lidar sees a single horizontal slice ~0.11 m above the
ground. Anything shorter (a fist-sized rock, a step down, the lip of a hole)
is invisible to it, and on sand those are exactly the things that stop a
robot with 7 cm wheels. Terrain-aware autonomy adds the
**OAK-D Lite stereo depth camera** and uses it to judge the ground itself:

- **`perseus_terrain`** classifies the depth camera's view into _drivable_,
  _rough_ and _not drivable_ ground, keeps a long-term memory of what it saw,
  and feeds both into Nav2's costmaps.
- **`perseus_monitoring`** watches the stack's topics, and detects a stalled
  drive or wheels spinning in sand.
- A **terrain Nav2 overlay** loads extra planners and controllers you can
  switch between live, smooths paths, and adds terrain-memory clearing to the
  recovery behaviour tree.

Everything is opt-in (`terrain:=true`). With it off, the robot runs exactly the
robot-validated Nav2 configuration it always has.

```{contents}
:local:
:depth: 2
```

## Where the ideas come from

The design follows the concepts of the
[perseus-v3](https://github.com/ROAR-QUTRC/perseus-v3) autonomy stack
(`software/ros_ws/src/nav-stack` on its `test/autonomy` branch), built for the
full-size Perseus at the NASA Lunabotics challenge. **No code was copied**: the
upstream packages are GPL-3.0-only and Nix-built, while perseus-lite is MIT and
Pixi-built, so every concept was re-implemented from its description and adapted
to the lite robot's very different sensors (a forward depth camera and a 2D
lidar instead of a 360° 3D LiDAR, and a 2D wheel-odometry EKF instead of
LiDAR-inertial odometry).

| Upstream concept                                                                                                    | Lite implementation                                                  | Status                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Height-difference terrain classifier (step up, step down, height above ground, slope allowance, small-blob pruning) | `perseus_terrain::classify_terrain`                                  | **Ported**, plus robust percentiles for stereo noise and a roughness cost                                                 |
| Reactive local traversability grid                                                                                  | `terrain_mapper` → `/terrain/local_grid`                             | **Adapted**: one depth frame at a time (see [Per-frame classification](#per-frame-classification))                        |
| Log-odds global traversability memory, growing map                                                                  | `terrain_mapper` → `/terrain/global_grid` (`LogOddsMap`)             | **Ported**                                                                                                                |
| Clear terrain memory as a recovery step                                                                             | `~/clear_around_robot`, used by `navigate_to_pose_terrain.xml`       | **Ported**                                                                                                                |
| Self-clearing of the cells under the robot                                                                          | `clear_under_robot` (every memory publish)                           | **Ported**                                                                                                                |
| Terrain grids into Nav2 via StaticLayer                                                                             | `perseus_terrain::TerrainGridLayer` costmap plugin                   | **Changed**: a real layer plugin that transforms between frames, so odom-frame grids work in the map-frame global costmap |
| Several planners and controllers loaded, picked per goal from latched topics                                        | `terrain_nav_params.yaml` + `PlannerSelector` / `ControllerSelector` | **Ported** (ThetaStar, Smac 2D, NavFn; DWB, RPP)                                                                          |
| Path smoothing with a degrade-not-fail fallback                                                                     | `SmoothPath` + `AlwaysSuccess` in the tree                           | **Ported**                                                                                                                |
| Smac State Lattice planner                                                                                          | —                                                                    | **Not loaded**: needs 10 cm primitives ([how to add](#adding-the-lattice-planner))                                        |
| Topic health monitor (rate, bandwidth, staleness)                                                                   | `topic_health_monitor` → `/health/topics`                            | **Ported**                                                                                                                |
| Mobility / slip watchdog (commanded vs measured, windowed)                                                          | `mobility_monitor` → `/mobility/status`                              | **Adapted**: the lite's EKF fuses wheel speed, so it compares command vs wheels (stall) and wheels vs gyro (slip)         |
| Planner probe tool                                                                                                  | `ros2 run autonomy plan_probe.py`                                    | **Ported**                                                                                                                |
| Flat `base_footprint` frame                                                                                         | —                                                                    | **Not needed**: the lite EKF is 2D, so `base_link` is already flat                                                        |
| LiDAR-inertial odometry, ORB-SLAM3 visual odometry                                                                  | —                                                                    | **Not ported**: no 3D LiDAR; VO is future work (see [Future work](#future-work))                                          |
| Arena server, fiducial arena localisation, dig/dump bucket missions                                                 | —                                                                    | **Not ported**: Lunabotics-specific; the lite has no bucket                                                               |
| Draco point-cloud compression, base-station mesher                                                                  | —                                                                    | **Not needed**: only small grids leave the robot                                                                          |

## Architecture

```{graphviz}
:caption: Terrain-aware autonomy data flow (new parts in pink)
:align: center

digraph terrain {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", nodesep=0.3, ranksep=0.55];
    node  [fontname="Roboto", fontsize=10, style="filled,rounded", shape=box, penwidth=1.0, fontcolor="white", fillcolor="#37474f"];
    edge  [fontname="Roboto", fontsize=8, color="#7a6cad"];

    subgraph cluster_cam {
        label="camera Pixi env"; fontname="Roboto"; color="#7a6cad";
        oak [label="OAK-D Lite\ndepthai_ros_driver", fillcolor="#ec407a"];
    }
    lidar [label="RPLidar C1\n→ scan_range_filter"];
    ekf   [label="EKF\nodom → base_link"];
    slam  [label="slam_toolbox\nmap → odom"];
    tm    [label="terrain_mapper", fillcolor="#ec407a"];
    lc    [label="local costmap\nobstacle + terrain_local\n+ terrain_memory + inflation"];
    gc    [label="global costmap\nstatic + obstacle\n+ terrain_memory + inflation"];
    bt    [label="bt_navigator\nnavigate_to_pose_terrain.xml", fillcolor="#ec407a"];
    plan  [label="planner_server\nGridBased | ThetaStar | Smac2D"];
    ctrl  [label="controller_server\nFollowPath (DWB) | RPP"];
    mux   [label="velocity_smoother → collision_monitor\n→ twist_mux → /cmd_vel"];
    mon   [label="topic_health_monitor\nmobility_monitor", fillcolor="#ec407a"];

    oak -> tm [label="/oak/stereo/image_raw\n+ camera_info"];
    ekf -> tm [label="TF"];
    tm -> lc [label="/terrain/local_grid"];
    tm -> lc [label="/terrain/global_grid"];
    tm -> gc [label="/terrain/global_grid"];
    lidar -> lc [label="/scan_filtered"];
    lidar -> gc;
    slam -> gc [label="/map, TF"];
    bt -> plan; bt -> ctrl;
    bt -> tm [label="clear_around_robot", style=dashed];
    plan -> gc [dir=back];
    ctrl -> lc [dir=back];
    ctrl -> mux;
    mux -> mon [label="/cmd_vel"];
}
```

The camera driver runs in its own Pixi environment (see [OAK-D Lite](../hardware/oak-d-lite.md)):
RoboStack builds `depthai_ros_driver` only against a newer ROS snapshot than
the robot environment is locked to. It is a separate process either way and
talks to the rest of the robot over DDS.

### Frames and topics

| Name                                                                 | Type                                      | Notes                                                                                     |
| -------------------------------------------------------------------- | ----------------------------------------- | ----------------------------------------------------------------------------------------- |
| `oak_link` → `oak_{rgb,left,right}_camera_frame` → `…_optical_frame` | TF (URDF)                                 | The URDF is the single source of the camera frames; the driver's own TF publisher is off. |
| `/oak/stereo/image_raw`                                              | `sensor_msgs/Image`                       | 16UC1 millimetres (robot), 32FC1 metres (Gazebo), frame `oak_right_camera_optical_frame`. |
| `/oak/stereo/camera_info`                                            | `sensor_msgs/CameraInfo`                  | Intrinsics; also the health monitor's camera liveness proxy.                              |
| `/terrain/local_grid`                                                | `nav_msgs/OccupancyGrid`, volatile        | Latest frame's verdicts, ±4 m around the robot, in `odom`.                                |
| `/terrain/global_grid`                                               | `nav_msgs/OccupancyGrid`, transient local | Terrain memory, in `odom`, 1 Hz.                                                          |
| `/terrain/debug/{step_up,step_down,height_above_ground,points}`      | grids / `PointCloud2`                     | Only with `terrain_debug:=true`.                                                          |
| `/terrain_mapper/clear_around_robot`, `/terrain_mapper/clear_all`    | `nav2_msgs/srv/ClearEntireCostmap`        | Forget memory near the robot / everywhere.                                                |
| `/planner_selector`, `/controller_selector`                          | `std_msgs/String`, latched                | Pick the plugin for the next goal.                                                        |
| `/health/topics`                                                     | `perseus_interfaces/SystemHealth`         | 1 Hz.                                                                                     |
| `/mobility/status`                                                   | `perseus_interfaces/MobilityStatus`       | 10 Hz.                                                                                    |

Grid values follow `nav_msgs/OccupancyGrid`: `-1` unknown, `0` free, `1–99` a
roughness cost, `100` not drivable.

## How terrain classification works

`terrain_mapper` turns each depth frame into points (every `pixel_stride`th
pixel, depth between `min_depth_m` and `max_depth_m`), drops the robot's own
body and anything above `max_height_above_robot_m` (both in `base_link`), and
transforms the rest into `odom` using TF at the frame's timestamp. It then bins
the points into 5 cm cells.

For every cell with at least `min_points_per_cell` points:

1. **Floor height**: the `floor_percentile` (10th) of the cell's heights. A
   LiDAR would use the minimum, but stereo depth noise makes the minimum an
   outlier; a low percentile is robust to it.
2. **Ground estimate**: if the ground can slope by at most `max_slope_deg`, a
   neighbour _d_ metres away with floor _z_ says this cell's ground cannot be
   higher than _z + d·tan(max_slope)_. The lowest of those bounds, over a disc of
   radius `ground_window_m`, is the ground estimate. No plane fit and no flat-world
   assumption: a uniform slope, or a robot that is itself tilted on a dune, stays
   drivable.
3. **Three tests** against the ground estimate:
   - **step up**: the floor rises more than `max_step_up_m` above it (a rock face);
   - **step down**: the floor sits more than `max_step_down_m` below what its
     neighbours allow (a hole, an edge);
   - **height above ground**: the cell's `top_percentile` (90th) height,
     ignoring anything higher than `overhead_clearance_m`, stands more than
     `max_height_above_ground_m` proud (a post, a pipe on its side).
4. **Verdict**: past any threshold the cell is _not drivable_ (100). Otherwise
   its worst fraction of a threshold becomes a **roughness cost**: 0 below
   `rough_start_fraction`, rising to `rough_max_cost` just under a threshold.
   The planner then prefers flat sand when the detour is cheap, and RPP slows
   down on rough cells.
5. Obstacle blobs smaller than `min_obstacle_cells` (8-connected) are erased as
   speckle.

Consequences worth knowing:

- **Obstacles grow by up to `ground_window_m`.** The floor next to a rock sits
  below what the rock "allows", so step-down marks it too. Keep the window
  small (0.2 m); Nav2's inflation adds the rest.
- **A flat-topped step smaller than the slope allowance per cell can hide.** At
  5 cm cells and 25°, neighbouring cells may differ by 2.3 cm, so a 4 cm slab
  with its edge exactly on a cell boundary is mostly absorbed. A cell that
  straddles the edge holds both floor and top returns, and height-above-ground
  sees it. In practice edges straddle cells; lowering `max_slope_deg` sees
  smaller steps at the cost of reading steeper drivable slopes as rough.
- **Drop-offs often show as missing data, not low data.** Beyond an edge the
  camera may simply see nothing, and unknown ground is not an obstacle. See
  [Safety](#safety).

### Per-frame classification

The upstream stack classifies the last **second** of scans together: more
points per cell, and safe because its LiDAR-inertial odometry tracks the
sensor's full 3D motion. Perseus Lite's EKF is **2D**: TF cannot follow the
chassis bouncing, pitching or rolling on its rockers. Two frames a few hundred
milliseconds apart then disagree about the height of the same flat ground, and
classifying them together reads that disagreement as steps.

This was measured in Gazebo on flat ground, with the multi-frame window:

|                              | Flat-ground height spread at 2.5–3.5 m (95th percentile) | Rough / lethal cells in the local grid |
| ---------------------------- | -------------------------------------------------------- | -------------------------------------- |
| Robot still                  | ±5 mm                                                    | 16 / 80 (the rocks)                    |
| Spinning in place, 0.6 rad/s | up to +15 cm                                             | 1122 / 190                             |

Gazebo's ground truth showed the chassis bouncing ~3.5 cm vertically while
moving. The real robot on sand will do at least as much.

So by default (`point_buffer_s: 0`) heights are **only ever compared within one
depth frame**, where every point shares one camera pose. Temporal fusion
happens only in memory, which counts verdicts and never compares heights across
frames. With the same spin, per-frame mode kept the local grid at 38–47 rough
and 66–84 lethal cells, i.e. just the rocks. The price is sparser far coverage
in each frame, which is why `pixel_stride` defaults to 2 (≈77k points per frame).
Set `point_buffer_s > 0` only on a robot with a 6-DOF state estimate.

### Terrain memory

Every frame's verdicts are folded into a per-cell log-odds of "not drivable"
(`LogOddsMap`): `+log_odds_hit` for an obstacle verdict, `−log_odds_miss` for
a drivable one, clamped to `[log_odds_min, log_odds_max]`; a cell is reported
as an obstacle above `occupied_threshold`. Free cells remember their last
roughness cost. With the defaults (per frame, 10 Hz):

- a rock seen in **two** consecutive frames is remembered (0.8 > 0.5); a
  one-frame speckle (0.4) is not;
- a well-established rock is forgotten after **8** frames (~1 s) of the camera
  looking at the spot and seeing flat ground.

The memory is what keeps a rock on the map after it leaves the camera's
narrow (~73°) field of view. It starts at `initial_size_m` around the first
observation, grows in `growth_step_m` steps up to `max_size_m`, and lives in
`odom` (continuous, so it never jumps when SLAM corrects `map → odom`).

Every memory publish forces the cells under the (padded) footprint to free:
nothing can be under the robot, the camera cannot see there to clear a stale
verdict, and a remembered obstacle under the footprint would fail every plan
with `START_OCCUPIED`.

:::{warning}
Because the memory lives in `odom`, it inherits odometry drift. The lite's EKF
yaw currently comes from wheel odometry only (the IMU input is disabled), and
skid-steer yaw on sand drifts. After a long run with lots of turning, remembered
rocks can smear or double. Use `~/clear_all` between runs, and consider
`global_frame: map` (untested; see [Future work](#future-work)).
:::

## Nav2 integration

### The overlay

`autonomy/config/terrain_nav_params.yaml` is applied **on top of**
`perseus_nav_params.yaml` through `perseus_nav_bringup.launch.py`'s new
`overlay_params_file` argument. It defaults to an inert file
(`nav_overlay_none.yaml`), so without it every node gets exactly the base
configuration. Later files win per key, and a list (such as `plugins`) replaces
the base list wholesale, so the overlay's lists repeat the base entries.

| Costmap                     | Layers with terrain on                                                                     |
| --------------------------- | ------------------------------------------------------------------------------------------ |
| local (`odom`, rolling 3 m) | `obstacle_layer`, **`terrain_local_layer`**, **`terrain_memory_layer`**, `inflation_layer` |
| global (`map`)              | `static_layer`, `obstacle_layer`, **`terrain_memory_layer`**, `inflation_layer`            |

### The `TerrainGridLayer` plugin

`perseus_terrain::TerrainGridLayer` copies a terrain grid into a costmap. For
each costmap cell it looks up the grid through TF (so the odom-frame memory
lands correctly in the map-frame global costmap and moves with each SLAM
correction), takes the **worst** terrain cell under it (a 5 cm rock never falls
between the 10 cm costmap's samples), maps `100` to lethal and `1–99` linearly
onto `1–252`, and combines with what is already there by maximum.

| Parameter          | Default                | Meaning                                                                                                                                                                        |
| ------------------ | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `enabled`          | `true`                 |                                                                                                                                                                                |
| `topic`            | `/terrain/global_grid` |                                                                                                                                                                                |
| `transient_local`  | `true`                 | **Must match the publisher**: `true` for the memory, `false` for the local grid. A transient-local subscriber never receives from a volatile publisher, and nothing is logged. |
| `max_age_s`        | `0.0`                  | Ignore a grid older than this (0 = never). The local layer uses 1.0, so a dead camera stops counting instead of freezing its last view in front of the robot.                  |
| `lethal_threshold` | `100`                  | Grid values at or above this are lethal. Lower it to make rough cells lethal.                                                                                                  |

Clearing a Nav2 costmap does **not** clear terrain memory: the layer copies it
straight back on the next update. That is what `~/clear_around_robot` is for.

### Planners and controllers

The overlay loads several plugins; the behaviour tree picks one of each **per
goal** from latched topics, with no restart. With nothing published the
defaults are the base configuration's `GridBased` and `FollowPath`.

```shell
ros2 topic pub -1 /planner_selector std_msgs/String "{data: ThetaStar}"
ros2 topic pub -1 /controller_selector std_msgs/String "{data: RPP}"
```

| Planner (`planner_id`) | Plugin                | Good at                                                                                    | Weak at                            |
| ---------------------- | --------------------- | ------------------------------------------------------------------------------------------ | ---------------------------------- |
| `GridBased` (default)  | NavFn                 | Always finds a path if one exists; robot-validated                                         | 45°/90° grid steps → more pivoting |
| `ThetaStar`            | Theta\* any-angle A\* | Straight, short paths with few, gentle turns; `w_traversal_cost` 4 lets roughness steer it | Ignores heading; pivots at start   |
| `Smac2D`               | Smac 2D A\*           | Cost-aware, shortest of the three in tests; good fallback in clutter                       | Ignores heading                    |

| Controller (`controller_id`) | Plugin                 | Notes                                                                                                                 |
| ---------------------------- | ---------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `FollowPath` (default)       | DWB                    | The robot-validated base tuning, unchanged. Steers around obstacles near its footprint.                               |
| `RPP`                        | Regulated Pure Pursuit | Smooth arcs, few knobs, slows on high-cost (including rough) cells. Never steers off the path: prefer DWB in clutter. |

Compare planners on one goal without driving, from the robot's current pose:

```shell
ros2 run autonomy plan_probe.py 3.0 1.5
ros2 run autonomy plan_probe.py 3.0 1.5 --planners ThetaStar Smac2D --frame odom
```

A detour ratio much above 1.5 on open ground means something is in the way:
check `/terrain/global_grid` for a false obstacle before blaming the planner.

#### Adding the Lattice planner

Smac State Lattice plans in x, y **and heading** and checks the real footprint,
but needs motion primitives at the global costmap's resolution (0.10 m); Nav2
ships only 0.05 m sets. Generate a set once with Nav2's own generator
(`nav2_smac_planner/lattice_primitives` in the
[navigation2 repository](https://github.com/ros-navigation/navigation2), jazzy
branch; needs `numpy`, `matplotlib`, `rtree`), with a `config.json` of
`{"motion_model": "diff", "turning_radius": 0.5, "grid_resolution": 0.1,
"stopping_threshold": 5, "num_of_headings": 16}`, save the output as
`autonomy/config/lattice/diff_10cm_0.5m.json`, then add `Lattice` to
`planner_plugins` in the overlay:

```yaml
Lattice:
  plugin: "nav2_smac_planner::SmacPlannerLattice"
  lattice_filepath: "$(find-pkg-share autonomy)/config/lattice/diff_10cm_0.5m.json"
  allow_unknown: true
  tolerance: 0.25
  max_planning_time: 3.0
  rotation_penalty: 5.0
  allow_reverse_expansion: false
  smooth_path: true
```

### The behaviour tree

`navigate_to_pose_terrain.xml` is Nav2's stock replan-and-recover tree plus:

1. **Selectors**: `PlannerSelector` / `ControllerSelector` on the topics above.
2. **Smoothing**: every new path goes through `simple_smoother`
   (0.2 s budget, collision-checked). If smoothing fails, `AlwaysSuccess`
   carries on with the raw path: degraded, never broken.
3. **Terrain-aware recovery**: on `START_OCCUPIED` (205) or `GOAL_OCCUPIED`
   (206), which stock Nav2 does not try to recover from, and as the last step
   of the general recovery cycle, the tree forgets terrain memory around the
   robot, waits 1 s for the memory to republish, then clears the Nav2 costmap.

:::{important}
Nav2 refuses to load a tree whose service nodes cannot reach their service:
it waits 1 s per service at activation, then throws. The terrain tree calls
`terrain_mapper/clear_around_robot`, so **`terrain_mapper` must be running
before Nav2 activates**. The launch files start it first; never use the
terrain overlay without `terrain_mapper`.
:::

## Monitoring

Both monitors are passive: they never command the robot. They start by
default (`monitoring:=true`) with the SLAM + Nav2 launches.

### Topic health (`topic_health_monitor`)

Subscribes to each listed topic generically (its type is read from the ROS
graph, matching the publishers' reliability) and publishes one
`perseus_interfaces/SystemHealth` per second on `/health/topics`:

| Status             | Meaning                                                        |
| ------------------ | -------------------------------------------------------------- |
| `OK` (0)           | Publishing at ≥ `rate_tolerance` × expected.                   |
| `SLOW` (1)         | Publishing, but below that.                                    |
| `STALE` (2)        | A publisher exists, but nothing arrived for `stale_timeout_s`. |
| `NO_PUBLISHER` (3) | Nobody publishes it.                                           |

Status changes are also logged. Adding a topic is a config change: append it
and its expected rate to `topics` / `expected_rates_hz` in
`perseus_monitoring/config/topic_health.yaml` (core stack) or
`topic_health_terrain.yaml` (core + camera + terrain, used with `terrain:=true`).
Subscribing costs the topic's bandwidth, so watch a small companion topic
instead of a big one (`/oak/stereo/camera_info`, not the depth image).

```shell
ros2 topic echo /health/topics --once
```

### Mobility (`mobility_monitor`)

The lite's EKF fuses wheel speed, so its velocity says what the **wheels** did,
not the body. Comparing a command against it can never show slip. The monitor
therefore makes independent comparisons, each a sliding-window mean of the
shortfall `1 − measured / commanded`, evaluated only after a grace period of
steady command and only above a minimum command:

| Channel         | Commanded               | Measured               | A fault means                                                                                                              |
| --------------- | ----------------------- | ---------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `drive_linear`  | `/cmd_vel` linear x     | wheel odometry         | Servos not reaching the commanded speed: bogged, stalled, current-limited, or a servo off the bus.                         |
| `drive_angular` | `/cmd_vel` angular z    | wheel odometry         | Same, for turning.                                                                                                         |
| `yaw_slip`      | wheel-odometry yaw rate | IMU gyro z             | The wheels say "turning", the body is not: wheels spinning in sand.                                                        |
| `linear_slip`   | wheel speed             | `reference_odom_topic` | The wheels say "moving", the body is not. **Off by default**: needs an independent velocity source (e.g. visual odometry). |

`MobilityStatus.stalled` is set by a drive channel, `slipping` by a slip
channel. Thresholds are in `perseus_monitoring/config/mobility_monitor.yaml`
(default: fault at 50 % shortfall over 1 s, after 0.6 s of command).

In Gazebo, commanding a 0.6 rad/s spin faulted `yaw_slip` after ~3.5 s
(shortfall 61–72 %) while `drive_angular` stayed healthy: the servos delivered
the command, but the simulated body barely turned. That is the sim's real,
pre-existing turning problem (see [Known issues](#known-issues)).

## Running it

### On the robot

```shell
# Terminal 1: the camera (separate Pixi env; see the OAK-D Lite page)
pixi run -e camera oak

# Terminal 2: bringup + SLAM + Nav2 with terrain
pixi shell
cd software/ros_ws && source install/setup.bash
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py terrain:=true
```

The TUI's **SLAM + Nav2** and **Autonomy stack** profiles have _Terrain
mapping_ and _Health + mobility monitors_ options, and an **OAK-D Lite camera
driver** task (Sensors group).

### In simulation

The URDF's Gazebo `rgbd_camera` stands in for the OAK-D Lite and publishes the
same topics in the same frame, so nothing else changes. The `terrain_test`
world is a walled yard with features chosen to exercise the classifier:

| Feature                                                             | Expected verdict                                      |
| ------------------------------------------------------------------- | ----------------------------------------------------- |
| `low_rock_*`: four 8 cm rocks, below the lidar's nominal scan plane | Not drivable                                          |
| `step_too_tall`: a 15 cm step                                       | Edge not drivable, flat top drivable                  |
| `dune_*_12deg`: a 12° ridge (raised slab)                           | Slope drivable; slab sides (up to 12 cm) not drivable |

```shell
pixi shell -e simulation
cd software/ros_ws && source install/setup.bash
ros2 launch perseus_lite_simulation perseus_sim.launch.py gz_world:=terrain_test.sdf \
    initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.1
# second terminal (same env):
ros2 launch autonomy autonomy.launch.py terrain:=true terrain_debug:=true
```

`headless:=true` on the sim launch runs the Gazebo server only (EGL rendering,
no GUI, no RViz), for CI or a machine without a display; it needs access to
`/dev/dri/renderD*` (the `render` group). The sim RViz config shows
**Terrain Memory** (on), **Terrain Local** (off), the OAK-D depth image, and
the debug points.

### Launch arguments

| Launch file                                                  | Argument                                               | Default                        | Effect                                                                             |
| ------------------------------------------------------------ | ------------------------------------------------------ | ------------------------------ | ---------------------------------------------------------------------------------- |
| `perseus_lite_slam_and_nav2.launch.py`, `autonomy.launch.py` | `terrain`                                              | `false`                        | Start `terrain_mapper` (before Nav2) and apply the terrain overlay.                |
|                                                              | `terrain_debug`                                        | `false`                        | `terrain_mapper` publishes `/terrain/debug/*`.                                     |
|                                                              | `monitoring`                                           | `true`                         | Start both monitors (watching camera and terrain topics too when `terrain:=true`). |
| `perseus_nav_bringup.launch.py`                              | `overlay_params_file`                                  | `config/nav_overlay_none.yaml` | Parameter file applied on top of `params_file`.                                    |
| `perseus_terrain/terrain.launch.py`                          | `terrain_params_file`, `publish_debug`, `use_sim_time` |                                | Run the mapper alone.                                                              |
| `perseus_monitoring/monitoring.launch.py`                    | `terrain`, `use_sim_time`                              |                                | Run the monitors alone.                                                            |
| `perseus_lite_simulation/perseus_sim.launch.py`              | `headless`                                             | `false`                        | Gazebo server only, no RViz.                                                       |

## Configuration reference

All `terrain_mapper` parameters live in `perseus_terrain/config/terrain_mapper.yaml`,
each with a comment explaining its value.

| Parameter                                                             | Default                        | What it does                                                |
| --------------------------------------------------------------------- | ------------------------------ | ----------------------------------------------------------- |
| `input_type`                                                          | `depth`                        | `depth` (image + camera_info, cheapest) or `pointcloud`.    |
| `depth_topic`, `camera_info_topic`, `pointcloud_topic`                | `/oak/stereo/…`, `/oak/points` | Inputs.                                                     |
| `max_input_rate_hz`                                                   | 10                             | Skip frames above this rate.                                |
| `pixel_stride` / `cloud_point_stride`                                 | 2 / 1                          | Decimation.                                                 |
| `min_depth_m`, `max_depth_m`                                          | 0.3, 3.0                       | Trusted depth range.                                        |
| `global_frame`, `robot_frame`                                         | `odom`, `base_link`            | Grid frame and robot frame.                                 |
| `transform_wait_s`                                                    | 0.5                            | How long a frame may wait for its TF.                       |
| `body_crop_min`, `body_crop_max`                                      | robot box                      | Points inside (in `base_link`) are the robot.               |
| `max_height_above_robot_m`                                            | 1.0                            | Ceiling crop (in `base_link`).                              |
| `resolution_m`                                                        | 0.05                           | Cell size.                                                  |
| `point_buffer_s`                                                      | 0.0                            | 0 = per-frame; > 0 = multi-frame window (6-DOF state only). |
| `local_half_extent_m`                                                 | 4.0                            | Local grid half-size.                                       |
| `update_rate_hz`, `memory_rate_hz`                                    | 10, 1                          | Classification rate (≥ camera rate) / memory publish rate.  |
| `clear_under_robot`, `footprint`, `footprint_padding_m`               | on, lite footprint, 0.05       | Self-clearing.                                              |
| `clear_radius_m`                                                      | 1.5                            | Radius for `~/clear_around_robot`.                          |
| `publish_debug`                                                       | false                          | Debug layers and points.                                    |
| `classifier.max_step_up_m`                                            | 0.06                           | Rock faces taller than this.                                |
| `classifier.max_step_down_m`                                          | 0.08                           | Holes and edges deeper than this (0 disables).              |
| `classifier.max_height_above_ground_m`                                | 0.06                           | Things standing taller than this in a cell.                 |
| `classifier.overhead_clearance_m`                                     | 0.45                           | Returns higher than this above ground are ignored.          |
| `classifier.max_slope_deg`                                            | 25                             | Steepest slope that is still ground.                        |
| `classifier.ground_window_m`                                          | 0.2                            | Neighbourhood of the ground estimate.                       |
| `classifier.min_points_per_cell`, `min_obstacle_cells`                | 3, 2                           | Unknown below / speckle filter.                             |
| `classifier.floor_percentile`, `top_percentile`                       | 0.1, 0.9                       | Robust floor / top heights.                                 |
| `classifier.rough_start_fraction`, `rough_max_cost`                   | 0.5, 60                        | Roughness cost ramp.                                        |
| `memory.log_odds_hit` / `miss` / `min` / `max` / `occupied_threshold` | 0.4 / 0.2 / −1 / 2 / 0.5       | Per-frame evidence weights.                                 |
| `memory.initial_size_m`, `growth_step_m`, `max_size_m`                | 20, 5, 200                     | Memory allocation.                                          |

## Tuning

Tune against a recorded bag or the simulator before touching the robot, and
change one thing at a time.

1. Launch with `terrain_debug:=true` and show `/terrain/debug/points` and the
   debug layers in RViz (50 = at threshold, 100 = twice it).
2. **Is the projection right?** The debug points of flat ground should lie flat
   and line up with the lidar scan. If they tilt or float, the camera mount in
   `perseus_lite_description/urdf/sensors.urdf.xacro` (`oak_mount_*`) is wrong:
   measure it on the robot.
3. **False obstacles on flat sand**: look at which debug layer fires. Noise far
   out means `max_depth_m` is too long. Step-up/height speckle means raising
   `min_points_per_cell` or `min_obstacle_cells`, or (if driving makes it worse)
   check that `point_buffer_s` is 0.
4. **Missed rocks**: lower the matching threshold, or `max_slope_deg` (see the
   slope-allowance note above).
5. **Memory too sticky / too forgetful**: `log_odds_max / log_odds_miss` is the
   number of clear frames needed to forget; `log_odds_hit` vs
   `occupied_threshold` is how many frames to remember.
6. Copy what works into the YAML with a comment saying why, and what bag or
   measurement it came from.

Useful checks:

```shell
ros2 topic echo /health/topics --once          # is every input alive?
ros2 topic hz /terrain/local_grid               # one per depth frame (~10 Hz)
ros2 topic echo /mobility/status                # stalled? slipping?
ros2 service call /terrain_mapper/clear_all nav2_msgs/srv/ClearEntireCostmap
ros2 run autonomy plan_probe.py 3.0 0.0         # compare planners
```

## Verification

Done in Gazebo (`terrain_test` world, headless, a CUDA desktop), and with unit
tests:

- **Unit tests** (gtest, run by `pixi run -e default test`): classifier (flat,
  tilted robot, slopes either side of the limit, rocks, holes, disabling
  step-down, sparse cells, overhead returns, thin posts, stereo noise,
  roughness cost, out-of-grid points, speckle removal), memory (one sighting,
  forgetting count, unknowns, roughness, growth, maximum size, resolution
  mismatch, forget/mark-free/reset), depth projection (both encodings, invalid
  pixels, stride, big-endian, bad input), cost mapping and geometry; monitors
  (slip detector and topic tracker); sim bridge has the OAK-D topics.
- **Detection**: all four low rocks were in memory at their true positions
  from a standing start; the 12° dune read drivable up to its (occluded) crest;
  the step's edge and sides read lethal and its flat top free.
- **Lidar comparison**: the lidar did not see rocks 2 and 3. It did clip rocks 0
  and 1 at 1.5–2 m, because the simulated chassis sits slightly nose-down and
  its scan plane dips; on the robot the C1's mount height and tilt decide this.
- **Costmaps**: both layers loaded; the first ThetaStar plan through the rock
  garden kept 0.33–0.57 m from every rock.
- **Live switching**: the same goal produced a 68-pose path with `GridBased`,
  31 with `ThetaStar`, then 68 again, without restarting anything.
  `plan_probe.py` around the step: NavFn 3.99 m (×1.73), ThetaStar 3.43 m
  (×1.52), Smac2D 3.27 m (×1.42).
- **Monitors**: all nine topics OK in `/health/topics`; `yaw_slip` faulted on a
  commanded spin, as above.

**Not yet verified on hardware**: the OAK-D Lite driver parameters (the driver
loads and starts in the `camera` env, but validates its pipeline only once a
device is connected), the camera mount pose, the classifier thresholds on real
sand and stereo noise, and the mobility thresholds on the real servos. End-to-end
_driving_ to a goal could not be validated in simulation (see below).

## Known issues

Found while validating this feature; pre-existing, and **not** fixed here:

- **Sim wheel separation.** The sim uses the robot's sand-calibrated effective
  `wheel_separation` (0.8286 m). In Gazebo the effective value is ~0.35 m, so
  sim odometry under-reports yaw ~2.4×.
- **Sim skid-steer turning.** Gazebo does not turn the robot in place: a
  0.6 rad/s spin command (correct, equal and opposite wheel speeds) moved the
  body ~0.8 m forward and turned it ~7°. Any sim test that turns therefore has
  badly wrong odometry, so `odom`-frame memory smears and Nav2 goals fail.
- **SLAM in sim** did not correct `map → odom` in these runs, so the odometry
  errors above were not rescued.

One sim defect **was** fixed because it blocked everything: the lidar and IMU
Gazebo sensors used `<frame>`, which Gazebo Harmonic ignores, so their messages
were stamped `perseus_lite/base_link/laser` / `…/imu`. Those frames are not in
TF, so slam_toolbox dropped every scan and `map` never appeared. They now use
`<gz_frame_id>` (see `ERRORS.md`).

## Safety

`terrain:=true` changes what Nav2 believes, not who is in charge. `twist_mux`
still ranks the gamepad (deadman) above Nav2, and the e-stop is still the one to
rely on.

- **Unknown ground is not an obstacle.** A drop-off often shows as _no_ depth,
  so the terrain layers cannot stop the robot driving off an edge they never saw.
  Keep speed low near edges.
- **The camera looks forward only.** Reversing (DWB can back up; recoveries back
  up 0.30 m) relies on memory and the lidar. Keep the area behind the robot clear.
- **A dead camera** makes the local layer expire after 1 s and the health monitor
  report `/oak/stereo/camera_info` stale; memory keeps its last state.

## Troubleshooting

| Symptom                                                                                    | Likely cause                                                                                                                                                               |
| ------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Nav2 fails to activate; `"terrain_mapper/clear_around_robot" service server not available` | Terrain overlay without a running `terrain_mapper`, or it started too late. Use `terrain:=true` (which starts it first).                                                   |
| `Depth image received but no camera_info yet`                                              | `camera_info_topic` wrong, or the driver is not publishing it.                                                                                                             |
| `No transform odom <- oak_right_camera_optical_frame`                                      | URDF without the OAK-D (old build), robot_state_publisher not running, or the driver's frame name differs (check `ros2 topic echo --once --no-arr /oak/stereo/image_raw`). |
| `Unsupported depth encoding`                                                               | Driver publishing disparity or RGB on the depth topic; expect 16UC1 or 32FC1.                                                                                              |
| `/terrain/*` publishing but nothing in the costmaps                                        | Layer `transient_local` does not match the publisher (memory: true, local: false).                                                                                         |
| Flat sand reads rough or lethal while driving                                              | `point_buffer_s` > 0 on the lite, or `max_depth_m` too long.                                                                                                               |
| Rocks smeared or doubled after a long run                                                  | Odometry yaw drift (memory is in `odom`). `~/clear_all`; see the warning in [Terrain memory](#terrain-memory).                                                             |
| Planner fails with 205/206 repeatedly                                                      | A stale remembered obstacle at the robot or goal; the tree clears memory around the robot, but not around a distant goal. `~/clear_all`.                                   |
| `yaw_slip` faults on firm ground                                                           | `wheel_separation` no longer matches the surface (the 0.8286 value is calibrated for sand), or IMU bias.                                                                   |

## Future work

- **Visual odometry** from the OAK-D's stereo pair (e.g. Isaac ROS cuVSLAM on the
  Jetson, see [Isaac ROS](isaac-ros.md)) would give the EKF a second, slip-free
  velocity (enabling `linear_slip`) and, with 6-DOF output, allow the denser
  multi-frame window.
- **IMU orientation** (roll/pitch) in TF would remove most of the per-frame
  restriction.
- **Memory in `map`**: `global_frame: map` is supported by the code (and the
  layer handles any frame) but untested; it trades odometry drift for small
  jumps on SLAM corrections.
- **Lattice planner** primitives, as above.
- **Sim fidelity**: fix the sim wheel separation and turning (see
  [Known issues](#known-issues)) so driving can be validated end to end.
