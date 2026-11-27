# AprilTag landmark localisation

SLAM gives Perseus Lite a consistent map, but that map is anchored wherever the
robot happened to start: its origin and heading change every run. Landmark
localisation ties it to **fixed, surveyed coordinates** using AprilTags at known
places:

- **One tag near where the rover starts** is enough to know where the rover is in
  your own coordinates.
- **Tags at destinations** re-anchor the estimate on arrival, correcting the drift
  accumulated on the way.
- The rover can **search** for tags (turn in place, stop and look, then visit
  search points), and **drive to named destinations**.

It is opt-in (`landmarks:=true`) and uses the OAK-D Lite (see
[OAK-D Lite](../hardware/oak-d-lite.md)).

```{contents}
:local:
:depth: 2
```

## How it fits in

```{graphviz}
:caption: Landmark localisation (new parts in pink)
:align: center

digraph landmarks {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto", nodesep=0.3, ranksep=0.5];
    node  [fontname="Roboto", fontsize=10, style="filled,rounded", shape=box, fontcolor="white", fillcolor="#37474f"];
    edge  [fontname="Roboto", fontsize=8, color="#7a6cad"];

    cam  [label="OAK-D Lite\nright rectified mono"];
    det  [label="tag_detector\nOpenCV AprilTag + IPPE", fillcolor="#ec407a"];
    loc  [label="landmark_localiser\nsite → map", fillcolor="#ec407a"];
    nav  [label="landmark_navigator\nsearch / go_to_destination", fillcolor="#ec407a"];
    map  [label="landmarks.yaml\ntags, destinations,\nsearch points (site)", shape=note, fillcolor="#5e35b1"];
    tf   [label="TF\nsite → map → odom → base_link"];
    nav2 [label="Nav2\nSpin, NavigateToPose"];

    cam -> det [label="/oak/right_rect/image_raw"];
    det -> loc [label="/landmarks/detections"];
    det -> nav [label="/landmarks/detections"];
    map -> det; map -> loc; map -> nav;
    loc -> tf [label="site → map"];
    nav -> loc [label="~/calibrate", style=dashed];
    nav -> nav2 [label="spin, goals in site"];
}
```

### Frames: why a `site` frame

The TF contract has exactly one owner per edge: slam_toolbox owns `map → odom`
and the EKF owns `odom → base_link`. Tag localisation must not fight either. So
it adds a **parent** frame instead:

```text
site ──landmark_localiser──▶ map ──slam_toolbox──▶ odom ──EKF──▶ base_link
```

`site` is your surveyed coordinate system (the one the landmark map is written
in). The localiser estimates where the SLAM map sits in it and publishes
`site → map`. Nothing about SLAM, the EKF or Nav2's frames changes. A Nav2 goal
given in `site` is transformed through TF automatically, so destinations "just
work" once calibrated. Until the first fix, `site → map` is **not published**, so
goals in `site` fail rather than go somewhere wrong.

## The landmark map

One YAML file (`landmarks_file`) describes everything, in the site frame.
`perseus_landmarks/config/landmarks.yaml` is a template;
`landmarks_sim.yaml` matches the Gazebo `terrain_test` world.

```yaml
site_frame: site
tag_family: 36h11 # 16h5, 25h9, 36h10 or 36h11
default_tag_size_m: 0.16 # edge of the BLACK square

tags:
  # [x, y, z, yaw_deg] of the tag centre; yaw = the direction the tag FACES
  # (its outward normal). A tag readable by someone looking west faces east: 0.
  - { id: 0, pose: [2.0, 0.0, 0.25, 180.0] }
  - { id: 1, pose: [4.88, 2.5, 0.25, 180.0], size_m: 0.20 }

destinations:
  # [x, y, yaw_deg]; "tag" (optional) is visible from the destination
  - { name: east_dock, pose: [4.0, 2.5, 0.0], tag: 1 }

search_points:
  # where to look if a full turn finds nothing. frame "start": relative to the
  # robot when the search began (x forward, y left); "site": site coordinates
  - { xy: [1.0, 0.0] }
  - { xy: [0.0, 2.0], frame: site }
```

Survey tips:

- **Print `tag36h11`** tags. Measure `size_m` on the print (the black square), not
  the paper.
- Mount tags **upright at roughly camera height** (0.2–0.35 m). The camera is
  pitched 20° down, so tags much higher fall out of view up close.
- **Measure positions to the tag centre**, and the facing direction as carefully
  as you can for any tag that will be seen on its own (see
  [Accuracy](#accuracy)).
- **Two tags in view at once**, at least 0.5 m apart, is much better than one: the
  heading then comes from their positions, not from a tag's orientation.

## How a fix is computed

For each image, `tag_detector` finds tags (OpenCV's AprilTag dictionaries, subpixel
corners) and solves each tag's pose with **IPPE**, a square-specific PnP. IPPE
returns two poses: a planar square seen nearly head-on (or small and far) fits two
mirror-tilted poses almost equally well. The detector:

1. drops any pose in which the tag faces **away** from the camera (impossible for a
   printed tag, but IPPE returns it, and on the optical axis it can even rank first);
2. reports the **ambiguity ratio**, the second-best pose's reprojection error over
   the best. Below about 3, the tag's **orientation** is untrustworthy; its
   **position** is good either way.

`landmark_localiser` then:

- transforms each sighting into the SLAM map using TF **at the image's timestamp**;
- gates it: known id, range ≤ `max_range_m`, viewing angle ≤ `max_view_angle_deg`,
  reprojection ≤ `max_reprojection_px`, and the robot not turning faster than
  `max_yaw_rate_rad_s` (stop-and-look);
- weights it by `cos(view angle) / (1 + distance²)`;
- solves `site → map` over a sliding window:
  - **one distinct tag**: heading from the tag's facing (only unambiguous sightings
    count, ratio ≥ `min_ambiguity_ratio`), position from its position. With no
    unambiguous sighting yet, **no fix** rather than a wrong one;
  - **two or more tags ≥ `min_tag_separation_m` apart**: a weighted 2D rigid fit of
    positions only (closed-form Kabsch), which needs no orientations at all;
- rejects fixes whose residual (how far the observed tags land from their surveyed
  positions) exceeds `max_residual_m`: a moved tag, or a misdetection;
- **blends** each accepted fix into the estimate (`blend_alpha`). A fix that
  disagrees by more than `max_jump_m` / `max_jump_yaw_deg` is a **jump** and is
  only accepted once it has persisted for `jump_confirm_s`, so a single bad sighting
  cannot yank the site frame, but a real change (SLAM drift, a re-surveyed tag)
  eventually wins.

The first fix is taken as soon as `estimator.min_observations` good sightings are in
the window, so a robot that starts with a tag in view localises within seconds,
with no action needed.

## Searching for tags

`/landmarks/search` (`perseus_interfaces/action/SearchForLandmarks`):

| Goal field          | Meaning                                                                                   |
| ------------------- | ----------------------------------------------------------------------------------------- |
| `tag_ids`           | Tags to look for; empty = any tag in the landmark map.                                    |
| `max_rotation_rad`  | Rotation allowed at each search point (0 = `max_rotation_deg`, default 360°).             |
| `use_search_points` | If turning in place finds nothing, visit the landmark map's search points and look again. |
| `calibrate`         | Only succeed once a stationary fix (`~/calibrate`) has been taken from what was found.    |

The search **stops and looks**: it turns `step_deg` (45°) with Nav2's Spin
behaviour, waits `dwell_s` (1.5 s) watching for the targets, and repeats. Every
motion goes through Nav2 (Spin is collision-checked against the local costmap;
search points are ordinary NavigateToPose goals), so the gamepad deadman and the
collision monitor always apply. If Spin refuses (for example `Collision Ahead`),
the search looks from where it is and moves on.

What counts as **found** is deliberately strict, because each looser rule failed in
simulation:

- A sighting only counts if the localiser would accept it: within `max_range_m`,
  `max_view_angle_deg` and `max_reprojection_px` (keep these equal to the
  localiser's gates). A tag seen at 5.5 m, or at 63° off its normal, used to stop
  the search and then fail calibration with "0 usable sightings".
- A target must be seen `min_sightings` (3) times within `seen_timeout_s` (1 s). A
  tag flickering at the edge of the view is not found.
- The search **watches while turning**: at the first usable glimpse it cancels the
  Spin. Spin overshoots its target by tens of degrees, enough to carry a tag out of
  the 73° view, so if the tag is gone after the stop, the robot turns back to the
  heading where it was glimpsed and looks again.
- With `calibrate: true`, a search only succeeds once the fix is taken; if
  calibration fails (too few usable sightings), it keeps turning. The failure
  message lists the rejections by reason (too far, too oblique, poor corner fit,
  while turning).

```shell
# find any known tag and calibrate from it
ros2 action send_goal -f /landmarks/search perseus_interfaces/action/SearchForLandmarks \
    "{calibrate: true}"
# find tag 3, allowing a drive to the search points
ros2 action send_goal -f /landmarks/search perseus_interfaces/action/SearchForLandmarks \
    "{tag_ids: [3], use_search_points: true}"
```

With `search_on_start:=true` the navigator runs a search-and-calibrate on its own,
`startup_delay_s` (15 s) after starting. **This turns the robot without a command**;
it is off by default.

## Driving to destinations

`/landmarks/go_to_destination` (`perseus_interfaces/action/GoToDestination`):

1. If not calibrated (and `auto_localise`), **search and calibrate** first.
2. **Drive** to the destination's pose in the site frame (NavigateToPose).
3. If the destination has a tag: if it is not in view, **search** for it (turn in
   place); take a **stationary fix** from it (`~/calibrate`); and if the corrected
   estimate puts the robot further than `refine_tolerance_m` from the destination,
   **drive the last bit again**. (Waiting for the live estimate to absorb the tag
   is not enough: drift accumulated on the way is often bigger than `max_jump_m`,
   which the live path only accepts after `jump_confirm_s`, so the error was measured
   against the stale estimate and the refine skipped.)
4. Report the remaining error by the (tag-corrected) estimate.

```shell
ros2 action send_goal -f /landmarks/go_to_destination \
    perseus_interfaces/action/GoToDestination "{destination: east_dock}"
```

Only one landmark action runs at a time (they all move the robot); a second goal is
rejected.

## Running it

On the robot (the camera runs in its own Pixi environment):

```shell
pixi run -e camera oak                     # terminal 1
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py \
    landmarks:=true landmarks_file:=/path/to/your_site.yaml     # terminal 2
```

In simulation (the `terrain_test` world has tags 0–3 and `site` = the Gazebo world
frame, so the estimate can be checked against ground truth):

```shell
ros2 launch perseus_lite_simulation perseus_sim.launch.py gz_world:=terrain_test.sdf \
    initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.1
ros2 launch autonomy autonomy.launch.py landmarks:=true
```

| Launch argument   | Default                                            | Effect                                                            |
| ----------------- | -------------------------------------------------- | ----------------------------------------------------------------- |
| `landmarks`       | `false`                                            | Start `tag_detector`, `landmark_localiser`, `landmark_navigator`. |
| `landmarks_file`  | robot: `landmarks.yaml`; sim: `landmarks_sim.yaml` | The landmark map.                                                 |
| `search_on_start` | `false`                                            | Search and calibrate at startup (moves the robot).                |

The TUI's **SLAM + Nav2** and **Autonomy stack** profiles have _AprilTag
localisation_ and _Search for a tag at start_ options. With monitoring on, the health
monitor also watches `/landmarks/detections` (the tag camera's liveness: one message
per processed image, even empty) and `/landmarks/localisation`.

Useful checks:

```shell
ros2 topic echo /landmarks/localisation       # calibrated? residual? accepted/rejected
ros2 topic echo /landmarks/detections         # what the camera sees, per tag
ros2 service call /landmark_localiser/calibrate perseus_interfaces/srv/CalibrateFromLandmarks "{}"
ros2 service call /landmark_localiser/reset std_srvs/srv/Trigger
ros2 run tf2_ros tf2_echo site base_link      # where the robot is, in site coordinates
```

In RViz, `/landmarks/markers` (transient local) draws the surveyed tags and
destinations in the site frame, and `/landmarks/debug/image` (with
`landmarks_debug:=true`) shows detections with their axes.

## Accuracy

**Detector** (synthetic renders through the OAK-D Lite's 480P intrinsics, 16 cm tag,
`apriltag` vs default `subpix` corner refinement): positions stayed within
**3 cm (`apriltag`) and 6 cm (`subpix`) out to 2.5 m**. `subpix` kept detecting to 4 m and at 60°;
`apriltag` refinement has tighter corners but stopped detecting at 3 m beyond 20°
and at 4 m entirely. Tag facings were within ~3° whenever the ambiguity ratio was
≥ 3; below ~2 they were off by tens of degrees, including 180° flips. Between 2 and 3,
MuJoCo measured heading errors of 0.7° (ratio 2.56) and 2.4° (2.80), so
`min_ambiguity_ratio` is 2.0: at 3.0 a single tag seen cleanly at 59° (ratio 2.80)
could never calibrate. For longer range, use bigger tags: range scales with tag size.

**End to end in Gazebo** (`terrain_test`, single tag 0, robot still, estimate of the
robot's pose in `site` against Gazebo ground truth):

| Spawn (x, y, yaw) | Tag distance | Calibrated `site → map` | Robot pose error |
| ----------------- | ------------ | ----------------------- | ---------------- |
| (0, 0, 0°)        | 2.1 m        | (−0.068, 0.050, −0.6°)  | 8.5 cm, 0.6°     |
| (0.5, 0.5, −30°)  | 1.95 m       | (0.467, 0.506, −29.6°)  | 3.4 cm, 0.4°     |

**End to end in MuJoCo** ([MuJoCo simulation](mujoco-simulation.md), which, unlike
Gazebo, turns this robot in place; robot pose in `site` against MuJoCo ground truth):

| Test                                                           | Result                                                                                |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| Calibrate, still, single tag 0 at 1.3–2.0 m, four spawns       | 0.9–4.0 cm, 0.1–0.7°                                                                  |
| Calibrate, still, tag 0 at 59° off-normal                      | 9.2 cm, 2.4°                                                                          |
| Search + calibrate, spawned facing away from every tag         | Found tag 0 after two turns; 1.0–2.3 cm, 0.2–0.6°                                     |
| `go_to_destination start` from (0, −1.5, 180°), uncalibrated   | Search, calibrate, drive, re-anchor: 17 cm from the destination, estimate within 3 cm |
| `go_to_destination north_bay` from (0.5, −1, 0°)               | 6 cm from the destination, estimate within 1.6 cm                                     |
| `go_to_destination east_dock` from the origin, `terrain:=true` | 12 cm from the destination, estimate within 0.1 cm                                    |

The distance to the destination is Nav2's goal tolerance (`xy_goal_tolerance` 0.1 m)
plus `refine_tolerance_m` (0.15 m); the estimate itself is centimetre-level.

A single tag's heading error is multiplied by the distance to it; the remaining
error at 2 m is mostly range. The simulated image is ideal; expect more noise from
the real camera. Gate on `/landmarks/localisation`'s `residual_m` and the
accepted/rejected counts.

## Limitations

- **Heading from a single tag** is the weak link: a few degrees error at a 2 m lever
  arm is several centimetres. Two tags in view fix that.
- **Detection range** for a 16 cm tag at 480P is about 3.5 m (`min_marker_perimeter_rate`
  0.03). Bigger tags reach further.
- **Stop-and-look**: sightings while turning faster than `max_yaw_rate_rad_s` are
  ignored (motion blur, timestamp skew).
- **Too close is out of view**: the OAK-D is pitched 20° down for terrain, so a
  tag 0.25 m above the ground leaves the top of the image when it is nearer than
  about 0.55 m to the camera (about 0.75 m to `base_link`). Put a destination's stop
  pose at least 0.8 m from its tag.
- **Validated in simulation only**: Gazebo cannot turn this skid-steer in place
  (see [Terrain-aware autonomy, Known issues](terrain-autonomy.md#known-issues)), so
  the rotating search and destinations were validated in MuJoCo. Nothing has run on
  the robot yet; confirm the OAK-D's rectified-image topic name first
  (`ros2 topic list`, look for `rect`).

## Troubleshooting

| Symptom                                                                                   | Likely cause                                                                                                                                                                                                                 |
| ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/landmarks/detections` silent                                                            | Wrong `image_topic` (look for `rect` in `ros2 topic list`), camera env not running, or no `camera_info` yet.                                                                                                                 |
| Detections, but never calibrated                                                          | Only one tag in view and all its sightings ambiguous (head-on or far): move so it is seen at an angle (20–50°) and closer, or put two tags in view. Check `ambiguity_ratio` in `/landmarks/detections`.                      |
| `Tag fix rejected: residual above ...`                                                    | A tag has moved, its surveyed pose is wrong, or two tags share an id.                                                                                                                                                        |
| `... waiting for it to persist`                                                           | A fix disagrees with the estimate by more than the jump limits; it will be accepted if it persists `jump_confirm_s`.                                                                                                         |
| `No transform map <- oak_..._optical_frame`                                               | SLAM not running yet, or the URDF without the OAK-D.                                                                                                                                                                         |
| Goals in `site` fail with a TF error                                                      | Not calibrated yet (`site → map` is only published after the first fix).                                                                                                                                                     |
| Search: `saw tags [..] but timed out with N of 10 usable sightings; rejected so far: ...` | The tag was found but the fix failed; the counts say why (too far / too oblique / poor corner fit / while turning). The search keeps turning.                                                                                |
| Search: `spin refused or failed (obstacle?)`                                              | Nav2's Spin found an obstacle in the footprint's sweep; clear it, or move the robot.                                                                                                                                         |
| Nodes die with `Failed to find a free participant index`                                  | CycloneDDS with localhost-only discovery has a small participant pool; raise it: `CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'`. |
