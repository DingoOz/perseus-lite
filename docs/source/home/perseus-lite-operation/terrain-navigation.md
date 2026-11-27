# Navigating with terrain mapping

With the OAK-D Lite fitted, Nav2 can avoid what the 2D lidar cannot see: low
rocks, steps, holes and rough ground. This page is the operator's checklist; the
design, parameters and tuning are in
[Terrain-aware autonomy](project:/systems/software/terrain-autonomy.md).

## Start it

```console
# Terminal 1: the camera driver (its own Pixi environment)
pixi run -e camera oak

# Terminal 2: robot + SLAM + Nav2 with terrain
pixi shell
source software/ros_ws/install/setup.bash
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py terrain:=true
```

Or use the mission-control TUI: run the **OAK-D Lite camera driver** task, then
the **SLAM + Nav2** profile with _Terrain mapping_ ticked.

:::{warning}
`terrain:=true` makes Nav2 trust the camera. Keep the gamepad (deadman) and the
e-stop in reach. Ground the camera has never seen counts as drivable, and a
drop-off usually looks like "nothing" to a depth camera.
:::

## Check it before driving

```console
ros2 topic echo /health/topics --once     # all_ok: true
```

`/oak/stereo/camera_info`, `/terrain/local_grid` and `/terrain/global_grid` should
all read `status: 0` (OK). In RViz, add **Map** displays for `/terrain/global_grid`
(durability _Transient Local_) and `/terrain/local_grid` (_Volatile_): rocks in front
of the robot should appear as dark cells, flat sand as free.

## Pick a planner and controller

Defaults are the robot-validated NavFn planner and DWB controller. To try others
for the next goal (no restart needed):

```console
ros2 topic pub -1 /planner_selector std_msgs/String "{data: ThetaStar}"   # GridBased | ThetaStar | Smac2D
ros2 topic pub -1 /controller_selector std_msgs/String "{data: RPP}"      # FollowPath | RPP
```

Rule of thumb: ThetaStar + DWB in clutter; ThetaStar + RPP on open sand.

## While it drives

```console
ros2 topic echo /mobility/status
```

- `stalled: true`: the wheels are not reaching the commanded speed (bogged
  down, a servo current-limited or off the bus). Stop and look.
- `slipping: true`: the wheels report turning that the gyro does not see:
  wheels spinning in sand.

## If it gets stuck

Nav2's recovery already forgets terrain memory around the robot when it thinks
it is boxed in. To forget everything (e.g. after moving obstacles, or at the
start of a new run):

```console
ros2 service call /terrain_mapper/clear_all nav2_msgs/srv/ClearEntireCostmap
```
