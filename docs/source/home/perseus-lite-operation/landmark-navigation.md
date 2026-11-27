# Localising with AprilTags

AprilTags at surveyed places let the rover know where it is in **your** coordinates
(the "site" frame), not just relative to wherever it switched on. One tag near the
start is enough; tags at destinations correct the position on arrival. Design,
parameters and accuracy: [AprilTag landmark localisation](project:/systems/software/landmark-localisation.md).

## Before the first run

1. Print `tag36h11` tags (e.g. 16 cm black square). Mount them upright, roughly
   0.25 m above the ground.
2. Survey each tag's centre and the direction it faces, and write them, plus any
   named destinations, into a copy of `perseus_landmarks/config/landmarks.yaml`.
   Put each destination's stop pose **at least 0.8 m in front of its tag**: the
   camera looks down for terrain, and a tag closer than that leaves the top of the
   image.

To try it without the robot, use the MuJoCo simulation (it has four tags and three
destinations):

```console
pixi run -e simulation sim-mujoco
# second terminal: pixi shell -e simulation, then
ros2 launch autonomy autonomy.launch.py use_sim_time:=true landmarks:=true
```

## Start it

```console
# Terminal 1: camera
pixi run -e camera oak

# Terminal 2: robot + SLAM + Nav2 + landmarks
pixi shell
source software/ros_ws/install/setup.bash
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py \
    landmarks:=true landmarks_file:=$HOME/my_site.yaml
```

If a tag is in view at the start, the rover localises by itself within a few
seconds:

```console
ros2 topic echo /landmarks/localisation --once    # calibrated: true
```

## If no tag is in view

Ask the rover to look for one. **It will turn on the spot** (and with
`use_search_points`, drive a little):

```console
ros2 action send_goal -f /landmarks/search perseus_interfaces/action/SearchForLandmarks \
    "{calibrate: true}"
```

Or launch with `search_on_start:=true` to do this automatically 15 s after start.

## Go somewhere

```console
ros2 action send_goal -f /landmarks/go_to_destination \
    perseus_interfaces/action/GoToDestination "{destination: east_dock}"
```

Any Nav2 goal given in the `site` frame works too (RViz: set the fixed frame to
`site`, then use 2D Goal Pose).

:::{warning}
Searching and destinations move the robot. Keep the gamepad deadman (which
overrides Nav2) and the e-stop in reach.
:::
