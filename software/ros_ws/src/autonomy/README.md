## Autonomy Package

To run without actual Perseus hardware set use_mock_hardware to true when launching:

```
ros2 launch autonomy mapping_using_slam_toolbox.launch.py use_mock_hardware:=true
```

This ROS2 package is intended to cover functionality to:

- create maps of the locally experienced environment from sensor data (lidar and depth cameras)
- determine navigability of the map for large rovers
- localisation of the rover in its known map
- support self-driving to human-selected waypoints
- implement self-driving exploration routines to explore the local area
- implement self-correction activities for loss of network events
- create heatmaps of network connectivity for display to operators
- monitor and generate position related status events (e.g. lost traction and bogged"

### Dependencies

SLAM toolbox
Nav2

### Terrain-aware navigation (overlay)

`config/terrain_nav_params.yaml` is a Nav2 parameter **overlay** for the OAK-D
Lite terrain mapper (`perseus_terrain`). `perseus_nav_bringup.launch.py` applies
it on top of `perseus_nav_params.yaml` via `overlay_params_file` (default: the
inert `config/nav_overlay_none.yaml`). `autonomy.launch.py` and
`perseus_lite_slam_and_nav2.launch.py` do it for you with `terrain:=true`, and
start `terrain_mapper` first. It adds:

- `perseus_terrain::TerrainGridLayer` costmap layers (local: live grid + memory;
  global: memory);
- extra planners (`ThetaStar`, `Smac2D`) and a controller (`RPP`), picked per goal
  from `/planner_selector` / `/controller_selector`; the defaults stay
  `GridBased` + `FollowPath`;
- `behavior_trees/navigate_to_pose_terrain.xml`: path smoothing, plus
  terrain-memory clearing in recovery.

`scripts/plan_probe.py` compares planners on one goal without driving:
`ros2 run autonomy plan_probe.py <x> <y> [--planners ...] [--frame map]`.

Full documentation: `docs/source/systems/software/terrain-autonomy.md`.
