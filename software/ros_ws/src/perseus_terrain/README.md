# perseus_terrain

Terrain-aware costmaps for Perseus Lite from the OAK-D Lite depth camera.

| Component                                                 | What it is                                                                                                                                                                                                                    |
| --------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `terrain_mapper` (node)                                   | Depth image in; `/terrain/local_grid` (latest view, volatile) and `/terrain/global_grid` (log-odds memory, latched) out, both in `odom`. Services `~/clear_around_robot`, `~/clear_all` (`nav2_msgs/srv/ClearEntireCostmap`). |
| `perseus_terrain::TerrainGridLayer` (Nav2 costmap plugin) | Copies a terrain grid into any costmap through TF, keeping the worst terrain cell under each costmap cell.                                                                                                                    |
| `perseus_terrain_core` (library)                          | The ROS-free algorithms, unit-tested: `classify_terrain` (height-difference classifier), `LogOddsMap`, `project_depth`, cost mapping, grid and footprint geometry.                                                            |

```shell
ros2 launch perseus_terrain terrain.launch.py [publish_debug:=true] [use_sim_time:=true]
```

Usually started by `terrain:=true` on `perseus_lite_slam_and_nav2.launch.py` (robot) or
`autonomy.launch.py` (sim), which also load the Nav2 overlay
(`autonomy/config/terrain_nav_params.yaml`).

How it works in one paragraph: each depth frame is decimated, cropped (robot body,
ceiling) and transformed into `odom`, then binned into 5 cm cells. Each cell's robust
floor height is compared with a ground estimate derived from its neighbours and a
maximum slope; a floor too far above it (step up), below it (step down), or a return
standing too tall (height above ground) makes the cell not drivable, and approaching a
threshold adds a roughness cost. Heights are only compared within one frame, because
the lite's 2D EKF cannot track the chassis bouncing on its rockers; verdicts are fused
over time in the log-odds memory.

Parameters: `config/terrain_mapper.yaml` (every value commented).
Design, tuning, verification and troubleshooting:
`docs/source/systems/software/terrain-autonomy.md`.

Concepts follow the perseus-v3 autonomy stack; the code is an independent MIT
implementation (no upstream code was copied).
