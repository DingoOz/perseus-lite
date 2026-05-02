# Perseus Lite

Bringup package for the Perseus Lite rover.

## Teleop Operation

### Terminal 1 - Launch the rover

```bash
nix run .#perseus-lite
```

Or with colcon (local development):

```bash
cd ~/perseus-v2/software/ros_ws
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch perseus_lite perseus_lite.launch.py
```

### Terminal 2 - Launch the Xbox controller (wireless)

```bash
nix run .#generic_controller
```

This launches the wireless Xbox controller by default. Other options:

```bash
# Wired Xbox controller
nix run .#generic_controller -- wireless:=false

# Other controllers
nix run .#generic_controller -- type:=8bitdo
nix run .#generic_controller -- type:=logitech wireless:=false
```

### Terminal 3 - Visualize in rviz2 (on laptop)

The robot runs on `ROS_DOMAIN_ID=42` with **CycloneDDS** and a bumped
`MaxAutoParticipantIndex` (Nav2 + SLAM spawn many participants). Your laptop
must match all three or you'll see `RTPS_READER_HISTORY` payload errors,
"Send goal call failed" from the Nav2 panel, and the SlamToolbox plugin
hanging on "Waiting for the slam_toolbox node configuration."

**Recommended (Ubuntu 24.04 with Nix installed):**

```bash
cd ~/perseus-v2
nix run .#rviz2-perseus-lite
```

That single command sets all the env vars, restarts the ROS daemon, and
launches rviz2 with the bundled `nav2.rviz` config — no need to install
ROS or rviz separately on the laptop, the flake provides everything.

If you'd rather not use Nix (system ROS Jazzy on Ubuntu 24.04):

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-nav2-rviz-plugins ros-jazzy-slam-toolbox
source /opt/ros/jazzy/setup.bash
bash ~/perseus-v2/software/ros_ws/src/perseus_lite/scripts/rviz_remote.sh
```

The script and the `nix run` app do the same thing. Pass extra rviz
args (or a different config) by appending them.

If launching rviz manually, export these first:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 daemon stop && ros2 daemon start
rviz2 -d software/ros_ws/src/perseus_lite/rviz/nav2.rviz
```

NVIDIA-only note: if `nix run .#rviz2-perseus-lite` fails with an
OpenGL error on Ubuntu, prefix with NixGL:
`nix run --impure github:nix-community/nixGL -- nix run .#rviz2-perseus-lite`

In rviz2:

1. Set **Fixed Frame** to `map` (or `odom` / `base_link` if SLAM hasn't
   published a map yet)
2. Add displays:
   - **Add** → **By topic** → `/scan` → **LaserScan**
   - **Add** → **By topic** → `/tf` → **TF**
   - **Add** → **RobotModel** (uses `/robot_description`)

### Verify operation

```bash
# Check nodes are running
ROS_DOMAIN_ID=42 ros2 node list

# Expected nodes include:
# /controller_manager
# /diff_drive_base_controller
# /twist_mux
# /joy_node
# /generic_controller

# Check cmd_vel is being published
ROS_DOMAIN_ID=42 ros2 topic echo /cmd_vel
```

## Launch Arguments

| Argument            | Default        | Description                              |
| ------------------- | -------------- | ---------------------------------------- |
| `use_sim_time`      | `False`        | Use simulation time                      |
| `use_mock_hardware` | `False`        | Use mock hardware instead of real servos |
| `serial_port`       | `/dev/ttyACM0` | ST3215 servo serial port                 |
| `baud_rate`         | `1000000`      | Servo baud rate                          |

## Configuration Files

Perseus Lite uses dedicated config files in `autonomy/config/` with hardware-specific settings:

| Config File                                     | Description                         |
| ----------------------------------------------- | ----------------------------------- |
| `slam_toolbox_params_perseus_lite.yaml`         | SLAM Toolbox (main)                 |
| `slam_toolbox_params_clean_perseus_lite.yaml`   | SLAM Toolbox (tuned for clean maps) |
| `slam_toolbox_params_minimal_perseus_lite.yaml` | SLAM Toolbox (minimal/fast)         |
| `nav_params_perseus_lite.yaml`                  | Nav2 navigation stack               |
| `ekf_params_perseus_lite.yaml`                  | Robot localization EKF              |

These differ from the full Perseus rover configs:

- `base_frame`: `base_link` (Perseus uses `base_footprint`)
- `scan_topic`: `/scan` (Perseus uses `/livox/scan`)
- `robot_radius`: `0.12` (smaller footprint)

Override configs via launch arguments:

```bash
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py \
    slam_params_file:=/path/to/custom_slam.yaml \
    nav_params_file:=/path/to/custom_nav.yaml \
    ekf_params_file:=/path/to/custom_ekf.yaml
```

## Coloured-cube detection

Perseus Lite runs a colour-based 100 mm cube detector alongside the ArUco
detector. It segments Red/Blue/Green/White cubes in HSV space, fits a quad to
each visible face contour, and recovers the face's 6-DoF pose with `solvePnP`
using the known 100 mm edge length. No depth camera or ML model required —
just the existing Logitech C920.

Brought up automatically by `perseus_lite.launch.py`. To run it standalone
(camera node must already be publishing `/image_raw` and `/camera_info`):

```bash
bash ~/perseus-v2/software/ros_ws/src/perseus_lite/scripts/cube_color_detector.sh
```

The script sources the workspace, sets `ROS_DOMAIN_ID=42` and the bumped
CycloneDDS `MaxAutoParticipantIndex`, and launches
`perseus_lite cube_color_detector.launch.py`. Override the domain by
exporting `ROS_DOMAIN_ID` first; pass extra `key:=value` launch args after
the script name.

If you need the camera too (e.g. testing without the full bringup):

```bash
ros2 run v4l2_camera v4l2_camera_node \
    --ros-args -p video_device:=/dev/c920 -p image_size:=[320,240] \
                -p pixel_format:=YUYV -p camera_frame_id:=camera_link
```

### Topics, TF, and service

| Resource                                | Type                                    | Notes                                                                                                      |
| --------------------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `/perseus_vision/cube_color/detections` | `perseus_interfaces/ObjectDetections`   | `ids[i]` is the colour code below                                                                          |
| `/perseus_vision/cube_color/markers`    | `visualization_msgs/MarkerArray`        | RViz `CUBE` markers, coloured + labelled                                                                   |
| `/perseus_vision/cube_color/image`      | `sensor_msgs/Image`                     | Annotated debug stream (drawn quads + axes)                                                                |
| TF: `cube_<colour>_<index>`             | broadcast in `tf_output_frame` (`odom`) | One frame per detection per tick                                                                           |
| `/detect_cubes`                         | `perseus_interfaces/DetectObjects`      | Service: returns latest cached detections; `capture_image:=true` saves an annotated PNG to `img_save_path` |

Detection ID encoding (matches the existing ML `cube_detector`):

| ID  | Colour |
| --- | ------ |
| 0   | blue   |
| 1   | green  |
| 2   | red    |
| 3   | white  |

### Inspect detections

```bash
# Watch the annotated image in real time
ros2 run rqt_image_view rqt_image_view /perseus_vision/cube_color/image

# Stream raw detections
ros2 topic echo /perseus_vision/cube_color/detections

# Verify TF frames are publishing
ros2 run tf2_tools view_frames
```

In RViz, add a `MarkerArray` display on `/perseus_vision/cube_color/markers`
to see each cube rendered as a coloured 100 mm box at its localised pose.

### Tune HSV thresholds

The defaults in `config/cube_color_detector.yaml` are conservative starting
points for indoor LED light. To tune for your cubes/lighting, edit the
`hsv_<colour>` arrays — OpenCV scale (`H ∈ [0,180]`, `S/V ∈ [0,255]`):

```yaml
hsv_blue: [h_low, h_high, s_low, s_high, v_low, v_high]
hsv_red: [h_low1, h_high1, s_low, s_high, v_low, v_high, h_low2, h_high2]
```

Red is the only colour that needs the optional 7th/8th elements (the hue
wraps around 180/0). Watch `/perseus_vision/cube_color/image` while you
tune — colours whose mask is too tight will simply produce no detections;
masks that are too loose will produce false-positive quads on background
clutter (filtered out by `min_face_solidity` and `max_aspect_ratio`).

To skip a colour entirely, drop it from `enabled_colors`.

## Architecture

```
Controller (xbox/keyboard)
        │
        ▼
    /joy_vel
        │
        ▼
    twist_mux ──── /cmd_vel_nav (from Nav2)
        │
        ▼
    /cmd_vel
        │
        ▼
diff_drive_controller
        │
        ▼
  ST3215 Servos
```

The `twist_mux` node arbitrates between joystick input (`/joy_vel`, priority 10) and navigation commands (`/cmd_vel_nav`, priority 1). Joystick always overrides navigation when active.

## Teleop with SLAM Mapping

To drive the robot while building a map in real-time:

### Terminal 1 - Launch rover with SLAM and Nav2 (on robot)

```bash
ROS_DOMAIN_ID=42 ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py
```

This launches:

- Perseus Lite hardware (diff_drive_controller, LIDAR, IMU)
- SLAM Toolbox for mapping
- Nav2 navigation stack
- EKF for odometry fusion
- twist_mux for cmd_vel arbitration

### Terminal 2 - Launch the Xbox controller (on robot or laptop)

```bash
nix run .#generic_controller
```

**Important**: Hold the **left trigger (LT)** as a dead-man switch while using the left stick to drive. Release LT to stop.

### Terminal 3 - Visualize in rviz2 (on laptop)

```bash
ROS_DOMAIN_ID=42 rviz2
```

On non-NixOS systems:

```bash
ROS_DOMAIN_ID=42 nixgl rviz2
```

In rviz2:

1. Set **Fixed Frame** to `map`
2. Add displays:
   - **Add** → **By topic** → `/map` → **Map** (to see SLAM map)
   - **Add** → **By topic** → `/scan` → **LaserScan**
   - **Add** → **By topic** → `/tf` → **TF**
   - **Add** → **RobotModel** (uses `/robot_description`)

### Verify SLAM is working

```bash
# Check SLAM node is running
ROS_DOMAIN_ID=42 ros2 node list | grep slam

# Expected: /slam_toolbox or /async_slam_toolbox_node

# Check map is being published
ROS_DOMAIN_ID=42 ros2 topic hz /map

# Should show ~0.1-1 Hz when moving
```

### Save the map

```bash
ROS_DOMAIN_ID=42 ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## Waypoint Navigation

Drive the robot to waypoints using Nav2. The robot builds a map with SLAM while navigating.

### Terminal 1 - Launch rover with SLAM and Nav2 (on robot)

```bash
nix run .#perseus-lite
```

### Terminal 2 - Launch the Xbox controller (optional, on robot or laptop)

```bash
nix run .#generic_controller
```

The joystick has higher priority than Nav2 and will override autonomous navigation when active. Hold **left trigger (LT)** as dead-man switch.

### Terminal 3 - Open rviz2 with Nav2 config (on laptop)

```bash
ROS_DOMAIN_ID=42 rviz2 -d ~/perseus-v2/software/ros_ws/src/perseus_lite/rviz/nav2.rviz
```

On non-NixOS systems:

```bash
ROS_DOMAIN_ID=42 nixgl rviz2 -d ~/perseus-v2/software/ros_ws/src/perseus_lite/rviz/nav2.rviz
```

### Set waypoints and navigate

1. Wait for SLAM to build an initial map (drive the robot around briefly with the joystick)
2. In the **Navigation 2** panel (bottom-left of rviz2), select **Waypoint / Nav Through Poses** mode
3. Click multiple points on the map to place waypoints
4. Click **Start Waypoint Following** to execute - the robot will drive to each waypoint in sequence
5. To send a single goal instead, select **Navigate To Pose** mode and click a point on the map

The joystick can override navigation at any time (hold dead-man switch).

### Tune Nav2 parameters at runtime

Nav2 parameters can be adjusted in real time without rebuilding. Use `rqt_reconfigure` on your laptop for a GUI with sliders:

```bash
ROS_DOMAIN_ID=42 ros2 run rqt_reconfigure rqt_reconfigure
```

Or use the CLI directly:

```bash
# Max linear speed (m/s)
ROS_DOMAIN_ID=42 ros2 param set /controller_server FollowPath.max_vel_x 2.0

# Max rotation speed (rad/s)
ROS_DOMAIN_ID=42 ros2 param set /controller_server FollowPath.max_vel_theta 3.0

# Slow-down factor near goal (lower = less slowdown)
ROS_DOMAIN_ID=42 ros2 param set /controller_server FollowPath.RotateToGoal.slowing_factor 2.0

# Goal tolerance - how close before declaring "arrived" (meters)
ROS_DOMAIN_ID=42 ros2 param set /controller_server goal_checker.xy_goal_tolerance 0.15

# Trajectory simulation time (lower = less cautious)
ROS_DOMAIN_ID=42 ros2 param set /controller_server FollowPath.sim_time 0.8

# Collision monitor slowdown ratio (higher = less slowdown near obstacles)
ROS_DOMAIN_ID=42 ros2 param set /collision_monitor PolygonSlow.slowdown_ratio 0.8

# Velocity smoother acceleration limits [x, y, theta]
ROS_DOMAIN_ID=42 ros2 param set /velocity_smoother max_accel "[5.0, 0.0, 5.0]"
ROS_DOMAIN_ID=42 ros2 param set /velocity_smoother max_decel "[-5.0, 0.0, -5.0]"
```
