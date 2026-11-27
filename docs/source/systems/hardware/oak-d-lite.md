# OAK-D Lite depth camera

The Luxonis **OAK-D Lite** ("OpenCV AI Kit: Depth Lite") is Perseus Lite's
stereo depth camera. Its depth image feeds the terrain mapper (see
[Terrain-aware autonomy](../software/terrain-autonomy.md)), which sees what the 2D
lidar cannot: low rocks, steps, holes and slopes.

| Property                        | Value                                                                                                |
| ------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Depth                           | Stereo pair of global-shutter mono sensors (OV7251, 640×480), 75 mm baseline, computed on the camera |
| Horizontal field of view (mono) | ~73°                                                                                                 |
| Useful depth for terrain        | ~0.3 m to ~3 m (error grows with distance squared)                                                   |
| Interface                       | USB-C (USB 3 recommended; USB 2 works at reduced bandwidth)                                          |
| Driver                          | `depthai_ros_driver` (RoboStack `ros-jazzy-depthai-ros-driver`)                                      |

## Mounting and frames

The camera mounts at the front of the chassis, pitched down so its vertical field
of view covers the ground from ~0.25 m ahead of the lens out to the horizon. The
mount pose is set in `perseus_lite_description/urdf/sensors.urdf.xacro`:

| xacro property    | Default        | Meaning                                      |
| ----------------- | -------------- | -------------------------------------------- |
| `oak_mount_x`     | 0.20 m         | Lens centre ahead of `base_link`             |
| `oak_mount_z`     | 0.28 m         | Lens centre above `base_link` (ground level) |
| `oak_mount_pitch` | 0.35 rad (20°) | Positive = nose down                         |

:::{important}
These are design values. **Measure the real bracket** and update them: a 1° pitch
error moves the ground 5 cm at 3 m, which the terrain mapper would read as a slope
or a step. With `terrain_debug:=true`, flat floor in `/terrain/debug/points` must lie
flat in RViz.
:::

The URDF defines the frames with the names the driver (node `oak`) stamps on its
messages, so the real camera and the Gazebo one resolve through the same TF tree:

```text
base_link
└── oak_link                         (camera body, x forward)
    ├── oak_rgb_camera_frame   → oak_rgb_camera_optical_frame
    ├── oak_left_camera_frame  → oak_left_camera_optical_frame    (y +37.5 mm)
    └── oak_right_camera_frame → oak_right_camera_optical_frame   (y −37.5 mm)
```

Depth is published unaligned, in `oak_right_camera_optical_frame`. The driver's own
TF/description publisher is **not** used (`i_publish_tf_from_calibration: false`, and
the launch file does not start its `robot_state_publisher`): the URDF is the single
source of the camera frames.

## The `camera` Pixi environment

RoboStack builds `depthai_ros_driver` only against a newer `ros2-distro-mutex`
(0.20+) than the robot's `default` environment is locked to (0.14). Adding it to
`default` would mean upgrading the robot's entire ROS stack. Instead, a slim
`camera` environment holds just ROS base, CycloneDDS and the driver, and runs the
driver as a separate process. It talks to the rest of the robot over DDS (same
Jazzy, same RMW, same `ROS_DOMAIN_ID`).

The `camera` environment does **not** build or source the colcon workspace. Its
launch file, `perseus_lite/launch/oak_d_lite.launch.py`, is run straight from the
source tree and finds `perseus_lite/config/oak_d_lite.yaml` relative to itself:

```shell
pixi run -e camera oak                  # on the robot; Ctrl-C to stop
```

The TUI offers the same as the **OAK-D Lite camera driver** task (Sensors group).

## First-time setup on the robot

1. **USB permissions.** The camera enumerates as a Movidius device (vendor
   `03e7`). Without a udev rule the driver cannot open it as a normal user:

   ```shell
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
       sudo tee /etc/udev/rules.d/80-movidius.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

   Then unplug and replug the camera.

2. **Install the environment** (large download the first time): `pixi install -e camera`.

3. **Start it and check**, in a robot-env terminal:

   ```shell
   ros2 topic hz /oak/stereo/image_raw              # ~10 Hz
   ros2 topic echo --once --no-arr /oak/stereo/image_raw | grep -E 'encoding|frame_id'
   #   encoding: 16UC1
   #   frame_id: oak_right_camera_optical_frame
   ros2 run tf2_ros tf2_echo base_link oak_right_camera_optical_frame
   ```

If the driver logs `Cannot find any device with given deviceInfo` and retries, it is
not seeing the camera: check the cable, the udev rule, and `lsusb | grep 03e7`.

## Driver configuration

`perseus_lite/config/oak_d_lite.yaml` tunes the driver for terrain mapping, not
imaging:

| Parameter                                           | Value       | Why                                                                       |
| --------------------------------------------------- | ----------- | ------------------------------------------------------------------------- |
| `camera.i_pipeline_type`                            | `DEPTH`     | Stereo depth only: no RGB stream, saving USB bandwidth and host CPU.      |
| `camera.i_nn_type`                                  | `none`      | No on-camera neural network.                                              |
| `camera.i_enable_imu`, `i_enable_ir`                | `false`     | Not used (the Lite has no IR projector).                                  |
| `left/right.i_resolution`, `i_fps`                  | `480P`, 10  | Native mono resolution; 10 Hz is plenty for a 0.2 m/s robot.              |
| `stereo.i_align_depth`                              | `false`     | Depth stays in the right camera's frame; aligning needs the RGB pipeline. |
| `stereo.i_subpixel`, `i_lr_check`                   | `true`      | Finer disparity far out; drop occluded pixels instead of guessing.        |
| `stereo.i_depth_preset`                             | `ROBOTICS`  |                                                                           |
| `stereo.i_enable_speckle_filter`                    | `true`      | Removes isolated blobs on low-texture sand.                               |
| `stereo.i_enable_temporal_filter`                   | `false`     | Smears depth while moving.                                                |
| `stereo.i_enable_threshold_filter`, `min/max_range` | 300–4000 mm | Drops depth the mapper would discard anyway.                              |

These are validated only as far as the driver loading and searching for a device;
the pipeline parameters are checked by the driver once a camera is connected. Treat
the first hardware run as part of setup, and adjust here.

## Simulation

The URDF also carries a Gazebo `rgbd_camera` at the right lens (640×480, 1.274 rad
horizontal field of view, 10 Hz, small Gaussian noise). The sim bridge
(`perseus_lite_simulation/config/gz_bridge.yaml`) republishes it as
`/oak/stereo/image_raw` (32FC1) and `/oak/stereo/camera_info`, in
`oak_right_camera_optical_frame`. That is enough for the terrain mapper to run
unchanged. The simulated depth is ideal; real stereo is noisier and has holes, so
tune thresholds against recorded bags, not the simulator.
