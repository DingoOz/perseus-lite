# MuJoCo simulation

Perseus Lite has two simulators. **Gazebo** (`perseus_lite_simulation`) is the
original: full sensor plugins, mesh and heightmap worlds. **MuJoCo**
(`perseus_lite_mujoco`) is the second. It is lighter, it starts in seconds, and it
**turns this skid-steer in place**, which Gazebo's DART contact model cannot (see
[Terrain-aware autonomy, Known issues](terrain-autonomy.md#known-issues)). Anything
that spins the robot (the AprilTag search, Nav2's Spin, rotate-to-goal) is validated
in MuJoCo.

Both run the same robot stack: the same URDF, controllers, EKF, SLAM, Nav2, terrain
mapping and landmark nodes, and the same topic/frame contract. Only the simulator
underneath changes.

## Running it

```shell
pixi run -e simulation sim-mujoco                      # MuJoCo viewer + RViz
pixi run -e simulation sim-mujoco headless:=true       # no windows (CI, SSH)
pixi run -e simulation sim-mujoco initial_pose_x:=1.0 initial_pose_yaw:=1.57
```

Then start autonomy in a second terminal, exactly as for Gazebo:

```shell
pixi shell -e simulation
cd software/ros_ws && source install/setup.bash
ros2 launch autonomy autonomy.launch.py use_sim_time:=true [terrain:=true] [landmarks:=true]
```

Launch arguments of `mujoco_sim.launch.py`:

| Argument                                       | Default            | Meaning                                                                                                        |
| ---------------------------------------------- | ------------------ | -------------------------------------------------------------------------------------------------------------- |
| `world`                                        | `terrain_test.sdf` | A world from `perseus_lite_simulation/worlds` (or an absolute path). See [Worlds](#worlds).                    |
| `initial_pose_x`/`_y`/`_z`, `initial_pose_yaw` | 0, 0, 0.1, 0       | Spawn pose in the world frame.                                                                                 |
| `headless`                                     | `false`            | No MuJoCo viewer, no RViz.                                                                                     |
| `camera`                                       | `true`             | Render the OAK-D Lite (depth + image). Rendering costs about 30 ms per image; turn it off if nothing needs it. |
| `use_sim_time`                                 | `true`             | MuJoCo publishes `/clock`.                                                                                     |

MuJoCo is the `mujoco` Python package from conda-forge in the `simulation` env. The
headless renderer uses EGL (`MUJOCO_GL=egl`); the viewer uses GLFW.

## How it works

RoboStack's `mujoco_ros2_control` and `topic_based_ros2_control` are built only
against a newer `ros2-distro-mutex` than this workspace is locked to, so the bridge
is a small node of its own:

```text
 twist_mux ─► diff_drive_base_controller ─► ros2_control (mock_components/GenericSystem)
                                                  │ commanded wheel velocity, mirrored to state
                                                  ▼
                                            /joint_states ─► mujoco_sim ─► MuJoCo wheel actuators
                                                                 │
     /clock /scan /imu/data /oak/stereo/image_raw /oak/right_rect/image_raw /mujoco/ground_truth
```

- **ros2_control** runs `mock_components/GenericSystem`: each wheel's state is its
  commanded velocity, as the ST3215 servos closely track their setpoint. The wheel
  odometry is therefore the commanded motion, not the simulated one.
- **`mujoco_sim`** reads the commanded wheel velocities from `/joint_states`, drives
  MuJoCo's velocity actuators (gain `wheel_kv` 5, stall torque `wheel_max_torque`
  3 N m), steps the physics in real time (implicit-fast integrator, elliptic friction
  cones) and publishes:
  - `/clock`;
  - `/scan`: the RPLidar C1, by ray casting from `c1_lidar_frame`; the rays never hit
    the robot itself;
  - `/imu/data`: gyro and accelerometer at 100 Hz, no orientation (like the real I2C
    IMU), with the Gazebo sensors' noise;
  - `/oak/stereo/image_raw` (32FC1 depth) and `/oak/right_rect/image_raw` (RGB), with
    `camera_info`, by offscreen rendering, in the URDF's OAK-D optical frames;
  - `/mujoco/ground_truth`: the robot's true pose and twist in the world frame
    (`nav_msgs/Odometry`), for validation only. Nothing in the stack reads it.
- **TF** is not published by the simulator: `robot_state_publisher`, the EKF and
  SLAM own it, as on the robot.
- **The model** is built at start-up from the same sources Gazebo uses: the
  xacro-generated URDF (fixed-joint links lumped into one body, with masses, centres
  of mass and inertias combined exactly; wheels as hinge bodies) and the SDF world.
  A URDF or world change applies to both simulators.

## Calibration and its differences from the robot

The launch file adds two MuJoCo-specific overlays. Both are deliberate and both are
documented in the files themselves.

**`config/mujoco_controllers.yaml`: effective wheel separation 2.693 m.** A skid-steer
turns slower than its wheel speeds predict, because the wheels scrub sideways, so
`diff_drive_base_controller` uses an _effective_ separation (0.8286 m on the robot,
whose true track is 0.376 m). MuJoCo's contact model scrubs differently and needs its
own value. Find it with:

```shell
ros2 run perseus_lite_mujoco calibrate_turning
```

It drives the model directly (no ROS) and bisects until the achieved yaw rate matches
the command. Re-run it after changing the robot's mass, wheels or the friction
parameters in `config/mujoco_sim.yaml`. Turning is still not linear in the command:
about 0.97x at 0.3 rad/s, 1.03x at 0.6 rad/s, and up to 1.1–1.5x on short, fast turns
at 1 rad/s, where the robot's yaw momentum carries it on.

**`config/mujoco_ekf.yaml`: EKF overrides.** Two changes on top of
`perseus_lite/config/ekf.yaml`:

1. **Wheel-odometry outlier gates opened.** The robot config gates wheel-odometry
   twist at a Mahalanobis distance of 1.0. A yaw rate that changes faster than about
   0.1 rad/s between 15 Hz EKF updates is then rejected, which happens at the start of
   nearly every turn. In MuJoCo the EKF reported +44–52° for a 113° turn. The simulator
   has no wheel-odometry outliers, so the gates are opened.
2. **Gyro yaw rate fused.** The robot config fuses no IMU axis, so its heading is
   wheel odometry alone. Here wheel odometry is the commanded twist, and the
   non-linear turning above put the heading 40–60° off after a few Nav2 spins: beyond
   slam_toolbox's ±20° rotation search, so the map smeared and goals were driven to the
   wrong place. Fusing the simulated gyro (the real IMU's noise model) fixed it: the
   EKF heading then matched ground truth to within a degree through every test.

Both point at the real robot too: see [Findings for the robot](#findings-for-the-robot).

## Worlds

Worlds come from `perseus_lite_simulation/worlds/*.sdf`. The converter handles static
models with **box, cylinder, sphere and plane** geometry, `<include>`d local models
(such as the generated AprilTags, with their PBR albedo textures), and directional
lights. Anything else (meshes, heightmaps, actors, plugins) is skipped with a log
line. So `terrain_test.sdf` works fully, while mesh or heightmap worlds such as the
arc world do not: use Gazebo for those.

Two rendering details matter for the camera:

- The **clip planes are pinned in metres** (near 2 cm, far 60 m). MuJoCo scales them
  with the model's extent by default, and a 40 m ground box put the near plane about
  0.5 m out, so a tag or wall closer than that vanished from the image.
- The AprilTag models are self-lit (`emissive_map`) as in Gazebo, so tags decode
  whatever the sun angle.

## Validation

Measured against `/mujoco/ground_truth` in `terrain_test.sdf`:

| Test                                                             | Result                                                                            |
| ---------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| Turn in place, 0.3 / 0.6 / 1.0 rad/s steady                      | 0.28 / 0.62 / 1.11 rad/s achieved                                                 |
| EKF heading vs truth (gyro fused), pulses and Nav2 spins to 172° | Within 1°                                                                         |
| Arcs at 0.25 m/s, 0.1–0.3 rad/s for 4 s                          | 21° / 41° / 69° achieved for 23° / 46° / 69° commanded                            |
| Nav2 goal 1.5 m to the left (needs a turn)                       | SUCCEEDED, 8 cm from the goal                                                     |
| AprilTag calibration, search and destinations                    | See [AprilTag landmark localisation, Accuracy](landmark-localisation.md#accuracy) |

## Findings for the robot

Testing in MuJoCo turned up three things in the shared configuration that also
affect the real robot:

- **Nav2's DWB could not turn toward a goal** (fixed in
  `autonomy/config/perseus_nav_params.yaml`). The `PreferForward` critic at scale 32
  added 320 to any trajectory turning at 1 rad/s (its `theta_scale` 10 × |wz| × 32),
  against about 60 for every other critic combined. DWB always chose (0.11 m/s,
  0 rad/s) and drove straight on into whatever lay ahead for any goal not dead
  ahead. With `min_vel_x: 0` reverse is never sampled, so the critic did nothing
  else; it was removed. **Re-check on the robot.**
- **The EKF's wheel-odometry gate** (`odom0_twist_rejection_threshold: 1.0`) very
  likely rejects the start of fast turns on the robot too. SLAM's scan matching hides
  it, up to ±20°. Not changed for the robot.
- **The robot's EKF fuses no gyro.** On the robot the wheel odometry is the servos'
  measured velocity, not the command, so the case for the gyro is weaker than in
  simulation. But fusing the yaw rate is the standard remedy for skid-steer heading
  drift. Not changed for the robot.

## Limitations

- **Contact is not sand.** Friction 0.6 with soft contacts (`impratio` 1) lets the
  wheels slip as on loose ground, but there is no sinkage or terrain deformation.
- **Turning in place moves the centre**: skid-steer scrub walks the robot 10–50 cm
  over a full turn in MuJoCo. Whether the robot does the same in sand is untested.
- **No mesh or heightmap worlds** (see [Worlds](#worlds)).
- **The camera is ideal**: a pinhole render with depth noise only, without stereo
  artefacts, blur or exposure.
- `perseus_lite_mujoco` builds only in the `simulation` env (the default and
  machine-learning envs skip it).
