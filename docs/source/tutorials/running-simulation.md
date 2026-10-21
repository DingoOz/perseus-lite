# Running the Gazebo Simulation

This tutorial provides the basic commands needed to run the Perseus Lite rover in Gazebo simulation.

## Prerequisites

- Pixi installed
- perseus-lite repository cloned locally
- Basic familiarity with command line interface

## Steps

### 1. Enter the Simulation Environment

```bash
pixi shell -e simulation
```

This command enters the specialised Pixi environment configured for simulation (Gazebo, `ros_gz_*`, `gz_ros2_control`) with all necessary dependencies.

### 2. Build the Workspace

From the repository root:

```bash
pixi run -e simulation build
```

This builds every ROS2 package the simulation environment can build, including the simulation components.
It skips `perseus_vision`, which needs the `machine-learning` environment's CUDA `onnxruntime-cpp` - a bare `colcon build` would try to build it and fail.

### 3. Navigate to the ROS Workspace

```bash
cd software/ros_ws
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

This sets up the ROS2 environment variables to use the built packages.

### 5. Launch the Simulation

```bash
ros2 launch perseus_lite_simulation perseus_sim.launch.py
```

This command launches the Gazebo simulation with the Perseus Lite rover model.

:::{warning}
Some simulation environments (e.g. worlds with falling rock obstacles) use external assets hosted online.

- An **active internet connection** may be required on first launch to download these assets.
- Once downloaded, assets are typically **cached locally** by the simulator and can be reused offline.
- If you experience missing models or world load failures, please ensure you are connected to the internet and restart the simulator.

Future work may migrate these assets to local storage to fully support offline simulation.
:::

### 6. Activate the Drive Controller

After the simulation has fully started, you need to activate the drive controller. In a **separate terminal**, run:

```bash
pixi shell -e simulation
cd software/ros_ws
source install/setup.bash
ros2 control set_controller_state diff_drive_base_controller active
```

This step is required to enable the rover's wheel movement in simulation.

## Controlling the Rover

Once the simulation is running and the controller is active, you can control the rover using keyboard input. In a **separate terminal**, run:

```bash
pixi shell -e simulation
cd software/ros_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel -p stamped:=true
```

Keyboard commands go to `/key_vel`; `twist_mux` forwards the highest-priority
active source (gamepad `/joy_vel` > keyboard `/key_vel` > web `/web_vel` > Nav2)
to `/cmd_vel`, which the base controller subscribes to.

## Troubleshooting

### Rover Not Moving in Teleop

If the rover doesn't respond to keyboard commands:

0. **Workaround for broken ROS controller**:

There is a bug being investigated where the ROS2 controller daemon is non-responsive.

Instead of activating the controller with `ros2 control set_controller_state diff_drive_base_controller active` use:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['diff_drive_base_controller'], deactivate_controllers: [], strictness: 1, activate_asap: false, timeout: {sec: 5, nanosec: 0}}"
```

This sets the state directly.

1. **Check if the controller is active**:

   ```bash
   ros2 control list_controllers
   ```

   You should see `diff_drive_base_controller` as `active`. If it shows `inactive`, run:

   ```bash
   ros2 control set_controller_state diff_drive_base_controller active
   ```

2. **Verify topic connections**:

   ```bash
   ros2 topic info /cmd_vel
   ```

   This should show `twist_mux` as the publisher and `diff_drive_base_controller` as the subscriber.

3. **Test topic communication**:
   ```bash
   ros2 topic echo /cmd_vel
   ```
   You should see messages when pressing keys in the teleop terminal.

:::

- The first time you run the simulation, Gazebo may download the lunar landscape model, which can take up to 20 minutes
- Ensure you have an internet connection for the initial setup
- If the system has a firewall, the network must be set to trusted to download assets needed for the simulation
- The simulation requires significant system resources and may perform better with hardware acceleration
- If the rover doesn't move, make sure the drive controller is activated (step 6)

:::
