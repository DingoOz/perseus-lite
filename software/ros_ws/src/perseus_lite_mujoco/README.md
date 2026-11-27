# perseus_lite_mujoco

Perseus Lite in MuJoCo: the second simulator beside Gazebo
(`perseus_lite_simulation`). It runs the same robot stack and topic/frame contract,
and unlike Gazebo it turns the skid-steer in place.

```shell
pixi run -e simulation sim-mujoco [headless:=true] [world:=terrain_test.sdf]
# then, as with Gazebo:
ros2 launch autonomy autonomy.launch.py use_sim_time:=true [terrain:=true] [landmarks:=true]
```

| Component                     | What it is                                                                                                                                                                          |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mujoco_sim` (`sim_node.py`)  | Steps MuJoCo in real time; wheel commands from `/joint_states` (mock ros2_control hardware); publishes `/clock`, `/scan`, `/imu/data`, the OAK-D images and `/mujoco/ground_truth`. |
| `model_builder.py`            | Builds the MJCF from the xacro URDF and a Gazebo SDF world (simple geometry and included models only). ROS-free, unit-tested.                                                       |
| `calibrate_turning`           | Finds the effective skid-steer `wheel_separation` for `config/mujoco_controllers.yaml`.                                                                                             |
| `launch/mujoco_sim.launch.py` | robot_state_publisher, ros2_control (mock hardware), controllers, EKF, twist_mux, scan filter, the simulator, RViz.                                                                 |
| `config/mujoco_ekf.yaml`      | EKF overrides: wheel-odometry gates opened, gyro yaw rate fused (why: inline).                                                                                                      |

Builds only in the `simulation` env (`mujoco` from conda-forge). Full documentation:
`docs/source/systems/software/mujoco-simulation.md`.
