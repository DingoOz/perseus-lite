# perseus_monitoring

Passive health monitoring for Perseus Lite. Neither node ever commands the robot.

| Node                   | Publishes                                                       | Purpose                                                                                                                                                                                                                                                         |
| ---------------------- | --------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `topic_health_monitor` | `/health/topics` (`perseus_interfaces/SystemHealth`, 1 Hz)      | Rate, bandwidth and staleness of a list of topics, subscribed generically (any type). Status per topic: `OK`, `SLOW`, `STALE`, `NO_PUBLISHER`. Logs every change.                                                                                               |
| `mobility_monitor`     | `/mobility/status` (`perseus_interfaces/MobilityStatus`, 10 Hz) | Sliding-window commanded-vs-measured checks: `drive_linear` / `drive_angular` (command vs wheel odometry: stalled servos), `yaw_slip` (wheel yaw rate vs IMU gyro: wheels spinning in sand), optional `linear_slip` (wheels vs an independent velocity source). |

```shell
ros2 launch perseus_monitoring monitoring.launch.py [terrain:=true] [use_sim_time:=true]
ros2 topic echo /health/topics --once
ros2 topic echo /mobility/status
```

Started by default (`monitoring:=true`) with `perseus_lite_slam_and_nav2.launch.py`
and `autonomy.launch.py`.

Config: `config/topic_health.yaml` (core stack), `config/topic_health_terrain.yaml`
(core + OAK-D Lite + terrain mapper; used with `terrain:=true`),
`config/mobility_monitor.yaml`. Watching a new topic is a config change: append it
and its expected rate. Prefer a small companion topic for big ones (camera_info, not
the image): subscribing costs the full bandwidth.

Why the mobility checks look the way they do: the lite's EKF fuses wheel speed, so its
velocity reflects the wheels, not the body, and cannot reveal slip. See
`docs/source/systems/software/terrain-autonomy.md#monitoring`.
