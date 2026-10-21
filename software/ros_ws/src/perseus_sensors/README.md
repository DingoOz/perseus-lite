# Perseus Sensors - IMU and Topic Nodes

This package provides ROS2 nodes for **IMU bias estimation and correction**, as well as **dynamic topic remapping** for downsampling sensor data.

Included nodes:

1. **BiasEstimator** – estimates IMU gyro bias when the robot is stationary.
2. **BiasRemover** – removes the estimated bias from IMU readings.
3. **TopicRemapper** – dynamically detects a topic type and republishes it at a lower frequency.

---

> ⚠️ **Open-Source Attribution:**  
> The **BiasEstimator** and **BiasRemover** ROS2 nodes in this package are adapted from the
> [ros-perception/imu_pipeline](https://github.com/ros-perception/imu_pipeline/tree/ros2) project.  
> In particular, the bias removal logic is based on the implementation in `imu_processors/src/imu_bias_remover.cpp` from that repository, with extensions for stationary detection, configurable parameters, and composable node support in the Perseus platform.

## 1. BiasEstimator Node

**Node Name:** `imu_bias_estimator`
**Package:** `perseus_sensors`
**Plugin (Composable):** `imu_processors::BiasEstimator`

### Description

Estimates the bias of the IMU angular velocity (`gyro`) when the robot is stationary
and publishes it for the BiasRemover. Launch both with
`ros2 launch perseus_sensors bias_remover.launch.py` (parameters in
`config/imu_bias.yaml`: `/imu/data` in, `/imu/data_corrected` out).

Stationary detection:

- Only **enabled** sources (`use_odom`, `use_cmd_vel`) vote; a disabled source never
  counts as stationary. `stationary_mode` combines the enabled votes (`AND` / `OR`).
- **odom** votes stationary when every twist component is below `odom_threshold`. No
  odometry for `odom_timeout` means motion is unknown, so it votes _not_ stationary.
  Use wheel odometry (`/diff_drive_base_controller/odom`), not the EKF output, which
  already contains the IMU and would make the estimate circular.
- **cmd_vel** votes stationary when the last command is zero, or no command has arrived
  for `cmd_vel_timeout` (the controller stops the wheels on its own timeout). Before
  the first message it votes _not_ stationary.
- A sample with any `|rate|` above `max_gyro_rate` is rejected even when the sources
  say stationary.
- **If no source is enabled the node refuses to estimate**: it logs an error and
  publishes nothing, so the BiasRemover passes the IMU data through uncorrected. A
  gyro-only fallback was rejected because it cannot tell a slow, steady turn from bias.
- Nothing is published until the first stationary sample has been accepted.

### Subscribed Topics

| Topic           | Type                                    | Description                                        |
| --------------- | --------------------------------------- | -------------------------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`                       | Raw IMU data                                       |
| `cmd_vel_topic` | `geometry_msgs/TwistStamped` or `Twist` | (Optional) Robot velocity for stationary detection |
| `odom_topic`    | `nav_msgs/Odometry`                     | (Optional) Robot odometry for stationary detection |

### Published Topics

| Topic            | Type                           | Description                     |
| ---------------- | ------------------------------ | ------------------------------- |
| `bias_out_topic` | `geometry_msgs/Vector3Stamped` | Estimated angular velocity bias |

### Parameters

| Parameter           | Type   | Default                              | Description                                             |
| ------------------- | ------ | ------------------------------------ | ------------------------------------------------------- |
| `imu_in_topic`      | string | `"/imu/data"`                        | Input IMU topic                                         |
| `bias_out_topic`    | string | `"/imu/gyro_bias"`                   | Output bias topic                                       |
| `cmd_vel_topic`     | string | `"/cmd_vel"`                         | Cmd_vel topic for stationary detection                  |
| `odom_topic`        | string | `"/diff_drive_base_controller/odom"` | Odom topic for stationary detection                     |
| `use_cmd_vel`       | bool   | `false`                              | Enable cmd_vel for stationary detection                 |
| `use_odom`          | bool   | `true`                               | Enable odometry for stationary detection                |
| `use_stamped`       | bool   | `true`                               | cmd_vel is `TwistStamped` (Jazzy) rather than `Twist`   |
| `stationary_mode`   | string | `"AND"`                              | Combine enabled sources: `"AND"` or `"OR"`              |
| `accumulator_alpha` | double | `0.01`                               | EWMA alpha for bias accumulation, (0, 1]                |
| `cmd_vel_threshold` | double | `0.001`                              | Cmd_vel threshold for stationary detection              |
| `odom_threshold`    | double | `0.001`                              | Odometry threshold for stationary detection             |
| `cmd_vel_timeout`   | double | `0.5`                                | Seconds without cmd_vel after which it counts as zero   |
| `odom_timeout`      | double | `0.5`                                | Seconds without odom after which motion is unknown      |
| `max_gyro_rate`     | double | `0.1`                                | Reject samples above this rate (rad/s); `<= 0` disables |
| `estimator_rate_hz` | double | `50.0`                               | Bias estimation and publishing frequency                |

---

## 2. BiasRemover Node

**Node Name:** `imu_bias_remover`
**Package:** `perseus_sensors`
**Plugin (Composable):** `imu_processors::BiasRemover`

### Description

Removes the latest bias from the IMU angular velocity and republishes corrected IMU messages. Can throttle output rate.

### Subscribed Topics

| Topic           | Type                           | Description                         |
| --------------- | ------------------------------ | ----------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`              | Raw IMU data                        |
| `bias_in_topic` | `geometry_msgs/Vector3Stamped` | Bias estimated from `BiasEstimator` |

### Published Topics

| Topic           | Type              | Description        |
| --------------- | ----------------- | ------------------ |
| `imu_out_topic` | `sensor_msgs/Imu` | Bias-corrected IMU |

### Parameters

| Parameter           | Type   | Default                 | Description                                       |
| ------------------- | ------ | ----------------------- | ------------------------------------------------- |
| `imu_in_topic`      | string | `"/imu/data"`           | Input IMU topic                                   |
| `imu_out_topic`     | string | `"/imu/data_corrected"` | Corrected IMU output topic                        |
| `bias_in_topic`     | string | `"/imu/gyro_bias"`      | Input bias topic                                  |
| `zero_when_no_bias` | bool   | `false`                 | If no bias received, set angular velocity to zero |
| `output_rate_hz`    | double | `50.0`                  | Throttle publishing rate                          |

---

## 3. TopicRemapper Node

**Node Name:** `topic_remapper`
**Package:** `perseus_sensors`

### Description

Dynamically detects the message type of an input topic and republishes it at a lower frequency. Useful for downsampling high-frequency topics such as IMU or LiDAR.

### Subscribed Topics

| Topic         | Type    | Description           |
| ------------- | ------- | --------------------- |
| `input_topic` | dynamic | Source topic to remap |

### Published Topics

| Topic          | Type    | Description                       |
| -------------- | ------- | --------------------------------- |
| `output_topic` | dynamic | Remapped topic at lower frequency |

### Parameters

| Parameter             | Type   | Default      | Description               |
| --------------------- | ------ | ------------ | ------------------------- |
| `input_topic`         | string | -            | Topic to remap (required) |
| `output_topic`        | string | `"remapped"` | Remapped output topic     |
| `reduction_frequency` | double | `10.0`       | Output frequency in Hz    |
