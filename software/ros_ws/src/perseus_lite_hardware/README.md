ST3215 System Hardware for Perseus Lite

## Implementation Notes

ROS2 Jazzy diff_drive_controller requires stamped messages. This code is useful to debug with:

```
ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  twist: {
    linear: {x: 0.5, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

## Overview

This ROS 2 hardware interface provides a comprehensive driver for controlling the ST3215 servo motors used in the Perseus Lite robot. The implementation supports precise motor control, state monitoring, and integration with ROS 2 Control framework.
Features

Integration with ROS 2 Control hardware interface
Support for multiple ST3215 servo motors
Real-time velocity and position control
Detailed servo state monitoring including:

- Position
- Velocity
- Load
- Temperature

## Hardware Requirements

Feetech ST3215 Continuous Servo (12-bit encoder, Serial communication interface)

## Configuration

The hardware parameters are set by
`perseus_lite_description/ros2_control/perseus_lite.ros2_control.xacro`:

| Parameter            | Default        | Meaning                                                                           |
| -------------------- | -------------- | --------------------------------------------------------------------------------- |
| `serial_port`        | `/dev/ttyACM0` | Serial device of the servo bus adapter                                            |
| `baud_rate`          | `1000000`      | Bus speed (ST3215 factory default is 1 Mbps)                                      |
| `inverted_servo_ids` | `2,3`          | Servos mounted mirrored (left side); commands **and** feedback are negated        |
| `servo_max_rpm`      | `485.1`        | RPM represented by a full-scale (32767) wheel-mode speed; calibrated on the robot |
| `acceleration`       | `0`            | Servo `ACC` register; 0 disables the on-servo speed ramp                          |

Servo IDs are set per joint with an `id` parameter. The wiring on the robot is:

| Joint                     | Servo ID | Side (inverted) |
| ------------------------- | -------- | --------------- |
| `front_left_wheel_joint`  | 3        | left (yes)      |
| `front_right_wheel_joint` | 4        | right           |
| `rear_left_wheel_joint`   | 2        | left (yes)      |
| `rear_right_wheel_joint`  | 1        | right           |

## Communication Protocol

- Packet structure: header (`0xFF 0xFF`), ID, length, instruction, parameters, checksum
- Wheel-mode speed is sign-magnitude: bit 15 is the direction, the low 15 bits the
  magnitude (nominally one encoder step per second)
- Wheel-mode position wraps every revolution (0-4095); the driver unwraps it into a
  continuous joint angle
- A background thread polls each servo's status (position, speed, temperature); a
  servo whose status has not been received for 1 s reports zero velocity. Write
  acknowledgements are discarded and never count as feedback.

## Safety behaviour

The ST3215 has no communication watchdog: it keeps running at its last goal speed
if the host stops talking to it. The driver therefore:

- zeroes the goal speed before enabling torque at configure time
- writes zero speed on deactivate, and zero speed plus torque-off on cleanup,
  shutdown and on any ros2_control error transition (`on_error`)
- relies on `diff_drive_base_controller`'s `cmd_vel_timeout` (0.5 s) to zero the
  command when velocity input stops

A crash or `SIGKILL` of the controller manager while driving still leaves the servos
running until the next configure; cut power to stop them in that case.

## Threading Model

Utilises Boost ASIO for asynchronous I/O.
Dedicated IO thread for non-blocking communication. Thread-safe operations using mutex
