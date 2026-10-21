# perseus-arm-teleop

Terminal based Arm Teleop for Perseus

By default this will connect two arms connected via serial ports on the one machine.

It will display torque readings and position.

Calibration values for leader and follower arms can be saved to a .yaml file.

Always make sure all servos are on the same baud rate. The default is 1 000 000 baud (1 Mbps, the
ST3215 factory default); pass `--baud <rate>` if your servos are configured differently
(eg `--baud 115200`).

# Build

This is a standalone CMake project (not part of the colcon workspace) and has no ROS dependencies:
it needs Boost (headers only), ncurses and yaml-cpp, all of which are in the Pixi `default` env.

```
pixi run -e default cmake -S software/arm-teleop-direct -B software/arm-teleop-direct/build -DCMAKE_BUILD_TYPE=Release
pixi run -e default cmake --build software/arm-teleop-direct/build -j
pixi run -e default ctest --test-dir software/arm-teleop-direct/build --output-on-failure
```

(A plain `mkdir build && cd build && cmake .. && make` works too if the dependencies are installed
system-wide.)

# Use

```
arm-teleop-direct [--baud <rate>] [<arm1-port> <arm2-port>]
```

With no ports given, the available serial ports are listed for selection. Arm 1 is the leader and
arm 2 the follower.

Keys: `1`-`6` toggle mirroring (and follower torque) per servo, `s` saves calibration to
`arm_calibration.yaml`, `l` loads it, `t` toggles software torque protection. Ctrl+C (or
SIGTERM/SIGHUP) disables torque on every servo of both arms and exits.

A servo that does not answer within 200 ms is reported as `Timeout waiting for servo response`
rather than freezing the program; replies with a bad checksum are rejected.

Alternatively see below on how to use for teleoperation over a local network.

## To use over a network for teleoperation

We need to virtualise a remote serial port and present it as a local device. We can do this using the
Linux tool socat.

To install on Ubuntu

```
sudo apt install socat
```

#### First set up the Follower

This assumes serial port being used for the arm is /dev/ttyACM0.

```
socat TCP-LISTEN:7100,reuseaddr /dev/ttyACM0,b1000000,raw,echo=0
```

#### Second set up the Leader

This sets up a local file which is actually network connected to the remote serial port.
This assumes the remote machine has an IP Address of 192.168.1.15

```
socat PTY,link=/home/dingo/leader_follower,raw,echo=0 TCP:192.168.1.15:7100
```

Then run the application and the virtualised remote port should be detected automatically.

## TODO

- Handle rollover of values (joint #2 really needs this)
- Add PID to movements to reduce jerkiness
- Add ability to include 24 servos (second leader and second follower)
- Add additional diagnostic info

## DONE

- Add yaml config file loading
- Colour code torque values (mostly for gripper)
