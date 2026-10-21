# Vendored: Slamtec rplidar_ros (ROS 2 branch)

- Upstream: https://github.com/Slamtec/rplidar_ros (branch `ros2`)
- Commit: `24cc9b6dea97e045bda1408eaa867ce730fd3fc3` (package.xml 2.1.4, 2025-04-27)
- License: BSD-2-Clause (see `LICENSE`)
- Local changes: none (README images and `debian/` dropped).

Why it is vendored: the RoboStack `ros-jazzy-rplidar-ros` package is 2.1.0,
which predates RPLidar **C1** support (no C1 protocol handling and no
`rplidar_c1_launch.py`). The lite robot uses a C1, so the conda package must
not be installed alongside this one (colcon would refuse to override it).

To update: replace this directory with a newer upstream `ros2` checkout, keep
this file, and bump the commit above. It is excluded from treefmt so the diff
against upstream stays clean.
