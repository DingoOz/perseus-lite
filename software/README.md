# About

This folder contains:

- All code running on the rover
- Container definitions and system services that support it
- Tooling helper scripts

Its subdirectories are as follows. Most contain their own README.md detailing their purpose and contents.

- `arm-teleop-direct`: Standalone serial teleop for the Feetech-servo manipulator arm (plain CMake project, not a ROS package; build with `pixi run -e default build-arm-teleop`).
- `assets`: Non-code assets (eg. mission scripts/briefs).
- `daemons`: systemd units and setup scripts for services running on the robot (eg. time sync).
- `docker`: Container stacks, currently the Isaac ROS (Humble) GPU perception stack for the Jetson Orin Nano (`docker/isaac-ros`).
- `home-manager`: Legacy Nix home-manager machine/user configuration. Orphaned since the Nix to Pixi migration (non-functional without a flake); kept for reference only.
- `native`: Pure C/C++/Python/etc programs which are built and run directly on machines.
- `ros_ws`: The ROS 2 software workspace containing all the actual logic.
- `scripts`: Short bash scripts for some commonly run tasks (incl. the Pixi activation sanitiser).
- `shared`: C++ libraries shared between the `native` and `ros_ws` workspaces.
- `templates`: Template files for bringing up new projects.
- `utilities`: One-off developer utilities and demos.
- `web_ui`: The rover web interface.

The configuration for the Pixi-managed dev/build environments lives in the repository root (`pixi.toml`, `pixi.lock`), not in this folder.

# Build System

This project uses [Pixi](https://pixi.sh) (with ROS 2 and its dependencies pulled from the `robostack-jazzy` conda channel) to provide the dev/build toolchain, then invokes `colcon` inside it to actually build the `ros_ws` workspace. See the root `pixi.toml` for the environments available (`default`, `simulation`, `machine-learning`, `docs`, `format`) and `CLAUDE.md` for the quick-start commands.

This gets you:

- Reproducibility: `pixi.lock` pins exact package versions, so a clean `pixi install` reproduces the same environment on any machine.
- Distro independence: RoboStack packages ROS 2 for conda, so the same environment works regardless of your host distro's own package versions.
- No system ROS install required: Pixi environments are self-contained under `.pixi/`, so they don't conflict with (or require) a system-wide ROS installation.

## Overview

Each project is built using standard tools (eg CMake via `ament_cmake`/colcon); Pixi just supplies the toolchain and dependencies that `colcon build` needs.
