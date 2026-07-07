---
name: deploy-isaac-jetson
description: Bring up the Isaac ROS Docker stack on the Jetson Orin Nano and verify it bridges to the native Jazzy host, or debug that stack. Use when deploying, first-running, or troubleshooting software/docker/isaac-ros/ on the robot's Jetson.
---

# Deploying the Isaac ROS stack on the Jetson Orin Nano

The Isaac ROS perception stack runs in **ROS 2 Humble** containers (Isaac ROS
3.2 — the terminal release for Orin Nano; 4.x is Thor-only) and bridges to the
**Jazzy** host over zenoh, because Humble↔Jazzy don't interoperate over raw DDS.
Scaffold: `software/docker/isaac-ros/`. Full reference:
`docs/source/systems/software/isaac-ros.md`. As of scaffold time this is **not
yet GPU-validated on hardware** — this skill is the bring-up path.

## Host prep (once)

1. Confirm JetPack 6.x: `cat /etc/nv_tegra_release`.
2. `default-runtime: nvidia` in `/etc/docker/daemon.json`, then
   `sudo systemctl restart docker`.
3. Power + memory: `sudo nvpmodel -m 0` (MAXN/Super); add zram/swap (8 GB is
   shared CPU/GPU). Run headless.
4. `docker login nvcr.io` (NGC API key — the Isaac base is auth-gated; this is
   why CI never builds the image).
5. Pin the exact aarch64 Humble NGC image hash for your JetPack in
   `software/docker/isaac-ros/.env` (`ISAAC_ROS_IMAGE=`); confirm against the
   Isaac ROS 3.2 release notes.

## Deploy

```bash
cd software/docker/isaac-ros
docker compose build          # slow first time; pulls the NGC base
docker compose up -d
docker compose logs -f
```

First run builds the arch-specific TensorRT engine into the `isaac-models`
volume (minutes, memory-hungry) — cached thereafter. The TUI also exposes
`isaac_build/up/down/logs/status` tasks.

## Verify the bridge

From the native host stack (Jazzy/Pixi, domain 51):

```bash
ros2 topic hz /perseus_isaac/apriltag/detections
ros2 topic echo /perseus_isaac/health
```

If topics don't appear: the container graph is on **domain 52** (isolated) — you
should NOT see it directly on the host; you see it only via the zenoh bridge.
Check both `bridge-humble` and `bridge-jazzy` are up (`docker compose ps`), that
the `.env` domains are 52/51, and that only `/perseus_isaac/*` is allow-listed in
the two `config/zenoh-bridge-*.json5`.

## Budget & scope notes

- Keep to **AprilTag + at most one** TensorRT pipeline on the 8 GB board.
- **Object detection (yolov8/tensor_rt) is phase 2** — wire it into
  `perseus_isaac.launch.py` only after confirming the cube model's tensor
  input/output names and dims on-device (see `config/perseus_isaac.yaml`).
- Verify `vision_msgs/Detection2DArray` fields match Humble↔Jazzy on first run
  (the one type-compat assumption in the bridge contract).
- Log any surprises with `log-error`, and drop the "not GPU-validated" caveats
  in the docs/CLAUDE.md once it's running on hardware.
