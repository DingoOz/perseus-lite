# Isaac ROS container stack

CUDA/NITROS-accelerated perception (AprilTag + TensorRT object detection) for
the Jetson Orin Nano, running as Docker services in **ROS 2 Humble** (Isaac ROS
3.2 — the terminal release for Orin Nano) and bridged to the native **Jazzy**
host stack over zenoh.

**Why a bridge, not shared DDS:** Humble and Jazzy do not interoperate over raw
DDS (Iron+ changed serialization / added type hashes). The Humble graph runs
isolated on `ROS_DOMAIN_ID=52`; two `zenoh-bridge-ros2dds` services tunnel only
a small allow-list of standard-typed topics to the host on `51`.

Full architecture, topic contract, and the on-Jetson bring-up runbook:
`docs/source/systems/software/isaac-ros.md`.

## Quickstart (Jetson, after the runbook's host prep)

```console
docker login nvcr.io                    # NGC API key; pulls the Isaac ROS base
docker compose build                    # slow first time
docker compose up -d
docker compose logs -f
# from the host (Jazzy/Pixi) stack:
ros2 topic hz /perseus_isaac/apriltag/detections
```

Configuration (image pins, domains, zenoh port, camera device, engine build)
lives in `.env`. The TUI exposes these as tasks (`isaac_up`, `isaac_down`,
`isaac_logs`, …).

- `CAMERA_DEVICE` is the **host** device; it always appears as `/dev/video0`
  inside the container (what `usb_cam` is configured for).
- `ZENOH_PORT` is passed to both bridges on the command line (`-l`/`-e`).
- The YOLOv8 TensorRT engine build is **opt-in** (`ISAAC_BUILD_ENGINE=1`; the
  detector isn't wired into the launch yet). It runs in the background so the
  AprilTag graph starts immediately, and a failed build is remembered (not
  retried on every restart) until `ISAAC_REBUILD_ENGINE=1` or a new image.
- The `isaac-models` volume is re-synced from the image whenever the model hash
  or base image changes, so rebuilding the image never leaves stale models.
- `/perseus_isaac/health` reports `STALE` when no camera frames arrive and
  `WARN` when the camera is alive but AprilTag produces nothing (3 s default,
  relay parameter `stale_after_s`).

> Scaffold status: validated by `docker compose config` + lint/tests, **not yet
> GPU-validated on hardware**. The relay/launch package under `ws/` builds only
> inside the image, never by the host colcon workspace.
