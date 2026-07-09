# CUDA-acceleration demo plan

Five options for visually demonstrating the speed benefit of the GPU/CUDA
Isaac ROS packages built in `software/isaac-ros-native/`, discussed
2026-07-09. All run on the Jetson (headless); visualization happens on a
network-connected laptop over the same ROS domain, per the existing
"Live camera test" pattern in `software/isaac-ros-native/README.md`.

## Status

| # | Demo | Status |
|---|------|--------|
| 1 | Live AprilTag: GPU (VPI/cuAprilTag) vs CPU (pupil-apriltags), side by side | **Done** |
| 2 | Occupancy-grid localizer: GPU parallel scan-matching vs CPU brute-force, pose-grid heatmap | **Done** |
| 3 | YOLOv8: TensorRT (GPU) vs ONNX Runtime (CPU) on a recorded video, FPS overlay | Not started |
| 4 | U-Net segmentation: same GPU-vs-CPU structure as #3, dense per-pixel masks | Not started |
| 5 | Quantitative throughput benchmark: TensorRT vs ONNX Runtime CPU, latency/throughput chart | Not started |

## 1. Live AprilTag: GPU vs CPU, side by side — DONE

Same live camera feed (`usb_cam` → GPU `RectifyNode` → shared
rectified frame) is consumed by two independent detectors:
- **GPU**: the existing, live-camera-proven `isaac_ros_apriltag`
  (`AprilTagNode`, cuAprilTag via VPI) — unchanged, reused as-is.
- **CPU**: a new node wrapping `pupil-apriltags` (the official AprilTag
  C library's Python binding, CPU-only — no prebuilt aarch64 wheel on
  PyPI, builds from source via the env's existing `c-compiler`/
  `cxx-compiler`, confirmed working). Publishes the *same*
  `AprilTagDetectionArray` message type as the GPU node so both sides
  share overlay-drawing code.

Run via `pixi run -e isaac-nitros demo-apriltag-speed`. See
`software/isaac-ros-native/README.md` for the laptop-side RViz2/
rqt_image_view setup.

**What actually happened, and why the design changed twice:** the
camera itself is fixed at 10 fps, which would trivially cap both
detectors at the same rate and hide any real difference, so a
`demo1_frame_pump_node.py` republishes the latest frame at a much
higher rate to decouple detector throughput from camera rate. Getting
that pump right took real iteration (full history in
`demo1_frame_pump_node.py`'s docstring):
1. Naively republishing full-res color frames hit a `ros2 topic bw`
   ceiling around 12 MB/s — nowhere near the 200 Hz target.
2. Tried a smaller `mono8` encoding to cut message size — `cuAprilTags`
   (GPU) rejects `mono8` outright and crashed the whole container.
   Reverted to `bgr8`.
3. Shrank resolution instead. Rate scaled with pixel count as expected,
   but GPU and CPU stayed locked to *each other* and to the pump rate
   at every size tried (12 Hz at 640×480, 68 Hz at 320×180, 262 Hz at
   160×90) — never revealing either detector's real ceiling. An
   isolated 2-process pub/sub diagnostic (no ROS graph complexity)
   sustained 8-10x higher throughput at the same message size, ruling
   out a hard transport limit.
4. Found a real bug: the pump was calling `cv2_to_imgmsg()` (a fresh
   copy/serialize) on every timer tick instead of once per real camera
   frame. Fixed.
5. Suspected inter-process CPU contention (5 concurrent processes on a
   6-core board) next, and redesigned the pump to alternate GPU-only
   and CPU-only phases so each detector gets the system to itself.
   Result: no change — GPU-alone and CPU-alone still matched each
   other and the pump rate, at both 640×480 and 320×180.

That consistency — three very different resolutions, two different
contention conditions, same result every time — became the actual
finding: **the ROS2/DDS message-passing pipeline's own throughput was
the binding constraint in every configuration tried, not either
detector's compute time.** Neither cuAprilTag (GPU) nor pupil-apriltags
(CPU, 2 threads) was ever slow enough to be the limiting factor within
the range this pipeline can deliver frames at.

Given a closed compiled GPU binary can't be instrumented internally,
the demo now also times the CPU detector's `detect()` call directly
(wall-clock around just that call, zero ROS overhead) — logged every
3 seconds. Stable, repeatable result over multiple GPU/CPU phase
cycles at 640×480: **5.4–5.9 ms/frame → ~170–185 fps max, pure CPU
compute, 2 threads.** This is a genuinely fast number for a CPU-only
implementation — a more interesting and honest result than a simple
"GPU crushes CPU" headline would have been, and a real example of how
a naive benchmark methodology can silently measure the wrong thing
(message-passing overhead) instead of what it set out to measure
(algorithm speed).

## 2. Occupancy-grid localizer: GPU vs CPU pose-grid comparison — DONE

Reuses the real CUDA kernel from Stage 12
(`occupancy_grid_localizer_gpu.cu`), not borrowed TensorRT inference —
the most technically substantive of the five. Run via
`pixi run -e isaac-nitros demo-ogl-speed`.

**Design decision that made this the most defensible of all five demos:**
`isaac_ros_occupancy_grid_localizer`'s compiled node already contains a
genuine CPU fallback path (`SearchCandidates` → `ScorePose` →
`RaycastRange`, serial per-pose/per-beam ray-marching) alongside the GPU
path (`SearchCandidatesGpu`, batching every candidate pose through one
`gpu_->ScorePoses()` CUDA kernel call) — `LoadMap()` falls back to CPU
automatically if GPU context init throws. Rather than reimplement the
algorithm ourselves (real risk of an unfair or subtly-wrong comparison,
exactly the trap Demo 1 fell into with its own methodology, not its
code), this demo forces the identical compiled binary down each path by
hiding the GPU from CUDA entirely: `CUDA_VISIBLE_DEVICES=""`. Confirmed
this genuinely works on this Jetson via a standalone `ctypes` test
before relying on it (`cudaMalloc` returns `cudaErrorNoDevice`, which
*is* the `std::runtime_error` the node's own `catch` block expects — a
real fallback, not a simulated one). No reimplementation risk at all:
both numbers come from NVIDIA's own real code.

The coarse search level alone sweeps the *entire* bundled map (38.7m ×
22.25m at 0.05 m/cell, ~265k candidate poses, each scored against up to
128 lidar beams) — genuinely heavy compute, not a toy problem.

**Result (first real run on this Jetson):** GPU completed the full
three-level coarse→medium→fine search in **20.58s**, recovering the
identical pose Stage 12 already verified against NVIDIA's ground truth
(33.60, 7.70). The CPU fallback path **did not complete even the coarse
level within 900 seconds (15 minutes)** — a genuinely dramatic, fully
honest result: at least a 44x speedup, and that undersells it, since CPU
never actually finished. Rendered as a single result image (map +
recovered GPU pose + timing banner), saved to
`demo2_result.png`/`.json` and published to `/demo2_ogl_comparison` for
a few minutes so it's viewable via RViz2/`rqt_image_view` on a laptop
before the script exits.

## 3. YOLOv8: TensorRT (GPU) vs ONNX Runtime (CPU)

Same model (`dummy_yolov8s.onnx`), two backends, on a recorded video —
TensorRT (Stage 5f's existing pipeline) vs ONNX Runtime's CPU execution
provider on identical frames. Detections are meaningless (random
weights) but FPS is real; overlay an FPS counter on each and play back
side by side. Needs a new CPU-inference launch/check pair; no new model
work.

## 4. U-Net segmentation: GPU vs CPU

Same structure as #3 but for Stage 7's dense per-pixel segmentation
model, which does far more FLOPs/pixel than detection — expect the
largest GPU speedup ratio of any of the five. Visual payoff: the
colorized mask updates fast (GPU) vs. visibly crawling (CPU).

## 5. Quantitative throughput benchmark

Less a live demo, more a defensible number for the report: batch many
frames through the same ONNX model on TensorRT vs ONNX Runtime CPU,
plot images/sec and p50/p90/p99 latency (matplotlib, same style as
`software/isaac-ros-native/report/make_charts.py`). Good complement to
#3/#4 if the goal is a chart for the LaTeX report rather than a
live spectacle.
