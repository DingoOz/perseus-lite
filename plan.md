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
| 3 | YOLOv8: TensorRT (GPU) vs ONNX Runtime (CPU) on a recorded video, FPS overlay | **Done** |
| 4 | U-Net segmentation: same GPU-vs-CPU structure as #3, dense per-pixel masks | **Done** |
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

## 3. YOLOv8: TensorRT (GPU) vs ONNX Runtime (CPU) — DONE

Run via `pixi run -e isaac-nitros demo-yolov8-speed`.

**Design changed from the original plan, deliberately.** The original
idea (a ROS launch/check pair, FPS overlay via message rate) is exactly
the shape that turned out to be the entire problem in Demo 1 — so this
demo sidesteps ROS completely for the timed portion instead of risking
a repeat. TensorRT (`python3-libnvinfer`, the system apt package
matching the already-installed `tensorrt-dev` — not on PyPI for Jetson,
reached via `PYTHONPATH` rather than a pixi dependency) builds and runs
the same `dummy_yolov8s.onnx` model Stage 5 follow-up already validated
through the ROS pipeline; `onnxruntime`'s `CPUExecutionProvider` runs
the identical model. Both are called directly in a tight Python loop in
the same process — no publish/subscribe, no serialization, no DDS in
the timing path at all. GPU device buffers use `cuda-python`'s
`cuda.bindings.runtime` API (confirmed working with a standalone
`cudaMalloc`/`cudaMemcpy` round trip first — `cuda-python` 13.x moved
off the older `cuda.cuda`/`cuda.cudart` names most sample code online
still uses).

No recorded video exists on this headless robot, so the same single
640×640 test image Stage 5 follow-up already uses (`people_cycles.jpg`)
is reused as every "frame" — the network does identical work regardless
of pixel content, so this doesn't affect a throughput measurement.

**Result, reproduced across two separate runs:** TensorRT GPU
**4.1-4.3ms/frame (234-241 fps)** vs ONNX Runtime CPU **8.5-9.2ms/frame
(109-118 fps)** — a consistent **~2.0-2.2x speedup**. A believable,
moderate, real-world number (not a blowout, not a wash), and a
genuinely clean measurement this time: no message-passing pitfall to
find, because there was no message passing to hide one in. Result chart
saved to `demo3_result.png`/`.json` and published to
`/demo3_yolov8_comparison` for viewing via RViz2/`rqt_image_view`.

## 4. U-Net segmentation: GPU vs CPU — DONE

Run via `pixi run -e isaac-nitros demo-unet-speed`.

Same direct-Python-API structure as #3 (no ROS in the timing loop at
all), applied to Stage 7's `model.dummy.onnx` segmentation fixture
instead of YOLOv8's detector. Segmentation does far more FLOPs/pixel
than detection, so this was expected to show the largest GPU speedup
ratio of the series — confirmed.

One real structural difference from #3: `model.dummy.onnx` has a
**dynamic batch dimension** (`(-1,3,544,960)` input /
`(-1,544,960,20)` output), unlike YOLOv8's fixed batch of 1. This
needed an explicit TensorRT optimization profile
(`profile.set_shape(INPUT_NAME, INPUT_SHAPE, INPUT_SHAPE,
INPUT_SHAPE)`, min=opt=max=1) at engine-build time, plus a
per-context `context.set_input_shape(...)` call before inference —
neither was needed for #3's fixed-shape model.

**Result:** TensorRT GPU **114.4ms/frame (8.7 fps)** vs ONNX Runtime
CPU **2830.5ms/frame (0.4 fps)** — a **24.7x speedup**, by far the
largest of the four demos so far, exactly as the FLOPs/pixel argument
predicted. Result chart saved to `demo4_result.png`/`.json` and
published to `/demo4_unet_comparison` for viewing via
RViz2/`rqt_image_view`.

## 5. Quantitative throughput benchmark

Less a live demo, more a defensible number for the report: batch many
frames through the same ONNX model on TensorRT vs ONNX Runtime CPU,
plot images/sec and p50/p90/p99 latency (matplotlib, same style as
`software/isaac-ros-native/report/make_charts.py`). Good complement to
#3/#4 if the goal is a chart for the LaTeX report rather than a
live spectacle.
