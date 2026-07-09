# NITROS/GXF from-source build experiment (Orin + JetPack 7)

**Status: Stages 0–5, 7, 8, 10, and 12 all succeeded; Stage 6 and 9
blocked on hardware/upstream limitations; Stage 11 builds clean (real
CUDA kernels included) but is runtime-blocked on a Tegra memory-carveout
limit pending a system config decision.** Real AprilTag detection (`isaac_ros_apriltag`,
VPI/cuAprilTag) running on this Jetson Orin Nano under JetPack 7 using
entirely self-built binaries, verified to a **pixel-exact match** against
NVIDIA's own ground-truth test fixture — see Stage 4 below. Stage 4
follow-up added a live-camera pipeline (`pixi run test-apriltags`). Stage
5 added real GPU DNN inference (`isaac_ros_tensor_rt` +
`isaac_ros_dnn_image_encoder`, TensorRT 10.16.2/CUDA 13.2) — a full
image→resize/normalize→mobilenetv2 classification round trip, verified
end to end, then a Stage 5 follow-up added `isaac_ros_yolov8` as the
object-detector-level proof (real yolov8s architecture, NVIDIA's own
random-weight test fixture — full chain confirmed, detection content
not meaningful by design). Stage 6 (`isaac_ros_visual_slam`/cuVSLAM)
built clean but its published `aarch64_jetpack70` binary crashes with
`SIGILL` on `dlopen` — it contains **SVE2** instructions the Orin's
Cortex-A78AE CPU doesn't implement, a genuine upstream packaging bug, not
something fixable here — see Stage 6 below. This is a separate track from
the working
`software/docker/isaac-ros/` scaffold (which targets JetPack 6.x via
NVIDIA's prebuilt image) — it exists because NVIDIA hasn't shipped Isaac
ROS binaries for Orin+JetPack7 yet, and we're building our own from
source as a stopgap.

Full background, findings, and the staged plan: see
`docs/source/systems/software/isaac-ros-nitros-source-build.md`. Read that
before running anything here — it explains *why* this is staged the way it
is and what the licensing caveats are.

**Nothing cloned here is committed to perseus-lite.** `isaac_ros_common/`,
`isaac_ros_nitros/`, `gxf/`, and any `build/`/`install/`/`log/` output are
git-ignored (see repo-root `.gitignore`) — they're NVIDIA source-available /
proprietary-licensed code, not ours to redistribute. Only this README and any
scripts we write ourselves live in version control.

## Stage 0 — clone + environment stand-up

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
git clone https://github.com/NVIDIA-ISAAC-ROS/gxf
```

This host has no `docker` installed (checked 2026-07-08), so the classic
`isaac_ros_common/scripts/run_dev.sh` container workflow isn't available
without first standing up Docker + nvidia-container-runtime.

**Correction (found while attempting this live):** `isaac-ros-cli` is
**not** pip-installable — despite the outer repo being Apache-2.0, it ships
as a `.deb` (`sudo apt-get install isaac-ros-cli`, not in this host's apt
sources yet) built via its own `Makefile`, and even `isaac-ros init` itself
requires `sudo`. That `Makefile` also carries the same strict proprietary
NVIDIA header we already flagged for `gxf`'s core source in the doc page
("distribution... without an express license agreement... is strictly
prohibited") — another repo nominally open but with a stricter notice buried
inside.

So every documented path to a working Isaac ROS build environment — the CLI
tool, or the classic `run_dev.sh` container flow — crosses into installing
system packages / Docker with `sudo` at some point. Pixi deliberately can't
and shouldn't paper over that (see the pixi.toml `isaac-nitros` feature
comment). **Don't run any `sudo apt`/`sudo make install` here without
checking with whoever owns this box first** — this is real system state on
real robot hardware, not something scoped to the repo. See the doc page's
"Stage 0" section for the options and what was decided.

Once a toolchain is available (system CLI, Docker, or a manual bypass), the
`isaac-nitros` Pixi env provides the plain colcon/cmake/compiler toolchain
for the actual package builds: `pixi shell -e isaac-nitros`. Success
criterion for this stage: `colcon build` at least *configures* against
`isaac_ros_common`'s CMake macros — it is expected to fail at the GXF
resolution step (that's Stage 1).

## Stage 1 — does the published SBSA GXF binary load on Tegra?

**Result: yes.** See `isaac-ros-nitros-source-build.md` for the full story.
Summary of what actually worked, in order:

```console
# git-lfs is required — isaac_ros_nitros's .so files are LFS objects; without
# this they clone as ~132-byte text pointers that CMake happily "installs"
# without complaining, only failing much later at link time.
sudo apt-get install git-lfs
cd isaac_ros_nitros && git lfs install --local && git lfs pull && cd ..

# CMAKE_DEVICE auto-detects to sbsa on real Jetson hardware already (no
# forcing needed — only gxf_aarch64_cuda_13_0/gxf_x86_64_cuda_13_0 exist,
# no gxf_jetpack70/ yet).
#
# isaac_ros_gxf/CMakeLists.txt needed a local patch first: it uses
# $<INSTALL_PREFIX> (a generator expression only valid inside
# install(EXPORT)) directly in INTERFACE_LINK_LIBRARIES/
# INTERFACE_INCLUDE_DIRECTORIES, which newer CMake rejects outright
# ("should never be evaluated") — confirmed NOT a CMake-version issue
# (fails identically on 3.31.8 and 4.3.4). Fixed locally with:
sed -i 's/\$<INSTALL_PREFIX>/${CMAKE_INSTALL_PREFIX}/g' \
  isaac_ros_nitros/isaac_ros_gxf/CMakeLists.txt

mkdir -p ws/src
ln -sfn ../../isaac_ros_common ws/src/isaac_ros_common
ln -sfn ../../isaac_ros_nitros ws/src/isaac_ros_nitros
cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  colcon build --packages-select isaac_ros_common isaac_ros_gxf
'
cd ..
```

Then the actual smoke test (`gxf_smoke_test.cpp` in this directory —
`GxfContextCreate`/`GxfContextDestroy`, our own minimal code, not NVIDIA's).
**Link with the host's own `g++`, not Pixi's `isaac-nitros` conda-forge
cross-compiler** — the conda toolchain targets an old portable glibc
baseline and can't resolve versioned symbols
(`dlopen@GLIBC_2.34`, `__isoc23_strtoull@GLIBC_2.38`, ...) that NVIDIA's
natively-built (Ubuntu 24.04, glibc 2.39) binaries require:

```console
GXF_INSTALL=ws/install/isaac_ros_gxf
g++ gxf_smoke_test.cpp -o gxf_smoke_test \
  -I"$GXF_INSTALL/share/isaac_ros_gxf/gxf/include" \
  -L"$GXF_INSTALL/lib" -lgxf_core \
  -L"$GXF_INSTALL/share/isaac_ros_gxf/gxf/lib/logger" -lgxf_logger

# RUNPATH doesn't propagate transitively (libgxf_core.so's own NEEDED
# entries, like libgxf_logger.so, aren't resolved by our binary's rpath) —
# use LD_LIBRARY_PATH instead of fighting rpath for a quick smoke test:
LD_LIBRARY_PATH="$PWD/$GXF_INSTALL/lib:$PWD/$GXF_INSTALL/share/isaac_ros_gxf/gxf/lib/logger" \
  ./gxf_smoke_test
```

Expected output: `GxfContextCreate OK: context=0x...` /
`GxfContextDestroy OK`. This is the Stage 1 decision gate — success here
means Stage 2 (resurrecting the stale `gxf` build recipe) is likely
unnecessary; proceed to Stage 3 (build `isaac_ros_nitros` core) instead.

## Stage 2 — skipped

Not attempted. Stage 1 succeeded, so resurrecting the stale `gxf` build
recipe wasn't necessary.

## Stage 3 — build `isaac_ros_nitros` core + minimal round trip

**Result: yes, real NITROS round trip works.** See
`isaac-ros-nitros-source-build.md` for the full story (three more snags, all
resolved: the `$<INSTALL_PREFIX>` bug turned out to be in 17 files, not one;
`magic_enum` needed adding to the Pixi env plus a `CXXFLAGS` include-path
workaround for a conda-forge-vs-NVIDIA packaging-layout mismatch; and a
remapping bug in our own launch file). Commands:

```console
# Two more repos needed beyond Stage 0/1's clones:
# - isaac_ros_gxf_extensions is already inside isaac_ros_nitros (no new clone)
# - negotiated is a separate, fully open-source (Apache-2.0/Boost) repo:
git clone --depth 1 https://github.com/osrf/negotiated

# (isaac_ros_gxf_extensions needs no separate symlink -- colcon already
# recurses into isaac_ros_nitros/, which contains it)
ln -sfn ../../negotiated/negotiated ws/src/negotiated
ln -sfn ../../negotiated/negotiated_interfaces ws/src/negotiated_interfaces

# Same $<INSTALL_PREFIX> bug, 17 files this time (isaac_ros_managed_nitros +
# nearly every gxf_isaac_* extension — clearly from a shared template):
grep -rl '\$<INSTALL_PREFIX>' isaac_ros_nitros/ --include="CMakeLists.txt" \
  | xargs sed -i 's/\$<INSTALL_PREFIX>/${CMAKE_INSTALL_PREFIX}/g'

cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  # magic_enum: added to the isaac-nitros Pixi feature (conda-forge, MIT).
  # It packages the header under include/magic_enum/magic_enum.hpp; NVIDIA
  # code does a flat #include "magic_enum.hpp" -- point CXXFLAGS at the
  # subdirectory rather than patching their #include lines:
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum"
  colcon build --packages-up-to isaac_ros_nitros
'
cd ..
```

`BUILD_TESTING` defaults on, so this also builds NVIDIA's own minimal NITROS
proof-of-life test infra: `libnitros_empty_forward_node.so` (a composable
node plugin) and `test_cuda_stream_pool` (a standalone gtest — run it
directly, it needs no launch/ROS graph):

```console
GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS\$LD_LIBRARY_PATH\"
  ./ws/build/isaac_ros_nitros/test_cuda_stream_pool
"
# Expect: [ PASSED ] 14 tests. -- real CUDA stream acquire/release/reuse via
# libgxf_cuda.so, not just context lifecycle.
```

For the actual round trip: NVIDIA's own test
(`isaac_ros_nitros/test/isaac_ros_nitros_test_pol.py`) needs the
`isaac_ros_test`/`launch_testing`/`pytest.mark.rostest` harness, which pulls
in `torch` transitively (`isaac_ros_test/__init__.py` eagerly imports a
model-mocking helper unrelated to this test) — too heavy for a minimal
check. Used `nitros_roundtrip_launch.py` + `nitros_roundtrip_check.py` in
this directory instead (our own code, replicating the same
`ComposableNodeContainer` setup minus the torch dependency):

```console
pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS\$LD_LIBRARY_PATH\"
  ros2 launch nitros_roundtrip_launch.py &
"
sleep 4   # let negotiation + GXF graph load finish
pixi run -e isaac-nitros bash -c '
  source ws/install/setup.bash
  python3 nitros_roundtrip_check.py
'
# Expect: sent=1 received=1 / ROUNDTRIP OK
pkill -f component_container_mt   # clean up when done
```

If you see `sent=N received=0` with the container log showing both nodes
"Node was started" — check `ros2 topic list` for whether your remap
`from`/`to` strings are absolute (leading `/`). A relative remap resolves
*inside the node's own namespace* — `('mid1/x', 'y')` on a node namespaced
`mid1` silently becomes `/mid1/mid1/x`, matching nothing.

## Stage 4 — real perception: `isaac_ros_apriltag` + `isaac_ros_image_proc`

**Result: yes — pixel-exact match against NVIDIA's own ground truth.** See
`isaac-ros-nitros-source-build.md` for the full story. Two more repos, and a
dependency resolved through a different channel than Isaac ROS's own
binaries (NVIDIA's separately-released, actively-maintained
[CV-CUDA](https://github.com/CVCUDA/CV-CUDA), Apache-2.0):

```console
git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
# This repo is large enough that a plain `git clone` can time out fetching
# LFS content — skip the smudge, clone fast, then pull LFS deliberately:
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
cd isaac_ros_image_pipeline && git lfs install --local && git lfs pull && cd ..

ln -sfn ../../isaac_ros_apriltag ws/src/isaac_ros_apriltag
ln -sfn ../../isaac_ros_image_pipeline ws/src/isaac_ros_image_pipeline

# CV-CUDA (cvcuda0-dev) isn't in any apt repo, but CV-CUDA publishes its own
# GitHub release .debs -- find the one matching this platform (CUDA 13,
# aarch64) and install lib before dev:
curl -fSLO https://github.com/CVCUDA/CV-CUDA/releases/download/v0.16.0/cvcuda-lib-0.16.0-cuda13-aarch64-linux.deb
curl -fSLO https://github.com/CVCUDA/CV-CUDA/releases/download/v0.16.0/cvcuda-dev-0.16.0-cuda13-aarch64-linux.deb
sudo dpkg -i cvcuda-lib-0.16.0-cuda13-aarch64-linux.deb
sudo dpkg -i cvcuda-dev-0.16.0-cuda13-aarch64-linux.deb

cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  # VPI and CV-CUDA headers: same conda-sysroot issue as magic_enum in Stage 3
  # -- point CXXFLAGS at the exact include dirs. Do NOT add a bare
  # -I/usr/include "to be safe": it shadows condas own glibc headers with the
  # hosts and breaks the build worse (bits/timesize.h: No such file, in gtest).
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib"
  # BUILD_TESTING=OFF: several gtest binaries in these packages hit the same
  # conda-toolchain-vs-native-glibc link failure as the Stage 1 smoke test,
  # too many to individually re-link with the system compiler. The actual
  # libraries build and link fine either way.
  colcon build --packages-select isaac_ros_cvcuda_utils --cmake-args -DBUILD_TESTING=OFF
  colcon build --packages-select isaac_ros_image_proc --cmake-args -DBUILD_TESTING=OFF
  colcon build --packages-select isaac_ros_apriltag --cmake-args -DBUILD_TESTING=OFF
'
cd ..
```

`isaac_ros_image_proc` takes a few minutes (real CUDA/CV-CUDA kernel
compilation) — run it in the background rather than assuming a hang.
`colcon build --packages-select isaac_ros_apriltag` alone fails even though
`isaac_ros_image_proc` is only its `exec_depend`: colcon's `ament_cmake`
build task sources every dependency's environment hook regardless of
depend type, so `image_proc` has to actually be built first.

Verification uses NVIDIA's own ground-truth fixture
(`isaac_ros_apriltag/isaac_ros_apriltag/test/test_cases/apriltag0/`) rather
than a synthetic message, via `apriltag_launch.py` +
`apriltag_check.py` in this directory (our own code — loads the same PNG
via `cv_bridge`, bypassing `isaac_ros_test`'s torch dependency same as
Stage 3):

```console
GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS$EXTRA_LIB_DIRS\$LD_LIBRARY_PATH\"
  ros2 launch apriltag_launch.py &
"
sleep 5
pixi run -e isaac-nitros bash -c '
  source ws/install/setup.bash
  python3 apriltag_check.py
'
# Expect: id=0 family=tag36h11 center=(926.0,547.0) expected_center=(926.0, 547.0)
#         APRILTAG OK
pkill -f component_container_mt   # clean up when done
```

Not yet exercised (at the time the section above was written): a live
camera and `isaac_ros_image_proc::RectifyNode` — see the next section for
both, plus a real bug found in the rectify+apriltag combination.

## Live camera test (Logitech C920) + RViz on a different machine

```console
pixi run -e isaac-nitros test-apriltags
```

Launches `usb_cam` (1280×720, 10 Hz, `/dev/video0`) → `rectify` →
`apriltag` — publishes `/tag_detections` and a TF frame per detected tag
(`tag36h11:<id>`) at a steady 10 Hz on lens-corrected frames. Point the
C920 at a printed AprilTag (`tag36h11` family) to see it.

**History: this graph used to skip `RectifyNode`.** An earlier debugging
session found `rectify` → `apriltag` on live camera frames reproducibly
crashing (`nvcv::Exception: NVCV_ERROR_INVALID_OPERATION: The tensor
handle is null.`) and worked around it by dropping `RectifyNode`. After a
reboot, the identical graph ran clean for 30+ minutes with no crash at
all — the workaround was reverted and rectify is back in by default. See
`isaac-ros-nitros-source-build.md`'s "Follow-up after a reboot" section
for the full writeup; if the crash recurs, that section documents the
one-line revert. The bundled `camera_c920_info.yaml` is still NVIDIA's
placeholder calibration, not a real one for this camera — see that
file's header for how to generate a real one.

### Viewing it in RViz from another machine (e.g. your laptop)

This publishes on the same ROS graph as the rest of the robot
(`ROS_DOMAIN_ID=51`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, both set by
`pixi.toml`'s `[activation.env]`). Your laptop needs to join that same
graph:

1. **Same domain, compatible RMW.** On your laptop, before starting RViz:
   ```console
   export ROS_DOMAIN_ID=51
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # match the robot's RMW
   ```
   (Different RMW vendors — e.g. Fast DDS on your laptop vs Cyclone DDS
   here — are not guaranteed to discover each other even on the same
   domain ID. Use Cyclone DDS on both sides to avoid that entirely.)
2. **Network reachability.** Same LAN/Wi-Fi, no client isolation blocking
   multicast/UDP between the two machines (a common failure mode on
   guest/corporate Wi-Fi — a home network or a switch you control is
   simplest).
3. **RViz needs `isaac_ros_apriltag_interfaces`** installed to know the
   `AprilTagDetectionArray` message type if you want to inspect it directly
   (not needed just to see the `Image` + `TF` displays below, which are
   both standard types).
4. **Open RViz with the bundled config**, copied from this directory to
   your laptop: `apriltag_camera.rviz` (our own — adapted from NVIDIA's
   `isaac_ros_apriltag/rviz/usb_cam.rviz`; both point at `/image_rect`,
   the rectified/lens-corrected output):
   ```console
   rviz2 -d apriltag_camera.rviz
   ```
   Fixed Frame is `camera`; you should see the live image, and when a tag
   is in view, a `tag36h11:<id>` TF frame will appear (axes drawn relative
   to the camera). No config file handy? Just open RViz, set Fixed Frame to
   `camera`, add an `Image` display on `/image_rect`, and add a `TF`
   display — that's the whole config.

## Stage 5 — `isaac_ros_dnn_inference`: real GPU DNN inference (TensorRT)

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference
ln -sfn ../../isaac_ros_dnn_inference ws/src/isaac_ros_dnn_inference

sudo apt-get install tensorrt-dev   # 10.16.2.10-1+cuda13.2, from the
                                     # JetPack-7 L4T apt repo already
                                     # configured on this host

cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_ros_tensor_rt isaac_ros_tensor_proc isaac_ros_dnn_image_encoder \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
mkdir -p models
cp isaac_ros_dnn_inference/isaac_ros_tensor_rt/test/models/mobilenetv2-1.0.onnx models/

pixi run -e isaac-nitros test-dnn-classify
```

Deliberately skipped `isaac_ros_triton` (needs a separate Triton Inference
Server; not needed for a direct-TensorRT path). `-DCMAKE_CUDA_ARCHITECTURES=87`
is Orin's real SM (Ampere) — `isaac_ros_tensor_proc` compiles actual `.cu`
kernels (unlike `isaac_ros_tensor_rt`, pure C++ against the TensorRT API)
and CMake's own CUDA-arch auto-detection needs an explicit answer before
`isaac_ros_common-extras.cmake`'s own fallback logic gets a chance to run.

`test-dnn-classify` runs our own minimal launch/check pair (not NVIDIA's
`isaac_ros_test`-based suite): a static test image → `dnn_image_encoder`
(resize/normalize to 224×224) → `tensor_rt` (mobilenetv2-1.0), checking the
output tensor's shape/dtype/name *and* that it's not degenerate (finite,
non-constant — a real inference happened, not garbage). First run builds a
real TensorRT engine (~40s); later runs reuse the cached
`/tmp/trt_engine.plan`. Full findings, two real bugs hit while wiring the
encoder→tensor_rt chain, and the CCCL/CUDA-arch build issues:
`isaac-ros-nitros-source-build.md`'s "Stage 5" section.

## Stage 5 follow-up — `isaac_ros_yolov8`: object-detector proof-of-life

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection
ln -sfn ../../isaac_ros_object_detection ws/src/isaac_ros_object_detection
```

`vision_msgs` (for `Detection2DArray`) isn't in the default Jazzy
RoboStack channel — added as `ros-jazzy-vision-msgs` to
`[feature.isaac-nitros.dependencies]` in `pixi.toml`; run `pixi install -e
isaac-nitros` after pulling that change.

```console
cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_ros_yolov8 \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
cp isaac_ros_object_detection/isaac_ros_yolov8/test/dummy_model/yolov8/dummy_yolov8s.onnx models/

pixi run -e isaac-nitros test-yolov8
```

Only `isaac_ros_yolov8` built (the repo also has `isaac_ros_detectnet`,
`isaac_ros_rtdetr`, `isaac_ros_grounding_dino` — not needed here).
`YoloV8DecoderNode` built clean first try, no new CMake issues beyond the
`vision_msgs` dependency gap. `test-yolov8` runs the full `image →
dnn_image_encoder → tensor_rt → yolov8_decoder` chain against NVIDIA's own
`dummy_yolov8s.onnx` (real yolov8s architecture/IO names, **random
weights**) and `people_cycles.jpg` test image — checks that a structurally
valid `Detection2DArray` comes back, not detection accuracy (meaningless
with random weights — see `isaac-ros-nitros-source-build.md`'s "Stage 5
follow-up" section for why, and for the two over-strict-validation
findings from getting this check right).

**Not yet done:** a real trained YOLOv8 model for actually-meaningful
detections — this stopped at proving the plumbing, same scope as the
mobilenetv2 classification check. Also not wired into the live camera or
`perseus_isaac_relay`.

## Stage 6 — `isaac_ros_visual_slam` (cuVSLAM): BLOCKED, upstream binary bug

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
ln -sfn ../../isaac_ros_visual_slam ws/src/isaac_ros_visual_slam

cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_common isaac_ros_launch_utils \
    isaac_ros_visual_slam_interfaces isaac_ros_visual_slam \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
```

Builds clean — `isaac_ros_nitros` already ships a real
`lib_aarch64_jetpack70/libcuvslam.so` (unlike GXF core, no sbsa-fallback
needed; selected purely by `CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64"`).
**But loading it crashes with `SIGILL`, immediately, before any node code
runs**:

```console
$ python3 -c "import ctypes; ctypes.CDLL('.../lib_aarch64_jetpack70/libcuvslam.so')"
Illegal instruction (core dumped)
```

Root cause (confirmed via `gdb -batch -ex run -ex bt -ex 'x/4i $pc'`): the
binary's ELF constructor executes `whilewr` — an **SVE2** instruction —
and Orin's Cortex-A78AE CPU doesn't implement SVE at all
(`/proc/cpuinfo`'s `Features:` has `asimd`, no `sve`/`sve2`). Confirmed
`jetpack70`-specific: the sibling `lib_aarch64_jetpack61` binary
disassembles with zero SVE instructions, but *that* one wants
`libcusolver.so.11`/`libcublas.so.12` (CUDA 11/12-era, not present on this
CUDA-13.2-only host). Neither published aarch64 variant works here. Full
diagnosis: `isaac-ros-nitros-source-build.md`'s "Stage 6" section.

**Not fixed — genuinely blocked.** This isn't something a build flag on
our end can work around (cuVSLAM's source isn't published, only the
`.so`); recommend reporting to NVIDIA as a build-config bug in their
`aarch64_jetpack70` cuVSLAM release. Also worth noting independent of the
crash: cuVSLAM has no monocular-only tracking mode (stereo, stereo+IMU,
or RGBD only), and this robot's only camera is a monocular C920 — even a
fixed binary wouldn't be usable on the current robot hardware without
adding a depth or stereo camera. The proof-of-life scripts written for
this stage (`visual_slam_launch.py`/`visual_slam_check.py`, RGBD mode
against NVIDIA's own bundled RealSense test rosbag) are left in place,
currently non-functional pending the binary fix.

## Stage 7 — `isaac_ros_unet`: semantic segmentation, SUCCESS

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation
ln -sfn ../../isaac_ros_image_segmentation/isaac_ros_unet ws/src/isaac_ros_unet
ln -sfn ../../isaac_ros_image_segmentation/isaac_ros_unet_kernels ws/src/isaac_ros_unet_kernels
```

`isaac_ros_unet/package.xml` lists `<exec_depend>isaac_ros_triton</exec_depend>`
(an alternate inference backend, unused here — everything goes through
`isaac_ros_tensor_rt` same as Stage 5). Remove that line from the local
checkout or `colcon build` fails looking for a package that was never
cloned:

```console
cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_ros_unet_kernels isaac_ros_unet \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
cp isaac_ros_image_segmentation/isaac_ros_unet/test/dummy_model/model.dummy.onnx models/

pixi run -e isaac-nitros test-unet
```

Both packages built clean. `test-unet` runs the full `image ->
dnn_image_encoder -> tensor_rt -> unet_decoder` chain against NVIDIA's own
`model.dummy.onnx` (random weights) — checks that structurally valid raw
(`mono8`) and colorized (`rgb8`) segmentation masks come back at the
expected 960×544 network resolution, not mask accuracy (meaningless with
random weights, same scope as Stage 5 follow-up). See
`isaac-ros-nitros-source-build.md`'s "Stage 7" section for the
`isaac_ros_triton` exec_depend fix and full result detail.

**Not yet done:** a real trained segmentation model; wiring into the live
camera or `perseus_isaac_relay`.

## Stage 8 — `isaac_ros_centerpose`: monocular 3D pose estimation, SUCCESS

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation
ln -sfn ../../isaac_ros_pose_estimation/isaac_ros_centerpose ws/src/isaac_ros_centerpose
ln -sfn ../../isaac_ros_nitros/isaac_ros_gxf_extensions/gxf_isaac_messages ws/src/gxf_isaac_messages
```

`isaac_ros_centerpose/package.xml` has a hard `<depend>isaac_ros_triton</depend>`
(unused inference backend — same class of issue as Stage 7's
`isaac_ros_unet`, this time a `<depend>` not `exec_depend`). Remove that
line from the local checkout first.

```console
cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select gxf_isaac_messages isaac_ros_nitros_detection3_d_array_type isaac_ros_centerpose \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
cp isaac_ros_pose_estimation/isaac_ros_centerpose/test/models/centerpose_shoe.onnx models/

pixi run -e isaac-nitros test-centerpose
```

Unlike every prior DNN stage, `centerpose_shoe.onnx` is a **real trained
model** (not random weights), and NVIDIA ships a matching
`ground_truth.json` — so this is the first check in the series that
validates actual content (detection count + depth), not just message
structure. Result: `CENTERPOSE OK` — 2/2 detections matching ground
truth, depths within ~0.5 m of the measured values. Full detail:
`isaac-ros-nitros-source-build.md`'s "Stage 8" section.

**Not yet done:** wiring into the live camera or `perseus_isaac_relay`;
a robot-relevant trained model instead of NVIDIA's demo shoe.

## Stage 9 — `isaac_ros_compression`: BLOCKED, no hardware encoder on this Orin Nano SKU

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_compression
ln -sfn ../../isaac_ros_compression/isaac_ros_h264_encoder ws/src/isaac_ros_h264_encoder
ln -sfn ../../isaac_ros_compression/isaac_ros_h264_decoder ws/src/isaac_ros_h264_decoder
ln -sfn ../../isaac_ros_nitros/isaac_ros_nitros_type/isaac_ros_nitros_compressed_image_type ws/src/isaac_ros_nitros_compressed_image_type

cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_ros_nitros_compressed_image_type isaac_ros_h264_encoder isaac_ros_h264_decoder \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
```

Both packages build clean (no CUDA code, no `isaac_ros_triton`-style
dependency issue). **Blocked at runtime, both directions**: `EncoderNode`
can't open `/dev/v4l2-nvenc` — this Jetson **Orin Nano** SKU has no
hardware video encoder block at all (`ls /dev` shows `v4l2-nvdec` but no
`v4l2-nvenc`; only Orin NX/AGX have NVENC). `DecoderNode` (the hardware
this SoC *does* have) gets further — it parses NVIDIA's own test
`.h264` fixture and detects resolution (`Decoded video: 460x460`) — but
then fails inside NVIDIA's closed-source `libnvbufsurface`:
`NvBufSurfaceMapCudaBufferImpl: API is not supported on this platform`.
Neither half is fixable from our side. See
`isaac-ros-nitros-source-build.md`'s "Stage 9" section and `ERRORS.md`
for full diagnosis. `h264_decode_launch.py`/`h264_decode_check.py` are
left in place as a decode-only repro of the failure (no pixi task wired
— it can't pass).

## Stage 10 — combined pipeline: four capabilities running together, SUCCESS

```console
cd software/isaac-ros-native
pixi run -e isaac-nitros test-combined
```

No new clones needed — this composes AprilTag + YOLOv8 + U-Net +
CenterPose (Stages 4/5-follow-up/7/8) into **one** container, each in
its own ROS namespace so their identically-named relative topics
(`image`, `tensors`, `tensor_pub`, `tensor_sub`, ...) don't collide.
Every capability above had only ever been proven running alone; this is
the first test of whether they coexist — four TensorRT engines (plus
VPI/cuAprilTag) sharing one GPU/CUDA context/process.

First attempt found a real bug in the *test harness* (not Isaac ROS): an
all-zero camera matrix crashed the entire container via an uncaught
`cv::Exception` inside CenterPose's PnP solve — worth noting as a
general risk for multi-node containers (one node's unguarded assertion
takes down every other node sharing the process, not just itself).
Fixed by supplying real intrinsics. Result: `COMBINED OK` — AprilTag 1
detection, YOLOv8 10 detections, U-Net 960×544 mask, CenterPose 2
detections (matching Stage 8's ground truth). Full detail:
`isaac-ros-nitros-source-build.md`'s "Stage 10" section.

## Stage 11 — `isaac_ros_dnn_stereo_depth` (ESS): builds clean, runtime PARTIAL (Tegra CMA limit)

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth
ln -sfn ../../isaac_ros_nitros/isaac_ros_nitros_type/isaac_ros_nitros_disparity_image_type ws/src/isaac_ros_nitros_disparity_image_type
ln -sfn ../../isaac_ros_dnn_stereo_depth/isaac_ros_dnn_stereo_decoder ws/src/isaac_ros_dnn_stereo_decoder
ln -sfn ../../isaac_ros_nitros/isaac_ros_gxf_extensions/gxf_isaac_ros_messages ws/src/gxf_isaac_ros_messages
ln -sfn ../../isaac_ros_nitros/isaac_ros_nitros_type/isaac_ros_nitros_point_cloud_type ws/src/isaac_ros_nitros_point_cloud_type
ln -sfn ../../isaac_ros_image_pipeline/isaac_ros_stereo_image_proc ws/src/isaac_ros_stereo_image_proc

cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select gxf_isaac_ros_messages isaac_ros_nitros_point_cloud_type isaac_ros_stereo_image_proc isaac_ros_dnn_stereo_decoder isaac_ros_nitros_disparity_image_type \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
cp isaac_ros_dnn_stereo_depth/isaac_ros_ess/test/dummy_model.onnx models/ess_dummy_model.onnx

pixi run -e isaac-nitros bash -c './run_test_ess_stereo.sh'
```

All five packages build clean, including a real `.cu.cpp` CUDA kernel
(`filter_disparity.cu.cpp`, unlike Stage 5's pure-TensorRT-API node).
**Runtime crashes on the first frame** with `NvMapMemAllocInternalTagged
failed: error 12` → `Failed to create CUDA memory pool ... out of
memory` — despite gigabytes of general system RAM free. Root cause:
Tegra's CMA carveout (`/proc/meminfo`'s `CmaTotal`/`CmaFree`, 256MB
total, separate from ordinary RAM) is nearly exhausted by the desktop
GUI alone (~34MB free at rest), and this 15-node pipeline's six
VPI/`NvBufSurface`-backed image nodes need more contiguous headroom than
that. Two possible fixes — raising the boot-time `cma=` size, or freeing
the carveout by stopping the desktop compositor — both require a system
config decision, so this is left as build-verified/runtime-blocked
rather than force-fixed. Full diagnosis: `isaac-ros-nitros-source-build.md`'s
"Stage 11" section and `ERRORS.md`.

## Stage 12 — `isaac_ros_occupancy_grid_localizer`: lidar-matched capability, SUCCESS

```console
cd software/isaac-ros-native
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization
ln -sfn ../../isaac_ros_mapping_and_localization/isaac_ros_pointcloud_utils ws/src/isaac_ros_pointcloud_utils
ln -sfn ../../isaac_ros_mapping_and_localization/isaac_ros_occupancy_grid_localizer ws/src/isaac_ros_occupancy_grid_localizer
ln -sfn ../../isaac_ros_nitros/isaac_ros_nitros_type/isaac_ros_nitros_flat_scan_type ws/src/isaac_ros_nitros_flat_scan_type
ln -sfn ../../isaac_ros_common/isaac_ros_pointcloud_interfaces ws/src/isaac_ros_pointcloud_interfaces

cd ws
pixi run -e isaac-nitros bash -c '
  export CXX=/usr/bin/g++ CC=/usr/bin/gcc
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export CMAKE_PREFIX_PATH="/usr/local/cuda-13.2/targets/sbsa-linux:${CMAKE_PREFIX_PATH}"
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include ${CXXFLAGS:-}"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib ${LDFLAGS:-}"
  colcon build --packages-select isaac_ros_pointcloud_interfaces isaac_ros_nitros_flat_scan_type isaac_ros_pointcloud_utils isaac_ros_occupancy_grid_localizer \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CUDA_ARCHITECTURES=87
'
cd ..
pixi run -e isaac-nitros test-ogl
```

First capability in this whole experiment matched to the robot's **2D
lidar** (via `NitrosFlatScan`) rather than its camera — GPU-accelerated
global relocalization against an occupancy grid map, complementary to
(not redundant with) the existing `slam_toolbox`-based mapping stack.
Also new CUDA territory: `occupancy_grid_localizer_gpu.cu` does
GPU-parallel batch scan-matching, a different shape than every prior
TensorRT-based stage.

Ruled out the other two `isaac_ros_mapping_and_localization` sub-packages
first: `isaac_ros_visual_global_localization` `exec_depend`s on
`isaac_ros_visual_slam` (Stage 6's blocked cuVSLAM); `isaac_mapping_ros`
needs `nvblox_ros` (depth camera). `isaac_ros_occupancy_grid_localizer`
had a clean dependency chain.

Build was clean. Found one real bug in our own launch file (not Isaac
ROS): the map image path only resolves correctly if the `.yaml` file is
*also* passed as a raw `parameters=[...]` list entry (ROS 2 launch loads
a bare `.yaml` string as a parameters file), not just as a dict value —
first attempt left the `image` param empty and failed with a truncated
path. Fixed to match NVIDIA's own test exactly.

NVIDIA ships **real** test data here (unlike several earlier
dummy-weight stages): an actual occupancy grid map and a rosbag of 12
genuine recorded lidar scans. Result: `OGL OK` — recovered pose
`(33.60, 7.70)` against a ground truth of `(33.5, 7.75)`, orientation
matching within NVIDIA's own tolerances. Full detail:
`isaac-ros-nitros-source-build.md`'s "Stage 12" section.

**Not yet done:** wiring against this robot's actual lidar and a map of
its actual environment (this used NVIDIA's own fixtures).

## Full Isaac ROS map

A separate pass mapped all ~29 Isaac ROS GEM repositories against this
experiment's progress (pulled live via the GitHub API). As of Stage 12:
**seven GEM repos verified working from source** (AprilTag, DNN inference,
image_pipeline, object detection, image segmentation, pose estimation,
mapping_and_localization), **three built but blocked at runtime** (visual
SLAM/cuVSLAM — Stage 6, an upstream NVIDIA binary defect; compression —
Stage 9, this Orin Nano's missing hardware encoder plus an unsupported
decode CUDA-interop path; dnn_stereo_depth — Stage 11, a Tegra CMA
memory-carveout limit), **three need a depth/stereo camera** this robot
doesn't currently have, **seven are architecturally inapplicable** to
this robot (Nova-platform hardware, CSI camera drivers, ROS1 bridge,
Unitree G1-specific packages), and roughly nine more are relevant but
untried (cuMotion, manipulation, jetson-stats, teleop, etc.).

A standalone LaTeX/PDF summary of this whole effort — architecture
diagram, stage-outcome table, root-cause table for every blocked stage,
and the full GEM-repository map as charts — is in `report/`; see
`report/README.md` to rebuild it after future stages.
