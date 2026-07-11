# NITROS/GXF from-source build experiment (Orin + JetPack 7)

**Status: experimental, staged, not yet attempted on hardware.** This is a
separate track from the working `software/docker/isaac-ros/` scaffold
(NVIDIA's prebuilt image, JetPack 6.x only) — see
[isaac-ros.md](isaac-ros.md). It exists because NVIDIA has not shipped Isaac
ROS binaries for Jetson Orin under JetPack 7 yet, and this robot is now on
JetPack 7 (confirmed 2026-07-08: L4T R39.2.0, CUDA 13.2, Ubuntu 24.04). Rather
than wait, this track attempts to build NITROS (`isaac_ros_nitros`) and its
GXF dependency from source, targeting Orin/JetPack7/CUDA13 directly.

Working files live in `software/isaac-ros-native/` — see that
directory's README for the actual commands. This page is the "why" and the
staged plan; the README is the "how."

This track has been extracted into a standalone, generalized public repo
for other Jetson/JetPack7 users:
[DingoOz/isaac-nitros-jetpack7](https://github.com/DingoOz/isaac-nitros-jetpack7).
The copy under `software/isaac-ros-native/` here remains the working copy
for perseus-lite's own development and isn't automatically kept in sync
with the extracted repo.

## Why this might work at all

NVIDIA's official Isaac ROS 4.x line (JetPack 7 / Ubuntu 24.04 / ROS 2 Jazzy)
is Jetson Thor–only — confirmed repeatedly by NVIDIA staff:

> "Isaac ROS 4.0 does not currently support Orin platforms."
> — NVIDIA staff, [isaac_ros_nitros#64](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/issues/64), 2025-11-27

But two things suggest Orin+JetPack7 isn't architecturally excluded, just not
shipped yet:

1. Isaac ROS's own CMake config (`isaac_ros_common-extras.cmake`) explicitly
   compiles fresh aarch64 code for **`CMAKE_CUDA_ARCHITECTURES 87;110;120`** —
   SM 8.7 is Orin's Ampere compute capability, alongside Thor/Spark's
   Blackwell (110/120). This isn't a Thor-only build target by design.
2. A community report from a user running **JetPack 7.2 on an AGX Orin**
   (L4T 39.2, TensorRT 10.16.2.10, CUDA 13.0.85 — essentially this robot's
   stack) found the underlying CUDA/TensorRT/iGPU path functional; the only
   confirmed gap was TensorRT reporting zero DLA cores, which NVIDIA staff
   attributed to DLA support specifically, not the GPU compute path. ([forum
   thread](https://forums.developer.nvidia.com/t/jp-7-2-w-isaac-ros-and-dlas/374021))
3. NVIDIA staff, June 2026: *"We do plan to support JP 7.x on Orin, and this
   is an area the team is actively working on... not ready to share a public
   timeline yet."* ([forum
   thread](https://forums.developer.nvidia.com/t/eta-for-isaac-ros-release-for-jp-7-2-on-orin-devices/372134))

So the hardware/driver layer plausibly works; what's missing is NVIDIA
publishing the matching GXF binaries and doing the integration work.

## Why this might not work (the actual blocker)

`isaac_ros_nitros`'s own C++ (`NitrosNode`, type adapters,
`isaac_ros_managed_nitros`, etc.) is source-available and — once its
dependency resolves — should compile normally. The real blocker is its GXF
(Graph Execution Framework) dependency:

- The build system expects a `gxf_jetpack70/` binary directory for real
  embedded Jetson hardware on JetPack 7. **It does not exist in the public
  repo.** Only `gxf_aarch64_cuda_13_0` and `gxf_x86_64_cuda_13_0` (SBSA —
  server-ARM targets, e.g. DGX Spark / Thor) are published under
  `isaac_ros_nitros/isaac_ros_gxf/gxf/core/lib/`.
- The nominally "buildable GXF source" repo,
  [NVIDIA-ISAAC-ROS/gxf](https://github.com/NVIDIA-ISAAC-ROS/gxf), is
  **stale**: every release tag from `v3.2.0` through `v4.4.0` points at the
  *same* commit (dated 2025-01-10, "Isaac ROS 3.2.0"). That commit's
  `build_gxf_release_content.yaml` only defines `x86_cuda_12_6` and
  `jetpack61` build targets — nothing for JetPack7/CUDA13. The release tags
  are cosmetic; the actual buildable source hasn't moved past the JetPack
  6.1 era.
- `isaac_ros_common`'s `CMAKE_DEVICE` logic doesn't resolve to a Jetson-aware
  value in the generic path either — it deprecated the aarch64→"arm64" branch
  in favor of defaulting to `sbsa`. (The newer `isaac-ros-cli` tool's
  `platform.py` *does* distinguish real Jetson — `ARM64_JETPACK`, detected via
  `/etc/nv_tegra_release` — from DGX Spark's `ARM64_FASTOS`, but that
  detection doesn't currently have a matching binary to select on JetPack7.)

Net: nothing prevents *compiling* NITROS/GXF for SM 8.7 in principle, but the
actual published artifacts to link against for embedded Jetson on
JetPack7/CUDA13 don't exist yet, and the "source you could build them from
yourself" hasn't been updated to target that configuration either.

## Licensing caveats

- `isaac_ros_nitros` carries NVIDIA's source-available **"Isaac ROS Software
  License"**: permits install/use/modify, but restricts use to "systems with
  NVIDIA GPUs" and explicitly forbids relicensing under an open-source
  license (§4f).
- The `gxf` repo's actual core source (`com_nvidia_gxf/`) carries a
  **stricter, more classically proprietary** NVIDIA header: *"Any use,
  reproduction, disclosure or distribution of this software... without an
  express license agreement from NVIDIA CORPORATION is strictly
  prohibited."* This may just be stale boilerplate carried over from GXF's
  internal codebase, but it's a real ambiguity sitting inside a repo NVIDIA
  itself calls "buildable source."
- Practical stance: treat everything under `nitros-source/` as **local
  experimentation only** — nothing from these repos is committed to
  perseus-lite (see `.gitignore`), and nothing built here should be
  redistributed.

## Staged plan

The GXF binary gap is the real unknown, so the plan front-loads the cheapest
experiment that tests it directly, with an explicit decision gate before
committing to the much larger effort of resurrecting GXF's stale build
recipe.

| Stage | Goal | Gate |
| --- | --- | --- |
| 0 | Stand up a working build toolchain on this host (venv/baremetal `isaac-ros-cli`, since Docker isn't installed here) | `colcon build` configures against `isaac_ros_common` cmake macros |
| 1 | Force `CMAKE_DEVICE=sbsa`, link against the *existing* `gxf_aarch64_cuda_13_0` binaries, and run a minimal GXF smoke test on this Tegra hardware | **Decision gate.** Clean failure (ABI/arch mismatch) → binary-reuse path closed, go to Stage 2 or stop. Success → skip Stage 2 entirely |
| 2 | *(only if Stage 1 fails)* Attempt to extend the stale `gxf` repo's build recipe with a `jetpack70`/CUDA13 target | Time-boxed; stop if the `com_nvidia_gxf` licensing ambiguity turns out not to be stale boilerplate, or if progress stalls — fall back to waiting for NVIDIA's stated (undated) JP7-on-Orin work |
| 3 | Once a working `libgxf_core.so` exists (via 1 or 2), build `isaac_ros_nitros` core only (no perception packages) and run a minimal NITROS type-adapter round trip in one process | Round trip actually runs and prints expected content |
| 4 | *(out of scope for now)* `isaac_ros_image_proc` / `isaac_ros_apriltag` — introduces a second independent lock point, `isaac_ros_vpi_utils` (VPI), whose JetPack7 status is unchecked | Not started until Stage 3 succeeds |

Results and exact error output for each stage should be recorded in this
document as they happen, rather than in commit messages or PR descriptions —
this page is the running log.

### Stage 0 — result: succeeded

Cloned `isaac_ros_common`, `isaac_ros_nitros`, `gxf` into `nitros-source/`
(gitignored) without issue.

Standing up a build toolchain hit a bigger snag than expected: **every
documented path requires `sudo`/system-level installation.** After checking
with whoever owns this hardware, went with installing Docker +
`nvidia-container-runtime` directly on the host (matches what the existing
`software/docker/isaac-ros/` scaffold already assumes):

- `sudo apt-get install docker.io nvidia-container-toolkit` — both available
  straight from existing apt sources (`nvidia-container-toolkit` ships on
  NVIDIA's Jetson repo for `r39.2`/JetPack 7 already).
- `sudo nvidia-ctk runtime configure --runtime=docker --set-as-default`,
  `sudo systemctl enable --now docker` — verified with
  `docker run --gpus all nvcr.io/nvidia/l4t-base:r36.2.0 nvidia-smi`, which
  correctly reported the host's driver/GPU through the container (basic
  driver-passthrough sanity check, not a full CUDA-runtime compatibility
  test — see Stage 1).
- Did **not** add the invoking user to the `docker` group (that grants
  persistent root-equivalent host access, beyond what installing Docker
  itself implies) — verification commands use `sudo docker`.
- The newer `isaac-ros-cli` tool turned out **not pip-installable** despite
  its repo being Apache-2.0 — it ships as a `.deb` built via its own
  `Makefile`, which carries the same strict proprietary NVIDIA header
  flagged below for `gxf`'s core ("distribution... without an express
  license agreement... is strictly prohibited") sitting inside an
  otherwise-Apache-2.0 repo. Not pursued — the classic `run_dev.sh`/plain
  colcon route was used instead.
- Also installed: `cuda-toolkit-13-2` (nvcc; only the L4T CUDA *runtime* libs
  were preinstalled, not the compiler) and `vpi4-dev` (NVIDIA's Vision
  Programming Interface — turned out to be an `isaac_ros_common` build
  dependency, not just an `isaac_ros_apriltag` one as originally assumed;
  **VPI 4 is already published for this exact JetPack7/r39.2 repo**, a
  positive sign for that dependency specifically).
- A `pixi` feature (`isaac-nitros`, linux-aarch64 only) provides
  colcon/cmake/compilers in a self-contained Pixi prefix for the actual
  package builds, layered on top of the `default` feature's ROS 2 Jazzy/
  ament tooling (same pattern as `simulation`/`machine-learning`).

**Result: `isaac_ros_common` (the package) builds and installs cleanly** —
`libisaac_ros_common.so` compiled and installed via
`colcon build --packages-select isaac_ros_common` with
`CUDAToolkit_ROOT=/usr/local/cuda`, `CUDACXX=/usr/local/cuda/bin/nvcc`.
Notably, `CMAKE_DEVICE` **auto-detected as `sbsa` on this real Jetson**
without forcing it — confirming the research's prediction that the
build system's generic aarch64 path resolves to the SBSA/server-ARM branch
here, not a dedicated Jetson one.

One more data point worth flagging: the configure log reported
**`CUDA architectures: 75`** (Turing) for this build, not `87` (Orin/Ampere)
as `isaac_ros_common-extras.cmake`'s `CMAKE_CUDA_ARCHITECTURES "87;110;120"`
(found during earlier research) would suggest. Not yet root-caused — could be
a different code path taken under `CMAKE_DEVICE=sbsa` specifically, or a
default applied before that list is set. Compiling for SM 7.5 instead of 8.7
would still likely run on Orin via PTX JIT (if PTX is embedded, not just
cubin) but wouldn't use Orin-native code generation — worth revisiting if
Stage 3 is reached.

### Stage 1 — result: blocked on a build-tooling bug, not (yet) the
architecture question

Attempted `colcon build --packages-select isaac_ros_gxf` — the package that
actually links against the precompiled GXF `.so` binaries. Confirmed live
(matching the research exactly): only `gxf_aarch64_cuda_13_0` and
`gxf_x86_64_cuda_13_0` exist under `isaac_ros_gxf/gxf/core/lib/` — no
`gxf_jetpack70/`.

Never got far enough to test whether those binaries actually load on this
Tegra hardware — hit an earlier, unrelated **CMake generate-time error**:

```
CMake Error at CMakeLists.txt:71 (set_property):
  Error evaluating generator expression:
    $<INSTALL_PREFIX>
  INSTALL_PREFIX is a marker for install(EXPORT) only.  It should never be evaluated.
```

`isaac_ros_gxf/CMakeLists.txt` defines `Core`/`Logger`/`Std`/`Multimedia`/
`Serialization`/`Cuda` as `INTERFACE` libraries with
`INTERFACE_LINK_LIBRARIES`/`INTERFACE_INCLUDE_DIRECTORIES` set to
`$<INSTALL_PREFIX>/...` paths (lines 66–120), then does
`install(TARGETS ... EXPORT export_${PROJECT_NAME})` followed by
`ament_export_targets(...)` and `ament_auto_package(...)`. `$<INSTALL_PREFIX>`
is only valid when evaluated through `install(EXPORT)` machinery — something
in the `ament_export_targets`/`ament_auto_package` combination appears to
evaluate these target properties directly (not through a genuine
`install(EXPORT)` pass), which CMake rejects outright.

**Ruled out a simple CMake-version mismatch**: the package declares
`cmake_minimum_required(VERSION 3.22.1)`, and this failed *identically* on
both CMake 4.3.4 (conda-forge default) and CMake 3.31.8 (pinned down via the
`isaac-nitros` Pixi feature to test the hypothesis). Same error, same lines,
both versions. So this isn't "too-new-CMake" — it's either a bug that's
always been latent in this exact standalone-colcon build path (plausible:
NVIDIA's own CI/testing may never actually exercise the `sbsa` branch of
this file on real Jetson hardware, since Orin+JetPack7 isn't in their
matrix), or something about NVIDIA's actual reference build environment
(their `run_dev.sh` dev container, specific `CMAKE_BUILD_TYPE`/generator/cache
variables) avoids triggering it in a way not yet identified.

**Update: patched and unblocked.** `$<INSTALL_PREFIX>` (12 occurrences) was
replaced with `${CMAKE_INSTALL_PREFIX}` (a plain CMake variable, evaluated at
configure time rather than requiring `install(EXPORT)` context) via `sed` on
the local, gitignored clone — permitted under the Isaac ROS Software
License's modify rights, not redistributed. `isaac_ros_gxf` then built and
installed cleanly.

**Second snag, quickly resolved: the "binaries" were Git LFS pointers, not
real `.so` files.** `libgxf_core.so` etc. were 132-byte ASCII text (LFS
pointer format), because `git-lfs` wasn't installed when the repos were
cloned — CMake's `install(FILES ...)` doesn't validate binary format, so the
build "succeeded" while silently installing placeholder text files. Fixed
with `sudo apt-get install git-lfs` (small, standard package) +
`git lfs install --local && git lfs pull` inside `isaac_ros_nitros/`. After
that, `libgxf_core.so` was a real 2.5 MB aarch64 ELF shared object matching
the LFS pointer's declared size/hash.

### Stage 1 — the actual decision-gate test: SUCCESS

Wrote a minimal C++ smoke test (`nitros-source/gxf_smoke_test.cpp`, our own
code, not NVIDIA's) calling `GxfContextCreate`/`GxfContextDestroy` from
`gxf/core/gxf.h` — the most basic possible exercise of the GXF runtime.

Two more build-recipe snags on the way to actually running it, both worth
recording for whoever continues this:

1. **Link with the system compiler, not Pixi's conda-forge cross-toolchain.**
   Linking with `aarch64-conda-linux-gnu-g++` (the compiler `isaac-nitros`'s
   `cxx-compiler` package provides) failed with `undefined reference` to
   versioned glibc symbols (`dlopen@GLIBC_2.34`, `__isoc23_strtoull@GLIBC_2.38`,
   etc.) — conda-forge's toolchain targets an old, portable glibc baseline by
   design, but NVIDIA's binaries are built natively against Ubuntu 24.04's
   glibc 2.39. Switched to the host's own `/usr/bin/g++` (already present via
   `build-essential`, pulled in as a dependency somewhere along the way) for
   this link step, which resolved it immediately.
2. **`RUNPATH` doesn't propagate transitively.** `libgxf_core.so` itself
   needs `libgxf_logger.so`; setting `-Wl,-rpath` on our executable covers
   *our* direct link but not `libgxf_core.so`'s own `NEEDED` entries (modern
   `DT_RUNPATH`, unlike legacy `DT_RPATH`, only applies to the object that
   carries it, not what that object subsequently loads). Ran with
   `LD_LIBRARY_PATH` set instead.

Result:

```
$ ./gxf_smoke_test
GxfContextCreate OK: context=0xaaaafcddaa00
GxfContextDestroy OK
```

**The published `gxf_aarch64_cuda_13_0` (SBSA/server-ARM) GXF core binary
loads and initializes correctly on this Jetson Orin Nano's Tegra driver/
unified-memory stack under JetPack 7.** This directly answers the Stage 1
decision gate: the "reuse NVIDIA's existing binaries" path is **open**, not
closed. **Stage 2 (resurrecting the stale `gxf` build recipe to produce a
dedicated `gxf_jetpack70` target) can likely be skipped entirely** — there's
no evidence yet that the SBSA build is unsuitable for Orin at the level
tested (context lifecycle only; no CUDA kernel execution, no NITROS message
passing, no perception pipeline).

Caveats on how far this result reaches:
- Only tested `GxfContextCreate`/`GxfContextDestroy` — no GXF graph execution,
  no CUDA interop (`libgxf_cuda.so`), no NITROS layer above this yet.
- Still unresolved: the `CUDA architectures: 75` (Turing) vs `87`
  (Orin/Ampere) discrepancy noted in Stage 0 — irrelevant to *this* binary
  (which is prebuilt, not something we compiled), but relevant once Stage 3
  compiles fresh NITROS C++ code that needs to target Orin's actual SM
  correctly.
- This is one data point on one board. Not a substitute for NVIDIA's own
  validation, and not something to treat as "Orin is officially supported."

### Stage 3 — result: SUCCESS, real NITROS round trip running on Orin/JetPack7

Built `isaac_ros_nitros` (the `NitrosNode`/`NitrosContext`/type-adapter core)
and its full dependency chain — `isaac_ros_common`, `isaac_ros_gxf`, all 17
`gxf_isaac_*` extension packages, `negotiated`/`negotiated_interfaces`
(cloned separately from
[osrf/negotiated](https://github.com/osrf/negotiated), Apache-2.0/Boost,
gitignored same as the other clones — not in RoboStack) — via
`colcon build --packages-up-to isaac_ros_nitros`.

Three more snags on the way, all resolved:

1. **The `$<INSTALL_PREFIX>` bug from Stage 1 is systemic, not a one-off.**
   17 `CMakeLists.txt` files across `isaac_ros_managed_nitros` and nearly
   every `gxf_isaac_*` extension use the identical broken pattern (clearly
   generated from a shared template). Batch-patched all 17 with the same
   `sed` substitution.
2. **`magic_enum` is a real, correctly-declared dependency
   (`isaac_ros_gxf/package.xml` already lists it) that just isn't installed
   anywhere.** Added `magic_enum` to the `isaac-nitros` Pixi feature
   (conda-forge has it, MIT-licensed). Once installed, `isaac_ros_gxf`
   linked fine, but consuming code failed with
   `fatal error: magic_enum.hpp: No such file or directory` — **conda-forge
   packages it under `include/magic_enum/magic_enum.hpp`, but NVIDIA's code
   does a flat `#include "magic_enum.hpp"`, expecting it directly on the
   include path** (their own build presumably vendors a single-file copy
   rather than this subdirectory layout). Worked around with
   `CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum"` rather than patching
   NVIDIA's `#include` lines. Needed a clean `rm -rf build/isaac_ros_nitros
   install/isaac_ros_nitros` afterward — CMake caches
   `CMAKE_CXX_FLAGS` at first configure and won't pick up an env var change
   without a fresh configure.
3. Built cleanly after that (some harmless C++20-designated-initializer
   warnings — the code assumes `-std=c++20`, worth setting explicitly if
   this becomes a permanent build, not investigated further here).

**NVIDIA ships its own minimal NITROS proof-of-life test** —
`isaac_ros_nitros/test/isaac_ros_nitros_test_pol.py` — built automatically
since `BUILD_TESTING` defaults on: two chained `NitrosEmptyForwardNode`
composable nodes in one `component_container_mt`, publish `std_msgs/Empty`
on one end, verify it arrives on the other after passing through NITROS
negotiation and GXF graph execution twice. Running it via the official
`isaac_ros_test`/`launch_testing`/`pytest.mark.rostest` harness would have
needed `pip install torch` — `isaac_ros_test/__init__.py` eagerly imports
`MockModelGenerator`, which imports `torch`, for a model-mocking helper
unrelated to this test. Skipped that weight for a **minimal** round trip:
wrote our own `nitros_roundtrip_launch.py` (replicates the exact
`ComposableNodeContainer` setup) and `nitros_roundtrip_check.py` (plain
`rclpy` publish/subscribe, no `launch_testing`/`isaac_ros_test`) instead —
both in `nitros-source/`, our own code, not NVIDIA's.

One bug in our own launch file along the way: the second node's remappings
used relative topic names (`'mid1/topic_forward_input'`), which resolved
*relative to that node's own `mid1` namespace* — silently doubling to
`/mid1/mid1/topic_forward_input`, which matched nothing. Fixed by making the
remap `from`/`to` strings absolute (leading `/`).

Result, run on this Jetson Orin Nano, JetPack 7, `ROS_DOMAIN_ID=51`:

```
$ ros2 launch nitros_roundtrip_launch.py &
...
[nitros_stage1]: [NitrosNode] Starting negotiation...
[nitros_stage1]: [NitrosPublisher] Use only the compatible publisher: topic_name="/topic_forward_output", data_format="nitros_empty"
[nitros_stage1]: [NitrosNode] Wrote the final top level YAML graph to "/tmp/isaac_ros_nitros/graphs/.../....yaml"
[nitros_stage1]: [NitrosNode] Initializing and running GXF graph
[nitros_stage1]: [NitrosNode] Node was started
[mid1.nitros_stage2]: [NitrosNode] Node was started
...

$ python3 nitros_roundtrip_check.py
[nitros_roundtrip_check]: sent=1 received=1
ROUNDTRIP OK
```

Also ran NVIDIA's own `test_cuda_stream_pool` gtest (built automatically
alongside the round-trip test, exercises `libgxf_cuda.so` directly — real
CUDA stream acquire/release/reuse/overflow, not just context lifecycle):
**14/14 passed.**

**What this establishes:** real NITROS negotiation, GXF graph
construction/execution, and CUDA stream management via the `gxf_aarch64_cuda_13_0`
(SBSA) binaries all work correctly on this Orin Nano under JetPack 7 — well
beyond Stage 1's basic context-lifecycle check. This is still one board, one
minimal (`nitros_empty` — no actual payload/tensor data) message type, and
no camera/perception pipeline (`isaac_ros_apriltag`/`isaac_ros_image_proc`,
gated on `isaac_ros_vpi_utils`, is Stage 4 — not started).

Still unresolved from Stage 0: the `CUDA architectures: 75` vs `87`
discrepancy — now more relevant, since Stage 3 compiled real NITROS C++
(unlike Stage 1's prebuilt GXF binaries). Worth checking whether Orin-native
(SM 8.7) codegen is actually happening before scaling this up.

### Stage 4 — result: SUCCESS, real AprilTag detection matches NVIDIA's exact ground truth

Built `isaac_ros_apriltag` (real perception, not just NITROS plumbing) and
its dependency chain: `isaac_ros_vpi_utils`, `isaac_ros_cvcuda_utils`,
`isaac_ros_image_proc`, `isaac_ros_nitros_camera_info_type`. Two more repos
cloned (same gitignore treatment as before, not committed):
[NVIDIA-ISAAC-ROS/isaac_ros_apriltag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag)
and
[NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)
(contains `isaac_ros_image_proc`, `isaac_ros_cvcuda_utils`,
`isaac_ros_vpi_utils`, plus depth/stereo variants we didn't need).

**A new external dependency, resolved via a different channel than
Isaac ROS's own binaries:** `isaac_ros_image_proc` (and `isaac_ros_cvcuda_utils`)
directly requires NVIDIA's **CV-CUDA** library (`cvcuda0-dev`), which isn't in
any apt repo here. Unlike GXF, CV-CUDA is a
[separately maintained, actively released open-source project](https://github.com/CVCUDA/CV-CUDA)
(Apache-2.0) with its own GitHub releases — found a prebuilt `.deb` matching
this exact platform: `cvcuda-{lib,dev}-0.16.0-cuda13-aarch64-linux.deb`.
Downloaded and `sudo dpkg -i`'d both (lib first, then dev).

Four more build-recipe snags, all following patterns already established in
Stages 1/3:

1. **VPI/CV-CUDA headers not found despite being installed** — same root
   cause as `magic_enum` in Stage 3: conda-forge's cross-compiler doesn't
   search `/usr/include` (even though `/usr/include/vpi` and
   `/usr/include/cvcuda` are valid `update-alternatives` symlinks to
   `/opt/nvidia/{vpi4,cvcuda0}/include`). Fixed by pointing `CXXFLAGS`
   directly at `/opt/nvidia/vpi4/include` and `/opt/nvidia/cvcuda0/include`
   — **not** `-I/usr/include` broadly, which was tried first and **broke
   the build worse**: it made the conda cross-compiler pick up the host's
   `glibc` `features.h` ahead of its own bundled one, cascading into
   `bits/timesize.h: No such file or directory` (a header that exists in
   Ubuntu's glibc layout but not where conda's toolchain expects it). Scope
   `-I` additions to the exact directory needed, never a broad system path.
2. **`isaac_ros_cvcuda_utils`'s library needed `-L`/`-rpath` for
   `/opt/nvidia/cvcuda0/lib`** too — added via `LDFLAGS`.
3. **Test executables in `isaac_ros_cvcuda_utils` hit the same
   conda-toolchain-vs-native-glibc linking failure as the Stage 1 GXF smoke
   test** (`undefined reference to dlopen@GLIBC_2.34` etc.) — but this time
   there were many of them across multiple packages, so re-linking each
   individually with the system compiler wasn't practical. Used
   `--cmake-args -DBUILD_TESTING=OFF` instead once the actual libraries were
   confirmed building fine; skips gtest binaries for these packages, not
   full build breakage.
4. **`colcon build --packages-select isaac_ros_apriltag` alone fails** even
   though `isaac_ros_image_proc` is only an `exec_depend`, not a `depend` —
   colcon's `ament_cmake` build task sources every dependency's environment
   hook (`package.sh`) regardless of depend type, so `isaac_ros_image_proc`
   genuinely has to be built first regardless of whether apriltag's own
   compiled code touches it.
5. `isaac_ros_image_proc` itself just took a while to compile (real CUDA/
   CV-CUDA kernel code) — ran it in the background rather than assuming a
   hang.

**Verification — not just "it compiles," but a pixel-exact match against
NVIDIA's own test fixture.** `isaac_ros_apriltag` ships a real ground-truth
test case (`isaac_ros_apriltag/test/test_cases/apriltag0/`: a `1920x1080`
PNG with a physical `tag36h11` id=0 tag, matching `camera_info.json`, and
exact expected corner/center/pose values baked into
`isaac_ros_apriltag_pol_test.py`). Running that official test needs
`isaac_ros_test` (torch, again) — wrote `apriltag_check.py` instead: loads
the same PNG via `cv_bridge`, publishes it + the camera info, subscribes to
`tag_detections`, and checks the detection against NVIDIA's exact expected
values (2px corner/center tolerance, matching their own test).

```
$ ros2 launch apriltag_launch.py &
...
[apriltag]: Using cuAprilTag implementation.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/apriltag' in container '/apriltag_container'

$ python3 apriltag_check.py
[apriltag_check]: sent=5 received=True
id=0 family=tag36h11 center=(926.0,547.0) expected_center=(926.0, 547.0)
APRILTAG OK
```

Exact pixel match (`926.0, 547.0`) against NVIDIA's own precomputed ground
truth — this is real `cuAprilTag`/VPI-based detection producing correct
results, on this Orin Nano, under JetPack 7, end to end: image →
NITROS-wrapped tensor → VPI/cuAprilTag GPU detection → `AprilTagDetectionArray`.

**What Stage 4 establishes beyond Stage 3:** actual GPU compute correctness
(not just "the process didn't crash") for a real perception algorithm,
across three additional NVIDIA-maintained native libraries (VPI, CV-CUDA,
plus the AprilTag detector itself) all functioning correctly together on
this hardware/JetPack combination.

### Stage 4 follow-up — live camera (Logitech C920): works, with a real bug found in rectify+apriltag

Added `ros-jazzy-usb-cam` to the `isaac-nitros` Pixi feature and a
`pixi run -e isaac-nitros test-apriltags` task
(`run_test_apriltags.sh` → `apriltag_camera_launch.py`) for a live-camera
test, since the ground-truth ap test above only used a static image.

**Two more issues found, one fixed, one worked around (not yet root-caused):**

1. **Resolution/calibration mismatch in NVIDIA's own bundled config.**
   `isaac_ros_apriltag_usb_cam.launch.py` + `usb_cam_params.yaml` doesn't
   set `image_width`/`image_height`, so `usb_cam` defaults to capturing
   640×480 — but the bundled `camera_info.yaml` it loads is calibrated for
   1280×720. `usb_cam` still publishes that mismatched calibration on
   `/camera_info` without erroring, and `RectifyNode` then produces a
   wrongly-scaled `/image_rect`. Fixed by writing our own
   `camera_c920_params.yaml` (1280×720 capture, matching a
   `camera_c920_info.yaml` also at 1280×720) and our own launch file rather
   than editing NVIDIA's installed config (which colcon would overwrite on
   rebuild anyway). **Caveat:** `camera_c920_info.yaml`'s intrinsics are
   NVIDIA's own placeholder test values, not a real calibration for this
   physical C920 — fine for demoing detection, not for trusting the
   reported pose. A real calibration needs
   `ros2 run camera_calibration cameracalibrator` with a checkerboard.

2. **`rectify` → `apriltag` on live camera frames reproducibly crashes —
   not yet root-caused.**
   ```
   terminate called after throwing an instance of 'nvcv::Exception'
     what():  NVCV_ERROR_INVALID_OPERATION: The tensor handle is null.
   ```
   Isolated carefully (each combination tested independently, with a clean
   process state — an earlier false lead came from a zombie `ros2 launch`
   process left over from a prior crash still holding the `apriltag_container`
   node name, which produced a *different*, misleading symptom: the next
   launch attempt hung indefinitely with zero node-loading progress. Always
   force-kill (`pkill -9 -f component_container_mt`) and verify with `ps`
   before re-testing after a crash.):
   - `apriltag` alone (static test image, no rectify): works
     (Stage 4 ground-truth test, and again standalone here).
   - `rectify` + `apriltag` loaded together, **no live frames flowing**
     (no publisher feeding rectify): loads fine, no crash.
   - `usb_cam` + `rectify` + `apriltag`, **live frames actively flowing**:
     crashes consistently, within ~1 second of `apriltag` finishing its
     `load_node` call — i.e. specifically when `apriltag` starts actually
     consuming `rectify`'s live NITROS-wrapped image output, not during
     either node's own initialization alone.
   - Node load order doesn't matter (tried apriltag-before-usb_cam too;
     same crash once real frames reach the rectify→apriltag handoff).

   This points at the interaction between `RectifyNode`'s NITROS/CV-CUDA
   image output and `AprilTagNode`'s consumption of it under real
   throughput — plausibly related to the still-unresolved `CUDA
   architectures: 75` vs `87` question from Stage 0, or a CV-CUDA
   version/ABI mismatch specific to that data path, but not confirmed.
   **Worked around, not fixed:** `apriltag_camera_launch.py` skips
   `RectifyNode` entirely and feeds `usb_cam`'s raw image straight to
   `AprilTagNode`. Verified stable (`/tag_detections` publishing steadily
   at 10 Hz, matching the camera's framerate, no crash) — but the C920's
   lens distortion goes uncorrected, so corner/pose accuracy is worse than
   the rectified path would give. Revisit this if accurate pose (not just
   "is a tag visible") matters later.

**Result:** live AprilTag detection works —
`pixi run -e isaac-nitros test-apriltags` launches `usb_cam` (1280×720,
10 Hz) → `apriltag` directly, publishing `/tag_detections` and per-tag TF
frames (`tag36h11:<id>`) at a steady 10 Hz. RViz visualization from a
second machine: see `nitros-source/README.md` "Live camera test."

Also found and fixed a bug in *our own* launch file while getting this far:
`os.path.dirname(__file__)` stays relative when `ros2 launch` is invoked
with a relative path (as `run_test_apriltags.sh` does) — this produced an
invalid `file://../camera_c920_info.yaml` calibration URL, which `usb_cam`
silently failed to load, falling back to an all-zero `camera_info`. Corner/
ID detection doesn't need the K matrix, so it kept working — but pose
estimation produced `NaN`, which is also why the `camera` TF frame appeared
to never exist (it only shows up as the parent of a *published* transform,
and nothing valid was ever published). Fixed with `os.path.abspath()`.

### Root-caused: the rectify+apriltag crash is a genuine race condition

Confirmed via a controlled experiment: launched the **full**
`camera → rectify → apriltag` graph (`usb_cam` → `RectifyNode` →
`AprilTagNode`, the actual deployed pipeline, not the rectify-skipping
workaround) wrapped in `gdb -batch -ex run -ex "thread apply all bt"`.
**It ran stably for several minutes with all three nodes loaded,
`/tag_detections` publishing at a steady 10 Hz — no crash.** Running the
identical graph without `gdb` crashes reproducibly within ~1 second of
`apriltag` finishing its `load_node` call, every time. That's the
signature of a genuine data race: `gdb`'s ptrace overhead perturbs
scheduling enough to avoid whatever timing window the bug depends on.

**Where the race most likely lives, based on reading the actual code**
(not yet proven with a live backtrace — gdb's startup overhead when
loading debug info for this many shared libraries made getting one
impractical in reasonable time; a core-dump-based post-mortem approach
was set up but not completed before the gdb experiment already gave a
confident answer):

- `RectifyNode` subscribes to `image_raw` as a NITROS-native `NitrosImage`
  (`isaac_ros_image_pipeline/isaac_ros_image_proc/src/rectify_node.cpp`).
  `usb_cam` is a plain RoboStack node with no NITROS awareness — it can
  only ever publish genuine wire-format `sensor_msgs/Image`. rclcpp's
  `TypeAdapter<NitrosImage, sensor_msgs::msg::Image>::convert_to_custom`
  (`isaac_ros_nitros_type/isaac_ros_nitros_image_type/src/nitros_image.cpp`)
  is what bridges this: it `cudaMallocAsync`s a fresh device buffer and
  `cudaMemcpyAsync`s the host image into it — **on a CUDA stream acquired
  from a separate, global `CudaStreamPool`**, not `RectifyNode`'s own
  dedicated `cuda_stream_` (created in its constructor via
  `createCudaStream("RectifyNode")`).
- By contrast, `RectifyNode`'s own output (`image_rect`) is a **native**
  NITROS-to-NITROS hop into `AprilTagNode` — no `TypeAdapter` conversion,
  no second stream involved. This asymmetry matches the crash pattern
  exactly: the only foreign→NITROS conversion in this graph is the
  `usb_cam → rectify` hop, and that's the one that's timing-sensitive.
- The cross-stream handoff is *supposed* to be safe — `NitrosBuffer`'s
  `WriteHandle`/`ReadHandle` (`isaac_ros_nitros/include/isaac_ros_nitros/types/nitros_buffer.hpp`)
  implement exactly this via CUDA events: a `WriteHandle`'s destructor
  records an event on its stream, and a subsequent `ReadHandle` does
  `cudaStreamWaitEvent` against it before the consumer stream touches the
  data. Somewhere in that handshake — plausibly the exact moment
  `convert_to_custom`'s local `write_handle` destructs (finalizing the
  write event) relative to when `RectifyNode::InputCallback` calls
  `image->get_read_handle(*cuda_stream_)` and then immediately builds an
  `nvcv::Tensor` from the result — there's a window where the tensor gets
  wrapped around a pointer/state that isn't valid yet, which NVCV's own
  validation catches as "the tensor handle is null" the instant
  `remap_op_` tries to use it.

**This is very likely an upstream NVIDIA bug** (in `isaac_ros_nitros_image_type`'s
`TypeAdapter` or the `NitrosBuffer` event-synchronization contract), not
something in code we wrote, and not specific to this from-source build —
it would presumably reproduce on an official NGC-image deployment too, any
time a non-NITROS-aware camera driver feeds a NITROS-consuming node
directly. Not fixed here; `apriltag_camera_launch.py` continues to work
around it by skipping `RectifyNode` entirely. If picking this up again:
a live `gdb` backtrace at the actual crash point (not just "does it crash
under gdb at all") would confirm this precisely — try starting `gdb`
detached from the process group so its own I/O buffering doesn't hide the
node-loading log lines (that delay is what made the first gdb attempt look
hung), or use `rr` (record-and-replay) if available, which is specifically
built for pinning down non-deterministic race conditions like this one
without perturbing timing on replay.

### Follow-up after a reboot: the crash did not reproduce in an extended soak test

`rr` turned out not to be installed, and isn't a solid option here anyway —
it doesn't officially support aarch64. Instead, re-attempted a live `gdb`
backtrace with `catch throw` (rather than `catch throw nvcv::Exception`,
since that type-filtered form wasn't reliably matching) on the **full**
`usb_cam → rectify → apriltag` graph, attached *after* all three nodes had
already loaded and frames were flowing (to minimize how much ptrace
overhead could shift timing, versus attaching from process start as
before). `gdb -p <pid>` needed `sudo` — the host's default
`kernel.yama.ptrace_scope=1` blocks a non-parent process from attaching
even as the same user; root bypasses that check without needing a
persistent `sysctl` change.

The catchpoint never fired. More notably: **after detaching gdb entirely,
the identical graph — the exact one that reproducibly crashed within ~1s
of `apriltag` finishing load in every previous test — ran clean for over
33 minutes**, `/tag_detections` publishing a steady 10 Hz throughout, valid
(non-empty, non-NaN) detections once a tag was back in frame, no memory
growth or thermal issues (`tegrastats`: ~1.8/7.5 GB RAM, ~53°C). This is
the same build, same launch graph, same camera, same physical setup as the
run that crashed reliably pre-reboot — the only variable that changed is
the reboot itself.

**Revised assessment:** this doesn't rule out a genuine race in the
`TypeAdapter`/`NitrosBuffer` cross-stream handoff (the code-level
reasoning above still stands as a plausible mechanism), but the crash is
evidently not purely a function of the code path — it depends on some
piece of runtime state that a reboot resets. Plausible candidates:
GPU/CUDA context fragmentation or a stale `CudaStreamPool` left behind by
one of the many crashed/killed `component_container_mt` processes from the
earlier debugging session (each crash was `nvcv::Exception` — an
unhandled C++ exception during active CUDA work, which is exactly the
kind of abnormal termination that can leave device memory or driver-level
state inconsistent for subsequent processes on the same boot), or a
Jetson clock/power-state artifact from extended debugging. A single clean
reboot plus one long soak isn't enough to fully retract the "genuine race
condition" conclusion — it needs another crash (or another long
crash-free run under harder conditions, e.g. deliberately induced GPU
memory pressure) to be confident either way.

**Practical consequence:** `apriltag_camera_launch.py` now uses the full
`rectify → apriltag` graph again (lens distortion corrected) instead of
skipping `RectifyNode`, given the extended clean run. If the crash
recurs, the previous workaround (drop `RectifyNode`, remap `apriltag`
directly to `usb_cam`'s `image_raw`) is still valid and trivial to
restore — see git history for `apriltag_camera_launch.py` pre-dating this
change. Anyone hitting the crash again should note whether it happened
fresh after a reboot or after other `component_container_mt` processes had
already crashed earlier in the same boot session — that would help
confirm or rule out the stale-CUDA-context hypothesis above.

## Stage 5 — result: SUCCESS, real GPU DNN inference (TensorRT) from source on Orin/JetPack7

Extended the from-source build to `isaac_ros_dnn_inference`
(`isaac_ros_tensor_rt`, `isaac_ros_tensor_proc`, `isaac_ros_dnn_image_encoder`
— skipped `isaac_ros_triton`, which needs the separate Triton Inference
Server and wasn't needed for this). Motivation: `perseus_vision` (this
repo's existing ONNX detector package) only builds in the x86
`machine-learning` Pixi env and isn't built on the Jetson at all — there
was no GPU-accelerated object detection running natively on the Orin. This
closes that gap with a real, working TensorRT path.

**TensorRT itself wasn't installed.** Unlike VPI/CV-CUDA (Stage 4, needed
manual `.deb` downloads from GitHub releases), TensorRT ships directly in
NVIDIA's JetPack-7 L4T apt repo (`repo.download.nvidia.com/jetson/common
r39.2`), already configured on this host: `sudo apt-get install
tensorrt-dev` pulled `libnvinfer-dev`/`libnvonnxparsers-dev`/plugins at
**10.16.2.10-1+cuda13.2** — an exact match for this system's CUDA 13.2, no
version hunting needed.

### New build issues (beyond the Stage 3/4 magic_enum/VPI/CV-CUDA pattern)

- **`CCCL::CCCL` target not found**, even though `isaac_ros_tensor_list_interfaces`
  (built back in Stage 3) exports it as a link dependency. Root cause:
  `isaac_ros_common-extras.cmake` does `find_package(CCCL CONFIG QUIET)`
  and only links it `if(TARGET CCCL::CCCL)` — CCCL's actual CMake package
  lives at `/usr/local/cuda-13.2/targets/sbsa-linux/lib/cmake/cccl/`, which
  isn't on the Pixi env's `CMAKE_PREFIX_PATH`. It was apparently found
  during whatever earlier build first compiled `isaac_ros_tensor_list_interfaces`
  (baking `CCCL::CCCL` into its exported config), but a *fresh* configure
  for a new package doesn't find it, so re-importing that dependency's
  exported target fails. Fixed by prepending
  `/usr/local/cuda-13.2/targets/sbsa-linux` to `CMAKE_PREFIX_PATH` for every
  build in this stage. (Exactly why it was found once but not
  reproducibly since — likely a `PATH`/env difference between an earlier
  interactive `pixi shell` session and later `pixi run -e ... bash -c`
  invocations — wasn't fully pinned down; the explicit `CMAKE_PREFIX_PATH`
  addition sidesteps it either way.)
- **`isaac_ros_tensor_proc` needs `project(... LANGUAGES ... CUDA)`** (it
  compiles real `.cu` kernels, unlike `isaac_ros_tensor_rt` which is pure
  C++ against the TensorRT API) — this requires `CUDACXX` to point at
  `nvcc` (same fix as documented in Stage 0/3: `export
  CUDACXX=/usr/local/cuda/bin/nvcc`) *and*, separately, an explicit
  `-DCMAKE_CUDA_ARCHITECTURES=87` cmake arg. Without it, CMake's own CUDA
  architecture auto-detection (which runs immediately at the `project()`
  call, before `isaac_ros_common-extras.cmake`'s own
  `CMAKE_CUDA_ARCHITECTURES` fallback logic gets a chance to execute) fails
  with "CMAKE_CUDA_ARCHITECTURES must be non-empty if set." `87` is Orin's
  real SM (Ampere) — picking it explicitly here sidesteps the SM75-vs-87
  ambiguity flagged back in Stage 0 entirely, since TensorRT itself builds
  its inference engine at runtime against whatever GPU is present (no
  fixed-architecture kernels to get wrong), and `isaac_ros_tensor_proc`'s
  own handful of `.cu` files just need *a* valid target, not a
  contentious one.
- Same magic_enum/VPI/CV-CUDA `CXXFLAGS`/`LDFLAGS` scoping as Stage 3/4
  applied unchanged.

### Verification: two round trips, both our own minimal scripts (no `isaac_ros_test`/torch)

1. **`tensor_rt_launch.py` + `tensor_rt_check.py`**: loads NVIDIA's own
   `isaac_ros_tensor_rt` test fixture (`mobilenetv2-1.0.onnx`, git-lfs,
   copied out to `models/`), publishes a zero-filled `[1,3,224,224]` input
   tensor directly on `tensor_pub`, and checks the response on `tensor_sub`
   matches mobilenetv2's known output signature (`name=output,
   dims=[1,1000], float32`) — same checks as NVIDIA's own
   `isaac_ros_tensor_rt_test.py`, just without pulling in
   `isaac_ros_test`/`IsaacROSBaseTest`. **First real TensorRT engine build
   on this hardware**: ONNX parse → optimization pass → serialized a
   14.5 MB engine in **41.8 seconds**, then ran inference and returned the
   exact expected tensor shape/dtype/name on the very first published
   message. Confirms TensorRT's builder and runtime both work natively
   against this JetPack7/CUDA13.2/Orin stack.
2. **`dnn_classify_launch.py` + `dnn_classify_check.py`**: the real
   pipeline — `DnnImageEncoderNode` (resize/pad to 224×224, normalize) →
   `TensorRTNode` (mobilenetv2), fed NVIDIA's own
   `isaac_ros_dnn_image_encoder` test image
   (`test_cases/pose_estimation_0/image.jpg`, 1920×1080). Checks the same
   shape/dtype/name properties *plus* that the 1000-way output isn't
   degenerate (finite, non-constant: `std≈3.67`, a real logit spread) —
   catching the case where the shape happens to match but the
   resize/normalize step fed TensorRT garbage. Passed on the first
   publish once two bugs (below) were fixed.

### Two real bugs found while wiring the encoder→tensor_rt chain (both ours, not NVIDIA's — fixed)

- **`enable_padding: False` crashes**: `NVCV_ERROR_INVALID_ARGUMENT:
  INVALID_DATA_SHAPE` — `terminate called after throwing an instance of
  'nvcv::Exception'`, immediately on the first frame. The underlying
  CV-CUDA resize path used when padding is disabled apparently requires
  the input and output to already match some shape constraint (didn't
  dig further into CV-CUDA's own resize op internals); NVIDIA's own
  `DnnImageEncoderNode::declare_parameter` default for `enable_padding`
  is `true` anyway — the crash only happened because the test launch file
  explicitly overrode it to `false`. Fixed by not overriding it.
- **Wrong output topic name → silent no-op, not a crash**: initially
  remapped `('encoded_tensor', 'tensor_pub')`, guessing the topic name
  from the *launch-wrapper* parameter (`tensor_output_topic`, used by
  NVIDIA's own `dnn_image_encoder.launch.py` when composing nodes via
  `LoadComposableNodes`). But `DnnImageEncoderNode`'s C++ constructor
  hardcodes its actual publisher topic as `"tensors"`
  (`create_publisher<NitrosTensorList>("tensors", ...)`) — the launch
  wrapper's parameter name is unrelated to the wire topic name when
  instantiating the `ComposableNode` directly (bypassing that wrapper, as
  our minimal launch files do). Symptom was a hang with **zero errors**:
  the encoder's own debug log showed it receiving images and building
  `NitrosTensorList` objects correctly (`[DnnImageEncoderNode]: [Dnn Image
  Encoder] Image received`, followed by a full `NitrosBuffer`
  `WriteHandle`/`ReadHandle`/CUDA-event sequence), but `tensor_rt` never
  saw anything on `tensor_pub` because the encoder was actually publishing
  on `/tensors`, not `/tensor_pub`. Found by bumping the container to
  `--log-level debug` and grepping for `Initializing publisher for topic
  name` — confirmed the real topic name directly rather than trusting the
  launch-wrapper parameter's naming. Fixed by remapping `('tensors',
  'tensor_pub')` instead.

### Stage 5 follow-up — `isaac_ros_yolov8`: perception-level proof-of-life, done

Extended Stage 5 with `isaac_ros_yolov8` (from the separate
`isaac_ros_object_detection` repo, alongside `isaac_ros_detectnet`,
`isaac_ros_rtdetr`, `isaac_ros_grounding_dino` — only `isaac_ros_yolov8`
cloned/built), the object-detector-level proof analogous to Stage 4's
AprilTag. `YoloV8DecoderNode` is pure C++ (no CUDA kernels of its own) —
built clean on the first try with the same toolchain/flags as the rest of
Stage 5, no new CMake issues, once one missing RoboStack package was
added: `vision_msgs` (for `Detection2DArray`) isn't in the default Jazzy
RoboStack channel, added as `ros-jazzy-vision-msgs` to
`[feature.isaac-nitros.dependencies]`.

**Verification** (`yolov8_launch.py` + `yolov8_check.py`, wired as
`pixi run -e isaac-nitros test-yolov8`): the full `image →
dnn_image_encoder → tensor_rt → yolov8_decoder` chain, using NVIDIA's own
`isaac_ros_yolov8` test fixtures — `dummy_yolov8s.onnx` (real yolov8s
architecture/IO names, but **randomly initialized weights**, same fixture
NVIDIA's own `isaac_ros_yolov8_decoder_node_pol.py` uses) and
`test_cases/single_detection/people_cycles.jpg` (640×640, exactly the
network's input size). TensorRT engine build for this model took **5.9
seconds** (smaller/simpler graph than mobilenetv2's 41.8s). Result: **10
detections** came back through NMS/box decoding on the very first
publish — full chain confirmed working end to end.

Because the model's weights are random, detection *content* is
meaningless by design — NVIDIA's own POL test explicitly only checks that
a message arrives, not the values in it ("the data is not verified
because the model is initialized with random weights"). First pass of
`yolov8_check.py` over-validated this (asserted `score ∈ [0,1]` and
positive bbox dimensions) and failed on scores like `1.222` and boxes like
`-1.0×-1.0` — genuinely produced by an untrained network's raw regression
output, not a bug in the chain. Relaxed the check to match NVIDIA's own
scope: a structurally valid `Detection2DArray` (any content) is success.

**Not yet done**: swapping in a real trained YOLOv8 model (from Ultralytics
or NGC) for actually-meaningful detections — this stage stopped at
proving the plumbing, same scope boundary as the mobilenetv2 check. Also
not done: wiring any of Stage 5 into `perseus_isaac_relay` or the live
camera.

## Stage 6 — `isaac_ros_visual_slam` (cuVSLAM): result: BLOCKED, published aarch64_jetpack70 binary needs SVE2, Orin's CPU doesn't have it

Attempted to extend the from-source build to GPU-accelerated visual SLAM
(cuVSLAM), as a complement to the robot's existing lidar-based
`slam_toolbox`. Unlike GXF core (Stage 0/1), this looked like the
promising case going in: `isaac_ros_nitros/isaac_ros_nitros/lib/cuvslam/`
ships a **genuine native `lib_aarch64_jetpack70/libcuvslam.so`** — real
ELF ARM64 binary, not an LFS pointer — and `isaac_ros_nitros`'s own
`CMakeLists.txt` selects it purely by `CMAKE_SYSTEM_PROCESSOR MATCHES
"aarch64"`, no sbsa-vs-jetson fallback logic the way GXF core needed. The
wrapper package (`isaac_ros_visual_slam`, cloned alongside
`isaac_ros_visual_slam_interfaces`) is genuine Apache-2.0 C++ and built
clean — needed `isaac_common` and `isaac_ros_launch_utils` built first
(both already-cloned parts of `isaac_ros_common`, same "colcon needs
transitive deps built even for exec_depend" pattern as before) plus
`-Wno-dev`-worthy `CMP0167`/Boost warnings (harmless).

**The crash**: loading the composable node (`ros2 launch` →
`component_container_mt` → `dlopen(libvisual_slam_node.so)` → transitively
`dlopen(libcuvslam.so)`) died instantly with `SIGILL` (exit code -4),
before the node class was even instantiated — a `component_container_mt`
log line reading "Load Library" then immediate death, no ROS error, no
GXF/NITROS involvement at all. Isolated with a two-line reproduction,
decoupled entirely from ROS:

```console
$ python3 -c "import ctypes; ctypes.CDLL('.../lib_aarch64_jetpack70/libcuvslam.so')"
Illegal instruction (core dumped)
```

`gdb -batch -ex run -ex bt -ex 'x/4i $pc' --args python3 -c "..."` caught
it in `libcuvslam.so`'s own ELF constructor (`call_init` → `_dl_init`,
i.e. a global/static initializer running before any application code),
crashing on:

```
=> 0xfffff6d44544:  whilewr p0.s, x6, x3
```

**`whilewr` is an SVE2 (Scalable Vector Extension 2) instruction.**
`/proc/cpuinfo`'s `Features` line on this Orin (`CPU part: 0xd42` =
Cortex-A78AE) lists `asimd` (NEON) but **no `sve`/`sve2` at all** — this
CPU cannot execute that instruction, full stop; it's not a driver/runtime
issue, it's the physical core lacking the ISA extension the binary was
compiled to require. Confirmed this is specific to the `jetpack70` build,
not a general cuVSLAM property: `objdump -d` on the sibling
`lib_aarch64_jetpack61/libcuvslam.so` (NVIDIA's officially-supported
JetPack 6.1/Orin target) contains **zero** SVE/SVE2 instructions
(`whilewr`/`whilerw`/`ptrue`/`ld1w`/`st1w` all absent) — clean NEON-only
code. The `jetpack70` binary looks like it was built with SVE2 codegen
enabled for a newer ARM core (plausibly targeting Thor/Blackwell-class
aarch64 silicon, which the repo's own `CUAPRILTAGS_LIB_PATH`/
`CUMOTION_PATH` selection comments elsewhere hint may share the generic
"aarch64" bucket with actual Orin targets) without accounting for Orin's
older, non-SVE Cortex-A78AE cores — a genuine upstream packaging mismatch,
not something fixable from our side (cuVSLAM's own source isn't
published, only this compiled `.so`).

**Is `lib_aarch64_jetpack61` a viable fallback?** Checked and ruled out:
`ldd` on it reports `libcusolver.so.11 => not found` and `libcublas.so.12
=> not found` — it wants CUDA 11/12-era sonames this host's CUDA-13.2-only
install doesn't provide. So neither published aarch64 cuVSLAM variant
works on this exact stack: `jetpack70` has the right CUDA ABI but the
wrong CPU ISA; `jetpack61` has (plausibly) the right CPU ISA but the wrong
CUDA ABI. Genuinely blocked both ways without either a corrected NVIDIA
binary or building cuVSLAM from source (not available to us).

**Also worth noting independent of the crash**: even if cuVSLAM loaded,
it has no monocular-only tracking mode (`tracking_mode` is
Multicamera/stereo, VIO/stereo+IMU, or RGBD only — see
`visual_slam_node.cpp`), and this robot's only camera is the monocular
Logitech C920. The proof-of-life launch/check pair written for this stage
(`visual_slam_launch.py`/`visual_slam_check.py`, RGBD mode) plays back
NVIDIA's own bundled test fixture — a real Intel RealSense 455 recording
(`test_cases/rosbags/rgbd_static/rosbag2_rs455_rgbd.mcap`, 6s/92 frames) —
rather than anything from this robot's hardware, since there's no
depth/stereo source to feed it regardless of the SVE2 blocker. Left in
place, currently non-functional pending the binary issue above; would
also need a depth or stereo camera added to the robot to ever be useful
here even if unblocked.

**Not fixed. Recommend reporting the SVE2/ISA mismatch to NVIDIA** (e.g.
via the Isaac ROS GitHub issues or forum) — this reads like a genuine
build-config bug in their `aarch64_jetpack70` cuVSLAM release, not
something specific to this from-source experiment; it would presumably
reproduce for anyone on real Orin hardware pulling this artifact.

## Stage 7 — `isaac_ros_unet`: result: SUCCESS, semantic segmentation from source on Orin/JetPack7

Extended the DNN inference infrastructure built in Stage 5 (TensorRT +
`isaac_ros_dnn_image_encoder`) to semantic segmentation, cloning
`isaac_ros_image_segmentation` and building two packages:
`isaac_ros_unet_kernels` (CUDA `.cu.cpp` postprocessing/colorization
kernels — same `CUDACXX`/`-DCMAKE_CUDA_ARCHITECTURES=87` build recipe as
Stage 5's `isaac_ros_tensor_proc`) and `isaac_ros_unet` (the
`UNetDecoderNode`, pure C++). Chosen over the remaining `isaac_ros_image_proc`
utility nodes as the next capability because it's directly relevant to
this robot's sandy-terrain autonomy mission (traversability masks) and
reuses already-proven infra rather than opening a new dependency
surface — unlike Stage 6, which needed a closed-source binary.

**Build fix**: `isaac_ros_unet`'s `package.xml` lists
`<exec_depend>isaac_ros_triton</exec_depend>` (an alternate inference
backend we don't use — everything in this experiment goes through
`isaac_ros_tensor_rt`, same as Stage 5/5-follow-up). Since `isaac_ros_triton`
isn't cloned into the workspace, `colcon build` failed with "Failed to
find package.sh for isaac_ros_triton" (same class of issue as Stage 6's
`isaac_ros_launch_utils` transitive-dependency gap). Fixed by removing
that one `exec_depend` line from the local (gitignored) checkout — Triton
was never going to be built here regardless.

Both packages built clean (`isaac_ros_unet_kernels` in 13.3s,
`isaac_ros_unet` in 54.8s). Wrote `unet_launch.py`/`unet_check.py`
following the same shape as `dnn_classify_launch.py`/`yolov8_launch.py`:
grepped `DnnImageEncoderNode`'s hardcoded topic names again (`image`/
`camera_info` in, `tensors` out — confirmed once more, no repeat of the
Stage 5 topic-name bug) and `UNetDecoderNode`'s (`tensor_sub` in,
`unet/raw_segmentation_mask` + `unet/colored_segmentation_mask` out) —
`TensorRTNode`'s own hardcoded `tensor_pub`/`tensor_sub` already line up
with both ends, so only the encoder→tensor_rt link needs a remap.

Test used NVIDIA's own `model.dummy.onnx` fixture from
`isaac_ros_unet/test/dummy_model/` (124 MB, random weights, 20-class
output) and its companion `test_cases/unet_sample/image.jpg` (1200×632),
with the exact input/output binding names and 960×544 network resolution
from NVIDIA's own `isaac_ros_unet_pol_test.py` (not chosen by us). The
larger model took noticeably longer to build a TensorRT engine than
Stage 5's classifiers — 182s vs. mobilenetv2's 41.8s or yolov8's 5.9s —
so the check script's engine-ready timeout was raised from the initial
60s (which timed out on the first run) to 300s.

Result: **`UNET OK`** — both `unet/raw_segmentation_mask` (960×544,
`mono8`) and `unet/colored_segmentation_mask` (960×544, `rgb8`) arrived
with correct shapes/encodings; the raw mask contained 15 distinct class
IDs (structurally valid, not semantically meaningful given random
weights — same scope caveat as Stage 5 follow-up's YOLOv8 detections).

**Not yet done**: a real trained segmentation model for meaningful
terrain/traversability masks (same "proved the plumbing, not the
weights" scope as Stage 5/5-follow-up); wiring into the live camera or
`perseus_isaac_relay`.

## Stage 8 — `isaac_ros_centerpose`: result: SUCCESS, monocular 3D pose estimation with a real trained model

Extended the DNN inference chain to 3D object pose estimation, cloning
`isaac_ros_pose_estimation` and building `isaac_ros_centerpose`
(`CenterPoseDecoderNode` — pure C++, no CUDA of its own) plus two
dependencies not yet built in this experiment: `gxf_isaac_messages` (a
GXF extension already present locally under `isaac_ros_nitros/isaac_ros_gxf_extensions/`,
just never symlinked into the workspace before) and
`isaac_ros_nitros_detection3_d_array_type` (likewise already present
under `isaac_ros_nitros/isaac_ros_nitros_type/`). Chosen over the other
untried GEMs surfaced by the full Isaac ROS map (see below) because it's
monocular-compatible — unlike Stage 6's cuVSLAM — and directly useful to
a robot with a manipulator arm (knowing an object's 3D pose is a
prerequisite for grasping it).

**Build fix (same pattern as Stage 7)**: `isaac_ros_centerpose`'s
`package.xml` has a hard `<depend>isaac_ros_triton</depend>` (not even
`exec_depend` this time) for an inference backend we don't use.
`isaac_ros_centerpose`'s own `CMakeLists.txt` only references Triton in
an optional test file (`test_centerpose_pol_triton.py`, gated behind
`BUILD_TESTING`), so removing the `<depend>` line from the local
checkout is safe — colcon's dependency resolution runs off `package.xml`
independent of `BUILD_TESTING`, so leaving it in fails the build even
though nothing in the actual compiled code needs it.

Wrote `centerpose_launch.py`/`centerpose_check.py` following the same
shape as Stage 5 onward, with one new wrinkle: `TensorRTNode` here has
**seven** named outputs (`bboxes`, `scores`, `kps`, `clses`, `obj_scale`,
`kps_displacement_mean`, `kps_heatmap_mean`) instead of one, since
CenterPose's architecture predicts bounding box, keypoints, and object
scale simultaneously — all copied verbatim from NVIDIA's own
`test_centerpose_pol.py`, not derived by us.

**What's different from every DNN stage before this one**:
`centerpose_shoe.onnx` is a **real trained model** (PyTorch→ONNX,
opset 16), not random weights — NVIDIA ships a matching
`ground_truth.json` alongside its test image (two real shoes, with
measured 3D location/quaternion/scale per object). This is the first
check in the series that validates actual output content instead of
just structural validity: `centerpose_check.py` compares detected
object count and depth (Z) against the ground-truth values with a 1.0 m
tolerance.

Result: **`CENTERPOSE OK`** — 2 detections against a ground truth of 2,
depths `[4.11 m, 5.02 m]` against ground-truth `[4.40 m, 5.50 m]`, both
comfortably inside tolerance. The larger 7-output engine took longer to
compile than prior stages (raised `centerpose_check.py`'s engine-ready
timeout to 300 s after an initial 120 s run timed out mid-build, same
"bigger model, longer first-run engine build" pattern as Stage 7).

This is the strongest proof-of-life in the whole from-source experiment
so far: not just "the NITROS/TensorRT/CUDA plumbing runs without
crashing" but "a real trained model produces numerically correct
real-world 3D measurements" — end to end, natively, on Orin/JetPack7.

**Full Isaac ROS map, for context on what's next**: a separate pass
enumerated all ~29 Isaac ROS GEM repositories (via the GitHub API, not
from memory) against this experiment's progress. Five repos are now
verified working from source (AprilTag, DNN inference, image_pipeline,
object detection, image segmentation) plus this one makes six; one is
built but blocked at runtime (visual SLAM, Stage 6); four need a
depth/stereo camera this robot doesn't have; seven are architecturally
inapplicable (Nova-platform hardware, CSI camera drivers, ROS1 bridge,
Unitree G1-specific packages); the remaining dozen (pose estimation
beyond CenterPose, mapping/localization, cuMotion, manipulation,
compression, jetson-stats, teleop, etc.) are relevant but untried.

**Not yet done**: wiring into the live camera or `perseus_isaac_relay`;
a robot-relevant trained model (a real object this robot's arm might
need to pick up, rather than NVIDIA's demo shoe).

## Stage 9 — `isaac_ros_compression`: result: BLOCKED, this Orin Nano SKU has no hardware video encoder, and hardware decode fails downstream of NVDEC itself

Attempted hardware H.264 encode/decode next, from the "relevant, not yet
attempted" list surfaced by the Stage 8 map — useful for streaming video
off-robot without burning CPU/GPU compute cycles the perception stack
needs. Unlike Stages 5–8, this doesn't touch TensorRT/CUDA inference at
all: `isaac_ros_h264_encoder`/`isaac_ros_h264_decoder` wrap Orin's
dedicated hardware video codec block via the V4L2 M2M kernel driver
(vendored in-repo as `codec/libv4l2`). Both packages built clean —
no `isaac_ros_triton`-style dependency issue this time, and no CUDA
architecture flags needed since there's no `.cu` code in this path.

**Encoder: genuinely blocked, no fix possible.** `EncoderNode` failed
every attempt with `[V4L2Encoder] Failed to open encoder device`.
Traced to `encoder_v4l2_impl.cpp` hardcoding `/dev/v4l2-nvenc` — and
`ls /dev` on this hardware shows `v4l2-nvdec` but no `v4l2-nvenc`
counterpart at all. This Jetson is a **Jetson Orin Nano Engineering
Reference Developer Kit Super** (confirmed via
`/proc/device-tree/model`); the Orin **Nano** SKU (unlike Orin NX/AGX)
has no NVENC hardware block in silico — this is a real hardware
omission on this specific module, not a missing driver, permission, or
JetPack7 support gap. No amount of build or config work on our side can
open a device node that doesn't exist.

**Decoder: gets further, but also blocked, one layer deeper.**
`DecoderNode` (hardware NVDEC, which this SoC does have) initializes
cleanly against NVIDIA's own `compressed.h264` test fixture (a
single-keyframe clip from `isaac_ros_h264_decoder`'s own test suite) and
successfully parses the bitstream far enough to detect resolution:

```
[V4L2Decoder]: Got event type=5 (RESOLUTION_CHANGE=5)
[V4L2Decoder]: Decoded video: 460x460
[V4L2Decoder]: NvBufSurfaceMapCudaBuffer for destination buffer failed
NvBufSurfaceMapCudaBufferImpl: API is not supported on this platform
```

The V4L2 M2M decode itself works; the subsequent step — mapping the
decoded surface into a CUDA buffer, needed for the ROS node to hand the
frame off as a `NitrosImage` — fails inside NVIDIA's proprietary
`libnvbufsurface`, a closed-source library with no available source to
patch. Ruled out a test-harness bug first: the fixture is a single
keyframe (3 NAL units: SPS/PPS/IDR, confirmed by scanning for Annex-B
start codes), and re-publishing it in a tight loop (as every other check
script in this series does) could plausibly desync a stateful decode
thread — retried with real 1-second gaps between publishes and got the
identical failure on the very first attempt, ruling that out. This
reads as a genuine platform/BSP gap in the CUDA-interop path for decoded
buffers on this unsupported Orin-Nano+JetPack7 pairing, not a bug in the
Isaac ROS wrapper code itself.

Removed the originally-planned encoder→decoder round-trip test (it can
never pass — no encoder engine exists to feed the decoder from), and
replaced it with a decode-only proof (`h264_decode_launch.py`/
`h264_decode_check.py`) that documents the `NvBufSurfaceMapCudaBuffer`
failure directly rather than silently skipping the stage.

**Not fixed — genuinely blocked on both halves**, for different
reasons: encoder by absent hardware, decoder by an unsupported
CUDA-interop path in a closed-source NVIDIA library. Unlike Stage 6
(cuVSLAM), there's no fallback binary variant to try here — this is the
actual hardware in front of us. Full detail logged in `ERRORS.md`.

## Stage 10 — combined pipeline: result: SUCCESS, four capabilities running together for the first time

Every capability from Stages 4, 5-follow-up, 7, and 8 had only ever been
proven running **alone** in its own `component_container_mt`. With two
stages in a row blocked on hardware limits rather than software gaps,
extending to more untried GEMs looked like diminishing returns — the
more useful open question was whether anything already built could
actually coexist: four TensorRT engines (or VPI/cuAprilTag plus three
TensorRT chains) sharing one GPU, one CUDA context, one process, without
resource contention or topic collisions. That's a real prerequisite
before any of this could plausibly run together on the robot, and it had
never been tested.

`combined_pipeline_launch.py` loads AprilTag, YOLOv8 (encoder + tensor_rt
+ decoder), U-Net (encoder + tensor_rt + decoder), and CenterPose
(encoder + tensor_rt + decoder) — ten composable nodes total — into one
container, each pipeline in its own ROS namespace (`/apriltag`,
`/yolov8`, `/unet`, `/centerpose`). This matters specifically because of
the Stage 5 topic-name bug documented earlier in this file: every
encoder publishes to the literal hardcoded topic `"tensors"`, every
`TensorRTNode` subscribes to `"tensor_pub"` and publishes `"tensor_sub"`
— relative names that would collide directly if three encoder/tensor_rt
pairs ran unnamespaced in the same container (each `TensorRTNode` would
receive whichever pipeline's tensor arrived last, silently producing
wrong results, not a crash). ROS namespace resolution prepends each
node's namespace to these relative names, isolating them.
`combined_pipeline_check.py` publishes each pipeline's own Stage
4/5-follow-up/7/8 test fixture into its namespace concurrently and waits
for all four outputs.

**First attempt found a real bug** (in our test harness, not in Isaac
ROS): the check script's `simple_camera_info()` helper left `K` as all
zeros. That's harmless for YOLOv8/U-Net (no calibration-dependent code
in that path), but `CenterPoseDecoderNode` performs a PnP solve using
`K` to recover 3D pose — an all-zero (degenerate) camera matrix crashed
the **entire container**, not just the CenterPose node:

```
terminate called after throwing an instance of 'cv::Exception'
what():  OpenCV(4.12.0) .../calibration_base.cpp:1384: error: (-215:Assertion
  failed) fabs(sc) > DBL_EPSILON in function 'findExtrinsicCameraParams2'
process has died [pid ..., exit code -6, ...]
```

This is worth flagging as a real operational risk independent of this
experiment: an uncaught C++ exception in one composable node running
inside a shared multi-node container brings down every other node in
that container, including ones that had nothing to do with the fault
(AprilTag and the just-loaded U-Net/YOLOv8 pipelines all died with it,
mid-run, having previously produced valid output). A production
deployment combining multiple Isaac ROS nodes in one container should
budget for this — either validate all inputs before they reach nodes
with unguarded assertions, or accept that one bad `CameraInfo` message
can take down an entire perception stack, not just the node it was
meant for.

Fixed by supplying the real intrinsics (same K matrix as Stage 8) for
CenterPose's `CameraInfo`, matching every prior stage's practice of
using real/matching calibration data rather than placeholders.

Result: **`COMBINED OK`** — all four fired concurrently: AprilTag 1
detection, YOLOv8 10 detections, U-Net 960×544 mask, CenterPose 2
detections (matching Stage 8's ground-truth count exactly, run inside a
shared container this time). No resource contention, no topic
cross-talk, no GPU memory exhaustion observed across four simultaneous
TensorRT/VPI workloads on this Orin Nano.

This is the first result in the series that says something about the
whole system rather than one capability in isolation — a meaningful
step toward "could this run on the robot," even without a reconnected
live camera to close that last gap.

## Stage 11 — `isaac_ros_dnn_stereo_depth` (ESS): result: PARTIAL — build succeeds (real CUDA kernels included), runtime blocked on Tegra CMA exhaustion

Extended into genuinely new CUDA territory: deep stereo disparity
estimation (the ESS network) rather than another classify/detect/segment
TensorRT chain. Unlike Stage 5-8's single encoder→tensor_rt→decoder
shape, this is a 15-composable-node graph straight-ported from NVIDIA's
own `isaac_ros_ess_test.py`: two parallel per-side chains (format
convert → resize → normalize → to-tensor → planar → reshape, one per
stereo camera) synced by a `TensorPairSyncNode`, feeding one two-input/
two-output `TensorRTNode`, decoded by `DNNStereoDecoderNode` — which,
notably, has its own real `.cu.cpp` CUDA kernel
(`filter_disparity.cu.cpp`, confidence-thresholding the raw disparity
map), unlike Stage 5's `TensorRTNode` which is pure TensorRT-API C++
with no CUDA kernels of its own.

**Build: fully successful, no new fix classes needed.** Four new
packages built clean: `isaac_ros_nitros_disparity_image_type`,
`gxf_isaac_ros_messages`, `isaac_ros_nitros_point_cloud_type`,
`isaac_ros_stereo_image_proc`, and `isaac_ros_dnn_stereo_decoder` (the
one with the CUDA kernel) — all already-known dependency-resolution
patterns from earlier stages (packages present in already-cloned repos,
just not yet symlinked into the workspace). The six per-side image
processing node types (`ImageFormatConverterNode`, `ResizeNode`,
`ImageNormalizeNode`, `ImageToTensorNode`, `InterleavedToPlanarNode`,
`ReshapeNode`) all already existed in `isaac_ros_image_proc`/
`isaac_ros_tensor_proc`, built back in Stage 4/5 — this stage just
exercises plugin classes within those libraries we hadn't used yet.

**Runtime: blocked, but on a real, diagnosed platform resource limit,
not a build or code defect.** The container crashed on the very first
published frame:

```
NvMapMemAllocInternalTagged failed: error 12
Failed ioctlCmd: 1075858947
NvMapMemHandleAlloc failed: error 12
terminate called after throwing an instance of 'std::runtime_error'
what():  Failed to create CUDA memory pool, cuda_error: cudaErrorMemoryAllocation,
  error_str: out of memory
```

`free -h`/`tegrastats` showed 5.4GB of general system RAM free at the
time — this is not ordinary memory pressure. `NvMapMemAllocInternalTagged`
allocates from Tegra's **CMA (Contiguous Memory Allocator) carveout**, a
small physically-contiguous region reserved at boot
(`cat /proc/meminfo | grep -i cma` → `CmaTotal: 262144 kB`, 256MB total),
completely separate from the general RAM pool `free`/`tegrastats` report.
`CmaFree` measured only ~34MB even at rest with no test running — the
desktop GUI/display compositor already holds the bulk of the 256MB
carveout on this dev-kit configuration. A 15-node pipeline with six
VPI/`NvBufSurface`-backed image-processing nodes (each drawing from this
same small pool) exceeded the ~34MB of remaining headroom on the very
first real frame.

Considered and rejected shrinking the test's input resolution as a
workaround: the memory-heavy intermediate buffers are fixed at the ESS
model's 960×576 output resolution regardless of input image size (only
the pre-resize raw-image buffers would shrink), so this likely wouldn't
meaningfully reduce the structural pressure of many simultaneous
NvBufSurface-backed nodes.

**Not fixed — the two real remediations both need explicit user
sign-off, not autonomous action**: (1) increase the boot-time `cma=`
carveout size in `/boot/extlinux/extlinux.conf` (a persistent system
config change requiring a reboot), or (2) free up the existing carveout
by stopping the desktop GUI/compositor (disruptive to the active
session). Full diagnosis logged in `ERRORS.md`. Unlike Stage 6 (upstream
binary defect) and Stage 9 (absent hardware), this one is plausibly
fixable — it just needs a decision about system configuration that isn't
this experiment's call to make alone.

## Stage 12 — `isaac_ros_occupancy_grid_localizer`: result: SUCCESS, first capability matched to the robot's lidar rather than its camera

Every capability through Stage 11 targeted the robot's camera. This robot
also carries a 2D lidar, already used by the existing (non-Isaac-ROS)
`slam_toolbox`-based SLAM stack — so the natural next question was
whether anything in Isaac ROS complements that sensor. Surveyed
`isaac_ros_mapping_and_localization`'s three sub-packages first:
`isaac_ros_visual_global_localization` (camera-based) `exec_depend`s on
`isaac_ros_visual_slam`, i.e. Stage 6's blocked cuVSLAM — transitively
blocked. `isaac_mapping_ros` depends on `nvblox_ros`, which needs a
depth camera this robot doesn't have. `isaac_ros_occupancy_grid_localizer`
(+ its `isaac_ros_pointcloud_utils` dependency) was the one clean fit:
it consumes `NitrosFlatScan` (a NITROS 2D-LaserScan-equivalent type) and
an occupancy grid map, with no cuVSLAM, no depth camera, no video codec
hardware anywhere in its dependency chain.

Conceptually this doesn't duplicate `slam_toolbox`: `slam_toolbox`
*builds* the map (SLAM); this node does *relocalization* — finding the
robot's pose within an already-built map, conceptually similar to
`amcl`/Nav2's localization stack but GPU-accelerated grid-search instead
of a particle filter. It also introduces genuinely new CUDA code in this
experiment: `occupancy_grid_localizer_gpu.cu` does GPU-parallel batch
scan-matching (scoring many candidate poses against the map
concurrently), a different computational shape than every prior
TensorRT-inference-based stage.

**Build**: clean, no dependency-resolution surprises. Four packages —
`isaac_ros_pointcloud_interfaces` (already present from an earlier
clone, just not yet symlinked/built), `isaac_ros_nitros_flat_scan_type`,
`isaac_ros_pointcloud_utils`, `isaac_ros_occupancy_grid_localizer` (the
one with the `.cu` kernel) — all built without any of the
`isaac_ros_triton`-style dependency edits earlier stages needed.

**One real bug in our own launch file, not Isaac ROS**: the node's map
image path resolves as `dirname(map_yaml_path) + "/" + image_param`,
where `image_param` is a ROS parameter the node expects to be populated
*separately* from `map_yaml_path` — normally satisfied by ROS 2 launch's
special handling of a bare `.yaml` string in a `parameters=[...]` list
(it's loaded as a **parameters file**, not just a value; nav2-format
map.yaml files happen to use the same top-level keys — `image`,
`resolution`, `origin`, ... — that this node declares as parameters).
Our first launch attempt only passed `map_yaml_path` as a dict value,
never as a parameters-file list entry, so `image` stayed empty and the
node failed with `Could not load occupancy grid map image:
.../maps/` (path truncated, no filename). Fixed by adding the
`map.yaml` path as its own list entry in `parameters=[...]`, matching
NVIDIA's own `test_occupancy_grid_localizer_pol_test.py` exactly, which
does the same thing.

**Real test data, real result**: NVIDIA ships an actual occupancy grid
map (`maps/map.yaml` + `map.png`) and a real recorded rosbag
(`data/rosbags/flatscan`, 12 genuine `FlatScan` messages from
2023-02-25) with their own POL test — not dummy/random fixtures like
several earlier stages. `ogl_check.py` plays the bag, calls the
`trigger_grid_search_localization` service, and checks the resulting
pose against NVIDIA's own recorded ground truth. Also confirmed the
node's TF fallback behavior directly: with no TF data in the bag and
none published by our launch, `LookupBaseLinkToLidarTransform` logs
`Could not transform base_link to lidar_frame: ... does not exist. Using
identity transform.` and proceeds correctly rather than failing — matches
NVIDIA's own test setup (no TF publisher there either).

Result: **`OGL OK`** — position `(33.60, 7.70, 0.00)` against a ground
truth of `(33.5, 7.75, 0.0)` (0.15 m tolerance), orientation quaternion
`(0, 0, -0.5628, 0.8266)` against `(0, 0, -0.56573, 0.824589)` (0.013
tolerance) — both comfortably inside NVIDIA's own test tolerances. The
GPU scan-matching kernel produces a numerically correct pose from real
lidar data on this from-source Orin/JetPack7 build.

**Not yet done**: wiring against the live robot's actual lidar and a map
of its actual environment (this used NVIDIA's own map/scan fixtures, not
anything from this robot); comparing accuracy/performance against the
existing Nav2/`amcl` localization approach if one is in use.
