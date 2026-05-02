# Error Log

### Ghost wall detection used meter-scale thresholds in pixel-space — 2026-02-20

- **Severity:** Medium
- **Category:** Logic
- **File(s):** `software/ros_ws/src/mapping_autotune/mapping_autotune/map_analyzer.py`
- **Pattern:** Using `resolution` (meters/cell) as a multiplier for thresholds compared against pixel-coordinate distances from OpenCV functions like `HoughLinesP`.
- **Root cause:** `_ghost_wall_score` computed `min_dist = 2.0 * resolution` and `max_dist = 8.0 * resolution`. With resolution=0.05m, this gave thresholds of 0.1 and 0.4 pixels — far too small to detect ghost walls 3 cells apart. HoughLinesP returns coordinates in pixel (cell) units, not meters.
- **Fix applied:** Changed thresholds to direct cell counts: `min_dist = 2.0` and `max_dist = 10.0`.
- **Prevention rule:** When working with OpenCV geometry functions (HoughLinesP, distanceTransform, etc.), always verify whether output coordinates are in pixel or world units before applying distance thresholds.

### Run allocator could not trim single-run phases — 2026-02-20

- **Severity:** Low
- **Category:** Logic
- **File(s):** `software/ros_ws/src/mapping_autotune/mapping_autotune/param_manager.py`
- **Pattern:** Budget-trimming loop that floors allocations at 1 when the total budget is less than the number of active phases.
- **Root cause:** `allocate_runs(5)` gave every non-zero-weight phase `max(1, ...)` = 1 run each (6 phases = 6 runs). The trimming loop used `can_trim = max(0, allocation - 1)` which was always 0, making it impossible to reduce the total below 6.
- **Fix applied:** Changed trimming to allow reduction to 0: `trim = min(excess, allocation[phase_num])`.
- **Prevention rule:** When implementing budget allocation with a trim-down pass, ensure the trim floor matches the actual minimum (0 if phases can be skipped, not 1).

### Stale generated headers in install/perseus_interfaces after .msg/.srv edits — 2026-04-26

- **Severity:** Medium
- **Category:** Build
- **File(s):** `software/ros_ws/install/perseus_interfaces/include/perseus_interfaces/msg/object_detections.hpp` (and `srv/detect_objects.hpp`)
- **Pattern:** Editing a `.msg` / `.srv` file in `perseus_interfaces` (e.g. adding `regions_of_interest`, `message`) without rebuilding the package. The `.msg` source is copied to `install/.../share/...` correctly, but the C++ headers under `install/.../include/...` are only regenerated when the _package itself_ is rebuilt — so consumers (`aruco_detector`, `cube_detector`, etc.) compile against the old field set and fail with `'ObjectDetections' has no member named 'regions_of_interest'`.
- **Root cause:** colcon's incremental build does not rerun rosidl codegen for downstream consumers when only their .cpp source changes; the generator runs as part of the interface package's own build. Additionally, an even older copy lives in the Nix store (`/nix/store/.../perseus_interfaces`) and `AMENT_PREFIX_PATH` may resolve to it if the local install isn't sourced first.
- **Fix applied:** `rm -rf build/perseus_interfaces install/perseus_interfaces && colcon build --packages-select perseus_interfaces --symlink-install`, then `source install/setup.bash` _before_ building any consumer.
- **Prevention rule:** Whenever you edit a `.msg`, `.srv`, or `.action` file, force-rebuild the interface package (`rm -rf build/<pkg> install/<pkg>` then `colcon build --packages-select <pkg>`) before building any consumer. If the consumer build still reports missing fields, check that `AMENT_PREFIX_PATH` puts the local `install/` ahead of any Nix-store fallback.
