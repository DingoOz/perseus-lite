---
name: triage-gui-segfault
description: Diagnose a Pixi/RoboStack GUI or rendering tool (gz sim, rviz2, any Qt/OpenGL ROS tool) that segfaults with little or no output. Use when a GUI ROS tool crashes silently in the simulation env, exits with 139, or dies seconds into startup deep in a system library.
---

# Triaging a silent GUI segfault (gz sim / rviz2)

In this repo, GUI/render crashes with no useful traceback are almost always
**environment contamination or a shared version-sensitive cache**, not a code
bug. Pixi/conda activation is *additive*, not isolating, for vars and cache dirs
it doesn't own. Check these before touching any code. (Full write-ups: ERRORS.md
2026-07-03 and 2026-07-05.)

## First: is a foreign ROS/toolchain leaking in?

If `~/.bashrc` sources a native `/opt/ros/jazzy/setup.bash` (or another colcon
workspace), its paths leak into the Pixi env. Dump the env inside the activated
shell and look for foreign paths:

```bash
env | grep -iE 'GZ_CONFIG_PATH|LD_LIBRARY_PATH|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH'
```

- **`gz sim` segfault (exit 139), zero log output** → `GZ_CONFIG_PATH` points at
  `/opt/ros/...` so gz loads plugins built for a different gz version (ABI
  mismatch). Fixed by setting `GZ_CONFIG_PATH=$CONDA_PREFIX/share/gz` in the
  launching `ExecuteProcess`'s `additional_env` (already done in
  `gazebo.launch.py`). `software/scripts/pixi-env-sanitize.sh` strips known
  foreign entries on activation, but a contaminated *parent shell* still leaks —
  guard the offending `~/.bashrc` source line to skip inside Pixi/conda shells.

## Second: a shared, version-sensitive cache under $HOME?

- **Crash inside `libfontconfig` during Qt text shaping** → the system
  fontconfig and the Pixi env's newer fontconfig share `~/.cache/fontconfig`;
  reading a cache written by the other version corrupts an `FcCharSet`
  traversal. Fixed by pointing `XDG_CACHE_HOME` at `$CONDA_PREFIX/var/cache`
  (env-private) in the GUI `ExecuteProcess`'s `additional_env` (done in
  `gazebo.launch.py`, `perseus_sim.launch.py`, `mapping_using_slam_toolbox.launch.py`).
  Same "two installs, one shared mutable resource" pattern as `GZ_CONFIG_PATH`.

## General method

1. Reproduce and note the crashing library from the trace (fontconfig, Mesa,
   a gz plugin, …) — a system lib deep in the stack ⇒ suspect a shared
   cache/config, not your code.
2. `env | grep -i <tool>` inside the activated shell; look for the *other*
   installation's paths bleeding through.
3. Prefer an env-private override (`$CONDA_PREFIX/...` via `XDG_*`/tool-specific
   var) over trying to reconcile versions.
4. Don't stop at the first plausible cause (a prior session wrongly blamed the
   `render` group) — confirm the fix in isolation before concluding.
5. Log the resolution with `log-error`.
