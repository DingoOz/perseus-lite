#!/usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs
#
# Pixi activation hook (see pixi.toml's [activation] scripts). Defends against
# a parent shell that has already sourced a native/system ROS install (eg
# `/opt/ros/jazzy/setup.bash`) or a foreign colcon workspace (eg
# ~/turtlebot3_ws/install/setup.bash) before `pixi shell`/`pixi run`
# activates this env. Pixi's own activation is additive -- it does not reset
# vars a different toolchain's activation script already set in the same
# shell -- so without this, those stray entries leak into the Pixi env and can
# cause ABI-mismatch crashes (see ERRORS.md: "gz sim segfaults from a
# contaminated GZ_CONFIG_PATH").
#
# IMPORTANT: this file is SOURCED, not executed -- `pixi shell-hook` (and so
# direnv via .envrc) emits `. software/scripts/pixi-env-sanitize.sh` into the
# user's interactive shell. Therefore it must:
#   - never change shell options (no `set -u`/`set -e`/`set -o pipefail`:
#     they would leak into the rest of the hook and the user's shell),
#   - be safe under a caller's `set -u` (every expansion has a default),
#   - never print anything to stdout (that would corrupt activation),
#   - never fail activation on a clean/uncontaminated shell,
#   - leave no functions/variables behind.
#
# Matching rule: an entry is foreign when it is NOT under $CONDA_PREFIX or
# this repo ($PIXI_PROJECT_ROOT, which covers software/ros_ws/install and
# any worktree under it) AND either
#   - it is under /opt/ros/, or
#   - it, or one of its ancestor directories, is a ROS/colcon install prefix
#     (contains `local_setup.bash`, which both ament system installs and every
#     colcon `install/` directory write).

# Path-like (colon-separated) variables to strip foreign entries from.
_PIXI_SANITIZE_PATH_VARS=(
  PATH
  LD_LIBRARY_PATH
  AMENT_PREFIX_PATH
  CMAKE_PREFIX_PATH
  PYTHONPATH
  COLCON_PREFIX_PATH
  ROS_PACKAGE_PATH
  GZ_CONFIG_PATH
  GZ_SIM_RESOURCE_PATH
  GZ_SIM_SYSTEM_PLUGIN_PATH
  GZ_GUI_PLUGIN_PATH
)

# Prefixes that are always ours (never stripped). Empty values are skipped.
_PIXI_SANITIZE_OWN_PREFIXES=(
  "${CONDA_PREFIX:-}"
  "${PIXI_PROJECT_ROOT:-}"
)

# Cache of directory -> 1 (is a ROS/colcon prefix) / 0 (is not).
declare -A _PIXI_SANITIZE_PREFIX_CACHE=()

_pixi_sanitize_is_own() {
  local entry="$1" prefix
  for prefix in "${_PIXI_SANITIZE_OWN_PREFIXES[@]}"; do
    [[ -z $prefix ]] && continue
    prefix="${prefix%/}"
    [[ $entry == "$prefix" || $entry == "$prefix"/* ]] && return 0
  done
  return 1
}

# Returns 0 if $1 or any ancestor directory is a ROS/colcon install prefix.
# The answer is cached for every directory walked, so each directory is
# stat'ed at most once per activation.
_pixi_sanitize_in_ros_prefix() {
  local dir="${1%/}" found=0 walked
  local -a walked_dirs=()
  while [[ -n $dir ]]; do
    if [[ -n ${_PIXI_SANITIZE_PREFIX_CACHE[$dir]:-} ]]; then
      found="${_PIXI_SANITIZE_PREFIX_CACHE[$dir]}"
      break
    fi
    walked_dirs+=("$dir")
    if [[ -f $dir/local_setup.bash ]]; then
      found=1
      break
    fi
    [[ $dir != */* ]] && break
    dir="${dir%/*}"
  done
  for walked in "${walked_dirs[@]}"; do
    _PIXI_SANITIZE_PREFIX_CACHE[$walked]="$found"
  done
  [[ $found == 1 ]]
}

_pixi_sanitize_is_foreign() {
  local entry="$1"
  # Relative entries (eg "." in PATH) are left alone.
  [[ $entry != /* ]] && return 1
  _pixi_sanitize_is_own "$entry" && return 1
  [[ $entry == /opt/ros/* ]] && return 0
  _pixi_sanitize_in_ros_prefix "$entry" && return 0
  return 1
}

_pixi_sanitize_strip_var() {
  local var_name="$1" old_value entry new_value changed=0
  local -a kept=() entries=()

  old_value="${!var_name:-}"
  [[ -z $old_value ]] && return 0

  IFS=":" read -r -a entries <<<"$old_value"
  for entry in "${entries[@]}"; do
    [[ -z $entry ]] && continue
    if _pixi_sanitize_is_foreign "$entry"; then
      changed=1
    else
      kept+=("$entry")
    fi
  done
  [[ $changed == 0 ]] && return 0

  if ((${#kept[@]} == 0)); then
    unset "$var_name"
    return 0
  fi
  new_value="$(
    IFS=:
    printf '%s' "${kept[*]}"
  )"
  export "${var_name}=${new_value}"
}

for _pixi_sanitize_var in "${_PIXI_SANITIZE_PATH_VARS[@]}"; do
  _pixi_sanitize_strip_var "$_pixi_sanitize_var"
done

unset -f _pixi_sanitize_is_own _pixi_sanitize_in_ros_prefix \
  _pixi_sanitize_is_foreign _pixi_sanitize_strip_var
unset _pixi_sanitize_var _PIXI_SANITIZE_PATH_VARS \
  _PIXI_SANITIZE_OWN_PREFIXES _PIXI_SANITIZE_PREFIX_CACHE

true
