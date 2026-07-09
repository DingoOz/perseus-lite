#!/usr/bin/env bash
# `pixi run -e isaac-nitros test-ogl` — Stage 12 proof-of-life. Launches
# ogl_launch.py (occupancy_grid_localizer against NVIDIA's own real map
# fixture), then runs ogl_check.py (plays NVIDIA's own real FlatScan
# rosbag, triggers localization, checks the result against NVIDIA's own
# recorded ground-truth pose) and reports pass/fail.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 5 workspace first" \
       "(see README.md Stage 5 commands)." >&2
  exit 1
fi

GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

# shellcheck disable=SC1091
source ws/install/setup.bash
export LD_LIBRARY_PATH="${GXF_LIB_DIRS}${INSTALL_LIB_DIRS}${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:-}"

ros2 launch ogl_launch.py &
LAUNCH_PID=$!
# `ros2 launch`'s own process dying doesn't kill its component_container_mt
# child (SIGKILL gives it no chance to propagate shutdown) -- pattern-kill
# the container by node name too, or it's orphaned and holds the name for
# the next run.
trap 'kill -9 "$LAUNCH_PID" 2>/dev/null || true
      pkill -9 -f "__node:=ogl_container" 2>/dev/null || true' EXIT

sleep 5

python3 ogl_check.py
