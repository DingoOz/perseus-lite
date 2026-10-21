#!/usr/bin/env bash
# `pixi run -e isaac-nitros demo-ogl-speed` — Demo 2 (plan.md): occupancy-grid
# localizer GPU vs CPU, using NVIDIA's own real compiled code on both sides
# (forces the genuine CPU fallback path via CUDA_VISIBLE_DEVICES="", not a
# reimplementation). Runs the full localization pipeline twice sequentially
# (GPU pass, then CPU pass -- the CPU pass can take a while, real compute
# over ~250k candidate poses), then publishes a result image to
# /demo2_ogl_comparison for a few minutes. View from a laptop on the same
# ROS domain: ros2 run rqt_image_view rqt_image_view /demo2_ogl_comparison
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 12 workspace first" \
       "(see README.md Stage 12 commands)." >&2
  exit 1
fi

GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

# shellcheck disable=SC1091
source ws/install/setup.bash
export LD_LIBRARY_PATH="${GXF_LIB_DIRS}${INSTALL_LIB_DIRS}${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:-}"

python3 demo2_ogl_speed_compare.py
