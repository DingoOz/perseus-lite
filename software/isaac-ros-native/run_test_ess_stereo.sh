#!/usr/bin/env bash
# `pixi run -e isaac-nitros test-ess-stereo` — Stage 11 proof-of-life.
# Launches ess_stereo_launch.py (synthetic stereo pair -> ESS TensorRT ->
# DNNStereoDecoderNode, NVIDIA's own dummy_model.onnx), then runs
# ess_stereo_check.py against it and reports pass/fail. First run builds
# a real TensorRT engine; later runs reuse the cached /tmp/ess_dummy.plan.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 5 workspace first" \
       "(see README.md Stage 5 commands)." >&2
  exit 1
fi

if [ ! -f models/ess_dummy_model.onnx ]; then
  echo "models/ess_dummy_model.onnx not found -- copy it from" \
       "isaac_ros_dnn_stereo_depth/isaac_ros_ess/test/dummy_model.onnx" \
       "first (see README.md Stage 11)." >&2
  exit 1
fi

GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

# shellcheck disable=SC1091
source ws/install/setup.bash
export LD_LIBRARY_PATH="${GXF_LIB_DIRS}${INSTALL_LIB_DIRS}${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:-}"

ros2 launch ess_stereo_launch.py &
LAUNCH_PID=$!
# `ros2 launch`'s own process dying doesn't kill its component_container_mt
# child (SIGKILL gives it no chance to propagate shutdown) -- pattern-kill
# the container by node name too, or it's orphaned and holds the name for
# the next run.
trap 'kill -9 "$LAUNCH_PID" 2>/dev/null || true
      pkill -9 -f "__node:=ess_stereo_container" 2>/dev/null || true' EXIT

sleep 10

python3 ess_stereo_check.py
