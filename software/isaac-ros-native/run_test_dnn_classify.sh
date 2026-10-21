#!/usr/bin/env bash
# `pixi run -e isaac-nitros test-dnn-classify` — Stage 5 DNN inference
# smoke test. Launches dnn_classify_launch.py (image -> dnn_image_encoder
# -> tensor_rt, mobilenetv2-1.0), then runs dnn_classify_check.py against
# it and reports pass/fail. First run builds a real TensorRT engine
# (~40s); later runs reuse the cached /tmp/trt_engine.plan.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 5 workspace first" \
       "(see README.md Stage 5 commands)." >&2
  exit 1
fi

if [ ! -f models/mobilenetv2-1.0.onnx ]; then
  echo "models/mobilenetv2-1.0.onnx not found -- copy it from" \
       "isaac_ros_dnn_inference/isaac_ros_tensor_rt/test/models/ first" \
       "(see README.md Stage 5)." >&2
  exit 1
fi

GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

# shellcheck disable=SC1091
source ws/install/setup.bash
export LD_LIBRARY_PATH="${GXF_LIB_DIRS}${INSTALL_LIB_DIRS}${EXTRA_LIB_DIRS}${LD_LIBRARY_PATH:-}"

ros2 launch dnn_classify_launch.py &
LAUNCH_PID=$!
# `ros2 launch`'s own process dying doesn't kill its component_container_mt
# child (SIGKILL gives it no chance to propagate shutdown) -- pattern-kill
# the container by node name too, or it's orphaned and holds the name for
# the next run.
trap 'kill -9 "$LAUNCH_PID" 2>/dev/null || true
      pkill -9 -f "__node:=dnn_classify_container" 2>/dev/null || true' EXIT

# Give the container time to load both composable nodes before the check
# script starts publishing (engine load from cache is fast, but first-ever
# run builds a fresh TensorRT engine -- see dnn_classify_check.py's own
# ENGINE_READY_TIMEOUT_SEC for the actual wait budget).
sleep 5

python3 dnn_classify_check.py
