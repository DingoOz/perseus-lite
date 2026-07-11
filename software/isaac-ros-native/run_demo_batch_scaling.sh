#!/usr/bin/env bash
# `pixi run -e isaac-nitros demo-batch-scaling` — Demo 5 (plan.md): U-Net
# batch-size scaling study, TensorRT (GPU) vs ONNX Runtime (CPU), direct
# Python API on both sides. Prints results, saves
# demo5_result.{json,png}, then publishes the result chart to
# /demo5_batch_scaling for 2 minutes -- view from a laptop on the same
# ROS domain via RViz2 or rqt_image_view.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 7 workspace first" \
       "(see README.md Stage 7 commands)." >&2
  exit 1
fi

if [ ! -f models/model.dummy.onnx ]; then
  echo "models/model.dummy.onnx not found -- copy it from" \
       "isaac_ros_image_segmentation/isaac_ros_unet/test/dummy_model/" \
       "first (see README.md Stage 7)." >&2
  exit 1
fi

# shellcheck disable=SC1091
source ws/install/setup.bash
export PYTHONPATH="/usr/lib/python3.12/dist-packages:${PYTHONPATH:-}"

python3 -u demo5_batch_scaling.py
