#!/usr/bin/env bash
# `pixi run -e isaac-nitros demo-yolov8-speed` — Demo 3 (plan.md): YOLOv8
# TensorRT (GPU) vs ONNX Runtime (CPU), direct Python API on both sides, no
# ROS in the timing loop. Prints results, saves demo3_result.{json,png},
# then publishes the result chart to /demo3_yolov8_comparison for 2 minutes
# -- view from a laptop on the same ROS domain via RViz2 or rqt_image_view.
set -eo pipefail
# NOTE: no `set -u` -- colcon's generated install/setup.bash isn't
# nounset-safe (references e.g. $COLCON_TRACE without a default).
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ ! -f ws/install/setup.bash ]; then
  echo "ws/install/setup.bash not found -- build the Stage 5/5-follow-up workspace first" \
       "(see README.md commands)." >&2
  exit 1
fi

if [ ! -f models/dummy_yolov8s.onnx ]; then
  echo "models/dummy_yolov8s.onnx not found -- copy it from" \
       "isaac_ros_object_detection/isaac_ros_yolov8/test/dummy_model/yolov8/" \
       "first (see README.md Stage 5 follow-up)." >&2
  exit 1
fi

# shellcheck disable=SC1091
source ws/install/setup.bash
export PYTHONPATH="/usr/lib/python3.12/dist-packages:${PYTHONPATH:-}"

python3 demo3_yolov8_speed_compare.py
