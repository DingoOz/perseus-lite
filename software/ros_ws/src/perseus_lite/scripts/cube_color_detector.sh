#!/usr/bin/env bash
# Launch the Perseus Lite coloured-cube detector standalone.
#
# Subscribes to /image_raw and /camera_info from the v4l2_camera node and
# publishes:
#   /perseus_vision/cube_color/detections   (perseus_interfaces/ObjectDetections)
#   /perseus_vision/cube_color/markers      (visualization_msgs/MarkerArray)
#   /perseus_vision/cube_color/image        (sensor_msgs/Image, annotated debug)
#   TF: cube_<colour>_<index> in odom (or whatever tf_output_frame is set to)
#   Service: /detect_cubes (perseus_interfaces/DetectObjects)
#
# Detection IDs in the ObjectDetections message:
#   0 = blue, 1 = green, 2 = red, 3 = white
#
# Prerequisites:
#   - perseus_vision and perseus_lite have been built
#       (cd software/ros_ws && colcon build --packages-select \
#        perseus_interfaces perseus_vision perseus_lite --symlink-install)
#   - The camera node is running (either via the full perseus_lite bringup,
#     or standalone: `ros2 run v4l2_camera v4l2_camera_node \
#         --ros-args -p video_device:=/dev/c920 -p image_size:=[320,240]`)
#
# Usage:
#   bash cube_color_detector.sh
#   bash cube_color_detector.sh use_sim_time:=true        # forward launch args
#   ROS_DOMAIN_ID=51 bash cube_color_detector.sh          # override domain
#
# The script matches the production env (ROS_DOMAIN_ID=42, CycloneDDS with the
# bumped MaxAutoParticipantIndex) so the detector's TF and topics are visible
# to Nav2, RViz, and the rest of the autonomy stack on the same network.

set -euo pipefail

: "${ROS_DOMAIN_ID:=42}"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"   # .../software/ros_ws
SETUP="${WS_DIR}/install/setup.bash"

if [[ ! -f "${SETUP}" ]]; then
    echo "ERROR: ${SETUP} not found." >&2
    echo "Build the workspace first:" >&2
    echo "    cd ${WS_DIR}" >&2
    echo "    colcon build --packages-select perseus_interfaces perseus_vision perseus_lite --symlink-install" >&2
    exit 1
fi

# colcon's generated setup.bash references COLCON_TRACE without checking
# whether it's set, so temporarily relax nounset across the source.
set +u
# shellcheck disable=SC1090
source "${SETUP}"
set -u

echo "Launching cube_color_detector on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Annotated image:  /perseus_vision/cube_color/image"
echo "Detections topic: /perseus_vision/cube_color/detections"
echo "RViz markers:     /perseus_vision/cube_color/markers"
echo "Service:          /detect_cubes"
echo

exec ros2 launch perseus_lite cube_color_detector.launch.py "$@"
