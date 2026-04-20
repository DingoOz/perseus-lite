#!/usr/bin/env bash
# Launch rviz2 on a remote machine (laptop) configured to talk to the Perseus
# Lite robot over the network.
#
# The robot uses CycloneDDS on ROS_DOMAIN_ID=42 and bumps
# MaxAutoParticipantIndex so all Nav2 + SLAM participants are discoverable.
# This script matches that environment so action/service discovery works
# (otherwise you see "Send goal call failed" RTPS payload errors and the
# SlamToolbox plugin hangs on "Waiting for the slam_toolbox node configuration").
#
# Usage:
#   bash rviz_remote.sh                # uses bundled perseus_lite/rviz/nav2.rviz
#   bash rviz_remote.sh --config foo.rviz
#   USE_NIXGL=1 bash rviz_remote.sh    # wrap with nixgl on non-NixOS systems

set -euo pipefail

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_RVIZ="${SCRIPT_DIR}/../rviz/nav2.rviz"

RVIZ_ARGS=()
if [[ $# -eq 0 && -f "${DEFAULT_RVIZ}" ]]; then
    RVIZ_ARGS=(-d "${DEFAULT_RVIZ}")
else
    RVIZ_ARGS=("$@")
fi

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

if [[ "${USE_NIXGL:-0}" == "1" ]]; then
    exec nixgl rviz2 "${RVIZ_ARGS[@]}"
else
    exec rviz2 "${RVIZ_ARGS[@]}"
fi
