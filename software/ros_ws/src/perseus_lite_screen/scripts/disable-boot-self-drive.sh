#!/usr/bin/env bash
# Disables the perseus-self-drive-boot.service systemd unit, so the robot
# no longer shows a countdown nor auto-launches perseus-lite-roam at boot.
# If the service is currently running, it is also stopped.
#
# Run from anywhere:
#   sudo software/ros_ws/src/perseus_lite_screen/scripts/disable-boot-self-drive.sh

set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

# disable --now → also stop the unit if it's active right now (mid-countdown
# or running roam), which is exactly the abort behaviour we want.
systemctl disable --now perseus-self-drive-boot.service 2>/dev/null || true

echo "Boot self-drive disabled."
echo "Re-enable: sudo $(dirname "$0")/enable-boot-self-drive.sh"
