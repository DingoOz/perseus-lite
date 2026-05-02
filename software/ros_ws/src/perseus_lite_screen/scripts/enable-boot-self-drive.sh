#!/usr/bin/env bash
# Enables the perseus-self-drive-boot.service systemd unit so the robot,
# at next boot, shows a 2-minute "Starting Self Driving in..." countdown
# on the DP and then launches `nix run .#perseus-lite-roam`. The screen
# kiosk service is automatically held off until the countdown ends.
#
# Run from the perseus-v2 checkout:
#   sudo software/ros_ws/src/perseus_lite_screen/scripts/enable-boot-self-drive.sh

set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

install -m 0755 "$SCRIPT_DIR/perseus-self-drive-countdown"   /usr/local/bin/perseus-self-drive-countdown
install -m 0755 "$SCRIPT_DIR/perseus-self-drive-roam-launch" /usr/local/bin/perseus-self-drive-roam-launch
install -m 0644 "$SCRIPT_DIR/perseus-self-drive-boot.service" /etc/systemd/system/perseus-self-drive-boot.service

systemctl daemon-reload
systemctl enable perseus-self-drive-boot.service

echo
echo "Boot self-drive enabled."
echo "Will run on next boot: 2 min countdown on DP-1, then perseus-lite-roam."
echo
echo "Disable now:  sudo $SCRIPT_DIR/disable-boot-self-drive.sh"
echo "Trigger now:  sudo systemctl start perseus-self-drive-boot.service"
echo "Abort run:    sudo systemctl stop  perseus-self-drive-boot.service"
echo "Logs:         journalctl -u perseus-self-drive-boot.service -f"
