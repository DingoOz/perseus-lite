#!/bin/bash
set -e # Exit immediately on any error

SERVICE_NAME="timesync-google.service"
SCRIPT_NAME="timesync-google.sh"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_SRC="$SRC_DIR/$SERVICE_NAME"
SCRIPT_SRC="$SRC_DIR/$SCRIPT_NAME"
SERVICE_DEST="/etc/systemd/system/$SERVICE_NAME"
SCRIPT_DEST="/usr/local/sbin/timesync-google"

# Check the service and script exist next to this script
for f in "$SERVICE_SRC" "$SCRIPT_SRC"; do
  if [ ! -f "$f" ]; then
    echo "ERROR: $(basename "$f") not found next to this script ($f)"
    exit 1
  fi
done

echo "Installing time-sync script to $SCRIPT_DEST..."
sudo install -m 755 "$SCRIPT_SRC" "$SCRIPT_DEST"

echo "Installing service..."
sudo install -m 644 "$SERVICE_SRC" "$SERVICE_DEST"

echo "Reloading daemons..."
sudo systemctl daemon-reload

echo "Enabling service for persistence..."
sudo systemctl enable "$SERVICE_NAME"

echo "All done. The clock will be set on the next boot; to set it now run:"
echo "  sudo systemctl start $SERVICE_NAME && systemctl status $SERVICE_NAME"
