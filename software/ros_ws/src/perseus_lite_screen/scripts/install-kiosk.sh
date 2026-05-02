#!/usr/bin/env bash
# Install perseus_lite_screen as a kiosk: stops & disables gdm, switches the
# default boot target to multi-user, installs a wrapper + systemd unit that
# starts the Qt EGLFS map screen on the DP. Reversible; see "rollback" at
# the end of the script's output.
#
# Run from the perseus-v2 checkout:
#   sudo software/ros_ws/src/perseus_lite_screen/scripts/install-kiosk.sh

set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

# The user whose nix profile we use for `nix build`. Falls back to dingo
# when invoked outside sudo (e.g. as root from a console).
NIX_USER="${SUDO_USER:-dingo}"
NIX_USER_HOME=$(getent passwd "$NIX_USER" | cut -d: -f6)
if [ -z "$NIX_USER_HOME" ]; then
    echo "could not resolve home for $NIX_USER" >&2
    exit 1
fi

# Find the perseus-v2 checkout. Prefer the directory the script lives in,
# walking up to the repo root; fall back to $NIX_USER_HOME/perseus-v2.
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || true)
REPO=${REPO:-$NIX_USER_HOME/perseus-v2}
if [ ! -f "$REPO/flake.nix" ]; then
    echo "perseus-v2 flake not found at $REPO" >&2
    exit 1
fi

ROOT=/var/lib/perseus-lite-screen
mkdir -p "$ROOT"
# nix build runs as $NIX_USER, so the GC-root dir needs to be writable by them.
chown "$NIX_USER:$NIX_USER" "$ROOT"

echo "=== building & GC-rooting nix outputs (this may take a minute on first run) ==="
# Use the system-wide nix profile (sudo resets PATH).
NIX=/nix/var/nix/profiles/default/bin/nix
sudo -u "$NIX_USER" -H "$NIX" build --out-link "$ROOT/screen" \
    "path:$REPO#pkgs.ros.perseus-lite-screen"
sudo -u "$NIX_USER" -H "$NIX" build --out-link "$ROOT/workspace" \
    "path:$REPO#default"

echo "=== installing wrapper + service ==="
install -m 0755 "$SCRIPT_DIR/perseus-lite-screen-kiosk" /usr/local/bin/perseus-lite-screen-kiosk
install -m 0644 "$SCRIPT_DIR/perseus-lite-screen.service" /etc/systemd/system/perseus-lite-screen.service
systemctl daemon-reload

echo "=== switching default boot target to multi-user (no GDM at boot) ==="
systemctl set-default multi-user.target

echo "=== disabling gdm at boot ==="
systemctl disable gdm.service || true

echo "=== stopping gdm now ==="
systemctl stop gdm.service || true
sleep 2

# Confirm /dev/dri/card1 is free
if fuser -v /dev/dri/card1 2>&1 | grep -q .; then
    echo "WARNING: /dev/dri/card1 still has open handles after stopping gdm:" >&2
    fuser -v /dev/dri/card1 >&2 || true
fi

echo "=== enabling + starting perseus-lite-screen.service ==="
systemctl enable --now perseus-lite-screen.service
sleep 3
systemctl --no-pager --full status perseus-lite-screen.service | head -25 || true

echo
echo "=== rollback (run any time) ==="
echo "  sudo systemctl disable --now perseus-lite-screen.service"
echo "  sudo systemctl enable gdm.service"
echo "  sudo systemctl set-default graphical.target"
echo "  sudo systemctl start gdm.service"
