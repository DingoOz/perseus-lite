# perseus_lite_screen

Fullscreen Qt6 EGLFS top-down map display for the on-robot 1024x600
DisplayPort screen on Perseus Lite. Subscribes to `/map`, `/scan_filtered`,
`/plan`, `/goal_pose`; uses TF (`map -> base_link`) to keep the robot
centred while the map stays world-aligned (north-up regardless of robot
heading). HUD overlay shows pose, IPv4 address, available memory,
map-freshness indicator and a 1 m scale bar. ESC/Q quits, +/- zooms.

The package is built via Nix (`nix build .#pkgs.ros.perseus-lite-screen`).
For Qt-mismatch reasons it is intentionally **excluded from the default
ROS workspace closure** — see [Why a separate workspace](#why-a-separate-workspace).

## Run modes

### 1. Ad-hoc (development, on top of GDM)

For iterative dev work where you still want the desktop. Stop GDM
manually first to release DRM master:

```bash
sudo systemctl stop gdm
ros2 launch perseus_lite_screen perseus_lite_screen.launch.py
sudo systemctl start gdm   # when done
```

The launch file defaults `qpa_platform=eglfs` and
`eglfs_integration=eglfs_kms_egldevice`. To preview on the laptop /
desktop instead, override:

```bash
QT_QPA_PLATFORM=xcb ros2 launch perseus_lite_screen \
  perseus_lite_screen.launch.py qpa_platform:=xcb
```

### 2. Kiosk (production, no desktop)

The robot Jetson normally runs **no graphical session at all** — GDM,
Xorg and gnome-shell are disabled and the only thing on the DP is
perseus_lite_screen, brought up by systemd at boot. This frees ~280 MB
RAM and a couple of CPUs that would otherwise idle in the desktop.

#### Install

From the perseus-v2 checkout on the robot:

```bash
sudo software/ros_ws/src/perseus_lite_screen/scripts/install-kiosk.sh
```

The script:

1. `nix build --out-link /var/lib/perseus-lite-screen/{screen,workspace}` —
   creates GC roots so the binary survives `nix-collect-garbage`.
2. Installs `/usr/local/bin/perseus-lite-screen-kiosk` (wrapper that
   sets ROS, Tegra-EGL and Qt env, then execs the binary).
3. Installs `/etc/systemd/system/perseus-lite-screen.service`.
4. `systemctl set-default multi-user.target` — boots without graphical.
5. `systemctl disable gdm.service` — gdm won't auto-start.
6. `systemctl stop gdm.service` — kills the current desktop session.
7. `systemctl enable --now perseus-lite-screen.service`.

The script is idempotent — re-run it after pulling new perseus-v2
commits to refresh the GC-rooted nix store paths and pick up new
binaries.

#### Update after a rebuild

```bash
sudo software/ros_ws/src/perseus_lite_screen/scripts/install-kiosk.sh
sudo systemctl restart perseus-lite-screen.service
```

(The script overwrites the GC roots, which point at fresh nix store
paths.)

#### Rollback to GDM/desktop

```bash
sudo systemctl disable --now perseus-lite-screen.service
sudo systemctl enable gdm.service
sudo systemctl set-default graphical.target
sudo systemctl start gdm.service
```

#### Logs / debugging

```bash
journalctl -u perseus-lite-screen.service -f
journalctl -u perseus-lite-screen.service -b   # since boot
```

For verbose Qt EGLFS logging, edit
`/usr/local/bin/perseus-lite-screen-kiosk` and add
`export QT_LOGGING_RULES="qt.qpa.*=true"` before the final `exec`.

### 3. Auto self-drive on boot (optional, opt-in)

When enabled, the robot at every boot:

1. Shows a 2-minute **"Starting Self Driving in… `MM:SS`"** countdown on
   the DP-1 console (still text-mode at that point — the kiosk hasn't
   grabbed DRM yet).
2. After the countdown, launches `nix run .#perseus-lite-roam` (frontier
   exploration on top of Nav2 + SLAM).
3. The kiosk then takes the screen and starts rendering the live map.

The countdown and the roam launch are a single systemd unit
(`perseus-self-drive-boot.service`) ordered `Before=perseus-lite-screen.service`,
so the screen waits until roam is up before grabbing the DP.

Disabled by default; opt in with the enable script.

#### Enable

```bash
sudo software/ros_ws/src/perseus_lite_screen/scripts/enable-boot-self-drive.sh
```

#### Disable

```bash
sudo software/ros_ws/src/perseus_lite_screen/scripts/disable-boot-self-drive.sh
```

`disable` doubles as a runtime abort: if the unit is mid-countdown or
already running roam, it gets stopped immediately.

#### Trigger / abort manually

```bash
sudo systemctl start perseus-self-drive-boot.service   # run countdown + roam now
sudo systemctl stop  perseus-self-drive-boot.service   # abort an in-progress run
journalctl -u perseus-self-drive-boot.service -f
```

## Hardware / driver requirements

This runs on Jetson Orin (L4T 36.x). The kiosk depends on the
host-system Nvidia EGL driver, picked up by libglvnd via:

| Path                                                        | Why                                       |
| ----------------------------------------------------------- | ----------------------------------------- |
| `/usr/lib/aarch64-linux-gnu/tegra-egl/libEGL_nvidia.so.0`   | Nvidia EGL ICD                            |
| `/usr/lib/aarch64-linux-gnu/tegra/`                         | CUDA / display libs the EGL ICD links to  |
| `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`              | Tells libglvnd to load `libEGL_nvidia.so` |

The wrapper script prepends these to `LD_LIBRARY_PATH` and points
`__EGL_VENDOR_LIBRARY_DIRS` at `/usr/share/glvnd/egl_vendor.d`. Without
them you'll see `EGL_EXT_device_base missing` and an abort.

## Qt EGLFS configuration

`config/eglfs_kms.json` selects the DRM device and the connected
output:

```json
{
  "device": "/dev/dri/card1",
  "hwcursor": false,
  "outputs": [
    { "name": "DP-1", "mode": "1024x600",
      "physicalWidth": 220, "physicalHeight": 130 }
  ]
}
```

On this Jetson `/dev/dri/card0` has no connectors — DP-1 lives on
**`card1`**. `udev`-style enumeration is not used by Qt EGLFS, so the
correct card has to be named explicitly.

The launch file sets `QT_QPA_EGLFS_INTEGRATION=eglfs_kms_egldevice`
(NVIDIA EGLStream KMS), not the GBM-based `eglfs_kms`. The Qt6 nix
package only ships `eglfs_kms_egldevice`, `eglfs_x11` and `eglfs_emu`.

## Why a separate workspace

The package links Qt6, while the default workspace also pulls in
`rviz2-fixed` (Qt5). Mixing Qt5 + Qt6 in one Nix workspace closure
trips `wrapQtAppsHook` with *"detected mismatched Qt dependencies"*
and aborts the shell-env build, so `nix develop` becomes unusable.

To avoid that, `perseus-lite-screen` is removed from the default
workspace's `devPackages` (see `software/ros_ws/overlay.nix`). It is
still reachable as `pkgs.ros.perseus-lite-screen` and from the
`workspace` GC root that the kiosk installer creates.

Side effect: `ros2 launch perseus_lite perseus_lite.launch.py` won't
find `perseus_lite_screen` via `FindPackageShare` from the default
workspace alone. Either (a) run the kiosk service, which has its own
ROS env stack, or (b) build perseus-lite-screen separately and prepend
its share dir to `AMENT_PREFIX_PATH` for ad-hoc launches.

## Files in this package

```
config/
  eglfs_kms.json          Qt EGLFS KMS config (DRM device + output)
  screen_params.yaml      Topic names, frames, pixels-per-meter
launch/
  perseus_lite_screen.launch.py
scripts/
  install-kiosk.sh                            Kiosk installer (sudo)
  perseus-lite-screen-kiosk                   Wrapper called by the kiosk systemd unit
  perseus-lite-screen.service                 Kiosk systemd unit template
  perseus-lite-display-modload.conf           modules-load.d entry forcing tegra-drm + nvidia-drm at boot
  enable-boot-self-drive.sh                   Opt in to 2-min countdown + auto-roam at boot
  disable-boot-self-drive.sh                  Opt out / abort the boot self-drive
  perseus-self-drive-countdown                Countdown shown on tty1 during the wait
  perseus-self-drive-roam-launch              Wrapper that exec's `nix run .#perseus-lite-roam`
  perseus-self-drive-boot.service             systemd unit gluing the countdown + launch
src/
  main.cpp                QApplication + rclcpp glue
  map_screen_node.cpp/.hpp  ROS subscriptions + TF lookup
  map_view_widget.cpp/.hpp  Qt rendering (map, scan, plan, HUD)
```
