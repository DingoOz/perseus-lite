# Hand-written: Qt6 is a system dependency satisfied from nixpkgs (not nix-ros-overlay).
{
  lib,
  buildRosPackage,
  ament-cmake,
  geometry-msgs,
  nav-msgs,
  rclcpp,
  sensor-msgs,
  tf2,
  tf2-geometry-msgs,
  tf2-ros,
  qt6,
}:
buildRosPackage rec {
  pname = "ros-jazzy-perseus-lite-screen";
  version = "0.0.0";

  src = ./../src/perseus_lite_screen;

  buildType = "ament_cmake";

  nativeBuildInputs = [
    ament-cmake
    qt6.wrapQtAppsHook
  ];

  buildInputs = [
    ament-cmake
    qt6.qtbase
  ];

  # Qt6 is intentionally NOT in propagatedBuildInputs. The default workspace
  # also pulls in rviz2-fixed (Qt5); having both Qt5 and Qt6 in the workspace
  # closure aborts the shell-env build with "detected mismatched Qt
  # dependencies". Linking to qt6 via buildInputs is enough — the resulting
  # binary's RPATH points at qt6.qtbase in the store.
  propagatedBuildInputs = [
    geometry-msgs
    nav-msgs
    rclcpp
    sensor-msgs
    tf2
    tf2-geometry-msgs
    tf2-ros
  ];

  dontWrapQtApps = false;

  # ROS installs the executable to lib/<pkg>/ but wrapQtAppsHook only scans
  # bin/, sbin/, libexec/, Applications/ — so the auto-wrap is a no-op for
  # ROS layout. Wrap the ROS-located binary explicitly so QT_PLUGIN_PATH /
  # QT_QPA_PLATFORM_PLUGIN_PATH are set when the launch file invokes it.
  postFixup = ''
    if [ -x "$out/lib/perseus_lite_screen/perseus_lite_screen" ]; then
      wrapQtApp "$out/lib/perseus_lite_screen/perseus_lite_screen"
    fi
  '';

  meta = {
    description = "Fullscreen Qt EGLFS top-down map display for Perseus Lite (1024x600 DisplayPort)";
    license = with lib.licenses; [ mit ];
  };
}
