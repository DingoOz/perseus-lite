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

  propagatedBuildInputs = [
    geometry-msgs
    nav-msgs
    rclcpp
    sensor-msgs
    tf2
    tf2-geometry-msgs
    tf2-ros
    qt6.qtbase
  ];

  dontWrapQtApps = false;

  meta = {
    description = "Fullscreen Qt EGLFS top-down map display for Perseus Lite (1024x600 DisplayPort)";
    license = with lib.licenses; [ mit ];
  };
}
