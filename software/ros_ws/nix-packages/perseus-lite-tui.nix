{
  lib,
  buildRosPackage,
  ament-copyright,
  ament-flake8,
  ament-pep257,
  python3Packages,
  rcl-interfaces,
  rclpy,
}:
buildRosPackage rec {
  pname = "ros-jazzy-perseus-lite-tui";
  version = "0.1.0";

  src = ./../src/perseus_lite_tui;

  buildType = "ament_python";
  checkInputs = [
    ament-copyright
    ament-flake8
    ament-pep257
    python3Packages.pytest
  ];
  propagatedBuildInputs = [
    rcl-interfaces
    rclpy
  ];

  meta = {
    description = "Curses TUI for tuning Perseus Lite roam (frontier_explorer) parameters at runtime.";
    license = with lib.licenses; [ mit ];
  };
}
