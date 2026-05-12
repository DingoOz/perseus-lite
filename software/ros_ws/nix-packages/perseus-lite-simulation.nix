# Hand-derived from the upstream perseus-simulation.nix (now deleted).
# Mirrors perseus_lite_simulation/package.xml after the Phase 3 fork.
{
  lib,
  buildRosPackage,
  ament-cmake,
  ament-cmake-gtest,
  gz-ros2-control,
  perseus-lite,
  ros-gz-bridge,
  ros-gz-image,
  ros-gz-interfaces,
  ros-gz-sim,
  rosbridge-server,
  twist-mux,
  twist-stamper,
  yaml-cpp-vendor,
}:
buildRosPackage rec {
  pname = "ros-jazzy-perseus-lite-simulation";
  version = "0.0.1";

  src = ./../src/perseus_lite_simulation;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  checkInputs = [
    ament-cmake-gtest
    yaml-cpp-vendor
  ];
  propagatedBuildInputs = [
    gz-ros2-control
    perseus-lite
    ros-gz-bridge
    ros-gz-image
    ros-gz-interfaces
    ros-gz-sim
    rosbridge-server
    twist-mux
    twist-stamper
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "Package for simulating the Perseus Lite rover using Gazebo.";
    license = with lib.licenses; [ mit ];
  };
}
