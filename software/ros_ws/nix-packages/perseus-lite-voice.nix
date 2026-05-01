{
  lib,
  buildRosPackage,
  ament-copyright,
  ament-flake8,
  ament-pep257,
  cv-bridge,
  geometry-msgs,
  nav2-msgs,
  python3Packages,
  rcl-interfaces,
  rclpy,
  sensor-msgs,
  std-msgs,
  std-srvs,
}:
buildRosPackage rec {
  pname = "ros-jazzy-perseus-lite-voice";
  version = "0.1.0";

  src = ./../src/perseus_lite_voice;

  buildType = "ament_python";
  checkInputs = [
    ament-copyright
    ament-flake8
    ament-pep257
    python3Packages.pytest
  ];
  # Python deps that exist in nixpkgs are taken straight from there.
  # `openwakeword` (and its `tflite_runtime` dep) are NOT in nixpkgs; they live
  # in the Jetson venv at ~/Programming/Piper/oww-env/ and are picked up at
  # runtime via PYTHONPATH set by the `perseus-lite-voice` flake app. They
  # only matter when wake_enabled:=true, and they're imported lazily, so the
  # Nix build itself does not need them.
  propagatedBuildInputs = [
    cv-bridge
    geometry-msgs
    nav2-msgs
    rcl-interfaces
    rclpy
    sensor-msgs
    std-msgs
    std-srvs
    python3Packages.requests
    python3Packages.numpy
    python3Packages.openai-whisper
    python3Packages.ultralytics
  ];

  meta = {
    description = "Voice assistant ROS 2 node: Ollama + Piper TTS + Whisper + openwakeword + YOLO, with voice-to-robot intent bridge.";
    license = with lib.licenses; [ mit ];
  };
}
