# perseus_landmarks

AprilTag landmark localisation for Perseus Lite: anchors the SLAM map to surveyed
"site" coordinates, searches for tags, and drives to named destinations.

| Component                     | What it is                                                                                                                                                                                 |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `tag_detector` (C++)          | OAK-D Lite rectified image in; `/landmarks/detections` (`perseus_interfaces/TagDetectionArray`) out. OpenCV AprilTag dictionaries + IPPE square PnP, with an ambiguity ratio per tag.      |
| `landmark_localiser` (C++)    | Sightings of known tags in; TF `site -> map` out (once calibrated), plus `/landmarks/localisation` and `/landmarks/markers`. Services `~/calibrate` (`CalibrateFromLandmarks`), `~/reset`. |
| `landmark_navigator` (Python) | Actions `/landmarks/search` (`SearchForLandmarks`) and `/landmarks/go_to_destination` (`GoToDestination`), through Nav2 Spin / NavigateToPose.                                             |
| `perseus_landmarks_core`      | ROS-free, unit-tested: landmark map parsing, the `site -> map` solver/estimator, tag detection.                                                                                            |

```shell
ros2 launch perseus_landmarks landmarks.launch.py landmarks_file:=/path/to/site.yaml
```

Usually started with `landmarks:=true` on `perseus_lite_slam_and_nav2.launch.py`
(robot) or `autonomy.launch.py` (sim, with `config/landmarks_sim.yaml` for the
`terrain_test` world).

Files: `config/landmarks.yaml` (landmark-map template, documented inline),
`config/landmarks_sim.yaml`, `config/landmarks_params.yaml` (node parameters).
Full documentation: `docs/source/systems/software/landmark-localisation.md`.

Why `site -> map`: slam_toolbox owns `map -> odom` and the EKF `odom -> base_link`;
tag localisation adds a parent frame instead of fighting either.
