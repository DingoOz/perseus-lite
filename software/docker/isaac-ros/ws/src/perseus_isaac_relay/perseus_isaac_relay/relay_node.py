# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Relay Isaac AprilTag detections to standard, bridgeable message types.

The zenoh bridge only forwards `/perseus_isaac/*` topics, and we deliberately
keep Isaac-specific message types (isaac_ros_apriltag_interfaces) OFF the wire so
the Humble container and Jazzy host only ever exchange stable, widely-compatible
types. This node subscribes to the Isaac AprilTag output and republishes:

- geometry_msgs/PoseArray      -> /perseus_isaac/apriltag/poses
- vision_msgs/Detection2DArray -> /perseus_isaac/apriltag/detections
- diagnostic_msgs/DiagnosticArray -> /perseus_isaac/health (1 Hz heartbeat)

Health is OK only while both camera frames (camera_info, a cheap proxy for the
image stream) and AprilTag detection arrays (published per processed frame,
even when empty) keep arriving; STALE when no camera frame has arrived for
`stale_after_s` seconds, WARN when the camera is alive but the AprilTag
pipeline has produced nothing for that long.

This module imports isaac_ros_apriltag_interfaces, which only exists inside the
Isaac ROS container image — importing it on the Jazzy host will fail by design.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Isaac-specific — present only in the container image (see module docstring).
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class AprilTagRelay(Node):
    """Republish Isaac AprilTag detections as standard message types."""

    def __init__(self):
        super().__init__("perseus_isaac_relay")
        self._sub = self.create_subscription(
            AprilTagDetectionArray, "tag_detections", self._on_tags, 10
        )
        self._poses = self.create_publisher(
            PoseArray, "/perseus_isaac/apriltag/poses", 10
        )
        self._dets = self.create_publisher(
            Detection2DArray, "/perseus_isaac/apriltag/detections", 10
        )
        self._health = self.create_publisher(
            DiagnosticArray, "/perseus_isaac/health", 1
        )
        self._camera_sub = self.create_subscription(
            CameraInfo, "camera_info", self._on_camera_info, 10
        )
        self._stale_after_s = float(self.declare_parameter("stale_after_s", 3.0).value)
        self._relayed = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_tags_ns = None
        self._last_camera_ns = None
        self.create_timer(1.0, self._on_health)

    def _on_camera_info(self, _msg):
        self._last_camera_ns = self.get_clock().now().nanoseconds

    def _on_tags(self, msg):
        self._last_tags_ns = self.get_clock().now().nanoseconds
        poses = PoseArray()
        poses.header = msg.header
        dets = Detection2DArray()
        dets.header = msg.header
        for tag in msg.detections:
            pose = _extract_pose(tag)
            poses.poses.append(pose)
            det = Detection2D()
            det.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = _tag_class_id(tag)
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose = pose
            det.results.append(hypothesis)
            dets.detections.append(det)
        self._poses.publish(poses)
        self._dets.publish(dets)
        self._relayed += 1

    def _on_health(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        camera_age = _age_s(now_ns, self._last_camera_ns, self._start_ns)
        tags_age = _age_s(now_ns, self._last_tags_ns, self._start_ns)
        level, message = health_level(
            camera_age,
            tags_age,
            self._last_camera_ns is not None,
            self._last_tags_ns is not None,
            self._stale_after_s,
        )
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        status = DiagnosticStatus()
        status.name = "perseus_isaac_relay"
        status.level = level
        status.message = message
        status.values.append(KeyValue(key="relayed", value=str(self._relayed)))
        status.values.append(
            KeyValue(key="seconds_since_camera_frame", value=f"{camera_age:.1f}")
        )
        status.values.append(
            KeyValue(key="seconds_since_detections", value=f"{tags_age:.1f}")
        )
        array.status.append(status)
        self._health.publish(array)


def _age_s(now_ns, last_ns, start_ns):
    """Seconds since `last_ns` (or since node start if never seen)."""
    return (now_ns - (start_ns if last_ns is None else last_ns)) / 1e9


def health_level(camera_age, tags_age, camera_seen, tags_seen, stale_after_s):
    """Return (DiagnosticStatus level, message) for the relay heartbeat."""
    if camera_age > stale_after_s:
        what = "since start" if not camera_seen else "for %.1fs" % camera_age
        return DiagnosticStatus.STALE, "no camera frames " + what
    if tags_age > stale_after_s:
        what = "since start" if not tags_seen else "for %.1fs" % tags_age
        return DiagnosticStatus.WARN, "camera alive but no apriltag output " + what
    return DiagnosticStatus.OK, "relaying apriltag detections"


def _extract_pose(tag) -> Pose:
    """Pull a geometry_msgs/Pose out of an AprilTagDetection.

    The pose field layout differs across isaac_ros_apriltag_interfaces versions
    (Pose vs PoseStamped vs PoseWithCovariance*); resolve defensively and confirm
    against the installed version on-device.
    """
    candidate = getattr(tag, "pose", None)
    if candidate is None:
        return Pose()
    inner = getattr(candidate, "pose", candidate)
    return getattr(inner, "pose", inner)


def _tag_class_id(tag) -> str:
    """Format a tag as `<family>:<id>` for the detection class label."""
    family = getattr(tag, "family", "tag")
    tag_id = getattr(tag, "id", 0)
    return f"{family}:{tag_id}"


def main(args=None):
    """Spin the relay node."""
    rclpy.init(args=args)
    node = AprilTagRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
