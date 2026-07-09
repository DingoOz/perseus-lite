"""
Demo 1 (see plan.md): CPU-only AprilTag detector, the baseline half of the
GPU-vs-CPU speed comparison. Wraps pupil-apriltags (the official AprilTag C
library's Python binding -- genuinely CPU-only, no CUDA/NEON intrinsics of
its own beyond whatever the C library does internally), publishing the same
isaac_ros_apriltag_interfaces/AprilTagDetectionArray message type the GPU
node (isaac_ros_apriltag's AprilTagNode) publishes, so both sides are
directly comparable and share the same overlay-drawing code in
demo1_speed_compare_node.py.

Subscribes to the SAME image_rect the GPU node consumes (not raw image) --
this holds the GPU-accelerated rectify cost constant on both sides, so the
comparison isolates the detection stage itself, not rectify+detect vs
detect alone. Publishes on every frame regardless of detection count
(matching AprilTagNode's own behavior -- see apriltag_node.cpp, both
detections_pub_->publish() call sites are unconditional), so message rate
directly reflects processing FPS for both pipelines.
"""
from collections import deque
import math
import time

from cv_bridge import CvBridge
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from isaac_ros_apriltag_interfaces.msg import AprilTagDetection, AprilTagDetectionArray


def rotation_matrix_to_quaternion(rot_mat):
    """Standard trace-based rotation-matrix-to-quaternion conversion (no
    scipy dependency -- this is the only place a quaternion is needed)."""
    m = rot_mat
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2][1] - m[1][2]) * s
        y = (m[0][2] - m[2][0]) * s
        z = (m[1][0] - m[0][1]) * s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


class CpuAprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_cpu')
        self.declare_parameter('size', 0.22)
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('nthreads', 2)
        self.tag_size = self.get_parameter('size').value
        family = self.get_parameter('family').value
        nthreads = self.get_parameter('nthreads').value

        self.detector = Detector(families=family, nthreads=nthreads)
        self.bridge = CvBridge()
        self.camera_params = None  # (fx, fy, cx, cy), set from camera_info_rect

        # Pure algorithm compute time -- wall-clock around detector.detect()
        # only, excluding ROS callback/message/cv_bridge overhead. The GPU
        # side (a closed compiled binary) can't be instrumented the same
        # way, so this is the one number in the whole demo that's fully
        # decoupled from the ROS2/DDS message-passing throughput ceiling
        # that turned out to dominate every other measurement in this demo
        # (see demo1_frame_pump_node.py's docstring for the full story).
        self.compute_times = deque(maxlen=60)

        self.detections_pub = self.create_publisher(
            AprilTagDetectionArray, 'tag_detections_cpu', 10)
        # *_cpu_active -- the pump (demo1_frame_pump_node.py) only
        # publishes here during its CPU phase, so this node only receives
        # frames when it isn't competing with the GPU detector.
        self.image_sub = self.create_subscription(
            Image, 'image_cpu_active', self.on_image, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera_info_cpu_active', self.on_camera_info, 10)

        self.create_timer(3.0, self.log_compute_stats)
        self.get_logger().info(
            f'CPU AprilTag detector ready: family={family} nthreads={nthreads}')

    def log_compute_stats(self):
        if not self.compute_times:
            return
        mean_ms = 1000.0 * sum(self.compute_times) / len(self.compute_times)
        max_fps = 1000.0 / mean_ms if mean_ms > 0 else 0.0
        self.get_logger().info(
            f'[pure CPU compute, no ROS overhead] detect(): {mean_ms:.2f} ms/frame '
            f'mean over last {len(self.compute_times)} calls -> {max_fps:.1f} fps max')

    def on_camera_info(self, msg):
        # Post-rectification P matrix -- fx, fy, cx, cy at indices 0, 5, 2, 6.
        p = msg.p
        self.camera_params = (p[0], p[5], p[2], p[6])

    def on_image(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        estimate_pose = self.camera_params is not None
        t0 = time.perf_counter()
        results = self.detector.detect(
            gray,
            estimate_tag_pose=estimate_pose,
            camera_params=self.camera_params if estimate_pose else None,
            tag_size=self.tag_size,
        )
        self.compute_times.append(time.perf_counter() - t0)

        out = AprilTagDetectionArray()
        out.header = msg.header
        for r in results:
            det = AprilTagDetection()
            det.family = (
                r.tag_family.decode() if isinstance(r.tag_family, bytes) else str(r.tag_family)
            )
            det.id = int(r.tag_id)
            det.center = Point(x=float(r.center[0]), y=float(r.center[1]), z=0.0)
            det.corners = [Point(x=float(c[0]), y=float(c[1]), z=0.0) for c in r.corners]

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = msg.header
            if estimate_pose and r.pose_R is not None and r.pose_t is not None:
                qx, qy, qz, qw = rotation_matrix_to_quaternion(r.pose_R)
                pose_msg.pose.pose.position.x = float(r.pose_t[0][0])
                pose_msg.pose.pose.position.y = float(r.pose_t[1][0])
                pose_msg.pose.pose.position.z = float(r.pose_t[2][0])
                pose_msg.pose.pose.orientation.x = qx
                pose_msg.pose.pose.orientation.y = qy
                pose_msg.pose.pose.orientation.z = qz
                pose_msg.pose.pose.orientation.w = qw
            det.pose = pose_msg
            out.detections.append(det)

        self.detections_pub.publish(out)


def main():
    rclpy.init()
    node = CpuAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
