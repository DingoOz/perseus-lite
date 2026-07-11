"""
Demo 1 (see plan.md): the comparison/overlay node. Subscribes to the shared
image_rect plus both detection streams (isaac_ros_apriltag's GPU
tag_detections, demo1_cpu_apriltag_node's CPU tag_detections_cpu), measures
rolling FPS and stamp-to-arrival latency for each independently -- measured
at the subscriber, the same way any real downstream consumer would
experience it, not an isolated microbenchmark -- draws tag outlines + a
stats overlay on each side, and publishes:
  - image_gpu_annotated, image_cpu_annotated: each side alone
  - image_comparison: both concatenated side by side with a speedup banner

View image_comparison in a single RViz2 Image display (or rqt_image_view /
image_view) on a laptop on the same ROS domain -- see README.md.
"""
from collections import deque

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

FPS_WINDOW = 30
GPU_LABEL_COLOR = (90, 220, 130)   # green-ish (BGR)
CPU_LABEL_COLOR = (60, 140, 235)   # amber-ish (BGR)
BANNER_HEIGHT = 60


class SpeedCompareNode(Node):
    def __init__(self):
        super().__init__('apriltag_speed_compare')
        self.bridge = CvBridge()

        self.latest_bgr = None
        self.latest_stamp = None

        self.gpu_times = deque(maxlen=FPS_WINDOW)
        self.cpu_times = deque(maxlen=FPS_WINDOW)
        self.gpu_latency_ms = None
        self.cpu_latency_ms = None
        self.gpu_annotated = None
        self.cpu_annotated = None

        # Both phase-gated topics -- only one is actively publishing at a
        # time (see demo1_frame_pump_node.py), but subscribing to both
        # means this node always has a current background frame to draw
        # on regardless of which phase is active.
        self.image_sub_gpu = self.create_subscription(
            Image, 'image_gpu_active', self.on_image, 10)
        self.image_sub_cpu = self.create_subscription(
            Image, 'image_cpu_active', self.on_image, 10)
        self.gpu_sub = self.create_subscription(
            AprilTagDetectionArray, 'tag_detections', self.on_gpu_detections, 10)
        self.cpu_sub = self.create_subscription(
            AprilTagDetectionArray, 'tag_detections_cpu', self.on_cpu_detections, 10)

        self.gpu_pub = self.create_publisher(Image, 'image_gpu_annotated', 10)
        self.cpu_pub = self.create_publisher(Image, 'image_cpu_annotated', 10)
        self.comparison_pub = self.create_publisher(Image, 'image_comparison', 10)

        self.create_timer(0.2, self.publish_comparison)  # 5 Hz
        self.get_logger().info('Speed comparison node ready')

    def on_image(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.latest_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.latest_stamp = msg.header.stamp

    @staticmethod
    def _fps_from(times):
        if len(times) < 2:
            return 0.0
        span = (times[-1] - times[0]).nanoseconds / 1e9
        return (len(times) - 1) / span if span > 0 else 0.0

    def _latency_ms(self, stamp):
        now = self.get_clock().now()
        dt = now - Time.from_msg(stamp)
        return dt.nanoseconds / 1e6

    def on_gpu_detections(self, msg):
        now = self.get_clock().now()
        self.gpu_times.append(now)
        self.gpu_latency_ms = self._latency_ms(msg.header.stamp)
        self.gpu_annotated = self._draw(
            msg, 'GPU (VPI / cuAprilTag)', GPU_LABEL_COLOR,
            self._fps_from(self.gpu_times), self.gpu_latency_ms)
        if self.gpu_annotated is not None:
            self.gpu_pub.publish(self.bridge.cv2_to_imgmsg(self.gpu_annotated, encoding='bgr8'))

    def on_cpu_detections(self, msg):
        now = self.get_clock().now()
        self.cpu_times.append(now)
        self.cpu_latency_ms = self._latency_ms(msg.header.stamp)
        self.cpu_annotated = self._draw(
            msg, 'CPU (pupil-apriltags)', CPU_LABEL_COLOR,
            self._fps_from(self.cpu_times), self.cpu_latency_ms)
        if self.cpu_annotated is not None:
            self.cpu_pub.publish(self.bridge.cv2_to_imgmsg(self.cpu_annotated, encoding='bgr8'))

    def _draw(self, detections_msg, label, color, fps, latency_ms):
        if self.latest_bgr is None:
            return None
        img = self.latest_bgr.copy()

        for det in detections_msg.detections:
            pts = np.array([(int(c.x), int(c.y)) for c in det.corners], dtype=int)
            cv2.polylines(img, [pts], isClosed=True, color=color, thickness=2)
            cv2.putText(img, f'id={det.id}', (int(det.center.x) - 10, int(det.center.y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        banner = f'{label}  |  {fps:5.1f} fps  |  {latency_ms:6.1f} ms  |  {len(detections_msg.detections)} tag(s)'
        cv2.rectangle(img, (0, 0), (img.shape[1], BANNER_HEIGHT), (20, 20, 20), -1)
        cv2.putText(img, banner, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        return img

    def publish_comparison(self):
        if self.gpu_annotated is None or self.cpu_annotated is None:
            return
        combined = cv2.hconcat([self.gpu_annotated, self.cpu_annotated])

        gpu_fps = self._fps_from(self.gpu_times)
        cpu_fps = self._fps_from(self.cpu_times)
        speedup = gpu_fps / cpu_fps if cpu_fps > 0 else 0.0
        h, w = combined.shape[:2]
        # NOTE: this is the end-to-end ROS pipeline message rate, not pure
        # detector compute speed -- in this demo's actual measurements
        # (see plan.md / demo1_frame_pump_node.py's docstring), both sides
        # land within a few percent of each other AND of the pump's own
        # achievable rate at every resolution tried, because the ROS2/DDS
        # message-passing pipeline itself, not either detector, was the
        # binding constraint throughout. Look at apriltag_cpu's periodic
        # "[pure CPU compute, no ROS overhead]" log line for a number that
        # actually isolates algorithm speed.
        cv2.rectangle(combined, (0, h - 34), (w, h), (20, 20, 20), -1)
        text = (f'pipeline msg-rate ratio (GPU/CPU): {speedup:4.1f}x -- see apriltag_cpu log '
                f'for pure compute time'
                if speedup > 0 else 'pipeline msg-rate ratio: (waiting for CPU detections)')
        cv2.putText(combined, text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2)
        cv2.line(combined, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

        self.comparison_pub.publish(self.bridge.cv2_to_imgmsg(combined, encoding='bgr8'))


def main():
    rclpy.init()
    node = SpeedCompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
