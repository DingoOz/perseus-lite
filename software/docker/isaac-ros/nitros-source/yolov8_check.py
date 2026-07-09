"""
Companion to yolov8_launch.py: publishes NVIDIA's own isaac_ros_yolov8
test image (test_cases/single_detection/people_cycles.jpg, 640x640) +
matching CameraInfo, and checks that a structurally valid
vision_msgs/Detection2DArray comes back through the full
encoder->tensor_rt->yolov8_decoder chain.

dummy_yolov8s.onnx has RANDOM weights (see yolov8_launch.py), so detected
boxes/classes are not semantically meaningful -- with untrained weights the
decoder's raw box regression/objectness outputs aren't guaranteed to be
positive or bounded (NVIDIA's own isaac_ros_yolov8_decoder_node_pol.py
explicitly only checks that a message arrives, not detection content, for
the same reason -- "the data is not verified because the model is
initialized with random weights"). This only proves the full
encode->infer->decode chain runs end to end on real tensor_rt output
without crashing and produces a structurally valid Detection2DArray, not
that YOLO correctly found the people/bicycles in the image.
"""
import pathlib
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

IMAGE_PATH = pathlib.Path(__file__).parent / 'isaac_ros_object_detection' / \
    'isaac_ros_yolov8' / 'test' / 'test_cases' / 'single_detection' / 'people_cycles.jpg'

ENGINE_READY_TIMEOUT_SEC = 60


class Yolov8Check(Node):
    def __init__(self):
        super().__init__('yolov8_check')
        self.received = None
        self.sub = self.create_subscription(
            Detection2DArray, 'detections_output', self.on_msg, 10)
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def main():
    cv_image_bgr = cv2.imread(str(IMAGE_PATH))
    cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(cv_image_rgb, encoding='rgb8')

    camera_info_msg = CameraInfo()
    camera_info_msg.width = cv_image_rgb.shape[1]
    camera_info_msg.height = cv_image_rgb.shape[0]
    camera_info_msg.distortion_model = 'plumb_bob'

    rclpy.init()
    node = Yolov8Check()

    end_time = time.time() + ENGINE_READY_TIMEOUT_SEC
    sent = 0
    while time.time() < end_time and node.received is None:
        sent += 1
        stamp = node.get_clock().now().to_msg()
        image_msg.header.stamp = stamp
        camera_info_msg.header.stamp = stamp
        node.image_pub.publish(image_msg)
        node.camera_info_pub.publish(camera_info_msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f'sent={sent} received={node.received is not None}')

    # A Detection2DArray (even empty, even with nonsensical scores/boxes from
    # the random-weight model) arriving at all is success -- see module
    # docstring for why content isn't validated here.
    ok = node.received is not None
    if ok:
        dets = node.received.detections
        node.get_logger().info(f'num_detections={len(dets)}')
        for det in dets:
            bbox = det.bbox
            has_result = len(det.results) > 0
            ok = ok and has_result
            if has_result:
                node.get_logger().info(
                    f'  class_id={det.results[0].hypothesis.class_id} '
                    f'score={det.results[0].hypothesis.score:.3f} '
                    f'bbox=({bbox.center.position.x:.1f},{bbox.center.position.y:.1f} '
                    f'{bbox.size_x:.1f}x{bbox.size_y:.1f})'
                )

    print('YOLOV8 OK' if ok else 'YOLOV8 FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
