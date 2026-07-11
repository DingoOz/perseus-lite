"""
Companion to dnn_classify_launch.py: publishes NVIDIA's own
isaac_ros_dnn_image_encoder test fixture image (test_cases/pose_estimation_0/
image.jpg, 1920x1080 RGB) + matching CameraInfo, and checks that a real
mobilenetv2-1.0 classification comes back through the full
encoder->tensor_rt chain: correct output shape/dtype (same check as
tensor_rt_check.py) plus a basic sanity check that the 1000-way output
isn't degenerate (all-zero/constant, which would mean the resize/normalize
step fed tensor_rt garbage even if the shape happened to match).
"""
import pathlib
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from isaac_ros_tensor_list_interfaces.msg import TensorList

IMAGE_PATH = pathlib.Path(__file__).parent / 'isaac_ros_dnn_inference' / \
    'isaac_ros_dnn_image_encoder' / 'test' / 'test_cases' / 'pose_estimation_0' / 'image.jpg'

EXPECTED_NAME = 'output'
EXPECTED_DATA_TYPE = 9  # float32
EXPECTED_DIMS = [1, 1000]
ENGINE_READY_TIMEOUT_SEC = 60


class DnnClassifyCheck(Node):
    def __init__(self):
        super().__init__('dnn_classify_check')
        self.received = None
        self.sub = self.create_subscription(
            TensorList, 'tensor_sub', self.on_msg, 10)
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def main():
    cv_image = cv2.imread(str(IMAGE_PATH))
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

    camera_info_msg = CameraInfo()
    camera_info_msg.width = cv_image.shape[1]
    camera_info_msg.height = cv_image.shape[0]

    rclpy.init()
    node = DnnClassifyCheck()

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

    ok = node.received is not None and len(node.received.tensors) >= 1
    if ok:
        tensor = node.received.tensors[0]
        data = np.frombuffer(bytes(tensor.data), dtype=np.float32)
        checks = {
            'name': (tensor.name, EXPECTED_NAME),
            'data_type': (tensor.data_type, EXPECTED_DATA_TYPE),
            'dims': (list(tensor.shape.dims), EXPECTED_DIMS),
        }
        for key, (actual, expected) in checks.items():
            if actual != expected:
                ok = False
            node.get_logger().info(f'{key}: got={actual} expected={expected}')

        non_degenerate = bool(data.size == 1000 and np.isfinite(data).all()
                               and np.std(data) > 1e-6)
        ok = ok and non_degenerate
        top5 = np.argsort(data)[-5:][::-1] if data.size == 1000 else []
        node.get_logger().info(
            f'output stats: min={data.min():.4f} max={data.max():.4f} '
            f'std={data.std():.4f} top5_class_idx={list(top5)}'
        )

    print('DNN_CLASSIFY OK' if ok else 'DNN_CLASSIFY FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
