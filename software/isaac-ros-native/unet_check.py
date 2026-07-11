"""
Companion to unet_launch.py: publishes NVIDIA's own isaac_ros_unet test
image (test_cases/unet_sample/image.jpg, 1200x632) + matching CameraInfo,
and checks that structurally valid raw (mono8) and colorized (rgb8)
segmentation masks come back through the full
encoder->tensor_rt->unet_decoder chain, at the expected network resolution
(960x544).

model.dummy.onnx has RANDOM weights (see unet_launch.py), so per-pixel
class assignments are not semantically meaningful -- NVIDIA's own
isaac_ros_unet_pol_test.py explicitly only checks output shape/encoding,
not mask content, for the same reason. This only proves the full
encode->infer->decode chain runs end to end on real tensor_rt output
without crashing and produces structurally valid segmentation masks.
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

IMAGE_PATH = pathlib.Path(__file__).parent / 'isaac_ros_image_segmentation' / \
    'isaac_ros_unet' / 'test' / 'test_cases' / 'unet_sample' / 'image.jpg'

EXPECTED_WIDTH = 960
EXPECTED_HEIGHT = 544

ENGINE_READY_TIMEOUT_SEC = 300


class UnetCheck(Node):
    def __init__(self):
        super().__init__('unet_check')
        self.raw_mask = None
        self.color_mask = None
        self.raw_sub = self.create_subscription(
            Image, 'unet/raw_segmentation_mask', self.on_raw, 10)
        self.color_sub = self.create_subscription(
            Image, 'unet/colored_segmentation_mask', self.on_color, 10)
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

    def on_raw(self, msg):
        if self.raw_mask is None:
            self.raw_mask = msg

    def on_color(self, msg):
        if self.color_mask is None:
            self.color_mask = msg


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
    node = UnetCheck()

    end_time = time.time() + ENGINE_READY_TIMEOUT_SEC
    sent = 0
    while time.time() < end_time and (node.raw_mask is None or node.color_mask is None):
        sent += 1
        stamp = node.get_clock().now().to_msg()
        image_msg.header.stamp = stamp
        camera_info_msg.header.stamp = stamp
        node.image_pub.publish(image_msg)
        node.camera_info_pub.publish(camera_info_msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f'sent={sent} raw_received={node.raw_mask is not None} '
        f'color_received={node.color_mask is not None}'
    )

    ok = node.raw_mask is not None and node.color_mask is not None
    if ok:
        raw = node.raw_mask
        color = node.color_mask
        ok = (
            raw.width == EXPECTED_WIDTH and raw.height == EXPECTED_HEIGHT
            and raw.encoding == 'mono8'
            and color.width == EXPECTED_WIDTH and color.height == EXPECTED_HEIGHT
            and color.encoding == 'rgb8'
        )
        node.get_logger().info(
            f'raw: {raw.width}x{raw.height} {raw.encoding} | '
            f'color: {color.width}x{color.height} {color.encoding}'
        )
        raw_arr = bridge.imgmsg_to_cv2(raw, desired_encoding='mono8')
        num_classes_seen = len(np.unique(raw_arr))
        node.get_logger().info(f'distinct class ids in raw mask: {num_classes_seen}')

    print('UNET OK' if ok else 'UNET FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
