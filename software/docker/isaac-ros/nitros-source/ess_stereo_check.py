"""
Companion to ess_stereo_launch.py: publishes synthetic all-zero left/right
1920x1080 images (same as NVIDIA's own isaac_ros_ess_test.py -- disparity
CONTENT is meaningless with a dummy/random-weight model and blank input,
so this isn't tested) plus real calibration (test/camera_info.json,
baseline encoded in P[0][3]=160.0), and checks the resulting
stereo_msgs/DisparityImage against NVIDIA's own ground-truth assertions
for the CALIBRATION-DERIVED fields (f, t, min/max_disparity) -- those
come straight from the input CameraInfo, not the model weights, so
unlike the raw disparity content they *are* meaningful to check exactly.
"""
import json
import math
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage

TEST_DIR = pathlib.Path(__file__).parent / 'isaac_ros_dnn_stereo_depth' / \
    'isaac_ros_ess' / 'test'

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
EXPECTED_WIDTH = 960
EXPECTED_HEIGHT = 576
EXPECTED_F = 434.9440002 * (EXPECTED_WIDTH / IMAGE_WIDTH)
EXPECTED_T = -0.3678634
EXPECTED_MIN_DISPARITY = 0.0
EXPECTED_MAX_DISPARITY = 10000.0
DELTA = 1e-3
ENGINE_READY_TIMEOUT_SEC = 300


def load_camera_info(path):
    d = json.loads(path.read_text())
    msg = CameraInfo()
    msg.header.frame_id = d['header']['frame_id']
    msg.width = d['width']
    msg.height = d['height']
    msg.distortion_model = d['distortion_model']
    msg.d = d['D']
    msg.k = d['K']
    msg.r = d['R']
    msg.p = d['P']
    return msg


def blank_image():
    msg = Image()
    msg.height = IMAGE_HEIGHT
    msg.width = IMAGE_WIDTH
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = IMAGE_WIDTH * 3
    msg.data = bytes(IMAGE_HEIGHT * IMAGE_WIDTH * 3)
    return msg


class EssStereoCheck(Node):
    def __init__(self):
        super().__init__('ess_stereo_check')
        self.received = None
        self.sub = self.create_subscription(
            DisparityImage, '/ess/disparity', self.on_msg, 10)
        self.left_image_pub = self.create_publisher(Image, '/ess/left/image_rect', 10)
        self.right_image_pub = self.create_publisher(Image, '/ess/right/image_rect', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/ess/left/camera_info_rect', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/ess/right/camera_info_rect', 10)

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def main():
    camera_info = load_camera_info(TEST_DIR / 'camera_info.json')
    left_image = blank_image()
    right_image = blank_image()

    rclpy.init()
    node = EssStereoCheck()

    end_time = time.time() + ENGINE_READY_TIMEOUT_SEC
    sent = 0
    while time.time() < end_time and node.received is None:
        sent += 1
        stamp = node.get_clock().now().to_msg()
        left_image.header.stamp = stamp
        right_image.header.stamp = stamp
        camera_info.header.stamp = stamp
        node.left_image_pub.publish(left_image)
        node.right_image_pub.publish(right_image)
        node.left_info_pub.publish(camera_info)
        node.right_info_pub.publish(camera_info)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f'sent={sent} received={node.received is not None}')

    ok = node.received is not None
    if ok:
        disp = node.received
        img = disp.image
        node.get_logger().info(
            f'disparity: {img.width}x{img.height} {img.encoding} '
            f'f={disp.f:.4f} t={disp.t:.4f} '
            f'min_disparity={disp.min_disparity} max_disparity={disp.max_disparity}'
        )
        ok = (
            img.width == EXPECTED_WIDTH and img.height == EXPECTED_HEIGHT
            and img.encoding == '32FC1'
            and img.step == img.width * 4
            and math.isclose(disp.f, EXPECTED_F, abs_tol=DELTA)
            and math.isclose(disp.t, EXPECTED_T, abs_tol=DELTA)
            and math.isclose(disp.min_disparity, EXPECTED_MIN_DISPARITY, abs_tol=DELTA)
            and math.isclose(disp.max_disparity, EXPECTED_MAX_DISPARITY, abs_tol=DELTA)
        )

    print('ESS_STEREO OK' if ok else 'ESS_STEREO FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
