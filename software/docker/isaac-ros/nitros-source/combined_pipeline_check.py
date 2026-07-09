"""
Companion to combined_pipeline_launch.py: publishes each pipeline's own
Stage 4/5-follow-up/7/8 test fixture into its namespace concurrently, and
checks that all four produce valid output while running together in one
container/one GPU context. This is a composability test, not a new
capability -- correctness of each individual output was already proven in
its own stage; here the only new question is "do they still work when
they're not alone."
"""
import json
import pathlib
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3DArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

THIS_DIR = pathlib.Path(__file__).parent

APRILTAG_DIR = THIS_DIR / 'isaac_ros_apriltag' / 'isaac_ros_apriltag' / 'test' / \
    'test_cases' / 'apriltag0'
YOLOV8_IMAGE = THIS_DIR / 'isaac_ros_object_detection' / 'isaac_ros_yolov8' / \
    'test' / 'test_cases' / 'single_detection' / 'people_cycles.jpg'
UNET_IMAGE = THIS_DIR / 'isaac_ros_image_segmentation' / 'isaac_ros_unet' / \
    'test' / 'test_cases' / 'unet_sample' / 'image.jpg'
CENTERPOSE_IMAGE = THIS_DIR / 'isaac_ros_pose_estimation' / 'isaac_ros_centerpose' / \
    'test' / 'test_cases' / 'shoe' / 'image.png'

TIMEOUT_SEC = 300
BRIDGE = CvBridge()


def load_camera_info_json(path):
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


def simple_camera_info(width, height):
    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    return msg


class CombinedCheck(Node):
    def __init__(self):
        super().__init__('combined_pipeline_check')
        self.results = {}

        self.apriltag_sub = self.create_subscription(
            AprilTagDetectionArray, '/apriltag/tag_detections',
            lambda m: self.results.setdefault('apriltag', m), 10)
        self.apriltag_image_pub = self.create_publisher(Image, '/apriltag/image', 10)
        self.apriltag_info_pub = self.create_publisher(CameraInfo, '/apriltag/camera_info', 10)

        self.yolov8_sub = self.create_subscription(
            Detection2DArray, '/yolov8/detections_output',
            lambda m: self.results.setdefault('yolov8', m), 10)
        self.yolov8_image_pub = self.create_publisher(Image, '/yolov8/image', 10)
        self.yolov8_info_pub = self.create_publisher(CameraInfo, '/yolov8/camera_info', 10)

        self.unet_sub = self.create_subscription(
            Image, '/unet/unet/raw_segmentation_mask',
            lambda m: self.results.setdefault('unet', m), 10)
        self.unet_image_pub = self.create_publisher(Image, '/unet/image', 10)
        self.unet_info_pub = self.create_publisher(CameraInfo, '/unet/camera_info', 10)

        self.centerpose_sub = self.create_subscription(
            Detection3DArray, '/centerpose/centerpose/detections',
            lambda m: self.results.setdefault('centerpose', m) if len(m.detections) else None, 10)
        self.centerpose_image_pub = self.create_publisher(Image, '/centerpose/image', 10)
        self.centerpose_info_pub = self.create_publisher(CameraInfo, '/centerpose/camera_info', 10)


def main():
    rclpy.init()
    node = CombinedCheck()

    apriltag_json = json.loads((APRILTAG_DIR / 'image.json').read_text())
    apriltag_cv = cv2.imread(str(APRILTAG_DIR / apriltag_json['image']))
    apriltag_msg = BRIDGE.cv2_to_imgmsg(apriltag_cv, encoding=apriltag_json['encoding'])
    apriltag_info = load_camera_info_json(APRILTAG_DIR / 'camera_info.json')

    yolov8_cv = cv2.cvtColor(cv2.imread(str(YOLOV8_IMAGE)), cv2.COLOR_BGR2RGB)
    yolov8_msg = BRIDGE.cv2_to_imgmsg(yolov8_cv, encoding='rgb8')
    yolov8_info = simple_camera_info(yolov8_cv.shape[1], yolov8_cv.shape[0])

    unet_cv = cv2.cvtColor(cv2.imread(str(UNET_IMAGE)), cv2.COLOR_BGR2RGB)
    unet_msg = BRIDGE.cv2_to_imgmsg(unet_cv, encoding='rgb8')
    unet_info = simple_camera_info(unet_cv.shape[1], unet_cv.shape[0])

    centerpose_cv = cv2.cvtColor(cv2.imread(str(CENTERPOSE_IMAGE)), cv2.COLOR_BGR2RGB)
    centerpose_msg = BRIDGE.cv2_to_imgmsg(centerpose_cv, encoding='rgb8')
    # Unlike YOLOv8/U-Net, CenterPoseDecoderNode does a PnP solve using K --
    # an all-zero K (as simple_camera_info() above provides) is a degenerate
    # camera matrix and crashes the whole container with an uncaught
    # cv::Exception (findExtrinsicCameraParams2, fabs(sc) > DBL_EPSILON).
    # Needs the real intrinsics, same as Stage 8's centerpose_check.py.
    centerpose_info = simple_camera_info(centerpose_cv.shape[1], centerpose_cv.shape[0])
    centerpose_info.k = [
        651.2994384765625, 0.0, 298.3225504557292,
        0.0, 651.2994384765625, 392.1635182698568,
        0.0, 0.0, 1.0,
    ]

    end_time = time.time() + TIMEOUT_SEC
    sent = 0
    expected = {'apriltag', 'yolov8', 'unet', 'centerpose'}
    while time.time() < end_time and not expected.issubset(node.results.keys()):
        sent += 1
        stamp = node.get_clock().now().to_msg()

        apriltag_msg.header.stamp = stamp
        apriltag_info.header.stamp = stamp
        node.apriltag_image_pub.publish(apriltag_msg)
        node.apriltag_info_pub.publish(apriltag_info)

        yolov8_msg.header.stamp = stamp
        yolov8_info.header.stamp = stamp
        node.yolov8_image_pub.publish(yolov8_msg)
        node.yolov8_info_pub.publish(yolov8_info)

        unet_msg.header.stamp = stamp
        unet_info.header.stamp = stamp
        node.unet_image_pub.publish(unet_msg)
        node.unet_info_pub.publish(unet_info)

        centerpose_msg.header.stamp = stamp
        centerpose_info.header.stamp = stamp
        node.centerpose_image_pub.publish(centerpose_msg)
        node.centerpose_info_pub.publish(centerpose_info)

        rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(f'sent={sent} received={sorted(node.results.keys())}')

    ok = expected.issubset(node.results.keys())
    if ok:
        node.get_logger().info(
            f'apriltag detections={len(node.results["apriltag"].detections)} '
            f'yolov8 detections={len(node.results["yolov8"].detections)} '
            f'unet mask={node.results["unet"].width}x{node.results["unet"].height} '
            f'centerpose detections={len(node.results["centerpose"].detections)}'
        )
    else:
        missing = expected - node.results.keys()
        node.get_logger().info(f'missing: {sorted(missing)}')

    print('COMBINED OK' if ok else 'COMBINED FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
