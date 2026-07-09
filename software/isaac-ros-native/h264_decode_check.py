"""
Companion to h264_decode_launch.py: publishes NVIDIA's own pre-encoded
isaac_ros_h264_decoder test fixture
(test_cases/isaac_ros_h264_decoder/compressed.h264) as a plain
sensor_msgs/CompressedImage (NITROS accepts the compatible ROS type
directly on DecoderNode's "image_compressed" subscription -- same as
NVIDIA's own isaac_ros_decoder_pol.py, no NitrosCompressedImage-specific
publisher needed on our side), and checks that a structurally valid,
non-blank decoded image comes back on image_uncompressed -- proof that
Orin's hardware NVDEC block (present on this SoC, unlike NVENC) works
through our from-source build.
"""
import pathlib
import sys
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

H264_PATH = pathlib.Path(__file__).parent / 'isaac_ros_compression' / \
    'isaac_ros_h264_decoder' / 'test' / 'test_cases' / \
    'isaac_ros_h264_decoder' / 'compressed.h264'

TIMEOUT_SEC = 30


class H264DecodeCheck(Node):
    def __init__(self):
        super().__init__('h264_decode_check')
        self.received = None
        self.sub = self.create_subscription(
            Image, 'image_uncompressed', self.on_msg, 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, 'image_compressed', 10)

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def main():
    compressed_msg = CompressedImage()
    compressed_msg.format = 'h264'
    compressed_msg.data = H264_PATH.read_bytes()

    rclpy.init()
    node = H264DecodeCheck()

    # This fixture is a single-keyframe clip (SPS+PPS+one IDR NAL, not a
    # multi-frame stream) -- resubmitting it in a tight publish loop (as
    # every other check script in this series does) re-feeds the same
    # SPS/PPS/IDR into the stateful V4L2 decode thread faster than it can
    # drain, which desyncs it ("Decode thread error" on every subsequent
    # frame). Publish it a few times with real gaps instead.
    end_time = time.time() + TIMEOUT_SEC
    sent = 0
    while time.time() < end_time and node.received is None:
        sent += 1
        compressed_msg.header.stamp = node.get_clock().now().to_msg()
        node.compressed_pub.publish(compressed_msg)
        deadline = time.time() + 1.0
        while time.time() < deadline and node.received is None:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f'sent={sent} received={node.received is not None}')

    ok = node.received is not None
    if ok:
        out = node.received
        ok = out.width > 0 and out.height > 0
        node.get_logger().info(f'decoded: {out.width}x{out.height} {out.encoding}')
        if ok:
            bridge = CvBridge()
            arr = bridge.imgmsg_to_cv2(out)
            non_blank = bool(np.std(arr) > 1.0)
            node.get_logger().info(f'pixel stddev={np.std(arr):.2f} (non-blank={non_blank})')
            ok = ok and non_blank

    print('H264_DECODE OK' if ok else 'H264_DECODE FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
