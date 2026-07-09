"""
Companion to visual_slam_launch.py: waits for the first nav_msgs/Odometry
message on /visual_slam/tracking/odometry while the RGBD test rosbag
plays, and checks it has a finite, non-degenerate pose (real tracking
happened, not a zeroed/garbage message) plus a valid orientation
quaternion (norm ~1).
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

TIMEOUT_SEC = 30.0


class VisualSlamCheck(Node):
    def __init__(self):
        super().__init__('visual_slam_check')
        self.received = []
        self.sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.on_msg, 10)

    def on_msg(self, msg):
        self.received.append(msg)


def main():
    rclpy.init()
    node = VisualSlamCheck()

    end_time = time.time() + TIMEOUT_SEC
    while time.time() < end_time and not node.received:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = bool(node.received)
    if ok:
        odom = node.received[-1]
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        finite = all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        quat_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        quat_ok = finite and abs(quat_norm - 1.0) < 0.05
        ok = finite and quat_ok
        node.get_logger().info(
            f'received {len(node.received)} odometry messages; last: '
            f'position=({p.x:.4f},{p.y:.4f},{p.z:.4f}) '
            f'orientation_norm={quat_norm:.4f}'
        )
    else:
        node.get_logger().info('no odometry message received before timeout')

    print('VISUAL_SLAM OK' if ok else 'VISUAL_SLAM FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
