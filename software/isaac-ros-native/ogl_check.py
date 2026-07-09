"""
Companion to ogl_launch.py: plays back NVIDIA's own real FlatScan rosbag
(isaac_ros_occupancy_grid_localizer/data/rosbags/flatscan, 12 real 2D
lidar scans recorded 2023-02-25), waits for the localizer to buffer a
scan, calls trigger_grid_search_localization, and checks the resulting
PoseWithCovarianceStamped against NVIDIA's own
isaac_ros_occupancy_grid_localizer_pol_test.py ground truth -- a real
recorded pose, not a dummy/random value. This is the first check in the
series to validate a real GPU CUDA scan-matching result against known
lidar data, not a DNN inference chain.
"""
import math
import pathlib
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

BAG_PATH = pathlib.Path(__file__).parent / 'isaac_ros_mapping_and_localization' / \
    'isaac_ros_occupancy_grid_localizer' / 'data' / 'rosbags' / 'flatscan'

EXPECTED_POSITION = (33.5, 7.75, 0.0)
EXPECTED_QUATERNION = (0.0, 0.0, -0.56573, 0.824589)
POS_TOLERANCE = 0.15
QUAT_TOLERANCE = 0.013
COV_DIAGONAL_INDICES = {0, 7, 35}
COV_TOLERANCE = 0.001

TIMEOUT_SEC = 60


class OglCheck(Node):
    def __init__(self):
        super().__init__('ogl_check')
        self.received = None
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/localization_result', self.on_msg, 10)
        self.cli = self.create_client(Empty, '/trigger_grid_search_localization')

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def main():
    rclpy.init()
    node = OglCheck()

    bag_proc = subprocess.Popen(
        ['ros2', 'bag', 'play', str(BAG_PATH)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not node.cli.wait_for_service(timeout_sec=30.0):
        node.get_logger().info('trigger_grid_search_localization service never appeared')
        print('OGL FAILED')
        bag_proc.terminate()
        sys.exit(1)

    end_time = time.time() + TIMEOUT_SEC
    calls = 0
    while time.time() < end_time and node.received is None:
        calls += 1
        future = node.cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        rclpy.spin_once(node, timeout_sec=0.2)

    bag_proc.terminate()
    bag_proc.wait(timeout=5)

    node.get_logger().info(f'trigger_calls={calls} received={node.received is not None}')

    ok = node.received is not None
    if ok:
        p = node.received.pose.pose.position
        q = node.received.pose.pose.orientation
        cov = node.received.pose.covariance
        node.get_logger().info(
            f'position=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
            f'orientation=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})'
        )

        ex, ey, ez = EXPECTED_POSITION
        eqx, eqy, eqz, eqw = EXPECTED_QUATERNION
        ok = (
            math.isclose(p.x, ex, abs_tol=POS_TOLERANCE)
            and math.isclose(p.y, ey, abs_tol=POS_TOLERANCE)
            and math.isclose(p.z, ez, abs_tol=POS_TOLERANCE)
            and math.isclose(q.x, eqx, abs_tol=QUAT_TOLERANCE)
            and math.isclose(q.y, eqy, abs_tol=QUAT_TOLERANCE)
            and math.isclose(q.z, eqz, abs_tol=QUAT_TOLERANCE)
            and math.isclose(q.w, eqw, abs_tol=QUAT_TOLERANCE)
        )
        for i in range(36):
            if i in COV_DIAGONAL_INDICES:
                ok = ok and cov[i] >= 0.0
            else:
                ok = ok and math.isclose(cov[i], 0.0, abs_tol=COV_TOLERANCE)

    print('OGL OK' if ok else 'OGL FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
