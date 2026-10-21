#!/usr/bin/env python3
"""
LaserScan minimum-range filter.

The lite's RPLidar is mounted under the chassis and sees the robot's own
rocker arms and wheels. This node republishes /scan as /scan_filtered with
every reading closer than ``min_range`` (and any NaN) replaced by +inf, and
raises ``range_min`` to match, so SLAM and Nav2 ignore the robot's body.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def filter_ranges(ranges, min_range):
    """Return ``ranges`` with readings below ``min_range`` (or NaN) set to inf."""
    inf = math.inf
    return [r if r >= min_range else inf for r in ranges]


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__("scan_range_filter")
        self._min_range = float(self.declare_parameter("min_range", 0.5).value)

        self._pub = self.create_publisher(
            LaserScan, "scan_filtered", qos_profile_sensor_data
        )
        self._sub = self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Filtering scan -> scan_filtered (min_range={self._min_range} m)"
        )

    def _on_scan(self, msg: LaserScan):
        msg.ranges = filter_ranges(msg.ranges, self._min_range)
        msg.range_min = max(msg.range_min, self._min_range)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
