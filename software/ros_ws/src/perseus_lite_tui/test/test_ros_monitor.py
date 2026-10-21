# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the ROS monitor's pure helpers (no rclpy needed)."""

from collections import deque

from perseus_lite_tui import ros_monitor


def test_bulk_types_are_skipped():
    for type_name in (
        'sensor_msgs/msg/Image',
        'sensor_msgs/msg/CompressedImage',
        'sensor_msgs/msg/PointCloud2',
        'nav_msgs/msg/OccupancyGrid',
    ):
        assert ros_monitor.is_bulk_type(type_name), type_name
    for type_name in (
        'geometry_msgs/msg/TwistStamped',
        'sensor_msgs/msg/LaserScan',
        'sensor_msgs/msg/Imu',
    ):
        assert not ros_monitor.is_bulk_type(type_name), type_name


def test_snapshot_marks_unsampled_rows_and_pins_contract_topics():
    monitor = ros_monitor.RosMonitor()
    monitor._types = {
        '/zzz': 'std_msgs/msg/String',
        '/camera/image_raw': 'sensor_msgs/msg/Image',
        '/cmd_vel': 'geometry_msgs/msg/TwistStamped',
    }
    monitor._stamps = {name: deque([1.0, 2.0]) for name in monitor._types}
    monitor._skipped = {'/camera/image_raw'}
    rows = monitor.snapshot()
    assert rows[0]['topic'] == '/cmd_vel'
    by_topic = {row['topic']: row for row in rows}
    assert by_topic['/camera/image_raw']['sampled'] is False
    assert by_topic['/zzz']['sampled'] is True
    assert '/odometry/filtered' in ros_monitor.PINNED
    assert '/odom' not in ros_monitor.PINNED


def test_init_failure_is_surfaced_not_silent():
    class _BrokenRclpy:
        def Context(self):  # noqa: N802 - mimics the rclpy API name
            raise RuntimeError('boom')

    monitor = ros_monitor.RosMonitor()
    monitor._rclpy = _BrokenRclpy()
    monitor.available = True
    monitor._run()
    assert monitor.available is False
    # 'boom' normally; an ImportError message if rclpy.qos/rosidl are missing.
    assert monitor.error.startswith('monitor init failed:')
