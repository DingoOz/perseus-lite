# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the Twist -> TwistStamped conversion."""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from twist_stamper.twist_stamper import stamp_twist


def test_stamp_twist_copies_twist_and_header():
    twist = Twist()
    twist.linear.x = 0.5
    twist.angular.z = -1.25
    stamp = Time(sec=12, nanosec=34)
    stamped = stamp_twist(twist, stamp, 'base_link')
    assert stamped.twist.linear.x == 0.5
    assert stamped.twist.angular.z == -1.25
    assert stamped.header.stamp.sec == 12
    assert stamped.header.stamp.nanosec == 34
    assert stamped.header.frame_id == 'base_link'


def test_stamp_twist_default_frame_is_empty():
    stamped = stamp_twist(Twist(), Time())
    assert stamped.header.frame_id == ''
