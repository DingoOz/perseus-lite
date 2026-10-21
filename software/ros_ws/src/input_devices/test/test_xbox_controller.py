# Copyright (c) 2024 The Remote Offworld Autonomous Robotics Team
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the Xbox controller node's Joy handling."""

from geometry_msgs.msg import TwistStamped
from input_devices.xbox_controller import axis_or_default, XboxController
import pytest
import rclpy
from sensor_msgs.msg import Joy


def test_axis_or_default_bounds():
    axes = [0.1, -0.2]
    assert axis_or_default(axes, 1, 9.0) == -0.2
    assert axis_or_default(axes, 2, 9.0) == 9.0
    assert axis_or_default(axes, -1, 9.0) == 9.0
    assert axis_or_default([], 0, 1.0) == 1.0


@pytest.fixture
def controller():
    rclpy.init()
    node = XboxController()
    published = []
    node.velocity_publisher.publish = published.append
    yield node, published
    node.destroy_node()
    rclpy.shutdown()


def test_short_joy_message_does_not_raise(controller):
    node, published = controller
    # Fewer axes than the trigger indexes: used to raise IndexError in the
    # callback and kill the node.
    node.process_joystick_input(Joy(axes=[0.0, 1.0], buttons=[]))
    assert len(published) == 1
    msg = published[0]
    assert isinstance(msg, TwistStamped)
    assert msg.twist.linear.x == 0.0  # missing deadman reads as released


def test_deadman_enables_motion(controller):
    node, published = controller
    axes = [0.0, 1.0, 0.0, 0.0, -1.0, 1.0]  # right trigger fully pressed
    node.process_joystick_input(Joy(axes=axes, buttons=[]))
    assert published[-1].twist.linear.x == pytest.approx(node.translation_scale)
