# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Republish an unstamped Twist as a TwistStamped."""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def stamp_twist(msg: Twist, stamp, frame_id: str = '') -> TwistStamped:
    """Wrap `msg` in a TwistStamped with the given header stamp/frame."""
    stamped = TwistStamped()
    stamped.header.stamp = stamp
    stamped.header.frame_id = frame_id
    stamped.twist = msg
    return stamped


class TwistStamper(Node):
    """Republishes an unstamped Twist on cmd_vel_in as a TwistStamped on cmd_vel_out."""

    def __init__(self):
        super().__init__('twist_stamper')
        self._pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self._sub = self.create_subscription(Twist, 'cmd_vel_in', self._on_twist, 10)

    def _on_twist(self, msg):
        self._pub.publish(stamp_twist(msg, self.get_clock().now().to_msg()))


def main(args=None):
    """Run the twist_stamper node until shutdown."""
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
