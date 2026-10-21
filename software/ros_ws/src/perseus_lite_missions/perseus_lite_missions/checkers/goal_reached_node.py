# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Challenge 2 checker -- reports when Nav2 has driven the rover to the leak.

Reads the leak coords from mission.yaml and looks up ``map -> base_link``
on /tf. Every tick it publishes ``/mission/challenge_2/passed`` with the
*current* condition (not latched): True while the rover is within
``tolerance_m`` of the leak and -- when ``require_nav_goal`` is set (the
default) -- the most recent NavigateToPose goal is executing or succeeded.
The orchestrator decides whether that counts (mission order, params fixed),
so a rover driven there by hand, or parked there before Challenge 1, does
not pass Challenge 2.

A future version could also gate on collision count (no contacts during
the run).
"""

import math
from pathlib import Path

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
import tf2_ros

from .config_validation import load_yaml_file

NAV_OK_STATUSES = (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED)


def latest_goal_ok(statuses) -> bool:
    """
    Return True when the newest goal is executing or has succeeded.

    ``statuses`` is an iterable of ``(stamp_ns, status)`` pairs, as taken
    from a ``GoalStatusArray``.
    """
    latest = max(statuses, key=lambda s: s[0], default=None)
    return latest is not None and latest[1] in NAV_OK_STATUSES


def leak_from_mission(doc):
    """Return the leak (x, y) from a parsed mission document, or None."""
    if not isinstance(doc, dict):
        return None
    poses = doc.get('poses')
    leak = poses.get('leak') if isinstance(poses, dict) else None
    if not isinstance(leak, dict):
        return None
    try:
        x, y = leak['x'], leak['y']
    except KeyError:
        return None
    if isinstance(x, bool) or isinstance(y, bool):
        return None
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    return float(x), float(y)


def within_tolerance(robot_xy, leak_xy, tolerance_m: float) -> bool:
    """Return True when the robot is within ``tolerance_m`` of the leak."""
    return math.hypot(robot_xy[0] - leak_xy[0], robot_xy[1] - leak_xy[1]) <= (
        tolerance_m
    )


class GoalReachedChecker(Node):
    """Publish whether the rover is currently at the leak under Nav2."""

    def __init__(self):
        super().__init__('goal_reached_checker')

        self.declare_parameter('mission_yaml', '')
        self.declare_parameter('tolerance_m', 0.5)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('require_nav_goal', True)
        self.declare_parameter('nav_status_topic', '/navigate_to_pose/_action/status')

        mission_path = Path(self.get_parameter('mission_yaml').value)
        self.tolerance_m = float(self.get_parameter('tolerance_m').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.require_nav_goal = bool(self.get_parameter('require_nav_goal').value)

        doc, error = load_yaml_file(mission_path)
        self.leak_xy = leak_from_mission(doc)
        if self.leak_xy is None:
            self.get_logger().warning(
                f'No leak pose in {mission_path} ({error or "poses.leak missing"}); '
                'checker will idle.'
            )
        else:
            self.get_logger().info(
                f'Leak target: ({self.leak_xy[0]:.2f}, {self.leak_xy[1]:.2f}) '
                f'tolerance {self.tolerance_m:.2f} m'
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._nav_ok = False
        if self.require_nav_goal:
            status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                GoalStatusArray,
                self.get_parameter('nav_status_topic').value,
                self._on_nav_status,
                status_qos,
            )

        self._was_at_leak = False
        self.pub = self.create_publisher(Bool, '/mission/challenge_2/passed', 10)
        self.create_timer(0.5, self.tick)

    def _on_nav_status(self, msg: GoalStatusArray):
        self._nav_ok = latest_goal_ok(
            (
                Time.from_msg(s.goal_info.stamp).nanoseconds,
                s.status,
            )
            for s in msg.status_list
        )

    def tick(self):
        """Publish whether the rover is at the leak right now."""
        at_leak = self._at_leak()
        ok = at_leak and (self._nav_ok or not self.require_nav_goal)
        if at_leak and not ok:
            self.get_logger().warning(
                'Rover is at the leak but no NavigateToPose goal is driving it '
                'there -- send the goal through Nav2 to pass Challenge 2.',
                throttle_duration_sec=10.0,
            )
        if ok and not self._was_at_leak:
            self.get_logger().info('Rover reached the leak under Nav2.')
        self._was_at_leak = ok
        self.pub.publish(Bool(data=ok))

    def _at_leak(self) -> bool:
        if self.leak_xy is None:
            return False
        try:
            tx = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as e:
            self.get_logger().warning(
                f'No {self.map_frame} -> {self.robot_frame} transform yet '
                f'(is slam_toolbox running?): {e}',
                throttle_duration_sec=10.0,
            )
            return False
        robot_xy = (tx.transform.translation.x, tx.transform.translation.y)
        return within_tolerance(robot_xy, self.leak_xy, self.tolerance_m)


def main(args=None):
    rclpy.init(args=args)
    node = GoalReachedChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
