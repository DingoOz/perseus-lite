#!/usr/bin/env python3
# Copyright (c) 2024 The Remote Offworld Autonomous Robotics Team
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Xbox gamepad -> velocity command node (dual deadman, two speed modes)."""

from typing import List, Sequence

from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    SetParametersResult,
)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy


def axis_or_default(axes: Sequence[float], index: int, default: float) -> float:
    """Return axes[index], or `default` if the index is out of range."""
    if 0 <= index < len(axes):
        return axes[index]
    return default


class XboxController(Node):
    """
    Process Xbox controller inputs and publish velocity commands.

    This node subscribes to joystick inputs and converts them into velocity commands
    for robot control. It includes two dead man's switches for different speed modes
    and configurable scaling factors through ROS2 parameters.

    Publishes ``input_devices/cmd_vel`` (geometry_msgs/TwistStamped, or Twist
    if ``use_stamped_msg`` is false) and subscribes to ``joy``
    (sensor_msgs/Joy).

    ROS parameters: ``translation_scale`` (m/s, default 0.25),
    ``rotation_scale`` (rad/s, default 0.50), ``high_speed_multiplier``
    (default 2.0), ``deadman_threshold`` (default -0.95), ``use_stamped_msg``
    (default True; Jazzy's twist_mux/diff_drive_controller expect stamped) and
    ``base_frame`` (default base_link).
    """

    def __init__(self):
        """Initialize the Xbox controller node with configurable parameters."""
        super().__init__('xbox_controller')

        # Xbox controller axis mappings
        self.LEFT_STICK_Y_AXIS = 1
        # NEEDS HARDWARE CHECK: axis 2 matches the wireless layout implied by
        # perseus_input_config's old wireless bucket mapping (2 = right X,
        # 3 = right Y), but commit 8dd3c70a changed the wireless dual-stick
        # turn axis to 3, and wired xpad pads report right X on axis 3. The
        # layout differs by driver/firmware; confirm with `ros2 topic echo
        # /joy` on the robot's pad before changing it.
        self.RIGHT_STICK_X_AXIS = 2
        self.RIGHT_TRIGGER_AXIS = 4  # Regular speed deadman switch
        self.LEFT_TRIGGER_AXIS = 5  # High speed deadman switch

        # Create range constraints as instance variables for use in callback
        self._speed_range = FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
        self._multiplier_range = FloatingPointRange(
            from_value=1.0, to_value=5.0, step=0.1
        )
        self._threshold_range = FloatingPointRange(
            from_value=-1.0, to_value=0.0, step=0.01
        )

        # Declare and load parameters with proper constraints and descriptions
        self._declare_parameters()
        self._load_parameters()

        # Create appropriate publisher based on message type parameter
        if self.use_stamped_msg:
            self.velocity_publisher = self.create_publisher(
                TwistStamped, 'input_devices/cmd_vel', 10
            )
        else:
            self.velocity_publisher = self.create_publisher(
                Twist, 'input_devices/cmd_vel', 10
            )

        self.joy_subscriber = self.create_subscription(
            Joy, 'joy', self.process_joystick_input, 10
        )

        self.get_logger().info(
            f'Xbox controller initialized with translation scale: {self.translation_scale}, '
            f'rotation scale: {self.rotation_scale}, '
            f'high speed multiplier: {self.high_speed_multiplier}, '
            f'using {"stamped" if self.use_stamped_msg else "unstamped"} messages'
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with proper constraints and descriptions."""
        # Parameter declarations
        self.declare_parameter(
            'translation_scale',
            0.25,
            ParameterDescriptor(
                description='Base scaling factor for linear motion in m/s',
                floating_point_range=[self._speed_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            'rotation_scale',
            0.50,
            ParameterDescriptor(
                description='Base scaling factor for angular motion in rad/s',
                floating_point_range=[self._speed_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            'high_speed_multiplier',
            2.0,
            ParameterDescriptor(
                description='Multiplier for high-speed mode',
                floating_point_range=[self._multiplier_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            'deadman_threshold',
            -0.95,
            ParameterDescriptor(
                description="Threshold for dead man's switch activation",
                floating_point_range=[self._threshold_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            'use_stamped_msg',
            True,
            ParameterDescriptor(
                description='Whether to publish TwistStamped instead of Twist messages',
                read_only=True,
            ),
        )

        self.declare_parameter(
            'base_frame',
            'base_link',
            ParameterDescriptor(
                description='Frame id stamped onto TwistStamped messages',
                read_only=True,
            ),
        )
        self.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        """
        Handle parameter updates during runtime.

        Parameters
        ----------
        params
            List of parameters that are being updated.

        Returns
        -------
        SetParametersResult
            Success/failure of the parameter updates.

        """
        for param in params:
            if param.name == 'high_speed_multiplier':
                if (
                    param.value < self._multiplier_range.from_value
                    or param.value > self._multiplier_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason='Multiplier out of valid range'
                    )
                self.high_speed_multiplier = param.value

            elif param.name == 'deadman_threshold':
                if (
                    param.value < self._threshold_range.from_value
                    or param.value > self._threshold_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason='Threshold out of valid range'
                    )
                self.deadman_threshold = param.value

            elif param.name == 'translation_scale':
                if (
                    param.value < self._speed_range.from_value
                    or param.value > self._speed_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason='Translation scale out of valid range'
                    )
                self.translation_scale = param.value

            elif param.name == 'rotation_scale':
                if (
                    param.value < self._speed_range.from_value
                    or param.value > self._speed_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason='Rotation scale out of valid range'
                    )
                self.rotation_scale = param.value

        return SetParametersResult(successful=True)

    def _load_parameters(self) -> None:
        """Load all declared parameters into instance variables."""
        self.translation_scale = self.get_parameter('translation_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.high_speed_multiplier = self.get_parameter('high_speed_multiplier').value
        self.deadman_threshold = self.get_parameter('deadman_threshold').value
        self.use_stamped_msg = self.get_parameter('use_stamped_msg').value
        self.base_frame = self.get_parameter('base_frame').value

    def process_joystick_input(self, joy_msg: Joy) -> None:
        """
        Process incoming joystick messages and publish velocity commands.

        The node supports two speed modes:
        - Regular speed (RIGHT_TRIGGER): Normal operation
        - High speed (LEFT_TRIGGER): Operates at high_speed_multiplier times normal speed

        Parameters
        ----------
        joy_msg
            The incoming joystick message.

        """
        twist = Twist()
        axes = joy_msg.axes
        self._check_axes(len(axes))

        # Check deadman switches. A missing trigger axis reads as released
        # (+1.0, the triggers' rest value) so a short Joy message can never
        # enable driving, nor raise IndexError and kill the node.
        is_regular_speed = (
            axis_or_default(axes, self.RIGHT_TRIGGER_AXIS, 1.0)
            <= self.deadman_threshold
        )
        is_high_speed = (
            axis_or_default(axes, self.LEFT_TRIGGER_AXIS, 1.0) <= self.deadman_threshold
        )

        if is_regular_speed or is_high_speed:
            # Calculate base movement
            # Joystick forward = positive axis value = positive linear.x = forward (ROS convention)
            linear_input = axis_or_default(axes, self.LEFT_STICK_Y_AXIS, 0.0)
            # Joystick left = positive angular.z = turn left (ROS convention)
            rotation_input = axis_or_default(axes, self.RIGHT_STICK_X_AXIS, 0.0)

            # Apply appropriate scaling based on which deadman switch is engaged
            speed_multiplier = self.high_speed_multiplier if is_high_speed else 1.0

            twist.linear.x = linear_input * self.translation_scale * speed_multiplier

            twist.angular.z = rotation_input * self.rotation_scale * speed_multiplier

        # Create and publish appropriate message type
        if self.use_stamped_msg:
            stamped_twist = TwistStamped()
            stamped_twist.header.stamp = self.get_clock().now().to_msg()
            stamped_twist.header.frame_id = self.base_frame
            stamped_twist.twist = twist
            self.velocity_publisher.publish(stamped_twist)
        else:
            self.velocity_publisher.publish(twist)

        self._log_debug_info(joy_msg, twist, is_regular_speed, is_high_speed)

    def _check_axes(self, count: int) -> None:
        """Warn (throttled) when the pad reports fewer axes than mapped."""
        needed = max(
            self.LEFT_STICK_Y_AXIS,
            self.RIGHT_STICK_X_AXIS,
            self.RIGHT_TRIGGER_AXIS,
            self.LEFT_TRIGGER_AXIS,
        )
        if count <= needed:
            self.get_logger().warn(
                f'Joy message has {count} axes but axis {needed} is mapped; '
                'missing axes read as released/zero',
                throttle_duration_sec=5.0,
            )

    def _log_debug_info(
        self, joy_msg: Joy, twist: Twist, is_regular_speed: bool, is_high_speed: bool
    ) -> None:
        """
        Log debug information about controller state and outputs.

        Parameters
        ----------
        joy_msg
            The current joystick message.
        twist
            The calculated velocity command.
        is_regular_speed
            State of the regular speed deadman switch.
        is_high_speed
            State of the high speed deadman switch.

        """
        self.get_logger().debug('Xbox Controller Axes:')
        for idx, axis_value in enumerate(joy_msg.axes):
            self.get_logger().debug(f'  Axis {idx}: {axis_value:.2f}')

        self.get_logger().debug('Velocity Output:')
        self.get_logger().debug(f'  Linear X: {twist.linear.x:.2f} m/s')
        self.get_logger().debug(f'  Angular Z: {twist.angular.z:.2f} rad/s')

        speed_mode = (
            'High Speed'
            if is_high_speed
            else 'Regular Speed'
            if is_regular_speed
            else 'Disabled'
        )
        self.get_logger().debug(f'  Speed Mode: {speed_mode}')

        self.get_logger().debug('------------------------')


def main(args=None):
    """Run the xbox_controller node."""
    rclpy.init(args=args)
    controller = XboxController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
