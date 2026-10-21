# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Challenge 2 params checker -- watches the nav2_params YAML on disk.

Publishes ``/mission/challenge_2/params_fixed`` (latched True) once both
costmaps describe the rover at a believable size and the inflation buffer
no longer seals the 0.8 m corridor. The orchestrator then restarts Nav2
with the answer-key params, and Challenge 2 itself passes when the rover
navigates to the leak (see ``goal_reached_node``).

Default thresholds: robot half-width (``robot_radius``, or a polygon
footprint's largest ``|y|``) within [0.2, 0.35] m -- the rover is ~0.43 m
wide -- and ``inflation_radius <= 0.35`` m. The broken starting values
(0.85 / 0.75) fail; the hinted answer (0.28 / 0.25) and the answer key's
polygon footprint pass. Malformed files are reported as ``invalid`` on
``/mission/challenge_2/params_status``.
"""

import rclpy
from rclpy.executors import ExternalShutdownException

from .config_validation import check_nav2_params, CheckResult
from .file_watch import ParamsFileChecker


class Nav2ParamsChecker(ParamsFileChecker):
    """Report when the nav2 params file opens the corridor."""

    def __init__(self):
        super().__init__(
            'nav2_params_checker',
            'nav2_params_file',
            '/mission/challenge_2/params_fixed',
            '/mission/challenge_2/params_status',
        )
        self.declare_parameter('max_half_width', 0.35)
        self.declare_parameter('min_half_width', 0.2)
        self.declare_parameter('max_inflation_radius', 0.35)
        self.max_half_width = float(self.get_parameter('max_half_width').value)
        self.min_half_width = float(self.get_parameter('min_half_width').value)
        self.max_inflation_radius = float(
            self.get_parameter('max_inflation_radius').value
        )

    def check(self, doc) -> CheckResult:
        """Check a parsed nav2 params document."""
        return check_nav2_params(
            doc, self.max_half_width, self.min_half_width, self.max_inflation_radius
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ParamsChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
