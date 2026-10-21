# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Challenge 1 checker -- watches the slam_params YAML on disk.

The simplest viable gate: pass when the user has saved a slam_params file
with ``resolution <= 0.1 m`` and ``max_laser_range >= 8 m``. The thresholds
are loose enough that any reasonable answer (the script suggests 0.05 and
12.0) passes, but the obviously-broken starting values (0.30 and 3.0) do
not. A malformed file (bad YAML, quoted numbers, missing keys, the file
vanishing mid-save) is reported as ``invalid`` on
``/mission/challenge_1/status`` and never crashes the node.

A future iteration can replace the YAML-watch with a real /map quality
analysis (e.g. wall-pixel density along a known corridor segment), but the
YAML check is robust enough to ship Mission Zero on.
"""

import rclpy
from rclpy.executors import ExternalShutdownException

from .config_validation import check_slam_params, CheckResult
from .config_validation import evaluate_slam_params  # noqa: F401 (re-export)
from .file_watch import ParamsFileChecker


class MapQualityChecker(ParamsFileChecker):
    """Pass Challenge 1 once the slam params file has sane map settings."""

    def __init__(self):
        super().__init__(
            'map_quality_checker',
            'slam_params_file',
            '/mission/challenge_1/passed',
            '/mission/challenge_1/status',
        )
        self.declare_parameter('min_resolution_inverse', 10.0)  # res <= 0.1
        self.declare_parameter('min_max_laser_range', 8.0)
        self.min_resolution_inverse = float(
            self.get_parameter('min_resolution_inverse').value
        )
        self.min_max_laser_range = float(
            self.get_parameter('min_max_laser_range').value
        )

    def check(self, doc) -> CheckResult:
        """Check a parsed slam params document."""
        return check_slam_params(
            doc, self.min_resolution_inverse, self.min_max_laser_range
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapQualityChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
