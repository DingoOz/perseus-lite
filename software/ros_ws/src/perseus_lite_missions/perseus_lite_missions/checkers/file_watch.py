# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Shared base for checkers that watch a student-edited params file on disk.

Each tick it re-reads the file when its (mtime, size) changed, runs the
subclass's pure ``check(doc)`` and publishes:

* ``<passed_topic>`` (std_msgs/Bool) every tick -- True once passed (latched);
* ``<status_topic>`` (std_msgs/String, JSON) on every state change:
  ``{"status": "pass|fail|invalid|missing", "reason": "...", "file": ...}``.

Nothing the student writes can raise out of ``tick``: read/parse errors and
wrong-shaped documents become an ``invalid`` status, and an unexpected
exception from ``check`` is logged and reported the same way.
"""

import json
import os
from pathlib import Path

from rclpy.node import Node
from std_msgs.msg import Bool, String

from .config_validation import CheckResult, INVALID, load_yaml_file

MISSING = 'missing'


class ParamsFileChecker(Node):
    """Watch one YAML file and publish whether it passes ``check``."""

    def __init__(
        self, node_name: str, file_param: str, passed_topic: str, status_topic: str
    ):
        super().__init__(node_name)
        self.declare_parameter(file_param, '')
        self.params_path = Path(self.get_parameter(file_param).value)
        self.passed = False
        self._signature = None
        self._last_status = None
        self.pub = self.create_publisher(Bool, passed_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(1.0, self.tick)

    def check(self, doc) -> CheckResult:  # pragma: no cover - overridden
        """Return the CheckResult for a parsed document."""
        raise NotImplementedError

    def _report(self, result: CheckResult) -> None:
        key = (result.status, result.reason)
        if key == self._last_status:
            return
        self._last_status = key
        msg = String()
        msg.data = json.dumps(
            {
                'status': result.status,
                'reason': result.reason,
                'file': str(self.params_path),
            }
        )
        self.status_pub.publish(msg)
        log = self.get_logger()
        if result.status == INVALID:
            log.warning(f'{self.params_path.name}: invalid config: {result.reason}')
        elif result.status == MISSING:
            log.warning(result.reason)
        else:
            log.info(f'{self.params_path.name}: {result.status}: {result.reason}')

    def tick(self) -> None:
        """Re-check the file if it changed and publish the result."""
        try:
            self._tick()
        except Exception as e:  # noqa: B902 - a checker must never die
            self.get_logger().error(
                f'unexpected checker error: {e!r}', throttle_duration_sec=10.0
            )
            self._report(CheckResult(INVALID, f'checker error: {e!r}'))
        self.pub.publish(Bool(data=self.passed))

    def _tick(self) -> None:
        if self.passed:
            return
        try:
            st = os.stat(self.params_path)
        except OSError:
            self._signature = None
            self._report(
                CheckResult(MISSING, f'params file not found at {self.params_path}')
            )
            return
        signature = (st.st_mtime_ns, st.st_size)
        if signature == self._signature:
            return
        self._signature = signature
        doc, error = load_yaml_file(self.params_path)
        result = CheckResult(INVALID, error) if error else self.check(doc)
        self._report(result)
        if result.passed:
            self.passed = True
