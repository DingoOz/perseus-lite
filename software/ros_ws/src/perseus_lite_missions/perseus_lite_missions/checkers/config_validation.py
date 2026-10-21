# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Pure validation of the student-edited mission YAML files (no rclpy).

The checker nodes watch files a student is editing live, so every input is
hostile: half-saved YAML, a list where a mapping should be, a quoted number
("0.05"), an emptied-out ``ros__parameters`` block, or the file vanishing
between ``stat`` and ``open``. Every function here turns those into a
``CheckResult`` with status ``invalid`` instead of raising, so a checker can
never be killed by what the student types.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

PASS = 'pass'
FAIL = 'fail'
INVALID = 'invalid'


@dataclass(frozen=True)
class CheckResult:
    """Outcome of checking one config document."""

    status: str
    reason: str

    @property
    def passed(self) -> bool:
        """Return True when the config passes the challenge."""
        return self.status == PASS


class ConfigError(ValueError):
    """Raised internally when a config document has the wrong shape."""


def load_yaml_file(path) -> tuple:
    """
    Read and parse a YAML file without ever raising.

    Returns ``(doc, None)`` on success or ``(None, message)`` when the file
    is missing, unreadable, not UTF-8, or not valid YAML.
    """
    try:
        text = Path(path).read_text(encoding='utf-8')
    except (OSError, UnicodeDecodeError) as e:
        return None, f'cannot read {path}: {e}'
    try:
        return yaml.safe_load(text), None
    except yaml.YAMLError as e:
        return None, f'YAML syntax error: {e}'


def get_mapping(doc, *keys) -> dict:
    """Walk nested mappings by key, raising ConfigError on any wrong shape."""
    node = doc
    walked = []
    for key in keys:
        walked.append(key)
        if not isinstance(node, dict):
            raise ConfigError(
                f'expected a mapping above {".".join(walked)!r}, '
                f'found {type(node).__name__}'
            )
        if key not in node:
            raise ConfigError(f'missing {".".join(walked)!r}')
        node = node[key]
    if not isinstance(node, dict):
        raise ConfigError(
            f'{".".join(walked)!r} must be a mapping, found {type(node).__name__}'
        )
    return node


def as_number(value, name: str) -> float:
    """
    Coerce a YAML scalar to a finite float, or raise ConfigError.

    Quoted numbers are rejected on purpose: ROS refuses a string where a
    double parameter is expected, so ``"0.05"`` would crash the real node.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(
            f'{name} must be a number, found {type(value).__name__} {value!r}'
        )
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(f'{name} must be finite, found {value!r}')
    return number


def check_slam_params(
    doc, min_resolution_inverse: float, min_max_laser_range: float
) -> CheckResult:
    """Check a parsed slam_toolbox params document against the thresholds."""
    try:
        params = get_mapping(doc, 'slam_toolbox', 'ros__parameters')
        if 'resolution' not in params or 'max_laser_range' not in params:
            raise ConfigError('resolution and max_laser_range must both be set')
        resolution = as_number(params['resolution'], 'resolution')
        max_range = as_number(params['max_laser_range'], 'max_laser_range')
    except ConfigError as e:
        return CheckResult(INVALID, str(e))
    if resolution <= 0.0:
        return CheckResult(INVALID, f'resolution must be > 0, found {resolution}')
    if (1.0 / resolution) < min_resolution_inverse:
        return CheckResult(
            FAIL,
            f'resolution {resolution} m is too coarse '
            f'(need <= {1.0 / min_resolution_inverse:g} m)',
        )
    if max_range < min_max_laser_range:
        return CheckResult(
            FAIL,
            f'max_laser_range {max_range} m is too short '
            f'(need >= {min_max_laser_range:g} m)',
        )
    return CheckResult(PASS, 'map params look right')


def evaluate_slam_params(
    doc, min_resolution_inverse: float, min_max_laser_range: float
) -> bool:
    """Return True when a slam_toolbox params document passes Challenge 1."""
    return check_slam_params(doc, min_resolution_inverse, min_max_laser_range).passed


def parse_footprint(value):
    """
    Parse a Nav2 ``footprint`` value into ``[(x, y), ...]``.

    Returns None when no polygon footprint is set (absent or ``"[]"``), in
    which case Nav2 falls back to ``robot_radius``. Raises ConfigError for a
    malformed polygon.
    """
    if value is None:
        return None
    if isinstance(value, str):
        try:
            value = yaml.safe_load(value)
        except yaml.YAMLError as e:
            raise ConfigError(f'footprint is not a valid list: {e}') from e
    if value in (None, []):
        return None
    if not isinstance(value, list) or len(value) < 3:
        raise ConfigError('footprint must be a list of at least 3 [x, y] points')
    points = []
    for point in value:
        if not isinstance(point, list) or len(point) != 2:
            raise ConfigError(f'footprint point {point!r} is not an [x, y] pair')
        points.append(
            (as_number(point[0], 'footprint x'), as_number(point[1], 'footprint y'))
        )
    return points


def _inflation_radius(params: dict, costmap: str) -> float:
    plugins = params.get('plugins')
    if not isinstance(plugins, list):
        raise ConfigError(f'{costmap}: plugins must be a list')
    for name in plugins:
        layer = params.get(name)
        if (
            isinstance(layer, dict)
            and layer.get('plugin') == 'nav2_costmap_2d::InflationLayer'
        ):
            if 'inflation_radius' not in layer:
                raise ConfigError(f'{costmap}: {name}.inflation_radius is not set')
            return as_number(
                layer['inflation_radius'], f'{costmap} {name}.inflation_radius'
            )
    raise ConfigError(f'{costmap}: no nav2_costmap_2d::InflationLayer in plugins')


def check_nav2_params(
    doc,
    max_half_width: float,
    min_half_width: float,
    max_inflation_radius: float,
) -> CheckResult:
    """
    Check both Nav2 costmaps' robot size and inflation for Challenge 2.

    The robot's size counts as its lateral half-width: ``robot_radius`` when
    no polygon footprint is set, otherwise the polygon's largest ``|y|``
    (the corridor runs along the rover's x axis). ``min_half_width`` stops a
    "shrink the robot to nothing" non-answer.
    """
    try:
        for costmap in ('local_costmap', 'global_costmap'):
            params = get_mapping(doc, costmap, costmap, 'ros__parameters')
            polygon = parse_footprint(params.get('footprint'))
            if polygon is not None:
                half_width = max(abs(y) for _, y in polygon)
                size_name = 'footprint half-width'
            elif 'robot_radius' in params:
                half_width = as_number(
                    params['robot_radius'], f'{costmap} robot_radius'
                )
                size_name = 'robot_radius'
            else:
                raise ConfigError(f'{costmap}: set robot_radius or footprint')
            inflation = _inflation_radius(params, costmap)
            if half_width > max_half_width:
                return CheckResult(
                    FAIL,
                    f'{costmap} {size_name} {half_width:g} m is larger than '
                    f'the rover (need <= {max_half_width:g} m)',
                )
            if half_width < min_half_width:
                return CheckResult(
                    FAIL,
                    f'{costmap} {size_name} {half_width:g} m is smaller than '
                    f'the rover really is (need >= {min_half_width:g} m)',
                )
            if inflation < 0.0:
                raise ConfigError(f'{costmap} inflation_radius must be >= 0')
            if inflation > max_inflation_radius:
                return CheckResult(
                    FAIL,
                    f'{costmap} inflation_radius {inflation:g} m still seals '
                    f'the corridor (need <= {max_inflation_radius:g} m)',
                )
    except ConfigError as e:
        return CheckResult(INVALID, str(e))
    return CheckResult(PASS, 'costmaps leave a drivable channel')
