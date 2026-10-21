# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Selene Base corridor must be passable with the answer key, not the puzzle.

Regression test for the review finding that debris_2 left only ~0.5 m of
the 0.8 m corridor clear, so Challenge 2 was impossible even with the
reference params. Parses selene_base.sdf, projects each obstacle box (with
its full roll/pitch/yaw) onto the ground plane, and checks the narrowest
clear width along the corridor against the rover's real footprint and the
Nav2 planner's keep-out distance.
"""

import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from perseus_lite_missions.checkers.config_validation import (
    get_mapping,
    load_yaml_file,
    parse_footprint,
)
import pytest

PKG = Path(__file__).resolve().parents[1]
WORLD = PKG.parent / 'perseus_lite_simulation' / 'worlds' / 'selene_base.sdf'
CONFIGS = PKG / 'missions' / 'm00-cold-open' / 'configs'

CORRIDOR_X = (3.0, 10.0)
CORRIDOR_HALF_WIDTH = 0.4
LIDAR_PLANE_Z = 0.15  # c1_lidar_frame height above ground (lite URDF)
PHYSICAL_MARGIN = 0.1  # required body clearance on each side
MIN_PLANNER_CHANNEL = 0.15  # width of valid base_link centres (3 cells)

pytestmark = pytest.mark.skipif(
    not WORLD.is_file(), reason='perseus_lite_simulation source not alongside'
)


def _rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _obstacles():
    """Yield (name, corners) for every box collision in the habitat."""
    root = ET.parse(WORLD).getroot()
    habitat = root.find(".//model[@name='habitat']")
    for link in habitat.findall('link'):
        size = link.find('collision/geometry/box/size')
        if size is None:
            continue
        x, y, z, roll, pitch, yaw = (float(v) for v in link.find('pose').text.split())
        half = [float(v) / 2 for v in size.text.split()]
        rot = _rot(roll, pitch, yaw)
        corners = []
        for signs in itertools.product((-1, 1), repeat=3):
            local = [s * h for s, h in zip(signs, half)]
            corners.append(
                tuple(
                    c + sum(rot[i][j] * local[j] for j in range(3))
                    for i, c in enumerate((x, y, z))
                )
            )
        yield link.get('name'), corners


def _intrusions():
    """Return [(x_min, x_max, y_low_limit, y_high_limit)] inside the corridor."""
    spans = []
    for name, corners in _obstacles():
        if name.startswith('corridor_') or name.startswith('wall_'):
            continue
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        zs = [c[2] for c in corners]
        if min(zs) > 0.3:  # overhead (collapsed roof panels): rover is ~0.25 m
            continue
        if max(xs) < CORRIDOR_X[0] or min(xs) > CORRIDOR_X[1]:
            continue
        if min(ys) >= CORRIDOR_HALF_WIDTH or max(ys) <= -CORRIDOR_HALF_WIDTH:
            continue
        low, high = -CORRIDOR_HALF_WIDTH, CORRIDOR_HALF_WIDTH
        if (min(ys) + max(ys)) / 2 > 0:
            high = min(ys)
        else:
            low = max(ys)
        spans.append((name, min(xs), max(xs), low, high, min(zs), max(zs)))
    return spans


def _clear_band(x_lo, x_hi):
    low, high = -CORRIDOR_HALF_WIDTH, CORRIDOR_HALF_WIDTH
    for _, xmin, xmax, lo, hi, _, _ in _intrusions():
        if xmax >= x_lo and xmin <= x_hi:
            low, high = max(low, lo), min(high, hi)
    return low, high


def _reference_polygon():
    doc, error = load_yaml_file(CONFIGS / 'nav2_params.reference.yaml')
    assert error is None
    params = get_mapping(doc, 'global_costmap', 'global_costmap', 'ros__parameters')
    polygon = parse_footprint(params.get('footprint'))
    assert polygon, 'answer key should describe the real footprint polygon'
    return polygon


def test_debris_reaches_the_lidar_plane():
    # Debris shorter than the scan plane would be invisible to Nav2.
    for name, *_, zmin, zmax in _intrusions():
        assert zmax > LIDAR_PLANE_Z + 0.03, name


def test_reference_footprint_physically_clears_debris():
    polygon = _reference_polygon()
    length_back = -min(x for x, _ in polygon)
    length_front = max(x for x, _ in polygon)
    width = max(y for _, y in polygon) - min(y for _, y in polygon)
    # Worst case: the whole body alongside the debris field at once.
    for _, xmin, xmax, *_ in _intrusions():
        low, high = _clear_band(xmin - length_front, xmax + length_back)
        clearance = (high - low - width) / 2
        assert clearance >= PHYSICAL_MARGIN, (xmin, xmax, low, high, clearance)


def test_reference_planner_channel_is_open():
    polygon = _reference_polygon()
    # Nav2's inscribed radius: distance from base_link to the nearest edge
    # of the (axis-aligned rectangular) footprint.
    inscribed = min(
        max(x for x, _ in polygon),
        -min(x for x, _ in polygon),
        max(y for _, y in polygon),
        -min(y for _, y in polygon),
    )
    for x in [CORRIDOR_X[0] + 0.05 * i for i in range(int(7.0 / 0.05) + 1)]:
        low, high = _clear_band(x, x)
        assert (high - inscribed) - (low + inscribed) >= MIN_PLANNER_CHANNEL, x


def test_hinted_radius_answer_also_opens_the_channel():
    # Hints: robot_radius ~0.28, inflation_radius ~0.25. With inflation below
    # the radius, Nav2 only marks lethal/inscribed out to the inflation
    # radius, so the keep-out distance is min(radius, inflation).
    keep_out = min(0.28, 0.25)
    for x in [CORRIDOR_X[0] + 0.05 * i for i in range(int(7.0 / 0.05) + 1)]:
        low, high = _clear_band(x, x)
        assert (high - keep_out) - (low + keep_out) >= MIN_PLANNER_CHANNEL - 0.05, x


def test_broken_values_still_seal_the_corridor():
    # Puzzle must stay a puzzle: a 0.75 m keep-out (min(0.85, 0.75)) leaves
    # no valid centre anywhere in a 0.8 m corridor.
    keep_out = min(0.85, 0.75)
    assert 2 * keep_out > 2 * CORRIDOR_HALF_WIDTH
