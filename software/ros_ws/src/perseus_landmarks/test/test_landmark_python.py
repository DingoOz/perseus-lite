"""Tests for the ROS-free Python helpers of perseus_landmarks."""

import math

from perseus_landmarks.landmark_map import parse_landmark_map, SearchPoint
from perseus_landmarks.search_plan import rotation_steps, start_relative_to_map
import pytest

MAP = """
site_frame: yard
tags:
  - {id: 0, pose: [2.0, 0.0, 0.25, 180]}
  - {id: 4, pose: [0.0, 3.0, 0.25, -90]}
destinations:
  - {name: dock, pose: [1.0, 2.0, 90], tag: 4}
  - {name: gate, pose: [-1.0, 0.0, 0]}
search_points:
  - {xy: [1.0, 0.0]}
  - {xy: [0.0, 2.0], frame: site}
"""


def test_parses_destinations_and_search_points():
    landmarks = parse_landmark_map(MAP)
    assert landmarks.site_frame == "yard"
    assert landmarks.tag_ids == [0, 4]
    assert landmarks.destinations["dock"].tag == 4
    assert landmarks.destinations["dock"].yaw == pytest.approx(math.pi / 2)
    assert landmarks.destinations["gate"].tag is None
    assert [p.frame for p in landmarks.search_points] == ["start", "site"]


@pytest.mark.parametrize(
    "bad",
    [
        "tags: [{id: 1, pose: [0,0,0,0]}, {id: 1, pose: [0,0,0,0]}]",
        "destinations: [{name: a, pose: [0, 0]}]",
        "destinations: [{name: a, pose: [0, 0, 0], tag: 3}]",
        "search_points: [{xy: [0, 0], frame: moon}]",
        "- just a list",
    ],
)
def test_rejects_bad_maps(bad):
    with pytest.raises(ValueError):
        parse_landmark_map(bad)


def test_rotation_steps_cover_the_rotation_evenly():
    steps = rotation_steps(2 * math.pi, math.radians(45))
    assert len(steps) == 8
    assert sum(steps) == pytest.approx(2 * math.pi)
    assert rotation_steps(math.radians(100), math.radians(45)) == pytest.approx(
        [math.radians(100) / 3] * 3
    )
    assert rotation_steps(0.0, 1.0) == []


def test_start_relative_points_follow_the_start_pose():
    x, y = start_relative_to_map(SearchPoint(1.0, 0.0), 2.0, 3.0, math.pi / 2)
    assert (x, y) == pytest.approx((2.0, 4.0))
    x, y = start_relative_to_map(SearchPoint(0.0, 1.0), 0.0, 0.0, 0.0)
    assert (x, y) == pytest.approx((0.0, 1.0))
