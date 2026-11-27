"""Pure planning helpers for the landmark search (no ROS)."""

import math
from typing import List, Tuple

from perseus_landmarks.landmark_map import SearchPoint


def rotation_steps(max_rotation_rad: float, step_rad: float) -> List[float]:
    """
    Split a total rotation into spin steps of at most step_rad.

    The robot stops after each step to look (stop-and-look: sharp images, and the
    localiser only trusts sightings while the robot is still).
    """
    if max_rotation_rad <= 0.0 or step_rad <= 0.0:
        return []
    count = max(1, math.ceil(max_rotation_rad / step_rad - 1e-9))
    return [max_rotation_rad / count] * count


def start_relative_to_map(
    point: SearchPoint, start_x: float, start_y: float, start_yaw: float
) -> Tuple[float, float]:
    """Place a 'start'-frame search point (x forward, y left of the start pose) in map."""
    c = math.cos(start_yaw)
    s = math.sin(start_yaw)
    return (start_x + c * point.x - s * point.y, start_y + s * point.x + c * point.y)


def facing_away_yaw(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Heading from one point to another (used to arrive facing along the search leg)."""
    return math.atan2(to_y - from_y, to_x - from_x)
