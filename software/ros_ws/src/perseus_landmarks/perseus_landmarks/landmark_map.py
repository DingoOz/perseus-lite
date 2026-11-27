"""
Landmark map loading for the Python side (same YAML as the C++ landmark_map.cpp).

Only what the navigator needs is parsed: destinations, the tags they refer to, and
search points. The C++ parser is the strict one (the localiser refuses to start on
a bad file); this one raises ValueError with a readable message on the same errors.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional

import yaml


@dataclass
class Destination:
    """A named place in the site frame."""

    name: str
    x: float
    y: float
    yaw: float
    tag: Optional[int] = None


@dataclass
class SearchPoint:
    """A place to look from: 'start' (robot-relative at search start) or 'site'."""

    x: float
    y: float
    frame: str = "start"


@dataclass
class LandmarkMap:
    """The parts of the landmark map the navigator uses."""

    site_frame: str = "site"
    tag_ids: List[int] = field(default_factory=list)
    destinations: Dict[str, Destination] = field(default_factory=dict)
    search_points: List[SearchPoint] = field(default_factory=list)


def parse_landmark_map(text: str) -> LandmarkMap:
    """Parse landmark-map YAML text."""
    try:
        root = yaml.safe_load(text) or {}
    except yaml.YAMLError as error:
        raise ValueError(f"landmark map is not valid YAML: {error}") from error
    if not isinstance(root, dict):
        raise ValueError("landmark map must be a YAML mapping")
    result = LandmarkMap(site_frame=str(root.get("site_frame", "site")))
    for tag in root.get("tags") or []:
        tag_id = int(tag["id"])
        if tag_id in result.tag_ids:
            raise ValueError(f"tag {tag_id} is listed twice")
        result.tag_ids.append(tag_id)
    for entry in root.get("destinations") or []:
        name = str(entry["name"])
        pose = entry.get("pose")
        if not isinstance(pose, list) or len(pose) != 3:
            raise ValueError(f"destination '{name}' pose must be [x, y, yaw_deg]")
        tag = entry.get("tag")
        if tag is not None and int(tag) not in result.tag_ids:
            raise ValueError(f"destination '{name}' refers to unknown tag {tag}")
        if name in result.destinations:
            raise ValueError(f"destination '{name}' is listed twice")
        result.destinations[name] = Destination(
            name,
            float(pose[0]),
            float(pose[1]),
            math.radians(float(pose[2])),
            None if tag is None else int(tag),
        )
    for entry in root.get("search_points") or []:
        xy = entry.get("xy")
        if not isinstance(xy, list) or len(xy) != 2:
            raise ValueError("search point xy must be [x, y]")
        frame = str(entry.get("frame", "start"))
        if frame not in ("start", "site"):
            raise ValueError(
                f"search point frame must be 'start' or 'site', got '{frame}'"
            )
        result.search_points.append(SearchPoint(float(xy[0]), float(xy[1]), frame))
    return result


def load_landmark_map(path: str) -> LandmarkMap:
    """Load a landmark map file."""
    with open(path, encoding="utf-8") as handle:
        return parse_landmark_map(handle.read())
