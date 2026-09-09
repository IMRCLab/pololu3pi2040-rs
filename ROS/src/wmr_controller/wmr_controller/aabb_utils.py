"""Reusable helpers for 2-D axis-aligned bounding-box snapshots.

The canonical planner representation is ``[min_x, min_y, max_x, max_y]``.
These helpers intentionally do not depend on DBA* so MPC, RL, tests, and
perception bridges can use the same validation and change-detection rules.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence
import numpy as np


AABB = tuple[float, float, float, float]
AABBMap = dict[str, AABB]


@dataclass(frozen=True)
class AABBChanges:
    added: tuple[str, ...]
    removed: tuple[str, ...]
    moved: tuple[str, ...]

    @property
    def changed(self) -> bool:
        return bool(self.added or self.removed or self.moved)


def validate_aabb(values: Sequence[float]) -> AABB:
    """Return a canonical AABB or raise ValueError for invalid coordinates."""
    if len(values) != 4:
        raise ValueError(f"an AABB needs 4 coordinates, got {len(values)}")

    min_x, min_y, max_x, max_y = (float(value) for value in values)
    box = (min_x, min_y, max_x, max_y)
    if not all(math.isfinite(value) for value in box):
        raise ValueError("AABB coordinates must be finite")
    if min_x >= max_x:
        raise ValueError("AABB requires min_x < max_x")
    if min_y >= max_y:
        raise ValueError("AABB requires min_y < max_y")
    return box


def aabb_map_from_messages(boxes: Iterable[object]) -> AABBMap:
    """Convert AABB2D-like messages to an ID-to-AABB snapshot.

    Any object exposing id/min_x/min_y/max_x/max_y can be used, which keeps the
    helper easy to unit-test without constructing ROS messages.
    """
    result: AABBMap = {}
    for message in boxes:
        obstacle_id = str(message.id).strip()
        if not obstacle_id:
            raise ValueError("AABB ID must not be empty")
        if obstacle_id in result:
            raise ValueError(f"duplicate AABB ID: {obstacle_id}")
        result[obstacle_id] = validate_aabb(
            (message.min_x, message.min_y, message.max_x, message.max_y)
        )
    return result


def make_aabb_thick_box(p1, p2, thickness: float) -> AABB:
    x_min, x_max = min(p1[0], p2[0]), max(p1[0], p2[0])
    y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
    half_thickness = thickness / 2.

    if np.isclose(p1[1], p2[1], atol=0.05):
        y_min -= half_thickness
        y_max += half_thickness
    elif np.isclose(p1[0], p2[0], atol=0.05):
        x_min -= half_thickness
        x_max += half_thickness

    return (x_min, y_min, x_max, y_max)


def compare_aabb_maps(
    old: Mapping[str, Sequence[float]],
    new: Mapping[str, Sequence[float]],
    tolerance: float = 0.0,
) -> AABBChanges:
    """Describe added, removed, and moved boxes between complete snapshots."""
    if tolerance < 0.0:
        raise ValueError("AABB comparison tolerance must be non-negative")

    old_ids = set(old)
    new_ids = set(new)
    moved = []
    for obstacle_id in sorted(old_ids & new_ids):
        old_box = validate_aabb(old[obstacle_id])
        new_box = validate_aabb(new[obstacle_id])
        if any(abs(a - b) > tolerance for a, b in zip(old_box, new_box)):
            moved.append(obstacle_id)

    return AABBChanges(
        added=tuple(sorted(new_ids - old_ids)),
        removed=tuple(sorted(old_ids - new_ids)),
        moved=tuple(moved),
    )


def planner_obstacles(boxes: Mapping[str, Sequence[float]]) -> list[list[float]]:
    """Convert a named snapshot to DBA*/YAML ``[[xmin,ymin,xmax,ymax], ...]``."""
    return [list(validate_aabb(boxes[key])) for key in sorted(boxes)]
