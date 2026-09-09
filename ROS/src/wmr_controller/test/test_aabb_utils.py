from types import SimpleNamespace

import pytest

from wmr_controller.aabb_utils import (
    aabb_map_from_messages,
    compare_aabb_maps,
    planner_obstacles,
    validate_aabb,
)


def test_validate_aabb():
    assert validate_aabb([0, 1, 2, 3]) == (0.0, 1.0, 2.0, 3.0)


@pytest.mark.parametrize(
    "box",
    ([0, 0, 0, 1], [0, 1, 1, 0], [0, 0, float("nan"), 1]),
)
def test_validate_aabb_rejects_invalid_boxes(box):
    with pytest.raises(ValueError):
        validate_aabb(box)


def test_message_conversion_and_planner_order():
    messages = [
        SimpleNamespace(id="b", min_x=2, min_y=2, max_x=3, max_y=3),
        SimpleNamespace(id="a", min_x=0, min_y=0, max_x=1, max_y=1),
    ]
    assert planner_obstacles(aabb_map_from_messages(messages)) == [
        [0.0, 0.0, 1.0, 1.0],
        [2.0, 2.0, 3.0, 3.0],
    ]


def test_compare_snapshots():
    old = {"removed": (0, 0, 1, 1), "moved": (2, 2, 3, 3)}
    new = {"added": (-1, -1, 0, 0), "moved": (2.1, 2, 3.1, 3)}
    changes = compare_aabb_maps(old, new, tolerance=0.01)
    assert changes.added == ("added",)
    assert changes.removed == ("removed",)
    assert changes.moved == ("moved",)
