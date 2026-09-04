import math
from types import SimpleNamespace

import pytest

from core.topo_plan import MissionSegment
from planner.image_point_access import (
    POINT_ACCESS_TRIGGER_DISTANCE_PX,
    apply_to_json_segments,
    apply_to_object_mission,
    geometry_length,
)


def _image_point(point_id, raw, snapped=(0.0, 5.0), edge_id="PL_000"):
    return {
        "point_id": point_id,
        "edge_id": edge_id,
        "point_type": "image_detected",
        "pixel_position": list(snapped),
        "detection_result": {
            "raw_coord": list(raw),
            "snapped_coord": list(snapped),
        },
    }


def _mission(points):
    segment = MissionSegment(
        type="inspect",
        edge_id="PL_000",
        geometry=[(0.0, 0.0), (0.0, 10.0)],
        length=10.0,
    )
    task = SimpleNamespace(inspection_points=points)
    return SimpleNamespace(
        segments=[segment],
        task_by_id={"PL_000": task},
        inspect_length=10.0,
        connect_length=0.0,
        total_length=10.0,
        full_path=list(segment.geometry),
    )


def test_trigger_threshold_is_centralized_and_boundary_is_not_accessed():
    assert POINT_ACCESS_TRIGGER_DISTANCE_PX == 5.0
    mission = _mission([_image_point("IP_0001", (5.0, 5.0))])

    assert apply_to_object_mission(mission) == 0
    assert len(mission.segments) == 1


def test_object_mission_adds_explicit_continuous_round_trip_access():
    point = _image_point("IP_0007", (6.0, 5.0))
    mission = _mission([point])

    assert apply_to_object_mission(mission) == 1
    assert [segment.type for segment in mission.segments] == [
        "inspect",
        "connect",
        "inspect",
    ]
    access = mission.segments[1]
    assert access.connect_mode == "free_flight"
    assert access.planner == "euclidean"
    assert access.reason == "point_access"
    assert access.fallback_reason is None
    assert access.inspection_point_id == "IP_0007"
    assert access.physical_id == "PL_000"
    assert access.anchor_coord == pytest.approx((0.0, 5.0))
    assert access.raw_coord == pytest.approx((6.0, 5.0))
    assert access.geometry == pytest.approx([(0.0, 5.0), (6.0, 5.0), (0.0, 5.0)])
    assert access.length == pytest.approx(geometry_length(access.geometry))
    assert point["detection_result"]["route_visit_coord"] == [6.0, 5.0]
    assert mission.connect_length == pytest.approx(12.0)
    assert mission.total_length == pytest.approx(22.0)
    assert all(
        math.dist(left.geometry[-1], right.geometry[0]) <= 1e-6
        for left, right in zip(mission.segments, mission.segments[1:])
    )
    assert apply_to_object_mission(mission) == 0


def test_json_segments_preserve_schema_length_continuity_and_identity():
    points = [
        _image_point("IP_0005", (1.0, 3.0)),
        _image_point("IP_0007", (6.0, 5.0)),
        _image_point("IP_0012", (2.0, 8.0)),
    ]
    segments = [{
        "segment_id": "seg_0001",
        "type": "inspect",
        "edge_id": "PL_000",
        "physical_id": "PL_000",
        "from_edge_id": None,
        "to_edge_id": "PL_000",
        "direction": "forward",
        "geometry_2d": [[0.0, 0.0], [0.0, 10.0]],
        "length": 10.0,
    }]

    output, count = apply_to_json_segments(segments, points)

    assert count == 1
    accesses = [segment for segment in output if segment.get("reason") == "point_access"]
    assert len(accesses) == 1
    access = accesses[0]
    assert access["inspection_point_id"] == "IP_0007"
    assert access["physical_id"] == "PL_000"
    assert access["geometry_2d"][1] == [6.0, 5.0]
    assert access["length"] == pytest.approx(geometry_length(access["geometry_2d"]))
    assert all(
        math.dist(left["geometry_2d"][-1], right["geometry_2d"][0]) <= 1e-6
        for left, right in zip(output, output[1:])
    )
    assert points[1]["segment_id"] == access["segment_id"]
    assert points[0].get("segment_id") is None
    assert points[2].get("segment_id") is None
    again, again_count = apply_to_json_segments(output, points)
    assert again_count == 0
    assert again == output
