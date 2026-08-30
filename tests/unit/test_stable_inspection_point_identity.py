from types import SimpleNamespace

import pytest

from core.topo_plan import _stable_export_point_id
from planner.mission_result_builder import (
    _inspection_points_from_json,
    _normalize_dashboard_inspection_point_id,
    enrich_inspection_points_for_dashboard,
)


def test_image_point_export_uses_original_stable_identity():
    point = SimpleNamespace(id="IP_0012", original_point_id="IP_0012")
    assert _stable_export_point_id(point, image_point=True, legacy_counter=1) == "IP_0012"


def test_image_point_without_identity_fails_explicitly():
    point = SimpleNamespace(id=None)
    with pytest.raises(ValueError, match="identity_missing"):
        _stable_export_point_id(point, image_point=True, legacy_counter=1)


def test_existing_mission_identity_is_preserved_verbatim():
    assert _normalize_dashboard_inspection_point_id({"point_id": "IP_00001"}, 11) == "IP_00001"
    assert _normalize_dashboard_inspection_point_id({"point_id": "IP_0012"}, 0) == "IP_0012"
    assert _normalize_dashboard_inspection_point_id({"point_id": "stable-point-A"}, 0) == "stable-point-A"


def test_non_image_legacy_point_keeps_compatibility_fallback():
    point = SimpleNamespace(id=None)
    assert _stable_export_point_id(point, image_point=False, legacy_counter=7) == "IP_00007"


def test_dashboard_rejects_missing_image_identity():
    with pytest.raises(ValueError, match="identity_missing"):
        _normalize_dashboard_inspection_point_id(
            {"point_type": "image_detected"}, 0
        )


def test_dashboard_rejects_duplicate_identity_for_different_raw_points():
    points = [
        {"point_id": "IP_0012", "x": 1, "y": 2, "raw_x": 1, "raw_y": 2},
        {"point_id": "IP_0012", "x": 3, "y": 4, "raw_x": 3, "raw_y": 4},
    ]
    with pytest.raises(ValueError, match="different raw coordinates"):
        enrich_inspection_points_for_dashboard(points, [])


def test_dashboard_preserves_mission_identity_strings_and_order():
    mission = {
        "inspection_points": [
            {
                "point_id": "IP_00012",
                "x": 12,
                "y": 20,
                "detection_result": {"raw_coord": [12, 20]},
            },
            {
                "point_id": "IP_00001",
                "x": 1,
                "y": 2,
                "detection_result": {"raw_coord": [1, 2]},
            },
        ],
        "image_inspection_overlay": [
            {"id": "IP_0012", "raw_coord": [999, 999]},
        ],
    }

    dashboard_points = _inspection_points_from_json(mission)

    assert [point["point_id"] for point in dashboard_points] == [
        "IP_00012",
        "IP_00001",
    ]
    assert [point["id"] for point in dashboard_points] == [
        "IP_00012",
        "IP_00001",
    ]
    assert "IP_0012" not in {point["point_id"] for point in dashboard_points}
    assert dashboard_points[0]["image_url"] == "/api/inspection-image/IP_00012.jpg"
