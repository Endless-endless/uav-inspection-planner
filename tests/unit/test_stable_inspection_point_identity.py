from types import SimpleNamespace

import pytest

from core.topo_plan import _stable_export_point_id
from planner.mission_result_builder import (
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


def test_legacy_five_digit_id_is_only_format_normalized():
    assert _normalize_dashboard_inspection_point_id({"point_id": "IP_00001"}, 11) == "IP_0001"
    assert _normalize_dashboard_inspection_point_id({"point_id": "IP_0012"}, 0) == "IP_0012"


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
