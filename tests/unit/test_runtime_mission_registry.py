from __future__ import annotations

import copy

import pytest

from perception.mission_identity import MissionSnapshot
from perception.mission_registry import (
    RuntimeMissionIdentityMismatchError,
    RuntimeMissionRegistry,
    RuntimeMissionUnavailableError,
)


def mission(start: int) -> dict:
    return {
        "inspection_points": [
            {"point_id": "IP_0001"},
            {"point_id": "IP_0012"},
        ],
        "segments": [{"segment_id": "seg", "geometry_2d": [[start, 0], [2, 0]]}],
    }


def test_same_content_has_stable_identity_and_registration_is_non_mutating():
    source = mission(0)
    before = copy.deepcopy(source)
    first = MissionSnapshot.from_dict(source)
    second = MissionSnapshot.from_dict(copy.deepcopy(source))
    registry = RuntimeMissionRegistry()

    registered_first = registry.register(source)
    registered_second = registry.register(copy.deepcopy(source))

    assert first.mission_id == second.mission_id == registered_first.mission_id
    assert first.mission_sha256 == second.mission_sha256 == registered_first.mission_sha256
    assert registered_second.mission_id == registered_first.mission_id
    assert source == before


def test_different_a_b_with_same_point_ids_have_distinct_bindings():
    registry = RuntimeMissionRegistry()
    a = registry.register(mission(0))
    b = registry.register(mission(1))

    assert a.authoritative_inspection_point_ids == b.authoritative_inspection_point_ids
    assert a.mission_sha256 != b.mission_sha256
    assert a.mission_id != b.mission_id
    assert registry.get(b.mission_id, b.mission_sha256).mission == b.mission

    with pytest.raises(RuntimeMissionIdentityMismatchError):
        registry.get(b.mission_id, a.mission_sha256)
    with pytest.raises(RuntimeMissionUnavailableError):
        registry.get("mission_missing", b.mission_sha256)


def test_registry_clear_never_falls_back_to_another_mission():
    registry = RuntimeMissionRegistry()
    snapshot = registry.register(mission(0))
    registry.clear()

    with pytest.raises(RuntimeMissionUnavailableError):
        registry.get(snapshot.mission_id, snapshot.mission_sha256)


def test_dashboard_metadata_identifies_the_exact_registered_mission(monkeypatch):
    import app

    registry = RuntimeMissionRegistry()
    monkeypatch.setattr(app, "_runtime_mission_registry", registry)
    mission_a = mission(0)
    mission_b = mission(1)
    dashboard_a = {"metadata": {"pipeline": "image"}}
    dashboard_b = {"metadata": {"pipeline": "image"}}

    snapshot_a = registry.register(mission_a)
    app._attach_runtime_mission_identity(dashboard_a, snapshot_a)
    snapshot_b = registry.register(mission_b)
    app._attach_runtime_mission_identity(dashboard_b, snapshot_b)

    assert dashboard_a["metadata"]["mission_id"] == snapshot_a.mission_id
    assert dashboard_a["metadata"]["mission_sha256"] == snapshot_a.mission_sha256
    assert dashboard_b["metadata"]["mission_id"] == snapshot_b.mission_id
    assert dashboard_b["metadata"]["mission_sha256"] == snapshot_b.mission_sha256
    assert snapshot_a.mission_sha256 != snapshot_b.mission_sha256
    assert registry.current().mission_sha256 == snapshot_b.mission_sha256
