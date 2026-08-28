import copy
import inspect
import math

import pytest

from tests.helpers.replan_diagnostics import classify_mission_connects, effective_image_points, physical_identity_map, physical_ids, point_identity, polyline_length, segment_gap

def test_baseline_mission_structure(baseline_mission):
    segments = baseline_mission["segments"]
    assert segments and physical_ids(baseline_mission)
    assert all(segment_gap(a, b) <= 0.5 for a, b in zip(segments, segments[1:]))
    assert math.isclose(sum(float(s.get("length", 0)) for s in segments), baseline_mission["statistics"]["total_length"], abs_tol=0.05)
    assert len(effective_image_points(baseline_mission)) == baseline_mission["statistics"]["num_inspection_points"]

@pytest.mark.xfail(strict=True, reason="Baseline seg_0007 declares 42.07 px but geometry measures about 41.88 px")
def test_each_baseline_segment_length_matches_geometry(baseline_mission):
    assert all(
        math.isclose(
            polyline_length(segment.get("geometry_2d") or []),
            float(segment.get("length", 0)),
            abs_tol=0.02,
        )
        for segment in baseline_mission["segments"]
    )

def test_current_connect_classification_is_reported(baseline_mission, record_property):
    counts, evidence = classify_mission_connects(baseline_mission)
    for key in ("topology", "free_flight_between_components", "free_flight_endpoint_access", "unexpected_fallback", "unknown"):
        record_property(f"connect_{key}", counts[key])
    record_property("connect_evidence", repr(evidence))
    print("CONNECT_COUNTS", dict(counts)); print("CONNECT_EVIDENCE", evidence)
    assert sum(counts.values()) == len([s for s in baseline_mission["segments"] if s.get("type") == "connect"])

def test_current_pl_identity_missing_list_is_reported(baseline_mission, record_property):
    missing = sorted(physical_ids(baseline_mission) - set(physical_identity_map(baseline_mission)))
    record_property("missing_pl_identity", ",".join(missing)); print("MISSING_PL_IDENTITY", missing)
    assert missing == [f"PL_{i:03d}" for i in range(8)]

@pytest.mark.xfail(strict=True, reason="Mission omits PL -> chain/topo/line/component mapping")
def test_every_inspect_has_complete_physical_topology_identity(baseline_mission):
    identity = physical_identity_map(baseline_mission)
    for pid in physical_ids(baseline_mission):
        item = identity[pid]
        assert item["member_chain_ids"] and item["topo_edge_ids"] and item["line_ids"]
        assert item["connected_component_id"] is not None

def _image_context(mission):
    points = []
    for item in effective_image_points(mission):
        point = copy.deepcopy(item); snapped = point.get("snapped_coord") or point.get("raw_coord")
        point.update(point_id=point.get("point_id") or point.get("id"), x=snapped[0], y=snapped[1], pixel_position=list(snapped[:2]), point_type="image_detected")
        points.append(point)
    return {"inspection_point_source": "image", "image_path": "data/chengdu_real_point.png", "inspection_points": points, "image_inspection_overlay": copy.deepcopy(effective_image_points(mission))}

@pytest.fixture
def pure_replan(monkeypatch):
    import planner.replan_start_end as replan
    # Topology rebuild writes debug images; bypass only it to keep real result/ untouched.
    monkeypatch.setattr(replan, "_try_load_topo_for_replan", lambda *_a, **_k: (None, {}))
    return replan.build_start_end_replan_mission

def test_start_end_replan_current_behavior(baseline_mission, pure_replan):
    context, start, end = _image_context(baseline_mission), [801, 410], [751, 892]
    mission = pure_replan(baseline_mission, start, end, mission_context=context)
    assert mission["segments"][0]["geometry_2d"][0] == start
    assert mission["segments"][-1]["geometry_2d"][-1] == end
    assert all(segment_gap(a, b) <= 0.5 for a, b in zip(mission["segments"], mission["segments"][1:]))
    assert physical_ids(mission) == physical_ids(baseline_mission)
    assert [point_identity(p) for p in mission["inspection_points"]] == [point_identity(p) for p in context["inspection_points"]]
    inspect_len = sum(s["length"] for s in mission["segments"] if s["type"] == "inspect")
    connect_len = sum(s["length"] for s in mission["segments"] if s["type"] == "connect")
    assert math.isclose(mission["statistics"]["inspect_length"], inspect_len, abs_tol=0.01)
    assert math.isclose(mission["statistics"]["connect_length"], connect_len, abs_tol=0.01)
    assert math.isclose(mission["statistics"]["total_length"], inspect_len + connect_len, abs_tol=0.01)

def test_replan_connect_classification_is_reported(baseline_mission, pure_replan, record_property):
    mission = pure_replan(
        baseline_mission,
        [801, 410],
        [751, 892],
        mission_context=_image_context(baseline_mission),
    )
    counts, evidence = classify_mission_connects(mission)
    for key in ("topology", "free_flight_between_components", "free_flight_endpoint_access", "unexpected_fallback", "unknown"):
        record_property(f"replan_connect_{key}", counts[key])
    record_property("replan_connect_evidence", repr(evidence))
    print("REPLAN_CONNECT_COUNTS", dict(counts)); print("REPLAN_CONNECT_EVIDENCE", evidence)
    assert counts["free_flight_endpoint_access"] == 2
    assert counts["unknown"] == 7

def test_out_of_bounds_is_rejected(baseline_mission, pure_replan):
    from planner.replan_start_end import ReplanValidationError
    with pytest.raises(ReplanValidationError):
        pure_replan(baseline_mission, [-1, 0], [10, 10], mission_context=_image_context(baseline_mission))

def test_same_start_end_is_currently_allowed_as_closed_mission(baseline_mission, pure_replan):
    point = [801, 410]
    mission = pure_replan(baseline_mission, point, point, mission_context=_image_context(baseline_mission))
    assert mission["segments"][0]["geometry_2d"][0] == point
    assert mission["segments"][-1]["geometry_2d"][-1] == point

def test_a_b_c_currently_recomputes_each_request_from_disk_a(baseline_mission, pure_replan):
    import app
    assert "with LATEST_MISSION_PATH.open" in inspect.getsource(app._plan_image_pipeline)
    context = _image_context(baseline_mission)
    b = pure_replan(baseline_mission, [801, 410], [751, 892], mission_context=context)
    c = pure_replan(baseline_mission, [780, 430], [700, 850], mission_context=context)
    assert physical_ids(b) == physical_ids(c) == physical_ids(baseline_mission)
    assert not baseline_mission.get("metadata", {}).get("mission_id")
    assert not b.get("metadata", {}).get("mission_id") and not c.get("metadata", {}).get("mission_id")

@pytest.mark.xfail(strict=True, reason="Current /api/plan reloads disk A; C cannot identify B as baseline")
def test_c_declares_b_as_its_baseline(baseline_mission, pure_replan):
    context = _image_context(baseline_mission)
    b = pure_replan(baseline_mission, [801, 410], [751, 892], mission_context=context)
    c = pure_replan(baseline_mission, [780, 430], [700, 850], mission_context=context)
    assert c["metadata"]["baseline_mission_id"] == b["metadata"]["mission_id"]
