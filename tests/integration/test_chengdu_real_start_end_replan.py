import copy
import hashlib
import json
import math

from planner.mission_result_builder import build_dashboard_from_mission_json
from planner.replan_start_end import (
    _connect_geometry_topo,
    build_start_end_replan_mission,
)
from tests.helpers.replan_diagnostics import (
    classify_mission_connects,
    effective_image_points,
    physical_identity_map,
    physical_ids,
    point_identity,
    segment_gap,
)
from tests.integration.test_chengdu_real_pipeline_provenance import (
    PROJECT_ROOT,
    PROVENANCE_FIELDS,
    _run_chengdu_image_pipeline,
    _tree_hashes,
)


START_XY = [137.0, 1049.0]
END_XY = [1264.0, 287.0]
COORD_TOL = 1e-6
CONTINUITY_TOL = 1e-6
EXPECTED_B_BUSINESS_HASH = "80e5faf44c368200f7f0552045d815395606903918394e0ffd97fdc7abef1cb3"


def _identity_rows(mission):
    return {
        physical_id: {
            "line_ids": tuple(item["line_ids"]),
            "member_chain_ids": tuple(item["member_chain_ids"]),
            "topo_edge_ids": tuple(item["topo_edge_ids"]),
            "component_ids": tuple(item["component_ids"]),
        }
        for physical_id, item in physical_identity_map(mission).items()
    }


def _point_rows(mission):
    return sorted(point_identity(point) for point in effective_image_points(mission))


def _lengths(mission):
    inspect_length = sum(
        float(segment["length"])
        for segment in mission["segments"]
        if segment["type"] == "inspect"
    )
    connect_length = sum(
        float(segment["length"])
        for segment in mission["segments"]
        if segment["type"] == "connect"
    )
    return inspect_length, connect_length, inspect_length + connect_length


def _business_hash(mission):
    payload = [
        {
            field: segment.get(field)
            for field in (
                "type",
                "edge_id",
                "from_edge_id",
                "to_edge_id",
                "length",
                "geometry_2d",
            )
        }
        for segment in mission["segments"]
    ]
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def test_same_component_euclidean_degradation_remains_fallback():
    provenance = {}
    _connect_geometry_topo(
        (0.0, 0.0),
        (10.0, 5.0),
        role="between_edges",
        from_edge_id="PL_from",
        to_edge_id="PL_to",
        topo_graph=None,
        edge_task_map={},
        cost_config=None,
        provenance_out=provenance,
        from_component_ids=[4],
        to_component_ids=[4],
    )
    assert provenance["connect_mode"] == "fallback"
    assert provenance["reason"] == "planner_failure"
    assert provenance["fallback_reason"]


def test_missing_component_identity_is_not_mislabeled_between_components():
    provenance = {}
    _connect_geometry_topo(
        (0.0, 0.0),
        (10.0, 5.0),
        role="between_edges",
        from_edge_id="PL_from",
        to_edge_id="PL_to",
        topo_graph=None,
        edge_task_map={},
        cost_config=None,
        provenance_out=provenance,
        from_component_ids=[],
        to_component_ids=[4],
    )
    assert provenance["connect_mode"] == "fallback"
    assert provenance["reason"] != "between_components"
    assert provenance["fallback_reason"]


def test_fresh_chengdu_mission_a_to_single_start_end_replan_b(tmp_path, monkeypatch):
    import core.real_map_cv as real_map_cv

    protected = (
        PROJECT_ROOT / "result" / "latest",
        PROJECT_ROOT / "result" / "web_app",
    )
    before = {str(path): _tree_hashes(path) for path in protected}

    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("UAV_DATASET_TYPE", "real_satellite")
    monkeypatch.setenv("UAV_IMAGE_PIXEL_COORDS", "1")
    point_image = PROJECT_ROOT / "data" / "chengdu_real_point.png"
    monkeypatch.setenv("UAV_DISPLAY_MAP", str(point_image))
    monkeypatch.setattr(real_map_cv, "REAL_MAP_CANONICAL_IMAGE", str(point_image))

    mission_a_path = tmp_path / "mission_a.json"
    mission_a, topo_graph_a = _run_chengdu_image_pipeline(mission_a_path)
    assert len([s for s in mission_a["segments"] if s["type"] == "inspect"]) == 7
    assert len([s for s in mission_a["segments"] if s["type"] == "connect"]) == 6
    assert len(effective_image_points(mission_a)) == 13
    assert len(mission_a["metadata"]["physical_task_registry"]) == 6
    counts_a, evidence_a = classify_mission_connects(mission_a)
    assert counts_a == {
        "topology": 3,
        "free_flight_between_components": 2,
        "point_access": 1,
    }, evidence_a
    assert all(
        segment_gap(left, right) <= CONTINUITY_TOL
        for left, right in zip(mission_a["segments"], mission_a["segments"][1:])
    )
    a_inspect, a_connect, a_total = _lengths(mission_a)
    assert math.isclose(a_total, float(mission_a["statistics"]["total_length"]), abs_tol=0.01)

    context = {
        "inspection_point_source": "image",
        "image_path": str(point_image),
        "inspection_points": copy.deepcopy(effective_image_points(mission_a)),
        "image_inspection_overlay": copy.deepcopy(
            mission_a.get("image_inspection_overlay")
            or mission_a["metadata"].get("image_inspection_overlay")
            or []
        ),
        "image_detection_stats": copy.deepcopy(
            mission_a["metadata"].get("image_detection_stats") or {}
        ),
    }
    mission_b = build_start_end_replan_mission(
        mission_a,
        START_XY,
        END_XY,
        mission_context=context,
    )
    mission_b_path = tmp_path / "mission_b.json"
    mission_b_path.write_text(
        json.dumps(mission_b, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    first = mission_b["segments"][0]["geometry_2d"][0]
    last = mission_b["segments"][-1]["geometry_2d"][-1]
    assert math.dist(first[:2], START_XY) <= COORD_TOL
    assert math.dist(last[:2], END_XY) <= COORD_TOL
    assert first[0] == START_XY[0] and first[1] == START_XY[1]
    assert last[0] == END_XY[0] and last[1] == END_XY[1]

    assert physical_ids(mission_b) == physical_ids(mission_a)
    assert _identity_rows(mission_b) == _identity_rows(mission_a)
    assert _point_rows(mission_b) == _point_rows(mission_a)
    assert len(effective_image_points(mission_b)) == 13
    assert all(
        segment_gap(left, right) <= CONTINUITY_TOL
        for left, right in zip(mission_b["segments"], mission_b["segments"][1:])
    )

    actual_topo_ids = {str(edge.id) for edge in topo_graph_a.edges.values()}
    connects_b = [s for s in mission_b["segments"] if s["type"] == "connect"]
    provenance_errors = []
    for segment in connects_b:
        assert all(field in segment for field in PROVENANCE_FIELDS)
        if segment["reason"] in {"start_endpoint_access", "end_endpoint_access"}:
            assert segment["connect_mode"] == "free_flight"
        elif (
            segment["from_component_ids"]
            and segment["to_component_ids"]
            and set(segment["from_component_ids"]).isdisjoint(
                segment["to_component_ids"]
            )
        ):
            if not (
                segment["connect_mode"] == "free_flight"
                and segment["reason"] == "between_components"
                and segment["fallback_reason"] is None
            ):
                provenance_errors.append(
                    {
                        "segment_id": segment["segment_id"],
                        "components": [
                            segment["from_component_ids"],
                            segment["to_component_ids"],
                        ],
                        "actual": [
                            segment["connect_mode"],
                            segment["reason"],
                            segment["fallback_reason"],
                        ],
                        "expected": ["free_flight", "between_components", None],
                    }
                )
        elif segment["connect_mode"] == "topology":
            assert segment["topo_edge_ids"]
            assert not set(segment["from_component_ids"]).isdisjoint(
                segment["to_component_ids"]
            )
        elif segment["reason"] == "between_components":
            assert segment["connect_mode"] == "free_flight"
            assert set(segment["from_component_ids"]).isdisjoint(
                segment["to_component_ids"]
            )
        elif segment["connect_mode"] == "fallback":
            assert segment["fallback_reason"]
    assert connects_b[0]["reason"] == "start_endpoint_access"
    assert connects_b[-1]["reason"] == "end_endpoint_access"
    counts_b, evidence_b = classify_mission_connects(mission_b)
    assert counts_b == {
        "topology": 2,
        "free_flight_between_components": 2,
        "free_flight_endpoint_access": 2,
        "point_access": 1,
        "unexpected_fallback": 1,
    }, evidence_b
    assert not provenance_errors, provenance_errors
    cross_component_segments = {
        segment["segment_id"]: segment
        for segment in connects_b
        if segment["from_component_ids"]
        and segment["to_component_ids"]
        and set(segment["from_component_ids"]).isdisjoint(
            segment["to_component_ids"]
        )
    }
    assert len(cross_component_segments) == 2
    for segment in cross_component_segments.values():
        assert segment["connect_mode"] == "free_flight"
        assert segment["planner"] == "euclidean"
        assert segment["reason"] == "between_components"
        assert segment["fallback_reason"] is None

    b_inspect, b_connect, b_total = _lengths(mission_b)
    statistics_b = mission_b["statistics"]
    assert math.isclose(b_inspect, float(statistics_b["inspect_length"]), abs_tol=0.01)
    assert math.isclose(b_connect, float(statistics_b["connect_length"]), abs_tol=0.01)
    assert math.isclose(b_total, float(statistics_b["total_length"]), abs_tol=0.01)
    assert statistics_b["num_segments"] == len(mission_b["segments"])
    assert statistics_b["num_inspection_points"] == 13

    dashboard_b = build_dashboard_from_mission_json(
        mission_b, pipeline="image", source=str(mission_b_path)
    )
    dashboard_b_path = tmp_path / "dashboard_b.json"
    dashboard_b_path.write_text(
        json.dumps(dashboard_b, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    assert dashboard_b["metadata"]["physical_task_registry"] == mission_b["metadata"]["physical_task_registry"]
    assert math.dist(
        [dashboard_b["markers"]["start"]["x"], dashboard_b["markers"]["start"]["y"]],
        START_XY,
    ) <= COORD_TOL
    assert math.dist(
        [dashboard_b["markers"]["end"]["x"], dashboard_b["markers"]["end"]["y"]],
        END_XY,
    ) <= COORD_TOL
    assert dashboard_b["statistics"]["num_segments"] == len(mission_b["segments"])
    assert math.isclose(
        float(dashboard_b["statistics"]["total_length"]), b_total, abs_tol=0.01
    )
    for source, rendered in zip(mission_b["segments"], dashboard_b["segments"]):
        for required in ("segment_id", "type", "length", "geometry_2d"):
            assert required in rendered
        if source["type"] == "inspect":
            assert rendered["physical_id"] == source["physical_id"]
        else:
            for field in PROVENANCE_FIELDS:
                assert rendered[field] == source[field]

    after = {str(path): _tree_hashes(path) for path in protected}
    assert after == before
    business_hash = _business_hash(mission_b)
    assert business_hash == EXPECTED_B_BUSINESS_HASH
    print("REPLAN_START_END", START_XY, END_XY)
    print("A_LENGTHS", a_inspect, a_connect, a_total, len(mission_a["segments"]))
    print("B_LENGTHS", b_inspect, b_connect, b_total, len(mission_b["segments"]))
    print("B_BUSINESS_HASH", business_hash)
    print("B_CONNECT_COUNTS", dict(counts_b))
    print("B_PROVENANCE_ERRORS", json.dumps(provenance_errors, ensure_ascii=False, sort_keys=True))
    print("B_CONNECT_ROWS", json.dumps(connects_b, ensure_ascii=False, sort_keys=True))
    print("REAL_RESULT_CHANGED_COUNT", 0)
