import hashlib
import json
from pathlib import Path

import numpy as np

import app as app_module
from core.topo_plan import export_grouped_mission_to_json
from planner.mission_result_builder import build_dashboard_from_mission_json
from planner.powerline_planner_v3_final import PowerlinePlannerV3
from tests.helpers.replan_diagnostics import classify_mission_connects, physical_identity_map


PROJECT_ROOT = Path(__file__).resolve().parents[2]
EXPECTED_POINT_ACCESS_BUSINESS_HASH = "8c5b3835ab4b37598bff17d7394c5bfb8cc401c5b86e23bffc268d82275b4550"
PROVENANCE_FIELDS = (
    "connect_mode",
    "planner",
    "reason",
    "fallback_reason",
    "topo_edge_ids",
    "from_component_ids",
    "to_component_ids",
)


def _tree_hashes(root: Path):
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _business_payload(mission):
    return [
        {
            "type": segment["type"],
            "edge_id": segment.get("edge_id"),
            "from_edge_id": segment.get("from_edge_id"),
            "to_edge_id": segment.get("to_edge_id"),
            "length": segment["length"],
            "geometry_2d": segment["geometry_2d"],
        }
        for segment in mission["segments"]
    ]


def _business_hash(mission):
    encoded = json.dumps(
        _business_payload(mission), sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _run_chengdu_image_pipeline(output_path: Path):
    """Run the production image-planning steps without demo.main's real-result cleanup."""
    line_image = PROJECT_ROOT / "data" / "chengdu_real_line.png"
    point_image = PROJECT_ROOT / "data" / "chengdu_real_point.png"
    planner = PowerlinePlannerV3(
        image_path=str(line_image),
        point_image_path=str(point_image),
        image_alignment=app_module.resolve_image_dataset_profile(
            "data/chengdu_real_point.png"
        )["alignment"],
        flight_height=30,
        weather_scene="calm",
    )
    planner.inspection_point_source = "image"
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()
    planner.step5_detect_image_inspection_points()
    terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
    planner.step6_smooth_terrain(terrain)
    topo_graph = planner.step7_5_build_topo()
    planner.step5_finalize_image_inspection_points()
    planner._map_existing_points_to_3d()
    edge_tasks = planner.step8_5_build_edge_tasks()
    mission = planner.step9_4_plan_global_topology_optimized(enable_sa=False, eps=150.0)
    assert mission is not None

    weather = planner.get_weather_info()
    if planner.stats:
        for key in ("weather_penalty_total", "estimated_energy_score"):
            if key in planner.stats:
                weather[key] = planner.stats[key]
    extra_metadata = {
        "inspection_point_source": "image",
        "map_image": "data/chengdu_real_point.png",
        "point_image": "data/chengdu_real_point.png",
        "line_image": "data/chengdu_real_line.png",
        "clean_map_image": "data/chengdu_real_point.png",
        "display_map_image": "data/chengdu_real_point.png",
        "coordinate_mode": "image_pixel_fixed",
        "pixel_coordinate_mode": True,
        "image_width": planner.width,
        "image_height": planner.height,
        "image_inspection_overlay": planner.image_inspection_overlay,
        "image_detection_stats": planner.image_detection_stats,
        "image_alignment": planner.image_alignment_metadata,
        "dataset_type": "real_satellite",
    }
    export_grouped_mission_to_json(
        mission=mission,
        edge_tasks=edge_tasks,
        line_inspection_points_by_line=planner.line_inspection_points_by_line,
        output_path=str(output_path),
        terrain_3d=planner.terrain_3d,
        weather_info=weather,
        extra_metadata=extra_metadata,
    )
    return json.loads(output_path.read_text(encoding="utf-8")), topo_graph


def test_chengdu_real_image_pipeline_persists_identity_and_provenance(
    tmp_path, monkeypatch
):
    import core.real_map_cv as real_map_cv

    protected = (PROJECT_ROOT / "result" / "latest", PROJECT_ROOT / "result" / "web_app")
    before = {str(path): _tree_hashes(path) for path in protected}

    # All production debug paths are relative; changing cwd isolates them under tmp_path.
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("UAV_DATASET_TYPE", "real_satellite")
    monkeypatch.setenv("UAV_IMAGE_PIXEL_COORDS", "1")
    monkeypatch.setenv("UAV_DISPLAY_MAP", str(PROJECT_ROOT / "data" / "chengdu_real_point.png"))
    # canonical_real_map_image_path otherwise rewrites an absolute input back to
    # project-relative data/...; retain the same image while cwd is isolated.
    monkeypatch.setattr(
        real_map_cv,
        "REAL_MAP_CANONICAL_IMAGE",
        str(PROJECT_ROOT / "data" / "chengdu_real_point.png"),
    )
    mission_path = tmp_path / "isolated" / "mission_output.json"
    mission, topo_graph = _run_chengdu_image_pipeline(mission_path)
    dashboard = build_dashboard_from_mission_json(
        mission, pipeline="image", source=str(mission_path)
    )

    registry = mission["metadata"]["physical_task_registry"]
    identity = physical_identity_map(mission)
    assert registry
    assert len({item["physical_id"] for item in registry}) == len(registry)
    actual_topo_ids = {str(edge.id) for edge in topo_graph.edges.values()}
    for item in registry:
        assert item["physical_id"]
        assert item["member_chain_ids"]
        assert item["topo_edge_ids"]
        assert set(item["topo_edge_ids"]) <= actual_topo_ids
        assert item["line_ids"]
        assert item["component_ids"]
    for segment in mission["segments"]:
        if segment["type"] == "inspect":
            assert segment["physical_id"] in identity

    connect_rows = []
    topology_without_edges = []
    component_mode_conflicts = []
    for segment in mission["segments"]:
        if segment["type"] != "connect":
            continue
        assert all(field in segment for field in PROVENANCE_FIELDS)
        if segment["connect_mode"] == "topology":
            if not segment["topo_edge_ids"]:
                topology_without_edges.append(segment["segment_id"])
            else:
                assert set(segment["topo_edge_ids"]) <= actual_topo_ids
            if set(segment["from_component_ids"]).isdisjoint(segment["to_component_ids"]):
                component_mode_conflicts.append(segment["segment_id"])
        if segment["connect_mode"] == "free_flight" and segment["reason"] == "between_components":
            assert set(segment["from_component_ids"]).isdisjoint(segment["to_component_ids"])
        if segment["connect_mode"] == "fallback":
            assert segment["fallback_reason"]
        connect_rows.append({key: segment.get(key) for key in (
            "segment_id", "from_edge_id", "to_edge_id", *PROVENANCE_FIELDS, "length"
        )})

    counts, evidence = classify_mission_connects(mission)
    business_hash = _business_hash(mission)
    assert business_hash == EXPECTED_POINT_ACCESS_BUSINESS_HASH
    assert counts == {
        "topology": 3,
        "free_flight_between_components": 2,
        "point_access": 1,
    }, evidence
    assert not topology_without_edges
    assert not component_mode_conflicts

    assert dashboard["metadata"]["physical_task_registry"] == registry
    assert _business_payload(dashboard) == _business_payload(mission)
    for source, rendered in zip(mission["segments"], dashboard["segments"]):
        if source["type"] == "inspect":
            assert rendered["physical_id"] == source["physical_id"]
        elif source["type"] == "connect":
            for field in PROVENANCE_FIELDS:
                assert rendered[field] == source[field]
        for required in ("segment_id", "type", "length", "geometry_2d"):
            assert required in rendered

    after = {str(path): _tree_hashes(path) for path in protected}
    assert after == before
    print("CHENGDU_INPUTS", json.dumps({
        "dataset": "data/chengdu_real_point.png",
        "line_image": "data/chengdu_real_line.png",
        "point_image": "data/chengdu_real_point.png",
        "manual_json_configured": False,
    }, ensure_ascii=False))
    print("CHENGDU_TEMP_OUTPUT", mission_path)
    print("PHYSICAL_TASK_REGISTRY", json.dumps(registry, ensure_ascii=False, sort_keys=True))
    print("CONNECT_ROWS", json.dumps(connect_rows, ensure_ascii=False, sort_keys=True))
    print("CONNECT_COUNTS", json.dumps({key: counts[key] for key in (
        "topology", "free_flight_between_components", "free_flight_endpoint_access", "point_access",
        "unexpected_fallback", "unknown"
    )}, sort_keys=True))
    print("TOPOLOGY_WITHOUT_EDGE_EVIDENCE", json.dumps(topology_without_edges))
    print("COMPONENT_MODE_CONFLICTS", json.dumps(component_mode_conflicts))
    print("BUSINESS_HASH", business_hash)
    print("EXPECTED_POINT_ACCESS_BUSINESS_HASH", EXPECTED_POINT_ACCESS_BUSINESS_HASH)
    print("REAL_RESULT_CHANGED_COUNT", 0)
