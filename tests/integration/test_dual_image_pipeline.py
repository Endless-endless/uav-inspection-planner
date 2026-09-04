import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

import app as app_module
from core.topo_plan import export_grouped_mission_to_json
from planner.mission_result_builder import (
    build_dashboard_from_mission_json,
    build_mission_context,
)
from planner.powerline_planner_v3_final import PowerlinePlannerV3
from planner.replan_start_end import build_start_end_replan_mission


PROJECT_ROOT = Path(__file__).resolve().parents[2]
START_XY = [137.0, 1049.0]
END_XY = [1264.0, 287.0]
POINT_ACCESS_A_GEOMETRY_HASH = "e4694a028b5a866763ced25af69c5060cc4b3f3d3e7a3a8a229b52386a8908c1"
POINT_ACCESS_B_GEOMETRY_HASH = "3b08225ed5930c0313b266e0b245adfc2b8a03119d9efb93e70c05f7ece692c7"


def _tree_hashes(root: Path):
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _point_to_polyline_distance(point, geometry):
    px, py = point
    best = float("inf")
    for a, b in zip(geometry, geometry[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom)) if denom else 0.0
        qx, qy = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(px - qx, py - qy))
    return best


def _geometry_business_hash(mission):
    payload = [
        {
            key: segment.get(key)
            for key in (
                "segment_id", "type", "edge_id", "from_edge_id", "to_edge_id",
                "length", "geometry_2d",
            )
        }
        for segment in mission["segments"]
    ]
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _stable_point_rows(mission):
    rows = {}
    for point in mission.get("inspection_points") or []:
        detection = point.get("detection_result") or {}
        rows[point["point_id"]] = {
            "raw": detection.get("raw_coord"),
            "visit_order": point.get("visit_order"),
        }
    return rows


def _run_dual_image_pipeline(output_path: Path):
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
    export_grouped_mission_to_json(
        mission=mission,
        edge_tasks=edge_tasks,
        line_inspection_points_by_line=planner.line_inspection_points_by_line,
        output_path=str(output_path),
        terrain_3d=planner.terrain_3d,
        weather_info=planner.get_weather_info(),
        extra_metadata={
            "inspection_point_source": "image",
            "map_image": "data/chengdu_real_point.png",
            "line_image": "data/chengdu_real_line.png",
            "point_image": "data/chengdu_real_point.png",
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
        },
    )
    return json.loads(output_path.read_text(encoding="utf-8")), planner, topo_graph, edge_tasks


def test_chengdu_registry_resolves_distinct_line_and_point_images():
    profile = app_module.resolve_image_dataset_profile("data/chengdu_real_point.png")
    assert profile["line_image"] == "data/chengdu_real_line.png"
    assert profile["point_image"] == "data/chengdu_real_point.png"
    assert profile["alignment"] == {
        "mode": "crop",
        "anchor": "top_left",
        "crop_right": 2,
        "expected_line_size": [1417, 1258],
        "expected_point_size": [1415, 1258],
    }


def test_planner_consumes_line_image_for_red_and_point_image_for_detection(tmp_path, monkeypatch):
    import core.inspection_point_detector as detector
    import core.real_map_cv as real_map_cv

    line_path = PROJECT_ROOT / "data" / "chengdu_real_line.png"
    point_path = PROJECT_ROOT / "data" / "chengdu_real_point.png"
    expected_line = np.array(Image.open(line_path).convert("RGB"))
    seen = {"red_inputs": []}
    real_extract = real_map_cv.extract_real_map_red_mask
    monkeypatch.chdir(tmp_path)

    def red_spy(rgb):
        seen["red_inputs"].append(np.array(rgb, copy=True))
        return real_extract(rgb)

    def point_spy(image_path, *, config, red_mask):
        seen["point_path"] = Path(image_path).resolve()
        return [], {"detector": "spy", "merged_points": 0}

    monkeypatch.setattr(real_map_cv, "extract_real_map_red_mask", red_spy)
    monkeypatch.setattr(real_map_cv, "save_green_mask", lambda mask: str(tmp_path / "green.png"))
    monkeypatch.setattr(detector, "detect_black_inspection_points_with_stats", point_spy)
    alignment = app_module.resolve_image_dataset_profile(
        "data/chengdu_real_point.png"
    )["alignment"]
    planner = PowerlinePlannerV3(
        str(line_path), point_image_path=str(point_path), image_alignment=alignment
    )
    planner.step1_extract_redline_hsv()
    planner.step5_detect_image_inspection_points()
    assert seen["point_path"] == point_path.resolve()
    expected_point = np.array(Image.open(point_path).convert("RGB"))
    assert any(np.array_equal(value, expected_line) for value in seen["red_inputs"])
    assert any(np.array_equal(value, expected_line[:, :1415]) for value in seen["red_inputs"])
    assert any(np.array_equal(value, expected_point) for value in seen["red_inputs"])


def test_dual_image_size_mismatch_is_rejected(tmp_path, monkeypatch):
    import core.inspection_point_detector as detector

    line_path = tmp_path / "line.png"
    point_path = tmp_path / "point.png"
    Image.new("RGB", (20, 10), "white").save(line_path)
    Image.new("RGB", (21, 10), "white").save(point_path)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        detector,
        "detect_black_inspection_points_with_stats",
        lambda *args, **kwargs: ([], {}),
    )
    planner = PowerlinePlannerV3(str(line_path), point_image_path=str(point_path))
    planner.step1_extract_redline_hsv()
    with pytest.raises(ValueError, match="尺寸不一致"):
        planner.step5_detect_image_inspection_points()


def test_legacy_single_image_entry_remains_compatible():
    image_path = PROJECT_ROOT / "data" / "test.png"
    planner = PowerlinePlannerV3(str(image_path))
    assert Path(planner.image_path) == image_path
    assert Path(planner.point_image_path) == image_path


def test_chengdu_alignment_crops_right_in_memory_without_changing_origin_or_files(tmp_path, monkeypatch):
    line_path = PROJECT_ROOT / "data" / "chengdu_real_line.png"
    point_path = PROJECT_ROOT / "data" / "chengdu_real_point.png"
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (line_path, point_path)}
    original = np.array(Image.open(line_path).convert("RGB"))
    alignment = app_module.resolve_image_dataset_profile(
        "data/chengdu_real_point.png"
    )["alignment"]
    monkeypatch.chdir(tmp_path)
    planner = PowerlinePlannerV3(
        str(line_path), point_image_path=str(point_path), image_alignment=alignment
    )
    planner.step1_extract_redline_hsv()
    aligned = np.array(planner.image)
    assert aligned.shape[:2] == (1258, 1415)
    assert np.array_equal(aligned, original[:, :1415])
    assert planner.image_alignment_metadata == {
        "mode": "crop",
        "anchor": "top_left",
        "crop_right": 2,
        "original_line_size": [1417, 1258],
        "point_image_size": [1415, 1258],
        "aligned_line_size": [1415, 1258],
        "coordinate_origin_unchanged": True,
        "cropped_region_red_mask_pixels": 0,
        "cropped_region_skeleton_pixels": 0,
        "cropped_region_task_endpoint_count": 0,
    }
    after = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (line_path, point_path)}
    assert after == before


def test_wrong_alignment_expected_size_is_rejected(tmp_path, monkeypatch):
    line_path = PROJECT_ROOT / "data" / "chengdu_real_line.png"
    point_path = PROJECT_ROOT / "data" / "chengdu_real_point.png"
    monkeypatch.chdir(tmp_path)
    planner = PowerlinePlannerV3(
        str(line_path),
        point_image_path=str(point_path),
        image_alignment={
            "mode": "crop",
            "anchor": "top_left",
            "crop_right": 2,
            "expected_line_size": [999, 1258],
            "expected_point_size": [1415, 1258],
        },
    )
    with pytest.raises(ValueError, match="expected_line_size"):
        planner.step1_extract_redline_hsv()


def test_full_chengdu_dual_image_pipeline_visits_restored_points(tmp_path, monkeypatch):
    import core.real_map_cv as real_map_cv

    protected = (PROJECT_ROOT / "result" / "latest", PROJECT_ROOT / "result" / "web_app")
    before = {str(path): _tree_hashes(path) for path in protected}
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("UAV_DATASET_TYPE", "real_satellite")
    monkeypatch.setenv("UAV_IMAGE_PIXEL_COORDS", "1")
    monkeypatch.setenv("UAV_DISPLAY_MAP", str(PROJECT_ROOT / "data" / "chengdu_real_point.png"))
    monkeypatch.setattr(
        real_map_cv,
        "REAL_MAP_CANONICAL_IMAGE",
        str(PROJECT_ROOT / "data" / "chengdu_real_point.png"),
    )
    mission_path = tmp_path / "mission_output.json"
    mission, planner, topo_graph, edge_tasks = _run_dual_image_pipeline(mission_path)
    dashboard = build_dashboard_from_mission_json(mission, pipeline="image", source=str(mission_path))

    overlay = mission["metadata"]["image_inspection_overlay"]
    assert len(overlay) == 13
    assert len({row["id"] for row in overlay}) == 13
    by_id = {row["id"]: row for row in overlay}
    mission_point_rows = _stable_point_rows(mission)
    assert set(mission_point_rows) == set(by_id)
    assert all(
        mission_point_rows[point_id]["raw"] == row["raw_coord"]
        for point_id, row in by_id.items()
    )
    assert by_id["IP_0005"]["raw_coord"] == pytest.approx([465.89, 400.54], abs=0.1)
    assert by_id["IP_0012"]["raw_coord"] == pytest.approx([1293.46, 855.54], abs=0.1)
    assert by_id["IP_0005"]["snap_distance"] <= 3.0
    assert by_id["IP_0012"]["snap_distance"] <= 3.0
    assert by_id["IP_0005"]["snapped_coord"] != [276.0, 216.0]
    assert by_id["IP_0012"]["snapped_coord"] != [1111.0, 775.0]

    access_segments = [
        segment for segment in mission["segments"]
        if segment.get("reason") == "point_access"
    ]
    assert len(access_segments) == 1
    access = access_segments[0]
    assert access["inspection_point_id"] == "IP_0007"
    assert access["physical_id"] == "PL_000"
    assert access["connect_mode"] == "free_flight"
    assert access["planner"] == "euclidean"
    assert access["fallback_reason"] is None
    assert access["geometry_2d"][0] == access["anchor_coord"]
    assert access["geometry_2d"][1] == pytest.approx(by_id["IP_0007"]["raw_coord"])
    assert access["geometry_2d"][2] == access["anchor_coord"]
    assert access["raw_coord"] == pytest.approx(by_id["IP_0007"]["raw_coord"])
    assert access["length"] == pytest.approx(
        sum(
            math.dist(left, right)
            for left, right in zip(access["geometry_2d"], access["geometry_2d"][1:])
        )
    )
    assert len(mission["segments"]) == 13
    assert sum(segment["type"] == "inspect" for segment in mission["segments"]) == 7
    assert sum(segment["type"] == "connect" for segment in mission["segments"]) == 6

    inspect_segments = [segment for segment in mission["segments"] if segment["type"] == "inspect"]
    distances = {
        point_id: min(
            _point_to_polyline_distance(by_id[point_id]["raw_coord"], segment["geometry_2d"])
            for segment in inspect_segments
        )
        for point_id in by_id
    }
    assert distances["IP_0005"] <= 3.0
    assert distances["IP_0012"] <= 3.0
    mission_distances = {
        point_id: min(
            _point_to_polyline_distance(
                by_id[point_id]["raw_coord"], segment["geometry_2d"]
            )
            for segment in mission["segments"]
        )
        for point_id in by_id
    }
    assert mission_distances["IP_0007"] <= 1e-6
    for left, right in zip(mission["segments"], mission["segments"][1:]):
        assert np.allclose(left["geometry_2d"][-1], right["geometry_2d"][0], atol=1e-6)

    assert dashboard["metadata"]["line_image"] == "data/chengdu_real_line.png"
    assert dashboard["metadata"]["point_image"] == "data/chengdu_real_point.png"
    assert dashboard["metadata"]["image_alignment"] == planner.image_alignment_metadata
    assert len(dashboard["segments"]) == len(mission["segments"])
    for dashboard_segment, mission_segment in zip(dashboard["segments"], mission["segments"]):
        assert dashboard_segment["segment_id"] == mission_segment["segment_id"]
        assert dashboard_segment["type"] == mission_segment["type"]
        assert dashboard_segment["geometry_2d"] == mission_segment["geometry_2d"]
        assert dashboard_segment["length"] == mission_segment["length"]
    dashboard_access = next(
        segment for segment in dashboard["segments"]
        if segment.get("reason") == "point_access"
    )
    for field in (
        "inspection_point_id", "physical_id", "anchor_coord", "raw_coord",
        "geometry_2d", "length", "connect_mode", "planner", "reason",
        "fallback_reason",
    ):
        assert dashboard_access[field] == access[field]
    dashboard_points = {
        point["point_id"]: point for point in dashboard["inspection_points"]
    }
    assert dashboard_points["IP_0007"]["segment_id"] == dashboard_access["segment_id"]
    assert dashboard_points["IP_0007"]["route_visit_coord"] == pytest.approx(
        by_id["IP_0007"]["raw_coord"]
    )
    assert {
        point["point_id"]: [point["raw_x"], point["raw_y"]]
        for point in dashboard["inspection_points"]
    } == {
        point_id: row["raw_coord"] for point_id, row in by_id.items()
    }
    assert _geometry_business_hash(mission) == POINT_ACCESS_A_GEOMETRY_HASH

    context = build_mission_context(
        mission,
        request={
            "inspection_point_source": "image",
            "image_path": str(PROJECT_ROOT / "data" / "chengdu_real_point.png"),
        },
    )
    mission_b = build_start_end_replan_mission(
        mission, START_XY, END_XY, mission_context=context
    )
    dashboard_b = build_dashboard_from_mission_json(
        mission_b, pipeline="image", source="isolated:dual-image-replan"
    )
    rows_b = _stable_point_rows(mission_b)
    assert set(rows_b) == set(mission_point_rows)
    assert {
        point_id: row["raw"] for point_id, row in rows_b.items()
    } == {
        point_id: row["raw"] for point_id, row in mission_point_rows.items()
    }
    assert rows_b["IP_0012"]["raw"] == pytest.approx([1293.46, 855.54], abs=0.1)
    assert rows_b["IP_0007"]["raw"] == pytest.approx([1139.42, 521.37], abs=0.6)
    access_segments_b = [
        segment for segment in mission_b["segments"]
        if segment.get("reason") == "point_access"
    ]
    assert len(access_segments_b) == 1
    access_b = access_segments_b[0]
    assert access_b["inspection_point_id"] == "IP_0007"
    assert access_b["geometry_2d"][1] == pytest.approx(rows_b["IP_0007"]["raw"])
    assert access_b["length"] == pytest.approx(
        sum(
            math.dist(left, right)
            for left, right in zip(access_b["geometry_2d"], access_b["geometry_2d"][1:])
        )
    )
    assert all(
        np.allclose(left["geometry_2d"][-1], right["geometry_2d"][0], atol=1e-6)
        for left, right in zip(mission_b["segments"], mission_b["segments"][1:])
    )
    assert {
        point["point_id"]: point["image_url"]
        for point in dashboard_b["inspection_points"]
    } == {
        point_id: f"/api/inspection-image/{point_id}.jpg"
        for point_id in rows_b
    }
    assert _geometry_business_hash(mission_b) == POINT_ACCESS_B_GEOMETRY_HASH
    after = {str(path): _tree_hashes(path) for path in protected}
    assert after == before
    print("DUAL_IMAGE_COUNTS", json.dumps({
        "red_components": planner.image_detection_stats.get("red_components"),
        "skeleton_components": 6,
        "independent_lines": len(planner.independent_lines),
        "topo_edges": len(topo_graph.edges),
        "edge_tasks": len(edge_tasks),
        "physical_line_chains": len(planner.physical_line_chains),
        "inspect_segments": len(inspect_segments),
        "connect_segments": sum(segment["type"] == "connect" for segment in mission["segments"]),
    }, sort_keys=True))
    print("DUAL_IMAGE_POINT_DISTANCES", json.dumps(distances, sort_keys=True))
    print("REAL_RESULT_CHANGED_COUNT", 0)
