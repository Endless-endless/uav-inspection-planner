"""
将 Mission / EdgeTask 结果转换为 Web Dashboard 统一 JSON 结构。

前端仅依赖本模块输出的字典，不直接访问 Python 类。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union



def _segment_to_dict(seg: Any, index: int) -> Dict[str, Any]:
    geometry = getattr(seg, "geometry", None) or []
    geom_2d = [[float(p[0]), float(p[1])] for p in geometry if len(p) >= 2]
    return {
        "segment_id": f"seg_{index:04d}",
        "type": getattr(seg, "type", "unknown"),
        "edge_id": getattr(seg, "edge_id", None),
        "from_edge_id": getattr(seg, "from_edge_id", None),
        "to_edge_id": getattr(seg, "to_edge_id", None),
        "length": round(float(getattr(seg, "length", 0.0) or 0.0), 2),
        "direction": getattr(seg, "direction", None),
        "geometry_2d": geom_2d,
    }


def _collect_inspection_points(mission_result: Dict[str, Any]) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    edge_tasks = mission_result.get("edge_tasks") or []
    for task in edge_tasks:
        edge_id = getattr(task, "edge_id", None)
        for pt in getattr(task, "inspection_points", None) or []:
            if isinstance(pt, dict):
                pos = pt.get("pixel_position") or pt.get("position_2d") or pt.get("pos2d")
            else:
                pos = getattr(pt, "pixel_position", None) or getattr(pt, "pos2d", None)
            if not pos or len(pos) < 2:
                continue
            points.append({
                "point_id": pt.get("point_id") if isinstance(pt, dict) else getattr(pt, "point_id", None),
                "edge_id": edge_id,
                "x": float(pos[0]),
                "y": float(pos[1]),
                "point_type": pt.get("point_type") if isinstance(pt, dict) else getattr(pt, "point_type", "sample"),
            })
    return points


def _start_end_from_segments(segments: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not segments:
        return {"start": None, "end": None}
    first_geom = segments[0].get("geometry_2d") or []
    last_geom = segments[-1].get("geometry_2d") or []
    start = first_geom[0] if first_geom else None
    end = last_geom[-1] if last_geom else None
    return {
        "start": {"x": start[0], "y": start[1]} if start else None,
        "end": {"x": end[0], "y": end[1]} if end else None,
    }


def compute_geometry_bounds(
    segments: List[Dict[str, Any]],
    inspection_points: Optional[List[Dict[str, Any]]] = None,
    padding_ratio: float = 0.08,
    min_padding: float = 20.0,
) -> Dict[str, Any]:
    """根据 mission 几何自动计算 Unified 管线视口（不使用 test.png 尺寸）。"""
    xs: List[float] = []
    ys: List[float] = []

    for seg in segments:
        for p in seg.get("geometry_2d") or []:
            if len(p) >= 2:
                xs.append(float(p[0]))
                ys.append(float(p[1]))

    for pt in inspection_points or []:
        if pt.get("x") is not None and pt.get("y") is not None:
            xs.append(float(pt["x"]))
            ys.append(float(pt["y"]))

    if not xs or not ys:
        return {
            "x_range": [0, 1000],
            "y_range": [1000, 0],
            "width": 1000,
            "height": 1000,
        }

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span_x = max(xmax - xmin, 1.0)
    span_y = max(ymax - ymin, 1.0)
    pad_x = max(span_x * padding_ratio, min_padding)
    pad_y = max(span_y * padding_ratio, min_padding)

    return {
        "x_range": [xmin - pad_x, xmax + pad_x],
        "y_range": [ymax + pad_y, ymin - pad_y],
        "width": round(xmax - xmin + 2 * pad_x, 2),
        "height": round(ymax - ymin + 2 * pad_y, 2),
    }


def _segments_from_json(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    segments = []
    for i, seg in enumerate(data.get("segments", [])):
        segments.append({
            "segment_id": seg.get("segment_id", f"seg_{i:04d}"),
            "type": seg.get("type"),
            "edge_id": seg.get("edge_id"),
            "from_edge_id": seg.get("from_edge_id"),
            "to_edge_id": seg.get("to_edge_id"),
            "length": round(float(seg.get("length", 0)), 2),
            "direction": seg.get("direction"),
            "geometry_2d": seg.get("geometry_2d") or [],
        })
    return segments


def _inspection_points_from_json(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    inspection_points = []
    for pt in data.get("inspection_points", []):
        pos = (
            pt.get("pixel_position")
            or pt.get("position_2d")
            or pt.get("position")
        )
        if not pos and pt.get("x") is not None and pt.get("y") is not None:
            pos = [pt["x"], pt["y"]]
        if not pos or len(pos) < 2:
            continue
        inspection_points.append({
            "point_id": pt.get("point_id"),
            "edge_id": pt.get("edge_id"),
            "x": float(pos[0]),
            "y": float(pos[1]),
            "point_type": pt.get("point_type") or pt.get("type") or "sample",
        })
    return inspection_points


def _statistics_from_json(data: Dict[str, Any], inspection_points: List[Dict[str, Any]]) -> Dict[str, Any]:
    stats_raw = data.get("statistics", {})
    total = float(stats_raw.get("total_length", 0))
    connect_len = float(stats_raw.get("connect_length", 0))
    inspect_len = float(stats_raw.get("inspect_length", 0))
    connect_ratio = (connect_len / total) if total > 0 else 0.0
    inspect_ratio_raw = stats_raw.get("inspect_ratio", 0)
    inspect_ratio = (
        float(inspect_ratio_raw) / 100.0
        if float(inspect_ratio_raw) > 1
        else float(inspect_ratio_raw)
    )

    visit = data.get("visit_order", {})
    visit_order = visit.get("edge_visit_order", []) if isinstance(visit, dict) else (visit or [])

    return {
        "total_length": round(total, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "connect_ratio": round(connect_ratio, 4),
        "inspect_ratio": round(inspect_ratio, 4),
        "num_segments": stats_raw.get("num_segments", 0),
        "num_inspection_points": stats_raw.get(
            "num_inspection_points", len(inspection_points)
        ),
        "num_groups": stats_raw.get("num_groups", 0),
        "num_edges": stats_raw.get("num_edges", len(visit_order)),
    }


def build_dashboard_from_mission_json(
    data: Dict[str, Any],
    *,
    pipeline: str,
    source: str,
    planner: str = "legacy",
    input_file: str = "",
    spacing: float = 50.0,
    map_background: Optional[Dict[str, Any]] = None,
    coordinate_mode: str = "auto_fit",
    output_files: Optional[Dict[str, str]] = None,
    extra_metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """从已导出的 mission_output.json 构建 Dashboard 结构。"""
    segments = _segments_from_json(data)
    inspection_points = _inspection_points_from_json(data)
    statistics = _statistics_from_json(data, inspection_points)

    visit = data.get("visit_order", {})
    visit_order = visit.get("edge_visit_order", []) if isinstance(visit, dict) else (visit or [])

    metadata: Dict[str, Any] = {
        "pipeline": pipeline,
        "source": source,
        "planner": planner,
        "input_file": input_file,
        "spacing": spacing,
        "coordinate_mode": coordinate_mode,
    }
    if pipeline == "image":
        metadata["background"] = "data/test.png"
    if extra_metadata:
        metadata.update(extra_metadata)
    if data.get("metadata"):
        metadata["mission_metadata"] = data["metadata"]

    payload: Dict[str, Any] = {
        "segments": segments,
        "inspection_points": inspection_points,
        "statistics": statistics,
        "visit_order": visit_order,
        "group_visit_order": visit.get("group_visit_order", []) if isinstance(visit, dict) else [],
        "markers": _start_end_from_segments(segments),
        "metadata": metadata,
        "output_files": output_files or {},
        "map_background": map_background,
        "map_mode": "image_overlay" if pipeline == "image" else "topology_only",
    }

    if coordinate_mode == "auto_fit":
        payload["bounds"] = compute_geometry_bounds(segments, inspection_points)
    elif map_background and map_background.get("axis"):
        payload["bounds"] = {
            "x_range": map_background["axis"]["x_range"],
            "y_range": map_background["axis"]["y_range"],
            "width": map_background.get("width"),
            "height": map_background.get("height"),
        }

    return payload


def build_image_pipeline_dashboard(
    mission_json_path: Union[str, Path],
    root: Path,
    *,
    map_rel: str = "data/test.png",
    image_url: str = "/api/map/background",
) -> Dict[str, Any]:
    """
    图像主线：仅读取 result/latest/mission_output.json + data/test.png 坐标系。
    """
    from visualization.dashboard_map import get_background_map_config

    mission_json_path = Path(mission_json_path)
    if not mission_json_path.exists():
        raise FileNotFoundError(
            "Image Pipeline requires result/latest/mission_output.json. "
            "Please run python demo/demo_visualization_main.py first."
        )

    with mission_json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    map_bg = get_background_map_config(root, map_rel, image_url)
    rel_source = mission_json_path.relative_to(root).as_posix() if mission_json_path.is_relative_to(root) else str(mission_json_path)

    return build_dashboard_from_mission_json(
        data,
        pipeline="image",
        source=rel_source,
        planner="legacy",
        input_file="data/test.png",
        spacing=0,
        map_background=map_bg,
        coordinate_mode="image_fixed",
        extra_metadata={
            "background": map_rel,
            "map_image": map_rel,
        },
    )


def build_dashboard_payload(
    mission_result: Dict[str, Any],
    *,
    planner: str,
    input_file: str,
    spacing: float,
    connect_planner: Optional[str] = None,
    connect_planner_note: Optional[str] = None,
    output_files: Optional[Dict[str, str]] = None,
    extra_metadata: Optional[Dict[str, Any]] = None,
    map_mode: str = "topology_only",
) -> Dict[str, Any]:
    """Unified 管线：从实时规划结果构建 Dashboard（不绑定 test.png）。"""
    mission = mission_result["mission"]
    from planner.mission_analysis import analyze_mission

    stats = analyze_mission(mission)

    segments = [
        _segment_to_dict(seg, i)
        for i, seg in enumerate(getattr(mission, "segments", []) or [])
    ]
    inspection_points = _collect_inspection_points(mission_result)
    markers = _start_end_from_segments(segments)

    statistics = {
        "total_length": stats["total_length"],
        "inspect_length": stats["inspect_length"],
        "connect_length": stats["connect_length"],
        "connect_ratio": stats["connect_ratio"],
        "inspect_ratio": stats["inspect_ratio"],
        "num_segments": stats["num_segments"],
        "num_inspection_points": len(inspection_points),
        "num_groups": stats["num_groups"],
        "num_edges": stats["num_edges"],
        "num_inspect_segments": stats["num_inspect_segments"],
        "num_connect_segments": stats["num_connect_segments"],
    }

    metadata: Dict[str, Any] = {
        "pipeline": "unified",
        "source": input_file,
        "planner": planner,
        "input_file": input_file,
        "spacing": spacing,
        "connect_planner": connect_planner or ("bfs" if planner == "baseline" else planner),
        "coordinate_mode": "auto_fit",
    }
    if connect_planner_note:
        metadata["connect_planner_note"] = connect_planner_note
    if planner == "dijkstra":
        metadata.setdefault(
            "connect_planner_note",
            "连接段使用 Dijkstra；不可达时已按段 fallback 至 BFS。",
        )
    if extra_metadata:
        metadata.update(extra_metadata)

    effective_map_mode = "topology_only" if map_mode != "image_overlay" else "topology_only"
    # Unified 永不使用 test.png 底图（避免 T 型 toy 叠加到真实地图）

    return {
        "segments": segments,
        "inspection_points": inspection_points,
        "statistics": statistics,
        "visit_order": stats.get("visit_order", []),
        "group_visit_order": stats.get("group_visit_order", []),
        "markers": markers,
        "metadata": metadata,
        "output_files": output_files or {},
        "map_background": None,
        "map_mode": effective_map_mode,
        "bounds": compute_geometry_bounds(segments, inspection_points),
    }


def apply_custom_markers(
    payload: Dict[str, Any],
    start: Optional[Tuple[float, float]] = None,
    end: Optional[Tuple[float, float]] = None,
) -> Dict[str, Any]:
    """覆盖 Dashboard 起终点标记（用于重规划 UI）。"""
    markers = dict(payload.get("markers") or {})
    if start is not None:
        markers["start"] = {"x": float(start[0]), "y": float(start[1])}
    if end is not None:
        markers["end"] = {"x": float(end[0]), "y": float(end[1])}
    payload["markers"] = markers
    return payload


def save_json(path: Union[str, Path], payload: Dict[str, Any]) -> str:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    return str(path.resolve())
