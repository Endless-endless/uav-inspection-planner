"""
真实卫星图手工标注 JSON 加载与拓扑注入。

运行时仅读取 data/chengdu_real_manual.json，不执行红线/黑圈 CV 检测。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

from core.independent_lines import IndependentLine
from core.inspection_point_generator import LineInspectionPoint, _map_to_3d


def load_manual_dataset(json_path: Union[str, Path]) -> Dict[str, Any]:
    path = Path(json_path)
    if not path.exists():
        raise FileNotFoundError(f"手工标注 JSON 不存在: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("dataset_type") != "real_satellite_manual":
        raise ValueError(f"dataset_type 必须为 real_satellite_manual，当前: {data.get('dataset_type')}")
    return data


def _polyline_from_entry(entry: Dict[str, Any]) -> List[Tuple[int, int]]:
    raw = entry.get("polyline") or []
    pts: List[Tuple[int, int]] = []
    for p in raw:
        if not p or len(p) < 2:
            continue
        pts.append((int(round(float(p[0]))), int(round(float(p[1])))))
    if len(pts) < 2:
        raise ValueError(f"线路 {entry.get('id')} polyline 至少需要 2 个点")
    return pts


def build_independent_lines_from_manual(manual: Dict[str, Any]) -> List[IndependentLine]:
    lines: List[IndependentLine] = []
    for entry in manual.get("lines") or []:
        line_id = str(entry.get("id") or f"L_{len(lines) + 1:03d}")
        ordered = _polyline_from_entry(entry)
        line = IndependentLine(
            id=line_id,
            raw_pixels=list(ordered),
            ordered_pixels=list(ordered),
            endpoints=[ordered[0], ordered[-1]],
            is_valid=True,
        )
        line.compute_length_2d()
        lines.append(line)
    if not lines:
        raise ValueError("手工标注 JSON 中 lines 为空")
    return lines


def _nearest_line_id(
    position: Tuple[float, float],
    lines: List[IndependentLine],
) -> Tuple[Optional[str], float]:
    from core.topo_task import project_point_to_polyline

    best_line: Optional[str] = None
    best_dist = float("inf")
    for line in lines:
        proj = project_point_to_polyline(position, line.ordered_pixels)
        if proj is None:
            continue
        d = float(proj["distance"])
        if d < best_dist:
            best_dist = d
            best_line = line.id
    return best_line, best_dist


def build_manual_inspection_points(
    manual: Dict[str, Any],
    independent_lines: List[IndependentLine],
    *,
    terrain: Optional[np.ndarray] = None,
    flight_height: float = 30.0,
    snap_orphan_max_dist: float = 120.0,
) -> Tuple[List[LineInspectionPoint], Dict[str, List[LineInspectionPoint]]]:
    line_ids = {line.id for line in independent_lines}
    points_by_line: Dict[str, List[LineInspectionPoint]] = {}
    all_points: List[LineInspectionPoint] = []

    for idx, entry in enumerate(manual.get("inspection_points") or []):
        pos = entry.get("position")
        if not pos or len(pos) < 2:
            continue
        xy = (float(pos[0]), float(pos[1]))
        point_id = str(entry.get("id") or f"IP_{idx + 1:04d}")
        line_id = str(entry.get("line_id") or "").strip()

        if line_id and line_id not in line_ids:
            nearest, _ = _nearest_line_id(xy, independent_lines)
            if nearest:
                line_id = nearest

        if not line_id:
            nearest, _dist = _nearest_line_id(xy, independent_lines)
            if nearest:
                line_id = nearest

        if not line_id and independent_lines:
            line_id = independent_lines[0].id

        pos_3d = _map_to_3d(xy, terrain, flight_height)
        point = LineInspectionPoint(
            id=point_id,
            line_id=line_id,
            point_type="manual",
            pixel_position=xy,
            position_3d=pos_3d,
            line_index=len(points_by_line.get(line_id, [])),
            priority="high",
            status="uninspected",
            source_reason="manual_json",
        )
        points_by_line.setdefault(line_id, []).append(point)
        all_points.append(point)

    for line_id in points_by_line:
        points_by_line[line_id].sort(key=lambda p: (p.pixel_position[0], p.pixel_position[1]))

    return all_points, points_by_line


def apply_manual_dataset_to_planner(planner, manual_path: Union[str, Path]) -> Dict[str, Any]:
    """将手工 JSON 注入 PowerlinePlannerV3，跳过 CV 线路/点检测。"""
    manual = load_manual_dataset(manual_path)

    planner.width = int(manual.get("image_width") or planner.width or 0)
    planner.height = int(manual.get("image_height") or planner.height or 0)
    clean_map = str(manual.get("clean_map_image") or "data/chengdu_real_satellite.png")
    planner.clean_map_path = clean_map.replace("\\", "/")
    planner.inspection_point_source = "manual"
    planner.image_inspection_detections = []
    planner.image_inspection_overlay = []
    planner.image_detection_stats = {"source": "manual_json", "path": str(manual_path)}

    planner.independent_lines = build_independent_lines_from_manual(manual)
    if planner.independent_lines:
        planner.primary_line_id = max(
            planner.independent_lines,
            key=lambda ln: ln.length_2d,
        ).id

    terrain = np.zeros((max(1, planner.height), max(1, planner.width)), dtype=np.float32)
    planner.step6_smooth_terrain(terrain)

    (
        planner.line_inspection_points,
        planner.line_inspection_points_by_line,
    ) = build_manual_inspection_points(
        manual,
        planner.independent_lines,
        terrain=planner.height_map_smooth,
        flight_height=planner.flight_height,
    )

    print(f"[手工标注] 线路 {len(planner.independent_lines)} 条，巡检点 {len(planner.line_inspection_points)} 个")
    return manual
