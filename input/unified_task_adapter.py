"""
统一输入 → EdgeTask 适配层（第四阶段）

在不动 TopoGraph / EdgeTask / 规划核心实现的前提下，
从 UnifiedInput 生成 line_inspection_points_by_line 并调用 build_edge_tasks。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

from core.inspection_point_generator import (
    LineInspectionPoint,
    sample_points_along_polyline,
)
from core.topo_task import EdgeTask, build_edge_tasks
from input.unified_adapter import (
    AdaptedLine,
    adapted_line_to_ordered_pixels,
    build_topo_graph_from_independent_lines,
    unified_input_to_independent_lines,
    unified_lines_to_adapted_lines,
)
from input.unified_input import UnifiedInput


def _map_to_3d_placeholder(
    pixel_pos: Tuple[float, float],
    flight_height: float = 0.0,
) -> Tuple[float, float, float]:
    """统一输入管线暂无地形；z 使用 flight_height 占位。"""
    return (float(pixel_pos[0]), float(pixel_pos[1]), float(flight_height))


def generate_inspection_points_for_adapted_line(
    adapted_line: AdaptedLine,
    spacing: float = 50.0,
    include_vertices: bool = True,
) -> List[LineInspectionPoint]:
    """
    沿单条 AdaptedLine 折线生成巡检点。

    - 首尾为 endpoint（high）
    - 折线中间顶点可作为 sample（normal）
    - 按 spacing 等距补 sample 点
    """
    polyline = adapted_line_to_ordered_pixels(adapted_line)
    if len(polyline) < 2:
        raise ValueError(f"AdaptedLine[{adapted_line.id}] 至少需要 2 个点")

    line_id = adapted_line.id
    points: List[LineInspectionPoint] = []
    point_count = 0
    used_positions: set = set()

    def _add_point(
        pos: Tuple[float, float],
        point_type: str,
        line_index: int,
        priority: str,
        source_reason: str,
    ) -> None:
        nonlocal point_count
        key = (round(pos[0], 2), round(pos[1], 2))
        if key in used_positions and point_type == "sample":
            return
        used_positions.add(key)
        points.append(
            LineInspectionPoint(
                id=f"{line_id}_P_{point_count:03d}",
                line_id=line_id,
                point_type=point_type,
                pixel_position=(float(pos[0]), float(pos[1])),
                position_3d=_map_to_3d_placeholder(pos),
                line_index=line_index,
                priority=priority,
                status="uninspected",
                source_reason=source_reason,
            )
        )
        point_count += 1

    start = polyline[0]
    end = polyline[-1]
    _add_point(
        (float(start[0]), float(start[1])),
        "endpoint",
        0,
        "high",
        "unified_start_endpoint",
    )
    _add_point(
        (float(end[0]), float(end[1])),
        "endpoint",
        len(polyline) - 1,
        "high",
        "unified_end_endpoint",
    )

    if include_vertices and len(polyline) > 2:
        for idx in range(1, len(polyline) - 1):
            vx, vy = polyline[idx]
            _add_point(
                (float(vx), float(vy)),
                "sample",
                idx,
                "normal",
                "unified_polyline_vertex",
            )

    for _idx, _dist, sample_pos in sample_points_along_polyline(polyline, spacing=spacing):
        _add_point(
            sample_pos,
            "sample",
            _idx,
            "normal",
            f"unified_spacing_{spacing}px",
        )

    return points


def generate_inspection_points_for_adapted_lines(
    adapted_lines: List[AdaptedLine],
    spacing: float = 50.0,
) -> Dict[str, List[LineInspectionPoint]]:
    """
    为多条 AdaptedLine 生成巡检点字典。

    Returns:
        {line_id: [LineInspectionPoint, ...]}，与 build_edge_tasks 所需格式一致。
    """
    result: Dict[str, List[LineInspectionPoint]] = {}
    for line in adapted_lines:
        result[line.id] = generate_inspection_points_for_adapted_line(
            line, spacing=spacing
        )
    return result


def unified_input_to_edge_tasks(
    unified_input: UnifiedInput,
    spacing: float = 50.0,
    merge_thresh: float = 25.0,
) -> Tuple[Any, List[EdgeTask], Dict[str, List[LineInspectionPoint]]]:
    """
    UnifiedInput → TopoGraph + EdgeTask 列表。

    Returns:
        (topo_graph, edge_tasks, line_inspection_points_by_line)
    """
    adapted_lines = unified_lines_to_adapted_lines(unified_input)
    independent_lines = unified_input_to_independent_lines(unified_input)
    topo_graph, _nodes, _edges = build_topo_graph_from_independent_lines(
        independent_lines, merge_thresh=merge_thresh
    )
    line_inspection_points_by_line = generate_inspection_points_for_adapted_lines(
        adapted_lines, spacing=spacing
    )
    edge_tasks = build_edge_tasks(topo_graph, line_inspection_points_by_line)
    return topo_graph, edge_tasks, line_inspection_points_by_line


def _inspection_point_to_dict(point: LineInspectionPoint) -> Dict[str, Any]:
    return {
        "id": point.id,
        "line_id": point.line_id,
        "point_type": point.point_type,
        "pixel_position": list(point.pixel_position),
        "position_3d": list(point.position_3d) if point.position_3d else None,
        "line_index": point.line_index,
        "priority": point.priority,
        "source_reason": point.source_reason,
    }


def edge_task_to_debug_dict(task: EdgeTask) -> Dict[str, Any]:
    """EdgeTask 调试序列化（不写 mission_output.json）。"""
    pts = []
    for p in task.inspection_points:
        if hasattr(p, "pixel_position"):
            pts.append(_inspection_point_to_dict(p))
        elif isinstance(p, dict):
            pts.append(p)
        else:
            pts.append({"raw": str(p)})

    start_xy = list(task.polyline[0]) if task.polyline else None
    end_xy = list(task.polyline[-1]) if task.polyline else None

    return {
        "edge_id": task.edge_id,
        "line_id": task.line_id,
        "u": task.u,
        "v": task.v,
        "len2d": task.len2d,
        "num_points": task.num_points,
        "is_straight": task.is_straight,
        "polyline_start": start_xy,
        "polyline_end": end_xy,
        "inspection_points": pts,
    }


def build_edge_tasks_debug_payload(
    topo_graph,
    edge_tasks: List[EdgeTask],
    line_inspection_points_by_line: Dict[str, List[LineInspectionPoint]],
    source_path: Optional[str] = None,
) -> Dict[str, Any]:
    """组装 edge_tasks_debug.json 内容。"""
    per_line_counts = {
        lid: len(pts) for lid, pts in line_inspection_points_by_line.items()
    }
    return {
        "source": source_path,
        "topo_summary": {
            "node_count": len(topo_graph.nodes),
            "edge_count": len(topo_graph.edges),
        },
        "line_inspection_point_counts": per_line_counts,
        "total_inspection_points": sum(per_line_counts.values()),
        "edge_task_count": len(edge_tasks),
        "edge_tasks": [edge_task_to_debug_dict(t) for t in edge_tasks],
    }


def save_edge_tasks_debug_json(
    payload: Dict[str, Any],
    path: Union[str, Path],
) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return path


def print_edge_tasks_summary(
    topo_graph,
    edge_tasks: List[EdgeTask],
    line_inspection_points_by_line: Dict[str, List[LineInspectionPoint]],
) -> None:
    """打印 EdgeTask 摘要。"""
    print(f"TopoGraph nodes: {len(topo_graph.nodes)}")
    print(f"TopoGraph edges: {len(topo_graph.edges)}")
    print(f"EdgeTask count: {len(edge_tasks)}")
    total_pts = sum(len(v) for v in line_inspection_points_by_line.values())
    print(f"Total line-level inspection points: {total_pts}")
    print("-" * 50)
    for task in edge_tasks:
        start = task.polyline[0] if task.polyline else None
        end = task.polyline[-1] if task.polyline else None
        print(f"  EdgeTask: {task.edge_id}")
        print(f"    line_id: {task.line_id}")
        print(f"    u -> v: {task.u} -> {task.v}")
        print(f"    inspection_points: {task.num_points}")
        print(f"    polyline start: {start}")
        print(f"    polyline end: {end}")
        print(f"    len2d: {task.len2d:.2f}")
    print("-" * 50)
