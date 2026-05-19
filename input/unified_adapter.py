"""
统一输入 → 项目内部线路结构的适配层（第二阶段）

将 UnifiedInput.lines 转为 AdaptedLine，便于后续对接 IndependentLine / TopoGraph。
本模块不修改 TopoGraph、EdgeTask 或路径规划逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

import numpy as np

from input.unified_input import UnifiedInput, UnifiedLine, validate_unified_input

# 桥接目标类型（仅用于类型提示与构造，不修改 core 模块）
from core.independent_lines import IndependentLine


@dataclass
class AdaptedLine:
    """
    适配后的线路（介于 UnifiedLine 与 IndependentLine 之间）

    points 为 (N, 2) 的 float 数组，坐标系与 UnifiedInput 一致（通常为图像像素系）。
    """

    id: str
    points: np.ndarray
    voltage_level: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    length: float = 0.0

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError(f"AdaptedLine[{self.id}] points 必须为 (N, 2) 数组")
        if self.length <= 0.0 and len(self.points) >= 2:
            self.length = compute_polyline_length(self.points)


def compute_polyline_length(points: np.ndarray) -> float:
    """累计相邻点欧氏距离，与 IndependentLine.compute_length_2d 一致。"""
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def _unified_line_to_adapted(line: UnifiedLine) -> AdaptedLine:
    if not line.points:
        raise ValueError(f"lines[{line.id}] points 为空")
    if len(line.points) < 2:
        raise ValueError(f"lines[{line.id}] 至少需要 2 个点，当前为 {len(line.points)}")

    points = np.array(line.points, dtype=np.float64)
    length = compute_polyline_length(points)

    return AdaptedLine(
        id=line.id,
        points=points,
        voltage_level=line.voltage_level,
        metadata=dict(line.metadata),
        length=length,
    )


def unified_lines_to_adapted_lines(unified_input: UnifiedInput) -> List[AdaptedLine]:
    """
    将 UnifiedInput 中的线路转为 AdaptedLine 列表。

    Args:
        unified_input: 已通过校验的统一输入

    Returns:
        List[AdaptedLine]

    Raises:
        ValueError: 校验失败或某条线路点数不足
    """
    errors = validate_unified_input(unified_input)
    if errors:
        raise ValueError(
            "UnifiedInput 校验失败:\n" + "\n".join(f"  - {e}" for e in errors)
        )

    if not unified_input.lines:
        raise ValueError("UnifiedInput.lines 不能为空")

    return [_unified_line_to_adapted(line) for line in unified_input.lines]


def adapted_line_to_polyline(adapted_line: AdaptedLine) -> np.ndarray:
    """
    返回可用于 TopoGraph 准备的二维折线点列。

    说明：
    - TopoEdge.polyline 类型为 List[Tuple[float, float]]
    - IndependentLine.ordered_pixels 为 List[Tuple[int, int]]（骨架像素序）
    - 本函数返回 float64 的 (N, 2) ndarray；接入 Topo 时可：
        polyline = [tuple(p) for p in adapted_line_to_polyline(line)]
      或传入 detect_topo_nodes / split_lines_to_edges 的定制入口（后续适配器步骤）。
    """
    return np.asarray(adapted_line.points, dtype=np.float64)


def adapted_line_to_ordered_pixels(adapted_line: AdaptedLine) -> List[tuple]:
    """
    转为与 IndependentLine.ordered_pixels 相同风格的点列（四舍五入为整数像素）。
    供后续可选的 IndependentLine 构造使用，本阶段不自动写入规划器。
    """
    pts = adapted_line_to_polyline(adapted_line)
    return [(int(round(x)), int(round(y))) for x, y in pts]


def adapted_line_to_independent_line(adapted_line: AdaptedLine) -> IndependentLine:
    """
    将 AdaptedLine 转为 IndependentLine，供 detect_topo_nodes / split_lines_to_edges 使用。

    与图像骨架流程的差异：
    - 统一输入只有稀疏折线顶点，无稠密骨架像素；raw_pixels 与 ordered_pixels 相同。
    - endpoints 取折线首尾点（不跑 detect_endpoints 图遍历）。
    - length_2d 使用已计算的折线累计长度。
    """
    ordered_pixels = adapted_line_to_ordered_pixels(adapted_line)
    if len(ordered_pixels) < 2:
        raise ValueError(f"AdaptedLine[{adapted_line.id}] 至少需要 2 个点")

    line = IndependentLine(
        id=adapted_line.id,
        raw_pixels=list(ordered_pixels),
        ordered_pixels=list(ordered_pixels),
        length_2d=float(adapted_line.length),
        endpoints=[ordered_pixels[0], ordered_pixels[-1]],
        is_valid=True,
    )
    if line.length_2d <= 0.0:
        line.compute_length_2d()
    return line


def unified_input_to_independent_lines(unified_input: UnifiedInput) -> List[IndependentLine]:
    """
    UnifiedInput → AdaptedLine 列表 → IndependentLine 列表。
    """
    adapted = unified_lines_to_adapted_lines(unified_input)
    return [adapted_line_to_independent_line(a) for a in adapted]


def build_topo_graph_from_independent_lines(
    independent_lines: List[IndependentLine],
    merge_thresh: float = 25.0,
):
    """
    复用 step7_5 拓扑构建流程（不写入可视化、不修改 TopoGraph 定义）。

    Returns:
        (topo_graph, topo_nodes, topo_edges)
    """
    from core.topo import (
        build_topo_graph,
        detect_topo_nodes,
        merge_duplicate_nodes,
        split_lines_to_edges,
        update_edges_after_merge,
    )

    topo_nodes = detect_topo_nodes(independent_lines)
    topo_nodes, old_to_new_id = merge_duplicate_nodes(topo_nodes, thresh=merge_thresh)
    raw_edges = split_lines_to_edges(independent_lines, topo_nodes)
    topo_edges = update_edges_after_merge(raw_edges, old_to_new_id)
    topo_graph = build_topo_graph(topo_nodes, topo_edges)
    return topo_graph, topo_nodes, topo_edges


def print_independent_lines_summary(lines: List[IndependentLine]) -> None:
    """打印 IndependentLine 摘要。"""
    print(f"IndependentLine count: {len(lines)}")
    print("-" * 50)
    for line in lines:
        n = len(line.ordered_pixels)
        start = line.ordered_pixels[0] if n else None
        end = line.ordered_pixels[-1] if n else None
        print(f"  id: {line.id}")
        print(f"    ordered_pixels: {n}")
        print(f"    start: {start}")
        print(f"    end: {end}")
        print(f"    length_2d: {line.length_2d:.2f}")
        print(f"    bbox: {line.bbox}")
    print("-" * 50)


def print_topo_graph_summary(topo_graph) -> None:
    """打印 TopoGraph 节点/边摘要。"""
    print(f"TopoGraph nodes: {len(topo_graph.nodes)}")
    print(f"TopoGraph edges: {len(topo_graph.edges)}")
    print("-" * 50)
    for edge_id, edge in topo_graph.edges.items():
        u_node = topo_graph.get_node(edge.u)
        v_node = topo_graph.get_node(edge.v)
        u_pos = u_node.pos2d if u_node else ("?", "?")
        v_pos = v_node.pos2d if v_node else ("?", "?")
        print(f"  edge: {edge_id}")
        print(f"    line_id: {edge.line_id}")
        print(f"    u: {edge.u} @ {u_pos}")
        print(f"    v: {edge.v} @ {v_pos}")
        print(f"    len2d: {edge.len2d:.2f}")
    print("-" * 50)


def print_adapted_lines_summary(adapted_lines: List[AdaptedLine]) -> None:
    """打印适配线路摘要。"""
    print(f"Adapted lines count: {len(adapted_lines)}")
    print("-" * 50)
    for line in adapted_lines:
        start = tuple(line.points[0])
        end = tuple(line.points[-1])
        print(f"  id: {line.id}")
        print(f"    points: {len(line.points)}")
        print(f"    start: {start}")
        print(f"    end: {end}")
        print(f"    length: {line.length:.2f}")
        print(f"    voltage_level: {line.voltage_level}")
    print("-" * 50)
