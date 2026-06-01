"""
=====================================================
拓扑任务建模层
=====================================================

功能：
- 将巡检点映射到拓扑边
- 构建基于拓扑边的任务单元（EdgeTask）
- 为拓扑级路径规划奠定基础

EdgeTask：
- 基于 TopoEdge 的任务单元
- 包含该边的所有巡检点
- 可用于路径规划和任务调度

设计原则：
- 每条 EdgeTask 是一个独立的巡检任务
- 巡检点不遗漏、不重复
- 支持任务优先级和约束
"""

from dataclasses import dataclass, field
from typing import Any, List, Tuple, Dict, Optional, Sequence
import re
import numpy as np


# =====================================================
# 数据结构
# =====================================================

@dataclass
class EdgeTask:
    """
    边任务：基于拓扑边的巡检任务单元

    属性：
        edge_id: 拓扑边ID
        u: 起始节点ID
        v: 结束节点ID
        line_id: 所属线路ID
        polyline: 2D几何（骨架段）
        len2d: 长度
        inspection_points: 该边的巡检点列表
        num_points: 巡检点数量
        is_straight: 是否近似直线
    """
    edge_id: str                          # 拓扑边ID
    u: str                                # 起始节点ID
    v: str                                # 结束节点ID
    line_id: str                          # 所属线路ID
    polyline: List[Tuple[float, float]]   # 2D几何
    len2d: float                          # 长度
    pixel_polyline: List[Tuple[float, float]] = field(default_factory=list)
    original_polyline: List[Tuple[float, float]] = field(default_factory=list)
    image_polyline: List[Tuple[float, float]] = field(default_factory=list)
    inspection_points: List[Dict] = field(default_factory=list)  # 巡检点列表
    num_points: int = 0                    # 巡检点数量
    is_straight: bool = True              # 是否近似直线
    split_reason: Optional[str] = None     # 切分原因

    # 任务属性（后续扩展）
    priority: int = 0                     # 优先级
    estimated_time: float = 0.0           # 预估执行时间
    meta: Dict = field(default_factory=dict)


# =====================================================
# 点到边的映射
# =====================================================

def point_to_line_segment_distance(point, line_start, line_end):
    """
    计算点到线段的垂直距离
    """
    point = np.array(point)
    start = np.array(line_start)
    end = np.array(line_end)

    line_vec = end - start
    point_vec = point - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-6:
        return np.linalg.norm(point_vec)

    t = np.dot(point_vec, line_vec) / (line_len ** 2)
    t = np.clip(t, 0, 1)
    closest = start + t * line_vec
    return np.linalg.norm(point - closest)


def project_point_to_polyline(
    point: Sequence[float],
    polyline: Sequence[Sequence[float]],
) -> Optional[Dict[str, Any]]:
    """Project a point onto a polyline and return snap metadata."""
    if len(polyline) < 2:
        return None

    best: Optional[Dict[str, Any]] = None
    arc = 0.0
    for i in range(len(polyline) - 1):
        p1 = np.array(polyline[i], dtype=float)
        p2 = np.array(polyline[i + 1], dtype=float)
        seg_vec = p2 - p1
        seg_len = float(np.linalg.norm(seg_vec))
        if seg_len < 1e-6:
            continue

        pt = np.array(point, dtype=float)
        t = float(np.dot(pt - p1, seg_vec) / (seg_len ** 2))
        t = max(0.0, min(1.0, t))
        closest = p1 + t * seg_vec
        dist = float(np.linalg.norm(pt - closest))
        along = arc + t * seg_len

        if best is None or dist < float(best["distance"]):
            best = {
                "snapped_coord": (float(closest[0]), float(closest[1])),
                "distance": dist,
                "distance_along_edge": float(along),
                "segment_index": int(i),
            }
        arc += seg_len

    return best


def snap_point_to_topo_graph(
    point: Sequence[float],
    topo_graph,
    *,
    max_distance: float = 80.0,
) -> Optional[Dict[str, Any]]:
    """Snap a point to the nearest topo edge polyline."""
    best: Optional[Dict[str, Any]] = None
    for edge_id, edge in topo_graph.edges.items():
        proj = project_point_to_polyline(point, edge.polyline)
        if proj is None:
            continue
        if best is None or float(proj["distance"]) < float(best["distance"]):
            best = {
                **proj,
                "edge_id": edge_id,
                "line_id": edge.line_id,
            }

    if best is None:
        return None
    if float(best["distance"]) > float(max_distance):
        return None
    return best


def map_points_to_edges(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> Dict[str, List[Dict]]:
    """
    将巡检点映射到对应的拓扑边
    """
    print("[点边映射] 开始将巡检点映射到拓扑边...")

    edge_points = {edge_id: [] for edge_id in topo_graph.edges}

    line_to_edges: Dict[str, List[Tuple[str, Any]]] = {}
    for edge_id, edge in topo_graph.edges.items():
        line_to_edges.setdefault(edge.line_id, []).append((edge_id, edge))

    for line_id, points in line_inspection_points_by_line.items():
        candidates = line_to_edges.get(line_id, [])
        if not candidates or not points:
            continue

        for point in points:
            if hasattr(point, "pixel_position"):
                pt_pos = point.pixel_position
                point_type = getattr(point, "point_type", "unknown")
                source_reason = getattr(point, "source_reason", "")
            elif isinstance(point, dict):
                pt_pos = point.get("pixel_position") or point.get("pos2d") or point.get("position")
                point_type = point.get("point_type", point.get("type", "unknown"))
                source_reason = point.get("source_reason", "")
            else:
                pt_pos = None
                point_type = "unknown"
                source_reason = ""

            if pt_pos is None:
                continue

            if point_type == "image_detected":
                snap_threshold = 80.0
            else:
                snap_threshold = 10.0
            best_edge_id = None
            best_dist = float("inf")

            for edge_id, edge in candidates:
                polyline = np.array(edge.polyline)
                if len(polyline) < 2:
                    continue
                for i in range(len(polyline) - 1):
                    p1 = polyline[i]
                    p2 = polyline[i + 1]
                    dist = point_to_line_segment_distance(pt_pos, p1, p2)
                    if dist < best_dist:
                        best_dist = dist
                        best_edge_id = edge_id

            if best_edge_id is not None and best_dist < snap_threshold:
                edge_points[best_edge_id].append(point)
                print(
                    f"[DEBUG] visit target point_type={point_type} mapped_edge={best_edge_id} "
                    f"coord=({pt_pos[0]:.2f},{pt_pos[1]:.2f}) reason={source_reason or 'nearest_edge'}"
                )

    total_mapped = sum(len(pts) for pts in edge_points.values())
    print(f"  [点边映射] 完成: {len(edge_points)} 条边, {total_mapped} 个巡检点")

    return edge_points


# =====================================================
# 边任务构建
# =====================================================

def build_edge_tasks(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> List[EdgeTask]:
    """
    为每条拓扑边构建 EdgeTask

    Args:
        topo_graph: 拓扑图
        line_inspection_points_by_line: {line_id: [巡检点列表]}

    Returns:
        List[EdgeTask]: 边任务列表
    """
    print("[边任务构建] 开始构建边任务...")

    # 先映射点到边
    edge_points = map_points_to_edges(topo_graph, line_inspection_points_by_line)

    # 构建EdgeTask列表
    edge_tasks = []

    for edge_id, edge in topo_graph.edges.items():
        points = edge_points.get(edge_id, [])

        from core.image_pixel_coords import edge_pixel_polyline, freeze_pixel_polyline

        px = edge_pixel_polyline(edge) or freeze_pixel_polyline(edge.polyline)
        task = EdgeTask(
            edge_id=edge_id,
            u=edge.u,
            v=edge.v,
            line_id=edge.line_id,
            polyline=px,
            pixel_polyline=list(px),
            original_polyline=list(px),
            image_polyline=list(px),
            len2d=edge.len2d,
            inspection_points=points,
            num_points=len(points),
            is_straight=edge.is_straight,
            split_reason=edge.split_reason
        )

        edge_tasks.append(task)

    print(f"  [边任务构建] 完成: {len(edge_tasks)} 个任务")

    return edge_tasks


@dataclass
class LineTask:
    """
    物理线路任务：同一 line_id 下若干 EdgeTask 聚合成一条可巡检折线。
    """

    line_id: str
    edge_ids: List[str]
    polyline: List[Tuple[float, float]]
    inspection_points: List[Any]
    num_points: int
    s_min: float
    s_max: float
    len2d: float = 0.0
    rep_start_edge_id: str = ""
    rep_end_edge_id: str = ""


def _edge_task_topo_index(task: EdgeTask) -> Tuple[int, str]:
    m = re.search(r"_edge_(\d+)$", task.edge_id or "")
    if m:
        return (int(m.group(1)), task.edge_id)
    return (10**9, task.edge_id)


def _inspection_point_xy(point: Any) -> Optional[Tuple[float, float]]:
    if hasattr(point, "pixel_position"):
        pos = point.pixel_position
    elif isinstance(point, dict):
        pos = point.get("pixel_position") or point.get("pos2d") or point.get("position")
    else:
        pos = None
    if not pos or len(pos) < 2:
        return None
    return float(pos[0]), float(pos[1])


def _stitch_edge_task_polylines(tasks_sorted: List[EdgeTask], tol_join: float = 6.0) -> List[Tuple[float, float]]:
    from core.image_pixel_coords import edge_pixel_polyline

    merged: List[Tuple[float, float]] = []
    for t in tasks_sorted:
        pl = edge_pixel_polyline(t)
        if len(pl) < 2:
            continue
        if not merged:
            merged.extend(pl)
            continue
        last = np.array(merged[-1], dtype=np.float64)
        head = np.array(pl[0], dtype=np.float64)
        tail = np.array(pl[-1], dtype=np.float64)
        if float(np.linalg.norm(last - head)) <= tol_join:
            merged.extend(pl[1:])
        elif float(np.linalg.norm(last - tail)) <= tol_join:
            rpl = list(reversed(pl))
            merged.extend(rpl[1:])
        else:
            return []
    return merged


def build_line_tasks_from_edge_tasks(edge_tasks: List[EdgeTask]) -> List[LineTask]:
    """
    将 EdgeTask 按 line_id 聚合成 LineTask（含完整像素折线与按弧长排序的巡检点）。
    """
    by_line: Dict[str, List[EdgeTask]] = {}
    for t in edge_tasks:
        lid = getattr(t, "line_id", None) or ""
        if not lid:
            continue
        by_line.setdefault(lid, []).append(t)

    out: List[LineTask] = []
    from core.image_pixel_coords import edge_pixel_polyline
    from core.topo_plan import _polyline_length, _slice_polyline_by_distance

    for lid in sorted(by_line.keys()):
        tasks = by_line[lid]
        tasks_sorted = sorted(tasks, key=_edge_task_topo_index)
        edge_ids = [x.edge_id for x in tasks_sorted]

        merged = _stitch_edge_task_polylines(tasks_sorted)
        if len(merged) < 2:
            pl0 = edge_pixel_polyline(tasks_sorted[0]) if tasks_sorted else []
            merged = list(pl0) if len(pl0) >= 2 else []

        if len(merged) < 2:
            continue

        all_pts: List[Any] = []
        for t in tasks_sorted:
            pts = getattr(t, "inspection_points", None) or []
            all_pts.extend(pts)

        decorated: List[Tuple[float, Any]] = []
        for p in all_pts:
            xy = _inspection_point_xy(p)
            if xy is None:
                continue
            meta = project_point_to_polyline(xy, merged)
            if meta is None:
                continue
            if float(meta["distance"]) > 80.0:
                continue
            decorated.append((float(meta["distance_along_edge"]), p))

        decorated.sort(key=lambda x: x[0])
        if not decorated:
            out.append(
                LineTask(
                    line_id=lid,
                    edge_ids=edge_ids,
                    polyline=[tuple(x) for x in merged],
                    inspection_points=[],
                    num_points=0,
                    s_min=0.0,
                    s_max=0.0,
                    len2d=float(_polyline_length(merged)),
                    rep_start_edge_id=edge_ids[0] if edge_ids else "",
                    rep_end_edge_id=edge_ids[-1] if edge_ids else "",
                )
            )
            continue

        s_vals = [d[0] for d in decorated]
        s_min, s_max = min(s_vals), max(s_vals)
        ordered_pts = [d[1] for d in decorated]
        total_len = float(_polyline_length(merged))
        seg = _slice_polyline_by_distance(merged, s_min, s_max)
        seg_len = float(_polyline_length(seg)) if len(seg) >= 2 else 0.0

        print(
            f"[line-task] line_id={lid} edge_count={len(edge_ids)} point_count={len(ordered_pts)} "
            f"s_min={s_min:.1f} s_max={s_max:.1f} length={seg_len:.1f}"
        )

        out.append(
            LineTask(
                line_id=lid,
                edge_ids=edge_ids,
                polyline=[tuple(x) for x in merged],
                inspection_points=ordered_pts,
                num_points=len(ordered_pts),
                s_min=float(s_min),
                s_max=float(s_max),
                len2d=total_len,
                rep_start_edge_id=edge_ids[0] if edge_ids else "",
                rep_end_edge_id=edge_ids[-1] if edge_ids else "",
            )
        )

    return out


def summarize_edge_tasks(edge_tasks: List[EdgeTask]) -> Dict:
    """
    统计边任务信息

    Args:
        edge_tasks: 边任务列表

    Returns:
        Dict: 统计信息
    """
    total_edges = len(edge_tasks)
    total_points = sum(task.num_points for task in edge_tasks)
    total_length = sum(task.len2d for task in edge_tasks)

    # 按线路统计
    line_split_info = {}
    for task in edge_tasks:
        line_id = task.line_id
        line_split_info[line_id] = line_split_info.get(line_id, 0) + 1

    # 点数分布
    points_distribution = {}
    for task in edge_tasks:
        points_distribution[task.edge_id] = task.num_points

    # 直线/曲线分布
    straight_edges = sum(1 for task in edge_tasks if task.is_straight)
    curved_edges = total_edges - straight_edges

    stats = {
        'total_edges': total_edges,
        'total_points': total_points,
        'total_length': total_length,
        'avg_points_per_edge': total_points / total_edges if total_edges > 0 else 0,
        'avg_length': total_length / total_edges if total_edges > 0 else 0,
        'line_split_info': line_split_info,
        'points_distribution': points_distribution,
        'straight_edges': straight_edges,
        'curved_edges': curved_edges
    }

    return stats


# =====================================================
# 可视化
# =====================================================

def visualize_edge_numbers(topo_graph, line_inspection_points_by_line: Dict[str, List],
                            output_path: str):
    """
    可视化拓扑边及其巡检点

    Args:
        topo_graph: 拓扑图
        line_inspection_points_by_line: {line_id: [巡检点列表]}
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(14, 14))

    # 绘制拓扑边
    for edge in topo_graph.edges.values():
        polyline = np.array(edge.polyline)
        color = 'blue' if edge.is_straight else 'green'
        ax.plot(polyline[:, 0], polyline[:, 1], color=color, linewidth=2, alpha=0.7)

        # 显示边编号
        mid_idx = len(polyline) // 2
        mid_x, mid_y = polyline[mid_idx]
        edge_num = edge.id.split('_')[-1]
        ax.text(mid_x, mid_y, edge_num, fontsize=8, ha='center', va='center',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # 绘制巡检点
    for line_id, points in line_inspection_points_by_line.items():
        for point in points:
            # 获取位置 (LineInspectionPoint dataclass)
            if hasattr(point, 'pixel_position'):
                pos = point.pixel_position
                kind = point.point_type if hasattr(point, 'point_type') else 'middle'
            elif isinstance(point, dict):
                pos = point.get('pos2d', point.get('position', None))
                kind = point.get('kind', 'middle')
            else:
                continue

            if pos is None:
                continue

            if kind == 'endpoint' or kind == 'start':
                color = 'red'
                marker = 'o'
                size = 30
            elif kind == 'end':
                color = 'blue'
                marker = 'o'
                size = 30
            else:  # middle / turning / sample
                color = 'orange'
                marker = '.'
                size = 10

            ax.scatter(pos[0], pos[1], c=color, marker=marker, s=size, zorder=5)

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'purple'
            marker = 's'
            size = 50
        elif node.kind == 'split':
            color = 'darkorange'
            marker = '^'
            size = 60
        else:
            color = 'gray'
            marker = 'x'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=6,
                  edgecolors='black', linewidths=1)

    # 图例
    patches = [
        mpatches.Patch(color='blue', label='Straight Edge'),
        mpatches.Patch(color='green', label='Curved Edge'),
        mpatches.Patch(color='red', label='Endpoint/Turning Point'),
        mpatches.Patch(color='orange', label='Sample Point'),
        mpatches.Patch(color='purple', label='Endpoint Node'),
        mpatches.Patch(color='darkorange', label='Split Node')
    ]
    ax.legend(handles=patches, loc='upper right')

    ax.set_title('Topology Edges with Inspection Points')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 边任务图已保存: {output_path}")


def visualize_edge_task_summary(edge_tasks: List[EdgeTask], output_path: str):
    """
    可视化边任务统计信息

    Args:
        edge_tasks: 边任务列表
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 子图1: 每条边的点数分布
    ax1 = axes[0]
    edge_ids = [task.edge_id.split('_')[-1] for task in edge_tasks]
    point_counts = [task.num_points for task in edge_tasks]

    colors = ['green' if task.is_straight else 'orange' for task in edge_tasks]
    ax1.bar(range(len(edge_ids)), point_counts, color=colors, alpha=0.7)
    ax1.set_xlabel('Edge ID')
    ax1.set_ylabel('Number of Inspection Points')
    ax1.set_title('Inspection Points per Edge')
    ax1.set_xticks(range(len(edge_ids)))
    ax1.set_xticklabels(edge_ids, rotation=45, ha='right')

    # 子图2: 边长度分布
    ax2 = axes[1]
    edge_lengths = [task.len2d for task in edge_tasks]
    ax2.bar(range(len(edge_ids)), edge_lengths, color=colors, alpha=0.7)
    ax2.set_xlabel('Edge ID')
    ax2.set_ylabel('Length (px)')
    ax2.set_title('Edge Length Distribution')
    ax2.set_xticks(range(len(edge_ids)))
    ax2.set_xticklabels(edge_ids, rotation=45, ha='right')

    # 图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', alpha=0.7, label='Straight Edge'),
        Patch(facecolor='orange', alpha=0.7, label='Curved Edge')
    ]
    ax2.legend(handles=legend_elements, loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 统计图已保存: {output_path}")
