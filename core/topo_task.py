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
from typing import Any, List, Tuple, Dict, Optional, Sequence, Set
from collections import defaultdict
import re
import numpy as np


# =====================================================
# 数据结构
# =====================================================

@dataclass
class EdgeTask:
    """
    连续物理线路段巡检任务：同一 line_id 下若干连通 TopoEdge 合并为一条链，
    对上层规划器表现为单个任务单元（edge_id 形如 L_xxx_chain_0）。
    meta['chain_topo_edge_ids'] 记录组成链的原始拓扑边 ID。
    """
    edge_id: str                          # 合并链任务 ID（稳定命名）
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


CHAIN_TAG = "_chain_"


def legacy_topo_edge_id_to_chain_map(edge_task_map: Dict[str, EdgeTask]) -> Dict[str, str]:
    """碎片 topo edge_id -> 合并后 EdgeTask.edge_id（供 replan / 旧 JSON 兼容）。"""
    out: Dict[str, str] = {}
    for cid, et in edge_task_map.items():
        for tid in (et.meta or {}).get("chain_topo_edge_ids") or []:
            out[str(tid)] = str(cid)
    return out


def _polyline_len2d(poly: List[Tuple[float, float]]) -> float:
    if len(poly) < 2:
        return 0.0
    return float(
        sum(
            float(np.linalg.norm(np.array(poly[i + 1]) - np.array(poly[i])))
            for i in range(len(poly) - 1)
        )
    )


def _connected_topo_edge_components(edge_ids: List[str], topo_graph) -> List[List[str]]:
    """按端点连通性将同一集合内的拓扑边划分为连通分量。"""
    sset = set(edge_ids)
    visited: Set[str] = set()
    comps: List[List[str]] = []
    for start in edge_ids:
        if start not in sset or start in visited:
            continue
        stack = [start]
        comp: List[str] = []
        while stack:
            eid = stack.pop()
            if eid in visited or eid not in sset:
                continue
            visited.add(eid)
            comp.append(eid)
            e = topo_graph.edges.get(eid)
            if not e:
                continue
            for nbr_eid in sset:
                if nbr_eid in visited:
                    continue
                nb = topo_graph.edges.get(nbr_eid)
                if not nb or nb.line_id != e.line_id:
                    continue
                if nb.u == e.u or nb.v == e.u or nb.u == e.v or nb.v == e.v:
                    stack.append(nbr_eid)
        if comp:
            comps.append(comp)
    return comps


def _order_component_chain(comp: List[str], topo_graph) -> List[str]:
    """将连通分量内的拓扑边排成一条路径（端点度为 1 的链）。"""
    if len(comp) <= 1:
        return list(comp)
    sub_deg: Dict[str, int] = defaultdict(int)
    adj_n: Dict[str, List[Tuple[str, str]]] = defaultdict(list)
    for eid in comp:
        e = topo_graph.edges.get(eid)
        if not e:
            continue
        adj_n[e.u].append((e.v, eid))
        adj_n[e.v].append((e.u, eid))
        sub_deg[e.u] += 1
        sub_deg[e.v] += 1
    leaves = [n for n, d in sub_deg.items() if d == 1]
    start_node = leaves[0] if leaves else topo_graph.edges[comp[0]].u
    ordered: List[str] = []
    visited_e: Set[str] = set()
    curr = start_node
    prev: Optional[str] = None
    guard = 0
    while guard < len(comp) * 6 + 20:
        guard += 1
        cand = [(n, eid) for n, eid in adj_n[curr] if eid in comp and eid not in visited_e]
        if not cand:
            break
        chosen = None
        for n, eid in cand:
            if n != prev:
                chosen = (n, eid)
                break
        if chosen is None:
            chosen = cand[0]
        nxt, eid = chosen
        ordered.append(eid)
        visited_e.add(eid)
        prev, curr = curr, nxt
        if len(visited_e) == len(comp):
            break
    for eid in comp:
        if eid not in visited_e:
            ordered.append(eid)
    return ordered


def _stitch_topo_chain_polyline(
    chain_ids: List[str],
    topo_graph,
    tol_join: float = 6.0,
) -> List[Tuple[float, float]]:
    """按链顺序拼接各 TopoEdge 的像素折线，去重端点。"""
    from core.image_pixel_coords import freeze_pixel_polyline

    merged: List[Tuple[float, float]] = []
    for eid in chain_ids:
        e = topo_graph.edges.get(eid)
        if not e:
            continue
        pl = freeze_pixel_polyline(e.polyline)
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
            rpl = list(reversed(pl))
            rh = np.array(rpl[0], dtype=np.float64)
            if float(np.linalg.norm(last - rh)) <= tol_join:
                merged.extend(rpl[1:])
            else:
                merged.extend(pl[1:])
    return merged


def _chain_endpoint_nodes_uv(
    chain_ids: List[str],
    merged_poly: List[Tuple[float, float]],
    topo_graph,
) -> Tuple[str, str]:
    """链在拓扑图上的两个端点节点（与 merged_poly 首尾一致）。"""
    if not chain_ids or len(merged_poly) < 1:
        return "", ""
    ps = np.array(merged_poly[0], dtype=np.float64)
    pe = np.array(merged_poly[-1], dtype=np.float64)

    def _closer_node(eid: str, end_pt: np.ndarray) -> Optional[str]:
        e = topo_graph.edges.get(eid)
        if not e:
            return None
        nu = topo_graph.nodes.get(e.u)
        nv = topo_graph.nodes.get(e.v)
        if not nu or not nv:
            return e.u
        pu = np.array(nu.pos2d, dtype=np.float64)
        pv = np.array(nv.pos2d, dtype=np.float64)
        return e.u if float(np.linalg.norm(end_pt - pu)) <= float(np.linalg.norm(end_pt - pv)) else e.v

    u0 = _closer_node(chain_ids[0], ps) or ""
    v1 = _closer_node(chain_ids[-1], pe) or ""
    if u0 and v1 and u0 != v1:
        return u0, v1
    e0 = topo_graph.edges.get(chain_ids[0])
    if e0:
        return e0.u, e0.v
    return "", ""


def _build_line_chain_specs(
    topo_graph,
    line_id: str,
    *,
    log_merge: bool = True,
) -> List[Tuple[str, List[str], List[Tuple[float, float]], str, str]]:
    """
    返回 [(chain_edge_id, ordered_topo_ids, merged_poly, u, v), ...]
    """
    raw_ids = [eid for eid, e in topo_graph.edges.items() if e.line_id == line_id]
    if not raw_ids:
        return []
    comps = _connected_topo_edge_components(raw_ids, topo_graph)
    out: List[Tuple[str, List[str], List[Tuple[float, float]], str, str]] = []
    lengths: List[float] = []
    for ci, comp in enumerate(comps):
        ordered = _order_component_chain(comp, topo_graph)
        merged = _stitch_topo_chain_polyline(ordered, topo_graph)
        if len(merged) < 2:
            continue
        uu, vv = _chain_endpoint_nodes_uv(ordered, merged, topo_graph)
        cid = f"{line_id}{CHAIN_TAG}{ci}"
        out.append((cid, ordered, merged, uu, vv))
        lengths.append(round(_polyline_len2d(merged), 1))
    if log_merge:
        print(
            f"[edge-merge] line_id={line_id} raw_edges={len(raw_ids)} chains={len(out)} "
            f"chain_lengths={lengths}"
        )
    return out


def _enrich_point_on_chain(
    point: Any,
    chain_edge_id: str,
    line_id: str,
    proj: Dict[str, Any],
    merged_len: float,
) -> None:
    s = float(proj["distance_along_edge"])
    tnorm = float(s / merged_len) if merged_len > 1e-6 else 0.0
    if isinstance(point, dict):
        point["edge_id"] = chain_edge_id
        point["line_id"] = line_id
        point["distance_along_edge"] = s
        point["projected_t"] = tnorm
    else:
        try:
            setattr(point, "edge_id", chain_edge_id)
            setattr(point, "line_id", line_id)
            setattr(point, "distance_along_edge", s)
            setattr(point, "projected_t", tnorm)
        except Exception:
            pass


def map_points_to_edges(
    topo_graph,
    line_inspection_points_by_line: Dict[str, List],
    chain_specs_by_line: Optional[Dict[str, List[Tuple[str, List[str], List[Tuple[float, float]], str, str]]]] = None,
) -> Dict[str, List[Any]]:
    """
    将巡检点映射到合并链 EdgeTask（edge_id = line_id_chain_i）对应的折线上。
    若传入 chain_specs_by_line 则复用（避免重复计算与重复日志）。
    """
    print("[点边映射] 开始将巡检点映射到合并链任务...")

    if chain_specs_by_line is None:
        chain_specs_by_line = {
            lid: _build_line_chain_specs(topo_graph, lid, log_merge=True)
            for lid in sorted({e.line_id for e in topo_graph.edges.values()})
        }

    edge_points: Dict[str, List[Any]] = defaultdict(list)
    point_map_logs = 0
    point_map_limit = 60

    for line_id, points in line_inspection_points_by_line.items():
        chains = chain_specs_by_line.get(line_id, [])
        if not chains or not points:
            continue

        for point in points:
            if hasattr(point, "pixel_position"):
                pt_pos = point.pixel_position
                point_type = getattr(point, "point_type", "unknown")
            elif isinstance(point, dict):
                pt_pos = point.get("pixel_position") or point.get("pos2d") or point.get("position")
                point_type = point.get("point_type", point.get("type", "unknown"))
            else:
                pt_pos = None
                point_type = "unknown"

            if pt_pos is None or len(pt_pos) < 2:
                continue

            snap_threshold = 80.0 if point_type == "image_detected" else 10.0
            best_cid = None
            best_proj = None
            best_dist = float("inf")

            for cid, _ordered, merged, _uu, _vv in chains:
                if len(merged) < 2:
                    continue
                proj = project_point_to_polyline(pt_pos, merged)
                if proj is None:
                    continue
                d = float(proj["distance"])
                if d < best_dist:
                    best_dist = d
                    best_cid = cid
                    best_proj = proj

            if best_cid is not None and best_proj is not None and best_dist < snap_threshold:
                merged_sel = next(c[2] for c in chains if c[0] == best_cid)
                mlen = _polyline_len2d(merged_sel)
                _enrich_point_on_chain(point, best_cid, line_id, best_proj, mlen)
                edge_points[best_cid].append(point)
                if point_map_logs < point_map_limit:
                    pid = (
                        point.get("point_id", point.get("id", "?"))
                        if isinstance(point, dict)
                        else getattr(point, "point_id", getattr(point, "id", "?"))
                    )
                    print(
                        f"[point-map] point_id={pid} line_id={line_id} edge_id={best_cid} "
                        f"distance_along_edge={float(best_proj['distance_along_edge']):.2f}"
                    )
                    point_map_logs += 1

    total_mapped = sum(len(pts) for pts in edge_points.values())
    print(f"  [点边映射] 完成: {len(edge_points)} 条合并链, {total_mapped} 个巡检点")

    return dict(edge_points)


def build_edge_tasks(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> List[EdgeTask]:
    """
    按 line_id 连通链构建合并 EdgeTask（每链一个任务，仅包含有巡检点的链）。
    """
    print("[边任务构建] 开始构建合并链边任务...")

    line_ids = sorted({e.line_id for e in topo_graph.edges.values()})
    chain_specs_by_line = {
        lid: _build_line_chain_specs(topo_graph, lid, log_merge=True) for lid in line_ids
    }
    edge_points = map_points_to_edges(
        topo_graph, line_inspection_points_by_line, chain_specs_by_line=chain_specs_by_line
    )

    edge_tasks: List[EdgeTask] = []
    merged_topo_count = 0
    lines_with_tasks: Set[str] = set()

    for line_id in line_ids:
        for cid, ordered, merged, uu, vv in chain_specs_by_line.get(line_id, []):
            pts = edge_points.get(cid, [])
            if not pts:
                continue
            pts_sorted = sorted(
                pts,
                key=lambda p: float(
                    p.get("distance_along_edge", 0.0)
                    if isinstance(p, dict)
                    else getattr(p, "distance_along_edge", 0.0)
                ),
            )
            merged_len = _polyline_len2d(merged)
            straight = all(
                (topo_graph.edges[eid].is_straight for eid in ordered if eid in topo_graph.edges)
            ) if ordered else True
            task = EdgeTask(
                edge_id=cid,
                u=uu or (topo_graph.edges[ordered[0]].u if ordered else ""),
                v=vv or (topo_graph.edges[ordered[-1]].v if ordered else ""),
                line_id=line_id,
                polyline=list(merged),
                pixel_polyline=list(merged),
                original_polyline=list(merged),
                image_polyline=list(merged),
                len2d=float(merged_len),
                inspection_points=pts_sorted,
                num_points=len(pts_sorted),
                is_straight=bool(straight),
                split_reason="merged_chain",
                meta={"chain_topo_edge_ids": list(ordered)},
            )
            edge_tasks.append(task)
            merged_topo_count += len(ordered)
            lines_with_tasks.add(line_id)

    total_topo = len(topo_graph.edges)
    print(
        f"[edge-task] total_edge_tasks={len(edge_tasks)} total_lines={len(lines_with_tasks)} "
        f"merged_from_topo_edges={merged_topo_count} topo_edges_total={total_topo}"
    )
    print(f"  [边任务构建] 完成: {len(edge_tasks)} 个合并链任务")

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
    m = re.search(r"_chain_(\d+)$", task.edge_id or "")
    if m:
        return (int(m.group(1)), task.edge_id)
    m2 = re.search(r"_edge_(\d+)$", task.edge_id or "")
    if m2:
        return (int(m2.group(1)), task.edge_id)
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
