"""
=====================================================
全局拓扑路径优化器
=====================================================

基于《配电网无人机巡检路径优化算法》论文思想实现：
1. 拓扑感知的任务组织
2. 起始接入点优化
3. 全局顺序优化（非贪心）
4. 综合代价函数

核心改进：
- 从"几何距离贪心"升级为"拓扑结构感知的全局优化"
- Connect 代价综合考虑：几何距离、拓扑路径、方向变化、切换代价
- 起始点选择：枚举候选，选择总代价最低
- 顺序优化：模拟退火算法
"""

from typing import List, Dict, Tuple, Optional, Any, Set
import numpy as np
from dataclasses import dataclass
import random
from collections import defaultdict

from core.topo import TopoGraph, TopoNode
from core.topo_task import EdgeTask, LineTask, project_point_to_polyline
from core.image_pixel_coords import edge_pixel_polyline
from core.topo_plan import (
    EdgeGroup, GroupedContinuousMission, MissionSegment,
    build_edge_adjacency_simple, compute_transition_cost_simple,
    compute_edge_centroids,
    group_edges_spatially,
    get_edge_geometry_with_direction, interpolate_geometry,
    get_edge_inspection_geometry_with_direction,
    generate_connection_segment_along_topo,
    _slice_polyline_by_distance,
    _polyline_length,
)


# =====================================================
# 代价模型：综合多种因素
# =====================================================

@dataclass
class ConnectionCost:
    """连接代价模型"""
    geometric_distance: float  # 几何直线距离
    topo_path_length: float     # 拓扑图上的最短路径长度
    direction_change: float     # 方向变化角度（0-1）
    group_switch_penalty: float # group切换惩罚
    total_cost: float           # 综合总代价


def _merged_pixel_polyline_same_line_follow_visit(
    edge_order: List[str],
    edge_task_map: Dict[str, EdgeTask],
    line_id: str,
    tol_join: float = 6.0,
) -> List[Tuple[float, float]]:
    """
    按 visit_order 中该 line_id 出现顺序，将各 EdgeTask 的像素折线缝合成一条参考折线。
    若相邻任务无法在端点衔接，返回空列表（由调用方 fallback）。
    """
    merged: List[Tuple[float, float]] = []
    for eid in edge_order:
        t = edge_task_map.get(eid)
        if not t or (getattr(t, "line_id", "") or "") != line_id:
            continue
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
    return merged if len(merged) >= 2 else []


def _try_same_line_connect_geometry(
    edge_order: List[str],
    edge_task_map: Dict[str, EdgeTask],
    line_id: str,
    p_from: Tuple[float, float],
    p_to: Tuple[float, float],
    max_snap: float = 45.0,
) -> Optional[List[Tuple[float, float]]]:
    """
    在参考折线上截取从上一条 inspect 终点到下一条 inspect 起点的子路径。
    失败返回 None（调用方使用原有 topo connect）。
    """
    merged = _merged_pixel_polyline_same_line_follow_visit(
        edge_order, edge_task_map, line_id
    )
    if len(merged) < 2:
        return None

    pa = project_point_to_polyline(p_from, merged)
    pb = project_point_to_polyline(p_to, merged)
    if (
        pa is None
        or pb is None
        or float(pa["distance"]) > max_snap
        or float(pb["distance"]) > max_snap
    ):
        return None

    sa = float(pa["distance_along_edge"])
    sb = float(pb["distance_along_edge"])
    if sa <= sb:
        raw = _slice_polyline_by_distance(merged, sa, sb)
    else:
        raw = list(reversed(_slice_polyline_by_distance(merged, sb, sa)))

    if len(raw) < 2:
        raw = [tuple(p_from), tuple(p_to)]
    else:
        raw = [tuple(p_from)] + raw[1:-1] + [tuple(p_to)]

    if len(raw) < 2:
        return None
    ln = _polyline_length(raw)
    if ln < 1e-3:
        return None
    return raw


def _line_point_totals(edge_tasks: List[EdgeTask]) -> Dict[str, int]:
    """每条物理线路 line_id 上巡检点总数（跨多个 EdgeTask / PhysicalLineChain）。"""
    out: Dict[str, int] = {}
    for t in edge_tasks:
        pts = list(getattr(t, "inspection_points", None) or [])
        if pts:
            for p in pts:
                lid = ""
                if isinstance(p, dict):
                    lid = str(p.get("line_id") or "")
                else:
                    lid = str(getattr(p, "line_id", "") or "")
                if lid:
                    out[lid] = out.get(lid, 0) + 1
            continue
        lid = getattr(t, "line_id", None) or ""
        if not lid:
            continue
        n = int(getattr(t, "num_points", 0) or 0)
        if n <= 0 and getattr(t, "inspection_points", None):
            n = len(t.inspection_points)
        out[lid] = out.get(lid, 0) + max(0, n)
    return out


def _apply_edge_visit_line_counts(line_visited: Dict[str, int], edge: EdgeTask) -> None:
    pts = list(getattr(edge, "inspection_points", None) or [])
    if pts:
        for p in pts:
            lid = ""
            if isinstance(p, dict):
                lid = str(p.get("line_id") or "")
            else:
                lid = str(getattr(p, "line_id", "") or "")
            if lid:
                line_visited[lid] = line_visited.get(lid, 0) + 1
        return
    lid = getattr(edge, "line_id", None) or ""
    if not lid:
        return
    n = int(getattr(edge, "num_points", 0) or 0)
    if n <= 0 and getattr(edge, "inspection_points", None):
        n = len(edge.inspection_points)
    line_visited[lid] = line_visited.get(lid, 0) + max(0, n)


def _newly_completed_lines(
    line_visited: Dict[str, int],
    line_totals: Dict[str, int],
    already: Set[str],
) -> List[str]:
    done: List[str] = []
    for lid, tot in line_totals.items():
        if tot <= 0 or lid in already:
            continue
        if line_visited.get(lid, 0) >= tot:
            done.append(lid)
    return done


def _find_topo_edge_by_uv(topo_graph: TopoGraph, u: str, v: str):
    for e in topo_graph.edges.values():
        if (e.u == u and e.v == v) or (e.u == v and e.v == u):
            return e
    return None


def _union_topo_edges_for_completed_chains(
    edge_task_map: Dict[str, EdgeTask],
    completed_chain_edge_ids: Set[str],
) -> Set[str]:
    """已巡检完成的 chain（EdgeTask.edge_id）所覆盖的 TopoEdge.id 并集。"""
    out: Set[str] = set()
    for cid in completed_chain_edge_ids:
        t = edge_task_map.get(cid)
        if not t:
            continue
        meta = getattr(t, "meta", None) or {}
        for teid in meta.get("chain_topo_edge_ids") or []:
            out.add(str(teid))
    return out


def _union_topo_edges_for_completed_lines(
    topo_graph: TopoGraph,
    completed_line_ids: Set[str],
) -> Set[str]:
    """已完成线路上的全部 TopoEdge.id（线路级任务用）。"""
    out: Set[str] = set()
    if not completed_line_ids:
        return out
    for e in topo_graph.edges.values():
        lid = getattr(e, "line_id", "") or ""
        if lid and lid in completed_line_ids:
            out.add(str(e.id))
    return out


def _tasks_share_line(prev_t: Any, cur_t: Any) -> bool:
    """两条 Mission 边任务是否共享任一 line_id（支持 PhysicalLineChain.line_ids）。"""
    la = getattr(prev_t, "line_ids", None)
    lb = getattr(cur_t, "line_ids", None)
    if la and lb and set(la) & set(lb):
        return True
    a = getattr(prev_t, "line_id", "") or ""
    b = getattr(cur_t, "line_id", "") or ""
    return bool(a and a == b)


def _edge_mean_inspection_arc_s(edge_task: EdgeTask) -> Optional[float]:
    """同 line_id 内边序惩罚用：巡检点在边上的平均弧长坐标。"""
    try:
        from core.image_pixel_coords import edge_pixel_polyline
        from core.topo_plan import _point_xy_from_inspection_point, _project_point_to_polyline_distance
    except Exception:
        return None
    pts = list(getattr(edge_task, "inspection_points", None) or [])
    if not pts:
        return None
    poly = edge_pixel_polyline(edge_task)
    if len(poly) < 2:
        return None
    ss: List[float] = []
    for p in pts:
        pos = _point_xy_from_inspection_point(p)
        if pos is None:
            continue
        s = _project_point_to_polyline_distance(pos, poly)
        if s is not None:
            ss.append(float(s))
    if not ss:
        return None
    return float(sum(ss) / len(ss))


def _edge_mean_arc_map(edge_task_map: Dict[str, EdgeTask]) -> Dict[str, Optional[float]]:
    return {eid: _edge_mean_inspection_arc_s(t) for eid, t in edge_task_map.items()}


def _print_route_line_bootstrap(edge_tasks: List[EdgeTask], line_totals: Dict[str, int]) -> None:
    edges_per: Dict[str, int] = defaultdict(int)
    for t in edge_tasks:
        lid = getattr(t, "line_id", None) or ""
        if lid:
            edges_per[lid] += 1
    for lid in sorted(line_totals.keys()):
        print(
            f"[route-line] line_id={lid} total_points={line_totals[lid]} "
            f"edges_in_plan={edges_per.get(lid, 0)}"
        )


def _default_order_weights() -> Dict[str, float]:
    """与 evaluate_order_cost 默认权重对齐（供 line 连续性统计复用）。"""
    return {
        "geometric": 1.0,
        "topo": 0.5,
        "direction": 0.3,
        "group_switch": 50.0,
        "completed_line_edge_px": 380.0,
        "completed_line_cross_weight": 1.0,
        "same_line_transition_factor": 0.91,
        "line_reverse_jump": 0.35,
        "line_revisit_penalty": 12000.0,
        "same_line_streak_reward": 40.0,
        "completed_chain_edge_penalty": 400.0,
    }


def _line_continuity_metrics(
    edge_order: List[str],
    edge_task_map: Dict[str, EdgeTask],
    weights: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """
    按 edge_order 中 EdgeTask.line_id 序列：
    - 统计每条 line_id 的连续块数 blocks
    - 离开某 line_id 后再次进入时计 revisit，并累加 line_revisit_penalty
    """
    w = dict(_default_order_weights())
    if weights:
        w.update(weights)

    lines: List[str] = []
    for eid in edge_order:
        t = edge_task_map.get(eid)
        if not t:
            continue
        lines.append(getattr(t, "line_id", "") or "")

    unit = float(w.get("line_revisit_penalty", 12000.0))
    left: Set[str] = set()
    revisit_count = 0
    total_penalty = 0.0
    per_line_revisit_pen: Dict[str, float] = defaultdict(float)

    for i in range(1, len(lines)):
        prev, cur = lines[i - 1], lines[i]
        if prev != cur:
            if cur and cur in left:
                revisit_count += 1
                total_penalty += unit
                per_line_revisit_pen[cur] += unit
            if prev:
                left.add(prev)

    blocks_per_line: Dict[str, int] = defaultdict(int)
    if lines:
        run_lid = lines[0]
        run_len = 1
        for j in range(1, len(lines)):
            if lines[j] == run_lid:
                run_len += 1
            else:
                if run_lid:
                    blocks_per_line[run_lid] += 1
                run_lid = lines[j]
                run_len = 1
        if run_lid:
            blocks_per_line[run_lid] += 1

    return {
        "total_penalty": float(total_penalty),
        "revisit_count": int(revisit_count),
        "blocks_per_line": dict(blocks_per_line),
        "per_line_revisit_pen": dict(per_line_revisit_pen),
    }


def compute_connection_cost_enhanced(
    from_edge_id: str,
    to_edge_id: str,
    from_direction: str,
    to_direction: str,
    from_point: Tuple[float, float],
    to_point: Tuple[float, float],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    weights: Dict[str, float] = None,
    completed_lines: Optional[Set[str]] = None,
    target_line_id: Optional[str] = None,
    completed_topo_edge_ids: Optional[Set[str]] = None,
) -> ConnectionCost:
    """
    计算增强的连接代价

    Args:
        from_edge_id: 起始边ID
        to_edge_id: 目标边ID
        from_direction: 起始边方向
        to_direction: 目标边方向
        from_point: 起始点坐标
        to_point: 目标点坐标
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        groups: 分组信息
        weights: 代价权重配置

    Returns:
        ConnectionCost: 连接代价详情
    """
    if weights is None:
        weights = {
            'geometric': 1.0,      # 几何距离权重
            'topo': 0.1,           # 拓扑路径权重（降低，从0.5改为0.1）
            'direction': 0.1,      # 方向变化权重（降低，从0.3改为0.1）
            'group_switch': 10.0,  # group切换惩罚（降低，从50改为10）
            'completed_line_edge_px': 380.0,
            'completed_line_cross_weight': 1.0,
            'completed_chain_edge_penalty': 400.0,
        }

    # 1. 几何距离
    geo_dist = np.linalg.norm(np.array(to_point) - np.array(from_point))

    # 2. 拓扑路径长度（沿图的最短路径）
    from_edge = edge_task_map.get(from_edge_id)
    to_edge = edge_task_map.get(to_edge_id)

    topo_path_len = 0.0
    path_nodes: List[str] = []
    if from_edge and to_edge:
        # 计算从 from_edge 的终点到 to_edge 的起点的拓扑路径长度
        from_end_node = from_edge.v if from_direction == 'forward' else from_edge.u
        to_start_node = to_edge.u if to_direction == 'forward' else to_edge.v

        if from_end_node != to_start_node:
            # 使用 BFS 找最短路径
            path_nodes = find_topo_path(topo_graph, from_end_node, to_start_node)
            if path_nodes:
                # 计算路径上所有边的长度之和
                topo_path_len = compute_topo_path_length(topo_graph, path_nodes, edge_task_map)
            else:
                # 不连通，使用几何距离
                topo_path_len = geo_dist
        else:
            path_nodes = [from_end_node]

    # 2b. 已完成线路穿越惩罚（沿 BFS 节点路径上的 TopoEdge.line_id）
    cross_penalty_px = 0.0
    if completed_lines and path_nodes and len(path_nodes) >= 2 and to_edge:
        eff_target = target_line_id if target_line_id else (getattr(to_edge, "line_id", "") or "")
        if eff_target:
            w_edge = float(weights.get("completed_line_edge_px", 380.0))
            w_cross = float(weights.get("completed_line_cross_weight", 1.0))
            for ii in range(len(path_nodes) - 1):
                te = _find_topo_edge_by_uv(topo_graph, path_nodes[ii], path_nodes[ii + 1])
                if te is None:
                    continue
                lid = getattr(te, "line_id", "") or ""
                if lid and lid in completed_lines and lid != eff_target:
                    cross_penalty_px += w_edge
            cross_penalty_px *= w_cross

    chain_cross_penalty = 0.0
    ct_set: Optional[Set[str]] = None
    if completed_topo_edge_ids:
        ct_set = {str(x) for x in completed_topo_edge_ids}
    if ct_set and path_nodes and len(path_nodes) >= 2:
        w_chain = float(weights.get("completed_chain_edge_penalty", 400.0))
        for ii in range(len(path_nodes) - 1):
            te = _find_topo_edge_by_uv(topo_graph, path_nodes[ii], path_nodes[ii + 1])
            if te is None:
                continue
            if str(te.id) in ct_set:
                chain_cross_penalty += w_chain

    # 3. 方向变化（0-1，1表示反向）
    direction_change = compute_direction_change_penalty(
        from_point, to_point, from_direction, to_direction,
        edge_task_map.get(from_edge_id), edge_task_map.get(to_edge_id)
    )

    # 4. Group切换惩罚
    group_switch_penalty = 0.0
    from_group = find_edge_group(from_edge_id, groups)
    to_group = find_edge_group(to_edge_id, groups)
    if from_group and to_group and from_group != to_group:
        group_switch_penalty = weights['group_switch']

    # 综合代价
    total_cost = (
        weights['geometric'] * geo_dist +
        weights['topo'] * topo_path_len +
        weights['direction'] * direction_change * 100 +  # 方向变化放大
        group_switch_penalty +
        cross_penalty_px +
        chain_cross_penalty
    )

    return ConnectionCost(
        geometric_distance=geo_dist,
        topo_path_length=topo_path_len,
        direction_change=direction_change,
        group_switch_penalty=group_switch_penalty,
        total_cost=total_cost
    )


def find_topo_path(topo_graph: TopoGraph, start_node: str, end_node: str) -> List[str]:
    """BFS 找拓扑路径"""
    if start_node == end_node:
        return [start_node]

    from collections import deque
    visited = {start_node: None}
    queue = deque([start_node])

    while queue:
        curr = queue.popleft()
        if curr == end_node:
            # 重建路径
            path = []
            while curr is not None:
                path.append(curr)
                curr = visited[curr]
            return path[::-1]

        for neighbor in topo_graph.get_neighbors(curr):
            if neighbor not in visited:
                visited[neighbor] = curr
                queue.append(neighbor)

    return []


def compute_topo_path_length(topo_graph: TopoGraph, path: List[str],
                            edge_task_map: Dict[str, EdgeTask]) -> float:
    """计算拓扑路径的实际长度"""
    if len(path) < 2:
        return 0.0

    total_length = 0.0
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]

        # 找到连接 u 和 v 的边
        found = False
        te = _find_topo_edge_by_uv(topo_graph, u, v)
        if te is not None:
            total_length += float(te.len2d)
            found = True

        if not found:
            for edge in edge_task_map.values():
                if (edge.u == u and edge.v == v) or (edge.u == v and edge.v == u):
                    total_length += float(edge.len2d)
                    found = True
                    break

        if not found:
            # 没有直接边，使用节点间直线距离
            u_node = topo_graph.get_node(u)
            v_node = topo_graph.get_node(v)
            if u_node and v_node:
                pu = np.array(u_node.pos2d, dtype=np.float64)
                pv = np.array(v_node.pos2d, dtype=np.float64)
                total_length += float(np.linalg.norm(pv - pu))

    return total_length


def compute_direction_change_penalty(
    from_point: Tuple[float, float],
    to_point: Tuple[float, float],
    from_direction: str,
    to_direction: str,
    from_edge: EdgeTask,
    to_edge: EdgeTask
) -> float:
    """
    计算方向变化惩罚（0-1）

    0: 方向一致（延续性好）
    1: 完全反向（折返）
    """
    if not from_edge or not to_edge:
        return 0.5  # 默认中等惩罚

    # 计算从边到连接段的方向向量
    from_vec = np.array(to_point) - np.array(from_point)
    from_vec_norm = np.linalg.norm(from_vec)

    if from_vec_norm < 0.1:
        return 0.0

    from_vec = from_vec / from_vec_norm

    # 计算目标边的方向向量
    to_edge_geo = get_edge_geometry_with_direction(to_edge, to_direction)
    if len(to_edge_geo) < 2:
        return 0.5

    to_vec = np.array(to_edge_geo[1]) - np.array(to_edge_geo[0])
    to_vec_norm = np.linalg.norm(to_vec)

    if to_vec_norm < 0.1:
        return 0.5

    to_vec = to_vec / to_vec_norm

    # 计算夹角的余弦值
    dot_product = np.dot(from_vec, to_vec)
    cos_angle = np.clip(dot_product, -1.0, 1.0)

    # 转换为 0-1 的惩罚值
    # cos=1 (同向) -> penalty=0
    # cos=-1 (反向) -> penalty=1
    penalty = (1.0 - cos_angle) / 2.0

    return penalty


def find_edge_group(edge_id: str, groups: Dict[str, EdgeGroup]) -> Optional[str]:
    """查找边所属的group"""
    for group_id, group in groups.items():
        if edge_id in group.edge_ids:
            return group_id
    return None


# =====================================================
# 起始点优化
# =====================================================

def evaluate_start_edge_candidate(
    start_edge_id: str,
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: dict
) -> float:
    """
    评估从指定边开始的起始代价

    考虑因素：
    - 是否为端点边（deg=1 的节点）
    - 是否在大 group 中
    - 从该点出发的可达性

    Returns:
        float: 起始代价（越小越好）
    """
    edge = edge_task_map.get(start_edge_id)
    if not edge:
        return float('inf')

    cost = 0.0

    # 1. 端点边优先（deg=1 的节点是天然的起点/终点）
    u_node = topo_graph.get_node(edge.u)
    v_node = topo_graph.get_node(edge.v)
    has_endpoint = (u_node and u_node.deg == 1) or (v_node and v_node.deg == 1)

    if has_endpoint:
        cost -= 100.0  # 端点边降低代价（优先）
    else:
        cost += 20.0   # 中间边增加代价

    # 2. 大 group 优先
    group_id = find_edge_group(start_edge_id, groups)
    if group_id:
        group = groups[group_id]
        # group 越大，起始代价越低
        cost -= group.total_inspect_length * 0.1

    # 3. 中心性评估（越靠近中心的边，起始代价越低）
    # 简化：使用相邻边数量
    neighbors = adjacency.get(start_edge_id, [])
    cost -= len(neighbors) * 5.0

    return cost


def generate_start_edge_candidates(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: dict,
    max_candidates: int = 10
) -> List[Tuple[str, float]]:
    """
    生成候选起始边列表

    Returns:
        List[Tuple[edge_id, score]]: 按起始代价排序
    """
    candidates = []

    for edge in edge_tasks:
        score = evaluate_start_edge_candidate(
            edge.edge_id, edge_tasks, topo_graph,
            groups, edge_task_map, adjacency
        )
        candidates.append((edge.edge_id, score))

    # 按代价排序（低到高）
    candidates.sort(key=lambda x: x[1])

    # 返回前 N 个
    return candidates[:max_candidates]


# =====================================================
# 全局顺序优化（模拟退火）
# =====================================================

def evaluate_order_cost(
    edge_order: List[str],
    edge_directions: Dict[str, str],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    weights: Dict[str, float] = None
) -> float:
    """
    评估边访问顺序的总代价

    包括：
    - 所有 connect 段的代价（含已完成线路穿越惩罚）
    - group 切换惩罚、方向变化
    - 同 line_id 连续奖励（乘因子）、同线弧长反向跳跃惩罚
    """
    if weights is None:
        weights = _default_order_weights()
    else:
        merged = _default_order_weights()
        merged.update(weights)
        weights = merged

    line_totals = _line_point_totals(list(edge_task_map.values()))
    edge_mean = _edge_mean_arc_map(edge_task_map)
    line_visited: Dict[str, int] = defaultdict(int)
    completed_lines: Set[str] = set()

    total_cost = 0.0
    prev_edge_id: Optional[str] = None
    prev_direction: Optional[str] = None
    prev_point_end: Optional[Tuple[float, float]] = None
    first_edge_placed = False

    for i, edge_id in enumerate(edge_order):
        direction = edge_directions.get(edge_id, 'forward')
        edge = edge_task_map.get(edge_id)

        if not edge:
            continue

        geo = get_edge_inspection_geometry_with_direction(edge, direction)
        if len(geo) < 2:
            continue

        if not first_edge_placed:
            prev_edge_id = edge_id
            prev_direction = direction
            prev_point_end = geo[-1]
            _apply_edge_visit_line_counts(line_visited, edge)
            for lid in _newly_completed_lines(line_visited, line_totals, completed_lines):
                completed_lines.add(lid)
            first_edge_placed = True
            continue

        if prev_edge_id is None or prev_direction is None or prev_point_end is None:
            continue

        to_line = getattr(edge, "line_id", "") or None
        completed_chain_ids = set(edge_order[:i])
        completed_topo = _union_topo_edges_for_completed_chains(
            edge_task_map, completed_chain_ids
        )
        cost = compute_connection_cost_enhanced(
            prev_edge_id,
            edge_id,
            prev_direction,
            direction,
            prev_point_end,
            geo[0],
            topo_graph,
            edge_task_map,
            groups,
            weights,
            completed_lines=set(completed_lines) if completed_lines else None,
            target_line_id=to_line,
            completed_topo_edge_ids=completed_topo if completed_topo else None,
        )
        transition = float(cost.total_cost)

        prev_edge = edge_task_map.get(prev_edge_id)
        if (
            prev_edge
            and to_line
            and getattr(prev_edge, "line_id", "") == to_line
        ):
            factor = float(weights.get("same_line_transition_factor", 0.91))
            transition *= factor
            streak = float(weights.get("same_line_streak_reward", 40.0))
            transition = max(0.0, transition - streak)

        pm = edge_mean.get(prev_edge_id) if prev_edge_id else None
        cm = edge_mean.get(edge_id)
        if (
            prev_edge
            and to_line
            and getattr(prev_edge, "line_id", "") == to_line
            and pm is not None
            and cm is not None
            and cm < pm - 15.0
        ):
            transition += float(weights.get("line_reverse_jump", 0.35)) * float(pm - cm)

        total_cost += transition

        _apply_edge_visit_line_counts(line_visited, edge)
        for lid in _newly_completed_lines(line_visited, line_totals, completed_lines):
            completed_lines.add(lid)

        prev_edge_id = edge_id
        prev_direction = direction
        prev_point_end = geo[-1]

    lm = _line_continuity_metrics(edge_order, edge_task_map, weights)
    total_cost += float(lm["total_penalty"])

    return total_cost


def optimize_edge_order_simulated_annealing(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict,
    start_edge_id: str = None,
    initial_temp: float = 1000.0,
    cooling_rate: float = 0.95,
    iterations_per_temp: int = 100,
    min_temp: float = 1.0
) -> Tuple[List[str], Dict[str, str], float]:
    """
    使用模拟退火优化边的访问顺序

    Args:
        edge_tasks: 边任务列表
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        groups: 分组信息
        adjacency: 邻接表
        start_edge_id: 指定起始边（可选）
        initial_temp: 初始温度
        cooling_rate: 降温速率
        iterations_per_temp: 每个温度的迭代次数
        min_temp: 最小温度

    Returns:
        (best_order, best_directions, best_cost)
    """
    print("\n" + "="*60)
    print("[Global Optimizer] 模拟退火优化边访问顺序...")
    print("="*60)

    edge_ids = [e.edge_id for e in edge_tasks]

    # 选择起始边
    if start_edge_id is None:
        candidates = generate_start_edge_candidates(
            edge_tasks, topo_graph, groups, edge_task_map, adjacency
        )
        start_edge_id = candidates[0][0]
        print(f"  [起始边] 自动选择: {start_edge_id} (score={candidates[0][1]:.1f})")

    # 初始化：随机顺序，固定起始边
    current_order = [start_edge_id]
    remaining = [eid for eid in edge_ids if eid != start_edge_id]
    random.shuffle(remaining)
    current_order.extend(remaining)

    # 初始化方向（优先选择从端点开始的方向）
    current_directions = {}
    for eid in edge_ids:
        edge = edge_task_map.get(eid)
        if edge:
            u_node = topo_graph.get_node(edge.u)
            v_node = topo_graph.get_node(edge.v)

            # 优先从 deg=1 的节点开始
            if u_node and u_node.deg == 1:
                current_directions[eid] = 'reverse'  # 从 v 到 u
            elif v_node and v_node.deg == 1:
                current_directions[eid] = 'forward'   # 从 u 到 v
            else:
                current_directions[eid] = 'forward'

    # 计算初始代价
    current_cost = evaluate_order_cost(
        current_order, current_directions,
        topo_graph, edge_task_map, groups
    )

    best_order = current_order.copy()
    best_directions = current_directions.copy()
    best_cost = current_cost

    temp = initial_temp
    iteration = 0

    print(f"  [初始] 代价: {current_cost:.1f}")
    print(f"  [参数] T0={initial_temp}, 降温率={cooling_rate}, 每T迭代={iterations_per_temp}")

    while temp > min_temp:
        for _ in range(iterations_per_temp):
            iteration += 1

            # 生成邻域解
            new_order = current_order.copy()
            new_directions = current_directions.copy()

            # 随机选择一种操作
            operation = random.choice(['swap', 'reverse', 'direction'])

            if operation == 'swap' and len(new_order) > 2:
                # 交换两个非起始边的位置
                idx1, idx2 = random.sample(range(1, len(new_order)), 2)
                new_order[idx1], new_order[idx2] = new_order[idx2], new_order[idx1]

            elif operation == 'reverse' and len(new_order) > 3:
                # 反转一段子路径（不包括起始边）
                start = random.randint(1, len(new_order) - 2)
                end = random.randint(start + 1, len(new_order) - 1)
                new_order[start:end+1] = new_order[start:end+1][::-1]

            elif operation == 'direction':
                # 随机改变一条边的方向
                idx = random.randint(0, len(new_order) - 1)
                eid = new_order[idx]
                new_directions[eid] = 'reverse' if new_directions[eid] == 'forward' else 'forward'

            # 计算新解代价
            new_cost = evaluate_order_cost(
                new_order, new_directions,
                topo_graph, edge_task_map, groups
            )

            # 接受准则
            delta = new_cost - current_cost
            if delta < 0 or random.random() < np.exp(-delta / temp):
                current_order = new_order
                current_directions = new_directions
                current_cost = new_cost

                if current_cost < best_cost:
                    best_order = current_order.copy()
                    best_directions = current_directions.copy()
                    best_cost = current_cost

        # 降温
        temp *= cooling_rate

        if iteration % 500 == 0:
            print(f"  [进度] iter={iteration}, T={temp:.1f}, 当前最优={best_cost:.1f}")

    print(f"  [完成] 最终代价: {best_cost:.1f} (迭代 {iteration} 次)")
    print(f"  [最优顺序] {' -> '.join(best_order[:5])}... (共{len(best_order)}条边)")

    lm_sa = _line_continuity_metrics(best_order, edge_task_map, None)
    line_keys_sa = sorted(
        set(lm_sa["blocks_per_line"].keys()) | set(lm_sa["per_line_revisit_pen"].keys())
    )
    for lid in line_keys_sa:
        b = int(lm_sa["blocks_per_line"].get(lid, 0))
        rp = float(lm_sa["per_line_revisit_pen"].get(lid, 0.0))
        print(f"[line-continuity] line_id={lid} blocks={b} revisit_penalty={rp:.1f}")

    return best_order, best_directions, best_cost


def line_inspect_geometry(lt: LineTask, direction: str) -> List[Tuple[float, float]]:
    """沿 LineTask 折线在 [s_min,s_max] 上的巡检几何；forward 为弧长递增，reverse 为递减。"""
    raw = _slice_polyline_by_distance(list(lt.polyline), lt.s_min, lt.s_max)
    if len(raw) < 2:
        return []
    if direction == "reverse":
        return list(reversed(raw))
    return raw


def evaluate_line_order_cost(
    line_order: List[str],
    line_directions: Dict[str, str],
    line_task_map: Dict[str, LineTask],
    edge_task_map: Dict[str, EdgeTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    weights: Optional[Dict[str, float]] = None,
) -> float:
    """线路级顺序代价：仅相邻物理线路之间计 connect，无同线 connect。"""
    if weights is None:
        weights = _default_order_weights()
    else:
        merged = _default_order_weights()
        merged.update(weights)
        weights = merged

    line_totals_local = {lid: lt.num_points for lid, lt in line_task_map.items()}
    line_visited: Dict[str, int] = defaultdict(int)
    completed_lines: Set[str] = set()

    total_cost = 0.0
    prev_line_id: Optional[str] = None
    prev_point_end: Optional[Tuple[float, float]] = None
    first_placed = False

    for line_id in line_order:
        lt = line_task_map.get(line_id)
        if not lt or lt.num_points <= 0:
            continue
        direction = line_directions.get(line_id, "forward")
        geo = line_inspect_geometry(lt, direction)
        if len(geo) < 2:
            continue

        if not first_placed:
            line_visited[lt.line_id] += lt.num_points
            if line_visited[lt.line_id] >= line_totals_local.get(lt.line_id, 0):
                completed_lines.add(lt.line_id)
            prev_line_id = line_id
            prev_point_end = geo[-1]
            first_placed = True
            continue

        prev_lt = line_task_map.get(prev_line_id) if prev_line_id else None
        if prev_lt is None or prev_point_end is None:
            continue

        from_e = prev_lt.rep_end_edge_id or prev_lt.rep_start_edge_id
        to_e = lt.rep_start_edge_id or lt.rep_end_edge_id
        if not from_e or not to_e:
            continue

        to_line = lt.line_id
        completed_topo_lt = _union_topo_edges_for_completed_lines(
            topo_graph, set(completed_lines) if completed_lines else set()
        )
        cost = compute_connection_cost_enhanced(
            from_e,
            to_e,
            "forward",
            "forward",
            prev_point_end,
            geo[0],
            topo_graph,
            edge_task_map,
            groups,
            weights,
            completed_lines=set(completed_lines) if completed_lines else None,
            target_line_id=to_line,
            completed_topo_edge_ids=completed_topo_lt if completed_topo_lt else None,
        )
        total_cost += float(cost.total_cost)

        line_visited[lt.line_id] += lt.num_points
        if line_visited[lt.line_id] >= line_totals_local.get(lt.line_id, 0):
            completed_lines.add(lt.line_id)

        prev_line_id = line_id
        prev_point_end = geo[-1]

    return total_cost


def generate_start_line_candidates(
    line_tasks: List[LineTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: dict,
    max_candidates: int = 10,
) -> List[Tuple[str, float]]:
    edge_list = list(edge_task_map.values())
    candidates: List[Tuple[str, float]] = []
    for lt in line_tasks:
        eid = lt.rep_start_edge_id or (lt.edge_ids[0] if lt.edge_ids else "")
        if not eid:
            continue
        score = evaluate_start_edge_candidate(
            eid, edge_list, topo_graph, groups, edge_task_map, adjacency
        )
        candidates.append((lt.line_id, score))
    candidates.sort(key=lambda x: x[1])
    return candidates[:max_candidates]


def optimize_line_order_simulated_annealing(
    line_tasks: List[LineTask],
    topo_graph: TopoGraph,
    line_task_map: Dict[str, LineTask],
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict,
    start_line_id: Optional[str] = None,
    initial_temp: float = 1000.0,
    cooling_rate: float = 0.95,
    iterations_per_temp: int = 50,
    min_temp: float = 1.0,
) -> Tuple[List[str], Dict[str, str], float]:
    print("\n" + "=" * 60)
    print("[Global Optimizer] 模拟退火优化物理线路访问顺序...")
    print("=" * 60)

    line_ids = [lt.line_id for lt in line_tasks]

    if start_line_id is None or start_line_id not in line_task_map:
        candidates = generate_start_line_candidates(
            line_tasks, topo_graph, groups, edge_task_map, adjacency
        )
        start_line_id = candidates[0][0]
        print(f"  [起始线路] 自动选择: {start_line_id} (score={candidates[0][1]:.1f})")

    current_order = [start_line_id]
    remaining = [lid for lid in line_ids if lid != start_line_id]
    random.shuffle(remaining)
    current_order.extend(remaining)

    current_directions: Dict[str, str] = {lid: "forward" for lid in line_ids}

    current_cost = evaluate_line_order_cost(
        current_order,
        current_directions,
        line_task_map,
        edge_task_map,
        topo_graph,
        groups,
    )

    best_order = current_order.copy()
    best_directions = current_directions.copy()
    best_cost = current_cost

    temp = initial_temp
    iteration = 0

    print(f"  [初始] 代价: {current_cost:.1f}")
    print(f"  [参数] T0={initial_temp}, 降温率={cooling_rate}, 每T迭代={iterations_per_temp}")

    while temp > min_temp:
        for _ in range(iterations_per_temp):
            iteration += 1

            new_order = current_order.copy()
            new_directions = current_directions.copy()
            operation = random.choice(["swap", "reverse", "direction"])

            if operation == "swap" and len(new_order) > 2:
                idx1, idx2 = random.sample(range(1, len(new_order)), 2)
                new_order[idx1], new_order[idx2] = new_order[idx2], new_order[idx1]
            elif operation == "reverse" and len(new_order) > 3:
                start = random.randint(1, len(new_order) - 2)
                end = random.randint(start + 1, len(new_order) - 1)
                new_order[start : end + 1] = new_order[start : end + 1][::-1]
            elif operation == "direction":
                idx = random.randint(0, len(new_order) - 1)
                lid = new_order[idx]
                new_directions[lid] = (
                    "reverse" if new_directions[lid] == "forward" else "forward"
                )

            new_cost = evaluate_line_order_cost(
                new_order,
                new_directions,
                line_task_map,
                edge_task_map,
                topo_graph,
                groups,
            )

            delta = new_cost - current_cost
            if delta < 0 or random.random() < np.exp(-delta / temp):
                current_order = new_order
                current_directions = new_directions
                current_cost = new_cost
                if current_cost < best_cost:
                    best_order = current_order.copy()
                    best_directions = current_directions.copy()
                    best_cost = current_cost

        temp *= cooling_rate
        if iteration % 500 == 0:
            print(f"  [进度] iter={iteration}, T={temp:.1f}, 当前最优={best_cost:.1f}")

    print(f"  [完成] 最终代价: {best_cost:.1f} (迭代 {iteration} 次)")
    order_disp = "/".join(
        f"{lid}{'+' if best_directions.get(lid, 'forward') == 'forward' else '-'}"
        for lid in best_order
    )
    print(f"[line-order] order=[{order_disp}]")

    return best_order, best_directions, best_cost


def build_optimized_mission_from_line_tasks(
    line_order: List[str],
    line_directions: Dict[str, str],
    line_tasks: List[LineTask],
    line_task_map: Dict[str, LineTask],
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict,
    connect_planner: str = "bfs",
    cost_config: Optional[Dict[str, Any]] = None,
) -> GroupedContinuousMission:
    print("\n" + "=" * 60)
    print("[Global Optimizer] 构建线路级连续任务...")
    print("=" * 60)

    mission = GroupedContinuousMission()
    line_totals_lt = {lt.line_id: lt.num_points for lt in line_tasks}
    line_visited: Dict[str, int] = defaultdict(int)
    completed_lines: Set[str] = set()
    route_stats: Dict[str, Any] = {
        "completed_line_cross_count": 0,
        "cross_details": [],
        "same_line_connect_count": 0,
        "same_line_connect_fallback_count": 0,
    }

    visited_groups: List[str] = []
    current_group: Optional[str] = None
    current_end_point: Optional[Tuple[float, float]] = None
    current_line_id: Optional[str] = None
    first_inspect_placed = False

    line_visit_export: List[Tuple[str, str, List[Any]]] = []

    for line_id in line_order:
        lt = line_task_map.get(line_id)
        if not lt or lt.num_points <= 0:
            continue

        direction = line_directions.get(line_id, "forward")
        geo = line_inspect_geometry(lt, direction)
        if len(geo) < 2:
            print(
                f"[DEBUG] dead-end branch skipped line_id={line_id} reason=empty_line_inspect_geometry"
            )
            continue

        group_id = find_edge_group(lt.line_id, groups)
        if group_id and group_id != current_group:
            if current_group is not None:
                visited_groups.append(group_id)
            current_group = group_id

        if not first_inspect_placed:
            mission.segments.append(
                MissionSegment(
                    type="inspect",
                    from_edge_id=None,
                    to_edge_id=lt.line_id,
                    geometry=geo,
                    length=float(
                        np.sum(np.linalg.norm(np.diff(np.array(geo), axis=0), axis=1))
                    ),
                    edge_id=lt.line_id,
                    direction=direction,
                )
            )
            mission.visit_order.append(
                f"{lt.line_id}{'+' if direction == 'forward' else '-'}"
            )
            current_end_point = geo[-1]
            current_line_id = lt.line_id
            first_inspect_placed = True

            if group_id and group_id in groups:
                if group_id not in mission.groups:
                    mission.add_group(groups[group_id])

            pts_exp = (
                list(lt.inspection_points)
                if direction == "forward"
                else list(reversed(lt.inspection_points))
            )
            line_visit_export.append((lt.line_id, direction, pts_exp))

            line_visited[lt.line_id] += lt.num_points
            if line_visited[lt.line_id] >= line_totals_lt.get(lt.line_id, 0):
                completed_lines.add(lt.line_id)
            continue

        completed_for_connect = set(completed_lines) if completed_lines else None
        prev_lt = line_task_map.get(current_line_id) if current_line_id else None
        if prev_lt is None or current_end_point is None:
            continue

        from_e = prev_lt.rep_end_edge_id or prev_lt.rep_start_edge_id
        to_e = lt.rep_start_edge_id or lt.rep_end_edge_id
        if not from_e or not to_e:
            continue

        to_line = lt.line_id
        completed_topo_line = _union_topo_edges_for_completed_lines(
            topo_graph, set(completed_for_connect or [])
        )
        use_dijkstra_lt = (
            connect_planner
            and str(connect_planner).lower() == "dijkstra"
            and not completed_topo_line
        )
        if use_dijkstra_lt:
            from planner.topo_dijkstra import generate_connection_segment_with_planner

            connect_geo, connect_len = generate_connection_segment_with_planner(
                current_end_point,
                geo[0],
                topo_graph,
                edge_task_map,
                connect_planner="dijkstra",
                cost_config=cost_config,
                from_edge_id=from_e,
                to_edge_id=to_e,
            )
        else:
            connect_geo, connect_len = generate_connection_segment_along_topo(
                current_end_point,
                geo[0],
                topo_graph,
                edge_task_map,
                from_edge_id=from_e,
                to_edge_id=to_e,
                completed_lines=completed_for_connect,
                target_line_id=to_line,
                completed_line_edge_penalty=450.0,
                completed_topo_edge_ids=(
                    completed_topo_line if completed_topo_line else None
                ),
                completed_edge_penalty=450.0,
                route_connect_stats=route_stats,
            )

        print(
            f"[line-connect] from_line={current_line_id} to_line={lt.line_id} length={connect_len:.1f}"
        )

        connect_geo_dense = interpolate_geometry(
            connect_geo, step_size=10.0, min_points=20
        )
        is_inter_group = find_edge_group(current_line_id, groups) != find_edge_group(
            lt.line_id, groups
        )

        mission.segments.append(
            MissionSegment(
                type="connect",
                from_edge_id=from_e,
                to_edge_id=to_e,
                geometry=connect_geo_dense,
                length=connect_len,
            )
        )
        if is_inter_group:
            mission.inter_group_connect_length += connect_len
        else:
            mission.intra_group_connect_length += connect_len

        mission.segments.append(
            MissionSegment(
                type="inspect",
                from_edge_id=from_e,
                to_edge_id=lt.line_id,
                geometry=geo,
                length=float(
                    np.sum(np.linalg.norm(np.diff(np.array(geo), axis=0), axis=1))
                ),
                edge_id=lt.line_id,
                direction=direction,
            )
        )
        mission.visit_order.append(
            f"{lt.line_id}{'+' if direction == 'forward' else '-'}"
        )

        if group_id and group_id in groups:
            if group_id not in mission.groups:
                mission.add_group(groups[group_id])

        pts_exp = (
            list(lt.inspection_points)
            if direction == "forward"
            else list(reversed(lt.inspection_points))
        )
        line_visit_export.append((lt.line_id, direction, pts_exp))

        line_visited[lt.line_id] += lt.num_points
        if line_visited[lt.line_id] >= line_totals_lt.get(lt.line_id, 0):
            completed_lines.add(lt.line_id)

        current_end_point = geo[-1]
        current_line_id = lt.line_id

    mission.group_visit_order = visited_groups
    mission.line_visit_export = line_visit_export

    lids_raw: List[str] = []
    for vo in mission.visit_order:
        if len(vo) > 1 and vo[-1] in "+-":
            lids_raw.append(vo[:-1])
        else:
            lids_raw.append(vo)
    dup = len(lids_raw) - len(set(lids_raw))
    total_pts = sum(lt.num_points for lt in line_tasks)
    print(
        f"[line-verify] duplicate_line_count={dup} same_line_connect_count="
        f"{int(route_stats.get('same_line_connect_count', 0))} "
        f"total_lines={len(mission.visit_order)} total_points={total_pts}"
    )

    inspect_len = sum(s.length for s in mission.segments if s.type == "inspect")
    connect_len = sum(s.length for s in mission.segments if s.type == "connect")
    mission.total_length = inspect_len + connect_len
    mission.inspect_length = inspect_len
    mission.connect_length = connect_len

    inter_group_count = 0
    for seg in mission.segments:
        if seg.type == "connect":
            fg = find_edge_group(seg.from_edge_id, groups)
            tg = find_edge_group(seg.to_edge_id, groups)
            if fg and tg and fg != tg:
                inter_group_count += 1

    connect_segments = sum(1 for s in mission.segments if s.type == "connect")
    inspect_segments = sum(1 for s in mission.segments if s.type == "inspect")
    route_stats["lines_completed"] = len(completed_lines)
    route_stats["connect_count"] = connect_segments
    setattr(mission, "route_plan_stats", route_stats)

    mission.edge_to_group = {}
    for gid, grp in groups.items():
        for eid in grp.edge_ids:
            mission.edge_to_group[eid] = gid

    if mission.visit_order:
        first_vo = mission.visit_order[0]
        mission.start_edge_id = (
            first_vo[:-1] if len(first_vo) > 1 and first_vo[-1] in "+-" else first_vo
        )

    cc = int(route_stats.get("completed_line_cross_count", 0))
    slc = int(route_stats.get("same_line_connect_count", 0))
    slf = int(route_stats.get("same_line_connect_fallback_count", 0))
    print(
        f"[route-summary] edge_tasks={len(edge_tasks)} lines={len(line_tasks)} "
        f"inspect_count={inspect_segments} connect_count={connect_segments} "
        f"completed_line_cross_count={cc} lines_completed={len(completed_lines)} "
        f"line_revisit_count=0 total_line_penalty=0.0 "
        f"same_line_connect_count={slc} same_line_connect_fallback_count={slf}"
    )

    print(
        f"\n  [统计] 总长度={mission.total_length:.1f}px, "
        f"inspect={mission.inspect_length:.1f}px, "
        f"connect={mission.connect_length:.1f}px"
    )
    print(
        f"  [统计] Group切换={inter_group_count}次, "
        f"内组连接={mission.intra_group_connect_length:.1f}px, "
        f"跨组连接={mission.inter_group_connect_length:.1f}px"
    )

    return mission


# =====================================================
# 构建优化后的任务
# =====================================================

def _strip_visit_order_token(tok: Any) -> str:
    s = str(tok or "").strip()
    if len(s) >= 2 and s[-1] in "+-":
        return s[:-1]
    return s


def build_optimized_mission(
    edge_order: List[str],
    edge_directions: Dict[str, str],
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict,
    connect_planner: str = "bfs",
    cost_config: Optional[Dict[str, Any]] = None,
    line_totals: Optional[Dict[str, int]] = None,
) -> GroupedContinuousMission:
    """
    根据优化后的顺序构建任务
    """
    print("\n" + "="*60)
    print("[Global Optimizer] 构建优化后的任务...")
    print("="*60)

    mission = GroupedContinuousMission()
    mission.task_by_id = {str(task.edge_id): task for task in edge_tasks}

    if line_totals is None:
        line_totals = _line_point_totals(edge_tasks)
    line_visited: Dict[str, int] = defaultdict(int)
    completed_lines: Set[str] = set()
    route_stats: Dict[str, Any] = {
        "completed_line_cross_count": 0,
        "cross_details": [],
        "same_line_connect_count": 0,
        "same_line_connect_fallback_count": 0,
    }

    # 记录 group 访问顺序
    visited_groups = []
    current_group = None

    current_end_point = None
    current_edge_id = None
    current_direction = None
    first_inspect_placed = False

    for i, edge_id in enumerate(edge_order):
        direction = edge_directions.get(edge_id, 'forward')
        edge = edge_task_map.get(edge_id)

        if not edge:
            continue

        group_id = find_edge_group(edge_id, groups)

        # 检查是否切换了 group
        if group_id and group_id != current_group:
            if current_group is not None:
                visited_groups.append(group_id)
            current_group = group_id

        # 获取当前边的几何
        geo = get_edge_inspection_geometry_with_direction(edge, direction, debug=True)
        if len(geo) < 2:
            print(f"[DEBUG] dead-end branch skipped/truncated edge={edge_id} reason=empty_trimmed_inspect_geometry")
            continue

        if not first_inspect_placed:
            # 第一个边：直接添加 inspect 段
            mission.segments.append(MissionSegment(
                type='inspect',
                from_edge_id=None,
                to_edge_id=edge_id,
                geometry=geo,
                length=float(np.sum(np.linalg.norm(np.diff(np.array(geo), axis=0), axis=1))),
                edge_id=edge_id,
                direction=direction,
            ))
            mission.visit_order.append(edge_id)
            print(
                f"[DEBUG] visit target edge={edge_id} type=inspection_point "
                f"start={geo[0]} end={geo[-1]}"
            )
            print(
                f"[DEBUG] segment generated type=inspect edge={edge_id} "
                f"length={mission.segments[-1].length:.2f} reason=required_inspection_points_only"
            )

            current_end_point = geo[-1]
            current_edge_id = edge_id
            current_direction = direction

            # 添加到 group（如果存在）
            if group_id and group_id in groups:
                if group_id not in mission.groups:
                    mission.add_group(groups[group_id])

            print(f"  [{len(mission.visit_order)}] {edge_id} ({direction}) - 起点")

            _apply_edge_visit_line_counts(line_visited, edge)
            for lid in _newly_completed_lines(line_visited, line_totals, completed_lines):
                completed_lines.add(lid)
                print(
                    f"[route-line] completed line_id={lid} at order={len(mission.visit_order)}"
                )

            first_inspect_placed = True
            continue

        to_line = getattr(edge, "line_id", "") or None
        completed_for_connect = set(completed_lines) if completed_lines else None

        completed_topo_for_connect = _union_topo_edges_for_completed_chains(
            edge_task_map, set(mission.visit_order)
        )
        use_dijkstra = (
            connect_planner
            and str(connect_planner).lower() == "dijkstra"
            and not completed_topo_for_connect
        )

        prev_edge_task = edge_task_map.get(current_edge_id) if current_edge_id else None
        same_line_adjacent = bool(
            to_line
            and prev_edge_task is not None
            and _tasks_share_line(prev_edge_task, edge)
        )

        connect_geo: List[Tuple[float, float]]
        connect_len: float
        connect_provenance: Dict[str, Any] = {}

        if same_line_adjacent:
            sl_raw = _try_same_line_connect_geometry(
                edge_order,
                edge_task_map,
                to_line,
                (float(current_end_point[0]), float(current_end_point[1])),
                (float(geo[0][0]), float(geo[0][1])),
            )
            if sl_raw and len(sl_raw) >= 2:
                connect_geo = sl_raw
                connect_len = float(_polyline_length(sl_raw))
                connect_provenance.update({
                    "connect_mode": "topology",
                    "planner": "same_line_polyline",
                    "reason": "same_line",
                    "fallback_reason": None,
                    "topo_edge_ids": sorted(set(
                        list(getattr(prev_edge_task, "topo_edge_ids", None) or [])
                        + list(getattr(edge, "topo_edge_ids", None) or [])
                    )),
                })
                route_stats["same_line_connect_count"] = (
                    int(route_stats.get("same_line_connect_count", 0)) + 1
                )
                print(
                    f"[same-line-connect] from={current_edge_id} to={edge_id} "
                    f"line_id={to_line} length={connect_len:.1f} fallback=False"
                )
            else:
                route_stats["same_line_connect_fallback_count"] = (
                    int(route_stats.get("same_line_connect_fallback_count", 0)) + 1
                )
                if use_dijkstra:
                    from planner.topo_dijkstra import generate_connection_segment_with_planner

                    connect_geo, connect_len = generate_connection_segment_with_planner(
                        current_end_point, geo[0], topo_graph, edge_task_map,
                        connect_planner="dijkstra",
                        cost_config=cost_config,
                        from_edge_id=current_edge_id,
                        to_edge_id=edge_id,
                        provenance_out=connect_provenance,
                    )
                else:
                    connect_geo, connect_len = generate_connection_segment_along_topo(
                        current_end_point,
                        geo[0],
                        topo_graph,
                        edge_task_map,
                        from_edge_id=current_edge_id,
                        to_edge_id=edge_id,
                        completed_lines=completed_for_connect,
                        target_line_id=to_line,
                        completed_line_edge_penalty=450.0,
                        completed_topo_edge_ids=(
                            completed_topo_for_connect if completed_topo_for_connect else None
                        ),
                        completed_edge_penalty=450.0,
                        route_connect_stats=route_stats,
                        provenance_out=connect_provenance,
                    )
                print(
                    f"[same-line-connect] from={current_edge_id} to={edge_id} "
                    f"line_id={to_line} length={connect_len:.1f} fallback=True"
                )
        elif use_dijkstra:
            from planner.topo_dijkstra import generate_connection_segment_with_planner

            connect_geo, connect_len = generate_connection_segment_with_planner(
                current_end_point, geo[0], topo_graph, edge_task_map,
                connect_planner="dijkstra",
                cost_config=cost_config,
                from_edge_id=current_edge_id,
                to_edge_id=edge_id,
                provenance_out=connect_provenance,
            )
        else:
            connect_geo, connect_len = generate_connection_segment_along_topo(
                current_end_point,
                geo[0],
                topo_graph,
                edge_task_map,
                from_edge_id=current_edge_id,
                to_edge_id=edge_id,
                completed_lines=completed_for_connect,
                target_line_id=to_line,
                completed_line_edge_penalty=450.0,
                completed_topo_edge_ids=(
                    completed_topo_for_connect if completed_topo_for_connect else None
                ),
                completed_edge_penalty=450.0,
                route_connect_stats=route_stats,
                provenance_out=connect_provenance,
            )

        # 插值密集路径点
        connect_geo_dense = interpolate_geometry(connect_geo, step_size=10.0, min_points=20)

        # 检查是否跨组
        is_inter_group = (find_edge_group(current_edge_id, groups) !=
                         find_edge_group(edge_id, groups))

        # 添加 connect 段（不使用 is_inter_group 参数）
        connect_segment = MissionSegment(
            type='connect',
            from_edge_id=current_edge_id,
            to_edge_id=edge_id,
            geometry=connect_geo_dense,
            length=connect_len,
            connect_mode=connect_provenance.get("connect_mode") or "unknown",
            planner=connect_provenance.get("planner") or "unknown",
            reason=connect_provenance.get("reason") or "unknown",
            fallback_reason=connect_provenance.get("fallback_reason"),
            topo_edge_ids=list(connect_provenance.get("topo_edge_ids") or []),
            from_component_ids=sorted(set((getattr(prev_edge_task, "meta", None) or {}).get("component_ids") or [])),
            to_component_ids=sorted(set((getattr(edge, "meta", None) or {}).get("component_ids") or [])),
        )
        mission.segments.append(connect_segment)
        print(
            f"[DEBUG] segment generated type=connect from={current_edge_id} to={edge_id} "
            f"length={connect_len:.2f} reason=connect_next_required_inspection"
        )

        # 统计跨组/内组连接
        if is_inter_group:
            mission.inter_group_connect_length += connect_len
        else:
            mission.intra_group_connect_length += connect_len

        # 添加 inspect 段
        mission.segments.append(MissionSegment(
            type='inspect',
            from_edge_id=current_edge_id,
            to_edge_id=edge_id,
            geometry=geo,
            length=float(np.sum(np.linalg.norm(np.diff(np.array(geo), axis=0), axis=1))),
            edge_id=edge_id,
            direction=direction,
        ))
        mission.visit_order.append(edge_id)
        print(
            f"[DEBUG] visit target edge={edge_id} type=inspection_point "
            f"start={geo[0]} end={geo[-1]}"
        )
        print(
            f"[DEBUG] segment generated type=inspect edge={edge_id} "
            f"length={mission.segments[-1].length:.2f} reason=trimmed_to_last_required_point"
        )

        # 添加到 group
        if group_id and group_id in groups:
            if group_id not in mission.groups:
                mission.add_group(groups[group_id])

        _apply_edge_visit_line_counts(line_visited, edge)
        for lid in _newly_completed_lines(line_visited, line_totals, completed_lines):
            completed_lines.add(lid)
            print(
                f"[route-line] completed line_id={lid} at order={len(mission.visit_order)}"
            )

        # 更新状态
        current_end_point = geo[-1]
        current_edge_id = edge_id
        current_direction = direction

        from_group = find_edge_group(current_edge_id, groups)
        to_group = find_edge_group(edge_id, groups)
        group_info = f" [{from_group}->{to_group}]" if from_group != to_group else ""
        print(f"  [{len(mission.visit_order)}] {edge_id} ({direction}) - connect={connect_len:.1f}px{group_info}")

    # 设置 group 访问顺序
    mission.group_visit_order = visited_groups

    # 统计
    inspect_len = sum(s.length for s in mission.segments if s.type == 'inspect')
    connect_len = sum(s.length for s in mission.segments if s.type == 'connect')

    mission.total_length = inspect_len + connect_len
    mission.inspect_length = inspect_len
    mission.connect_length = connect_len

    # 计算跨组切换次数
    inter_group_count = 0
    for seg in mission.segments:
        if seg.type == 'connect':
            from_group = find_edge_group(seg.from_edge_id, groups)
            to_group = find_edge_group(seg.to_edge_id, groups)
            if from_group and to_group and from_group != to_group:
                inter_group_count += 1

    connect_segments = sum(1 for s in mission.segments if s.type == "connect")
    inspect_segments = sum(1 for s in mission.segments if s.type == "inspect")
    route_stats["lines_completed"] = len(completed_lines)
    route_stats["connect_count"] = connect_segments

    ow: Optional[Dict[str, float]] = None
    if cost_config and isinstance(cost_config.get("order_weights"), dict):
        raw = cost_config["order_weights"]
        ow = {
            str(k): float(v)
            for k, v in raw.items()
            if isinstance(v, (int, float))
        }
    lm_route = _line_continuity_metrics(edge_order, edge_task_map, ow)
    route_stats["line_revisit_count"] = int(lm_route["revisit_count"])
    route_stats["total_line_penalty"] = float(lm_route["total_penalty"])

    setattr(mission, "route_plan_stats", route_stats)

    cc = int(route_stats.get("completed_line_cross_count", 0))
    lrc = int(route_stats.get("line_revisit_count", 0))
    tlp = float(route_stats.get("total_line_penalty", 0.0))
    slc = int(route_stats.get("same_line_connect_count", 0))
    slf = int(route_stats.get("same_line_connect_fallback_count", 0))
    print(
        f"[route-summary] edge_tasks={len(edge_tasks)} lines={len(line_totals)} "
        f"inspect_count={inspect_segments} connect_count={connect_segments} "
        f"completed_line_cross_count={cc} lines_completed={len(completed_lines)} "
        f"line_revisit_count={lrc} "
        f"total_line_penalty={tlp:.1f} same_line_connect_count={slc} "
        f"same_line_connect_fallback_count={slf}"
    )
    if cc > 5:
        print(f"[route-summary] listing cross_details (max 20)")
        for row in route_stats.get("cross_details", [])[:20]:
            print(f"[route-summary] cross_detail {row}")

    print(f"\n  [统计] 总长度={mission.total_length:.1f}px, "
          f"inspect={mission.inspect_length:.1f}px, "
          f"connect={mission.connect_length:.1f}px")
    print(f"  [统计] Group切换={inter_group_count}次, "
          f"内组连接={mission.intra_group_connect_length:.1f}px, "
          f"跨组连接={mission.inter_group_connect_length:.1f}px")

    insp_segs = [s for s in mission.segments if s.type == "inspect"]
    conn_segs = [s for s in mission.segments if s.type == "connect"]
    insp_eids = [s.edge_id for s in insp_segs if getattr(s, "edge_id", None)]
    dup_insp = len(insp_eids) - len(set(insp_eids))
    vo = list(getattr(mission, "visit_order", None) or [])
    stale = sum(1 for x in vo if _strip_visit_order_token(x) not in edge_task_map)
    print(
        f"[mission-debug] inspect_segments_count={len(insp_segs)} "
        f"connect_segments_count={len(conn_segs)} "
        f"duplicate_inspect_edge_count={dup_insp} stale_edge_id_count={stale}"
    )

    return mission


def _merged_chain_node_set(task: EdgeTask, topo_graph: TopoGraph) -> Set[str]:
    """合并链任务在拓扑图上的所有端点节点（用于链与链之间的邻接）。"""
    ids = (getattr(task, "meta", None) or {}).get("chain_topo_edge_ids") or []
    nodes: Set[str] = set()
    for eid in ids:
        e = topo_graph.edges.get(eid)
        if e:
            nodes.add(e.u)
            nodes.add(e.v)
    if not nodes and getattr(task, "u", None) and getattr(task, "v", None):
        nodes.update([str(task.u), str(task.v)])
    return nodes


def build_merged_edge_task_adjacency(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
) -> Dict[str, List[str]]:
    """合并链 EdgeTask 邻接：两链在拓扑上共享至少一个节点则相邻。"""
    tasks = [t for t in edge_tasks]
    ids = [t.edge_id for t in tasks]
    node_sets = {t.edge_id: _merged_chain_node_set(t, topo_graph) for t in tasks}
    adj: Dict[str, List[str]] = {i: [] for i in ids}
    for i, a in enumerate(tasks):
        sa = node_sets[a.edge_id]
        for b in tasks[i + 1 :]:
            if sa & node_sets[b.edge_id]:
                adj[a.edge_id].append(b.edge_id)
                adj[b.edge_id].append(a.edge_id)
    return adj


def _greedy_merged_edge_visit_order(
    required_edge_tasks: List[EdgeTask],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: Dict[str, List[str]],
    start_edge_id: str,
) -> Tuple[List[str], Dict[str, str]]:
    """无 SA 时：从起点链出发，按 inspect 出口到下一链入口的欧氏距离贪心。"""
    ids = [e.edge_id for e in required_edge_tasks]
    line_order = [start_edge_id]
    edge_directions: Dict[str, str] = {eid: "forward" for eid in ids}
    unvisited = set(ids) - {start_edge_id}
    while unvisited:
        cur_id = line_order[-1]
        cur_t = edge_task_map[cur_id]
        cur_geo = get_edge_inspection_geometry_with_direction(
            cur_t, edge_directions[cur_id], debug=False
        )
        if len(cur_geo) < 2:
            break
        cur_end = np.array(cur_geo[-1], dtype=np.float64)
        best_n: Optional[str] = None
        best_d = float("inf")
        best_dir = "forward"
        for nid in unvisited:
            ot = edge_task_map[nid]
            for d in ("forward", "reverse"):
                g = get_edge_inspection_geometry_with_direction(ot, d, debug=False)
                if len(g) < 2:
                    continue
                dist = float(np.linalg.norm(np.array(g[0], dtype=np.float64) - cur_end))
                if dist < best_d:
                    best_d, best_n, best_dir = dist, nid, d
        if best_n is None:
            break
        line_order.append(best_n)
        edge_directions[best_n] = best_dir
        unvisited.remove(best_n)
    for eid in ids:
        if eid not in line_order:
            line_order.append(eid)
    return line_order, edge_directions


# =====================================================
# 主入口：全局拓扑优化
# =====================================================

def plan_global_topology_optimized_mission(
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    start_edge_id: str = None,
    enable_sa: bool = True,
    eps: float = 150.0,
    task_granularity: str = "physical_line",
    mission_edge_tasks: Optional[List[Any]] = None,
) -> GroupedContinuousMission:
    """
    全局拓扑优化的主入口

    Args:
        topo_graph: 拓扑图
        edge_tasks: Chain EdgeTask 列表（始终为链级任务，供拓扑/兼容层使用）
        start_edge_id: 指定起始边（可选，不指定则自动优化）
        enable_sa: 是否启用模拟退火优化
        eps: 分组距离阈值
        task_granularity: "chain" | "physical_line"（Mission 优化粒度）
        mission_edge_tasks: 若传入则直接使用（否则按 task_granularity 从 edge_tasks 派生）

    Returns:
        GroupedContinuousMission: 优化后的任务
    """
    print("\n" + "="*70)
    print("[全局拓扑优化] 开始规划...")
    print("="*70)

    chain_edge_tasks = list(edge_tasks)
    if mission_edge_tasks is not None:
        mission_tasks = list(mission_edge_tasks)
    elif str(task_granularity).lower() == "physical_line":
        from core.physical_line_chain import build_physical_line_chains

        mission_tasks = build_physical_line_chains(chain_edge_tasks, topo_graph)
    else:
        mission_tasks = list(chain_edge_tasks)

    print(
        f"[mission-granularity] task_granularity={task_granularity!r} "
        f"chain_edge_tasks={len(chain_edge_tasks)} mission_tasks={len(mission_tasks)}"
    )

    edge_tasks_input = list(mission_tasks)
    required_edge_tasks = [t for t in edge_tasks_input if (t.num_points or 0) > 0]
    skipped_edges = [t for t in edge_tasks_input if (t.num_points or 0) <= 0]
    for t in skipped_edges:
        print(
            f"[DEBUG] dead-end branch skipped/truncated edge={t.edge_id} "
            "reason=no_required_inspection_points"
        )
    if not required_edge_tasks:
        print("[WARN] 无可巡检边任务（全部 edge 均无 inspection_points）")
        return GroupedContinuousMission()

    edge_task_map = {task.edge_id: task for task in required_edge_tasks}
    if start_edge_id and start_edge_id not in edge_task_map:
        orig = start_edge_id
        for t in required_edge_tasks:
            mids = (getattr(t, "meta", None) or {}).get("member_chain_ids") or []
            if mids and orig in mids:
                start_edge_id = t.edge_id
                print(f"[mission-granularity] start_edge_id {orig!r} -> {start_edge_id!r}")
                break
    if start_edge_id and start_edge_id not in edge_task_map:
        print(f"[WARN] start_edge_id={start_edge_id!r} 不在当前 EdgeTask 中，忽略")
        start_edge_id = None

    adjacency = build_merged_edge_task_adjacency(required_edge_tasks, topo_graph)

    print("\n[Step 1] 合并链 EdgeTask 空间分组...")
    centroids = compute_edge_centroids(required_edge_tasks)
    groups = group_edges_spatially(required_edge_tasks, centroids, eps=eps)

    if enable_sa:
        print("\n[Step 2] 全局顺序优化（合并链边级模拟退火）...")
        edge_order, edge_directions, _cost = optimize_edge_order_simulated_annealing(
            required_edge_tasks,
            topo_graph,
            edge_task_map,
            groups,
            adjacency,
            start_edge_id=start_edge_id,
            initial_temp=1000.0,
            cooling_rate=0.95,
            iterations_per_temp=100,
            min_temp=1.0,
        )
    else:
        print("\n[Step 2] 合并链边级贪心（最近邻）...")
        if start_edge_id is None:
            cand = generate_start_edge_candidates(
                required_edge_tasks, topo_graph, groups, edge_task_map, adjacency
            )
            start_edge_id = cand[0][0] if cand else required_edge_tasks[0].edge_id
        edge_order, edge_directions = _greedy_merged_edge_visit_order(
            required_edge_tasks, edge_task_map, adjacency, start_edge_id
        )

    print(f"[mission-order] visit_order={edge_order!r}")

    print("\n[Step 3] 构建连续任务（inspect 沿链折线截取）...")
    mission = build_optimized_mission(
        edge_order,
        edge_directions,
        required_edge_tasks,
        topo_graph,
        edge_task_map,
        groups,
        adjacency,
        connect_planner="dijkstra",
        cost_config=None,
    )
    mission.task_by_id = {str(task.edge_id): task for task in required_edge_tasks}

    print("\n" + "="*70)
    print("[全局拓扑优化] 完成")
    print("="*70)

    return mission
