"""
Topology-aware Mission 启发式优化（第六阶段）

第一版：全局 topology-aware greedy 边序优化，复用现有转移评估与 Mission 构建。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple
import math

from core.topo import TopoGraph
from core.topo_task import EdgeTask
from core.topo_plan import (
    build_edge_adjacency_simple,
    compute_edge_centroids,
    get_edge_inspection_geometry_with_direction,
    _polyline_length,
    group_edges_spatially,
)
from core.topo_global_optimizer import build_optimized_mission
from planner.topo_dijkstra import generate_connection_segment_with_planner


def _pick_start_edge(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
) -> Tuple[str, str]:
    """选择起点边与方向：端点边优先，巡检点更多者优先。"""
    best_id = edge_tasks[0].edge_id
    best_score = -1.0
    best_direction = "forward"

    for task in edge_tasks:
        u_node = topo_graph.get_node(task.u)
        v_node = topo_graph.get_node(task.v)
        endpoint_bonus = 0.0
        direction = "forward"
        if u_node and u_node.deg == 1:
            endpoint_bonus = 100.0
            direction = "forward"
        elif v_node and v_node.deg == 1:
            endpoint_bonus = 100.0
            direction = "reverse"
        score = task.num_points * 10.0 + endpoint_bonus + task.len2d * 0.01
        if score > best_score:
            best_score = score
            best_id = task.edge_id
            best_direction = direction

    return best_id, best_direction


def optimize_edge_task_order(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    adjacency: Optional[dict] = None,
    start_edge_id: Optional[str] = None,
    connect_planner: str = "dijkstra",
    cost_config: Optional[dict] = None,
) -> Tuple[List[str], Dict[str, str]]:
    """
    Topology-aware greedy：优先拓扑邻接边，connect 代价最短，巡检点更多者优先。

    Returns:
        (edge_visit_order, edge_directions)
    """
    if not edge_tasks:
        return [], {}

    edge_task_map = {t.edge_id: t for t in edge_tasks}
    edge_ids = [t.edge_id for t in edge_tasks]
    n = len(edge_ids)
    if adjacency is None:
        adjacency = build_edge_adjacency_simple(topo_graph)

    geoms: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}
    inspect_len = [[0.0, 0.0] for _ in range(n)]  # [edge_idx][dir_idx]
    for i, eid in enumerate(edge_ids):
        for d_idx, d_name in enumerate(("forward", "reverse")):
            g = get_edge_inspection_geometry_with_direction(edge_task_map[eid], d_name)
            geoms[(i, d_idx)] = g
            inspect_len[i][d_idx] = _polyline_length(g) if len(g) >= 2 else 0.0

    # cost[i][di][j][dj] = connect(i,di -> j,dj) + inspect_len(j,dj)
    inf = float("inf")
    cost = [[[[inf for _ in range(2)] for _ in range(n)] for _ in range(2)] for _ in range(n)]
    for i, eid_i in enumerate(edge_ids):
        for di in range(2):
            gi = geoms[(i, di)]
            if len(gi) < 2:
                continue
            p_end = gi[-1]
            for j, eid_j in enumerate(edge_ids):
                if i == j:
                    continue
                for dj in range(2):
                    gj = geoms[(j, dj)]
                    if len(gj) < 2:
                        continue
                    p_start = gj[0]
                    _, conn = generate_connection_segment_with_planner(
                        p_end,
                        p_start,
                        topo_graph,
                        edge_task_map,
                        connect_planner=connect_planner,
                        cost_config=cost_config,
                        from_edge_id=eid_i,
                        to_edge_id=eid_j,
                    )
                    cost[i][di][j][dj] = float(conn) + inspect_len[j][dj]

    idx_by_eid = {eid: i for i, eid in enumerate(edge_ids)}
    fixed_start = idx_by_eid.get(start_edge_id) if start_edge_id in idx_by_eid else None

    def route_cost_with_dirs(order_idx: List[int], dirs: List[int]) -> float:
        if not order_idx:
            return 0.0
        total = inspect_len[order_idx[0]][dirs[0]]
        for k in range(len(order_idx) - 1):
            i = order_idx[k]
            di = dirs[k]
            j = order_idx[k + 1]
            dj = dirs[k + 1]
            total += cost[i][di][j][dj]
        return total

    def solve_dirs_for_order(order_idx: List[int]) -> Tuple[List[int], float]:
        m = len(order_idx)
        if m == 0:
            return [], 0.0
        dp = [[inf, inf] for _ in range(m)]
        parent = [[None, None] for _ in range(m)]
        dp[0][0] = inspect_len[order_idx[0]][0]
        dp[0][1] = inspect_len[order_idx[0]][1]
        for k in range(1, m):
            i = order_idx[k - 1]
            j = order_idx[k]
            for dj in (0, 1):
                best = inf
                best_prev = None
                for di in (0, 1):
                    c = dp[k - 1][di] + cost[i][di][j][dj]
                    if c < best:
                        best = c
                        best_prev = di
                dp[k][dj] = best
                parent[k][dj] = best_prev
        end_dir = 0 if dp[m - 1][0] <= dp[m - 1][1] else 1
        best_total = dp[m - 1][end_dir]
        dirs = [0] * m
        dirs[m - 1] = end_dir
        for k in range(m - 1, 0, -1):
            dirs[k - 1] = parent[k][dirs[k]]
        return dirs, best_total

    def held_karp_exact() -> Tuple[List[int], List[int], float]:
        # dp[mask][last][dir]
        size = 1 << n
        dp = [[[inf, inf] for _ in range(n)] for _ in range(size)]
        parent = [[[(None, None), (None, None)] for _ in range(n)] for _ in range(size)]

        starts = [fixed_start] if fixed_start is not None else list(range(n))
        for s in starts:
            mask = 1 << s
            dp[mask][s][0] = inspect_len[s][0]
            dp[mask][s][1] = inspect_len[s][1]

        for mask in range(size):
            for last in range(n):
                if ((mask >> last) & 1) == 0:
                    continue
                for d_last in (0, 1):
                    base = dp[mask][last][d_last]
                    if not math.isfinite(base):
                        continue
                    for nxt in range(n):
                        if (mask >> nxt) & 1:
                            continue
                        nmask = mask | (1 << nxt)
                        for d_nxt in (0, 1):
                            cand = base + cost[last][d_last][nxt][d_nxt]
                            if cand < dp[nmask][nxt][d_nxt]:
                                dp[nmask][nxt][d_nxt] = cand
                                parent[nmask][nxt][d_nxt] = (last, d_last)

        full = (1 << n) - 1
        best = inf
        best_last = None
        best_dir = None
        for last in range(n):
            for d in (0, 1):
                val = dp[full][last][d]
                if val < best:
                    best = val
                    best_last = last
                    best_dir = d

        order_rev: List[int] = []
        dirs_rev: List[int] = []
        mask = full
        last = best_last
        dcur = best_dir
        while last is not None:
            order_rev.append(last)
            dirs_rev.append(dcur)
            prev_last, prev_dir = parent[mask][last][dcur]
            mask ^= (1 << last)
            last, dcur = prev_last, prev_dir
        order = list(reversed(order_rev))
        dirs = list(reversed(dirs_rev))
        return order, dirs, best

    def greedy_order_only() -> List[int]:
        trans_min = [[inf for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                trans_min[i][j] = min(
                    cost[i][0][j][0], cost[i][0][j][1], cost[i][1][j][0], cost[i][1][j][1]
                )
        if fixed_start is not None:
            start_idx = fixed_start
        else:
            sid, _ = _pick_start_edge(edge_tasks, topo_graph, edge_task_map)
            start_idx = idx_by_eid.get(sid, 0)
        unvisited = set(range(n))
        order = [start_idx]
        unvisited.remove(start_idx)
        while unvisited:
            cur = order[-1]
            nxt = min(unvisited, key=lambda j: trans_min[cur][j])
            order.append(nxt)
            unvisited.remove(nxt)
        return order

    def local_search_order(order: List[int]) -> Tuple[List[int], List[int], float]:
        cur_order = list(order)
        cur_dirs, cur_cost = solve_dirs_for_order(cur_order)
        improved = True
        loops = 0
        fixed_prefix = 1 if fixed_start is not None else 0
        while improved and loops < 8:
            improved = False
            loops += 1

            # 2-opt (order)
            for i in range(fixed_prefix, n - 1):
                for k in range(i + 1, n):
                    cand = cur_order[:i] + list(reversed(cur_order[i : k + 1])) + cur_order[k + 1 :]
                    cand_dirs, cand_cost = solve_dirs_for_order(cand)
                    if cand_cost + 1e-6 < cur_cost:
                        cur_order, cur_dirs, cur_cost = cand, cand_dirs, cand_cost
                        improved = True
                        break
                if improved:
                    break
            if improved:
                continue

            # Or-opt (single insertion)
            for i in range(fixed_prefix, n):
                node = cur_order[i]
                reduced = cur_order[:i] + cur_order[i + 1 :]
                for j in range(fixed_prefix, len(reduced) + 1):
                    cand = reduced[:j] + [node] + reduced[j:]
                    cand_dirs, cand_cost = solve_dirs_for_order(cand)
                    if cand_cost + 1e-6 < cur_cost:
                        cur_order, cur_dirs, cur_cost = cand, cand_dirs, cand_cost
                        improved = True
                        break
                if improved:
                    break
        return cur_order, cur_dirs, cur_cost

    if n <= 12:
        order_idx, dirs_idx, best_cost = held_karp_exact()
        print(f"[优化器] 使用精确动态规划（Held-Karp），任务数={n}，总代价={best_cost:.2f}")
    else:
        init_order = greedy_order_only()
        init_dirs, init_cost = solve_dirs_for_order(init_order)
        order_idx, dirs_idx, best_cost = local_search_order(init_order)
        print(
            f"[优化器] 使用贪心初解 + 局部搜索，任务数={n}，"
            f"初始总代价={init_cost:.2f}，优化后总代价={best_cost:.2f}，优化提升={init_cost-best_cost:.2f}"
        )

    order = [edge_ids[i] for i in order_idx]
    directions = {
        edge_ids[order_idx[k]]: ("forward" if dirs_idx[k] == 0 else "reverse")
        for k in range(len(order_idx))
    }
    return order, directions


def build_optimized_unified_mission(
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    groups: Optional[dict] = None,
    eps: float = 150.0,
    start_edge_id: Optional[str] = None,
    connect_planner: str = "bfs",
    cost_config: Optional[dict] = None,
):
    """
    基于优化边序构建 GroupedContinuousMission。

    流程：
        optimize_edge_task_order → build_optimized_mission

    说明：使用 build_optimized_mission 按显式边序生成 inspect/connect 段，
    与 build_grouped_continuous_mission 组内二次贪心不同，可保留拓扑贪心顺序。
    返回类型仍为 GroupedContinuousMission。
    """
    required_edge_tasks = [t for t in edge_tasks if (t.num_points or 0) > 0]
    skipped_edges = [t for t in edge_tasks if (t.num_points or 0) <= 0]
    for t in skipped_edges:
        print(
            f"[DEBUG] dead-end branch skipped/truncated edge={t.edge_id} "
            "reason=no_required_inspection_points"
        )
    edge_tasks = required_edge_tasks
    if not edge_tasks:
        raise ValueError("No required inspection edges after filtering edge_tasks")

    edge_task_map = {t.edge_id: t for t in edge_tasks}
    adjacency = build_edge_adjacency_simple(topo_graph)

    if groups is None:
        centroids = compute_edge_centroids(edge_tasks)
        groups = group_edges_spatially(edge_tasks, centroids, eps=eps)

    edge_order, edge_directions = optimize_edge_task_order(
        edge_tasks,
        topo_graph,
        adjacency,
        start_edge_id=start_edge_id,
        connect_planner=connect_planner,
        cost_config=cost_config,
    )

    mission = build_optimized_mission(
        edge_order,
        edge_directions,
        edge_tasks,
        topo_graph,
        edge_task_map,
        groups,
        adjacency,
        connect_planner=connect_planner,
        cost_config=cost_config,
    )
    return mission, edge_order, edge_directions
