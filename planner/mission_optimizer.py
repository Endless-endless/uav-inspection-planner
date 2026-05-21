"""
Topology-aware Mission 启发式优化（第六阶段）

第一版：全局 topology-aware greedy 边序优化，复用现有转移评估与 Mission 构建。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from core.topo import TopoGraph
from core.topo_task import EdgeTask
from core.topo_plan import (
    build_edge_adjacency_simple,
    compute_edge_centroids,
    evaluate_transition_with_geometry,
    get_edge_geometry_with_direction,
    group_edges_spatially,
)
from core.topo_global_optimizer import build_optimized_mission


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
) -> Tuple[List[str], Dict[str, str]]:
    """
    Topology-aware greedy：优先拓扑邻接边，connect 代价最短，巡检点更多者优先。

    Returns:
        (edge_visit_order, edge_directions)
    """
    if not edge_tasks:
        return [], {}

    edge_task_map = {t.edge_id: t for t in edge_tasks}
    if adjacency is None:
        adjacency = build_edge_adjacency_simple(topo_graph)

    if start_edge_id is None or start_edge_id not in edge_task_map:
        start_edge_id, start_direction = _pick_start_edge(
            edge_tasks, topo_graph, edge_task_map
        )
    else:
        start_edge = edge_task_map[start_edge_id]
        u_node = topo_graph.get_node(start_edge.u)
        v_node = topo_graph.get_node(start_edge.v)
        if u_node and u_node.deg == 1:
            start_direction = "forward"
        elif v_node and v_node.deg == 1:
            start_direction = "reverse"
        else:
            start_direction = "forward"

    unvisited = {t.edge_id for t in edge_tasks}
    order = [start_edge_id]
    directions = {start_edge_id: start_direction}
    unvisited.remove(start_edge_id)

    current_edge_id = start_edge_id
    current_direction = start_direction

    while unvisited:
        neighbor_candidates = [
            e for e in adjacency.get(current_edge_id, []) if e in unvisited
        ]
        candidates = neighbor_candidates if neighbor_candidates else list(unvisited)

        best_next = None
        best_direction = "forward"
        best_cost = float("inf")
        best_points = -1

        for candidate_id in candidates:
            plan = evaluate_transition_with_geometry(
                topo_graph,
                current_edge_id,
                current_direction,
                candidate_id,
                edge_task_map,
            )
            if plan is None:
                continue

            cost = float(plan.get("connect_length", plan.get("total_incremental_cost", float("inf"))))
            n_pts = edge_task_map[candidate_id].num_points

            is_neighbor = candidate_id in neighbor_candidates
            if is_neighbor:
                cost *= 0.5

            if cost < best_cost - 1e-6 or (
                abs(cost - best_cost) < 1e-6 and n_pts > best_points
            ):
                best_cost = cost
                best_next = candidate_id
                best_direction = plan["next_direction"]
                best_points = n_pts

        if best_next is None:
            best_next = max(
                candidates,
                key=lambda eid: edge_task_map[eid].num_points,
            )
            best_direction = "forward"

        order.append(best_next)
        directions[best_next] = best_direction
        unvisited.remove(best_next)
        current_edge_id = best_next
        current_direction = best_direction

    return order, directions


def build_optimized_unified_mission(
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    groups: Optional[dict] = None,
    eps: float = 150.0,
    start_edge_id: Optional[str] = None,
    connect_planner: str = "bfs",
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
        edge_tasks, topo_graph, adjacency, start_edge_id=start_edge_id
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
    )
    return mission, edge_order, edge_directions
