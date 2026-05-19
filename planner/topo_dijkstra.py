"""
拓扑图上的加权最短路径（Dijkstra）

与 BFS（最少跳数）并列，按边长度等代价最小化路径。
不修改 TopoGraph 内部结构。
"""

from __future__ import annotations

import heapq
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

from core.topo import TopoGraph, TopoEdge
from core.topo_plan import get_shortest_path, find_nearest_topo_node


def edge_cost(edge: TopoEdge, cost_config: Optional[Dict[str, Any]] = None) -> float:
    """
    单条拓扑边的通行代价（默认 = len2d / 折线几何长度）。
    """
    cfg = cost_config or {}
    weight_key = cfg.get("length_field", "len2d")

    if weight_key == "len2d" and edge.len2d and edge.len2d > 0:
        return float(edge.len2d)

    length_attr = getattr(edge, weight_key, None)
    if isinstance(length_attr, (int, float)) and length_attr > 0:
        return float(length_attr)

    if edge.polyline and len(edge.polyline) >= 2:
        pts = np.array(edge.polyline, dtype=np.float64)
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))

    return 1.0


def _find_edge_between(topo_graph: TopoGraph, u: str, v: str) -> Optional[TopoEdge]:
    for edge in topo_graph.edges.values():
        if (edge.u == u and edge.v == v) or (edge.u == v and edge.v == u):
            return edge
    return None


def build_weighted_node_adjacency(
    topo_graph: TopoGraph,
    cost_config: Optional[Dict[str, Any]] = None,
) -> Dict[str, List[Tuple[str, str, float]]]:
    """
    构建节点邻接表：node_id -> [(neighbor_id, edge_id, cost), ...]
    平行边取代价较小的一条。
    """
    adj: Dict[str, List[Tuple[str, str, float]]] = {
        nid: [] for nid in topo_graph.nodes
    }

    for edge_id, edge in topo_graph.edges.items():
        c = edge_cost(edge, cost_config)
        adj[edge.u].append((edge.v, edge_id, c))
        adj[edge.v].append((edge.u, edge_id, c))

    return adj


def dijkstra_shortest_path(
    topo_graph: TopoGraph,
    start_node_id: str,
    goal_node_id: str,
    cost_config: Optional[Dict[str, Any]] = None,
) -> Optional[Dict[str, Any]]:
    """
    节点间最小总代价路径（Dijkstra）。

    Returns:
        {"node_path", "edge_path", "total_cost"} 或 None（不可达）
    """
    if start_node_id not in topo_graph.nodes or goal_node_id not in topo_graph.nodes:
        return None

    if start_node_id == goal_node_id:
        return {
            "node_path": [start_node_id],
            "edge_path": [],
            "total_cost": 0.0,
        }

    adj = build_weighted_node_adjacency(topo_graph, cost_config)
    dist: Dict[str, float] = {start_node_id: 0.0}
    prev_node: Dict[str, Optional[str]] = {start_node_id: None}
    prev_edge: Dict[str, Optional[str]] = {start_node_id: None}
    heap: List[Tuple[float, str]] = [(0.0, start_node_id)]
    settled: set = set()

    while heap:
        d, u = heapq.heappop(heap)
        if u in settled:
            continue
        settled.add(u)

        if u == goal_node_id:
            break

        for v, eid, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev_node[v] = u
                prev_edge[v] = eid
                heapq.heappush(heap, (nd, v))

    if goal_node_id not in dist:
        return None

    node_path: List[str] = []
    edge_path: List[str] = []
    cur = goal_node_id
    while cur is not None:
        node_path.append(cur)
        eid = prev_edge.get(cur)
        if eid and cur != start_node_id:
            edge_path.append(eid)
        cur = prev_node.get(cur)
    node_path.reverse()
    edge_path.reverse()

    return {
        "node_path": node_path,
        "edge_path": edge_path,
        "total_cost": float(dist[goal_node_id]),
    }


def reconstruct_polyline_from_edge_path(
    edge_path: List[str],
    topo_graph: TopoGraph,
    node_path: Optional[List[str]] = None,
) -> List[Tuple[float, float]]:
    """
    将拓扑边序列展开为 2D 折线点列（用于 connect segment）。
    """
    if not edge_path:
        return []

    geometry: List[Tuple[float, float]] = []

    if node_path and len(node_path) >= 2:
        pairs = list(zip(node_path[:-1], node_path[1:]))
    else:
        pairs = []
        for eid in edge_path:
            e = topo_graph.edges.get(eid)
            if e:
                pairs.append((e.u, e.v))

    for i, eid in enumerate(edge_path):
        edge = topo_graph.edges.get(eid)
        if not edge or not edge.polyline:
            continue

        if node_path and i < len(pairs):
            u, v = pairs[i]
            poly = (
                edge.polyline
                if edge.u == u and edge.v == v
                else list(reversed(edge.polyline))
            )
        else:
            poly = edge.polyline

        if geometry:
            last = geometry[-1]
            if np.linalg.norm(np.array(poly[0]) - np.array(last)) > 0.1:
                geometry.extend(poly)
            else:
                geometry.extend(poly[1:])
        else:
            geometry.extend(poly)

    return geometry


def expand_node_path_to_geometry(
    point_a: Tuple[float, float],
    point_b: Tuple[float, float],
    node_path: List[str],
    topo_graph: TopoGraph,
) -> Tuple[List[Tuple[float, float]], float]:
    """将节点路径展开为完整 connect 几何（含起终点接驳）。"""
    geometry: List[Tuple[float, float]] = [point_a]

    if not node_path:
        geometry.append(point_b)
        length = float(np.linalg.norm(np.array(point_b) - np.array(point_a)))
        return geometry, length

    node_a = node_path[0]
    node_b = node_path[-1]
    node_a_obj = topo_graph.get_node(node_a)
    node_b_obj = topo_graph.get_node(node_b)

    if node_a_obj and np.linalg.norm(np.array(point_a) - np.array(node_a_obj.pos2d)) > 1.0:
        geometry.append(node_a_obj.pos2d)

    for i in range(len(node_path) - 1):
        u, v = node_path[i], node_path[i + 1]
        edge = _find_edge_between(topo_graph, u, v)
        if edge and edge.polyline:
            poly = (
                edge.polyline
                if edge.u == u and edge.v == v
                else list(reversed(edge.polyline))
            )
            if geometry:
                last = geometry[-1]
                if np.linalg.norm(np.array(poly[0]) - np.array(last)) > 0.1:
                    geometry.extend(poly)
                else:
                    geometry.extend(poly[1:])
            else:
                geometry.extend(poly)
        else:
            v_node = topo_graph.get_node(v)
            if v_node:
                geometry.append(v_node.pos2d)

    if node_b_obj and np.linalg.norm(np.array(point_b) - np.array(node_b_obj.pos2d)) > 1.0:
        geometry.append(node_b_obj.pos2d)

    geometry.append(point_b)

    length = 0.0
    for i in range(len(geometry) - 1):
        length += float(
            np.linalg.norm(np.array(geometry[i + 1]) - np.array(geometry[i]))
        )
    return geometry, length


def estimate_bfs_path_cost(
    topo_graph: TopoGraph,
    node_path: List[str],
) -> float:
    """按 BFS 节点路径累加拓扑边 len2d（用于对比）。"""
    if len(node_path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(node_path) - 1):
        edge = _find_edge_between(topo_graph, node_path[i], node_path[i + 1])
        if edge:
            total += edge_cost(edge)
        else:
            u = topo_graph.get_node(node_path[i])
            v = topo_graph.get_node(node_path[i + 1])
            if u and v:
                total += float(
                    np.linalg.norm(np.array(u.pos2d) - np.array(v.pos2d))
                )
    return total


def get_node_path_bfs(
    topo_graph: TopoGraph,
    start_node_id: str,
    goal_node_id: str,
) -> List[str]:
    """BFS 节点路径（复用 core.topo_plan.get_shortest_path）。"""
    return get_shortest_path(topo_graph, start_node_id, goal_node_id)


def generate_connection_segment_with_planner(
    point_a: Tuple[float, float],
    point_b: Tuple[float, float],
    topo_graph: TopoGraph,
    edge_task_map: dict,
    connect_planner: str = "bfs",
    cost_config: Optional[Dict[str, Any]] = None,
    use_proximity_bfs: bool = True,
) -> Tuple[List[Tuple[float, float]], float]:
    """
    沿拓扑图生成连接段；connect_planner: 'bfs' | 'dijkstra'。

    Dijkstra 失败时 fallback 到 BFS。
    """
    _ = edge_task_map  # 与现有 API 兼容

    node_a = find_nearest_topo_node(point_a, topo_graph)
    node_b = find_nearest_topo_node(point_b, topo_graph)

    if node_a is None or node_b is None:
        geometry = [point_a, point_b]
        length = float(np.linalg.norm(np.array(point_b) - np.array(point_a)))
        return geometry, length

    node_path: List[str] = []
    planner_used = connect_planner.lower()

    if planner_used == "dijkstra":
        result = dijkstra_shortest_path(
            topo_graph, node_a, node_b, cost_config=cost_config
        )
        if result and result["node_path"]:
            node_path = result["node_path"]
        else:
            planner_used = "bfs"

    if planner_used == "bfs" or not node_path:
        if use_proximity_bfs:
            from core.topo_plan import get_shortest_path_with_proximity

            direct_distance = np.linalg.norm(np.array(point_b) - np.array(point_a))
            proximity_threshold = max(1500.0, direct_distance * 1.2)
            node_path = get_shortest_path_with_proximity(
                topo_graph, node_a, node_b, proximity_threshold=proximity_threshold
            )
        else:
            node_path = get_shortest_path(topo_graph, node_a, node_b)

    if not node_path:
        geometry = [point_a, point_b]
        length = float(np.linalg.norm(np.array(point_b) - np.array(point_a)))
        return geometry, length

    return expand_node_path_to_geometry(point_a, point_b, node_path, topo_graph)
