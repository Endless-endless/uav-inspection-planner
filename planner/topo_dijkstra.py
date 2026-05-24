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
from core.topo_plan import (
    get_shortest_path,
    find_nearest_topo_node,
    _polyline_length,
    _project_point_to_polyline_distance,
    _slice_polyline_by_distance,
)
from weather.weather_cost import compute_edge_weather_penalty


def edge_cost(edge: TopoEdge, cost_config: Optional[Dict[str, Any]] = None) -> float:
    """
    单条拓扑边的通行代价（默认 = len2d / 折线几何长度）。
    """
    cfg = cost_config or {}
    weight_key = cfg.get("length_field", "len2d")

    if weight_key == "len2d" and edge.len2d and edge.len2d > 0:
        base = float(edge.len2d)
    else:
        length_attr = getattr(edge, weight_key, None)
        if isinstance(length_attr, (int, float)) and length_attr > 0:
            base = float(length_attr)
        elif edge.polyline and len(edge.polyline) >= 2:
            pts = np.array(edge.polyline, dtype=np.float64)
            base = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        else:
            base = 1.0

    weather_cfg = cfg.get("weather") or {}
    if not weather_cfg.get("enabled"):
        return base

    poly = list(edge.polyline or [])
    if len(poly) < 2:
        return base
    cache = cfg.setdefault("_weather_edge_cache", {})
    cache_key = edge.id
    penalty = cache.get(cache_key)
    if penalty is None:
        penalty_info = compute_edge_weather_penalty(
            poly,
            weather_cfg.get("weather_zones") or [],
            type_weights=weather_cfg.get("type_weights"),
            weather_weight=float(weather_cfg.get("weather_weight", 1.0)),
        )
        penalty = float(penalty_info.get("total_penalty", 0.0))
        cache[cache_key] = penalty
    return base + penalty


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
    from_edge_id: Optional[str] = None,
    to_edge_id: Optional[str] = None,
) -> Tuple[List[Tuple[float, float]], float]:
    """
    沿拓扑图生成连接段；connect_planner: 'bfs' | 'dijkstra'。

    Dijkstra 失败时 fallback 到 BFS。
    """
    def _append_polyline(dst: List[Tuple[float, float]], src: List[Tuple[float, float]]) -> None:
        if not src:
            return
        src2 = [(float(p[0]), float(p[1])) for p in src]
        if not dst:
            dst.extend(src2)
            return
        if np.linalg.norm(np.array(dst[-1]) - np.array(src2[0])) <= 1e-6:
            dst.extend(src2[1:])
        else:
            dst.extend(src2)

    def _geom_length(geom: List[Tuple[float, float]]) -> float:
        if len(geom) < 2:
            return 0.0
        return float(sum(np.linalg.norm(np.array(geom[i + 1]) - np.array(geom[i])) for i in range(len(geom) - 1)))

    def _point_to_anchor(edge_task, point_xy: Tuple[float, float], anchor_node: str):
        poly = list(getattr(edge_task, "polyline", None) or [])
        if len(poly) < 2:
            return None
        s = _project_point_to_polyline_distance(point_xy, poly)
        if s is None:
            return None
        total = _polyline_length(poly)
        s = max(0.0, min(float(s), total))
        if anchor_node == edge_task.u:
            geom = _slice_polyline_by_distance(poly, 0.0, s)
            return {"cost": s, "point_to_anchor": list(reversed(geom))}
        if anchor_node == edge_task.v:
            geom = _slice_polyline_by_distance(poly, s, total)
            return {"cost": total - s, "point_to_anchor": geom}
        return None

    def _anchor_to_point(edge_task, point_xy: Tuple[float, float], anchor_node: str):
        info = _point_to_anchor(edge_task, point_xy, anchor_node)
        if not info:
            return None
        return list(reversed(info["point_to_anchor"]))

    from_edge = edge_task_map.get(from_edge_id) if from_edge_id else None
    to_edge = edge_task_map.get(to_edge_id) if to_edge_id else None

    # 同边直接切片（target-aware）
    if from_edge is not None and to_edge is not None and from_edge.edge_id == to_edge.edge_id:
        poly = list(from_edge.polyline or [])
        if len(poly) >= 2:
            sa = _project_point_to_polyline_distance(point_a, poly)
            sb = _project_point_to_polyline_distance(point_b, poly)
            if sa is not None and sb is not None:
                sliced = _slice_polyline_by_distance(poly, sa, sb)
                return sliced, _polyline_length(sliced)

    # 端点锚点组合（dijkstra/bfs 按 connect_planner 选）
    if from_edge is not None and to_edge is not None:
        best = None
        for a_anchor in (from_edge.u, from_edge.v):
            a_info = _point_to_anchor(from_edge, point_a, a_anchor)
            if not a_info:
                continue
            for b_anchor in (to_edge.u, to_edge.v):
                b_info = _point_to_anchor(to_edge, point_b, b_anchor)
                if not b_info:
                    continue

                if a_anchor == b_anchor:
                    node_path = [a_anchor]
                    middle_cost = 0.0
                elif connect_planner.lower() == "dijkstra":
                    rs = dijkstra_shortest_path(topo_graph, a_anchor, b_anchor, cost_config=cost_config)
                    node_path = (rs or {}).get("node_path") or []
                    middle_cost = float((rs or {}).get("total_cost", float("inf")))
                else:
                    node_path = get_shortest_path(topo_graph, a_anchor, b_anchor) or []
                    middle_cost = estimate_bfs_path_cost(topo_graph, node_path) if node_path else float("inf")

                if not node_path:
                    continue

                middle_geo, _ = expand_node_path_to_geometry(
                    topo_graph.get_node(node_path[0]).pos2d,
                    topo_graph.get_node(node_path[-1]).pos2d,
                    node_path,
                    topo_graph,
                )
                if len(middle_geo) >= 2:
                    middle_geo = middle_geo[1:-1]  # 去除首尾节点点位，避免重复
                else:
                    middle_geo = []

                total_cost = float(a_info["cost"]) + float(middle_cost) + float(b_info["cost"])
                if best is None or total_cost < best["total_cost"]:
                    best = {
                        "total_cost": total_cost,
                        "a_geo": a_info["point_to_anchor"],
                        "middle_geo": middle_geo,
                        "b_geo": _anchor_to_point(to_edge, point_b, b_anchor) or [point_b],
                    }

        if best is not None:
            geometry: List[Tuple[float, float]] = []
            _append_polyline(geometry, best["a_geo"])
            _append_polyline(geometry, best["middle_geo"])
            _append_polyline(geometry, best["b_geo"])
            if not geometry:
                geometry = [point_a, point_b]
            if np.linalg.norm(np.array(geometry[0]) - np.array(point_a)) > 1e-6:
                geometry.insert(0, (float(point_a[0]), float(point_a[1])))
            if np.linalg.norm(np.array(geometry[-1]) - np.array(point_b)) > 1e-6:
                geometry.append((float(point_b[0]), float(point_b[1])))
            return geometry, _geom_length(geometry)

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
