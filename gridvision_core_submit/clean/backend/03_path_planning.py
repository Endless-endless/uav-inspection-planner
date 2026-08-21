"""
GridVision 毕设答辩核心代码摘录 — 拓扑路径规划
源文件: planner/topo_dijkstra.py
行号: 98-165, 303-400
Dijkstra 最短路径与连接段生成。
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""

# ===== topo_dijkstra.py L98-L165 =====
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



# ===== topo_dijkstra.py L303-L400 =====
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
