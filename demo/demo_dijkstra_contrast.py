"""
BFS vs Dijkstra 对比专用测试（sample_dijkstra_contrast_input.json）

预期：
- BFS：2 跳 highway 路径（代价 ~1727px）
- Dijkstra：4 跳 detour 路径（代价 1000px）

输出: result/dijkstra_test/dijkstra_contrast.json
"""

import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_adapter import build_topo_graph_from_independent_lines, unified_input_to_independent_lines
from planner.topo_dijkstra import (
    dijkstra_shortest_path,
    estimate_bfs_path_cost,
    expand_node_path_to_geometry,
    get_node_path_bfs,
)


def _resolve_node_id(topo_graph, preferred_ids, position, tolerance=30.0):
    """按 id 或坐标匹配节点。"""
    import numpy as np

    for pid in preferred_ids:
        if pid in topo_graph.nodes:
            return pid

    if position is None:
        return None

    best_id = None
    best_dist = float("inf")
    for nid, node in topo_graph.nodes.items():
        d = np.linalg.norm(np.array(node.pos2d) - np.array(position))
        if d < best_dist:
            best_dist = d
            best_id = nid
    if best_dist <= tolerance:
        return best_id
    return None


def _geometry_length(geom):
    import numpy as np
    if len(geom) < 2:
        return 0.0
    total = 0.0
    for i in range(len(geom) - 1):
        total += float(np.linalg.norm(np.array(geom[i + 1]) - np.array(geom[i])))
    return total


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample = os.path.join(root, "data", "sample_dijkstra_contrast_input.json")
    out_dir = os.path.join(root, "result", "dijkstra_test")
    out_json = os.path.join(out_dir, "dijkstra_contrast.json")

    print("=" * 70)
    print("Dijkstra Contrast Test (BFS hops vs Dijkstra cost)")
    print("=" * 70)
    print(f"Input: {sample}")
    print()

    unified = load_unified_input_from_json(sample)
    lines = unified_input_to_independent_lines(unified)
    topo_graph, _, _ = build_topo_graph_from_independent_lines(lines)

    start_id = _resolve_node_id(
        topo_graph,
        ["line_highway_start", "line_detour_1_start"],
        (0.0, 0.0),
    )
    goal_id = _resolve_node_id(
        topo_graph,
        ["line_highway_end", "line_detour_4_end"],
        (1000.0, 0.0),
    )

    if not start_id or not goal_id:
        print("[FAIL] cannot resolve start/goal nodes")
        print("Available nodes:")
        for nid, n in topo_graph.nodes.items():
            print(f"  {nid} @ {n.pos2d}")
        sys.exit(1)

    sa = topo_graph.get_node(start_id)
    sb = topo_graph.get_node(goal_id)
    pa, pb = sa.pos2d, sb.pos2d

    print(f"Start node: {start_id} @ {pa}")
    print(f"Goal node:  {goal_id} @ {pb}")
    print(f"Topo nodes: {len(topo_graph.nodes)}, edges: {len(topo_graph.edges)}")
    print()

    bfs_path = get_node_path_bfs(topo_graph, start_id, goal_id)
    bfs_hops = max(0, len(bfs_path) - 1) if bfs_path else None
    bfs_edge_sum = estimate_bfs_path_cost(topo_graph, bfs_path) if bfs_path else None
    bfs_geom, bfs_geom_len = (
        expand_node_path_to_geometry(pa, pb, bfs_path, topo_graph)
        if bfs_path
        else ([], 0.0)
    )

    dj = dijkstra_shortest_path(topo_graph, start_id, goal_id)
    dj_path = dj["node_path"] if dj else []
    dj_cost = dj["total_cost"] if dj else None
    dj_edge_path = dj["edge_path"] if dj else []
    dj_geom, dj_geom_len = (
        expand_node_path_to_geometry(pa, pb, dj_path, topo_graph)
        if dj_path
        else ([], 0.0)
    )

    paths_equal = bfs_path == dj_path
    dijkstra_shorter = (
        dj_cost is not None
        and bfs_edge_sum is not None
        and dj_cost < bfs_edge_sum - 1e-6
    )
    bfs_fewer_hops = (
        bfs_hops is not None
        and dj_path
        and bfs_hops < len(dj_path) - 1
    )

    print("BFS:")
    print(f"  node_path: {bfs_path}")
    print(f"  hop_count: {bfs_hops}")
    print(f"  sum_edge_len2d: {bfs_edge_sum}")
    print(f"  geometry_length: {bfs_geom_len:.2f}")
    print()
    print("Dijkstra:")
    print(f"  node_path: {dj_path}")
    print(f"  edge_path: {dj_edge_path}")
    print(f"  total_cost: {dj_cost}")
    print(f"  hop_count: {max(0, len(dj_path) - 1) if dj_path else None}")
    print(f"  geometry_length: {dj_geom_len:.2f}")
    print()
    print(f"paths_equal: {paths_equal}")
    print(f"bfs_fewer_hops: {bfs_fewer_hops}")
    print(f"dijkstra_shorter (cost): {dijkstra_shorter}")
    print()

    if dijkstra_shorter and bfs_fewer_hops:
        print("[OK] Contrast demonstrated: BFS fewer hops, Dijkstra lower cost")
    elif paths_equal:
        print("[WARN] Paths identical on this build — check topo split / node merge")
    else:
        print("[INFO] See metrics above for partial contrast")

    payload = {
        "input_file": sample,
        "start_node": start_id,
        "goal_node": goal_id,
        "topo_summary": {
            "node_count": len(topo_graph.nodes),
            "edge_count": len(topo_graph.edges),
            "node_ids": list(topo_graph.nodes.keys()),
        },
        "bfs": {
            "node_path": bfs_path,
            "hop_count": bfs_hops,
            "sum_edge_len2d": bfs_edge_sum,
            "geometry_length": bfs_geom_len,
        },
        "dijkstra": {
            "node_path": dj_path,
            "edge_path": dj_edge_path,
            "total_cost": dj_cost,
            "hop_count": max(0, len(dj_path) - 1) if dj_path else None,
            "geometry_length": dj_geom_len,
        },
        "contrast": {
            "paths_equal": paths_equal,
            "bfs_fewer_hops": bfs_fewer_hops,
            "dijkstra_shorter": dijkstra_shorter,
            "demonstrated": bool(dijkstra_shorter and bfs_fewer_hops),
        },
    }

    os.makedirs(out_dir, exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"[OK] saved: {out_json}")
    print("=" * 70)


if __name__ == "__main__":
    main()
