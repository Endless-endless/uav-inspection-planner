"""
拓扑节点路径：BFS vs Dijkstra 对比

输出: result/dijkstra_test/dijkstra_compare.json
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


def _path_length_from_geometry(geom):
    import numpy as np
    if len(geom) < 2:
        return 0.0
    total = 0.0
    for i in range(len(geom) - 1):
        total += float(np.linalg.norm(np.array(geom[i + 1]) - np.array(geom[i])))
    return total


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_dir = os.path.join(root, "result", "dijkstra_test")
    out_json = os.path.join(out_dir, "dijkstra_compare.json")

    print("=" * 60)
    print("BFS vs Dijkstra (topo node paths)")
    print("=" * 60)

    unified = load_unified_input_from_json(sample)
    lines = unified_input_to_independent_lines(unified)
    topo_graph, _, _ = build_topo_graph_from_independent_lines(lines)

    pairs = [
        ("merged_0", "line_trunk_end"),
        ("merged_1", "line_branch_n_end"),
        ("line_west_end", "line_branch_n_end"),
    ]

    results = []
    for start_id, goal_id in pairs:
        if start_id not in topo_graph.nodes or goal_id not in topo_graph.nodes:
            print(f"[SKIP] {start_id} -> {goal_id} (node missing)")
            continue

        bfs_path = get_node_path_bfs(topo_graph, start_id, goal_id)
        bfs_hops = max(0, len(bfs_path) - 1) if bfs_path else None
        bfs_edge_cost_sum = estimate_bfs_path_cost(topo_graph, bfs_path) if bfs_path else None

        sa = topo_graph.get_node(start_id)
        sb = topo_graph.get_node(goal_id)
        if sa and sb:
            pa, pb = sa.pos2d, sb.pos2d
            bfs_geom, bfs_geom_len = expand_node_path_to_geometry(pa, pb, bfs_path, topo_graph) if bfs_path else ([], 0.0)
        else:
            bfs_geom_len = None

        dj = dijkstra_shortest_path(topo_graph, start_id, goal_id)
        dj_path = dj["node_path"] if dj else []
        dj_cost = dj["total_cost"] if dj else None
        dj_edge_path = dj["edge_path"] if dj else []

        paths_equal = bfs_path == dj_path if bfs_path and dj_path else False
        dj_shorter_or_equal = (
            dj_cost is not None
            and bfs_edge_cost_sum is not None
            and dj_cost <= bfs_edge_cost_sum + 1e-6
        )

        entry = {
            "start_node": start_id,
            "goal_node": goal_id,
            "bfs": {
                "node_path": bfs_path,
                "hop_count": bfs_hops,
                "sum_edge_len2d": bfs_edge_cost_sum,
                "geometry_length": bfs_geom_len,
            },
            "dijkstra": {
                "node_path": dj_path,
                "edge_path": dj_edge_path,
                "total_cost": dj_cost,
            },
            "paths_equal": paths_equal,
            "dijkstra_cost_leq_bfs_edge_sum": dj_shorter_or_equal,
        }
        results.append(entry)

        print(f"\n{start_id} -> {goal_id}")
        print(f"  BFS hops={bfs_hops} path={bfs_path}")
        print(f"  BFS sum(len2d)={bfs_edge_cost_sum} geom_len={bfs_geom_len}")
        print(f"  Dijkstra cost={dj_cost} path={dj_path}")
        print(f"  equal={paths_equal} dijkstra_leq_bfs_sum={dj_shorter_or_equal}")

    payload = {
        "input": sample,
        "node_count": len(topo_graph.nodes),
        "edge_count": len(topo_graph.edges),
        "pairs": results,
    }

    os.makedirs(out_dir, exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"\n[OK] saved: {out_json}")
    print("=" * 60)


if __name__ == "__main__":
    main()
