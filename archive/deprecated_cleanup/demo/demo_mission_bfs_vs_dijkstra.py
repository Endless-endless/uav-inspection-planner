"""
Mission 级 BFS vs Dijkstra connect planner 对比

输出: result/dijkstra_test/mission_bfs_vs_dijkstra.json
"""

import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_task_adapter import unified_input_to_edge_tasks
from planner.mission_analysis import analyze_mission, compare_mission_stats
from planner.mission_optimizer import (
    build_optimized_unified_mission,
    optimize_edge_task_order,
)
from core.topo_plan import build_edge_adjacency_simple, compute_edge_centroids, group_edges_spatially
from core.topo_global_optimizer import build_optimized_mission


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_dir = os.path.join(root, "result", "dijkstra_test")
    out_json = os.path.join(out_dir, "mission_bfs_vs_dijkstra.json")

    print("=" * 70)
    print("Mission: BFS vs Dijkstra connect planner")
    print("=" * 70)

    unified = load_unified_input_from_json(sample)
    topo_graph, edge_tasks, line_pts = unified_input_to_edge_tasks(unified, spacing=50.0)

    edge_task_map = {t.edge_id: t for t in edge_tasks}
    adjacency = build_edge_adjacency_simple(topo_graph)
    centroids = compute_edge_centroids(edge_tasks)
    groups = group_edges_spatially(edge_tasks, centroids, eps=150.0)
    edge_order, edge_dirs = optimize_edge_task_order(
        edge_tasks, topo_graph, adjacency
    )

    print("[1] BFS connect planner...")
    mission_bfs = build_optimized_mission(
        edge_order, edge_dirs, edge_tasks, topo_graph,
        edge_task_map, groups, adjacency, connect_planner="bfs",
    )
    stats_bfs = analyze_mission(mission_bfs)

    print("[2] Dijkstra connect planner...")
    mission_dj = build_optimized_mission(
        edge_order, edge_dirs, edge_tasks, topo_graph,
        edge_task_map, groups, adjacency, connect_planner="dijkstra",
    )
    stats_dj = analyze_mission(mission_dj)

    improvement = compare_mission_stats(stats_bfs, stats_dj)

    note = None
    if (
        stats_bfs["connect_length"] == stats_dj["connect_length"]
        and stats_bfs["visit_order"] == stats_dj["visit_order"]
    ):
        note = (
            "当前测试图中 BFS 与 Dijkstra 路径一致或 connect 代价差异不足，"
            "因拓扑边代价（len2d）在唯一路径上相同。"
        )

    payload = {
        "input_file": sample,
        "edge_visit_order_shared": edge_order,
        "baseline_bfs": stats_bfs,
        "optimized_dijkstra": stats_dj,
        "improvement_dijkstra_vs_bfs": improvement,
        "note": note,
    }

    os.makedirs(out_dir, exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print("-" * 70)
    print("BFS:")
    print(f"  total={stats_bfs['total_length']} connect={stats_bfs['connect_length']} "
          f"ratio={stats_bfs['connect_ratio']:.2%}")
    print(f"  visit_order={stats_bfs['visit_order']}")
    print("Dijkstra:")
    print(f"  total={stats_dj['total_length']} connect={stats_dj['connect_length']} "
          f"ratio={stats_dj['connect_ratio']:.2%}")
    print(f"  visit_order={stats_dj['visit_order']}")
    cr = improvement["connect_ratio"]
    print(f"connect_ratio delta: {cr['delta_percent']:+.1f}% improved={cr['improved']}")
    if note:
        print(f"\nNote: {note}")
    print(f"\n[OK] {out_json}")
    print("=" * 70)


if __name__ == "__main__":
    main()
