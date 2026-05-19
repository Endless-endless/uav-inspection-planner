"""
Baseline vs Topology-aware Optimized Mission 对比

用法:
    python demo/demo_mission_compare.py

输出:
    result/unified_mission/mission_output.json (baseline, 若不存在则生成)
    result/unified_mission/optimized_mission_output.json
    result/unified_mission/mission_compare.json
"""

import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_mission_adapter import (
    export_unified_mission,
    unified_input_to_mission,
)
from input.unified_task_adapter import unified_input_to_edge_tasks
from planner.mission_analysis import (
    analyze_mission,
    analyze_mission_json,
    compare_mission_stats,
)
from planner.mission_optimizer import build_optimized_unified_mission


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_dir = os.path.join(root, "result", "unified_mission")
    baseline_json = os.path.join(out_dir, "mission_output.json")
    optimized_json = os.path.join(out_dir, "optimized_mission_output.json")
    compare_json = os.path.join(out_dir, "mission_compare.json")

    print("=" * 70)
    print("Mission Compare: baseline vs topology-aware optimized")
    print("=" * 70)
    print(f"输入: {sample_path}")
    print()

    unified = load_unified_input_from_json(sample_path)

    print("[1] Baseline mission (plan_global_topology_optimized_mission)...")
    baseline_result = unified_input_to_mission(
        unified, spacing=50.0, enable_sa=False, eps=150.0
    )
    baseline_stats = analyze_mission(baseline_result["mission"])
    export_unified_mission(
        baseline_result,
        unified,
        output_path=baseline_json,
        input_file=sample_path,
    )
    print("[OK] baseline exported")
    print()

    print("[2] Optimized mission (topology-aware greedy)...")
    topo_graph, edge_tasks, line_pts = unified_input_to_edge_tasks(
        unified, spacing=50.0
    )
    opt_mission, opt_order, opt_dirs = build_optimized_unified_mission(
        topo_graph, edge_tasks, eps=150.0
    )
    optimized_stats = analyze_mission(opt_mission)

    optimized_result = {
        "topo_graph": topo_graph,
        "edge_tasks": edge_tasks,
        "line_inspection_points_by_line": line_pts,
        "mission": opt_mission,
    }
    export_unified_mission(
        optimized_result,
        unified,
        output_path=optimized_json,
        input_file=sample_path,
    )
    with open(optimized_json, "r", encoding="utf-8") as f:
        opt_data = json.load(f)
    opt_data["metadata"]["planner_name"] = "UnifiedInputBridge+TopologyAwareGreedy"
    opt_data["metadata"]["optimizer"] = "topology_aware_greedy_v1"
    opt_data["metadata"]["optimized_edge_order"] = opt_order
    with open(optimized_json, "w", encoding="utf-8") as f:
        json.dump(opt_data, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print("[OK] optimized exported")
    print()

    improvement = compare_mission_stats(baseline_stats, optimized_stats)
    payload = {
        "input_file": sample_path,
        "baseline": baseline_stats,
        "optimized": optimized_stats,
        "improvement": improvement,
    }

    os.makedirs(out_dir, exist_ok=True)
    with open(compare_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print("-" * 70)
    print("Baseline:")
    print(f"  total={baseline_stats['total_length']}  connect={baseline_stats['connect_length']}")
    print(f"  connect_ratio={baseline_stats['connect_ratio']:.2%}")
    print(f"  visit_order={baseline_stats['visit_order']}")
    print()
    print("Optimized:")
    print(f"  total={optimized_stats['total_length']}  connect={optimized_stats['connect_length']}")
    print(f"  connect_ratio={optimized_stats['connect_ratio']:.2%}")
    print(f"  visit_order={optimized_stats['visit_order']}")
    print()
    print("Improvement:")
    cr = improvement["connect_ratio"]
    print(f"  connect_ratio: {cr['baseline']:.4f} -> {cr['optimized']:.4f} "
          f"({cr['delta_percent']:+.1f}%) improved={cr['improved']}")
    tl = improvement["total_length"]
    print(f"  total_length: {tl['baseline']:.1f} -> {tl['optimized']:.1f} "
          f"({tl['delta_percent']:+.1f}%) improved={tl['improved']}")
    print(f"  visit_order_changed: {improvement['visit_order_changed']}")
    print()
    print(f"[OK] compare JSON: {compare_json}")
    print("=" * 70)


if __name__ == "__main__":
    main()
