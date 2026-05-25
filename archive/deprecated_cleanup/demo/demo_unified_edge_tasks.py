"""
UnifiedInput → EdgeTask 测试（第四阶段）

用法:
    python demo/demo_unified_edge_tasks.py

输出: result/unified_edge_tasks/edge_tasks_debug.json
不写 result/latest，不生成 mission_output.json。
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_task_adapter import (
    build_edge_tasks_debug_payload,
    generate_inspection_points_for_adapted_lines,
    print_edge_tasks_summary,
    save_edge_tasks_debug_json,
    unified_input_to_edge_tasks,
)
from input.unified_adapter import unified_lines_to_adapted_lines


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_dir = os.path.join(root, "result", "unified_edge_tasks")
    debug_path = os.path.join(out_dir, "edge_tasks_debug.json")

    print("=" * 60)
    print("UnifiedInput → EdgeTask 测试")
    print("=" * 60)
    print(f"输入: {sample_path}")
    print(f"输出: {debug_path}")
    print()

    unified = load_unified_input_from_json(sample_path)
    print("[OK] unified input loaded")

    adapted_lines = unified_lines_to_adapted_lines(unified)
    line_pts = generate_inspection_points_for_adapted_lines(adapted_lines, spacing=50.0)
    print("[OK] inspection points generated")
    for lid, pts in line_pts.items():
        print(f"  {lid}: {len(pts)} points")
    print()

    topo_graph, edge_tasks, line_inspection_points_by_line = unified_input_to_edge_tasks(
        unified, spacing=50.0
    )
    print("[OK] EdgeTask built via build_edge_tasks")
    print()

    print_edge_tasks_summary(
        topo_graph, edge_tasks, line_inspection_points_by_line
    )

    payload = build_edge_tasks_debug_payload(
        topo_graph,
        edge_tasks,
        line_inspection_points_by_line,
        source_path=sample_path,
    )
    saved = save_edge_tasks_debug_json(payload, debug_path)
    print(f"[OK] debug JSON saved: {saved}")
    print()

    print("=" * 60)
    print("完成（未写入 result/latest）")
    print("=" * 60)


if __name__ == "__main__":
    main()
