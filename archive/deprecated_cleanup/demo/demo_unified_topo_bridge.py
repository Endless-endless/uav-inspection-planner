"""
UnifiedInput → IndependentLine → TopoGraph 桥接测试（第三阶段）

用法:
    python demo/demo_unified_topo_bridge.py

优先加载 data/sample_unified_topo_input.json（含交汇拓扑）；
不写入 result/latest，不生成 mission JSON。
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_adapter import (
    build_topo_graph_from_independent_lines,
    print_adapted_lines_summary,
    print_independent_lines_summary,
    print_topo_graph_summary,
    unified_input_to_independent_lines,
    unified_lines_to_adapted_lines,
)


def _resolve_sample_path(root: str) -> str:
    topo_path = os.path.join(root, "data", "sample_unified_topo_input.json")
    basic_path = os.path.join(root, "data", "sample_unified_input.json")
    if os.path.isfile(topo_path):
        return topo_path
    return basic_path


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = _resolve_sample_path(root)

    print("=" * 60)
    print("UnifiedInput → TopoGraph 桥接测试")
    print("=" * 60)
    print(f"输入文件: {sample_path}")
    print()

    unified = load_unified_input_from_json(sample_path)
    print("[OK] unified input loaded")
    print()

    adapted_lines = unified_lines_to_adapted_lines(unified)
    print("[OK] converted to adapted lines")
    print_adapted_lines_summary(adapted_lines)
    print()

    independent_lines = unified_input_to_independent_lines(unified)
    print("[OK] converted to IndependentLine")
    print_independent_lines_summary(independent_lines)
    print()

    print("[Topo] 调用现有 detect_topo_nodes / split_lines_to_edges / build_topo_graph ...")
    topo_graph, topo_nodes, topo_edges = build_topo_graph_from_independent_lines(
        independent_lines
    )
    print("[OK] TopoGraph built")
    print()

    print_topo_graph_summary(topo_graph)
    print(f"Summary: nodes={len(topo_graph.nodes)}, edges={len(topo_graph.edges)}")
    print()

    print("=" * 60)
    print("完成（未生成 mission_output.json，未写入 result/latest）")
    print("=" * 60)


if __name__ == "__main__":
    main()
