"""
统一输入适配器测试

用法:
    python demo/demo_unified_adapter.py

仅：加载 JSON → 转为 AdaptedLine → 打印摘要，不进入路径规划。
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_adapter import (
    adapted_line_to_polyline,
    print_adapted_lines_summary,
    unified_lines_to_adapted_lines,
)


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = os.path.join(root, "data", "sample_unified_input.json")

    print("=" * 60)
    print("统一输入适配器测试 (demo_unified_adapter)")
    print("=" * 60)
    print(f"文件: {sample_path}")
    print()

    unified = load_unified_input_from_json(sample_path)
    print("[OK] unified input loaded")
    print()

    adapted_lines = unified_lines_to_adapted_lines(unified)
    print("[OK] converted to adapted lines")
    print()

    print_adapted_lines_summary(adapted_lines)

    print("Polyline preview (first line, adapted_line_to_polyline):")
    if adapted_lines:
        poly = adapted_line_to_polyline(adapted_lines[0])
        print(f"  shape={poly.shape}, dtype={poly.dtype}")
        print(f"  first point={poly[0]}, last point={poly[-1]}")
    print()

    print("=" * 60)
    print("完成（未调用路径规划，未写入 result/latest）")
    print("=" * 60)


if __name__ == "__main__":
    main()
