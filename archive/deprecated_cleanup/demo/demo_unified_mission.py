"""
UnifiedInput → Mission 完整闭环测试（第五阶段）

用法:
    python demo/demo_unified_mission.py

输出: result/unified_mission/mission_output.json
不写 result/latest。
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json
from input.unified_mission_adapter import (
    DEFAULT_MISSION_JSON,
    export_unified_mission,
    print_mission_summary,
    unified_input_to_mission,
)


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_path = os.path.join(root, DEFAULT_MISSION_JSON.replace("/", os.sep))

    print("=" * 60)
    print("UnifiedInput → Mission 测试")
    print("=" * 60)
    print(f"输入: {sample_path}")
    print(f"输出: {out_path}")
    print()

    unified = load_unified_input_from_json(sample_path)
    print("[OK] unified input loaded")

    result = unified_input_to_mission(unified, spacing=50.0, enable_sa=False, eps=150.0)
    print("[OK] mission planned (GroupedContinuousMission)")
    print()

    print_mission_summary(result)
    print()

    json_path = export_unified_mission(
        result,
        unified,
        output_path=out_path,
        input_file=sample_path,
    )
    print(f"[OK] mission JSON exported: {json_path}")
    print()

    print("=" * 60)
    print("完成（未写入 result/latest）")
    print("=" * 60)


if __name__ == "__main__":
    main()
