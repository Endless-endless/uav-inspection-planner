"""
统一输入接口最小测试

用法:
    python demo/demo_unified_input.py

仅加载并校验 sample JSON，打印摘要，不进入路径规划。
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from input.unified_input import load_unified_input_from_json, validate_unified_input


def main() -> None:
    sample_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "data",
        "sample_unified_input.json",
    )

    print("=" * 60)
    print("统一输入接口测试 (demo_unified_input)")
    print("=" * 60)
    print(f"文件: {sample_path}")
    print()

    data = load_unified_input_from_json(sample_path)
    errors = validate_unified_input(data)

    if errors:
        print("[FAIL] 校验未通过:")
        for err in errors:
            print(f"  - {err}")
        sys.exit(1)

    print("[OK] 校验通过")
    print()

    print(f"Lines ({len(data.lines)}):")
    for line in data.lines:
        print(f"  - {line.id}: {len(line.points)} points, voltage={line.voltage_level}")
        if line.points:
            print(f"      start={line.points[0]}, end={line.points[-1]}")
    print()

    print(f"Towers ({len(data.towers)}):")
    for tower in data.towers:
        print(f"  - {tower.id}: position={tower.position}")
    print()

    print("Weather:")
    print(f"  wind_speed={data.weather.wind_speed} m/s")
    print(f"  wind_direction={data.weather.wind_direction} deg")
    print(f"  visibility={data.weather.visibility}")
    print()

    print("UAV:")
    print(f"  max_speed={data.uav.max_speed} m/s")
    print(f"  battery_capacity={data.uav.battery_capacity}")
    print(f"  safe_distance={data.uav.safe_distance} m")
    print()

    if data.metadata:
        print("Root metadata:", data.metadata)

    print()
    print("=" * 60)
    print("完成（未调用路径规划）")
    print("=" * 60)


if __name__ == "__main__":
    main()
