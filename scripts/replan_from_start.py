"""
起点驱动任务重规划脚本

使用方法：
    python scripts/replan_from_start.py --x 100 --y 200

功能：
1. 从指定起点重新规划任务
2. 与基线比较
3. 如果更优，更新 mission_output.json 和页面
4. 如果劣于基线，保留基线不替换
"""

import argparse
import sys
import json
import shutil
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.start_driven_planner_v2 import plan_from_start_point_v2


def load_baseline_data(baseline_path: Path) -> dict:
    """加载基线数据"""
    if baseline_path.exists():
        with open(baseline_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    return None


def compare_with_baseline(new_mission_data: dict, baseline_data: dict) -> dict:
    """
    与基线比较

    比较优先级：
    1. connect_length（一级，最重要）
    2. connect 段数（二级）
    3. total_length（三级）

    Returns:
        比较结果字典
    """
    if baseline_data is None:
        return {
            "has_baseline": False,
            "is_better": True,
            "reason": "无基线数据，采用新方案"
        }

    baseline_stats = baseline_data.get('statistics', {})
    new_stats = new_mission_data.get('statistics', {})

    baseline_connect = baseline_stats.get('connect_length', 0)
    new_connect = new_stats.get('connect_length', 0)

    baseline_total = baseline_stats.get('total_length', 0)
    new_total = new_stats.get('total_length', 0)

    baseline_segments = baseline_stats.get('num_segments', 0)
    new_segments = new_stats.get('num_segments', 0)

    baseline_connect_count = sum(1 for s in baseline_data.get('segments', [])
                                 if s.get('type') == 'connect')
    new_connect_count = sum(1 for s in new_mission_data.get('segments', [])
                            if s.get('type') == 'connect')

    # 一级：比较 connect_length
    # 如果new明显更优（即使幅度小），直接采纳
    # 只有当new明显更差时才拒绝
    # 相近情况下再比较二级指标
    connect_diff = new_connect - baseline_connect
    connect_diff_ratio = connect_diff / baseline_connect if baseline_connect > 0 else 0

    if connect_diff < 0:  # new的connect长度更短（更好）
        # 只要更短就采纳，不设容差限制
        return {
            "has_baseline": True,
            "is_better": True,
            "reason": f"connect长度更短 ({abs(connect_diff_ratio)*100:.1f}%)",
            "baseline": {
                "connect_length": baseline_connect,
                "total_length": baseline_total,
                "connect_count": baseline_connect_count,
                "segment_count": baseline_segments
            },
            "new": {
                "connect_length": new_connect,
                "total_length": new_total,
                "connect_count": new_connect_count,
                "segment_count": new_segments
            }
        }
    elif connect_diff_ratio > 0.05:  # connect长度明显增加（>5%）
        return {
            "has_baseline": True,
            "is_better": False,
            "reason": f"connect长度明显增加 ({connect_diff_ratio*100:.1f}%)，拒绝采用",
            "baseline": {
                "connect_length": baseline_connect,
                "total_length": baseline_total,
                "connect_count": baseline_connect_count,
                "segment_count": baseline_segments
            },
            "new": {
                "connect_length": new_connect,
                "total_length": new_total,
                "connect_count": new_connect_count,
                "segment_count": new_segments
            }
        }
    else:  # 差异在5%以内（new略差或相近）
        # 二级：比较 connect 段数
        if new_connect_count < baseline_connect_count:
            return {
                "has_baseline": True,
                "is_better": True,
                "reason": "connect长度相近，但段数更少",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }
        elif new_connect_count > baseline_connect_count:
            return {
                "has_baseline": True,
                "is_better": False,
                "reason": "connect长度相近，但段数更多",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }

        # 三级：比较 total_length
        total_diff = new_total - baseline_total
        if total_diff < 0:
            return {
                "has_baseline": True,
                "is_better": True,
                "reason": "connect和段数相近，但总长度更短",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }
        else:
            return {
                "has_baseline": True,
                "is_better": False,
                "reason": "connect和段数相近，但总长度更长",
                "baseline": {
                    "connect_length": baseline_connect,
                    "total_length": baseline_total,
                    "connect_count": baseline_connect_count,
                    "segment_count": baseline_segments
                },
                "new": {
                    "connect_length": new_connect,
                    "total_length": new_total,
                    "connect_count": new_connect_count,
                    "segment_count": new_segments
                }
            }


def main():
    parser = argparse.ArgumentParser(description='起点驱动的任务重规划')
    parser.add_argument('--x', type=float, required=True, help='起点X坐标')
    parser.add_argument('--y', type=float, required=True, help='起点Y坐标')
    parser.add_argument('--force', action='store_true', help='强制采用新方案（忽略基线比较）')

    args = parser.parse_args()

    print("="*70)
    print("起点驱动任务重规划")
    print("="*70)
    print(f"输入起点: ({args.x}, {args.y})")

    # 基线路径
    baseline_path = Path("result/latest/mission_output.json")

    # 加载基线数据
    baseline_data = load_baseline_data(baseline_path)
    if baseline_data:
        baseline_stats = baseline_data.get('statistics', {})
        print(f"\n当前基线:")
        print(f"  Total: {baseline_stats.get('total_length', 0):.1f}px")
        print(f"  Inspect: {baseline_stats.get('inspect_length', 0):.1f}px")
        print(f"  Connect: {baseline_stats.get('connect_length', 0):.1f}px")
    else:
        print("\n警告：未找到基线数据")

    # 执行重规划
    print(f"\n执行重规划...")
    new_mission_data = plan_from_start_point_v2(
        start_x=args.x,
        start_y=args.y,
        baseline_data=baseline_data
    )

    if new_mission_data is None:
        print("\n[失败] 重规划失败")
        return 1

    new_stats = new_mission_data.get('statistics', {})
    print(f"\n新方案:")
    print(f"  Total: {new_stats.get('total_length', 0):.1f}px")
    print(f"  Inspect: {new_stats.get('inspect_length', 0):.1f}px")
    print(f"  Connect: {new_stats.get('connect_length', 0):.1f}px")

    # 与基线比较
    if not args.force:
        print(f"\n比较与基线...")
        comparison = compare_with_baseline(new_mission_data, baseline_data)
        is_adopted = comparison["is_better"]

        if comparison["has_baseline"]:
            print(f"\n基线:")
            print(f"  Connect: {comparison['baseline']['connect_length']:.1f}px")
            print(f"  Total: {comparison['baseline']['total_length']:.1f}px")
            print(f"\n新方案:")
            print(f"  Connect: {comparison['new']['connect_length']:.1f}px")
            print(f"  Total: {comparison['new']['total_length']:.1f}px")

        if is_adopted:
            print(f"\n[OK] 采纳: {comparison['reason']}")
        else:
            print(f"\n[NG] 拒绝: {comparison['reason']}")
            print("新方案未优于基线，不更新主输出。")
            print("如需强制采用，请使用 --force 参数。")
            return 0
    else:
        print(f"\n强制模式：跳过基线比较")
        is_adopted = True

    # 采纳新方案
    if is_adopted:
        print(f"\n更新主输出...")

        # 备份旧数据
        backup_path = Path("result/latest/mission_output_before_replan.json")
        if baseline_path.exists():
            shutil.copy(baseline_path, backup_path)
            print(f"备份: {backup_path}")

        # 保存新数据
        with open(baseline_path, 'w', encoding='utf-8') as f:
            json.dump(new_mission_data, f, indent=2, ensure_ascii=False)
        print(f"更新: {baseline_path}")

        # 重新生成HTML
        from demo.generate_interactive_main_view import create_interactive_main_view
        create_interactive_main_view(
            map_image_path="data/test.png",
            mission_json_path=str(baseline_path),
            output_html_path="result/latest/main_view_interactive.html"
        )
        print(f"更新: result/latest/main_view_interactive.html")

    print("\n" + "="*70)
    print("完成！")
    print("="*70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
