"""
=====================================================
拓扑任务规划测试程序（贪心版本）
=====================================================

功能：
- 测试最小可运行的拓扑规划器
- 验证边任务访问顺序
- 输出调试信息和可视化
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np


def run_topo_plan_greedy_test():
    """
    运行拓扑任务规划测试（贪心版本）
    """
    print("="*70)
    print("拓扑任务规划测试程序（贪心版本 v9.0）")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 线路识别与巡检点生成（复用现有系统）
    # =====================================================
    print("Phase 1: 线路识别与巡检点生成")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/1.png',
        flight_height=30
    )

    # 执行step1-5
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()
    planner.step5_generate_line_inspection_points()

    # 设置地形
    terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
    planner.step6_map_line_points_to_3d(terrain)
    planner.step6_smooth_terrain(terrain)

    print()
    print(f"[完成] 独立线路: {len(planner.independent_lines)} 条")
    print(f"[完成] 巡检点: {len(planner.line_inspection_points)} 个")
    print()

    # =====================================================
    # Phase 2: 拓扑建模
    # =====================================================
    print()
    print("Phase 2: 拓扑建模")
    print("-"*70)

    # Step 7.5: 构建拓扑图
    topo_graph = planner.step7_5_build_topo()

    print()

    # =====================================================
    # Phase 3: EdgeTask 建模
    # =====================================================
    print()
    print("Phase 3: EdgeTask 建模")
    print("-"*70)

    # Step 8.5: 构建边任务
    edge_tasks = planner.step8_5_build_edge_tasks()

    print()
    print(f"[完成] 边任务: {len(edge_tasks)} 个")
    print()

    # =====================================================
    # Phase 4: 拓扑任务规划（贪心版本）
    # =====================================================
    print()
    print("Phase 4: 拓扑任务规划（贪心版本）")
    print("-"*70)

    # Step 9.0: 贪心规划
    mission = planner.step9_0_plan_topo_mission_greedy()

    print()

    # =====================================================
    # Phase 5: 结果验证
    # =====================================================
    print()
    print("Phase 5: 结果验证")
    print("-"*70)

    if mission is None:
        print("[FAIL] 规划失败")
        return planner

    # 验证1: 是否覆盖所有边
    print("[验证1] 边覆盖情况")
    total_edges = len(edge_tasks)
    visited_edges = len(mission.get_visit_sequence())
    coverage = visited_edges / total_edges * 100 if total_edges > 0 else 0

    print(f"  总边数: {total_edges}")
    print(f"  已访问: {visited_edges}")
    print(f"  覆盖率: {coverage:.1f}%")

    if coverage == 100:
        print("  [OK] 所有边都被访问")
    else:
        print(f"  [WARN] 有 {total_edges - visited_edges} 条边未访问")
    print()

    # 验证2: 相邻转移统计
    print("[验证2] 转移类型统计")
    n_adjacent = sum(1 for s in mission.steps if s.is_adjacent)
    n_jumps = len(mission.steps) - n_adjacent - 1  # -1 for start

    print(f"  相邻转移: {n_adjacent}")
    print(f"  跳转转移: {n_jumps}")
    print(f"  总转移: {n_adjacent + n_jumps}")

    if n_adjacent > n_jumps:
        print("  [OK] 主要是相邻转移（顺着图走）")
    else:
        print(f"  [WARN] 跳转过多 ({n_jumps} 次)")
    print()

    # 验证3: 总代价
    print("[验证3] 转移代价")
    print(f"  总代价: {mission.total_cost:.1f}px")
    print(f"  平均代价/转移: {mission.total_cost / max(len(mission.steps) - 1, 1):.1f}px")
    print()

    # =====================================================
    # Phase 6: 访问顺序
    # =====================================================
    print()
    print("Phase 6: 访问顺序")
    print("-"*70)

    visit_sequence = mission.get_visit_sequence()
    print("访问顺序:")
    for i, edge_id in enumerate(visit_sequence):
        step = mission.steps[i]
        if step.is_adjacent:
            print(f"  {i+1}. {edge_id} (相邻)")
        elif i == 0:
            print(f"  {i+1}. {edge_id} (起点)")
        else:
            print(f"  {i+1}. {edge_id} (跳转, cost={step.transition_cost:.1f}px)")
    print()

    print("="*70)
    print("拓扑任务规划测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边任务图)")
    print("  - result/step8_5_edge_task_summary.png (统计图)")
    print("  - result/step9_0_topo_plan_greedy.png (规划图)")
    print()

    return planner


if __name__ == "__main__":
    planner = run_topo_plan_greedy_test()
