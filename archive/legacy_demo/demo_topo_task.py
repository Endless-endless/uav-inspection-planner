"""
=====================================================
EdgeTask 建模测试程序
=====================================================

功能：
- 测试拓扑层和边任务层
- 验证巡检点映射的正确性
- 输出统计信息和可视化
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np


def run_topo_task_test():
    """
    运行拓扑层和边任务层测试
    """
    print("="*70)
    print("EdgeTask 建模测试程序")
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
    # Phase 2: 拓扑建模（保守版本）
    # =====================================================
    print()
    print("Phase 2: 拓扑建模（保守版本 v1.1 + 节点聚类）")
    print("-"*70)

    # Step 7.5: 构建拓扑图
    topo_graph = planner.step7_5_build_topo()

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
    # Phase 4: 验证与统计
    # =====================================================
    print()
    print("Phase 4: 验证与统计")
    print("-"*70)

    stats = planner.topo_stats

    # 验证1: 所有巡检点是否被覆盖
    print("[验证1] 巡检点覆盖情况")
    total_points_in_tasks = sum(len(task.inspection_points) for task in edge_tasks)
    total_original_points = len(planner.line_inspection_points)
    coverage_ratio = total_points_in_tasks / total_original_points * 100 if total_original_points > 0 else 0

    print(f"  原始巡检点: {total_original_points}")
    print(f"  已映射点: {total_points_in_tasks}")
    print(f"  覆盖率: {coverage_ratio:.1f}%")

    if coverage_ratio >= 95:
        print("  [OK] 巡检点覆盖充分")
    else:
        print(f"  [WARN] 巡检点覆盖不足 ({coverage_ratio:.1f}%)")
    print()

    # 验证2: 每条边是否有点
    print("[验证2] 边任务点数分布")
    edges_without_points = sum(1 for task in edge_tasks if task.num_points == 0)
    print(f"  无点的边: {edges_without_points} / {len(edge_tasks)}")

    if edges_without_points == 0:
        print("  [OK] 所有边都有巡检点")
    else:
        print(f"  [WARN] {edges_without_points} 条边没有巡检点")
    print()

    # 验证3: 点数分布合理性
    print("[验证3] 点数分布")
    min_points = min(task.num_points for task in edge_tasks)
    max_points = max(task.num_points for task in edge_tasks)
    avg_points = stats['avg_points_per_edge']

    print(f"  最少点/边: {min_points}")
    print(f"  最多点/边: {max_points}")
    print(f"  平均点/边: {avg_points:.1f}")

    if avg_points > 0 and avg_points < 50:
        print("  [OK] 点数分布合理")
    else:
        print(f"  [WARN] 点数分布异常 (平均={avg_points:.1f})")
    print()

    # =====================================================
    # Phase 5: 边任务详情
    # =====================================================
    print()
    print("Phase 5: 边任务详情")
    print("-"*70)

    print("[每条边任务详情]")
    for task in edge_tasks:
        print(f"  {task.edge_id}:")
        print(f"    - 线路: {task.line_id}")
        print(f"    - 节点: {task.u} -> {task.v}")
        print(f"    - 长度: {task.len2d:.1f}px")
        print(f"    - 类型: {'直线' if task.is_straight else '曲线'}")
        print(f"    - 巡检点: {task.num_points} 个")
        if task.split_reason:
            print(f"    - 切分原因: {task.split_reason}")
    print()

    print("="*70)
    print("EdgeTask 建模测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边任务图)")
    print("  - result/step8_5_edge_task_summary.png (统计图)")
    print()

    return planner


if __name__ == "__main__":
    planner = run_topo_task_test()
