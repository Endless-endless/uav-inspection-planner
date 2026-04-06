"""
=====================================================
分组感知连续航迹规划测试程序
=====================================================

功能：
- 测试分组感知连续航迹规划器
- 对比旧版本和新版本
- 验证 group-aware 策略效果
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np


def run_grouped_mission_test():
    """
    运行分组感知连续航迹规划测试
    """
    print("="*70)
    print("分组感知连续航迹规划测试程序")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 线路识别与巡检点生成
    # =====================================================
    print("Phase 1: 线路识别与巡检点生成")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/test.png',
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

    topo_graph = planner.step7_5_build_topo()

    print()

    # =====================================================
    # Phase 3: EdgeTask 建模
    # =====================================================
    print()
    print("Phase 3: EdgeTask 建模")
    print("-"*70)

    edge_tasks = planner.step8_5_build_edge_tasks()

    print()
    print(f"[完成] 边任务: {len(edge_tasks)} 个")
    print()

    # =====================================================
    # Phase 4: 旧版本连续航迹规划（用于对比）
    # =====================================================
    print()
    print("Phase 4: 旧版本连续航迹规划（对比基准）")
    print("-"*70)

    old_mission = planner.step9_1_plan_continuous_mission()

    if old_mission is None:
        print("[FAIL] 旧版本规划失败")
        return planner

    old_connect_length = old_mission.connect_length
    old_total_length = old_mission.total_length
    old_inspect_length = old_mission.inspect_length

    print()
    print("[旧版本统计]")
    print(f"  总长度: {old_total_length:.1f}px")
    print(f"  巡检长度: {old_inspect_length:.1f}px ({100*old_inspect_length/old_total_length:.1f}%)")
    print(f"  连接长度: {old_connect_length:.1f}px ({100*old_connect_length/old_total_length:.1f}%)")
    print()

    # =====================================================
    # Phase 5: 分组感知连续航迹规划
    # =====================================================
    print()
    print("Phase 5: 分组感知连续航迹规划")
    print("-"*70)

    grouped_mission = planner.step9_2_plan_grouped_continuous_mission(
        eps=150.0,
        old_connect_length=old_connect_length
    )

    print()

    # =====================================================
    # Phase 6: 结果验证与对比
    # =====================================================
    print()
    print("Phase 6: 结果验证与对比")
    print("-"*70)

    if grouped_mission is None:
        print("[FAIL] 新版本规划失败")
        return planner

    # 验证1: 是否覆盖所有边
    print("[验证1] 边覆盖情况")
    total_edges = len(edge_tasks)
    visited_edges = len(grouped_mission.visit_order)
    coverage = visited_edges / total_edges * 100 if total_edges > 0 else 0

    print(f"  总边数: {total_edges}")
    print(f"  已访问: {visited_edges}")
    print(f"  覆盖率: {coverage:.1f}%")

    if coverage == 100:
        print("  [OK] 所有边都被访问")
    else:
        print(f"  [WARN] 有 {total_edges - visited_edges} 条边未访问")
    print()

    # 验证2: 分组效果
    print("[验证2] 分组效果")
    print(f"  分组数: {len(grouped_mission.groups)}")
    print(f"  平均每组边数: {total_edges / len(grouped_mission.groups):.1f}")

    # 统计每组内的边数
    group_sizes = [len(g.edge_ids) for g in grouped_mission.groups.values()]
    print(f"  最小组: {min(group_sizes)} 条边")
    print(f"  最大组: {max(group_sizes)} 条边")
    print()

    # 验证3: 长度对比
    print("[验证3] 长度对比")
    new_total_length = grouped_mission.total_length
    new_inspect_length = grouped_mission.inspect_length
    new_connect_length = grouped_mission.connect_length
    new_intra_connect = grouped_mission.intra_group_connect_length
    new_inter_connect = grouped_mission.inter_group_connect_length

    print(f"  总长度:")
    print(f"    旧版: {old_total_length:.1f}px")
    print(f"    新版: {new_total_length:.1f}px")
    print(f"    变化: {new_total_length - old_total_length:+.1f}px "
          f"({100*(new_total_length-old_total_length)/old_total_length:+.1f}%)")
    print()

    print(f"  巡检长度:")
    print(f"    旧版: {old_inspect_length:.1f}px")
    print(f"    新版: {new_inspect_length:.1f}px")
    print(f"    变化: {new_inspect_length - old_inspect_length:+.1f}px")
    print()

    print(f"  连接长度:")
    print(f"    旧版: {old_connect_length:.1f}px ({100*old_connect_length/old_total_length:.1f}%)")
    print(f"    新版: {new_connect_length:.1f}px ({100*new_connect_length/new_total_length:.1f}%)")
    print(f"    变化: {new_connect_length - old_connect_length:+.1f}px "
          f"({100*(new_connect_length-old_connect_length)/old_connect_length:+.1f}%)")
    print()

    print(f"  组内连接: {new_intra_connect:.1f}px ({100*new_intra_connect/new_total_length:.1f}%)")
    print(f"  组间连接: {new_inter_connect:.1f}px ({100*new_inter_connect/new_total_length:.1f}%)")
    print()

    # 验证4: 路径连续性
    print("[验证4] 路径连续性")
    print(f"  总段数: {len(grouped_mission.segments)}")
    print(f"  路径点数: {len(grouped_mission.full_path)}")

    # 检查是否有断点
    gaps = 0
    for i in range(len(grouped_mission.segments) - 1):
        seg_end = grouped_mission.segments[i].geometry[-1]
        next_start = grouped_mission.segments[i+1].geometry[0]
        if np.linalg.norm(np.array(seg_end) - np.array(next_start)) > 1.0:
            gaps += 1

    if gaps == 0:
        print("  [OK] 路径连续，无断点")
    else:
        print(f"  [WARN] 发现 {gaps} 个潜在断点")
    print()

    # 验证5: 巡检效率
    print("[验证5] 巡检效率")
    old_efficiency = old_inspect_length / old_total_length * 100
    new_efficiency = new_inspect_length / new_total_length * 100

    print(f"  旧版巡检占比: {old_efficiency:.1f}%")
    print(f"  新版巡检占比: {new_efficiency:.1f}%")
    print(f"  提升: {new_efficiency - old_efficiency:+.1f}%")

    if new_efficiency > old_efficiency:
        print("  [OK] 巡检效率提升")
    elif new_efficiency < old_efficiency:
        print("  [WARN] 巡检效率下降")
    else:
        print("  [INFO] 巡检效率持平")
    print()

    # =====================================================
    # Phase 7: 访问顺序
    # =====================================================
    print()
    print("Phase 7: 访问顺序（带方向和分组）")
    print("-"*70)

    print("Group 访问顺序:")
    for i, group_id in enumerate(grouped_mission.group_visit_order):
        group = grouped_mission.groups[group_id]
        print(f"  {i+1}. {group_id}: {len(group.edge_ids)} 条边, "
              f"{group.total_inspect_length:.1f}px")
    print()

    print("Edge 访问顺序:")
    current_group = None
    for i, edge_with_dir in enumerate(grouped_mission.visit_order):
        # 解析方向标记
        if edge_with_dir.endswith('+'):
            edge_id = edge_with_dir[:-1]
            direction = "forward (→)"
        elif edge_with_dir.endswith('-'):
            edge_id = edge_with_dir[:-1]
            direction = "reverse (←)"
        else:
            edge_id = edge_with_dir
            direction = "unknown"

        # 找到所属 group
        edge_group = None
        for gid, group in grouped_mission.groups.items():
            if edge_id in group.edge_ids:
                edge_group = gid
                break

        # 查找对应的 segment 获取长度
        length = 0
        for seg in grouped_mission.segments:
            if seg.edge_id == edge_id and seg.type == 'inspect':
                length = seg.length
                break

        # 检查是否 group 切换
        group_marker = ""
        if edge_group != current_group:
            current_group = edge_group
            group_marker = f" [-> {edge_group}]"

        print(f"  {i+1}. {edge_id} {direction}, len={length:.1f}px{group_marker}")
    print()

    print("="*70)
    print("分组感知连续航迹规划测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边任务图)")
    print("  - result/step8_5_edge_task_summary.png (统计图)")
    print("  - result/step9_1_topo_plan_continuous.png (旧版连续航迹图)")
    print("  - result/step9_2_edge_groups.png (边分组图)")
    print("  - result/step9_2_topo_plan_grouped.png (新版分组连续航迹图)")
    print()
    print("[核心输出]")
    print(f"  - 分组数: {len(grouped_mission.groups)}")
    print(f"  - mission.segments: {len(grouped_mission.segments)} 个段")
    print(f"  - mission.full_path: {len(grouped_mission.full_path)} 个路径点")
    print(f"  - 总长度: {new_total_length:.1f}px")
    print(f"  - 巡检占比: {new_efficiency:.1f}%")
    print(f"  - 连接减少: {old_connect_length - new_connect_length:.1f}px "
          f"({100*(old_connect_length-new_connect_length)/old_connect_length:.1f}%)")
    print()

    return planner


if __name__ == "__main__":
    planner = run_grouped_mission_test()
