"""
=====================================================
优化的分组感知连续航迹规划测试程序
=====================================================

功能：
- 测试优化后的 grouped continuous mission planner
- 对比 step9_2 和 step9_3 的效果
- 验证 entry/exit-aware 策略的效果
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np


def run_optimized_grouped_mission_test():
    """
    运行优化后的分组感知连续航迹规划测试
    """
    print("="*70)
    print("优化的分组感知连续航迹规划测试程序")
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
    # Phase 4: Step9_2 基础分组规划（对比基准）
    # =====================================================
    print()
    print("Phase 4: Step9_2 基础分组规划（对比基准）")
    print("-"*70)

    step9_2_mission = planner.step9_2_plan_grouped_continuous_mission(eps=150.0)

    if step9_2_mission is None:
        print("[FAIL] Step9_2 规划失败")
        return planner

    print()
    print("[Step9_2 统计摘要]")
    print(f"  总长度: {step9_2_mission.total_length:.1f}px")
    print(f"  巡检: {step9_2_mission.inspect_length:.1f}px "
          f"({100*step9_2_mission.inspect_length/step9_2_mission.total_length:.1f}%)")
    print(f"  连接: {step9_2_mission.connect_length:.1f}px "
          f"({100*step9_2_mission.connect_length/step9_2_mission.total_length:.1f}%)")
    print(f"    - 组内: {step9_2_mission.intra_group_connect_length:.1f}px")
    print(f"    - 组间: {step9_2_mission.inter_group_connect_length:.1f}px")
    print()

    # =====================================================
    # Phase 5: Step9_3 优化分组规划
    # =====================================================
    print()
    print("Phase 5: Step9_3 优化分组规划")
    print("-"*70)

    step9_3_mission = planner.step9_3_plan_grouped_mission_optimized(
        eps=150.0,
        old_mission=step9_2_mission
    )

    print()

    # =====================================================
    # Phase 6: 结果验证与对比
    # =====================================================
    print()
    print("Phase 6: 结果验证与对比")
    print("-"*70)

    if step9_3_mission is None:
        print("[FAIL] Step9_3 规划失败")
        return planner

    # 验证1: 边覆盖
    print("[验证1] 边覆盖情况")
    total_edges = len(edge_tasks)
    visited_edges = len(step9_3_mission.visit_order)
    coverage = visited_edges / total_edges * 100 if total_edges > 0 else 0

    print(f"  总边数: {total_edges}")
    print(f"  已访问: {visited_edges}")
    print(f"  覆盖率: {coverage:.1f}%")

    if coverage == 100:
        print("  [OK] 所有边都被访问")
    else:
        print(f"  [WARN] 有 {total_edges - visited_edges} 条边未访问")
    print()

    # 验证2: 路径连续性
    print("[验证2] 路径连续性")
    print(f"  总段数: {len(step9_3_mission.segments)}")
    print(f"  路径点数: {len(step9_3_mission.full_path)}")

    gaps = 0
    for i in range(len(step9_3_mission.segments) - 1):
        seg_end = step9_3_mission.segments[i].geometry[-1]
        next_start = step9_3_mission.segments[i+1].geometry[0]
        if np.linalg.norm(np.array(seg_end) - np.array(next_start)) > 1.0:
            gaps += 1

    if gaps == 0:
        print("  [OK] 路径连续，无断点")
    else:
        print(f"  [WARN] 发现 {gaps} 个潜在断点")
    print()

    # 验证3: Step9_2 vs Step9_3 对比
    print("[验证3] Step9_2 vs Step9_3 对比")

    # 总长度
    total_change = step9_3_mission.total_length - step9_2_mission.total_length
    total_change_pct = 100 * total_change / step9_2_mission.total_length

    print(f"  总长度:")
    print(f"    Step9_2: {step9_2_mission.total_length:.1f}px")
    print(f"    Step9_3: {step9_3_mission.total_length:.1f}px")
    print(f"    变化: {total_change:+.1f}px ({total_change_pct:+.1f}%)")

    if total_change < 0:
        print(f"    [OK] 总长度减少 {-total_change:.1f}px")
    print()

    # 连接长度
    connect_change = step9_3_mission.connect_length - step9_2_mission.connect_length
    connect_change_pct = 100 * connect_change / step9_2_mission.connect_length

    print(f"  连接长度:")
    print(f"    Step9_2: {step9_2_mission.connect_length:.1f}px")
    print(f"    Step9_3: {step9_3_mission.connect_length:.1f}px")
    print(f"    变化: {connect_change:+.1f}px ({connect_change_pct:+.1f}%)")

    if connect_change < 0:
        print(f"    [OK] 连接长度减少 {-connect_change:.1f}px")
    print()

    # 组间连接
    inter_change = (step9_3_mission.inter_group_connect_length -
                    step9_2_mission.inter_group_connect_length)
    inter_change_pct = (100 * inter_change /
                        step9_2_mission.inter_group_connect_length
                        if step9_2_mission.inter_group_connect_length > 0 else 0)

    print(f"  组间连接:")
    print(f"    Step9_2: {step9_2_mission.inter_group_connect_length:.1f}px")
    print(f"    Step9_3: {step9_3_mission.inter_group_connect_length:.1f}px")
    print(f"    变化: {inter_change:+.1f}px ({inter_change_pct:+.1f}%)")

    if inter_change < 0:
        print(f"    [OK] 组间连接减少 {-inter_change:.1f}px")
    print()

    # 巡检效率
    inspect_ratio_9_2 = 100 * step9_2_mission.inspect_length / step9_2_mission.total_length
    inspect_ratio_9_3 = 100 * step9_3_mission.inspect_length / step9_3_mission.total_length
    inspect_change = inspect_ratio_9_3 - inspect_ratio_9_2

    print(f"  巡检占比:")
    print(f"    Step9_2: {inspect_ratio_9_2:.1f}%")
    print(f"    Step9_3: {inspect_ratio_9_3:.1f}%")
    print(f"    提升: {inspect_change:+.1f}%")

    if inspect_change > 0:
        print(f"    [OK] 巡检效率提升 {inspect_change:.1f}%")
    print()

    # 验证4: Group 访问顺序对比
    print("[验证4] Group 访问顺序对比")
    print(f"  Step9_2: {' -> '.join(step9_2_mission.group_visit_order)}")
    print(f"  Step9_3: {' -> '.join(step9_3_mission.group_visit_order)}")

    if step9_3_mission.group_visit_order != step9_2_mission.group_visit_order:
        print(f"  [INFO] Group 访问顺序已优化")
    print()

    # =====================================================
    # Phase 7: 详细访问顺序
    # =====================================================
    print()
    print("Phase 7: 详细访问顺序（Step9_3）")
    print("-"*70)

    print("Group 访问顺序:")
    for i, group_id in enumerate(step9_3_mission.group_visit_order):
        group = step9_3_mission.groups[group_id]
        print(f"  {i+1}. {group_id}: {len(group.edge_ids)} 条边")
    print()

    print("Edge 访问顺序（带 entry/exit）:")
    current_group = None

    for i, edge_with_dir in enumerate(step9_3_mission.visit_order):
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
        for gid, group in step9_3_mission.groups.items():
            if edge_id in group.edge_ids:
                edge_group = gid
                break

        # 查找对应的 segment 获取长度
        length = 0
        for seg in step9_3_mission.segments:
            if seg.edge_id == edge_id and seg.type == 'inspect':
                length = seg.length
                break

        # 检查是否 group 切换
        group_marker = ""
        is_entry = False
        is_exit = False

        if edge_group != current_group:
            current_group = edge_group
            group_marker = f" [-> {edge_group}]"
            is_entry = True

        # 检查是否是 group 的最后一条边
        group_edges = step9_3_mission.groups[edge_group].edge_ids
        if edge_id == group_edges[-1]:
            is_exit = True

        markers = []
        if is_entry:
            markers.append("ENTRY")
        if is_exit:
            markers.append("EXIT")

        marker_str = f" [{', '.join(markers)}]" if markers else ""

        print(f"  {i+1}. {edge_id} {direction}, len={length:.1f}px{group_marker}{marker_str}")
    print()

    print("="*70)
    print("优化的分组感知连续航迹规划测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边任务图)")
    print("  - result/step9_2_edge_groups.png (分组图)")
    print("  - result/step9_2_topo_plan_grouped.png (基础分组航迹图)")
    print("  - result/step9_3_edge_groups.png (分组图)")
    print("  - result/step9_3_topo_plan_grouped_optimized.png (优化分组航迹图)")
    print("  - result/step9_3_group_entry_exit_debug.png (Entry/Exit 调试图)")
    print()
    print("[核心输出]")
    print(f"  - 分组数: {len(step9_3_mission.groups)}")
    print(f"  - mission.segments: {len(step9_3_mission.segments)} 个段")
    print(f"  - mission.full_path: {len(step9_3_mission.full_path)} 个路径点")
    print(f"  - 总长度: {step9_3_mission.total_length:.1f}px")
    print(f"  - 巡检占比: {inspect_ratio_9_3:.1f}%")
    if connect_change < 0:
        print(f"  - 连接减少: {-connect_change:.1f}px ({-connect_change_pct:.1f}%)")
    if inter_change < 0:
        print(f"  - 组间连接减少: {-inter_change:.1f}px ({-inter_change_pct:.1f}%)")
    print()

    return planner


if __name__ == "__main__":
    planner = run_optimized_grouped_mission_test()
