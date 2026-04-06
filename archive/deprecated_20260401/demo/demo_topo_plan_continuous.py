"""
=====================================================
连续完整航迹规划测试程序
=====================================================

功能：
- 测试连续航迹规划器
- 验证 mission segments 生成
- 输出完整连续路径
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np


def run_continuous_mission_test():
    """
    运行连续航迹规划测试
    """
    print("="*70)
    print("连续完整航迹规划测试程序")
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
    # Phase 4: 连续航迹规划
    # =====================================================
    print()
    print("Phase 4: 连续航迹规划")
    print("-"*70)

    mission = planner.step9_1_plan_continuous_mission()

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
    visited_edges = len(mission.visit_order)
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
    print(f"  总段数: {len(mission.segments)}")
    print(f"  路径点数: {len(mission.full_path)}")

    # 检查是否有断点
    gaps = 0
    for i in range(len(mission.segments) - 1):
        seg_end = mission.segments[i].geometry[-1]
        next_start = mission.segments[i+1].geometry[0]
        if np.linalg.norm(np.array(seg_end) - np.array(next_start)) > 1.0:
            gaps += 1

    if gaps == 0:
        print("  [OK] 路径连续，无断点")
    else:
        print(f"  [WARN] 发现 {gaps} 个潜在断点")
    print()

    # 验证3: 长度统计
    print("[验证3] 长度统计")
    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0

    print(f"  总长度: {mission.total_length:.1f}px")
    print(f"  巡检长度: {mission.inspect_length:.1f}px ({100-connect_ratio:.1f}%)")
    print(f"  连接长度: {mission.connect_length:.1f}px ({connect_ratio:.1f}%)")

    if connect_ratio < 50:
        print("  [OK] 连接段占比较低，主要时间在巡检")
    elif connect_ratio < 70:
        print("  [INFO] 连接段占比中等")
    else:
        print(f"  [WARN] 连接段占比过高 ({connect_ratio:.1f}%)")
    print()

    # =====================================================
    # Phase 6: 访问顺序
    # =====================================================
    print()
    print("Phase 6: 访问顺序（带方向）")
    print("-"*70)

    print("访问顺序:")
    for i, edge_with_dir in enumerate(mission.visit_order):
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

        # 查找对应的 segment 获取长度
        for seg in mission.segments:
            if seg.edge_id == edge_id and seg.type == 'inspect':
                length = seg.length
                break

        print(f"  {i+1}. {edge_id} {direction}, len={length:.1f}px")
    print()

    print("="*70)
    print("连续完整航迹规划测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边任务图)")
    print("  - result/step8_5_edge_task_summary.png (统计图)")
    print("  - result/step9_1_topo_plan_continuous.png (连续航迹图)")
    print()
    print("[核心输出]")
    print(f"  - mission.segments: {len(mission.segments)} 个段")
    print(f"  - mission.full_path: {len(mission.full_path)} 个路径点")
    print(f"  - 总长度: {mission.total_length:.1f}px")
    print()

    return planner


if __name__ == "__main__":
    planner = run_continuous_mission_test()
