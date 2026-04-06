"""
=====================================================
保守拓扑层测试程序
=====================================================

功能：
- 测试保守版本的拓扑层设计
- 只执行拓扑图构建，不执行后续步骤
- 验证节点/边数量是否合理

测试目标：
1. 节点数量是否合理（目标：30-60个）
2. 边数量是否合理（目标：24-120条）
3. 拓扑图是否清晰可解释
4. 是否比上一版 495 节点 / 488 边明显收敛
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3


def run_conservative_topo_test():
    """
    运行保守拓扑层测试
    """
    print("="*70)
    print("保守拓扑层测试程序（v1.1）")
    print("="*70)
    print()
    print("设计参数:")
    print("  - EDGE_MAX_LEN = 600px")
    print("  - EDGE_MIN_LEN = 150px")
    print("  - ANGLE_THRESH = 60°")
    print("  - ANGLE_WINDOW = 10px")
    print("  - 切分优先级: 端点 > 角度切分 > 长度切分")
    print("  - 取消曲率切分")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 线路识别（复用现有系统）
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
    import numpy as np
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
    print("Phase 2: 拓扑建模（保守版本 v1.1）")
    print("-"*70)

    # Step 7.5: 构建拓扑图
    topo_graph = planner.step7_5_build_topo()

    print()

    # =====================================================
    # Phase 3: 结果验证
    # =====================================================
    print()
    print("Phase 3: 结果验证")
    print("-"*70)

    stats = planner.topo_stats

    # 验证节点数量
    node_count = stats['total_nodes']
    edge_count = stats['total_edges']

    print("[验证目标]")
    print(f"  节点数量目标: 30-60 个")
    print(f"  边数量目标: 24-120 条")
    print()

    print("[实际结果]")
    print(f"  节点数量: {node_count} 个")
    print(f"  边数量: {edge_count} 条")
    print()

    # 验证是否达标
    node_ok = 30 <= node_count <= 60
    edge_ok = 24 <= edge_count <= 120

    print("[验证结论]")
    if node_ok and edge_ok:
        print("  [OK] 节点数量在目标范围内")
        print("  [OK] 边数量在目标范围内")
        print("  [OK] 保守设计成功！")
    elif node_ok:
        print("  [OK] 节点数量在目标范围内")
        if edge_count > 120:
            print(f"  [FAIL] 边数量过多 ({edge_count} > 120)")
        else:
            print(f"  [FAIL] 边数量过少 ({edge_count} < 24)")
    elif edge_ok:
        print("  [OK] 边数量在目标范围内")
        if node_count > 60:
            print(f"  [FAIL] 节点数量过多 ({node_count} > 60)")
        else:
            print(f"  [FAIL] 节点数量过少 ({node_count} < 30)")
    else:
        print("  [FAIL] 节点和边数量均不在目标范围内")
        print("  [FAIL] 需要进一步调整参数")
    print()

    # 与上一版对比
    print()
    print("[版本对比]")
    print("  上一版（激进版本）:")
    print("    - 节点: 495 个")
    print("    - 边: 488 条")
    print()
    print("  当前版（保守版本）:")
    print(f"    - 节点: {node_count} 个")
    print(f"    - 边: {edge_count} 条")
    print()

    # 计算收敛率
    node_reduction = (495 - node_count) / 495 * 100
    edge_reduction = (488 - edge_count) / 488 * 100

    print("[收敛率]")
    print(f"  节点减少: {node_reduction:.1f}%")
    print(f"  边减少: {edge_reduction:.1f}%")
    print()

    if node_reduction > 80 and edge_reduction > 70:
        print("  [OK] 相比上一版明显收敛！")
    else:
        print("  [WARN] 收敛效果不理想")
    print()

    # =====================================================
    # Phase 4: 度数分布验证
    # =====================================================
    print()
    print("Phase 4: 度数分布验证")
    print("-"*70)

    degree_dist = stats['degree_dist']
    print("[节点度数分布]")
    for deg in sorted(degree_dist.keys()):
        count = degree_dist[deg]
        ratio = count / node_count * 100
        print(f"  度数={deg}: {count} 个 ({ratio:.1f}%)")
    print()

    # 验证度数分布是否合理
    deg1_count = degree_dist.get(1, 0)
    deg1_ratio = deg1_count / node_count * 100 if node_count > 0 else 0

    if deg1_ratio > 80:
        print("  [OK] 度数=1的节点占比 > 80%（大部分是端点，符合预期）")
    else:
        print(f"  [FAIL] 度数=1的节点占比 < 80%（{deg1_ratio:.1f}%）")
    print()

    # =====================================================
    # Phase 5: 可解释性验证
    # =====================================================
    print()
    print("Phase 5: 可解释性验证")
    print("-"*70)

    print("[拓扑图结构]")
    print("  能否用自然语言描述连接关系？")

    # 统计有多少条线路被细分
    split_lines = sum(1 for num_edges in stats['edges_by_line'].values() if num_edges > 1)
    total_lines = stats['num_lines']

    print(f"  - 总线路: {total_lines} 条")
    print(f"  - 被细分: {split_lines} 条")
    print(f"  - 未细分: {total_lines - split_lines} 条")
    print()

    if split_lines <= total_lines / 2:
        print("  [OK] 大部分线路保持完整，图结构清晰")
    else:
        print("  [WARN] 过多线路被细分，可能过于复杂")
    print()

    print("="*70)
    print("保守拓扑层测试完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print()

    return planner


if __name__ == "__main__":
    planner = run_conservative_topo_test()
