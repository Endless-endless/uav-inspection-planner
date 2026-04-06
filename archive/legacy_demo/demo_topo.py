"""
=====================================================
拓扑层演示程序
=====================================================

功能：
- 完整展示拓扑层的工作流程
- 对比线段级方法和拓扑级方法
- 生成拓扑可视化图

流程：
1. 线路识别（复用现有step1-5）
2. 构建拓扑图
3. 构建边任务
4. 拓扑路径规划
5. 对比结果
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3


def run_topo_demo():
    """
    运行拓扑层演示
    """
    print("="*70)
    print("拓扑层演示程序")
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

    # =====================================================
    # Phase 2: 拓扑建模（新增）
    # =====================================================
    print()
    print("Phase 2: 拓扑建模")
    print("-"*70)

    # Step 7.5: 构建拓扑图
    topo_graph = planner.step7_5_build_topo()

    # Step 8.5: 构建边任务
    edge_tasks = planner.step8_5_build_edge_tasks()

    print()
    print(f"[完成] 拓扑节点: {len(planner.topo_nodes)} 个")
    print(f"[完成] 拓扑边: {len(planner.topo_edges)} 条")
    print(f"[完成] 边任务: {len(edge_tasks)} 个")

    # =====================================================
    # Phase 3: 拓扑路径规划（新增）
    # =====================================================
    print()
    print("Phase 3: 拓扑路径规划")
    print("-"*70)

    # Step 9.5: 拓扑路径规划
    edge_ord, edge_dir, topo_path_3d = planner.step9_5_plan_topo_mission()

    print()
    print(f"[完成] 边任务顺序: {edge_ord}")
    print(f"[完成] 方向映射: {edge_dir}")
    print(f"[完成] 3D路径: {len(topo_path_3d)} 个点")

    # =====================================================
    # Phase 4: 对比分析
    # =====================================================
    print()
    print("Phase 4: 对比分析")
    print("-"*70)

    print("[对比] 线段级方法 vs 拓扑级方法:")
    print()
    print("线段级方法:")
    print(f"  - 任务单元: LineTask（整条线路）")
    print(f"  - 任务数量: {len(planner.independent_lines)} 个")
    print(f"  - 切换代价: 欧氏距离")
    print()
    print("拓扑级方法:")
    print(f"  - 任务单元: EdgeTask（可能被切开）")
    print(f"  - 任务数量: {len(edge_tasks)} 个")
    print(f"  - 切换代价: 图路径长度（相邻边代价=0）")

    # =====================================================
    # Phase 5: 拓扑层统计
    # =====================================================
    print()
    print("Phase 5: 拓扑层统计信息")
    print("-"*70)

    print("[节点统计]")
    node_kinds = {}
    for node in planner.topo_nodes:
        node_kinds[node.kind] = node_kinds.get(node.kind, 0) + 1

    for kind, count in node_kinds.items():
        print(f"  - {kind}: {count} 个")

    print()
    print("[边统计]")
    straight_edges = sum(1 for e in planner.topo_edges if e.is_straight)
    curved_edges = len(planner.topo_edges) - straight_edges
    print(f"  - 直线边: {straight_edges} 条")
    print(f"  - 曲线边: {curved_edges} 条")

    print()
    print("[细分统计]")
    if planner.topo_stats.get('line_split_info'):
        print("  线路细分情况:")
        for line_id, num_edges in planner.topo_stats['line_split_info'].items():
            print(f"    {line_id}: {num_edges} 条边")

    print()
    print("="*70)
    print("拓扑层演示完成！")
    print("="*70)

    # =====================================================
    # 输出文件
    # =====================================================
    print()
    print("[输出文件]")
    print("  - result/step7_5_topo_nodes.png (拓扑图)")
    print("  - result/step8_5_topo_edges.png (边编号图)")

    return planner


if __name__ == "__main__":
    planner = run_topo_demo()
