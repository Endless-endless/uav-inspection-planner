"""
=====================================================
起点驱动的电网巡检重规划 Demo
=====================================================

功能：
1. 接受用户指定的起点坐标
2. 基于新起点重新规划完整巡检任务
3. 生成新的 mission JSON 和可视化页面

使用方式：
    python demo/demo_start_point_driven.py --x 300 --y 400

    或不带参数使用默认起点：
    python demo/demo_start_point_driven.py

输出：
    result/latest/start_driven_mission.json - 起点驱动的任务 JSON
    result/latest/main_view_start_driven.html - 交互式展示页

特点：
- 真正的任务重规划（非旧路径切片）
- 自动找到最接近起点的电网分组
- 重新计算所有分组的访问顺序
- 生成完整的新巡检路径

=====================================================
"""

import sys
import os
import argparse
import numpy as np

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
from core.topo_plan import (
    export_grouped_mission_to_json,
    build_grouped_continuous_mission_from_start,
    group_edges_spatially,
    compute_edge_centroids,
    build_edge_adjacency_simple
)


def main(start_x=None, start_y=None):
    """
    主函数：起点驱动的重规划

    Args:
        start_x: 起点 X 坐标（可选）
        start_y: 起点 Y 坐标（可选）
    """
    print("="*70)
    print("起点驱动的电网巡检重规划")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 执行完整规划流程（复用现有数据）
    # =====================================================
    print("Phase 1: 加载现有规划数据...")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/test.png',
        flight_height=30
    )

    # 执行图像处理和线路识别
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()
    planner.step5_generate_line_inspection_points()

    # 设置地形
    terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
    planner.step6_map_line_points_to_3d(terrain)
    planner.step6_smooth_terrain(terrain)

    # 拓扑建模
    topo_graph = planner.step7_5_build_topo()

    # EdgeTask 建模
    edge_tasks = planner.step8_5_build_edge_tasks()

    print(f"[完成] 独立线路: {len(planner.independent_lines)} 条")
    print(f"[完成] 巡检点: {len(planner.line_inspection_points)} 个")
    print(f"[完成] 拓扑节点: {len(topo_graph.nodes)} 个")
    print(f"[完成] 拓扑边: {len(topo_graph.edges)} 条")
    print(f"[完成] 边任务: {len(edge_tasks)} 个")
    print()

    # =====================================================
    # Phase 2: 分组边任务
    # =====================================================
    print("Phase 2: 对边任务进行空间分组...")
    print("-"*70)

    centroids = compute_edge_centroids(edge_tasks)
    groups = group_edges_spatially(edge_tasks, centroids, eps=150.0)

    print(f"[完成] 分组数: {len(groups)} 个")
    for gid, group in list(groups.items())[:3]:
        print(f"  - {gid}: {len(group.edge_ids)} 条边")
    if len(groups) > 3:
        print(f"  - ...")
    print()

    # =====================================================
    # Phase 3: 确定起点
    # =====================================================
    print("Phase 3: 确定起始位置...")
    print("-"*70)

    if start_x is None or start_y is None:
        # 使用默认起点：路径中心
        all_coords = []
        for task in edge_tasks:
            all_coords.extend(task.polyline)
        if all_coords:
            start_x = np.mean([p[0] for p in all_coords])
            start_y = np.mean([p[1] for p in all_coords])
            print(f"[默认起点] 路径中心: ({start_x:.1f}, {start_y:.1f})")
        else:
            start_x = 400
            start_y = 400
            print(f"[默认起点] 固定值: ({start_x:.1f}, {start_y:.1f})")
    else:
        print(f"[用户起点] ({start_x:.1f}, {start_y:.1f})")
    print()

    # =====================================================
    # Phase 4: 起点驱动的任务重规划
    # =====================================================
    print("Phase 4: 起点驱动的任务重规划...")
    print("-"*70)

    adjacency = build_edge_adjacency_simple(topo_graph)

    # 使用新的起点驱动规划函数
    mission = build_grouped_continuous_mission_from_start(
        topo_graph=topo_graph,
        edge_tasks=edge_tasks,
        groups=groups,
        start_xy=(start_x, start_y),
        adjacency=adjacency
    )

    if mission is None:
        print("[FAIL] 规划失败")
        return None

    print(f"[完成] 总长度: {mission.total_length:.1f}px")
    print(f"[完成] 巡检: {mission.inspect_length:.1f}px")
    print(f"[完成] 连接: {mission.connect_length:.1f}px")
    if hasattr(mission, 'start_xy'):
        print(f"[完成] 使用起点: ({mission.start_xy[0]:.1f}, {mission.start_xy[1]:.1f})")
    if hasattr(mission, 'start_group_id'):
        print(f"[完成] 起始分组: {mission.start_group_id}")
    print()

    # =====================================================
    # Phase 5: 导出 JSON
    # =====================================================
    print("Phase 5: 导出任务 JSON...")
    print("-"*70)

    json_path = export_grouped_mission_to_json(
        mission=mission,
        edge_tasks=edge_tasks,
        line_inspection_points_by_line=planner.line_inspection_points_by_line,
        output_path="result/latest/start_driven_mission.json",
        terrain_3d=planner.terrain_3d
    )

    print(f"[完成] JSON 已导出: {json_path}")
    print()

    # =====================================================
    # Phase 6: 生成交互式展示页
    # =====================================================
    print("Phase 6: 生成交互式展示页...")
    print("-"*70)

    try:
        from demo.generate_interactive_main_view import create_interactive_main_view

        html_path = create_interactive_main_view(
            map_image_path='data/test.png',
            mission_json_path=json_path,
            output_html_path='result/latest/main_view_start_driven.html'
        )

        print(f"[完成] 交互式展示页已生成: {html_path}")
        print()

    except Exception as e:
        print(f"[WARN] 交互式展示页生成失败: {e}")
        print()

    # =====================================================
    # 完成
    # =====================================================
    print("="*70)
    print("起点驱动重规划完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/latest/start_driven_mission.json")
    print("  - result/latest/main_view_start_driven.html")
    print()
    print("[使用说明]")
    print("  在浏览器中打开 main_view_start_driven.html 查看结果")
    print("  使用飞行控制面板模拟飞行")
    print()

    return mission


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='起点驱动的电网巡检重规划')
    parser.add_argument('--x', type=float, help='起点 X 坐标')
    parser.add_argument('--y', type=float, help='起点 Y 坐标')

    args = parser.parse_args()

    mission = main(start_x=args.x, start_y=args.y)
