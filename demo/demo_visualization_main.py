"""
=====================================================
UAV 电网巡检 - 主展示页入口（交互式版本）
=====================================================

功能：
1. 执行拓扑规划并生成 3D 巡检路径
2. 导出标准 mission JSON
3. 创建交互式主展示页（基于 Plotly layout.images）
4. 显示：真实地图底图 + 巡检路径 + inspection points（可交互）

使用方式：
    python demo/demo_visualization_main.py

输出：
    result/latest/mission_output.json - 标准任务 JSON
    result/latest/main_view_interactive.html - 交互式主展示页

特点：
- 使用 Plotly layout.images 显示真实地图（data/test.png）
- 路径和 inspection points 支持 hover 交互
- 图层控制（sample points 可通过图例开关）
- 系统界面风格（非报告页）

=====================================================
"""

import sys
import os
import numpy as np

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
from core.topo_plan import export_grouped_mission_to_json


def main():
    """
    主函数：生成主展示页
    """
    print("="*70)
    print("UAV 电网巡检 - 主展示页生成")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 执行完整规划流程
    # =====================================================
    print("Phase 1: 执行拓扑规划...")
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
    # Phase 2: 生成分组规划路径（全局拓扑优化）
    # =====================================================
    print()
    print("Phase 2: 生成分组规划路径（全局拓扑优化）...")
    print("-"*70)

    try:
        # 使用 step9_4 全局拓扑优化（基于论文思想）
        mission = planner.step9_4_plan_global_topology_optimized(
            enable_sa=False,  # 使用纯贪心策略（模拟退火在此数据集上未表现优势）
            eps=150.0
        )
    except ImportError as e:
        print(f"[WARN] 全局优化需要 scikit-learn，切换到基础连续规划")
        print(f"  错误: {e}")
        mission = planner.step9_1_plan_continuous_mission_greedy()

    if mission is None:
        print("[FAIL] 规划失败")
        return None

    print(f"[完成] 总长度: {mission.total_length:.1f}px")
    print(f"[完成] 巡检: {mission.inspect_length:.1f}px")
    print(f"[完成] 连接: {mission.connect_length:.1f}px")
    print()

    # =====================================================
    # Phase 2.5: 导出 JSON
    # =====================================================
    print()
    print("Phase 2.5: 导出任务 JSON...")
    print("-"*70)

    json_path = export_grouped_mission_to_json(
        mission=mission,
        edge_tasks=edge_tasks,
        line_inspection_points_by_line=planner.line_inspection_points_by_line,
        output_path="result/latest/mission_output.json",
        terrain_3d=planner.terrain_3d
    )

    print(f"[完成] JSON 已导出: {json_path}")
    print()

    # =====================================================
    # Phase 3: 构建 3D 路径
    # =====================================================
    print()
    print("Phase 3: 构建 3D 路径...")
    print("-"*70)

    # 将 2D 路径转换为 3D（使用地形高度）
    path_3d = []
    for point_2d in mission.full_path:
        x, y = point_2d
        # 从地形获取高度，或使用默认飞行高度
        h_idx, w_idx = int(y), int(x)
        if 0 <= h_idx < planner.height and 0 <= w_idx < planner.width:
            z = planner.terrain_3d[h_idx, w_idx, 2]
        else:
            z = planner.flight_height
        path_3d.append((x, y, z))

    print(f"[完成] 3D 路径点: {len(path_3d)} 个")
    print()

    # =====================================================
    # Phase 4: 生成交互式主展示页
    # =====================================================
    print()
    print("Phase 4: 生成交互式主展示页（基于 Plotly layout.images）...")
    print("-"*70)

    try:
        from demo.generate_interactive_main_view import create_interactive_main_view

        # 生成交互式主展示页
        html_path = create_interactive_main_view(
            map_image_path='data/test.png',
            mission_json_path=json_path,
            output_html_path='result/latest/main_view_interactive.html'
        )

        print(f"[完成] 交互式主展示页已生成: {html_path}")
        print()

    except Exception as e:
        print(f"[WARN] 交互式主展示页生成失败: {e}")
        print(f"  请检查: {type(e).__name__}")
        print()
        return planner

    # =====================================================
    # Phase 5: 生成2D地图叠加页（基于 data/test.png）
    # =====================================================
    print()
    print("Phase 5: 生成2D地图叠加页（基于 data/test.png）...")
    print("-"*70)

    try:
        from visualization.map_overlay import plot_path_on_real_map, plot_path_on_real_map_with_coords

        # 使用 data/test.png 作为底图
        map_image_path = 'data/test.png'
        if os.path.exists(map_image_path):
            # 标准版（基于 data/test.png）
            plot_path_on_real_map(
                map_image_path=map_image_path,
                path=path_3d,
                resolution=1.0,
                output_path="result/latest/map_overlay_test.png",
                start_marker=True,
                end_marker=True,
                path_color='blue',  # 使用蓝色以突出路径
                path_width=3.0,     # 加粗路径线
                dpi=150
            )

            # 带坐标版（基于 data/test.png）
            plot_path_on_real_map_with_coords(
                map_image_path=map_image_path,
                path=path_3d,
                b_min=(0, 0, 0),
                resolution=1.0,
                output_path="result/latest/map_overlay_test_with_coords.png",
                show_coords=True
            )

            print("[完成] 2D地图叠加页已生成（基于 data/test.png）")
            print()
        else:
            print(f"[跳过] 地图文件不存在: {map_image_path}")
            print()

    except Exception as e:
        print(f"[WARN] 2D地图叠加生成失败: {e}")
        print(f"  错误详情: {type(e).__name__}")
        print()

    # =====================================================
    # 完成
    # =====================================================
    print("="*70)
    print("处理完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/latest/mission_output.json (标准任务 JSON)")
    print("  - result/latest/main_view_interactive.html (主展示页 - 基于 data/test.png 地图底图)")
    print("  - result/latest/map_overlay_test.png (2D地图叠加 - 基于 data/test.png)")
    print("  - result/latest/map_overlay_test_with_coords.png (2D地图叠加 - 带坐标)")
    print("  - result/latest/01-08*.png (各阶段结果)")
    print()
    print("[交互式主展示页说明]")
    print("  - 在浏览器中打开 main_view_interactive.html")
    print("  - 基于 data/test.png 真实地图的交互式展示")
    print("  - 显示：真实地图 + 蓝色路径 + inspection points")
    print("  - 交互功能：")
    print("    * 鼠标 hover 路径查看路径点信息")
    print("    * 鼠标 hover inspection points 查看点详情")
    print("    * 点击图例显示/隐藏 sample points")
    print("    * 使用工具栏缩放、平移、选择区域")
    print()
    print("[辅助页面]")
    print("  - 导航页: python demo/demo_ui_animation.py")
    print("    (输入坐标，找到最近工点并更新执行路径)")
    print()
    print("[底图用途]")
    print("  - data/test.png: 主展示页底图（统一数据源）")
    print()

    return planner


if __name__ == "__main__":
    planner = main()
