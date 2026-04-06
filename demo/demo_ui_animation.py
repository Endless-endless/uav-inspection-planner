"""
=====================================================
UAV 电网巡检 - 辅助导航页入口
=====================================================

功能：
1. 执行拓扑规划并生成 3D 巡检路径
2. 导出标准 mission JSON
3. 创建交互式导航页（右侧输入坐标，找到最近工点并更新执行路径）

使用方式：
    python demo/demo_ui_animation.py

输出：
    result/latest/mission_output.json - 标准任务 JSON
    result/latest/navigation_view.html - 辅助导航页

注意：
    - 这是辅助导航/调试页面，不是主展示页
    - 主展示页请使用 demo/demo_visualization_main.py

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
    主函数：生成 UAV 巡检动画和 JSON 导出
    """
    print("="*70)
    print("UAV 电网巡检 - 可视化动画 + JSON 导出")
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
    # Phase 2: 生成分组规划路径
    # =====================================================
    print()
    print("Phase 2: 生成分组规划路径...")
    print("-"*70)

    try:
        mission = planner.step9_2_plan_grouped_continuous_mission(eps=150.0)
    except ImportError as e:
        print(f"[WARN] 分组规划需要 scikit-learn，切换到基础连续规划")
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
    # Phase 4: 生成带 inspection point 的动画
    # =====================================================
    print()
    print("Phase 4: 生成交互式动画（带 inspection point）...")
    print("-"*70)

    try:
        from core.visualization_enhanced import create_animation_view_with_inspection_points

        # 创建输出目录
        os.makedirs("result/latest", exist_ok=True)

        # 生成导航页（带 inspection point 交互）
        create_animation_view_with_inspection_points(
            path_3d=path_3d,
            mission_json_path=json_path,
            edge_tasks=edge_tasks,
            output_file="result/latest/navigation_view.html",
            downsample_step=3
        )

        print("[完成] 动画已生成")
        print()

    except ImportError as e:
        print(f"[WARN] 动画生成需要 plotly")
        print(f"  错误: {e}")
        print(f"  安装: pip install plotly")
        print()
        return planner

    # =====================================================
    # 完成
    # =====================================================
    print("="*70)
    print("处理完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  - result/latest/mission_output.json (标准任务 JSON)")
    print("  - result/latest/navigation_view.html (辅助导航页)")
    print("  - result/latest/01-08*.png (各阶段结果)")
    print()
    print("[使用说明]")
    print("  1. 在浏览器中打开 navigation_view.html")
    print("  2. 右侧输入 X/Y/Z 坐标")
    print("  3. 点击\"开始工检\"找到最近工点并更新执行路径")
    print("  4. 点击 inspection point 查看详细信息")
    print()
    print("[注意]")
    print("  - 这是辅助导航/调试页面")
    print("  - 主展示页请运行: python demo/demo_visualization_main.py")
    print()

    return planner


if __name__ == "__main__":
    planner = main()
