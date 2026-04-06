"""
生成主展示页 - 基于真实地图 data/test.png 的高质量2D叠加
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image
import json

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
from core.topo_plan import export_grouped_mission_to_json


def create_main_view_with_real_map(
    map_image_path='data/test.png',
    mission_json_path='result/latest/mission_output.json',
    output_image_path='result/latest/main_view_map_based.png',
    output_html_path='result/latest/main_view_map_based.html'
):
    """
    创建基于真实地图的主展示页

    使用 matplotlib 直接在真实地图上叠加路径，
    确保地图的真实内容清晰可见
    """

    print("="*70)
    print("生成主展示页 - 基于真实地图2D叠加")
    print("="*70)

    # 创建输出目录
    os.makedirs(os.path.dirname(output_image_path), exist_ok=True)

    # =====================================================
    # 加载地图
    # =====================================================
    print(f"\n[INFO] 加载真实地图: {map_image_path}")

    if not os.path.exists(map_image_path):
        print(f"[ERROR] 地图文件不存在: {map_image_path}")
        return None

    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)
    map_height, map_width = map_array.shape[:2]

    print(f"[INFO] 地图尺寸: {map_width} x {map_height}")

    # =====================================================
    # 加载任务数据
    # =====================================================
    print(f"\n[INFO] 加载任务数据: {mission_json_path}")

    if not os.path.exists(mission_json_path):
        print(f"[ERROR] 任务JSON不存在: {mission_json_path}")
        return None

    with open(mission_json_path, 'r', encoding='utf-8') as f:
        mission_data = json.load(f)

    # 提取路径
    path = mission_data.get('full_path', [])
    inspection_points = mission_data.get('inspection_points', [])

    if not path:
        print("[ERROR] 路径数据为空")
        return None

    print(f"[INFO] 路径点数: {len(path)}")
    print(f"[INFO] 巡检点数: {len(inspection_points)}")

    # =====================================================
    # 创建高质量2D地图叠加
    # =====================================================
    print("\n[INFO] 创建2D地图叠加...")

    # 设置高分辨率
    dpi = 150

    # 计算图形尺寸（保持地图比例）
    aspect = map_width / map_height
    if aspect > 1:
        figsize = (16, 16 / aspect)
    else:
        figsize = (16 * aspect, 16)

    # 创建图形
    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    # =====================================================
    # 显示真实地图作为背景
    # =====================================================
    ax.imshow(map_array, origin='upper')

    # =====================================================
    # 叠加路径
    # =====================================================
    if path:
        # 提取坐标
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]

        # 绘制路径线（蓝色，清晰可见）
        ax.plot(path_x, path_y,
                color='blue',
                linewidth=2.5,
                alpha=0.8,
                label='规划路径')

        # 绘制路径方向箭头（每隔一段距离）
        arrow_interval = max(1, len(path) // 20)
        for i in range(0, len(path) - arrow_interval, arrow_interval):
            x1, y1 = path[i][0], path[i][1]
            x2, y2 = path[i + arrow_interval][0], path[i + arrow_interval][1]
            dx, dy = x2 - x1, y2 - y1

            # 避免零长度箭头
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                ax.arrow(x1, y1, dx * 0.3, dy * 0.3,
                        head_width=3, head_length=3,
                        fc='blue', ec='blue', alpha=0.5)

    # =====================================================
    # 叠加 inspection points
    # =====================================================
    if inspection_points:
        # 分类显示不同类型的点
        endpoint_points = [p for p in inspection_points if p.get('point_type') == 'endpoint']
        turning_points = [p for p in inspection_points if p.get('point_type') == 'turning']
        sample_points = [p for p in inspection_points if p.get('point_type') == 'sample']

        # 终点（红色菱形）
        if endpoint_points:
            end_x = [p['x'] for p in endpoint_points]
            end_y = [p['y'] for p in endpoint_points]
            ax.scatter(end_x, end_y,
                      c='red', marker='D',
                      s=100, edgecolors='white',
                      linewidths=2, zorder=5,
                      label='终点')

        # 转向点（橙色圆形）
        if turning_points:
            turn_x = [p['x'] for p in turning_points]
            turn_y = [p['y'] for p in turning_points]
            ax.scatter(turn_x, turn_y,
                      c='orange', marker='o',
                      s=60, edgecolors='white',
                      linewidths=1.5, zorder=4,
                      label='转向点')

        # 采样点（青色小点，默认不显示，避免太密集）
        # 如果需要显示，取消下面的注释
        # if sample_points:
        #     sample_x = [p['x'] for p in sample_points]
        #     sample_y = [p['y'] for p in sample_points]
        #     ax.scatter(sample_x, sample_y,
        #               c='cyan', marker='.',
        #               s=10, alpha=0.5,
        #               zorder=3, label='采样点')

    # =====================================================
    # 标记起点和终点
    # =====================================================
    if path:
        # 起点（绿色大方块）
        start_x, start_y = path[0][0], path[0][1]
        ax.scatter([start_x], [start_y],
                  c='green', marker='s',
                  s=150, edgecolors='white',
                  linewidths=2, zorder=6,
                  label='起点')

        # 终点（红色大方块，如果还没画过）
        if not endpoint_points:
            end_x, end_y = path[-1][0], path[-1][1]
            ax.scatter([end_x], [end_y],
                      c='red', marker='s',
                      s=150, edgecolors='white',
                      linewidths=2, zorder=6,
                      label='终点')

    # =====================================================
    # 设置图形属性
    # =====================================================
    ax.set_xlabel('X (像素)', fontsize=12)
    ax.set_ylabel('Y (像素)', fontsize=12)
    ax.set_title('UAV 电网巡检路径规划 - 基于真实地图', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.2, linestyle='--')

    # 保存图片
    plt.tight_layout()
    plt.savefig(output_image_path, dpi=dpi, bbox_inches='tight')
    print(f"[INFO] 地图叠加已保存: {output_image_path}")
    plt.close()

    # =====================================================
    # 生成 HTML
    # =====================================================
    print(f"\n[INFO] 生成 HTML: {output_html_path}")

    # 计算统计数据
    total_length = 0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        total_length += np.sqrt(dx**2 + dy**2)

    start_x, start_y = path[0][0], path[0][1]
    end_x, end_y = path[-1][0], path[-1][1]

    html_content = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>UAV 电网巡检 - 主展示页（基于真实地图）</title>
    <style>
        body {{
            margin: 0;
            padding: 20px;
            font-family: 'Segoe UI', Arial, sans-serif;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1600px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        h1 {{
            text-align: center;
            color: #2c3e50;
            margin-bottom: 10px;
            font-size: 28px;
        }}
        .subtitle {{
            text-align: center;
            color: #7f8c8d;
            margin-bottom: 30px;
            font-size: 16px;
        }}
        .map-container {{
            text-align: center;
            margin-bottom: 30px;
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
        }}
        .map-image {{
            max-width: 100%;
            height: auto;
            border: 1px solid #ddd;
            border-radius: 4px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }}
        .stats {{
            background-color: #ecf0f1;
            padding: 20px;
            border-radius: 8px;
            margin-top: 20px;
        }}
        .stats h3 {{
            margin-top: 0;
            color: #2c3e50;
            font-size: 20px;
            border-bottom: 2px solid #3498db;
            padding-bottom: 10px;
        }}
        .stat-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }}
        .stat-item {{
            background-color: white;
            padding: 15px;
            border-radius: 6px;
            border-left: 4px solid #3498db;
        }}
        .stat-label {{
            font-size: 14px;
            color: #7f8c8d;
            margin-bottom: 5px;
        }}
        .stat-value {{
            font-size: 24px;
            font-weight: bold;
            color: #2c3e50;
        }}
        .legend {{
            margin-top: 20px;
            padding: 15px;
            background-color: #fff3cd;
            border-radius: 6px;
            border-left: 4px solid #ffc107;
        }}
        .legend h4 {{
            margin-top: 0;
            color: #856404;
        }}
        .legend-item {{
            display: inline-block;
            margin-right: 20px;
            margin-bottom: 5px;
        }}
        .legend-marker {{
            display: inline-block;
            width: 16px;
            height: 16px;
            margin-right: 5px;
            vertical-align: middle;
            border: 1px solid #333;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>UAV 电网巡检 - 主展示页</h1>
        <p class="subtitle">基于真实地图 {map_image_path} 的高质量路径叠加可视化</p>

        <div class="map-container">
            <img src="main_view_map_based.png" alt="UAV路径规划结果" class="map-image">
        </div>

        <div class="stats">
            <h3>任务统计</h3>
            <div class="stat-grid">
                <div class="stat-item">
                    <div class="stat-label">总路径长度</div>
                    <div class="stat-value">{total_length:.1f} px</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">路径点数</div>
                    <div class="stat-value">{len(path)}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">巡检点数</div>
                    <div class="stat-value">{len(inspection_points)}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">起点坐标</div>
                    <div class="stat-value">({start_x:.1f}, {start_y:.1f})</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">终点坐标</div>
                    <div class="stat-value">({end_x:.1f}, {end_y:.1f})</div>
                </div>
            </div>
        </div>

        <div class="legend">
            <h4>图例说明</h4>
            <div class="legend-item">
                <span class="legend-marker" style="background-color: green; border-radius: 0;"></span>
                起点
            </div>
            <div class="legend-item">
                <span class="legend-marker" style="background-color: red; border-radius: 0;"></span>
                终点
            </div>
            <div class="legend-item">
                <span class="legend-marker" style="background-color: blue; width: 30px; height: 3px;"></span>
                规划路径
            </div>
            <div class="legend-item">
                <span class="legend-marker" style="background-color: orange; border-radius: 50%;"></span>
                转向点
            </div>
        </div>
    </div>
</body>
</html>"""

    with open(output_html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"[INFO] HTML已保存: {output_html_path}")

    print("\n" + "="*70)
    print("完成！")
    print("="*70)
    print(f"\n[输出文件]")
    print(f"  - 地图叠加: {output_image_path}")
    print(f"  - HTML页面: {output_html_path}")
    print(f"\n请在浏览器中打开: {output_html_path}")

    return output_image_path, output_html_path


def main():
    """
    主函数：执行完整流程
    """
    print("="*70)
    print("UAV 电网巡检 - 主展示页生成（基于真实地图）")
    print("="*70)
    print()

    # =====================================================
    # Phase 1: 执行拓扑规划
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

    print(f"[完成] 独立线路: {len(planner.independent_lines)} 条")
    print(f"[完成] 巡检点: {len(planner.line_inspection_points)} 个")
    print()

    # =====================================================
    # Phase 2: 生成分组规划路径
    # =====================================================
    print("Phase 2: 生成分组规划路径...")
    print("-"*70)

    # 设置地形
    terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
    planner.step6_map_line_points_to_3d(terrain)
    planner.step6_smooth_terrain(terrain)

    # 拓扑建模
    topo_graph = planner.step7_5_build_topo()
    edge_tasks = planner.step8_5_build_edge_tasks()

    # 规划路径
    try:
        mission = planner.step9_2_plan_grouped_continuous_mission(eps=150.0)
    except ImportError:
        print("[WARN] 分组规划需要 scikit-learn，切换到基础连续规划")
        mission = planner.step9_1_plan_continuous_mission_greedy()

    if mission is None:
        print("[FAIL] 规划失败")
        return None

    print(f"[完成] 总长度: {mission.total_length:.1f}px")
    print()

    # =====================================================
    # Phase 2.5: 导出 JSON
    # =====================================================
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
    # Phase 3: 生成主展示页（基于真实地图）
    # =====================================================
    print("Phase 3: 生成主展示页（基于真实地图）...")
    print("-"*70)

    create_main_view_with_real_map(
        map_image_path='data/test.png',
        mission_json_path=json_path,
        output_image_path='result/latest/main_view_map_based.png',
        output_html_path='result/latest/main_view_map_based.html'
    )

    print("\n" + "="*70)
    print("全部完成！")
    print("="*70)


if __name__ == "__main__":
    main()
