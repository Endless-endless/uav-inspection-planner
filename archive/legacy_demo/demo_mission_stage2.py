"""
=====================================================
任务优化演示 - 阶段2
=====================================================

功能：
1. 从图中识别多条独立红色电网线路
2. 为每条线路生成巡检点
3. 构建任务层
4. 优化任务顺序和方向
5. 生成全局巡检路径

使用方式：
    python demo/demo_mission_stage2.py

=====================================================
"""

import numpy as np
import os
import sys
from PIL import Image

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from planner.powerline_planner_v3_final import plan_powerline_mission_v2
from core.terrain import TerrainGenerator


def main():
    """主函数"""
    print("=" * 70)
    print("任务优化演示 - 阶段2")
    print("=" * 70)

    # 配置参数
    IMAGE_PATH = "data/test.png"

    # 飞行参数
    FLIGHT_HEIGHT = 25
    MIN_PIXELS = 20
    INSPECTION_SPACING = 100.0
    ANGLE_THRESHOLD = 25.0
    MAX_POINTS_PER_LINE = 30  # 每条线最大30个点（优化后）

    # 地形参数
    GAUSSIAN_SIGMA = 3.0

    # 稳定性参数
    ALLOW_BREAK_FIX = False

    # 任务优化参数
    START_POS = None  # 可设置起始位置，如 (100, 100)
    WIND = {'direction': 0, 'speed': 5}  # 风向信息

    # 检查图像是否存在
    if not os.path.exists(IMAGE_PATH):
        print(f"[错误] 图像不存在: {IMAGE_PATH}")
        return

    # 加载图像
    img = Image.open(IMAGE_PATH)
    w, h = img.size
    print(f"\n[图像] 尺寸: {w}x{h}")

    # 生成地形
    print(f"\n[地形] 生成真实感地形...")
    terrain_raw = TerrainGenerator.generate_realistic_terrain(w, h)

    # 执行任务规划
    print(f"\n[规划] 开始任务规划...")
    planner = plan_powerline_mission_v2(
        image_path=IMAGE_PATH,
        terrain_raw=terrain_raw,
        flight_height=FLIGHT_HEIGHT,
        min_pixels=MIN_PIXELS,
        inspection_spacing=INSPECTION_SPACING,
        angle_threshold_deg=ANGLE_THRESHOLD,
        max_points_per_line=MAX_POINTS_PER_LINE,
        gaussian_sigma=GAUSSIAN_SIGMA,
        allow_break_fix=ALLOW_BREAK_FIX,
        start_pos=START_POS,
        wind=WIND,
        export_html=True,  # 启用HTML导出
        html_output_path="result/mission_stage2_demo.html"
    )

    # 打印详细统计信息
    print(f"\n" + "=" * 70)
    print("详细统计信息")
    print("=" * 70)

    # 任务层信息
    if planner.tasks:
        print(f"\n[任务层信息]")
        print(f"  总任务数: {len(planner.tasks)}")

        for i, task in enumerate(planner.tasks):
            print(f"\n  任务 {i+1}: {task.id}")
            print(f"    - 类型: {task.kind}")
            print(f"    - 长度: {task.len2d:.1f}px")
            print(f"    - 点数: {task.n_pts}")
            print(f"    - 起点: ({task.p_start[0]:.1f}, {task.p_start[1]:.1f})")
            print(f"    - 终点: ({task.p_end[0]:.1f}, {task.p_end[1]:.1f})")

    # 优化结果
    if planner.line_ord and planner.line_dir:
        print(f"\n[优化结果]")
        print(f"  任务顺序: {[planner.tasks[i].id for i in planner.line_ord]}")
        print(f"  任务方向:")
        for task_id, direction in planner.line_dir.items():
            dir_str = "正向" if direction == 1 else "反向"
            print(f"    - {task_id}: {dir_str}")

    # 全局路径统计
    if planner.g_stats:
        print(f"\n[全局路径统计]")
        print(f"  总长度: {planner.g_stats['total_len']:.1f}px")
        print(f"  总成本: {planner.g_stats['total_cost']:.1f}")
        print(f"  总点数: {planner.g_stats['n_points']}")
        print(f"  任务数: {planner.g_stats['n_tasks']}")

    # 对比信息
    if planner.independent_lines and planner.g_stats:
        print(f"\n[对比信息]")
        total_line_len = sum(line.length_2d for line in planner.independent_lines)
        print(f"  原始线路总长: {total_line_len:.1f}px")
        print(f"  全局路径长度: {planner.g_stats['total_len']:.1f}px")
        print(f"  过渡距离: {planner.g_stats['total_len'] - total_line_len:.1f}px")
        print(f"  巡检点总数: {sum(len(planner.line_inspection_points_by_line.get(line.id, [])) for line in planner.independent_lines)}")

    # 显示输出文件
    print(f"\n[输出文件]")
    print(f"  - result/step1_hsv_mask.png")
    print(f"  - result/step3_skeleton.png")
    print(f"  - result/step4_independent_lines.png")
    print(f"  - result/step5_line_inspection_points.png")
    print(f"  - result/step9_g_path.png")
    print(f"  - result/mission_stage2_demo.html (3D交互演示)")

    print(f"\n" + "=" * 70)
    print("演示完成！")
    print("=" * 70)

    # 返回规划器，方便后续使用
    return planner


if __name__ == "__main__":
    planner = main()
