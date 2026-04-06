"""
=====================================================
多独立线路巡检规划演示 - 阶段1
=====================================================

功能：
1. 从图中识别多条独立红色电网线路
2. 每条线路保持独立，不自动连接
3. 为每条线路生成巡检点（端点、拐点、采样点）
4. 映射到3D高程
5. 输出统计信息和可视化

使用方式：
    python demo/demo_independent_lines_stage1.py

=====================================================
"""

import numpy as np
import os
import sys
from PIL import Image

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from planner.powerline_planner_v3_final import plan_powerline_independent_lines_v1
from core.terrain import TerrainGenerator


def main():
    """主函数"""
    print("=" * 70)
    print("多独立线路巡检规划演示 - 阶段1")
    print("=" * 70)

    # 配置参数
    IMAGE_PATH = "data/test.png"

    # 飞行参数
    FLIGHT_HEIGHT = 25
    MIN_PIXELS = 20  # 最小像素数阈值（过滤噪声）
    INSPECTION_SPACING = 100.0  # 巡检点采样间距（像素）
    ANGLE_THRESHOLD = 25.0  # 拐点角度阈值
    MAX_POINTS_PER_LINE = 50  # 每条线最大点数限制

    # 地形参数
    GAUSSIAN_SIGMA = 3.0

    # 稳定性参数
    ALLOW_BREAK_FIX = False  # 默认关闭断裂修复

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

    # 执行多独立线路规划
    print(f"\n[规划] 开始多独立线路规划...")
    planner = plan_powerline_independent_lines_v1(
        image_path=IMAGE_PATH,
        terrain_raw=terrain_raw,
        flight_height=FLIGHT_HEIGHT,
        min_pixels=MIN_PIXELS,
        inspection_spacing=INSPECTION_SPACING,
        angle_threshold_deg=ANGLE_THRESHOLD,
        max_points_per_line=MAX_POINTS_PER_LINE,
        gaussian_sigma=GAUSSIAN_SIGMA,
        allow_break_fix=ALLOW_BREAK_FIX
    )

    # 打印详细统计信息
    print(f"\n" + "=" * 70)
    print("详细统计信息")
    print("=" * 70)

    # 线路统计
    if planner.independent_lines:
        print(f"\n[线路信息]")
        print(f"  总线路数: {len(planner.independent_lines)}")
        print(f"  主线路ID: {planner.primary_line_id}")

        for i, line in enumerate(planner.independent_lines):
            print(f"\n  线路 {i+1}: {line.id}")
            print(f"    - 长度: {line.length_2d:.1f}px")
            print(f"    - 点数: {len(line.ordered_pixels)}")
            print(f"    - 端点: {line.endpoints}")
            print(f"    - BBox: {line.bbox}")

    # 巡检点统计
    if planner.line_inspection_points_by_line:
        print(f"\n[巡检点信息]")
        print(f"  总巡检点数: {len(planner.line_inspection_points)}")

        # 按线路统计
        for line_id, points in planner.line_inspection_points_by_line.items():
            endpoint_count = sum(1 for p in points if p.point_type == "endpoint")
            turning_count = sum(1 for p in points if p.point_type == "turning")
            sample_count = sum(1 for p in points if p.point_type == "sample")

            print(f"\n  {line_id}:")
            print(f"    - 总点数: {len(points)}")
            print(f"    - 端点: {endpoint_count}")
            print(f"    - 拐点: {turning_count}")
            print(f"    - 采样点: {sample_count}")

        # 按类型统计
        endpoint_count = sum(1 for p in planner.line_inspection_points if p.point_type == "endpoint")
        turning_count = sum(1 for p in planner.line_inspection_points if p.point_type == "turning")
        sample_count = sum(1 for p in planner.line_inspection_points if p.point_type == "sample")
        high_priority_count = sum(1 for p in planner.line_inspection_points if p.priority == "high")

        print(f"\n[类型统计]")
        print(f"  端点: {endpoint_count} (高优先级)")
        print(f"  拐点: {turning_count} (高优先级)")
        print(f"  采样点: {sample_count} (普通优先级)")
        print(f"  高优先级总计: {high_priority_count}")

    # 显示输出文件
    print(f"\n[输出文件]")
    print(f"  - result/step1_hsv_mask.png")
    print(f"  - result/step2_fixed_mask.png")
    print(f"  - result/step3_skeleton.png")
    print(f"  - result/step4_independent_lines.png")
    print(f"  - result/step5_line_inspection_points.png")

    print(f"\n" + "=" * 70)
    print("演示完成！")
    print("=" * 70)

    # 返回规划器，方便后续使用
    return planner


def test_with_multiple_images():
    """测试多张图片"""
    IMAGE_LIST = [
        "data/map_power1.png",
        "data/map_power2.png",
        "data/map_power3.png"
    ]

    for image_path in IMAGE_LIST:
        if not os.path.exists(image_path):
            print(f"\n[跳过] {image_path} 不存在")
            continue

        print(f"\n\n{'='*70}")
        print(f"处理图片: {image_path}")
        print(f"{'='*70}")

        img = Image.open(image_path)
        w, h = img.size

        terrain_raw = TerrainGenerator.generate_realistic_terrain(w, h)

        planner = plan_powerline_independent_lines_v1(
            image_path=image_path,
            terrain_raw=terrain_raw,
            flight_height=25,
            min_pixels=20,
            inspection_spacing=80.0,
            angle_threshold_deg=25.0
        )

        # 简要统计
        if planner.independent_lines:
            print(f"\n[结果] 识别到 {len(planner.independent_lines)} 条独立线路")
            print(f"[结果] 生成 {len(planner.line_inspection_points)} 个巡检点")


if __name__ == "__main__":
    # 单图演示
    planner = main()

    # 多图测试（可选）
    # test_with_multiple_images()
