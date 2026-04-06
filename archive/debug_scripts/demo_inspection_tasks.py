"""
完整的巡检任务节点标记系统演示

流程：
1. 生成/加载3D路径
2. 生成巡检任务点序列
3. 创建带任务点的动画
4. 创建带任务点的对比分析
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.planner import plan_powerline_inspection
from core.path_optimizer import optimize_path, compute_total_cost
from core.inspection_tasks import (
    generate_inspection_points,
    filter_photo_points,
    filter_high_priority,
    get_task_summary
)
from core.visualization_enhanced import (
    create_animation_view_with_tasks,
    create_analysis_view_with_tasks
)
from scipy.ndimage import gaussian_filter
from PIL import Image


def main():
    """完整的巡检任务系统演示"""
    print("=" * 70)
    print("      UAV巡检任务节点标记系统 - 完整演示")
    print("=" * 70)

    # =====================================================
    # 第1步：生成基础路径
    # =====================================================
    print("\n[第1步] 生成基础路径...")

    image_path = "data/map_power1.png"

    if os.path.exists(image_path):
        img = Image.open(image_path)
        w, h = img.size

        # 生成地形
        np.random.seed(42)
        terrain = gaussian_filter(np.random.rand(h, w), sigma=20)
        terrain = ((terrain - terrain.min()) / (terrain.max() - terrain.min()) * 60).astype(np.float32)

        # 规划路径
        planner = plan_powerline_inspection(
            image_path=image_path,
            terrain_raw=terrain,
            flight_height=25,
            sample_step=30,
            smooth_window=3,
            tower_interval=50,
            gaussian_sigma=2.0,
            output_file="temp.html",
            wind_direction=0,
            wind_speed=6.0
        )

        path_3d = planner.path_3d_smooth
    else:
        # 模拟路径
        print("  [模拟] 创建测试路径...")
        path_3d = []
        for i in range(0, 200, 10):
            x = i
            y = i * 0.3
            z = 50 + np.sin(i / 30) * 8
            path_3d.append((x, y, z))

    print(f"  路径点数: {len(path_3d)}")

    # =====================================================
    # 第2步：应用路径优化
    # =====================================================
    print("\n[第2步] 应用路径后处理优化...")

    wind_direction = 0
    wind_speed = 6.0
    wind_rad = np.radians(wind_direction)
    wind_vector = np.array([
        wind_speed * np.cos(wind_rad),
        wind_speed * np.sin(wind_rad)
    ])

    original_cost = compute_total_cost(path_3d, wind_vector)
    print(f"  原始成本: {original_cost['total_cost']:.2f}")

    optimized_path = optimize_path(
        path_3d,
        wind_direction=wind_direction,
        wind_speed=wind_speed,
        max_shift=2.0
    )

    optimized_cost = compute_total_cost(optimized_path, wind_vector)
    print(f"  优化成本: {optimized_cost['total_cost']:.2f}")
    print(f"  改善: {original_cost['total_cost'] - optimized_cost['total_cost']:.2f}")

    # =====================================================
    # 第3步：生成巡检任务点
    # =====================================================
    print("\n[第3步] 生成巡检任务点序列...")

    inspection_points = generate_inspection_points(
        optimized_path,
        wind_vector,
        photo_interval=5,           # 每5个点拍照一次
        height_change_threshold=3.0, # 高度变化3米标记
        high_cost_factor=0.5        # 高成本系数
    )

    # 任务统计
    summary = get_task_summary(inspection_points)
    print(f"\n  [任务统计]")
    print(f"  总任务点: {summary['total_points']}")
    print(f"  拍照点: {summary['photo_points']}")
    print(f"  高优先级: {summary['high_priority']}")
    print(f"  总成本: {summary['total_cost']:.2f}")
    print(f"  平均云台角: {summary['avg_gimbal_angle']:.1f}°")

    # 显示部分任务点详情
    print(f"\n  [任务点示例（前5个）]")
    for i, point in enumerate(inspection_points[:5]):
        print(f"  点{point.index}: {point.action} | {point.priority} | {point.reason}")

    # =====================================================
    # 第4步：创建带任务点的动画
    # =====================================================
    print("\n[第4步] 创建带任务点的UAV飞行动画...")

    animation_file = "output/animation_with_tasks.html"
    create_animation_view_with_tasks(
        optimized_path,
        inspection_points,
        output_file=animation_file
    )
    print(f"  [保存] {animation_file}")

    # =====================================================
    # 第5步：创建带任务点的对比分析
    # =====================================================
    print("\n[第5步] 创建带任务点的路径对比分析...")

    comparison_file = "output/comparison_with_tasks.html"
    result = create_analysis_view_with_tasks(
        path_3d,
        optimized_path,
        wind_vector,
        inspection_points,
        output_file=comparison_file
    )
    print(f"  [保存] {comparison_file}")

    # =====================================================
    # 完成
    # =====================================================
    print("\n" + "=" * 70)
    print("                     完成！")
    print("=" * 70)

    print(f"\n[输出文件]")
    print(f"  - {animation_file}")
    print(f"  - {comparison_file}")

    print(f"\n[任务摘要]")
    print(f"  总点数: {summary['total_points']}")
    print(f"  拍照点: {summary['photo_points']}")
    print(f"  高优先级: {summary['high_priority']}")

    print(f"\n[优化效果]")
    print(f"  原始成本: {original_cost['total_cost']:.2f}")
    print(f"  优化成本: {optimized_cost['total_cost']:.2f}")
    print(f"  改善: {original_cost['total_cost'] - optimized_cost['total_cost']:.2f}")

    print(f"\n[提示]")
    print(f"  1. 打开 {animation_file} 查看UAV飞行+任务点动画")
    print(f"  2. 打开 {comparison_file} 查看路径+任务点对比分析")
    print(f"  3. 黄色点 = 普通拍照点")
    print(f"  4. 红色点 = 高优先级点")
    print(f"  5. 动画中UAV在拍照点会变色显示")


if __name__ == "__main__":
    main()
