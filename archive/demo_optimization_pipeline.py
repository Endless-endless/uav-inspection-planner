"""
完整的优化闭环演示脚本

流程：
1. 生成/加载3D路径
2. 应用路径后处理优化（风+能耗）
3. 创建动画视图（仅优化路径）
4. 创建对比分析视图（原始vs优化）
5. 输出到output/目录
"""

import numpy as np
import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.planner import plan_powerline_inspection
from core.path_optimizer import optimize_path, compute_total_cost
from core.visualization_enhanced import create_animation_view, create_simple_comparison
from scipy.ndimage import gaussian_filter
from PIL import Image


def main():
    """完整的优化闭环演示"""
    print("=" * 70)
    print("         UAV路径优化闭环系统 - 完整演示")
    print("=" * 70)

    # =====================================================
    # 第1步：生成基础路径
    # =====================================================
    print("\n[第1步] 生成基础路径...")

    # 加载图像
    image_path = "data/map_power1.png"
    if not os.path.exists(image_path):
        print(f"  [错误] 图像不存在: {image_path}")
        print("  使用模拟路径代替...")
        # 创建模拟路径
        base_path = []
        for x in range(0, 200, 20):
            y = x * 0.3
            z = 50 + np.sin(x / 30) * 5
            base_path.append((x, y, z))
        path_3d = base_path
    else:
        # 使用真实图像生成路径
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
            sample_step=30,  # 稀疏采样以便观察
            smooth_window=3,
            tower_interval=50,
            gaussian_sigma=2.0,
            output_file="temp.html",  # 临时文件
            wind_direction=0,   # 东风
            wind_speed=6.0
        )

        path_3d = planner.path_3d_smooth

    print(f"  路径点数: {len(path_3d)}")
    print(f"  起点: {path_3d[0]}")
    print(f"  终点: {path_3d[-1]}")

    # =====================================================
    # 第2步：应用路径后处理优化
    # =====================================================
    print("\n[第2步] 应用路径后处理优化（风+能耗）...")

    # 设置风况（东风6m/s，逆风）
    wind_direction = 0   # 东风
    wind_speed = 6.0     # 6m/s

    # 风向量
    wind_rad = np.radians(wind_direction)
    wind_vector = np.array([
        wind_speed * np.cos(wind_rad),
        wind_speed * np.sin(wind_rad)
    ])

    print(f"  风况: {wind_speed}m/s, 方向: {wind_direction}° (东风)")

    # 原始成本
    original_cost = compute_total_cost(path_3d, wind_vector)
    print(f"  原始总成本: {original_cost['total_cost']:.2f}")

    # 执行优化
    optimized_path = optimize_path(
        path_3d,
        wind_direction=wind_direction,
        wind_speed=wind_speed,
        max_shift=2.0,  # 最大偏移2米
        wind_alpha=0.5,
        climb_factor=2.0,
        cost_threshold_factor=1.15
    )

    # 优化后成本
    optimized_cost = compute_total_cost(optimized_path, wind_vector)
    print(f"  优化后成本: {optimized_cost['total_cost']:.2f}")

    # 计算改善
    improvement = original_cost['total_cost'] - optimized_cost['total_cost']
    improvement_pct = (improvement / original_cost['total_cost']) * 100
    print(f"  成本降低: {improvement:.2f} ({improvement_pct:.1f}%)")

    # 计算偏移统计
    offsets = [np.linalg.norm(np.array(path_3d[i]) - np.array(optimized_path[i]))
               for i in range(len(path_3d))]
    print(f"  平均偏移: {np.mean(offsets):.2f}m")
    print(f"  最大偏移: {max(offsets):.2f}m")

    # =====================================================
    # 第3步：创建动画视图（仅优化路径）
    # =====================================================
    print("\n[第3步] 创建动画视图（优化路径UAV飞行）...")

    animation_file = "output/animation.html"
    create_animation_view(optimized_path, output_file=animation_file)
    print(f"  [动画] {len(optimized_path)} 帧")
    print(f"  [保存] {animation_file}")

    # =====================================================
    # 第4步：创建对比分析视图
    # =====================================================
    print("\n[第4步] 创建对比分析视图（原始vs优化）...")

    comparison_file = "output/comparison.html"
    result = create_simple_comparison(
        path_3d,
        optimized_path,
        wind_vector,
        output_file=comparison_file
    )
    print(f"  [保存] {comparison_file}")
    print(f"  [改善] {result['improvement_pct']:.1f}%")

    # =====================================================
    # 完成
    # =====================================================
    print("\n" + "=" * 70)
    print("                     完成！")
    print("=" * 70)
    print(f"\n[输出文件]")
    print(f"  - {animation_file}")
    print(f"  - {comparison_file}")
    print(f"\n[优化效果]")
    print(f"  原始成本: {original_cost['total_cost']:.2f}")
    print(f"  优化成本: {optimized_cost['total_cost']:.2f}")
    print(f"  改善: {improvement:.2f} ({improvement_pct:.1f}%)")
    print(f"\n[提示]")
    print(f"  1. 打开 {animation_file} 查看UAV飞行动画")
    print(f"  2. 打开 {comparison_file} 查看路径对比分析")
    print(f"  3. 红色段 = 高成本段（优化的主要目标）")
    print(f"  4. 绿色路径 = 优化后的路径")


if __name__ == "__main__":
    main()
