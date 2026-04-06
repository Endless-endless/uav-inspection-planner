"""
=====================================================
电网巡检路径规划演示 V3.0 - 多图处理版
=====================================================

功能：
1. 支持多张电网图批量处理
2. 路径平滑
3. 电塔节点标记
4. 3D地形可视化

=====================================================
"""

import numpy as np
import os
import sys
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from planner.powerline_planner_v3_final import plan_powerline_inspection_v3


def generate_realistic_terrain(width, height):
    """生成真实感地形"""
    print("\n[地形] 生成真实感地形...")
    from scipy.ndimage import gaussian_filter

    np.random.seed(0)
    terrain = np.random.rand(height, width)
    terrain = gaussian_filter(terrain, sigma=20)
    terrain = (terrain - terrain.min()) / (terrain.max() - terrain.min()) * 60

    print(f"  地形尺寸: {width}x{height}")
    print(f"  高度范围: [{terrain.min():.1f}, {terrain.max():.1f}] 米")

    return terrain.astype(np.float32)


def main():
    """主函数（多图处理）"""
    print("=" * 70)
    print("电网巡检路径规划 V3.0 - 多图处理版")
    print("=" * 70)

    # 多图处理列表
    IMAGE_LIST = [
        "data/map_power1.png",
        "data/map_power2.png",
        "data/map_power3.png"
    ]

    FLIGHT_HEIGHT = 25
    SAMPLE_STEP = 5
    SMOOTH_WINDOW = 5
    TOWER_INTERVAL = 20
    GAUSSIAN_SIGMA = 3.0

    for image_path in IMAGE_LIST:
        print("\n" + "=" * 70)
        print(f"处理图片: {image_path}")
        print("=" * 70)

        if not os.path.exists(image_path):
            print(f"  [WARN] 文件不存在，跳过")
            continue

        img = Image.open(image_path)
        w, h = img.size
        print(f"  图片尺寸: {w}x{h}")

        terrain_raw = generate_realistic_terrain(w, h)

        # 输出文件名
        name = os.path.basename(image_path).split('.')[0]
        output_file = f"figures/powerline_v3_{name}.html"

        planner = plan_powerline_inspection_v3(
            image_path=image_path,
            terrain_raw=terrain_raw,
            flight_height=FLIGHT_HEIGHT,
            sample_step=SAMPLE_STEP,
            smooth_window=SMOOTH_WINDOW,
            tower_interval=TOWER_INTERVAL,
            gaussian_sigma=GAUSSIAN_SIGMA,
            output_file=output_file
        )

        if planner.stats:
            print(f"\n[统计]")
            print(f"  航点: {planner.stats['waypoint_count']}")
            print(f"  电塔: {planner.stats['tower_count']}")
            print(f"  路径长度: {planner.stats['path_length']:.1f} 米")
            print(f"  输出: {output_file}")

    print("\n" + "=" * 70)
    print("处理完成！")
    print("=" * 70)

    print("\n[输出文件]")
    for image_path in IMAGE_LIST:
        name = os.path.basename(image_path).split('.')[0]
        print(f"  - figures/powerline_v3_{name}.html")


if __name__ == "__main__":
    main()
