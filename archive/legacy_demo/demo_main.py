"""
=====================================================
电网巡检路径规划演示 V3.0 - 主入口
=====================================================

这是系统的主入口文件，提供：
1. 多图批量处理
2. 完整功能演示
3. 统计信息输出

使用方式：
    python demo/demo_main.py

=====================================================
"""

import numpy as np
import os
import sys
from PIL import Image

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# 使用核心模块
from core.planner import plan_powerline_inspection
from core.terrain import TerrainGenerator


def main():
    """主函数（多图处理）"""
    print("=" * 70)
    print("电网巡检路径规划 V3.0 - 主入口")
    print("=" * 70)

    # 多图处理列表
    IMAGE_LIST = [
        "data/map_power1.png",
        "data/map_power2.png",
        "data/map_power3.png"
    ]

    # 配置参数
    FLIGHT_HEIGHT = 25
    SAMPLE_STEP = 5
    SMOOTH_WINDOW = 5
    TOWER_INTERVAL = 20
    GAUSSIAN_SIGMA = 3.0
    WIND_DIRECTION = 0
    WIND_SPEED = 5

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

        # 使用新的地形生成器
        terrain_raw = TerrainGenerator.generate_realistic_terrain(w, h)

        # 输出文件名
        name = os.path.basename(image_path).split('.')[0]
        output_file = f"figures/powerline_v3_{name}.html"

        # 调用规划函数
        planner = plan_powerline_inspection(
            image_path=image_path,
            terrain_raw=terrain_raw,
            flight_height=FLIGHT_HEIGHT,
            sample_step=SAMPLE_STEP,
            smooth_window=SMOOTH_WINDOW,
            tower_interval=TOWER_INTERVAL,
            gaussian_sigma=GAUSSIAN_SIGMA,
            output_file=output_file,
            wind_direction=WIND_DIRECTION,
            wind_speed=WIND_SPEED
        )

        # =============================
        # 巡检执行演示（新增）
        # =============================
        from core.execution import get_sub_path, simulate_execution

        print("\n" + "="*30)
        print("开始执行巡检模拟")
        print("="*30)

        print("\n[执行演示] 从路径中间开始巡检...")

        # 当前点（模拟）
        path_3d = planner.path_3d
        tasks = planner.tasks

        current_pos = path_3d[len(path_3d)//3][:2]
        print("DEBUG current_pos:", current_pos)

        # 截取子路径
        sub_path, sub_tasks, start_idx = get_sub_path(path_3d, tasks, current_pos)
        print(f"起始索引: {start_idx}")
        
        # 执行模拟
        simulate_execution(sub_path, sub_tasks)

        if planner.stats:
            print(f"\n[统计]")
            print(f"  航点: {planner.stats['waypoint_count']}")
            print(f"  电塔: {planner.stats['tower_count']}")
            print(f"  任务点: {planner.stats.get('task_count', 0)}")
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
