"""
=====================================================
巡检执行逻辑演示 - 从中间位置开始执行
=====================================================

功能：
1. 模拟从路径中任意位置开始执行巡检
2. 自动找到最近的路径点
3. 截取剩余路径和任务
4. 模拟UAV逐步执行巡检

=====================================================
"""

import numpy as np
import os
import sys

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from core.planner import plan_powerline_inspection
from core.terrain import TerrainGenerator
from core.execution import get_sub_path, simulate_execution
from scipy.ndimage import gaussian_filter
from PIL import Image


def main():
    """主函数"""
    print("=" * 70)
    print("巡检执行逻辑演示")
    print("=" * 70)
    print()

    # 1. 生成完整路径
    print("[1/4] 生成完整巡检路径...")
    img = Image.open("data/map_power1.png")
    w, h = img.size

    np.random.seed(0)
    terrain = gaussian_filter(np.random.rand(h, w), sigma=20)
    terrain = ((terrain - terrain.min()) / (terrain.max() - terrain.min()) * 60).astype(np.float32)

    planner = plan_powerline_inspection(
        image_path="data/map_power1.png",
        terrain_raw=terrain,
        flight_height=25,
        sample_step=15,  # 减少采样点以便演示
        smooth_window=5,
        tower_interval=30,
        gaussian_sigma=3.0,
        output_file="figures/execution_demo.html",
        wind_direction=0,
        wind_speed=5
    )

    path_3d = planner.path_3d
    tasks = planner.tasks

    print(f"  完整路径: {len(path_3d)} 个航点")
    print(f"  任务数量: {len(tasks)} 个")
    print()

    # 2. 模拟从路径1/3处开始
    print("[2/4] 模拟从路径1/3处开始执行...")
    start_index = len(path_3d) // 3
    current_pos = (path_3d[start_index][0], path_3d[start_index][1])
    print(f"  当前位置: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
    print(f"  在路径中的位置: 第 {start_index} 点（共 {len(path_3d)} 点）")
    print()

    # 3. 获取子路径
    print("[3/4] 获取剩余子路径...")
    sub_path, sub_tasks, found_idx = get_sub_path(path_3d, tasks, current_pos)
    print(f"  找到最近点索引: {found_idx}")
    print(f"  剩余路径: {len(sub_path)} 个航点")
    print(f"  剩余任务: {len(sub_tasks)} 个")
    print()

    # 4. 模拟执行
    print("[4/4] 模拟巡检执行...")
    print()

    result = simulate_execution(sub_path, sub_tasks)

    print()
    print("=" * 70)
    print("执行统计")
    print("=" * 70)
    print(f"执行步骤: {result['total_steps']}")
    print(f"完成任务: {result['tasks_completed']}")
    print(f"飞行距离: {result['total_distance']:.1f} 米")
    print()


def test_from_start():
    """测试：从起点开始执行"""
    print("=" * 70)
    print("测试：从起点开始执行")
    print("=" * 70)
    print()

    img = Image.open("data/map_power1.png")
    w, h = img.size

    np.random.seed(0)
    terrain = gaussian_filter(np.random.rand(h, w), sigma=20)
    terrain = ((terrain - terrain.min()) / (terrain.max() - terrain.min()) * 60).astype(np.float32)

    planner = plan_powerline_inspection(
        image_path="data/map_power1.png",
        terrain_raw=terrain,
        flight_height=25,
        sample_step=20,
        smooth_window=5,
        tower_interval=30,
        gaussian_sigma=3.0,
        output_file="figures/execution_start_demo.html",
        wind_direction=0,
        wind_speed=5
    )

    path_3d = planner.path_3d
    tasks = planner.tasks

    # 从起点开始
    current_pos = (path_3d[0][0], path_3d[0][1])
    print(f"从起点开始: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
    print()

    sub_path, sub_tasks, found_idx = get_sub_path(path_3d, tasks, current_pos)

    result = simulate_execution(sub_path, sub_tasks)

    return result


if __name__ == "__main__":
    main()

    print()
    print("=" * 70)
    print("额外测试：从起点开始")
    print("=" * 70)
    test_from_start()
