"""
=====================================================
A* 优化效果测试
=====================================================
"""

import numpy as np
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment.heightmap_loader import load_from_array
from planner.astar3d import AStar3D

def create_test_map(size=50):
    """创建测试地图"""
    x = np.linspace(-5, 5, size)
    y = np.linspace(-5, 5, size)
    X, Y = np.meshgrid(x, y)

    # 生成简单地形
    height_map = 20 * np.exp(-(X**2 + Y**2) / 10)
    height_map = np.maximum(height_map, 0)

    return height_map

def test_astar_performance():
    """测试A*性能"""
    print("=" * 60)
    print("A* 优化效果测试")
    print("=" * 60)

    # 创建测试地图
    print("\n[1/4] 创建测试地图...")
    height_map = create_test_map(50)

    # 创建3D地图
    print("[2/4] 构建3D栅格地图...")
    grid = load_from_array(
        height_array=height_map,
        resolution=1.0,
        ground_clearance=1.0,
        flight_height=30
    )

    # 设置起点和终点
    start = (5, 5, 20)
    goal = (45, 45, 25)

    print(f"   地图尺寸: {grid.width} x {grid.height} x {grid.depth}")
    print(f"   起点: {start}")
    print(f"   终点: {goal}")

    # 测试优化版本（使用heapq）
    print("\n[3/4] 测试优化版本 (heapq)...")
    planner_opt = AStar3D(grid, use_heapq=True, weight=1.2)

    start_time = time.time()
    path_opt = planner_opt.plan(start, goal)
    opt_time = time.time() - start_time

    if path_opt:
        print(f"   [OK] 路径长度: {len(path_opt)} 点")
        print(f"   耗时: {opt_time:.3f} 秒")
        print(f"   扩展节点: {planner_opt.expanded_nodes}")
    else:
        print(f"   [FAIL] 规划失败")
        opt_time = 0

    # 测试原始版本（不使用heapq）
    print("\n[4/4] 测试原始版本 (dict+min)...")
    planner_std = AStar3D(grid, use_heapq=False, weight=1.2)

    start_time = time.time()
    path_std = planner_std.plan(start, goal)
    std_time = time.time() - start_time

    if path_std:
        print(f"   [OK] 路径长度: {len(path_std)} 点")
        print(f"   耗时: {std_time:.3f} 秒")
        print(f"   扩展节点: {planner_std.expanded_nodes}")
    else:
        print(f"   [FAIL] 规划失败")
        std_time = 0

    # 性能对比
    print("\n" + "=" * 60)
    print("性能对比")
    print("=" * 60)

    if opt_time > 0 and std_time > 0:
        speedup = std_time / opt_time
        node_reduction = planner_std.expanded_nodes / planner_opt.expanded_nodes if planner_opt.expanded_nodes > 0 else 0

        print(f"原始版本耗时:     {std_time:.3f} 秒")
        print(f"优化版本耗时:     {opt_time:.3f} 秒")
        print(f"加速比:           {speedup:.1f}x")
        print(f"节点减少:         {node_reduction:.1f}x")
        print(f"原始版本扩展节点: {planner_std.expanded_nodes}")
        print(f"优化版本扩展节点: {planner_opt.expanded_nodes}")
    else:
        print("无法计算加速比（某版本规划失败）")

    print("=" * 60)

if __name__ == "__main__":
    test_astar_performance()
