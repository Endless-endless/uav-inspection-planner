"""
=====================================================
多目标任务模块 (Multi-Goal Mission Module)
=====================================================
功能说明:
    该模块实现了多目标巡检任务的路径规划功能。
    包括贪心策略的多目标访问和路径稀疏化处理。

主要功能:
    - 多目标贪心路径规划（最近邻策略）
    - 路径稀疏化（移除冗余点）
    - 坐标转换工具
    - 射线追踪算法（用于路径碰撞检测）
=====================================================
"""

import numpy as np
from planner.astar3d import AStar3D
from analysis.metrics import compute_path_length


def coord_to_index(coord, grid):
    """
    将物理坐标转换为栅格索引

    参数:
        coord: tuple - 物理坐标 (x, y, z)，单位：米
        grid: GridMap3D - 栅格地图对象

    返回:
        tuple - 栅格索引 (ix, iy, iz)
    """
    x = int((coord[0] - grid.b_min[0]) / grid.resolution)
    y = int((coord[1] - grid.b_min[1]) / grid.resolution)
    z = int((coord[2] - grid.b_min[2]) / grid.resolution)
    return (x, y, z)


def find_valid_inspection_point(coord, grid, search_radius=10):
    """
    查找有效的巡检点（确保不在障碍物内）

    如果指定的坐标对应的栅格单元是障碍物或越界，
    该函数会在附近搜索一个有效的自由空间栅格单元。

    搜索策略：螺旋式向外搜索，从最近的位置开始

    参数:
        coord: tuple - 物理坐标 (x, y, z)，单位：米
        grid: GridMap3D - 栅格地图对象
        search_radius: int - 搜索半径（栅格数），默认为10

    返回:
        tuple - 有效的栅格索引 (ix, iy, iz)，
                如果找不到有效点则返回None
    """
    # 首先检查原始坐标是否有效
    original_index = coord_to_index(coord, grid)
    if grid.is_valid(original_index):
        return original_index

    # 原始坐标无效，在附近搜索有效点
    # 使用螺旋式搜索策略
    ox, oy, oz = original_index

    # 从最近的位置开始逐步扩大搜索范围
    for radius in range(1, search_radius + 1):
        # 在当前半径的球形边界上搜索
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    # 跳过已经检查过的内层点
                    if max(abs(dx), abs(dy), abs(dz)) != radius:
                        continue

                    neighbor = (ox + dx, oy + dy, oz + dz)

                    # 检查是否是有效且可通行的栅格单元
                    if grid.is_valid(neighbor):
                        return neighbor

    # 搜索半径内未找到有效点
    return None


def raytrace(start, end):
    """
    3D射线追踪算法（Bresenham 3D直线算法）

    该算法生成从起点到终点的所有栅格点，用于检测直线路径是否与障碍物碰撞。
    使用3D Bresenham直线算法的简化版本。

    参数:
        start: tuple - 起点 (x, y, z)
        end: tuple - 终点 (x, y, z)

    返回:
        list - 从起点到终点的所有栅格点列表
    """
    points = []
    x1, y1, z1 = start
    x2, y2, z2 = end

    # 计算各方向的差值绝对值
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)

    # 确定采样步数（取最大差值）
    n = max(dx, dy, dz)

    # 沿直线均匀采样
    for i in range(n + 1):
        t = i / n if n != 0 else 0
        x = round(x1 + (x2 - x1) * t)
        y = round(y1 + (y2 - y1) * t)
        z = round(z1 + (z2 - z1) * t)
        points.append((x, y, z))

    return points


def sparsen_path(grid, path):
    """
    路径稀疏化：移除路径中的冗余点

    该算法检查路径中的每个点，如果从前一个关键点到当前点的直线路径无障碍，
    则中间的路径点可以移除。这可以显著减少路径点数量，便于飞行控制。

    算法思路：
    1. 从起点开始，尝试跳过中间点直接连接到更远的点
    2. 使用射线追踪检查直线路径是否与障碍物碰撞
    3. 如果碰撞，保留碰撞前的点作为关键点
    4. 重复直到终点

    参数:
        grid: GridMap3D - 栅格地图对象
        path: list - 原始路径点列表

    返回:
        list - 稀疏化后的路径点列表
    """
    if not path:
        return path

    # 第一个点总是保留
    new_path = [path[0]]
    last_index = 0

    # 遍历路径中的每个点（从第3个点开始检查）
    for i in range(2, len(path)):
        blocked = False

        # 检查从上一个关键点到当前点的直线路径
        ray = raytrace(path[last_index], path[i])

        # 检查射线上的每个点是否可通行
        for cell in ray:
            if not grid.is_valid(cell):
                blocked = True
                break

        # 如果直线路径被阻挡，保留前一个点作为关键点
        if blocked:
            new_path.append(path[i - 1])
            last_index = i - 1

    # 最后一个点总是保留
    new_path.append(path[-1])
    return new_path


def multi_goal_plan(grid, start, targets, weight=1.0):
    """
    多目标贪心路径规划（最近邻策略）

    该函数使用贪心策略解决多目标访问问题：
    每次选择距离当前位置最近的未访问目标作为下一个访问点。

    算法复杂度：O(n²)，其中n为目标数量

    参数:
        grid: GridMap3D - 栅格地图对象
        start: tuple - 起点栅格索引
        targets: list - 目标点栅格索引列表
        weight: float - A*启发式权重

    返回:
        list - 连接所有目标的完整路径点列表
    """
    planner = AStar3D(grid, weight=weight)

    current = start
    remaining = targets.copy()
    full_path = []

    # 循环访问所有剩余目标
    while remaining:
        best_target = None
        best_path = None
        best_length = float("inf")

        # 遍历所有剩余目标，找最近的
        for target in remaining:
            path = planner.plan(current, target)
            if path:
                length = compute_path_length(path)
                if length < best_length:
                    best_length = length
                    best_target = target
                    best_path = path

        # 如果找不到可行路径，提前退出
        if best_target is None:
            break

        # 将路径添加到完整路径（避免重复最后一个点）
        full_path.extend(best_path[:-1])
        current = best_target
        remaining.remove(best_target)

    return full_path
