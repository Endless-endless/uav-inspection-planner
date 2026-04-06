"""
=====================================================
TSP求解器模块 (TSP Solver Module)
=====================================================
功能说明:
    该模块实现了多目标巡检任务的最优访问顺序求解。
    使用穷举法解决旅行商问题（TSP），适用于目标数量较少的场景。
    同时提供贪心策略（Nearest Neighbor）用于快速模式。

主要功能:
    - 穷举所有可能的访问顺序（最优解）
    - 贪心策略访问顺序（快速解）
    - 计算每种顺序的总路径长度
    - 返回最优访问顺序和路径

适用场景:
    目标数量 ≤ 8 时效果较好（8! = 40320种排列）
    目标数量过多时建议使用启发式算法（如遗传算法、模拟退火等）
    快速模式下使用贪心策略可大幅减少计算时间
=====================================================
"""

import itertools
from planner.astar3d import AStar3D
from analysis.metrics import compute_path_length


def multi_goal_plan_optimal(grid, start, targets, weight=1.0):
    """
    最优多目标路径规划（TSP穷举解法）

    该函数使用穷举法寻找访问所有目标的最优顺序。
    对于n个目标，需要检查n!种排列组合。

    算法流程：
    1. 生成所有目标点的全排列
    2. 对每种排列计算完整路径长度
    3. 选择路径长度最短的排列

    参数:
        grid: GridMap3D - 栅格地图对象
        start: tuple - 起点栅格索引
        targets: list - 目标点栅格索引列表
        weight: float - A*启发式权重

    返回:
        tuple - (best_full_path, best_length, best_order)
            best_full_path: list or None - 最优完整路径
            best_length: float - 最优路径长度
            best_order: tuple - 最优访问顺序
    """
    planner = AStar3D(grid, weight=weight)

    best_order = None
    best_length = float("inf")
    best_full_path = None

    # 枚举所有访问顺序（全排列）
    for perm in itertools.permutations(targets):

        current = start
        total_length = 0
        full_path = []

        valid = True

        # 按当前顺序依次访问每个目标
        for target in perm:
            # 规划从当前点到目标的路径
            path = planner.plan(current, target)

            # 如果找不到路径，该排列不可行
            if not path:
                valid = False
                break

            # 累加路径长度
            length = compute_path_length(path)
            total_length += length

            # 添加路径点（避免重复最后一个点）
            full_path.extend(path[:-1])
            current = target

        # 如果该排列可行且更优，更新最优解
        if valid and total_length < best_length:
            best_length = total_length
            best_order = perm
            best_full_path = full_path

    return best_full_path, best_length, best_order


def multi_goal_plan_greedy(grid, start, targets, weight=1.0):
    """
    多目标路径规划（贪心策略/最近邻算法）

    该函数使用贪心策略解决多目标访问问题：
    每次选择距离当前位置最近的未访问目标作为下一个访问点。
    虽然不能保证全局最优，但计算速度快得多。

    算法复杂度：O(n²)，其中n为目标数量
    适用于快速模式或目标数量较多的场景。

    参数:
        grid: GridMap3D - 栅格地图对象
        start: tuple - 起点栅格索引
        targets: list - 目标点栅格索引列表
        weight: float - A*启发式权重

    返回:
        tuple - (full_path, total_length, visit_order)
            full_path: list - 完整路径点列表
            total_length: float - 总路径长度
            visit_order: tuple - 访问顺序
    """
    planner = AStar3D(grid, weight=weight)

    current = start
    remaining = list(targets)
    full_path = []
    visit_order = []
    total_length = 0

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
        total_length += best_length
        visit_order.append(best_target)
        current = best_target
        remaining.remove(best_target)

    return full_path, total_length, tuple(visit_order)


def create_closed_loop(grid, start, last_target, weight=1.0):
    """
    创建闭环巡检路径（返回起点）

    该函数规划从最后一个巡检点返回起点的路径，
    形成闭环巡检任务。

    参数:
        grid: GridMap3D - 栅格地图对象
        start: tuple - 起点栅格索引
        last_target: tuple - 最后一个目标点栅格索引
        weight: float - A*启发式权重

    返回:
        list or None - 返回路径，如果无路径则返回None
    """
    planner = AStar3D(grid, weight=weight)
    return planner.plan(last_target, start)


def compute_total_path_length_with_return(grid, path, start, weight=1.0):
    """
    计算包含返回段的总路径长度

    参数:
        grid: GridMap3D - 栅格地图对象
        path: list - 巡检路径（不含返回段）
        start: tuple - 起点栅格索引
        weight: float - A*启发式权重

    返回:
        float - 总路径长度（含返回段）
    """
    if not path:
        return 0

    # 计算巡检路径长度
    path_length = compute_path_length(path)

    # 规划返回路径
    return_path = create_closed_loop(grid, start, path[-1], weight)

    if return_path:
        return_length = compute_path_length(return_path)
        return path_length + return_length

    return path_length
