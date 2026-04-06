"""
成本计算模块

功能：
- 距离成本
- 转向成本
- 风向影响成本
- 点类型权重
- 任务间切换成本
- 路径总成本

设计原则：
- 成本函数通用化，为未来拓扑优化预留扩展
- cost = distance + 0.3 * turn + 0.2 * wind + type_weight + switch_penalty
"""

import numpy as np
from typing import Tuple, List, Dict, Optional


# 点类型权重配置
POINT_TYPE_WEIGHTS = {
    'turning': 1.0,    # 拐点最高优先级
    'endpoint': 0.8,   # 端点次之
    'sample': 0.3      # 采样点最低
}

# 跨线路惩罚配置
LINE_SWITCH_PENALTY = 50.0  # 跨线路固定惩罚


def dist_cost(p1: Tuple[float, ...], p2: Tuple[float, ...]) -> float:
    """
    欧氏距离成本

    Args:
        p1: 点1 (x, y) 或 (x, y, z)
        p2: 点2 (x, y) 或 (x, y, z)

    Returns:
        float: 欧氏距离
    """
    return np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))


def turn_cost(p0: Tuple[float, ...], p1: Tuple[float, ...], p2: Tuple[float, ...]) -> float:
    """
    转向角代价

    Args:
        p0: 前一点
        p1: 当前点
        p2: 下一点

    Returns:
        float: 转向角（弧度）
    """
    v1 = np.array(p0[:2]) - np.array(p1[:2])
    v2 = np.array(p2[:2]) - np.array(p1[:2])

    # 避免零向量
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    if norm1 < 1e-6 or norm2 < 1e-6:
        return 0.0

    # 计算夹角
    cos_angle = np.dot(v1, v2) / (norm1 * norm2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    return angle


def wind_cost(p1: Tuple[float, ...], p2: Tuple[float, ...], wind: Optional[Dict] = None) -> float:
    """
    风向影响成本

    Args:
        p1: 起点
        p2: 终点
        wind: 风向信息 {'direction': 角度(度), 'speed': 速度(m/s)}

    Returns:
        float: 风向额外成本
    """
    if wind is None:
        return 0.0

    wind_dir = np.radians(wind.get('direction', 0))
    wind_speed = wind.get('speed', 0)

    if wind_speed < 0.1:
        return 0.0

    # 飞行方向
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    flight_dir = np.arctan2(dy, dx)

    # 逆风/顺风角度差
    angle_diff = np.abs(flight_dir - wind_dir)
    angle_diff = min(angle_diff, 2 * np.pi - angle_diff)

    # 逆风惩罚：角度差越小（逆风），成本越高
    penalty = wind_speed * (1 + np.cos(angle_diff)) * 0.1

    return penalty


def trans_cost(
    task_a,
    dir_a: int,
    task_b,
    dir_b: int,
    pts_by_line: Dict[str, List],
    wind: Optional[Dict] = None
) -> float:
    """
    任务间切换成本（优化版）

    计算从 task_a（按方向 dir_a）切换到 task_b（按方向 dir_b）的成本。

    成本组成：
    - 距离成本
    - 转向成本（三点夹角）
    - 风向成本
    - 点类型权重（task_b 起点的类型权重）
    - 跨线路惩罚

    Args:
        task_a: 起始任务 (TaskLine)
        dir_a: task_a 的方向 (1=正向, -1=反向)
        task_b: 目标任务 (TaskLine)
        dir_b: task_b 的方向 (1=正向, -1=反向)
        pts_by_line: 按线路分组的巡检点
        wind: 风向信息

    Returns:
        float: 切换成本
    """
    # 获取 task_a 的终点
    if dir_a == 1:
        p_end = task_a.p_end
    else:
        p_end = task_a.p_start

    # 获取 task_b 的起点
    if dir_b == 1:
        p_start = task_b.p_start
    else:
        p_start = task_b.p_end

    # 基础距离成本
    dist = dist_cost(p_end, p_start)

    # 计算转向成本（基于三点夹角）
    turn = 0.0
    if task_a.pts and len(task_a.pts) >= 2:
        # 获取 task_a 的倒数第二个点作为转向计算的前一点
        if dir_a == 1:
            if len(task_a.pts) >= 2:
                p_prev = task_a.pts[-2].position_3d if task_a.pts[-2].position_3d else (
                    task_a.pts[-2].pixel_position[0], task_a.pts[-2].pixel_position[1], 0
                )
            else:
                p_prev = p_end
        else:
            if len(task_a.pts) >= 2:
                p_prev = task_a.pts[1].position_3d if task_a.pts[1].position_3d else (
                    task_a.pts[1].pixel_position[0], task_a.pts[1].pixel_position[1], 0
                )
            else:
                p_prev = p_end

        turn = turn_cost(p_prev, p_end, p_start)

    # 风向成本
    w_cost = wind_cost(p_end, p_start, wind)

    # 点类型权重：task_b 起点的类型权重
    type_weight = 0.0
    if task_b.pts:
        first_pt = task_b.pts[0] if dir_b == 1 else task_b.pts[-1]
        point_type = first_pt.point_type if hasattr(first_pt, 'point_type') else 'sample'
        type_weight = POINT_TYPE_WEIGHTS.get(point_type, 0.3)

    # 跨线路惩罚
    switch_penalty = LINE_SWITCH_PENALTY if task_a.line_id != task_b.line_id else 0.0

    # 综合成本：距离 + 0.5*转向 + 0.2*风向 + 20*类型权重 + 跨线路惩罚
    total = dist + 0.5 * turn + 0.2 * w_cost + 20.0 * type_weight + switch_penalty

    return total


def path_cost(
    path: List[Tuple[float, ...]],
    pts: Optional[List] = None,
    wind: Optional[Dict] = None
) -> float:
    """
    路径总成本（优化版）

    Args:
        path: 点序列 [(x, y, z), ...]
        pts: 巡检点列表（可选，用于获取点类型）
        wind: 风向信息

    Returns:
        float: 总成本
    """
    if len(path) < 2:
        return 0.0

    total = 0.0

    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]

        # 距离
        dist = dist_cost(p1, p2)

        # 转向
        turn = 0.0
        if i > 0:
            turn = turn_cost(path[i - 1], p1, p2)

        # 风向
        w_cost = wind_cost(p1, p2, wind)

        # 点类型权重（p2 的类型权重）
        type_weight = 0.0
        if pts and i < len(pts):
            pt = pts[i]
            point_type = pt.point_type if hasattr(pt, 'point_type') else 'sample'
            type_weight = POINT_TYPE_WEIGHTS.get(point_type, 0.3)

        total += dist + 0.5 * turn + 0.2 * w_cost + 5.0 * type_weight

    return total


def build_cost_mat(
    tasks: List,
    pts_by_line: Dict[str, List],
    wind: Optional[Dict] = None
) -> np.ndarray:
    """
    构建任务间代价矩阵

    Args:
        tasks: 任务列表
        pts_by_line: 按线路分组的巡检点
        wind: 风向信息

    Returns:
        np.ndarray: cost_mat[i,j] = 从 task_i 到 task_j 的成本
    """
    n = len(tasks)
    cost_mat = np.full((n, n), np.inf)

    for i in range(n):
        for j in range(n):
            if i == j:
                cost_mat[i, j] = 0.0
                continue

            # 尝试四个方向组合，取最小
            min_cost = np.inf
            for di in [1, -1]:
                for dj in [1, -1]:
                    c = trans_cost(tasks[i], di, tasks[j], dj, pts_by_line, wind)
                    min_cost = min(min_cost, c)

            cost_mat[i, j] = min_cost

    return cost_mat
