"""
=====================================================
性能分析模块 (Performance Metrics Module)
=====================================================
功能说明:
    该模块提供了路径规划性能评估的各类指标计算函数。
    包括路径长度、转角分析等关键指标。

主要功能:
    - 计算路径总长度（欧几里得距离）
    - 计算路径中相邻段的转角
    - 计算平均转角
    - 转角约束处理
=====================================================
"""

import math
import numpy as np


def compute_path_length(path):
    """
    计算路径的总长度（欧几里得距离）

    该函数遍历路径中的所有相邻点对，计算每段距离并求和。

    参数:
        path: list - 路径点列表，每个元素为 (x, y, z) 元组

    返回:
        float - 路径总长度（栅格单位）
    """
    total = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        dz = path[i][2] - path[i - 1][2]
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def compute_turn_angle(p1, p2, p3):
    """
    计算三点之间的转角

    给定三个连续点 p1 -> p2 -> p3，计算在 p2 处的转角。
    转角定义为向量 (p1->p2) 和 (p2->p3) 之间的夹角。

    参数:
        p1: tuple - 前一个点 (x, y, z)
        p2: tuple - 当前点（转角位置）
        p3: tuple - 下一个点 (x, y, z)

    返回:
        float - 转角（度），范围 [0, 180]
    """
    # 计算两个向量
    v1 = np.array(p2) - np.array(p1)  # p1 -> p2
    v2 = np.array(p3) - np.array(p2)  # p2 -> p3

    # 计算向量模长
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)

    # 处理零向量情况
    if norm1 == 0 or norm2 == 0:
        return 0

    # 计算夹角的余弦值
    cos_angle = np.dot(v1, v2) / (norm1 * norm2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    # 转换为角度（度）
    angle = np.arccos(cos_angle)
    return np.degrees(angle)


def compute_average_turn_angle(path):
    """
    计算路径的平均转角

    该函数计算路径中所有相邻三点的转角，并返回平均值。
    平均转角越小，路径越平滑，对无人机飞行越有利。

    参数:
        path: list - 路径点列表

    返回:
        float - 平均转角（度），如果路径点少于3个返回0
    """
    if len(path) < 3:
        return 0

    angles = []

    # 遍历所有连续的三点组合
    for i in range(1, len(path) - 1):
        v1 = np.array(path[i]) - np.array(path[i - 1])
        v2 = np.array(path[i + 1]) - np.array(path[i])

        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)

        if norm1 == 0 or norm2 == 0:
            continue

        # 计算转角
        cos_angle = np.dot(v1, v2) / (norm1 * norm2)
        cos_angle = np.clip(cos_angle, -1, 1)

        angle = np.degrees(np.arccos(cos_angle))
        angles.append(angle)

    # 返回平均转角
    return np.mean(angles) if angles else 0


def enforce_turn_constraint(path, max_angle=60):
    """
    对路径施加转角约束

    如果路径中某处的转角超过设定的最大值，通过插入中间点来平滑路径。
    这对于无人机等有最大转弯半径限制的载体非常重要。

    参数:
        path: list - 原始路径
        max_angle: float - 允许的最大转角（度），默认60度

    返回:
        list - 满足转角约束的平滑路径
    """
    if len(path) < 3:
        return path

    new_path = [path[0]]

    for i in range(1, len(path) - 1):
        # 计算当前点的转角
        angle = compute_turn_angle(path[i - 1], path[i], path[i + 1])

        if angle <= max_angle:
            # 转角在允许范围内，保留该点
            new_path.append(path[i])
        else:
            # 转角过大，插入中点进行平滑
            # 取前后两点的中点作为新的路径点
            mid = tuple((np.array(path[i - 1]) + np.array(path[i + 1])) // 2)
            new_path.append(mid)

    new_path.append(path[-1])

    return new_path
