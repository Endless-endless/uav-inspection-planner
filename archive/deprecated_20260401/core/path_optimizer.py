"""
路径后处理优化模块

功能：基于风和能耗对已有路径进行局部微调
特点：不重规划，只做启发式局部优化
"""

import numpy as np


def compute_segment_cost(p1, p2, wind_vector, wind_alpha=0.5, climb_factor=2.0):
    """
    计算单段路径的成本（风影响 + 能耗）

    Args:
        p1: 起点 (x, y, z)
        p2: 终点 (x, y, z)
        wind_vector: 风向量 [wx, wy]
        wind_alpha: 风影响系数
        climb_factor: 爬升能耗系数

    Returns:
        float: 总成本
    """
    p1 = np.array(p1)
    p2 = np.array(p2)

    # 路径方向向量
    direction = p2 - p1
    distance = np.linalg.norm(direction)

    if distance < 0.001:
        return 0.0

    # 归一化方向
    direction_norm = direction / distance

    # 只考虑水平方向的风影响（2D）
    direction_2d = direction_norm[:2]
    wind_2d = wind_vector / (np.linalg.norm(wind_vector) + 1e-6)

    # 计算风影响：cosθ
    # 顺风（cosθ > 0）: 风帮助飞行，降低成本
    # 逆风（cosθ < 0）: 风阻碍飞行，增加成本
    cos_theta = np.dot(direction_2d, wind_2d)

    # 风成本：逆风惩罚，顺风奖励
    # cosθ = 1 (完全顺风) -> wind_penalty = 1 - alpha = 降低成本
    # cosθ = -1 (完全逆风) -> wind_penalty = 1 + alpha = 增加成本
    wind_penalty = 1 - wind_alpha * cos_theta

    # 爬升高度
    climb = p2[2] - p1[2]

    # 总成本 = 距离 * (1 + 风影响) + 爬升能耗
    cost = distance * wind_penalty + climb_factor * max(0, climb)

    return cost


def compute_total_cost(path_3d, wind_vector, wind_alpha=0.5, climb_factor=2.0):
    """
    计算整条路径的总成本

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        wind_vector: 风向量 [wx, wy]
        wind_alpha: 风影响系数
        climb_factor: 爬升能耗系数

    Returns:
        dict: {total_cost, segment_costs, avg_cost, max_cost}
    """
    if len(path_3d) < 2:
        return {
            'total_cost': 0.0,
            'segment_costs': [],
            'avg_cost': 0.0,
            'max_cost': 0.0
        }

    segment_costs = []
    for i in range(len(path_3d) - 1):
        cost = compute_segment_cost(
            path_3d[i],
            path_3d[i + 1],
            wind_vector,
            wind_alpha,
            climb_factor
        )
        segment_costs.append(cost)

    total_cost = sum(segment_costs)

    return {
        'total_cost': total_cost,
        'segment_costs': segment_costs,
        'avg_cost': total_cost / len(segment_costs),
        'max_cost': max(segment_costs) if segment_costs else 0.0
    }


def optimize_path_with_wind_energy(path_3d, wind_vector,
                                   max_shift=2.0,
                                   wind_alpha=0.5,
                                   climb_factor=2.0,
                                   cost_threshold_factor=1.2):
    """
    基于风和能耗的路径后处理优化

    核心思路：
    1. 计算每段路径的成本
    2. 识别高成本段（高于平均值的段）
    3. 对高成本段的中间点做局部微调

    优化策略：
    - 逆风段：尝试向垂直于风向的方向偏移（减少逆风分量）
    - 高爬升段：尝试降低高度（减少爬升能耗）

    Args:
        path_3d: 原始3D路径 [(x, y, z), ...]
        wind_vector: 风向量 [wx, wy]
        max_shift: 最大偏移距离（米）
        wind_alpha: 风影响系数
        climb_factor: 爬升能耗系数
        cost_threshold_factor: 成本阈值系数（高成本判定）

    Returns:
        list: 优化后的3D路径 [(x, y, z), ...]
    """
    if len(path_3d) < 3:
        return path_3d.copy()

    # 转换为numpy数组便于计算
    original_path = np.array(path_3d, dtype=float)
    optimized_path = original_path.copy()

    # 计算原始路径成本
    cost_info = compute_total_cost(path_3d, wind_vector, wind_alpha, climb_factor)
    segment_costs = cost_info['segment_costs']
    avg_cost = cost_info['avg_cost']

    # 高成本阈值：高于平均值 20%
    high_cost_threshold = avg_cost * cost_threshold_factor

    # 识别高成本段的索引
    high_cost_segments = []
    for i, cost in enumerate(segment_costs):
        if cost > high_cost_threshold:
            high_cost_segments.append(i)

    if not high_cost_segments:
        return optimized_path.tolist()

    print(f"[优化] 检测到 {len(high_cost_segments)} 个高成本段（阈值: {high_cost_threshold:.2f}）")

    # 对每个高成本段进行局部优化
    total_adjustments = 0
    total_cost_reduction = 0.0

    # 扩大优化范围：不仅优化高成本段，也优化其相邻点
    optimize_indices = set()
    for seg_idx in high_cost_segments:
        optimize_indices.add(seg_idx)
        optimize_indices.add(seg_idx - 1)
        optimize_indices.add(seg_idx + 1)

    # 过滤掉边界点
    optimize_indices = [i for i in optimize_indices if 0 < i < len(original_path) - 1]

    print(f"[优化] 计划优化 {len(optimize_indices)} 个点")

    for i in sorted(optimize_indices):
        # 获取当前点及其前后点
        p_prev = original_path[i - 1]
        p_curr = original_path[i]
        p_next = original_path[i + 1]

        # 计算前后两段的原始成本
        old_cost_prev = segment_costs[i - 1]
        old_cost_curr = segment_costs[i]
        old_total = old_cost_prev + old_cost_curr

        # 尝试风优化：检查该点周围的风向
        for direction_factor in [1, -1]:
            old_point = optimized_path[i].copy()

            # 计算侧向偏移（垂直于风向）
            wind_2d = wind_vector / (np.linalg.norm(wind_vector) + 1e-6)
            wind_perp = np.array([-wind_2d[1], wind_2d[0]])  # 垂直向量

            # 偏移
            shift = wind_perp * max_shift * 0.6 * direction_factor
            optimized_path[i][:2] += shift

            # 限制偏移
            actual_shift = np.linalg.norm(optimized_path[i][:2] - old_point[:2])
            if actual_shift > max_shift:
                scale = max_shift / actual_shift
                optimized_path[i][:2] = old_point[:2] + (optimized_path[i][:2] - old_point[:2]) * scale

            # 计算新成本
            new_cost_prev = compute_segment_cost(optimized_path[i - 1], optimized_path[i], wind_vector, wind_alpha, climb_factor)
            new_cost_curr = compute_segment_cost(optimized_path[i], optimized_path[i + 1], wind_vector, wind_alpha, climb_factor)
            new_total = new_cost_prev + new_cost_curr

            if new_total < old_total - 0.5:  # 至少改善0.5
                total_cost_reduction += (old_total - new_total)
                total_adjustments += 1
                break  # 保留改善，停止尝试另一个方向
            else:
                optimized_path[i] = old_point  # 回退

        # 尝试爬升优化
        p_prev_orig = original_path[i - 1]
        p_curr_orig = original_path[i]
        p_next_orig = original_path[i + 1]

        # 检查是否有爬升
        climb_before = p_curr_orig[2] - p_prev_orig[2]
        climb_after = p_next_orig[2] - p_curr_orig[2]

        if climb_before > 2.0 or climb_after > 2.0:  # 有明显爬升
            old_point = optimized_path[i].copy()

            # 尝试降低高度
            climb_reduction = min(max_shift * 0.4, max(climb_before, climb_after) * 0.25)
            optimized_path[i][2] -= climb_reduction

            # 计算新成本
            new_cost_prev = compute_segment_cost(optimized_path[i - 1], optimized_path[i], wind_vector, wind_alpha, climb_factor)
            new_cost_curr = compute_segment_cost(optimized_path[i], optimized_path[i + 1], wind_vector, wind_alpha, climb_factor)
            new_total = new_cost_prev + new_cost_curr

            if new_total < old_total - 0.3:  # 至少改善0.3
                total_cost_reduction += (old_total - new_total)
                total_adjustments += 1
            else:
                optimized_path[i][2] = old_point[2]  # 回退高度

    # 计算优化后总成本
    final_cost_info = compute_total_cost(optimized_path.tolist(), wind_vector, wind_alpha, climb_factor)
    final_cost = final_cost_info['total_cost']
    original_total = cost_info['total_cost']

    print(f"[优化] 完成!")
    print(f"[优化] 原始总成本: {original_total:.2f}")
    print(f"[优化] 优化后成本: {final_cost:.2f}")
    print(f"[优化] 成本降低: {original_total - final_cost:.2f} ({(original_total - final_cost) / original_total * 100:.1f}%)")
    print(f"[优化] 调整点数: {total_adjustments}")

    return optimized_path.tolist()


# 便捷函数包装
def optimize_path(path_3d, wind_direction=0, wind_speed=5.0, **kwargs):
    """
    便捷的路径优化函数

    Args:
        path_3d: 3D路径
        wind_direction: 风向（角度，0=东，90=北）
        wind_speed: 风速（m/s）
        **kwargs: 其他参数传递给optimize_path_with_wind_energy

    Returns:
        优化后的路径
    """
    # 将风向角度转换为向量
    wind_rad = np.radians(wind_direction)
    wind_vector = np.array([
        wind_speed * np.cos(wind_rad),
        wind_speed * np.sin(wind_rad)
    ])

    return optimize_path_with_wind_energy(path_3d, wind_vector, **kwargs)
