"""
巡检任务节点标记系统

功能：
1. 将path_3d转换为任务节点序列
2. 标记拍照点和关键位置
3. 支持任务可视化
"""

import numpy as np
from dataclasses import dataclass
from typing import List
from core.path_optimizer import compute_total_cost


@dataclass
class InspectionPoint:
    """
    巡检任务点

    Attributes:
        position: 3D位置 (x, y, z)
        index: 在路径中的索引
        is_photo: 是否为拍照点
        priority: 优先级 ("normal" / "high")
        gimbal_angle: 云台角度（度，向下为负）
        action: 动作类型 ("fly" / "photo")
        cost: 到达该点的累计成本
        reason: 标记原因（用于可解释性）
    """
    position: tuple
    index: int
    is_photo: bool
    priority: str
    gimbal_angle: float
    action: str
    cost: float
    reason: str


def generate_inspection_points(path_3d, wind_vector,
                               photo_interval=8,  # 改为8，更稀疏
                               height_change_threshold=3.0,
                               high_cost_factor=0.5):
    """
    生成巡检任务点序列

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        wind_vector: 风向量 [wx, wy]
        photo_interval: 拍照间隔（点数）
        height_change_threshold: 高度变化阈值（米）
        high_cost_factor: 高成本系数（用于判定高优先级）

    Returns:
        List[InspectionPoint]: 任务点列表
    """
    print("[任务生成] 生成巡检任务点序列...")

    if len(path_3d) < 2:
        return []

    # 计算路径成本
    cost_info = compute_total_cost(path_3d, wind_vector)
    segment_costs = cost_info['segment_costs']

    avg_cost = cost_info['avg_cost']
    std_cost = np.std(segment_costs) if len(segment_costs) > 1 else 0

    # 使用百分位数阈值代替标准差（更适合平坦路径）
    sorted_costs = np.sort(segment_costs)
    percentile_idx = int(len(sorted_costs) * (1 - 0.2))  # top 20%
    high_cost_threshold = sorted_costs[percentile_idx] if len(sorted_costs) > 0 else avg_cost

    print(f"  平均段成本: {avg_cost:.2f}")
    print(f"  高成本阈值 (top 20%): {high_cost_threshold:.2f}")

    # 生成任务点
    inspection_points = []
    cumulative_cost = 0.0

    for i, pos in enumerate(path_3d):
        x, y, z = pos

        # 累计成本
        if i > 0:
            cumulative_cost += segment_costs[i - 1]

        # 判断任务类型
        is_photo = False
        priority = "normal"
        gimbal_angle = -30  # 默认云台角度
        action = "fly"
        reason = "normal_flight"

        # 规则1: 等间距拍照
        if i % photo_interval == 0 and i < len(path_3d) - 1:
            is_photo = True
            action = "photo"
            reason = f"periodic_photo_{i}"
            # print(f"    [规则1] 索引 {i} 满足间隔 {photo_interval}")

        # 规则2: 高成本段（仅提升优先级，不自动设为拍照点）
        if i > 0 and i <= len(segment_costs):
            if segment_costs[i - 1] > high_cost_threshold:
                priority = "high"
                # 注意：不自动设为拍照点，只提升优先级
                reason = f"high_cost_segment_{i-1}"
                # print(f"    [规则2] 索引 {i} 高成本段 {segment_costs[i-1]:.2f}")

        # 规则3: 高度变化
        if i > 0:
            height_change = abs(z - path_3d[i - 1][2])
            if height_change > height_change_threshold:
                priority = "high"
                gimbal_angle = -45  # 大角度俯拍
                reason = f"height_change_{height_change:.1f}m"
                # print(f"    [规则3] 索引 {i} 高度变化 {height_change:.1f}m")

        # 起点和终点总是拍照
        if i == 0:
            is_photo = True
            action = "photo"
            reason = "start_point"
        elif i == len(path_3d) - 1:
            is_photo = True
            action = "photo"
            reason = "end_point"

        # 创建任务点
        point = InspectionPoint(
            position=(x, y, z),
            index=i,
            is_photo=is_photo,
            priority=priority,
            gimbal_angle=gimbal_angle,
            action=action,
            cost=cumulative_cost,
            reason=reason
        )

        inspection_points.append(point)

    # 统计
    photo_count = sum(1 for p in inspection_points if p.is_photo)
    high_priority_count = sum(1 for p in inspection_points if p.priority == "high")
    photo_indices = [p.index for p in inspection_points if p.is_photo]

    print(f"  总任务点: {len(inspection_points)}")
    print(f"  拍照点: {photo_count}")
    print(f"  拍照点索引: {photo_indices}")
    print(f"  高优先级: {high_priority_count}")

    return inspection_points


def filter_photo_points(inspection_points: List[InspectionPoint]):
    """过滤出只包含拍照的点"""
    return [p for p in inspection_points if p.is_photo]


def filter_high_priority(inspection_points: List[InspectionPoint]):
    """过滤出高优先级点"""
    return [p for p in inspection_points if p.priority == "high"]


def get_task_summary(inspection_points: List[InspectionPoint]):
    """获取任务统计摘要"""
    return {
        'total_points': len(inspection_points),
        'photo_points': sum(1 for p in inspection_points if p.is_photo),
        'high_priority': sum(1 for p in inspection_points if p.priority == "high"),
        'total_cost': inspection_points[-1].cost if inspection_points else 0,
        'avg_gimbal_angle': np.mean([p.gimbal_angle for p in inspection_points if p.is_photo])
                               if any(p.is_photo for p in inspection_points) else 0
    }


# =====================================================
# 阶段1新增：多独立线路巡检点适配（增量兼容）
# =====================================================

def convert_line_points_to_legacy_inspection_points(
    line_points_by_line: dict
) -> List[InspectionPoint]:
    """
    将多线路巡检点转换为旧 InspectionPoint 格式（兼容适配）

    Args:
        line_points_by_line: Dict[line_id, List[LineInspectionPoint]]

    Returns:
        List[InspectionPoint]: 兼容旧接口的巡检点列表
    """
    from core.inspection_point_generator import LineInspectionPoint

    all_legacy = []
    index_counter = 0

    for line_id, points in line_points_by_line.items():
        for point in points:
            # 转换为旧格式
            legacy_point = InspectionPoint(
                position=point.position_3d if point.position_3d else (
                    point.pixel_position[0],
                    point.pixel_position[1],
                    25  # 默认高度
                ),
                index=index_counter,
                is_photo=point.point_type in ('endpoint', 'turning'),
                priority=point.priority,
                gimbal_angle=-30,
                action='photo' if point.point_type in ('endpoint', 'turning') else 'fly',
                cost=0,
                reason=point.source_reason
            )
            all_legacy.append(legacy_point)
            index_counter += 1

    # 按索引排序
    all_legacy.sort(key=lambda p: p.index)

    return all_legacy


def create_inspection_points_from_independent_lines(
    planner
) -> List[InspectionPoint]:
    """
    从规划器的多独立线路数据创建兼容的巡检点列表

    Args:
        planner: PowerlinePlannerV3 实例（已运行 step5_generate_line_inspection_points）

    Returns:
        List[InspectionPoint]: 兼容旧接口的巡检点列表
    """
    if not hasattr(planner, 'line_inspection_points_by_line') or not planner.line_inspection_points_by_line:
        print("[适配] 未找到多线路巡检点数据")
        return []

    return convert_line_points_to_legacy_inspection_points(
        planner.line_inspection_points_by_line
    )


def merge_multi_line_to_single_path(
    line_points_by_line: dict,
    sort_by_length: bool = True
) -> List[dict]:
    """
    将多线路巡检点合并为单一路径格式（用于兼容旧单路径可视化）

    Args:
        line_points_by_line: Dict[line_id, List[LineInspectionPoint]]
        sort_by_length: 是否按线路长度排序

    Returns:
        List[dict]: 合并后的路径点字典列表
    """
    from core.inspection_point_generator import LineInspectionPoint

    # 获取每条线路的巡检点
    line_items = list(line_points_by_line.items())

    # 按线路长度排序
    if sort_by_length:
        line_items.sort(key=lambda item: len(item[1]), reverse=True)

    # 合并为单一路径
    merged_path = []
    index_counter = 0

    for line_id, points in line_items:
        for point in points:
            # 使用 3D 坐标或像素坐标
            if point.position_3d:
                pos = point.position_3d
            else:
                pos = (point.pixel_position[0], point.pixel_position[1], 25)

            merged_path.append({
                'position': pos,
                'index': index_counter,
                'line_id': line_id,
                'point_type': point.point_type,
                'priority': point.priority
            })
            index_counter += 1

    return merged_path
