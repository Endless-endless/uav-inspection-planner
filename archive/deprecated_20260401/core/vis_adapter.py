"""
阶段2结果 → 可视化适配层

功能：
- 将planner的阶段2结果转换为visualization_enhanced.py可接受的格式
- 保持旧接口兼容性
- 支持任务分段显示和统计信息展示
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import numpy as np


@dataclass
class VisPoint:
    """可视化巡检点"""
    point_id: str
    line_id: str
    task_id: str
    point_type: str  # endpoint/turning/sample
    position_2d: Tuple[float, float]
    position_3d: Tuple[float, float, float]
    priority: str = "normal"  # high/normal


@dataclass
class VisTask:
    """可视化任务段"""
    task_id: str
    line_id: str
    direction: int  # 1=正向, -1=反向
    point_start_idx: int
    point_end_idx: int
    color: str
    length: float


@dataclass
class VisStats:
    """可视化统计信息"""
    total_tasks: int
    total_points: int
    total_length: float
    total_cost: float
    task_order: List[str]
    task_directions: Dict[str, int]


def build_vis_pts(planner) -> List[VisPoint]:
    """
    将planner的巡检点转换为可视化格式

    Args:
        planner: PowerlinePlannerV3实例

    Returns:
        List[VisPoint]: 可视化巡检点列表
    """
    vis_pts = []

    if not hasattr(planner, 'line_inspection_points') or not planner.line_inspection_points:
        return vis_pts

    for pt in planner.line_inspection_points:
        # 确定优先级（从LineInspectionPoint获取）
        priority = pt.priority if hasattr(pt, 'priority') else 'normal'

        # 确定task_id（根据line_id）
        task_id = pt.line_id

        # 获取2D位置
        pos_2d = pt.pixel_position if hasattr(pt, 'pixel_position') else (0, 0)

        # 获取3D位置
        if hasattr(pt, 'position_3d') and pt.position_3d is not None:
            pos_3d = pt.position_3d
        else:
            pos_3d = (pos_2d[0], pos_2d[1], 0)

        vis_pt = VisPoint(
            point_id=pt.id,
            line_id=pt.line_id,
            task_id=task_id,
            point_type=pt.point_type,
            position_2d=pos_2d,
            position_3d=pos_3d,
            priority=priority
        )
        vis_pts.append(vis_pt)

    return vis_pts


def build_vis_tasks(planner) -> List[VisTask]:
    """
    构建任务段信息

    Args:
        planner: PowerlinePlannerV3实例

    Returns:
        List[VisTask]: 可视化任务段列表
    """
    vis_tasks = []

    if not hasattr(planner, 'tasks') or not planner.tasks:
        return vis_tasks

    if not hasattr(planner, 'line_ord') or not planner.line_ord:
        return vis_tasks

    # 任务颜色映射（循环使用）
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    current_idx = 0

    for ord_idx in planner.line_ord:
        task = planner.tasks[ord_idx]
        direction = planner.line_dir.get(task.id, 1)

        # 计算点数
        pts = planner.line_inspection_points_by_line.get(task.line_id, [])
        n_pts = len(pts)

        vis_task = VisTask(
            task_id=task.id,
            line_id=task.line_id,
            direction=direction,
            point_start_idx=current_idx,
            point_end_idx=current_idx + n_pts - 1,
            color=colors[len(vis_tasks) % len(colors)],
            length=task.len2d
        )
        vis_tasks.append(vis_task)
        current_idx += n_pts

    return vis_tasks


def build_vis_stats(planner) -> VisStats:
    """
    汇总展示统计信息

    Args:
        planner: PowerlinePlannerV3实例

    Returns:
        VisStats: 可视化统计信息
    """
    # 获取任务顺序
    task_order = []
    if hasattr(planner, 'line_ord') and planner.line_ord:
        task_order = [planner.tasks[i].id for i in planner.line_ord]

    # 获取任务方向
    task_directions = {}
    if hasattr(planner, 'line_dir'):
        task_directions = planner.line_dir.copy()

    # 计算总点数
    total_points = 0
    if hasattr(planner, 'line_inspection_points'):
        total_points = len(planner.line_inspection_points)

    # 从g_stats获取统计
    total_length = 0
    total_cost = 0
    if hasattr(planner, 'g_stats'):
        total_length = planner.g_stats.get('total_len', 0)
        total_cost = planner.g_stats.get('total_cost', 0)

    return VisStats(
        total_tasks=len(task_order),
        total_points=total_points,
        total_length=total_length,
        total_cost=total_cost,
        task_order=task_order,
        task_directions=task_directions
    )


def build_anim_path(planner) -> List[Tuple[float, float, float]]:
    """
    返回适合动画的3D路径

    Args:
        planner: PowerlinePlannerV3实例

    Returns:
        List[Tuple]: 3D路径点列表
    """
    if hasattr(planner, 'g_path_3d') and planner.g_path_3d:
        return [(float(p[0]), float(p[1]), float(p[2])) for p in planner.g_path_3d]
    return []


def adapt_stage2_to_vis(planner) -> Tuple[List[VisPoint], List[VisTask], VisStats, List[Tuple]]:
    """
    整合以上内容，将阶段2结果转换为可视化格式

    Args:
        planner: PowerlinePlannerV3实例

    Returns:
        Tuple: (vis_pts, vis_tasks, vis_stats, anim_path_3d)
    """
    vis_pts = build_vis_pts(planner)
    vis_tasks = build_vis_tasks(planner)
    vis_stats = build_vis_stats(planner)
    anim_path_3d = build_anim_path(planner)

    print(f"[可视化适配] 转换完成:")
    print(f"  - 巡检点: {len(vis_pts)}")
    print(f"  - 任务段: {len(vis_tasks)}")
    print(f"  - 动画路径: {len(anim_path_3d)} 点")

    return vis_pts, vis_tasks, vis_stats, anim_path_3d
