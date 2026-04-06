"""
路径拼接模块

功能：
- 展开任务为点序列
- 连接两段路径
- 构建全局巡检路径

设计原则：
- 当前按 line task 展开
- 未来可扩展到 topo task
- 代码结构保留扩展能力
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from core.mission_opt import TaskLine


def expand_task(
    task: TaskLine,
    dir_flag: int,
    pts_by_line: Dict[str, List]
) -> List[Tuple[float, float, float]]:
    """
    展开任务为 3D 点序列

    当前：line task 展开为巡检点序列
    未来：topo task 可展开为边序列

    Args:
        task: 任务单元
        dir_flag: 方向 (1=正向, -1=反向)
        pts_by_line: 按线路分组的巡检点

    Returns:
        List[Tuple]: 3D 点序列
    """
    pts = pts_by_line.get(task.line_id, [])

    if not pts:
        return []

    # 按方向展开
    if dir_flag == 1:
        ordered_pts = pts
    else:
        ordered_pts = pts[::-1]

    # 提取 3D 坐标
    path_3d = []
    for pt in ordered_pts:
        if pt.position_3d:
            path_3d.append(pt.position_3d)
        else:
            path_3d.append((
                pt.pixel_position[0],
                pt.pixel_position[1],
                0
            ))

    return path_3d


def conn_seg(
    p_end: Tuple[float, float, float],
    p_start: Tuple[float, float, float],
    num_points: int = 10
) -> List[Tuple[float, float, float]]:
    """
    连接两段路径（优化版：使用更多插值点保证连续）

    在两个点之间生成平滑过渡路径，使用线性插值。

    Args:
        p_end: 前一段终点
        p_start: 下一段起点
        num_points: 插值点数（默认10个）

    Returns:
        List[Tuple]: 过渡点序列（不包含端点）
    """
    if num_points <= 0:
        return []

    points = []
    for i in range(1, num_points + 1):
        t = i / (num_points + 1)
        x = p_end[0] + t * (p_start[0] - p_end[0])
        y = p_end[1] + t * (p_start[1] - p_end[1])
        z = p_end[2] + t * (p_start[2] - p_end[2])
        points.append((x, y, z))

    return points


def build_g_path(
    ord1: List[int],
    dir_map: Dict[str, int],
    tasks: List[TaskLine],
    pts_by_line: Dict[str, List],
    start_pos: Optional[Tuple[float, float]] = None,
    add_transitions: bool = True,
    wind: Optional[Dict] = None
) -> Tuple[List[Tuple], List[Tuple], Dict]:
    """
    构建全局巡检路径（优化版：更多过渡点 + 计算总成本）

    当前：按 line task 拼接
    未来：可扩展到 topo task 拼接

    Args:
        ord1: 任务顺序
        dir_map: 方向映射
        tasks: 任务列表
        pts_by_line: 按线路分组的巡检点
        start_pos: 起始位置 (可选)
        add_transitions: 是否添加任务间过渡点
        wind: 风向信息

    Returns:
        Tuple: (g_path_2d, g_path_3d, g_stats)
    """
    from core.costs import path_cost

    if not ord1:
        return [], [], {
            'total_len': 0,
            'total_cost': 0,
            'n_tasks': 0,
            'n_points': 0
        }

    g_path_2d = []
    g_path_3d = []

    # 处理起始位置
    if start_pos is not None:
        g_path_2d.append(start_pos)
        g_path_3d.append((start_pos[0], start_pos[1], 0))

    # 收集所有巡检点用于成本计算
    all_inspection_pts = []

    # 按顺序展开每个任务
    for i, idx in enumerate(ord1):
        task = tasks[idx]
        dir_flag = dir_map[task.id]

        # 展开任务为点序列
        task_path_3d = expand_task(task, dir_flag, pts_by_line)

        if not task_path_3d:
            continue

        # 添加任务间过渡（使用10个插值点）
        if add_transitions and i > 0 and g_path_3d:
            prev_end = g_path_3d[-1]
            curr_start = task_path_3d[0]
            trans_pts = conn_seg(prev_end, curr_start, num_points=10)
            g_path_3d.extend(trans_pts)

        # 添加任务点
        g_path_3d.extend(task_path_3d)

        # 收集巡检点
        all_inspection_pts.extend(task.pts)

    # 生成 2D 路径
    g_path_2d = [(p[0], p[1]) for p in g_path_3d]

    # 计算统计信息
    total_len = 0.0
    for i in range(1, len(g_path_3d)):
        total_len += np.linalg.norm(np.array(g_path_3d[i][:2]) - np.array(g_path_3d[i-1][:2]))

    # 计算总成本
    total_cost = path_cost(g_path_3d, all_inspection_pts, wind)

    g_stats = {
        'total_len': total_len,
        'total_cost': total_cost,
        'n_tasks': len(ord1),
        'n_points': len(g_path_3d)
    }

    return g_path_2d, g_path_3d, g_stats


def save_g_path_visualization(
    g_path_2d: List[Tuple],
    tasks: List[TaskLine],
    ord1: List[int],
    dir_map: Dict[str, int],
    original_image_path: str,
    output_path: str = "result/step9_g_path.png"
):
    """
    保存全局路径可视化

    Args:
        g_path_2d: 全局 2D 路径
        tasks: 任务列表
        ord1: 任务顺序
        dir_map: 方向映射
        original_image_path: 原始图像路径
        output_path: 输出路径
    """
    from PIL import Image, ImageDraw
    import os

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    # 加载原始图像
    try:
        base_img = Image.open(original_image_path).convert("RGB")
    except:
        max_x = max([p[0] for p in g_path_2d]) if g_path_2d else 100
        max_y = max([p[1] for p in g_path_2d]) if g_path_2d else 100
        base_img = Image.new("RGB", (int(max_x) + 50, int(max_y) + 50), (255, 255, 255))

    draw = ImageDraw.Draw(base_img)

    # 绘制任务编号
    for i, idx in enumerate(ord1):
        task = tasks[idx]
        dir_flag = dir_map[task.id]

        # 获取任务的起点
        if dir_flag == 1:
            start_pos = task.p_start[:2]
        else:
            start_pos = task.p_end[:2]

        # 绘制任务编号
        x, y = int(start_pos[0]), int(start_pos[1])
        draw.text((x + 5, y - 15), f"{i+1}", fill=(0, 0, 255))

    # 绘制全局路径
    if len(g_path_2d) > 1:
        draw.line(g_path_2d, fill=(255, 0, 0), width=2)

        # 绘制起点
        start_x, start_y = int(g_path_2d[0][0]), int(g_path_2d[0][1])
        draw.ellipse([start_x-5, start_y-5, start_x+5, start_y+5], fill=(0, 255, 0))

        # 绘制终点
        end_x, end_y = int(g_path_2d[-1][0]), int(g_path_2d[-1][1])
        draw.ellipse([end_x-5, end_y-5, end_x+5, end_y+5], fill=(255, 0, 255))

    # 保存
    base_img.save(output_path)
    print(f"  [保存] {output_path}")
