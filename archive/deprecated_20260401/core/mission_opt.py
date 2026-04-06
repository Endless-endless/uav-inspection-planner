"""
任务层优化模块

功能：
- TaskLine 数据结构
- 从阶段1结果构建 task 层
- 任务顺序优化（最近邻 + 2-opt）
- 任务方向优化

设计原则：
- TaskLine 可扩展为拓扑任务
- cost_mat 围绕 task 设计，不是死写 line
- 为未来拓扑图优化预留接口
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
from core.costs import build_cost_mat, trans_cost


@dataclass
class TaskLine:
    """
    任务单元数据结构

    当前：对应一条独立线路
    未来：可扩展为拓扑边/分支/段

    Attributes:
        id: 任务唯一标识
        kind: 任务类型 ("line" / 未来 "branch" / "seg" / "edge")
        line_id: 原始线路ID
        pts: 巡检点序列
        p_start: 起点 3D 坐标
        p_end: 终点 3D 坐标
        len2d: 2D 长度
        n_pts: 点数
        meta: 预留拓扑扩展字段
    """
    id: str
    kind: str = "line"
    line_id: str = ""
    pts: List = field(default_factory=list)
    p_start: Tuple[float, float, float] = (0, 0, 0)
    p_end: Tuple[float, float, float] = (0, 0, 0)
    len2d: float = 0.0
    n_pts: int = 0
    meta: Dict = field(default_factory=dict)

    def __post_init__(self):
        """初始化后处理：填充 meta 默认值"""
        if not self.meta:
            self.meta = {
                'comp_id': None,      # 所属连通域
                'topo_id': None,      # 拓扑ID
                'parent': None,       # 父节点
                'children': [],       # 子节点
                'branch_flag': False  # 是否分支
            }


def build_tasks(
    lines: List,
    pts_by_line: Dict[str, List]
) -> List[TaskLine]:
    """
    从阶段1结果构建 task 层

    当前：一个 line 对应一个 task
    未来：可从 topo graph 构建多个 task

    Args:
        lines: IndependentLine 列表
        pts_by_line: 按线路分组的巡检点

    Returns:
        List[TaskLine]: 任务列表
    """
    tasks = []

    for line in lines:
        pts = pts_by_line.get(line.id, [])

        if not pts:
            continue

        # 获取起点和终点
        p_start = pts[0].position_3d if pts[0].position_3d else (
            pts[0].pixel_position[0], pts[0].pixel_position[1], 0
        )
        p_end = pts[-1].position_3d if pts[-1].position_3d else (
            pts[-1].pixel_position[0], pts[-1].pixel_position[1], 0
        )

        task = TaskLine(
            id=line.id,
            kind="line",
            line_id=line.id,
            pts=pts,
            p_start=p_start,
            p_end=p_end,
            len2d=line.length_2d,
            n_pts=len(pts),
            meta={
                'comp_id': None,
                'topo_id': None,
                'parent': None,
                'children': [],
                'branch_flag': False
            }
        )

        tasks.append(task)

    return tasks


def init_ord_nn(
    tasks: List[TaskLine],
    cost_mat: np.ndarray,
    start_pos: Optional[Tuple[float, float]] = None
) -> List[int]:
    """
    最近邻生成初始任务顺序

    Args:
        tasks: 任务列表
        cost_mat: 代价矩阵
        start_pos: 起始位置 (可选)

    Returns:
        List[int]: 任务索引顺序
    """
    n = len(tasks)
    if n == 0:
        return []

    visited = [False] * n
    order = []

    # 确定起始任务
    if start_pos is not None:
        # 找最近的任务作为起点
        start_idx = 0
        min_dist = float('inf')
        for i, task in enumerate(tasks):
            dist = np.linalg.norm(np.array(task.p_start[:2]) - np.array(start_pos))
            if dist < min_dist:
                min_dist = dist
                start_idx = i
    else:
        start_idx = 0

    current = start_idx
    visited[current] = True
    order.append(current)

    # 最近邻贪心
    for _ in range(n - 1):
        nearest = -1
        min_cost = float('inf')

        for j in range(n):
            if visited[j]:
                continue

            if cost_mat[current, j] < min_cost:
                min_cost = cost_mat[current, j]
                nearest = j

        if nearest >= 0:
            visited[nearest] = True
            order.append(nearest)
            current = nearest
        else:
            break

    # 添加剩余未访问的任务
    for i in range(n):
        if not visited[i]:
            order.append(i)

    return order


def opt_ord_2opt(
    ord0: List[int],
    cost_mat: np.ndarray,
    max_iter: int = 1000
) -> List[int]:
    """
    2-opt 优化任务顺序

    Args:
        ord0: 初始顺序
        cost_mat: 代价矩阵
        max_iter: 最大迭代次数

    Returns:
        List[int]: 优化后的顺序
    """
    order = ord0.copy()
    n = len(order)
    best_cost = _calc_path_cost(order, cost_mat)

    improved = True
    iterations = 0

    while improved and iterations < max_iter:
        improved = False
        iterations += 1

        for i in range(n - 1):
            for j in range(i + 2, n):
                # 尝试 2-opt 交换
                new_order = order[:i+1] + order[i+1:j+1][::-1] + order[j+1:]
                new_cost = _calc_path_cost(new_order, cost_mat)

                if new_cost < best_cost:
                    order = new_order
                    best_cost = new_cost
                    improved = True

    return order


def _calc_path_cost(order: List[int], cost_mat: np.ndarray) -> float:
    """计算路径总成本"""
    if len(order) < 2:
        return 0.0

    cost = 0.0
    for i in range(len(order) - 1):
        cost += cost_mat[order[i], order[i + 1]]

    return cost


def opt_line_dir(
    ord1: List[int],
    tasks: List[TaskLine],
    pts_by_line: Dict[str, List],
    wind: Optional[Dict] = None
) -> Dict[str, int]:
    """
    优化每个任务的方向

    Args:
        ord1: 任务顺序
        tasks: 任务列表
        pts_by_line: 按线路分组的巡检点
        wind: 风向信息

    Returns:
        Dict[str, int]: 方向映射 {task_id: 1=正向, -1=反向}
    """
    n = len(ord1)
    if n == 0:
        return {}

    dir_map = {}
    prev_p_end = None

    for i, idx in enumerate(ord1):
        task = tasks[idx]

        # 确定当前任务的起点位置
        if i == 0:
            # 第一个任务：默认正向
            dir_map[task.id] = 1
        else:
            # 后续任务：选择使切换成本最小的方向
            prev_task = tasks[ord1[i - 1]]
            prev_dir = dir_map[prev_task.id]

            # 获取前一个任务的终点
            if prev_dir == 1:
                prev_p_end = prev_task.p_end
            else:
                prev_p_end = prev_task.p_start

            # 计算两个方向的切换成本
            cost_forward = trans_cost(prev_task, prev_dir, task, 1, pts_by_line, wind)
            cost_reverse = trans_cost(prev_task, prev_dir, task, -1, pts_by_line, wind)

            if cost_forward <= cost_reverse:
                dir_map[task.id] = 1
            else:
                dir_map[task.id] = -1

    return dir_map


def calc_mission_cost(
    ord1: List[int],
    dir_map: Dict[str, int],
    tasks: List[TaskLine],
    pts_by_line: Dict[str, List],
    wind: Optional[Dict] = None
) -> float:
    """
    计算任务方案的总成本

    Args:
        ord1: 任务顺序
        dir_map: 方向映射
        tasks: 任务列表
        pts_by_line: 按线路分组的巡检点
        wind: 风向信息

    Returns:
        float: 总成本
    """
    if len(ord1) < 2:
        return 0.0

    total = 0.0

    for i in range(len(ord1) - 1):
        task_a = tasks[ord1[i]]
        task_b = tasks[ord1[i + 1]]
        dir_a = dir_map[task_a.id]
        dir_b = dir_map[task_b.id]

        cost = trans_cost(task_a, dir_a, task_b, dir_b, pts_by_line, wind)
        total += cost

    return total
