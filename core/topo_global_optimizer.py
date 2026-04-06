"""
=====================================================
全局拓扑路径优化器
=====================================================

基于《配电网无人机巡检路径优化算法》论文思想实现：
1. 拓扑感知的任务组织
2. 起始接入点优化
3. 全局顺序优化（非贪心）
4. 综合代价函数

核心改进：
- 从"几何距离贪心"升级为"拓扑结构感知的全局优化"
- Connect 代价综合考虑：几何距离、拓扑路径、方向变化、切换代价
- 起始点选择：枚举候选，选择总代价最低
- 顺序优化：模拟退火算法
"""

from typing import List, Dict, Tuple, Optional, Any
import numpy as np
from dataclasses import dataclass
import random

from core.topo import TopoGraph, TopoNode
from core.topo_task import EdgeTask
from core.topo_plan import (
    EdgeGroup, GroupedContinuousMission, MissionSegment,
    build_edge_adjacency_simple, compute_transition_cost_simple,
    get_edge_geometry_with_direction, interpolate_geometry,
    generate_connection_segment_along_topo
)


# =====================================================
# 代价模型：综合多种因素
# =====================================================

@dataclass
class ConnectionCost:
    """连接代价模型"""
    geometric_distance: float  # 几何直线距离
    topo_path_length: float     # 拓扑图上的最短路径长度
    direction_change: float     # 方向变化角度（0-1）
    group_switch_penalty: float # group切换惩罚
    total_cost: float           # 综合总代价


def compute_connection_cost_enhanced(
    from_edge_id: str,
    to_edge_id: str,
    from_direction: str,
    to_direction: str,
    from_point: Tuple[float, float],
    to_point: Tuple[float, float],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    weights: Dict[str, float] = None
) -> ConnectionCost:
    """
    计算增强的连接代价

    Args:
        from_edge_id: 起始边ID
        to_edge_id: 目标边ID
        from_direction: 起始边方向
        to_direction: 目标边方向
        from_point: 起始点坐标
        to_point: 目标点坐标
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        groups: 分组信息
        weights: 代价权重配置

    Returns:
        ConnectionCost: 连接代价详情
    """
    if weights is None:
        weights = {
            'geometric': 1.0,      # 几何距离权重
            'topo': 0.1,           # 拓扑路径权重（降低，从0.5改为0.1）
            'direction': 0.1,      # 方向变化权重（降低，从0.3改为0.1）
            'group_switch': 10.0   # group切换惩罚（降低，从50改为10）
        }

    # 1. 几何距离
    geo_dist = np.linalg.norm(np.array(to_point) - np.array(from_point))

    # 2. 拓扑路径长度（沿图的最短路径）
    from_edge = edge_task_map.get(from_edge_id)
    to_edge = edge_task_map.get(to_edge_id)

    topo_path_len = 0.0
    if from_edge and to_edge:
        # 计算从 from_edge 的终点到 to_edge 的起点的拓扑路径长度
        from_end_node = from_edge.v if from_direction == 'forward' else from_edge.u
        to_start_node = to_edge.u if to_direction == 'forward' else to_edge.v

        if from_end_node != to_start_node:
            # 使用 BFS 找最短路径
            path = find_topo_path(topo_graph, from_end_node, to_start_node)
            if path:
                # 计算路径上所有边的长度之和
                topo_path_len = compute_topo_path_length(topo_graph, path, edge_task_map)
            else:
                # 不连通，使用几何距离
                topo_path_len = geo_dist

    # 3. 方向变化（0-1，1表示反向）
    direction_change = compute_direction_change_penalty(
        from_point, to_point, from_direction, to_direction,
        edge_task_map.get(from_edge_id), edge_task_map.get(to_edge_id)
    )

    # 4. Group切换惩罚
    group_switch_penalty = 0.0
    from_group = find_edge_group(from_edge_id, groups)
    to_group = find_edge_group(to_edge_id, groups)
    if from_group and to_group and from_group != to_group:
        group_switch_penalty = weights['group_switch']

    # 综合代价
    total_cost = (
        weights['geometric'] * geo_dist +
        weights['topo'] * topo_path_len +
        weights['direction'] * direction_change * 100 +  # 方向变化放大
        group_switch_penalty
    )

    return ConnectionCost(
        geometric_distance=geo_dist,
        topo_path_length=topo_path_len,
        direction_change=direction_change,
        group_switch_penalty=group_switch_penalty,
        total_cost=total_cost
    )


def find_topo_path(topo_graph: TopoGraph, start_node: str, end_node: str) -> List[str]:
    """BFS 找拓扑路径"""
    if start_node == end_node:
        return [start_node]

    from collections import deque
    visited = {start_node: None}
    queue = deque([start_node])

    while queue:
        curr = queue.popleft()
        if curr == end_node:
            # 重建路径
            path = []
            while curr is not None:
                path.append(curr)
                curr = visited[curr]
            return path[::-1]

        for neighbor in topo_graph.get_neighbors(curr):
            if neighbor not in visited:
                visited[neighbor] = curr
                queue.append(neighbor)

    return []


def compute_topo_path_length(topo_graph: TopoGraph, path: List[str],
                            edge_task_map: Dict[str, EdgeTask]) -> float:
    """计算拓扑路径的实际长度"""
    if len(path) < 2:
        return 0.0

    total_length = 0.0
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]

        # 找到连接 u 和 v 的边
        found = False
        for edge in edge_task_map.values():
            if (edge.u == u and edge.v == v) or (edge.u == v and edge.v == u):
                total_length += edge.len2d
                found = True
                break

        if not found:
            # 没有直接边，使用节点间直线距离
            u_node = topo_graph.get_node(u)
            v_node = topo_graph.get_node(v)
            if u_node and v_node:
                total_length += np.linalg.norm(
                    np.array([v_node.x, v_node.y]) -
                    np.array([u_node.x, u_node.y])
                )

    return total_length


def compute_direction_change_penalty(
    from_point: Tuple[float, float],
    to_point: Tuple[float, float],
    from_direction: str,
    to_direction: str,
    from_edge: EdgeTask,
    to_edge: EdgeTask
) -> float:
    """
    计算方向变化惩罚（0-1）

    0: 方向一致（延续性好）
    1: 完全反向（折返）
    """
    if not from_edge or not to_edge:
        return 0.5  # 默认中等惩罚

    # 计算从边到连接段的方向向量
    from_vec = np.array(to_point) - np.array(from_point)
    from_vec_norm = np.linalg.norm(from_vec)

    if from_vec_norm < 0.1:
        return 0.0

    from_vec = from_vec / from_vec_norm

    # 计算目标边的方向向量
    to_edge_geo = get_edge_geometry_with_direction(to_edge, to_direction)
    if len(to_edge_geo) < 2:
        return 0.5

    to_vec = np.array(to_edge_geo[1]) - np.array(to_edge_geo[0])
    to_vec_norm = np.linalg.norm(to_vec)

    if to_vec_norm < 0.1:
        return 0.5

    to_vec = to_vec / to_vec_norm

    # 计算夹角的余弦值
    dot_product = np.dot(from_vec, to_vec)
    cos_angle = np.clip(dot_product, -1.0, 1.0)

    # 转换为 0-1 的惩罚值
    # cos=1 (同向) -> penalty=0
    # cos=-1 (反向) -> penalty=1
    penalty = (1.0 - cos_angle) / 2.0

    return penalty


def find_edge_group(edge_id: str, groups: Dict[str, EdgeGroup]) -> Optional[str]:
    """查找边所属的group"""
    for group_id, group in groups.items():
        if edge_id in group.edge_ids:
            return group_id
    return None


# =====================================================
# 起始点优化
# =====================================================

def evaluate_start_edge_candidate(
    start_edge_id: str,
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: dict
) -> float:
    """
    评估从指定边开始的起始代价

    考虑因素：
    - 是否为端点边（deg=1 的节点）
    - 是否在大 group 中
    - 从该点出发的可达性

    Returns:
        float: 起始代价（越小越好）
    """
    edge = edge_task_map.get(start_edge_id)
    if not edge:
        return float('inf')

    cost = 0.0

    # 1. 端点边优先（deg=1 的节点是天然的起点/终点）
    u_node = topo_graph.get_node(edge.u)
    v_node = topo_graph.get_node(edge.v)
    has_endpoint = (u_node and u_node.deg == 1) or (v_node and v_node.deg == 1)

    if has_endpoint:
        cost -= 100.0  # 端点边降低代价（优先）
    else:
        cost += 20.0   # 中间边增加代价

    # 2. 大 group 优先
    group_id = find_edge_group(start_edge_id, groups)
    if group_id:
        group = groups[group_id]
        # group 越大，起始代价越低
        cost -= group.total_inspect_length * 0.1

    # 3. 中心性评估（越靠近中心的边，起始代价越低）
    # 简化：使用相邻边数量
    neighbors = adjacency.get(start_edge_id, [])
    cost -= len(neighbors) * 5.0

    return cost


def generate_start_edge_candidates(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    groups: Dict[str, EdgeGroup],
    edge_task_map: Dict[str, EdgeTask],
    adjacency: dict,
    max_candidates: int = 10
) -> List[Tuple[str, float]]:
    """
    生成候选起始边列表

    Returns:
        List[Tuple[edge_id, score]]: 按起始代价排序
    """
    candidates = []

    for edge in edge_tasks:
        score = evaluate_start_edge_candidate(
            edge.edge_id, edge_tasks, topo_graph,
            groups, edge_task_map, adjacency
        )
        candidates.append((edge.edge_id, score))

    # 按代价排序（低到高）
    candidates.sort(key=lambda x: x[1])

    # 返回前 N 个
    return candidates[:max_candidates]


# =====================================================
# 全局顺序优化（模拟退火）
# =====================================================

def evaluate_order_cost(
    edge_order: List[str],
    edge_directions: Dict[str, str],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    weights: Dict[str, float] = None
) -> float:
    """
    评估边访问顺序的总代价

    包括：
    - 所有 connect 段的代价
    - group 切换惩罚
    - 方向变化惩罚
    """
    if weights is None:
        weights = {
            'geometric': 1.0,
            'topo': 0.5,
            'direction': 0.3,
            'group_switch': 50.0
        }

    total_cost = 0.0
    current_edge_id = None
    current_direction = None
    current_point = None

    for i, edge_id in enumerate(edge_order):
        direction = edge_directions.get(edge_id, 'forward')
        edge = edge_task_map.get(edge_id)

        if not edge:
            continue

        # 获取当前边的几何
        geo = get_edge_geometry_with_direction(edge, direction)

        if i == 0:
            # 第一个边，设置起点
            current_point = geo[0]
            current_edge_id = edge_id
            current_direction = direction
            continue

        # 计算从上一个边到当前边的连接代价
        cost = compute_connection_cost_enhanced(
            current_edge_id, edge_id,
            current_direction, direction,
            current_point, geo[0],
            topo_graph, edge_task_map, groups, weights
        )

        total_cost += cost.total_cost

        # 更新当前状态
        current_edge_id = edge_id
        current_direction = direction
        current_point = geo[-1]

    return total_cost


def optimize_edge_order_simulated_annealing(
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict,
    start_edge_id: str = None,
    initial_temp: float = 1000.0,
    cooling_rate: float = 0.95,
    iterations_per_temp: int = 100,
    min_temp: float = 1.0
) -> Tuple[List[str], Dict[str, str], float]:
    """
    使用模拟退火优化边的访问顺序

    Args:
        edge_tasks: 边任务列表
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        groups: 分组信息
        adjacency: 邻接表
        start_edge_id: 指定起始边（可选）
        initial_temp: 初始温度
        cooling_rate: 降温速率
        iterations_per_temp: 每个温度的迭代次数
        min_temp: 最小温度

    Returns:
        (best_order, best_directions, best_cost)
    """
    print("\n" + "="*60)
    print("[Global Optimizer] 模拟退火优化边访问顺序...")
    print("="*60)

    edge_ids = [e.edge_id for e in edge_tasks]

    # 选择起始边
    if start_edge_id is None:
        candidates = generate_start_edge_candidates(
            edge_tasks, topo_graph, groups, edge_task_map, adjacency
        )
        start_edge_id = candidates[0][0]
        print(f"  [起始边] 自动选择: {start_edge_id} (score={candidates[0][1]:.1f})")

    # 初始化：随机顺序，固定起始边
    current_order = [start_edge_id]
    remaining = [eid for eid in edge_ids if eid != start_edge_id]
    random.shuffle(remaining)
    current_order.extend(remaining)

    # 初始化方向（优先选择从端点开始的方向）
    current_directions = {}
    for eid in edge_ids:
        edge = edge_task_map.get(eid)
        if edge:
            u_node = topo_graph.get_node(edge.u)
            v_node = topo_graph.get_node(edge.v)

            # 优先从 deg=1 的节点开始
            if u_node and u_node.deg == 1:
                current_directions[eid] = 'reverse'  # 从 v 到 u
            elif v_node and v_node.deg == 1:
                current_directions[eid] = 'forward'   # 从 u 到 v
            else:
                current_directions[eid] = 'forward'

    # 计算初始代价
    current_cost = evaluate_order_cost(
        current_order, current_directions,
        topo_graph, edge_task_map, groups
    )

    best_order = current_order.copy()
    best_directions = current_directions.copy()
    best_cost = current_cost

    temp = initial_temp
    iteration = 0

    print(f"  [初始] 代价: {current_cost:.1f}")
    print(f"  [参数] T0={initial_temp}, 降温率={cooling_rate}, 每T迭代={iterations_per_temp}")

    while temp > min_temp:
        for _ in range(iterations_per_temp):
            iteration += 1

            # 生成邻域解
            new_order = current_order.copy()
            new_directions = current_directions.copy()

            # 随机选择一种操作
            operation = random.choice(['swap', 'reverse', 'direction'])

            if operation == 'swap' and len(new_order) > 2:
                # 交换两个非起始边的位置
                idx1, idx2 = random.sample(range(1, len(new_order)), 2)
                new_order[idx1], new_order[idx2] = new_order[idx2], new_order[idx1]

            elif operation == 'reverse' and len(new_order) > 3:
                # 反转一段子路径（不包括起始边）
                start = random.randint(1, len(new_order) - 2)
                end = random.randint(start + 1, len(new_order) - 1)
                new_order[start:end+1] = new_order[start:end+1][::-1]

            elif operation == 'direction':
                # 随机改变一条边的方向
                idx = random.randint(0, len(new_order) - 1)
                eid = new_order[idx]
                new_directions[eid] = 'reverse' if new_directions[eid] == 'forward' else 'forward'

            # 计算新解代价
            new_cost = evaluate_order_cost(
                new_order, new_directions,
                topo_graph, edge_task_map, groups
            )

            # 接受准则
            delta = new_cost - current_cost
            if delta < 0 or random.random() < np.exp(-delta / temp):
                current_order = new_order
                current_directions = new_directions
                current_cost = new_cost

                if current_cost < best_cost:
                    best_order = current_order.copy()
                    best_directions = current_directions.copy()
                    best_cost = current_cost

        # 降温
        temp *= cooling_rate

        if iteration % 500 == 0:
            print(f"  [进度] iter={iteration}, T={temp:.1f}, 当前最优={best_cost:.1f}")

    print(f"  [完成] 最终代价: {best_cost:.1f} (迭代 {iteration} 次)")
    print(f"  [最优顺序] {' -> '.join(best_order[:5])}... (共{len(best_order)}条边)")

    return best_order, best_directions, best_cost


# =====================================================
# 构建优化后的任务
# =====================================================

def build_optimized_mission(
    edge_order: List[str],
    edge_directions: Dict[str, str],
    edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: dict
) -> GroupedContinuousMission:
    """
    根据优化后的顺序构建任务
    """
    print("\n" + "="*60)
    print("[Global Optimizer] 构建优化后的任务...")
    print("="*60)

    mission = GroupedContinuousMission()

    # 记录 group 访问顺序
    visited_groups = []
    current_group = None

    current_end_point = None
    current_edge_id = None
    current_direction = None

    for i, edge_id in enumerate(edge_order):
        direction = edge_directions.get(edge_id, 'forward')
        edge = edge_task_map.get(edge_id)

        if not edge:
            continue

        group_id = find_edge_group(edge_id, groups)

        # 检查是否切换了 group
        if group_id and group_id != current_group:
            if current_group is not None:
                visited_groups.append(group_id)
            current_group = group_id

        # 获取当前边的几何
        geo = get_edge_geometry_with_direction(edge, direction)

        if i == 0:
            # 第一个边：直接添加 inspect 段
            mission.segments.append(MissionSegment(
                type='inspect',
                from_edge_id=None,
                to_edge_id=edge_id,
                geometry=geo,
                length=edge.len2d,
                edge_id=edge_id
            ))
            mission.visit_order.append(edge_id)

            current_end_point = geo[-1]
            current_edge_id = edge_id
            current_direction = direction

            # 添加到 group（如果存在）
            if group_id and group_id in groups:
                if group_id not in mission.groups:
                    mission.add_group(groups[group_id])

            print(f"  [{i+1}] {edge_id} ({direction}) - 起点")
            continue

        # 生成 connect 段
        connect_geo, connect_len = generate_connection_segment_along_topo(
            current_end_point, geo[0], topo_graph, edge_task_map
        )

        # 插值密集路径点
        connect_geo_dense = interpolate_geometry(connect_geo, step_size=10.0, min_points=20)

        # 检查是否跨组
        is_inter_group = (find_edge_group(current_edge_id, groups) !=
                         find_edge_group(edge_id, groups))

        # 添加 connect 段（不使用 is_inter_group 参数）
        connect_segment = MissionSegment(
            type='connect',
            from_edge_id=current_edge_id,
            to_edge_id=edge_id,
            geometry=connect_geo_dense,
            length=connect_len
        )
        mission.segments.append(connect_segment)

        # 统计跨组/内组连接
        if is_inter_group:
            mission.inter_group_connect_length += connect_len
        else:
            mission.intra_group_connect_length += connect_len

        # 添加 inspect 段
        mission.segments.append(MissionSegment(
            type='inspect',
            from_edge_id=current_edge_id,
            to_edge_id=edge_id,
            geometry=geo,
            length=edge.len2d,
            edge_id=edge_id
        ))
        mission.visit_order.append(edge_id)

        # 添加到 group
        if group_id and group_id in groups:
            if group_id not in mission.groups:
                mission.add_group(groups[group_id])

        # 更新状态
        current_end_point = geo[-1]
        current_edge_id = edge_id
        current_direction = direction

        from_group = find_edge_group(current_edge_id, groups)
        to_group = find_edge_group(edge_id, groups)
        group_info = f" [{from_group}->{to_group}]" if from_group != to_group else ""
        print(f"  [{i+1}] {edge_id} ({direction}) - connect={connect_len:.1f}px{group_info}")

    # 设置 group 访问顺序
    mission.group_visit_order = visited_groups

    # 统计
    inspect_len = sum(s.length for s in mission.segments if s.type == 'inspect')
    connect_len = sum(s.length for s in mission.segments if s.type == 'connect')

    mission.total_length = inspect_len + connect_len
    mission.inspect_length = inspect_len
    mission.connect_length = connect_len

    # 计算跨组切换次数
    inter_group_count = 0
    for seg in mission.segments:
        if seg.type == 'connect':
            from_group = find_edge_group(seg.from_edge_id, groups)
            to_group = find_edge_group(seg.to_edge_id, groups)
            if from_group and to_group and from_group != to_group:
                inter_group_count += 1

    print(f"\n  [统计] 总长度={mission.total_length:.1f}px, "
          f"inspect={mission.inspect_length:.1f}px, "
          f"connect={mission.connect_length:.1f}px")
    print(f"  [统计] Group切换={inter_group_count}次, "
          f"内组连接={mission.intra_group_connect_length:.1f}px, "
          f"跨组连接={mission.inter_group_connect_length:.1f}px")

    return mission


# =====================================================
# 主入口：全局拓扑优化
# =====================================================

def plan_global_topology_optimized_mission(
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    start_edge_id: str = None,
    enable_sa: bool = True,
    eps: float = 150.0
) -> GroupedContinuousMission:
    """
    全局拓扑优化的主入口

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        start_edge_id: 指定起始边（可选，不指定则自动优化）
        enable_sa: 是否启用模拟退火优化
        eps: 分组距离阈值

    Returns:
        GroupedContinuousMission: 优化后的任务
    """
    print("\n" + "="*70)
    print("[全局拓扑优化] 开始规划...")
    print("="*70)

    # 1. 构建辅助数据结构
    edge_task_map = {task.edge_id: task for task in edge_tasks}
    adjacency = build_edge_adjacency_simple(topo_graph)

    # 2. 空间分组
    print("\n[Step 1] 空间分组...")
    centroids = {
        task.edge_id: (
            (task.polyline[0][0] + task.polyline[-1][0]) / 2,
            (task.polyline[0][1] + task.polyline[-1][1]) / 2
        )
        for task in edge_tasks
    }

    # 检查 sklearn
    try:
        from sklearn.cluster import DBSCAN
        _has_sklearn = True
    except ImportError:
        _has_sklearn = False

    if _has_sklearn:
        from core.topo_plan import group_edges_spatially
        groups = group_edges_spatially(edge_tasks, centroids, eps=eps)
    else:
        # 不分组，每条边一个 group
        groups = {}
        for i, task in enumerate(edge_tasks):
            groups[f"Group_{i}"] = EdgeGroup(
                group_id=f"Group_{i}",
                edge_ids=[task.edge_id],
                centroid=centroids[task.edge_id],
                bbox=(
                    min(p[0] for p in task.polyline),
                    min(p[1] for p in task.polyline),
                    max(p[0] for p in task.polyline),
                    max(p[1] for p in task.polyline)
                ),
                total_inspect_length=task.len2d
            )

    # 3. 起始边优化（如果未指定）
    if start_edge_id is None:
        print("\n[Step 2] 起始边优化...")
        candidates = generate_start_edge_candidates(
            edge_tasks, topo_graph, groups, edge_task_map, adjacency, max_candidates=10
        )
        print(f"  [候选起始边] Top 5:")
        for eid, score in candidates[:5]:
            print(f"    {eid}: {score:.1f}")
        start_edge_id = candidates[0][0]

    # 4. 全局顺序优化
    if enable_sa:
        print("\n[Step 3] 全局顺序优化（模拟退火）...")
        edge_order, edge_directions, cost = optimize_edge_order_simulated_annealing(
            edge_tasks, topo_graph, edge_task_map, groups, adjacency,
            start_edge_id=start_edge_id,
            initial_temp=1000.0,
            cooling_rate=0.95,
            iterations_per_temp=50,  # 减少迭代次数加快速度
            min_temp=1.0
        )
    else:
        print("\n[Step 3] 使用贪心策略...")
        # 简单贪心：最近邻
        edge_order = [start_edge_id]
        edge_directions = {start_edge_id: 'forward'}
        unvisited = set(e.edge_id for e in edge_tasks) - {start_edge_id}

        while unvisited:
            current_edge_id = edge_order[-1]
            current_edge = edge_task_map[current_edge_id]
            current_geo = get_edge_geometry_with_direction(
                current_edge, edge_directions[current_edge_id]
            )
            current_point = current_geo[-1]

            # 找最近的未访问边
            best_next = None
            best_dist = float('inf')

            for eid in unvisited:
                edge = edge_task_map[eid]
                # 尝试两个方向
                for direction in ['forward', 'reverse']:
                    geo = get_edge_geometry_with_direction(edge, direction)
                    dist = np.linalg.norm(np.array(geo[0]) - np.array(current_point))
                    if dist < best_dist:
                        best_dist = dist
                        best_next = eid
                        best_direction = direction

            if best_next:
                edge_order.append(best_next)
                edge_directions[best_next] = best_direction
                unvisited.remove(best_next)

    # 5. 构建任务
    print("\n[Step 4] 构建优化任务...")
    mission = build_optimized_mission(
        edge_order, edge_directions,
        edge_tasks, topo_graph, edge_task_map, groups, adjacency
    )

    print("\n" + "="*70)
    print("[全局拓扑优化] 完成")
    print("="*70)

    return mission
