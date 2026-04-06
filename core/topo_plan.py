"""
=====================================================
拓扑路径规划模块
=====================================================

功能：
- 基于拓扑图进行任务排序
- 计算图约束下的切换代价
- 优化遍历顺序和方向
- 构建拓扑路径

与线段级方法的区别：
- 切换代价：相邻edge=0，非相邻edge=图路径长度
- 优化目标：基于图的遍历而非欧氏距离

Phase 1特点：
- 复用模拟退火算法，但代价矩阵体现图约束
- 为未来真正图算法（如中国邮路）预留接口
"""

from typing import List, Dict, Tuple, Optional
import numpy as np
from collections import deque

from core.topo import TopoGraph, TopoNode
from core.topo_task import EdgeTask


# =====================================================
# 几何插值：将稀疏路径转换为密集路径
# =====================================================

def interpolate_geometry(geometry: List[Tuple[float, float]],
                         step_size: float = 10.0,
                         min_points: int = 20) -> List[Tuple[float, float]]:
    """
    对稀疏 geometry 进行插值，生成密集的连续路径点

    Args:
        geometry: 原始稀疏几何路径
        step_size: 插值步长（像素），默认 10px
        min_points: 最少点数，确保短路径也有足够点

    Returns:
        插值后的密集几何路径
    """
    if not geometry:
        return []

    if len(geometry) < 2:
        # 单个点，无法插值
        return geometry.copy()

    # 计算原始路径总长度
    total_length = 0.0
    for i in range(1, len(geometry)):
        total_length += np.linalg.norm(np.array(geometry[i]) - np.array(geometry[i-1]))

    # 特殊处理：零长度路径（起点终点相同）
    if total_length < 0.01:
        # 返回 min_points 个相同的点
        return [geometry[0]] * min_points

    # 计算目标点数
    target_points = max(min_points, int(total_length / step_size) + 1)

    # 如果原始点数已经超过目标，直接返回
    if len(geometry) >= target_points:
        return geometry.copy()

    # 插值生成密集路径
    dense_geometry = [geometry[0]]

    for i in range(1, len(geometry)):
        start_pt = np.array(geometry[i-1])
        end_pt = np.array(geometry[i])
        segment_length = np.linalg.norm(end_pt - start_pt)

        # 计算该段应该插值的点数（按比例分配）
        segment_ratio = segment_length / total_length
        segment_target_points = max(1, int(target_points * segment_ratio))

        # 线性插值
        for j in range(1, segment_target_points + 1):
            t = j / (segment_target_points + 1)
            interpolated_pt = start_pt + t * (end_pt - start_pt)
            dense_geometry.append((interpolated_pt[0], interpolated_pt[1]))

        dense_geometry.append((end_pt[0], end_pt[1]))

    # 去重（避免重复点）
    unique_geometry = []
    for pt in dense_geometry:
        if not unique_geometry or np.linalg.norm(np.array(pt) - np.array(unique_geometry[-1])) > 0.1:
            unique_geometry.append(pt)

    # 如果去重后点数太少，进行简单线性插值补充
    if len(unique_geometry) < min_points and len(unique_geometry) >= 2:
        # 计算需要补充的点数
        additional_points = min_points - len(unique_geometry)

        # 在起点和终点之间均匀插入点
        start_pt = np.array(unique_geometry[0])
        end_pt = np.array(unique_geometry[-1])

        # 在现有点之间插入新点
        result = []
        for i in range(len(unique_geometry) - 1):
            result.append(unique_geometry[i])

            # 计算当前段应该插入多少点
            segment_progress = (i + 1) / len(unique_geometry)
            points_to_insert = int(additional_points * segment_progress) - int(additional_points * (i / len(unique_geometry)))

            if points_to_insert > 0:
                pt_curr = np.array(unique_geometry[i])
                pt_next = np.array(unique_geometry[i + 1])
                for j in range(1, points_to_insert + 1):
                    t = j / (points_to_insert + 1)
                    interpolated = pt_curr + t * (pt_next - pt_curr)
                    result.append((interpolated[0], interpolated[1]))

        result.append(unique_geometry[-1])
        return result

    return unique_geometry

    return unique_geometry


# =====================================================
# 图遍历：计算节点间最短路径
# =====================================================

def shortest_path_length(graph: TopoGraph, start: str, end: str) -> float:
    """
    计算图中两个节点间的最短路径长度（BFS）

    Args:
        graph: 拓扑图
        start: 起始节点ID
        end: 结束节点ID

    Returns:
        float: 最短路径长度
    """
    if start == end:
        return 0.0

    # BFS遍历
    visited = {start: 0}
    queue = deque([start])

    while queue:
        curr = queue.popleft()
        curr_dist = visited[curr]

        for neighbor in graph.get_neighbors(curr):
            if neighbor == end:
                return curr_dist + 1

            if neighbor not in visited:
                visited[neighbor] = curr_dist + 1
                queue.append(neighbor)

    # 不连通：返回无穷大
    return float('inf')


def get_shortest_path(graph: TopoGraph, start: str, end: str) -> List[str]:
    """
    获取两个节点间的最短路径（节点序列）

    Args:
        graph: 拓扑图
        start: 起始节点ID
        end: 结束节点ID

    Returns:
        List[str]: 路径上的节点ID序列
    """
    if start == end:
        return [start]

    # BFS + 路径记录
    visited = {start: None}
    queue = deque([start])

    while queue:
        curr = queue.popleft()

        for neighbor in graph.get_neighbors(curr):
            if neighbor == end:
                # 回溯路径
                path = [end]
                while curr is not None:
                    path.append(curr)
                    curr = visited[curr]
                return path[::-1]

            if neighbor not in visited:
                visited[neighbor] = curr
                queue.append(neighbor)

    return []  # 不连通


def build_proximity_augmented_neighbors(
    topo_graph: TopoGraph,
    proximity_threshold: float = 300.0
) -> Dict[str, List[str]]:
    """
    构建基于空间邻近性的增强邻居表

    为拓扑图中距离较近但不直接相连的节点添加虚拟边，
    使 pathfinding 能够在原本不连通的节点之间找到路径。

    Args:
        topo_graph: 拓扑图
        proximity_threshold: 距离阈值（像素），小于此距离的节点对将添加虚拟边

    Returns:
        Dict[str, List[str]]: 增强的邻居表 {node_id: [neighbor_ids]}
    """
    # 复制原始邻居表
    augmented_neighbors = {}

    for node_id in topo_graph.nodes:
        augmented_neighbors[node_id] = list(topo_graph.get_neighbors(node_id))

    # 为每对节点检查距离，添加虚拟边
    node_ids = list(topo_graph.nodes.keys())

    for i, node_a_id in enumerate(node_ids):
        node_a = topo_graph.nodes[node_a_id]

        for node_b_id in node_ids[i + 1:]:
            node_b = topo_graph.nodes[node_b_id]

            # 计算节点间距离
            dist = np.linalg.norm(np.array(node_a.pos2d) - np.array(node_b.pos2d))

            # 如果距离小于阈值且不是已有邻居，添加虚拟边
            if dist < proximity_threshold and node_b_id not in augmented_neighbors[node_a_id]:
                augmented_neighbors[node_a_id].append(node_b_id)
                augmented_neighbors[node_b_id].append(node_a_id)

    return augmented_neighbors


def get_shortest_path_with_proximity(
    topo_graph: TopoGraph,
    start: str,
    end: str,
    proximity_threshold: float = 300.0
) -> List[str]:
    """
    获取两个节点间的最短路径（支持基于邻近性的虚拟边）

    当原始拓扑图中不存在路径时，使用空间邻近性添加虚拟边来找到路径。

    Args:
        topo_graph: 拓扑图
        start: 起始节点ID
        end: 结束节点ID
        proximity_threshold: 邻近性阈值（像素）

    Returns:
        List[str]: 路径上的节点ID序列
    """
    if start == end:
        return [start]

    # 构建增强邻居表
    augmented_neighbors = build_proximity_augmented_neighbors(topo_graph, proximity_threshold)

    # BFS + 路径记录（使用增强邻居表）
    visited = {start: None}
    queue = deque([start])

    while queue:
        curr = queue.popleft()

        # 使用增强邻居表
        for neighbor in augmented_neighbors.get(curr, []):
            if neighbor == end:
                # 回溯路径
                path = [end]
                while curr is not None:
                    path.append(curr)
                    curr = visited[curr]
                return path[::-1]

            if neighbor not in visited:
                visited[neighbor] = curr
                queue.append(neighbor)

    return []  # 不连通


# =====================================================
# 代价矩阵构建（体现图约束）
# =====================================================

def build_adjacency_cost(topo_graph: TopoGraph, edge_tasks: List[EdgeTask]) -> Dict[Tuple[str, str], float]:
    """
    构建边任务之间的切换代价矩阵

    代价定义：
    - 相邻edge（共享节点）：代价 = 0
    - 非相邻edge：代价 = 图路径长度

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表

    Returns:
        Dict[Tuple[str, str], float]: {(edge1_id, edge2_id): cost}
    """
    print("[拓扑规划] 构建邻接代价矩阵...")

    # 构建edge到end node的映射
    edge_ends = {}
    for task in edge_tasks:
        edge_ends[task.edge_id] = {
            'start': task.u,
            'end': task.v
        }

    adj_cost = {}

    for i, task1 in enumerate(edge_tasks):
        for j, task2 in enumerate(edge_tasks):
            if i == j:
                adj_cost[(task1.edge_id, task2.edge_id)] = 0.0
                continue

            # 获取task1的结束节点和task2的起始节点
            # 注意：方向未定，先假设默认方向
            end1 = edge_ends[task1.edge_id]['end']
            start2 = edge_ends[task2.edge_id]['start']

            # 检查是否相邻（共享节点）
            if end1 == start2:
                # 相邻：代价 = 0
                adj_cost[(task1.edge_id, task2.edge_id)] = 0.0
            elif end1 in edge_ends[task2.edge_id].values():
                # task1的结束节点是task2的某个端点
                adj_cost[(task1.edge_id, task2.edge_id)] = 0.0
            elif start2 in edge_ends[task1.edge_id].values():
                # task2的起始节点是task1的某个端点
                adj_cost[(task1.edge_id, task2.edge_id)] = 0.0
            else:
                # 非相邻：代价 = 图路径长度
                path_len = shortest_path_length(topo_graph, end1, start2)
                adj_cost[(task1.edge_id, task2.edge_id)] = path_len

    # 统计
    zero_cost_count = sum(1 for v in adj_cost.values() if v == 0)
    print(f"[拓扑规划] 代价矩阵: {len(adj_cost)} 个条目, {zero_cost_count} 个零代价")

    return adj_cost


# =====================================================
# 边任务排序
# =====================================================

def simulated_annealing_edge(edge_tasks: List[EdgeTask],
                              adj_cost: Dict[Tuple[str, str], float],
                              temp=1000.0, cooling=0.95,
                              iterations=10000) -> List[str]:
    """
    使用模拟退火优化边任务顺序（Phase 1版本）

    与线段级的区别：
    - 代价矩阵体现图约束（相邻边代价=0）
    - 其他逻辑保持一致

    Args:
        edge_tasks: 边任务列表
        adj_cost: 邻接代价矩阵
        temp: 初始温度
        cooling: 冷却系数
        iterations: 迭代次数

    Returns:
        List[str]: 优化后的边任务ID顺序
    """
    print(f"[拓扑规划] 开始模拟退火优化...")

    n = len(edge_tasks)
    if n == 0:
        return []

    # 初始解
    current_order = [task.edge_id for task in edge_tasks]
    np.random.shuffle(current_order)

    # 计算初始代价
    current_cost = total_path_cost(current_order, adj_cost)

    best_order = current_order[:]
    best_cost = current_cost

    for i in range(iterations):
        # 降温
        temp *= cooling

        # 生成邻居解（交换两个位置）
        new_order = current_order[:]
        idx1, idx2 = np.random.choice(n, 2, replace=False)
        new_order[idx1], new_order[idx2] = new_order[idx2], new_order[idx1]

        # 计算新解代价
        new_cost = total_path_cost(new_order, adj_cost)

        # 接受准则
        if new_cost < current_cost:
            current_order = new_order
            current_cost = new_cost

            if current_cost < best_cost:
                best_order = current_order[:]
                best_cost = current_cost
        else:
            # 以概率接受
            delta = new_cost - current_cost
            prob = np.exp(-delta / temp)
            if np.random.random() < prob:
                current_order = new_order
                current_cost = new_cost

    print(f"[拓扑规划] 优化完成: 代价从 {best_cost:.1f} → (初始 → 最终)")

    return best_order


def total_path_cost(order: List[str], adj_cost: Dict[Tuple[str, str], float]) -> float:
    """
    计算给定顺序的总路径代价

    Args:
        order: 边任务ID顺序
        adj_cost: 代价矩阵

    Returns:
        float: 总代价
    """
    if len(order) <= 1:
        return 0.0

    total = 0.0
    for i in range(len(order) - 1):
        edge1, edge2 = order[i], order[i + 1]
        total += adj_cost.get((edge1, edge2), float('inf'))

    return total


# =====================================================
# 方向选择
# =====================================================

def choose_edge_directions(order: List[str],
                           edge_tasks: List[EdgeTask],
                           topo_graph: TopoGraph) -> Dict[str, int]:
    """
    为每个边任务选择遍历方向

    策略（Phase 1）：
    - 首边：从端点开始
    - 中间边：与前一条边连续（共享节点）
    - 末边：确保遍历完所有边

    Args:
        order: 边任务ID顺序
        edge_tasks: 边任务列表
        topo_graph: 拓扑图

    Returns:
        Dict[str, int]: {edge_id: direction}
    """
    print("[拓扑规划] 选择边遍历方向...")

    # 构建edge_id到EdgeTask的映射
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    directions = {}
    last_end_node = None

    for i, edge_id in enumerate(order):
        task = edge_task_map[edge_id]

        if i == 0:
            # 首边：从端点开始（度数=1的节点优先）
            u_deg = len(topo_graph.get_neighbors(task.u))
            v_deg = len(topo_graph.get_neighbors(task.v))

            if u_deg == 1 and v_deg != 1:
                # u是端点，从u开始
                directions[edge_id] = 1
                last_end_node = task.v
            elif v_deg == 1 and u_deg != 1:
                # v是端点，从v开始（反向）
                directions[edge_id] = -1
                last_end_node = task.u
            else:
                # 都是端点或都不是端点，默认正向
                directions[edge_id] = 1
                last_end_node = task.v
        else:
            # 中间边：与前一条边连续
            # 当前边的起始节点应该是上一条边的结束节点
            if task.u == last_end_node:
                # u是上一条边的结束节点，正向
                directions[edge_id] = 1
                last_end_node = task.v
            elif task.v == last_end_node:
                # v是上一条边的结束节点，反向
                directions[edge_id] = -1
                last_end_node = task.u
            else:
                # 不连续（需要路径连接）
                # 选择使路径更短的方向
                path_len_u = shortest_path_length(topo_graph, last_end_node, task.u)
                path_len_v = shortest_path_length(topo_graph, last_end_node, task.v)

                if path_len_u <= path_len_v:
                    directions[edge_id] = 1
                    last_end_node = task.v
                else:
                    directions[edge_id] = -1
                    last_end_node = task.u

    # 统计
    forward = sum(1 for d in directions.values() if d > 0)
    backward = len(directions) - forward
    print(f"[拓扑规划] 方向统计: 正向 {forward}, 反向 {backward}")

    return directions


# =====================================================
# 路径构建
# =====================================================

def build_topo_path(order: List[str],
                    directions: Dict[str, int],
                    edge_tasks: List[EdgeTask],
                    topo_graph: TopoGraph) -> List[Tuple[float, float, float]]:
    """
    构建拓扑路径（3D点序列）

    Args:
        order: 边任务ID顺序
        directions: 方向映射
        edge_tasks: 边任务列表
        topo_graph: 拓扑图

    Returns:
        List[Tuple[float, float, float]]: 3D路径点
    """
    print("[拓扑规划] 构建3D路径...")

    # 构建edge_id到EdgeTask的映射
    edge_task_map = {task.edge_id: task for task in edge_tasks}
    # 构建node_id到TopoNode的映射
    node_map = topo_graph.nodes

    path_3d = []

    for edge_id in order:
        task = edge_task_map[edge_id]
        direction = directions[edge_id]

        # 获取起始和结束节点
        if direction > 0:
            u_node = node_map[task.u]
            v_node = node_map[task.v]
        else:
            u_node = node_map[task.v]
            v_node = node_map[task.u]

        # 获取polyline（从topo_graph中获取）
        edge = topo_graph.edges[edge_id]
        polyline = edge.polyline

        # 转换为3D路径
        for pt in polyline:
            # 插值获取Z坐标
            z_start = u_node.pos3d[2]
            z_end = v_node.pos3d[2]

            # 简单线性插值
            if len(polyline) > 1:
                idx = polyline.index(pt)
                t = idx / (len(polyline) - 1)
                z = z_start + t * (z_end - z_start)
            else:
                z = (z_start + z_end) / 2

            path_3d.append((pt[0], pt[1], z))

    print(f"[拓扑规划] 路径构建完成: {len(path_3d)} 个点")

    return path_3d


# =====================================================
# 主规划函数
# =====================================================

def plan_topo_mission(topo_graph: TopoGraph,
                     edge_tasks: List[EdgeTask],
                     temp: float = 1000.0,
                     cooling: float = 0.95,
                     iterations: int = 10000) -> Tuple[List[str], Dict[str, int], List[Tuple]]:
    """
    拓扑路径规划主函数（Phase 1版本）

    流程：
    1. 构建邻接代价矩阵（体现图约束）
    2. 使用模拟退火优化边顺序
    3. 为每条边选择遍历方向
    4. 构建3D路径

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        temp: 模拟退火初始温度
        cooling: 冷却系数
        iterations: 迭代次数

    Returns:
        Tuple: (边ID顺序, 方向映射, 3D路径)
    """
    print("\n" + "="*60)
    print("[拓扑规划] 开始基于图的巡检路径规划")
    print("="*60)

    # Step 1: 构建邻接代价矩阵
    adj_cost = build_adjacency_cost(topo_graph, edge_tasks)

    # Step 2: 优化边顺序
    order = simulated_annealing_edge(edge_tasks, adj_cost, temp, cooling, iterations)

    # Step 3: 选择方向
    directions = choose_edge_directions(order, edge_tasks, topo_graph)

    # Step 4: 构建路径
    path_3d = build_topo_path(order, directions, edge_tasks, topo_graph)

    print("="*60)
    print("[拓扑规划] 规划完成")
    print("="*60 + "\n")

    return order, directions, path_3d


# =====================================================
# 规划结果评估
# =====================================================

def evaluate_plan(topo_graph: TopoGraph,
                  order: List[str],
                  directions: Dict[str, int],
                  edge_tasks: List[EdgeTask]) -> Dict:
    """
    评估规划结果

    Args:
        topo_graph: 拓扑图
        order: 边ID顺序
        directions: 方向映射
        edge_tasks: 边任务列表

    Returns:
        Dict: 评估指标
    """
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    # 统计连续边
    continuous_count = 0
    for i in range(len(order) - 1):
        edge1 = edge_task_map[order[i]]
        edge2 = edge_task_map[order[i + 1]]

        # 检查是否相邻
        end1 = edge1.end_node
        start2 = edge2.start_node

        if end1 == start2:
            continuous_count += 1

    # 计算总长度
    total_length = sum(task.len2d for task in edge_tasks)

    # 统计图遍历情况
    visited_edges = set(order)
    visited_nodes = set()
    for edge_id in order:
        task = edge_task_map[edge_id]
        visited_nodes.add(task.u)
        visited_nodes.add(task.v)

    return {
        'total_edges': len(edge_tasks),
        'visited_edges': len(visited_edges),
        'visited_nodes': len(visited_nodes),
        'continuous_transitions': continuous_count,
        'continuous_ratio': continuous_count / len(order) if len(order) > 0 else 0,
        'total_length': total_length,
        'avg_edge_length': total_length / len(edge_tasks) if edge_tasks else 0
    }


# =====================================================
# 最小可运行 Topo Planner (Greedy 版本)
# =====================================================
# 设计目标：先跑通流程，再优化
# 策略：贪心 - 优先访问相邻边，无相邻边时跳转至最近边
# =====================================================

from dataclasses import dataclass, field


@dataclass
class MissionStep:
    """
    任务步骤：访问一个 EdgeTask
    """
    edge_id: str                          # 要访问的边ID
    from_edge_id: str = None              # 从哪条边过来（None表示起点）
    transition_cost: float = 0.0          # 到达这条边的转移代价
    is_adjacent: bool = False             # 是否与前一条边相邻
    graph_path: list = None               # 如果不相邻，图上的路径（node IDs）

    def __post_init__(self):
        if self.graph_path is None:
            self.graph_path = []


@dataclass
class TopoMission:
    """
    拓扑任务：EdgeTask 的访问序列
    """
    steps: list = field(default_factory=list)
    total_cost: float = 0.0
    start_edge_id: str = None
    edge_adjacency: dict = field(default_factory=dict)

    def add_step(self, step: MissionStep):
        """添加一个步骤"""
        self.steps.append(step)
        self.total_cost += step.transition_cost
        if self.start_edge_id is None:
            self.start_edge_id = step.edge_id

    def get_visit_sequence(self) -> list:
        """获取访问顺序（边ID列表）"""
        return [step.edge_id for step in self.steps]

    def get_visited_edges(self) -> set:
        """获取已访问的边"""
        return set(step.edge_id for step in self.steps)

    def summary(self) -> str:
        """摘要信息"""
        n_adjacent = sum(1 for s in self.steps if s.is_adjacent)
        n_jumps = max(0, len(self.steps) - n_adjacent - 1)  # -1 for start
        return (
            f"TopoMission:\n"
            f"  Steps: {len(self.steps)}\n"
            f"  Total cost: {self.total_cost:.1f}px\n"
            f"  Adjacent transitions: {n_adjacent}\n"
            f"  Jump transitions: {n_jumps}\n"
            f"  Start edge: {self.start_edge_id}"
        )


def build_edge_adjacency_simple(topo_graph: TopoGraph) -> dict:
    """
    构建边邻接表（简单版本）

    规则：如果两条边共享至少一个节点，则视为相邻

    Args:
        topo_graph: 拓扑图

    Returns:
        dict: {edge_id: [neighbor_edge_ids]}
    """
    print("[Edge Adjacency] 构建边邻接表...")

    # 构建每个节点的出边列表
    node_to_edges = {}

    for edge_id, edge in topo_graph.edges.items():
        if edge.u not in node_to_edges:
            node_to_edges[edge.u] = []
        node_to_edges[edge.u].append(edge_id)

        if edge.v not in node_to_edges:
            node_to_edges[edge.v] = []
        node_to_edges[edge.v].append(edge_id)

    # 构建边邻接表
    adjacency = {}

    for edge_id, edge in topo_graph.edges.items():
        neighbors = set()

        # 通过起始节点找到的邻居
        if edge.u in node_to_edges:
            neighbors.update(node_to_edges[edge.u])

        # 通过结束节点找到的邻居
        if edge.v in node_to_edges:
            neighbors.update(node_to_edges[edge.v])

        # 移除自己
        neighbors.discard(edge_id)

        adjacency[edge_id] = list(neighbors)

    # 统计
    total_neighbors = sum(len(v) for v in adjacency.values())
    avg_neighbors = total_neighbors / len(adjacency) if adjacency else 0

    print(f"  [Edge Adjacency] 完成: {len(adjacency)} 条边")
    print(f"  [Edge Adjacency] 平均邻居数: {avg_neighbors:.1f}")

    return adjacency


def compute_transition_cost_simple(
    topo_graph: TopoGraph,
    edge_a_id: str,
    edge_b_id: str,
    adjacency: dict
) -> tuple:
    """
    计算从边 a 到边 b 的转移代价（简单版本）

    规则：
    - 如果相邻：cost = 0
    - 如果不相邻：cost = 图上最短路径长度（像素距离）
    - 如果不连通：cost = 边中心点直接欧氏距离（fallback）

    Args:
        topo_graph: 拓扑图
        edge_a_id: 起始边ID
        edge_b_id: 目标边ID
        adjacency: 边邻接表

    Returns:
        tuple: (代价, 是否相邻, 图路径节点列表)
    """
    # 检查是否相邻
    if edge_b_id in adjacency.get(edge_a_id, []):
        return 0.0, True, None

    # 不相邻，计算图上最短路径
    edge_a = topo_graph.edges.get(edge_a_id)
    edge_b = topo_graph.edges.get(edge_b_id)

    if edge_a is None or edge_b is None:
        return float('inf'), False, None

    # 尝试四种组合：a.u -> b.u, a.u -> b.v, a.v -> b.u, a.v -> b.v
    min_cost = float('inf')
    best_path = None

    for start_node in [edge_a.u, edge_a.v]:
        for end_node in [edge_b.u, edge_b.v]:
            path = get_shortest_path(topo_graph, start_node, end_node)
            if not path:
                continue

            # 计算路径长度（像素距离）
            path_length = 0.0
            for i in range(len(path) - 1):
                u_node = topo_graph.get_node(path[i])
                v_node = topo_graph.get_node(path[i + 1])
                if u_node and v_node:
                    path_length += np.linalg.norm(
                        np.array(u_node.pos2d) - np.array(v_node.pos2d)
                    )

            if path_length < min_cost:
                min_cost = path_length
                best_path = path

    # 如果图不连通，使用欧氏距离作为fallback
    if min_cost == float('inf') or best_path is None:
        # 计算两条边中心点的欧氏距离
        polyline_a = np.array(edge_a.polyline)
        polyline_b = np.array(edge_b.polyline)
        center_a = polyline_a[len(polyline_a) // 2]
        center_b = polyline_b[len(polyline_b) // 2]
        min_cost = np.linalg.norm(center_a - center_b)
        # 路径为空列表表示直接跳跃
        best_path = []

    return min_cost, False, best_path


def plan_topo_mission_greedy(
    topo_graph: TopoGraph,
    edge_tasks: list,
    start_edge_id: str = None
) -> TopoMission:
    """
    贪心策略规划拓扑任务访问顺序

    策略：
    1. 从起始边开始
    2. 每一步优先选择"未访问且相邻"的边
    3. 如果没有未访问的相邻边，选择 transition cost 最小的未访问边
    4. 直到所有边都被访问

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        start_edge_id: 起始边ID（可选，默认第一个）

    Returns:
        TopoMission: 任务规划结果
    """
    print("\n" + "="*60)
    print("[Topo Planner Greedy] 开始规划拓扑任务...")
    print("="*60)

    # 构建边邻接表
    adjacency = build_edge_adjacency_simple(topo_graph)

    # 初始化
    mission = TopoMission(edge_adjacency=adjacency)

    all_edge_ids = [task.edge_id for task in edge_tasks]
    unvisited = set(all_edge_ids)

    # 选择起始边
    if start_edge_id is None:
        start_edge_id = all_edge_ids[0] if all_edge_ids else None

    if start_edge_id is None:
        print("[Topo Planner Greedy] ERROR: 没有可用的边任务")
        return mission

    # 添加起始步骤
    mission.add_step(MissionStep(
        edge_id=start_edge_id,
        from_edge_id=None,
        transition_cost=0.0,
        is_adjacent=False
    ))
    unvisited.remove(start_edge_id)

    current_edge_id = start_edge_id

    print(f"[Topo Planner Greedy] 起始边: {current_edge_id}")
    print(f"[Topo Planner Greedy] 待访问边数: {len(unvisited)}")

    # 贪心策略：每次选择最优的下一条边
    step_count = 1
    while unvisited:
        # 1. 优先查找未访问的相邻边
        neighbors = adjacency.get(current_edge_id, [])
        unvisited_neighbors = [e for e in neighbors if e in unvisited]

        next_edge_id = None
        is_adjacent = False
        transition_cost = 0.0
        graph_path = None

        if unvisited_neighbors:
            # 有未访问的相邻边，选第一个
            next_edge_id = unvisited_neighbors[0]
            is_adjacent = True
            transition_cost = 0.0
            graph_path = None

            print(f"  Step {step_count}: {current_edge_id} -> {next_edge_id} (相邻)")
        else:
            # 没有相邻的未访问边，找 transition cost 最小的
            min_cost = float('inf')

            for candidate in unvisited:
                cost, adjacent, path = compute_transition_cost_simple(
                    topo_graph, current_edge_id, candidate, adjacency
                )

                if cost < min_cost:
                    min_cost = cost
                    next_edge_id = candidate
                    is_adjacent = adjacent
                    graph_path = path

            if next_edge_id is not None:
                transition_cost = min_cost
                if graph_path and len(graph_path) > 0:
                    path_str = " -> ".join(graph_path[-3:]) if len(graph_path) > 2 else " -> ".join(graph_path)
                else:
                    path_str = "direct"
                print(f"  Step {step_count}: {current_edge_id} -> {next_edge_id} (跳转 via [{path_str}], cost={transition_cost:.1f}px)")

        # 添加步骤
        if next_edge_id is not None:
            mission.add_step(MissionStep(
                edge_id=next_edge_id,
                from_edge_id=current_edge_id,
                transition_cost=transition_cost,
                is_adjacent=is_adjacent,
                graph_path=graph_path
            ))
            unvisited.remove(next_edge_id)
            current_edge_id = next_edge_id
            step_count += 1
        else:
            print(f"[Topo Planner Greedy] WARNING: 无法找到下一条边，剩余 {len(unvisited)} 条边未访问")
            break

    print(f"\n[Topo Planner Greedy] 规划完成:")
    print(f"  - 访问边数: {len(mission.steps)} / {len(all_edge_ids)}")
    print(f"  - 总代价: {mission.total_cost:.1f}px")
    print(f"  - 覆盖率: {len(mission.steps) / len(all_edge_ids) * 100:.1f}%")

    return mission


def visualize_topo_plan_greedy(
    topo_graph: TopoGraph,
    edge_tasks: list,
    mission: TopoMission,
    output_path: str
):
    """
    可视化拓扑任务规划结果（贪心版本）

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        mission: 任务规划结果
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(16, 16))

    # 绘制拓扑边（灰色底）
    for edge in topo_graph.edges.values():
        polyline = np.array(edge.polyline)
        ax.plot(polyline[:, 0], polyline[:, 1], color='lightgray',
               linewidth=3, alpha=0.5, zorder=1)

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'purple'
            marker = 's'
            size = 50
        elif node.kind == 'split':
            color = 'darkorange'
            marker = '^'
            size = 60
        else:
            color = 'gray'
            marker = 'o'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=3,
                  edgecolors='black', linewidths=1)

    # 按访问顺序绘制边
    visit_order = {edge_id: i for i, edge_id in enumerate(mission.get_visit_sequence())}

    for idx, step in enumerate(mission.steps):
        edge = topo_graph.edges.get(step.edge_id)
        if edge is None:
            continue

        polyline = np.array(edge.polyline)

        # 颜色根据访问顺序渐变
        visit_idx = visit_order.get(step.edge_id, 0)
        color = plt.cm.viridis(visit_idx / max(len(mission.steps), 1))

        # 绘制边
        linewidth = 4 if step.edge_id == mission.start_edge_id else 3
        ax.plot(polyline[:, 0], polyline[:, 1], color=color,
               linewidth=linewidth, alpha=0.8, zorder=2)

        # 显示访问序号
        mid_idx = len(polyline) // 2
        mid_x, mid_y = polyline[mid_idx]
        ax.text(mid_x, mid_y, str(visit_idx + 1),
               fontsize=10, ha='center', va='center',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                        edgecolor=color, linewidth=2, alpha=0.9),
               zorder=5)

        # 如果不是相邻转移，绘制跳转路径
        if step.from_edge_id and not step.is_adjacent and step.graph_path:
            from_edge = topo_graph.edges.get(step.from_edge_id)
            if from_edge:
                # 获取起点和终点
                from_polyline = np.array(from_edge.polyline)
                from_point = from_polyline[len(from_polyline) // 2]

                to_polyline = np.array(edge.polyline)
                to_point = to_polyline[len(to_polyline) // 2]

                # 绘制跳转箭头
                ax.annotate('', xy=to_point, xytext=from_point,
                          arrowprops=dict(arrowstyle='->', color='red',
                                        lw=2, alpha=0.6, linestyle='--'),
                          zorder=4)

    # 标记起点和终点
    if mission.start_edge_id:
        start_edge = topo_graph.edges.get(mission.start_edge_id)
        if start_edge:
            polyline = np.array(start_edge.polyline)
            mid_x, mid_y = polyline[len(polyline) // 2]
            ax.scatter(mid_x, mid_y, c='green', marker='*', s=200,
                      zorder=6, edgecolors='black', linewidths=2,
                      label='Start')

    if mission.steps:
        last_edge = topo_graph.edges.get(mission.steps[-1].edge_id)
        if last_edge:
            polyline = np.array(last_edge.polyline)
            mid_x, mid_y = polyline[len(polyline) // 2]
            ax.scatter(mid_x, mid_y, c='red', marker='X', s=200,
                      zorder=6, edgecolors='black', linewidths=2,
                      label='End')

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='s', color='w', markerfacecolor='purple',
               markersize=10, label='Endpoint Node'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='darkorange',
               markersize=10, label='Split Node'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
               markersize=15, label='Start Edge'),
        Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
               markersize=10, label='End Edge'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    ax.set_title(f'Topology Mission Plan - Greedy ({len(mission.steps)} edges, cost={mission.total_cost:.1f}px)')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 规划图已保存: {output_path}")


def print_edge_adjacency_summary(adjacency: dict):
    """打印边邻接表摘要"""
    print("\n[Edge Adjacency Summary]")
    print("-" * 60)

    for edge_id, neighbors in sorted(adjacency.items()):
        neighbor_str = ", ".join(neighbors) if neighbors else "(no neighbors)"
        print(f"  {edge_id}: [{neighbor_str}]")

    print()


def print_mission_details(mission: TopoMission):
    """打印任务详情"""
    print("\n[Mission Details]")
    print("-" * 60)

    for i, step in enumerate(mission.steps):
        if step.from_edge_id is None:
            print(f"  Step {i}: [START] {step.edge_id}")
        elif step.is_adjacent:
            print(f"  Step {i}: [{step.from_edge_id}] -> {step.edge_id} (adjacent, cost={step.transition_cost:.1f})")
        else:
            if step.graph_path and len(step.graph_path) > 0:
                path_str = " -> ".join(step.graph_path[-3:]) if len(step.graph_path) > 2 else " -> ".join(step.graph_path)
            else:
                path_str = "direct"
            print(f"  Step {i}: [{step.from_edge_id}] -> {step.edge_id} (jump via [{path_str}], cost={step.transition_cost:.1f}px)")

    print()


# =====================================================
# 连续完整航迹规划器 (Continuous Trajectory Planner)
# =====================================================
# 目标：输出一条真实连续的 mission path
# 包含：inspect segment + connect segment + inspect segment + ...
# =====================================================

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Literal


@dataclass
class MissionSegment:
    """
    任务段：连续航迹的基本单元
    """
    type: Literal['inspect', 'connect']  # 段类型
    edge_id: str = None                   # 边ID（inspect段专用）
    from_edge_id: str = None              # 源边ID（connect段专用）
    to_edge_id: str = None                # 目标边ID（connect段专用）
    geometry: List[Tuple[float, float]] = field(default_factory=list)  # 几何路径
    length: float = 0.0                   # 长度
    direction: Literal['forward', 'reverse'] = 'forward'  # 巡检方向（inspect段专用）

    def summary(self) -> str:
        """摘要信息"""
        if self.type == 'inspect':
            dir_str = f"({self.direction})" if hasattr(self, 'direction') else ""
            return f"Inspect {self.edge_id} {dir_str}, len={self.length:.1f}px"
        else:
            return f"Connect {self.from_edge_id} -> {self.to_edge_id}, len={self.length:.1f}px"


@dataclass
class ContinuousMission:
    """
    连续任务：完整航迹
    """
    segments: List[MissionSegment] = field(default_factory=list)
    full_path: List[Tuple[float, float]] = field(default_factory=list)
    total_length: float = 0.0
    inspect_length: float = 0.0
    connect_length: float = 0.0
    start_edge_id: str = None
    visit_order: List[str] = field(default_factory=list)  # 访问顺序（带方向）

    def add_segment(self, segment: MissionSegment):
        """添加一个段"""
        self.segments.append(segment)
        self.total_length += segment.length

        if segment.type == 'inspect':
            self.inspect_length += segment.length
            if self.start_edge_id is None:
                self.start_edge_id = segment.edge_id
            # 记录访问顺序：edge_id + 方向
            direction_mark = '+' if segment.direction == 'forward' else '-'
            self.visit_order.append(f"{segment.edge_id}{direction_mark}")
        else:
            self.connect_length += segment.length

        # 添加到完整路径
        self.full_path.extend(segment.geometry)

    def summary(self) -> str:
        """摘要信息"""
        n_inspect = sum(1 for s in self.segments if s.type == 'inspect')
        n_connect = sum(1 for s in self.segments if s.type == 'connect')
        connect_ratio = self.connect_length / self.total_length * 100 if self.total_length > 0 else 0

        return (
            f"ContinuousMission:\n"
            f"  Total segments: {len(self.segments)} (inspect={n_inspect}, connect={n_connect})\n"
            f"  Total length: {self.total_length:.1f}px\n"
            f"  Inspect length: {self.inspect_length:.1f}px ({100-connect_ratio:.1f}%)\n"
            f"  Connect length: {self.connect_length:.1f}px ({connect_ratio:.1f}%)\n"
            f"  Start edge: {self.start_edge_id}"
        )


def get_edge_geometry_with_direction(edge_task, direction: str = 'forward') -> List[Tuple[float, float]]:
    """
    获取指定方向的 edge geometry

    Args:
        edge_task: 边任务
        direction: 方向 ('forward' 或 'reverse')

    Returns:
        List[Tuple[float, float]]: 几何路径
    """
    polyline = edge_task.polyline

    if direction == 'reverse':
        # 反向：翻转 polyline
        return list(reversed(polyline))
    else:
        # 正向：保持原样
        return list(polyline)


def find_nearest_topo_node(point: Tuple[float, float], topo_graph: TopoGraph) -> str:
    """
    找到距离给定点最近的拓扑节点

    Args:
        point: (x, y) 坐标
        topo_graph: 拓扑图

    Returns:
        str: 最近的节点ID
    """
    min_dist = float('inf')
    nearest_node = None

    for node_id, node in topo_graph.nodes.items():
        dist = np.linalg.norm(np.array(node.pos2d) - np.array(point))
        if dist < min_dist:
            min_dist = dist
            nearest_node = node_id

    return nearest_node


def generate_connection_segment_along_topo(
    point_a: Tuple[float, float],
    point_b: Tuple[float, float],
    topo_graph: TopoGraph,
    edge_task_map: dict
) -> Tuple[List[Tuple[float, float]], float]:
    """
    沿拓扑图生成连接段（替代简单的直线连接）

    Args:
        point_a: 起点
        point_b: 终点
        topo_graph: 拓扑图
        edge_task_map: 边任务映射

    Returns:
        Tuple[List[Tuple[float, float]], float]: (几何路径, 长度)
    """
    # 1. 找到最近的拓扑节点
    node_a = find_nearest_topo_node(point_a, topo_graph)
    node_b = find_nearest_topo_node(point_b, topo_graph)

    if node_a is None or node_b is None:
        print(f"[WARNING] 无法找到最近的拓扑节点，退回到直线连接")
        print(f"  point_a: {point_a}, point_b: {point_b}")
        geometry = [point_a, point_b]
        length = np.linalg.norm(np.array(point_b) - np.array(point_a))
        return geometry, length

    # 2. 获取拓扑图最短路径（使用邻近性增强）
    # 使用动态阈值：至少是两点间直线距离的1.2倍，确保能找到路径
    direct_distance = np.linalg.norm(np.array(point_b) - np.array(point_a))
    proximity_threshold = max(1500.0, direct_distance * 1.2)

    node_path = get_shortest_path_with_proximity(topo_graph, node_a, node_b, proximity_threshold=proximity_threshold)

    if not node_path:
        print(f"[WARNING] 拓扑图中节点 {node_a} 和 {node_b} 不连通，退回到直线连接")
        print(f"  point_a: {point_a}")
        print(f"  point_b: {point_b}")
        geometry = [point_a, point_b]
        length = np.linalg.norm(np.array(point_b) - np.array(point_a))
        return geometry, length

    # 调试：显示路径信息
    print(f"[DEBUG] generate_connection_segment_along_topo called:")
    print(f"  point_a: ({point_a[0]:.1f}, {point_a[1]:.1f})")
    print(f"  point_b: ({point_b[0]:.1f}, {point_b[1]:.1f})")
    print(f"  node_a: {node_a}")
    print(f"  node_b: {node_b}")
    print(f"  node_path: {node_path} (length={len(node_path)})")

    # 3. 将节点路径展开为 polyline geometry
    geometry = []

    # 添加起点
    geometry.append(point_a)

    # 如果起点不是最近的节点，添加连接段
    node_a_obj = topo_graph.nodes[node_a]
    if np.linalg.norm(np.array(point_a) - np.array(node_a_obj.pos2d)) > 1.0:
        geometry.append(node_a_obj.pos2d)

    # 遍历节点路径，展开每条边的 geometry
    for i in range(len(node_path) - 1):
        u = node_path[i]
        v = node_path[i + 1]

        # 找到连接 u 和 v 的拓扑边
        edge = None
        for edge_id, e in topo_graph.edges.items():
            if (e.u == u and e.v == v) or (e.u == v and e.v == u):
                edge = e
                break

        if edge and edge.polyline:
            # 确定方向
            if edge.u == u and edge.v == v:
                # 正向
                poly = edge.polyline
            else:
                # 反向
                poly = list(reversed(edge.polyline))

            # 添加边的 geometry（避免重复点）
            if geometry:
                last_point = geometry[-1]
                if np.linalg.norm(np.array(poly[0]) - np.array(last_point)) > 0.1:
                    geometry.extend(poly)
                else:
                    # 跳过重复点
                    geometry.extend(poly[1:])
            else:
                geometry.extend(poly)
        else:
            # 没有找到边，使用节点位置
            if i < len(node_path) - 1:
                next_node = topo_graph.nodes[node_path[i + 1]]
                geometry.append(next_node.pos2d)

    # 如果终点不是最近的节点，添加连接段
    node_b_obj = topo_graph.nodes[node_b]
    if np.linalg.norm(np.array(point_b) - np.array(node_b_obj.pos2d)) > 1.0:
        geometry.append(node_b_obj.pos2d)

    # 添加终点
    geometry.append(point_b)

    # 4. 计算路径长度
    length = 0.0
    for i in range(len(geometry) - 1):
        length += np.linalg.norm(np.array(geometry[i + 1]) - np.array(geometry[i]))

    # 调试输出：显示长连接的详细信息
    if length > 50:  # 降低阈值，看看更多细节
        print(f"[DEBUG] Connection segment:")
        print(f"  Start: ({point_a[0]:.1f}, {point_a[1]:.1f})")
        print(f"  End: ({point_b[0]:.1f}, {point_b[1]:.1f})")
        print(f"  Node path: {node_path} (length={len(node_path)})")
        print(f"  Geometry points: {len(geometry)}")
        print(f"  Length: {length:.1f}px")
        if len(geometry) <= 3:
            print(f"  WARNING: Geometry only has {len(geometry)} points!")
        print()

    # 5. 验证路径合理性
    if len(geometry) < 2:
        print(f"[ERROR] 生成的连接段几何点数少于2: {len(geometry)}")
        print(f"  node_path: {node_path}")
        print(f"  退回到直线连接")
        geometry = [point_a, point_b]
        length = np.linalg.norm(np.array(point_b) - np.array(point_a))

    return geometry, length


def generate_connection_segment(point_a: Tuple[float, float],
                                point_b: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
    """
    生成连接段（简单直线版本）- 已废弃，保留用于向后兼容

    Args:
        point_a: 起点
        point_b: 终点

    Returns:
        Tuple[List[Tuple[float, float]], float]: (几何路径, 长度)
    """
    geometry = [point_a, point_b]
    length = np.linalg.norm(np.array(point_b) - np.array(point_a))
    return geometry, length


def evaluate_transition_with_geometry(
    topo_graph: TopoGraph,
    current_edge_id: str,
    current_direction: str,
    candidate_edge_id: str,
    edge_task_map: dict
) -> dict:
    """
    评估转移到下一条边的完整方案（考虑方向和连接）

    Args:
        topo_graph: 拓扑图
        current_edge_id: 当前边ID
        current_direction: 当前方向
        candidate_edge_id: 候选边ID
        edge_task_map: 边任务映射

    Returns:
        dict: 转移方案评估结果
    """
    current_edge = edge_task_map.get(current_edge_id)
    candidate_edge = edge_task_map.get(candidate_edge_id)

    if current_edge is None or candidate_edge is None:
        return None

    # 获取当前边的结束点
    current_geo = get_edge_geometry_with_direction(current_edge, current_direction)
    current_end_point = current_geo[-1] if current_geo else None

    # 尝试两种方向，选择更优的
    best_plan = None
    min_cost = float('inf')

    for next_direction in ['forward', 'reverse']:
        # 获取下一条边的起始点
        next_geo = get_edge_geometry_with_direction(candidate_edge, next_direction)
        next_start_point = next_geo[0] if next_geo else None

        if current_end_point is None or next_start_point is None:
            continue

        # 生成连接段（沿拓扑图）
        connect_geo, connect_len = generate_connection_segment_along_topo(
            current_end_point, next_start_point, topo_graph, edge_task_map
        )

        # 总增量代价 = 连接长度 + 下一条边长度
        total_incremental_cost = connect_len + candidate_edge.len2d

        if total_incremental_cost < min_cost:
            min_cost = total_incremental_cost
            best_plan = {
                'next_edge_id': candidate_edge_id,
                'next_direction': next_direction,
                'connect_geometry': connect_geo,
                'connect_length': connect_len,
                'total_incremental_cost': total_incremental_cost,
                'current_end_point': current_end_point,
                'next_start_point': next_start_point
            }

    return best_plan


def choose_next_edge_greedy(
    topo_graph: TopoGraph,
    current_edge_id: str,
    current_direction: str,
    unvisited: set,
    edge_task_map: dict,
    adjacency: dict
) -> dict:
    """
    贪心选择下一条最优边

    策略：
    1. 优先选择未访问的相邻边
    2. 如果没有相邻边，选择 total_incremental_cost 最小的边

    Args:
        topo_graph: 拓扑图
        current_edge_id: 当前边ID
        current_direction: 当前方向
        unvisited: 未访问边集合
        edge_task_map: 边任务映射
        adjacency: 边邻接表

    Returns:
        dict: 最优转移方案
    """
    neighbors = adjacency.get(current_edge_id, [])
    unvisited_neighbors = [e for e in neighbors if e in unvisited]

    candidates = unvisited_neighbors if unvisited_neighbors else list(unvisited)

    best_plan = None
    min_cost = float('inf')

    for candidate_id in candidates:
        plan = evaluate_transition_with_geometry(
            topo_graph, current_edge_id, current_direction, candidate_id, edge_task_map
        )

        if plan is None:
            continue

        # 优先选择相邻边（cost 小得多）
        if candidate_id in unvisited_neighbors:
            # 相邻边，降低连接代价的权重
            adjusted_cost = plan['connect_length']
        else:
            adjusted_cost = plan['total_incremental_cost']

        if adjusted_cost < min_cost:
            min_cost = adjusted_cost
            best_plan = plan

    return best_plan


def build_mission_segments(
    topo_graph: TopoGraph,
    edge_tasks: list,
    adjacency: dict,
    start_edge_id: str = None
) -> ContinuousMission:
    """
    构建连续任务的 mission segments

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        adjacency: 边邻接表
        start_edge_id: 起始边ID（可选）

    Returns:
        ContinuousMission: 连续任务
    """
    print("\n" + "="*60)
    print("[Continuous Planner] 开始构建连续航迹...")
    print("="*60)

    mission = ContinuousMission()

    # 构建边任务映射
    edge_task_map = {task.edge_id: task for task in edge_tasks}
    all_edge_ids = list(edge_task_map.keys())
    unvisited = set(all_edge_ids)

    # 选择起始边
    if start_edge_id is None:
        start_edge_id = all_edge_ids[0] if all_edge_ids else None

    if start_edge_id is None:
        print("[Continuous Planner] ERROR: 没有可用的边任务")
        return mission

    # 选择起始边的方向（优先从 endpoint 开始）
    start_edge = edge_task_map[start_edge_id]
    start_node = topo_graph.get_node(start_edge.u)
    end_node = topo_graph.get_node(start_edge.v)

    # 选择度数=1 的节点作为起点
    if start_node and start_node.deg == 1:
        start_direction = 'forward'  # u -> v
    elif end_node and end_node.deg == 1:
        start_direction = 'reverse'  # v -> u
    else:
        start_direction = 'forward'  # 默认正向

    print(f"[Continuous Planner] 起始边: {start_edge_id} ({start_direction})")
    print(f"[Continuous Planner] 待访问边数: {len(unvisited)}")

    # 添加起始 inspect segment
    start_geo = get_edge_geometry_with_direction(start_edge, start_direction)
    mission.add_segment(MissionSegment(
        type='inspect',
        edge_id=start_edge_id,
        geometry=start_geo,
        length=start_edge.len2d,
        direction=start_direction
    ))
    unvisited.remove(start_edge_id)

    current_edge_id = start_edge_id
    current_direction = start_direction
    segment_count = 1

    # 贪心构建 segments
    while unvisited:
        # 选择下一条最优边
        plan = choose_next_edge_greedy(
            topo_graph, current_edge_id, current_direction, unvisited, edge_task_map, adjacency
        )

        if plan is None:
            print(f"[Continuous Planner] WARNING: 无法找到下一条边，剩余 {len(unvisited)} 条边未访问")
            break

        next_edge_id = plan['next_edge_id']
        next_direction = plan['next_direction']
        connect_geo = plan['connect_geometry']
        connect_len = plan['connect_length']

        # 添加 connect segment
        mission.add_segment(MissionSegment(
            type='connect',
            from_edge_id=current_edge_id,
            to_edge_id=next_edge_id,
            geometry=connect_geo,
            length=connect_len
        ))
        segment_count += 1

        # 添加下一条边的 inspect segment
        next_edge = edge_task_map[next_edge_id]
        next_geo = get_edge_geometry_with_direction(next_edge, next_direction)

        mission.add_segment(MissionSegment(
            type='inspect',
            edge_id=next_edge_id,
            geometry=next_geo,
            length=next_edge.len2d,
            direction=next_direction
        ))
        unvisited.remove(next_edge_id)
        segment_count += 1

        # 打印进度
        adj_mark = "(adj)" if connect_len < 10 else f"(connect={connect_len:.1f}px)"
        dir_mark = '+' if next_direction == 'forward' else '-'
        print(f"  Segment {segment_count-1}: Connect [{current_edge_id}{dir_mark}] -> [{next_edge_id}{next_direction}] {adj_mark}")
        print(f"  Segment {segment_count}: Inspect {next_edge_id} ({next_direction}), len={next_edge.len2d:.1f}px")

        current_edge_id = next_edge_id
        current_direction = next_direction

    print(f"\n[Continuous Planner] 航迹构建完成:")
    print(f"  - 总段数: {len(mission.segments)}")
    print(f"  - 访问边数: {len(mission.visit_order)} / {len(all_edge_ids)}")
    print(f"  - 总长度: {mission.total_length:.1f}px")
    print(f"  - 巡检长度: {mission.inspect_length:.1f}px")
    print(f"  - 连接长度: {mission.connect_length:.1f}px")

    return mission


def flatten_mission_segments_to_path(mission: ContinuousMission) -> List[Tuple[float, float]]:
    """
    将 mission segments 展平为完整连续路径

    Args:
        mission: 连续任务

    Returns:
        List[Tuple[float, float]]: 完整路径点序列
    """
    # 生成完整路径（去除相邻重复点）
    full_path = []

    for segment in mission.segments:
        for point in segment.geometry:
            if not full_path or np.linalg.norm(np.array(point) - np.array(full_path[-1])) > 0.1:
                full_path.append(point)

    return full_path


def visualize_topo_plan_continuous(
    topo_graph: TopoGraph,
    edge_tasks: list,
    mission: ContinuousMission,
    output_path: str
):
    """
    可视化连续航迹规划结果

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        mission: 连续任务
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(16, 16))

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'purple'
            marker = 's'
            size = 50
        elif node.kind == 'split':
            color = 'darkorange'
            marker = '^'
            size = 60
        else:
            color = 'gray'
            marker = 'o'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=3,
                  edgecolors='black', linewidths=1)

    # 绘制 segments
    inspect_count = 0
    visit_order_map = {}

    for segment in mission.segments:
        if segment.type == 'inspect':
            inspect_count += 1
            visit_order_map[segment.edge_id] = inspect_count

    for segment in mission.segments:
        if segment.type == 'inspect':
            # 巡检段：粗实线，按访问顺序渐变色
            visit_idx = visit_order_map.get(segment.edge_id, 0)
            color = plt.cm.viridis(visit_idx / max(inspect_count, 1))

            geometry = np.array(segment.geometry)
            ax.plot(geometry[:, 0], geometry[:, 1], color=color,
                   linewidth=4, alpha=0.9, zorder=2,
                   label=f'Inspect {segment.edge_id}' if segment.edge_id == mission.start_edge_id else "")

            # 显示访问序号和方向
            mid_idx = len(geometry) // 2
            mid_x, mid_y = geometry[mid_idx]
            dir_mark = '→' if segment.direction == 'forward' else '←'
            ax.text(mid_x, mid_y, f"{visit_idx}{dir_mark}",
                   fontsize=10, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                            edgecolor=color, linewidth=2, alpha=0.9),
                   zorder=5)

        else:  # connect
            # 连接段：红色虚线
            geometry = np.array(segment.geometry)
            ax.plot(geometry[:, 0], geometry[:, 1], color='red',
                   linewidth=2, alpha=0.6, linestyle='--', zorder=1)

            # 显示连接长度
            if len(geometry) >= 2:
                mid_idx = len(geometry) // 2
                mid_x, mid_y = geometry[mid_idx]
                ax.text(mid_x, mid_y, f"{segment.length:.0f}px",
                       fontsize=7, ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow',
                                alpha=0.7, edgecolor='red'),
                       zorder=4)

    # 标记起点和终点
    if mission.segments:
        # 起点：第一个 segment 的起点
        first_segment = mission.segments[0]
        if first_segment.geometry:
            start_point = first_segment.geometry[0]
            ax.scatter(start_point[0], start_point[1], c='green', marker='*', s=200,
                      zorder=6, edgecolors='black', linewidths=2)

        # 终点：最后一个 segment 的终点
        last_segment = mission.segments[-1]
        if last_segment.geometry:
            end_point = last_segment.geometry[-1]
            ax.scatter(end_point[0], end_point[1], c='red', marker='X', s=200,
                      zorder=6, edgecolors='black', linewidths=2)

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', linewidth=2, linestyle='--', label='Connect Segment'),
        Line2D([0], [0], color='purple', linewidth=4, label='Inspect Segment'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='purple',
               markersize=10, label='Endpoint Node'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='darkorange',
               markersize=10, label='Split Node'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
               markersize=15, label='Start'),
        Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
               markersize=10, label='End'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    ax.set_title(f'Continuous Topo Mission ({len(mission.visit_order)} edges, '
                 f'total={mission.total_length:.0f}px, '
                 f'inspect={mission.inspect_length:.0f}px, '
                 f'connect={mission.connect_length:.0f}px)')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 连续航迹图已保存: {output_path}")


def print_mission_segments_detail(mission: ContinuousMission):
    """打印 mission segments 详情"""
    print("\n[Mission Segments Detail]")
    print("-" * 70)

    for i, segment in enumerate(mission.segments):
        prefix = f"Segment {i+1:02d}"
        print(f"  {prefix}: {segment.summary()}")

    print()


def print_mission_statistics(mission: ContinuousMission):
    """打印任务统计信息"""
    print("\n[Mission Statistics]")
    print("-" * 70)

    n_inspect = sum(1 for s in mission.segments if s.type == 'inspect')
    n_connect = sum(1 for s in mission.segments if s.type == 'connect')
    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0

    print(f"  总段数: {len(mission.segments)}")
    print(f"    - Inspect segments: {n_inspect}")
    print(f"    - Connect segments: {n_connect}")
    print()
    print(f"  总长度: {mission.total_length:.1f}px")
    print(f"    - Inspect length: {mission.inspect_length:.1f}px ({100-connect_ratio:.1f}%)")
    print(f"    - Connect length: {mission.connect_length:.1f}px ({connect_ratio:.1f}%)")
    print()
    print(f"  起始边: {mission.start_edge_id}")
    print(f"  访问顺序: {' -> '.join(mission.visit_order)}")
    print()


# =====================================================
# 分组感知连续航迹规划器 (Group-Aware Continuous Planner)
# =====================================================
# 目标：基于区域分组的连续航迹规划
# 策略：
#   1. 对 edges 进行空间分组
#   2. 先确定 group 访问顺序
#   3. 每个 group 内连续完成
#   4. 减少 inter-group long distance connects
# =====================================================

# 延迟导入 DBSCAN（仅在需要时导入，避免模块顶层导入失败）
try:
    from sklearn.cluster import DBSCAN
    _HAS_SKLEARN = True
except ImportError:
    _HAS_SKLEARN = False
    DBSCAN = None

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Literal, Optional


@dataclass
class EdgeGroup:
    """
    边分组：空间上接近的 edges 集合
    """
    group_id: str
    edge_ids: List[str] = field(default_factory=list)
    centroid: Tuple[float, float] = None
    bbox: Tuple[float, float, float, float] = None  # (min_x, min_y, max_x, max_y)
    total_inspect_length: float = 0.0


@dataclass
class GroupedContinuousMission(ContinuousMission):
    """
    分组连续任务：扩展版 ContinuousMission
    """
    groups: Dict[str, EdgeGroup] = field(default_factory=dict)
    group_visit_order: List[str] = field(default_factory=list)
    intra_group_connect_length: float = 0.0
    inter_group_connect_length: float = 0.0

    def add_group(self, group: EdgeGroup):
        """添加一个分组"""
        self.groups[group.group_id] = group

    def summary_with_groups(self) -> str:
        """带分组信息的摘要"""
        n_inspect = sum(1 for s in self.segments if s.type == 'inspect')
        n_connect = sum(1 for s in self.segments if s.type == 'connect')
        connect_ratio = self.connect_length / self.total_length * 100 if self.total_length > 0 else 0

        return (
            f"GroupedContinuousMission:\n"
            f"  Groups: {len(self.groups)}\n"
            f"  Group visit order: {' -> '.join(self.group_visit_order)}\n"
            f"  Total segments: {len(self.segments)} (inspect={n_inspect}, connect={n_connect})\n"
            f"  Total length: {self.total_length:.1f}px\n"
            f"  Inspect length: {self.inspect_length:.1f}px ({100-connect_ratio:.1f}%)\n"
            f"  Connect length: {self.connect_length:.1f}px ({connect_ratio:.1f}%)\n"
            f"    - Intra-group: {self.intra_group_connect_length:.1f}px\n"
            f"    - Inter-group: {self.inter_group_connect_length:.1f}px\n"
            f"  Start edge: {self.start_edge_id}"
        )


def compute_edge_centroids(edge_tasks: list) -> Dict[str, Tuple[float, float]]:
    """
    计算每条边的中心点

    Args:
        edge_tasks: 边任务列表

    Returns:
        Dict[str, Tuple[float, float]]: {edge_id: centroid}
    """
    centroids = {}

    for task in edge_tasks:
        polyline = np.array(task.polyline)
        centroid = tuple(np.mean(polyline, axis=0))
        centroids[task.edge_id] = centroid

    return centroids


def group_edges_spatially(edge_tasks: list, centroids: Dict[str, Tuple[float, float]],
                          eps: float = 150.0) -> Dict[str, EdgeGroup]:
    """
    基于空间位置对 edges 进行分组

    Args:
        edge_tasks: 边任务列表
        centroids: 边中心点映射
        eps: 聚类距离阈值（当前未使用，采用兜底方案）

    Returns:
        Dict[str, EdgeGroup]: {group_id: EdgeGroup}
    """
    print(f"[Edge Grouping] 基于空间位置分组 (eps={eps}px)...")
    print(f"  [DEBUG] edge_tasks 数量: {len(edge_tasks)}")
    print(f"  [DEBUG] centroids 数量: {len(centroids)}")

    # ============ 应急降级修复：跳过 DBSCAN，使用单任务分组兜底方案 ============
    print(f"  [DEBUG] 跳过 DBSCAN，使用单任务分组兜底方案")

    groups = {}
    for i, task in enumerate(edge_tasks):
        group_id = f"Group_{i}"
        groups[group_id] = EdgeGroup(
            group_id=group_id,
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

    print(f"  [DEBUG] 兜底分组完成，groups 数量: {len(groups)}")
    # ============================================================================

    # 原始 DBSCAN 逻辑（已注释，待后续修复）
    # # 检查 sklearn 依赖
    # if not _HAS_SKLEARN:
    #     raise ImportError(...)
    #
    # # 准备聚类数据
    # edge_ids = list(centroids.keys())
    # points = np.array([centroids[eid] for eid in edge_ids])
    #
    # # DBSCAN 聚类
    # clustering = DBSCAN(eps=eps, min_samples=1).fit(points)
    # labels = clustering.labels_
    #
    # # ... 后续分组逻辑 ...

    print(f"  [Edge Grouping] 完成: {len(groups)} 个分组")
    for gid, group in groups.items():
        print(f"    {gid}: {len(group.edge_ids)} 条边, "
              f"inspect_len={group.total_inspect_length:.1f}px")

    return groups


def order_groups_greedy(groups: Dict[str, EdgeGroup],
                        edge_task_map: dict) -> List[str]:
    """
    贪心确定 group 访问顺序

    策略：
    1. 从最大的 group 开始（inspect_length 最大）
    2. 每次选择距离当前 group 最近且未访问的 group

    Args:
        groups: 分组字典
        edge_task_map: 边任务映射

    Returns:
        List[str]: group 访问顺序
    """
    print("\n[Group Ordering] 确定 group 访问顺序...")

    unvisited = set(groups.keys())
    visit_order = []

    # 从最大的 group 开始
    start_group = max(groups.items(), key=lambda x: x[1].total_inspect_length)
    visit_order.append(start_group[0])
    unvisited.remove(start_group[0])

    current_group = start_group[1]

    while unvisited:
        # 找最近的未访问 group
        min_dist = float('inf')
        nearest_group = None

        for gid in unvisited:
            group = groups[gid]
            dist = np.linalg.norm(np.array(current_group.centroid) -
                                 np.array(group.centroid))
            if dist < min_dist:
                min_dist = dist
                nearest_group = gid

        if nearest_group:
            visit_order.append(nearest_group)
            unvisited.remove(nearest_group)
            current_group = groups[nearest_group]
        else:
            break

    print(f"  [Group Ordering] Group 访问顺序:")
    for i, gid in enumerate(visit_order):
        print(f"    {i+1}. {gid} ({len(groups[gid].edge_ids)} edges, "
              f"{groups[gid].total_inspect_length:.1f}px)")

    return visit_order


def plan_edges_within_group(topo_graph: TopoGraph,
                            edge_task_map: dict,
                            group_edge_ids: List[str],
                            adjacency: dict,
                            start_edge_id: str = None) -> List[MissionSegment]:
    """
    规划单个 group 内的 edges（复用现有逻辑）

    Args:
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        group_edge_ids: 组内边ID列表
        adjacency: 边邻接表
        start_edge_id: 起始边ID（可选）

    Returns:
        List[MissionSegment]: 组内 segments
    """
    segments = []
    unvisited = set(group_edge_ids)

    # 选择起始边
    if start_edge_id is None or start_edge_id not in unvisited:
        start_edge_id = list(unvisited)[0] if unvisited else None

    if start_edge_id is None:
        return segments

    # 选择起始边的方向
    start_edge = edge_task_map[start_edge_id]
    start_node = topo_graph.get_node(start_edge.u)
    end_node = topo_graph.get_node(start_edge.v)

    if start_node and start_node.deg == 1:
        start_direction = 'forward'
    elif end_node and end_node.deg == 1:
        start_direction = 'reverse'
    else:
        start_direction = 'forward'

    # 添加起始 inspect segment
    start_geo = get_edge_geometry_with_direction(start_edge, start_direction)
    segments.append(MissionSegment(
        type='inspect',
        edge_id=start_edge_id,
        geometry=start_geo,
        length=start_edge.len2d,
        direction=start_direction
    ))
    unvisited.remove(start_edge_id)

    current_edge_id = start_edge_id
    current_direction = start_direction

    # 贪心规划组内 edges
    while unvisited:
        # 只在组内选择
        unvisited_neighbors = [e for e in adjacency.get(current_edge_id, [])
                              if e in unvisited]

        candidates = unvisited_neighbors if unvisited_neighbors else list(unvisited)

        best_plan = None
        min_cost = float('inf')

        for candidate_id in candidates:
            plan = evaluate_transition_with_geometry(
                topo_graph, current_edge_id, current_direction,
                candidate_id, edge_task_map
            )

            if plan is None:
                continue

            # 优先组内邻居
            if candidate_id in unvisited_neighbors:
                adjusted_cost = plan['connect_length']
            else:
                adjusted_cost = plan['total_incremental_cost']

            if adjusted_cost < min_cost:
                min_cost = adjusted_cost
                best_plan = plan

        if best_plan is None:
            break

        next_edge_id = best_plan['next_edge_id']
        next_direction = best_plan['next_direction']

        # 添加 connect segment
        segments.append(MissionSegment(
            type='connect',
            from_edge_id=current_edge_id,
            to_edge_id=next_edge_id,
            geometry=best_plan['connect_geometry'],
            length=best_plan['connect_length']
        ))

        # 添加 inspect segment
        next_edge = edge_task_map[next_edge_id]
        next_geo = get_edge_geometry_with_direction(next_edge, next_direction)

        segments.append(MissionSegment(
            type='inspect',
            edge_id=next_edge_id,
            geometry=next_geo,
            length=next_edge.len2d,
            direction=next_direction
        ))

        unvisited.remove(next_edge_id)
        current_edge_id = next_edge_id
        current_direction = next_direction

    return segments


def build_grouped_continuous_mission(
    topo_graph: TopoGraph,
    edge_tasks: list,
    groups: Dict[str, EdgeGroup],
    group_visit_order: List[str],
    adjacency: dict
) -> GroupedContinuousMission:
    """
    构建分组感知的连续任务

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        groups: 分组字典
        group_visit_order: group 访问顺序
        adjacency: 边邻接表

    Returns:
        GroupedContinuousMission: 分组连续任务
    """
    print("\n" + "="*60)
    print("[Grouped Planner] 开始构建分组连续航迹...")
    print("="*60)

    mission = GroupedContinuousMission()
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    # 添加 groups
    for group in groups.values():
        mission.add_group(group)

    mission.group_visit_order = group_visit_order

    # 按 group 顺序规划
    current_end_point = None

    for i, group_id in enumerate(group_visit_order):
        group = groups[group_id]
        print(f"\n[Group {i+1}/{len(group_visit_order)}] {group_id}")
        print(f"  Edges: {group.edge_ids}")

        # 规划组内 edges
        if i == 0:
            # 第一个组：内部规划
            group_segments = plan_edges_within_group(
                topo_graph, edge_task_map, group.edge_ids, adjacency
            )
        else:
            # 后续组：从上一个组结束点连接
            # 获取上一个组的最后一条边
            last_segment = [s for s in mission.segments if s.type == 'inspect'][-1]
            last_edge_id = last_segment.edge_id

            # 找到本组中最接近上一个组结束点的边作为起始边
            best_start_edge = None
            min_dist = float('inf')

            for eid in group.edge_ids:
                edge = edge_task_map[eid]
                for direction in ['forward', 'reverse']:
                    geo = get_edge_geometry_with_direction(edge, direction)
                    start_point = geo[0]
                    dist = np.linalg.norm(np.array(current_end_point) -
                                         np.array(start_point))
                    if dist < min_dist:
                        min_dist = dist
                        best_start_edge = eid
                        best_start_dir = direction

            if best_start_edge:
                # 生成 inter-group connect segment（沿拓扑图）
                connect_geo, connect_len = generate_connection_segment_along_topo(
                    current_end_point,
                    get_edge_geometry_with_direction(
                        edge_task_map[best_start_edge], best_start_dir
                    )[0],
                    topo_graph,
                    edge_task_map
                )

                # 对 connect geometry 进行插值，生成密集路径点
                connect_geo_dense = interpolate_geometry(connect_geo, step_size=10.0, min_points=20)

                group_segments = []
                group_segments.append(MissionSegment(
                    type='connect',
                    from_edge_id=f"{group_visit_order[i-1]}_end",
                    to_edge_id=f"{group_id}_start",
                    geometry=connect_geo_dense,
                    length=connect_len
                ))

                # 标记为 inter-group connect
                group_segments[0].is_inter_group = True

                # 规划组内剩余 edges
                remaining_edges = [e for e in group.edge_ids if e != best_start_edge]
                if remaining_edges:
                    inner_segments = plan_edges_within_group(
                        topo_graph, edge_task_map,
                        [best_start_edge] + remaining_edges,
                        adjacency, best_start_edge
                    )
                    # 移除第一个 inspect（已作为起点）
                    group_segments.extend(inner_segments)
                else:
                    # 只有一条边
                    edge = edge_task_map[best_start_edge]
                    geo = get_edge_geometry_with_direction(edge, best_start_dir)
                    group_segments.append(MissionSegment(
                        type='inspect',
                        edge_id=best_start_edge,
                        geometry=geo,
                        length=edge.len2d,
                        direction=best_start_dir
                    ))
            else:
                # fallback
                group_segments = plan_edges_within_group(
                    topo_graph, edge_task_map, group.edge_ids, adjacency
                )

        # 添加 segments 到 mission
        for segment in group_segments:
            mission.segments.append(segment)
            mission.total_length += segment.length

            if segment.type == 'inspect':
                mission.inspect_length += segment.length
                if mission.start_edge_id is None:
                    mission.start_edge_id = segment.edge_id

                direction_mark = '+' if segment.direction == 'forward' else '-'
                mission.visit_order.append(f"{segment.edge_id}{direction_mark}")
            else:
                mission.connect_length += segment.length
                # 检查是否为 inter-group connect
                if hasattr(segment, 'is_inter_group') and segment.is_inter_group:
                    mission.inter_group_connect_length += segment.length
                else:
                    mission.intra_group_connect_length += segment.length

            mission.full_path.extend(segment.geometry)

            # 更新当前结束点
            if segment.geometry:
                current_end_point = segment.geometry[-1]

        print(f"  完成组内 {len([s for s in group_segments if s.type == 'inspect'])} 条边")

    print(f"\n[Grouped Planner] 航迹构建完成:")
    print(f"  - 总段数: {len(mission.segments)}")
    print(f"  - 访问边数: {len(mission.visit_order)}")
    print(f"  - 总长度: {mission.total_length:.1f}px")
    print(f"  - 巡检长度: {mission.inspect_length:.1f}px")
    print(f"  - 连接长度: {mission.connect_length:.1f}px")
    print(f"    - 组内连接: {mission.intra_group_connect_length:.1f}px")
    print(f"    - 组间连接: {mission.inter_group_connect_length:.1f}px")

    return mission


def find_nearest_group_from_point(
    start_xy: Tuple[float, float],
    groups: Dict[str, EdgeGroup],
    edge_tasks: list
) -> str:
    """
    从给定点找到最近的 group

    Args:
        start_xy: 起点坐标 (x, y)
        groups: 分组字典
        edge_tasks: 边任务列表

    Returns:
        str: 最近的 group_id
    """
    edge_task_map = {task.edge_id: task for task in edge_tasks}
    min_dist = float('inf')
    nearest_group = None

    for group_id, group in groups.items():
        # 计算 group 中心到起点的距离
        if group.centroid:
            dist = np.linalg.norm(np.array(start_xy) - np.array(group.centroid))
        else:
            # 如果没有 centroid，计算 group 中所有边的平均位置
            positions = []
            for edge_id in group.edge_ids:
                if edge_id in edge_task_map:
                    task = edge_task_map[edge_id]
                    if task.polyline:
                        mid_idx = len(task.polyline) // 2
                        positions.append(task.polyline[mid_idx])
            if positions:
                avg_pos = np.mean(positions, axis=0)
                dist = np.linalg.norm(np.array(start_xy) - avg_pos)
            else:
                continue

        if dist < min_dist:
            min_dist = dist
            nearest_group = group_id

    return nearest_group


def reorder_groups_from_start(
    start_group_id: str,
    groups: Dict[str, EdgeGroup],
    edge_tasks: list
) -> List[str]:
    """
    从指定 group 开始重新排序 groups

    策略：从 start_group 开始，使用贪心算法访问剩余 groups

    Args:
        start_group_id: 起始 group ID
        groups: 分组字典
        edge_tasks: 边任务列表

    Returns:
        List[str]: 新的 group 访问顺序
    """
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    # 计算所有 group 的 centroid
    group_centroids = {}
    for gid, group in groups.items():
        if group.centroid:
            group_centroids[gid] = group.centroid
        else:
            # 计算平均位置
            positions = []
            for edge_id in group.edge_ids:
                if edge_id in edge_task_map:
                    task = edge_task_map[edge_id]
                    if task.polyline:
                        mid_idx = len(task.polyline) // 2
                        positions.append(task.polyline[mid_idx])
            if positions:
                group_centroids[gid] = np.mean(positions, axis=0)

    # 新的访问顺序：从 start_group 开始
    new_order = [start_group_id]
    remaining = [gid for gid in groups.keys() if gid != start_group_id]
    current_gid = start_group_id

    # 贪心：每次访问最近的未访问 group
    while remaining:
        current_pos = group_centroids.get(current_gid)
        if current_pos is None:
            # fallback: 随机选择
            next_gid = remaining.pop(0)
        else:
            # 确保 current_pos 是 numpy array
            current_pos = np.array(current_pos)
            min_dist = float('inf')
            next_gid = None
            for gid in remaining:
                if gid in group_centroids:
                    gid_pos = np.array(group_centroids[gid])
                    dist = np.linalg.norm(current_pos - gid_pos)
                    if dist < min_dist:
                        min_dist = dist
                        next_gid = gid
            if next_gid is None:
                next_gid = remaining[0]
            remaining.remove(next_gid)

        new_order.append(next_gid)
        current_gid = next_gid

    return new_order


def build_grouped_continuous_mission_from_start(
    topo_graph: TopoGraph,
    edge_tasks: list,
    groups: Dict[str, EdgeGroup],
    start_xy: Tuple[float, float],
    adjacency: dict
) -> GroupedContinuousMission:
    """
    从给定起点构建分组连续任务（起点驱动重规划）

    核心逻辑：
    1. 找到最接近起点的 group
    2. 从该 group 开始重新排序所有 groups
    3. 构建新的 mission

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        groups: 分组字典
        start_xy: 用户指定的起点坐标 (x, y)
        adjacency: 边邻接表

    Returns:
        GroupedContinuousMission: 新的分组连续任务
    """
    print("\n" + "="*60)
    print("[Grouped Planner] 起点驱动的分组连续航迹规划...")
    print(f"[起点坐标] ({start_xy[0]:.1f}, {start_xy[1]:.1f})")
    print("="*60)

    # Step 1: 找到最近的 group
    start_group_id = find_nearest_group_from_point(start_xy, groups, edge_tasks)
    print(f"[最近分组] {start_group_id}")

    # Step 2: 从该 group 开始重新排序 groups
    new_group_visit_order = reorder_groups_from_start(start_group_id, groups, edge_tasks)
    print(f"[新访问顺序] {new_group_visit_order}")

    # Step 3: 使用新的 group 访问顺序构建 mission
    mission = build_grouped_continuous_mission(
        topo_graph=topo_graph,
        edge_tasks=edge_tasks,
        groups=groups,
        group_visit_order=new_group_visit_order,
        adjacency=adjacency
    )

    # 记录起点信息
    mission.start_xy = start_xy
    mission.start_group_id = start_group_id

    return mission


def visualize_edge_groups(
    topo_graph: TopoGraph,
    edge_tasks: list,
    groups: Dict[str, EdgeGroup],
    output_path: str
):
    """
    可视化边分组结果

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        groups: 分组字典
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(16, 16))

    # 为每个 group 分配颜色
    group_ids = list(groups.keys())
    colors = plt.cm.tab10(np.linspace(0, 1, len(group_ids)))
    group_colors = {gid: colors[i] for i, gid in enumerate(group_ids)}

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        ax.scatter(x, y, c='gray', marker='o', s=30, zorder=3,
                  edgecolors='black', linewidths=1)

    # 绘制每条边，用其 group 颜色
    for task in edge_tasks:
        # 找到该边所属的 group
        group_id = None
        for gid, group in groups.items():
            if task.edge_id in group.edge_ids:
                group_id = gid
                break

        if group_id is None:
            continue

        color = group_colors[group_id]
        polyline = np.array(task.polyline)
        ax.plot(polyline[:, 0], polyline[:, 1], color=color,
               linewidth=3, alpha=0.8, zorder=2)

        # 显示 group_id
        mid_idx = len(polyline) // 2
        mid_x, mid_y = polyline[mid_idx]
        ax.text(mid_x, mid_y, group_id.replace('Group_', 'G'),
               fontsize=8, ha='center', va='center',
               bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                        edgecolor=color, linewidth=2, alpha=0.8),
               zorder=4)

    # 标记 group centroids
    for group in groups.values():
        if group.centroid:
            ax.scatter(group.centroid[0], group.centroid[1],
                      c='red', marker='*', s=150, zorder=5,
                      edgecolors='black', linewidths=1)

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='*', color='w', markerfacecolor='red',
               markersize=12, label='Group Centroid'),
    ]

    for gid in group_ids:
        legend_elements.append(
            Line2D([0], [0], color=group_colors[gid], linewidth=3,
                   label=f'{gid} ({len(groups[gid].edge_ids)} edges)')
        )

    ax.legend(handles=legend_elements, loc='upper right')

    ax.set_title(f'Edge Groups ({len(groups)} groups, DBSCAN eps=150px)')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 分组图已保存: {output_path}")


def visualize_grouped_topo_plan(
    topo_graph: TopoGraph,
    edge_tasks: list,
    mission: GroupedContinuousMission,
    groups: Dict[str, EdgeGroup],
    output_path: str
):
    """
    可视化分组感知的拓扑规划结果

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        mission: 分组连续任务
        groups: 分组字典
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(16, 16))

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'purple'
            marker = 's'
            size = 50
        elif node.kind == 'split':
            color = 'darkorange'
            marker = '^'
            size = 60
        else:
            color = 'gray'
            marker = 'o'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=3,
                  edgecolors='black', linewidths=1)

    # 为每个 segment 分配颜色
    inspect_count = 0
    visit_order_map = {}
    edge_to_group = {}

    # 构建 edge -> group 映射
    for group_id, group in groups.items():
        for eid in group.edge_ids:
            edge_to_group[eid] = group_id

    for segment in mission.segments:
        if segment.type == 'inspect':
            inspect_count += 1
            visit_order_map[segment.edge_id] = inspect_count

    # 为每个 group 分配颜色
    group_ids = list(groups.keys())
    group_colors_map = {}
    for i, gid in enumerate(mission.group_visit_order):
        group_colors_map[gid] = plt.cm.tab10(i / len(mission.group_visit_order))

    for segment in mission.segments:
        if segment.type == 'inspect':
            # 巡检段：用 group 颜色
            visit_idx = visit_order_map.get(segment.edge_id, 0)
            group_id = edge_to_group.get(segment.edge_id, 'Unknown')
            color = group_colors_map.get(group_id, 'gray')

            geometry = np.array(segment.geometry)
            ax.plot(geometry[:, 0], geometry[:, 1], color=color,
                   linewidth=4, alpha=0.9, zorder=2)

            # 显示访问序号和方向
            mid_idx = len(geometry) // 2
            mid_x, mid_y = geometry[mid_idx]
            dir_mark = '→' if segment.direction == 'forward' else '←'
            ax.text(mid_x, mid_y, f"{visit_idx}{dir_mark}",
                   fontsize=9, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                            edgecolor=color, linewidth=2, alpha=0.9),
                   zorder=5)

        else:  # connect
            # 连接段
            geometry = np.array(segment.geometry)

            # 检查是否为 inter-group connect
            is_inter = hasattr(segment, 'is_inter_group') and segment.is_inter_group

            if is_inter:
                # 组间连接：粗红色虚线
                ax.plot(geometry[:, 0], geometry[:, 1], color='red',
                       linewidth=2.5, alpha=0.7, linestyle='--', zorder=1)
            else:
                # 组内连接：细灰色虚线
                ax.plot(geometry[:, 0], geometry[:, 1], color='gray',
                       linewidth=1.5, alpha=0.5, linestyle=':', zorder=1)

            # 显示连接长度
            if len(geometry) >= 2:
                mid_idx = len(geometry) // 2
                mid_x, mid_y = geometry[mid_idx]

                if is_inter:
                    label = f"INTER\n{segment.length:.0f}px"
                    bbox_color = 'red'
                else:
                    label = f"{segment.length:.0f}px"
                    bbox_color = 'lightgray'

                ax.text(mid_x, mid_y, label,
                       fontsize=6 if not is_inter else 7,
                       ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor=bbox_color,
                                alpha=0.7, edgecolor='black'),
                       zorder=4)

    # 标记 group 切换位置
    for i, group_id in enumerate(mission.group_visit_order):
        # 找到该 group 的第一个 inspect segment
        group_started = False
        for segment in mission.segments:
            if segment.type == 'inspect' and segment.edge_id in groups[group_id].edge_ids:
                if not group_started:
                    group_started = True
                    if segment.geometry:
                        start_point = segment.geometry[0]
                        ax.scatter(start_point[0], start_point[1],
                                  c='lime', marker='s', s=100, zorder=6,
                                  edgecolors='black', linewidths=1.5,
                                  label=f'Group {i+1} Start' if i == 0 else "")
                # 标记 group 结束位置
                if segment.edge_id == groups[group_id].edge_ids[-1] or \
                   (i == len(mission.group_visit_order) - 1 and
                    segment == [s for s in mission.segments if s.type == 'inspect'][-1]):
                    if segment.geometry:
                        end_point = segment.geometry[-1]
                        ax.scatter(end_point[0], end_point[1],
                                  c='orange', marker='^', s=100, zorder=6,
                                  edgecolors='black', linewidths=1.5)

    # 标记全局起点和终点
    if mission.segments:
        first_segment = mission.segments[0]
        if first_segment.geometry:
            start_point = first_segment.geometry[0]
            ax.scatter(start_point[0], start_point[1], c='green', marker='*', s=200,
                      zorder=7, edgecolors='black', linewidths=2,
                      label='Start')

        last_segment = mission.segments[-1]
        if last_segment.geometry:
            end_point = last_segment.geometry[-1]
            ax.scatter(end_point[0], end_point[1], c='red', marker='X', s=200,
                      zorder=7, edgecolors='black', linewidths=2,
                      label='End')

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', linewidth=2.5, linestyle='--', label='Inter-Group Connect'),
        Line2D([0], [0], color='gray', linewidth=1.5, linestyle=':', label='Intra-Group Connect'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
               markersize=15, label='Start'),
        Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
               markersize=10, label='End'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='lime',
               markersize=10, label='Group Start'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='orange',
               markersize=10, label='Group End'),
    ]

    # 添加 group 颜色图例
    for i, gid in enumerate(mission.group_visit_order[:5]):  # 最多显示5个
        color = group_colors_map.get(gid, 'gray')
        legend_elements.append(
            Line2D([0], [0], color=color, linewidth=4,
                   label=f'{gid}')
        )

    ax.legend(handles=legend_elements, loc='upper right', fontsize=8)

    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0

    ax.set_title(f'Grouped Continuous Mission ({len(mission.groups)} groups, '
                 f'{len(mission.visit_order)} edges, '
                 f'total={mission.total_length:.0f}px, '
                 f'inspect={mission.inspect_length:.0f}px ({100-connect_ratio:.1f}%), '
                 f'connect={mission.connect_length:.0f}px ({connect_ratio:.1f}%))')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 分组连续航迹图已保存: {output_path}")


def print_grouped_mission_statistics(mission: GroupedContinuousMission,
                                     old_connect_length: float = None):
    """打印分组任务统计信息"""
    print("\n[Grouped Mission Statistics]")
    print("-" * 70)

    n_inspect = sum(1 for s in mission.segments if s.type == 'inspect')
    n_connect = sum(1 for s in mission.segments if s.type == 'connect')
    n_inter = sum(1 for s in mission.segments if s.type == 'connect' and
                  hasattr(s, 'is_inter_group') and s.is_inter_group)
    n_intra = n_connect - n_inter

    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0
    inspect_ratio = 100 - connect_ratio

    print(f"  总段数: {len(mission.segments)}")
    print(f"    - Inspect segments: {n_inspect}")
    print(f"    - Connect segments: {n_connect} (intra={n_intra}, inter={n_inter})")
    print()
    print(f"  Groups: {len(mission.groups)}")
    print(f"  Group visit order: {' -> '.join(mission.group_visit_order)}")
    print()
    print(f"  总长度: {mission.total_length:.1f}px")
    print(f"    - Inspect: {mission.inspect_length:.1f}px ({inspect_ratio:.1f}%)")
    print(f"    - Connect: {mission.connect_length:.1f}px ({connect_ratio:.1f}%)")
    print(f"      - Intra-group: {mission.intra_group_connect_length:.1f}px")
    print(f"      - Inter-group: {mission.inter_group_connect_length:.1f}px")
    print()

    if old_connect_length is not None:
        reduction = old_connect_length - mission.connect_length
        reduction_pct = reduction / old_connect_length * 100 if old_connect_length > 0 else 0
        print(f"  [对比旧版本]")
        print(f"    旧版 connect length: {old_connect_length:.1f}px")
        print(f"    新版 connect length: {mission.connect_length:.1f}px")
        print(f"    减少: {reduction:.1f}px ({reduction_pct:.1f}%)")
        print()

    print(f"  访问顺序: {' -> '.join(mission.visit_order)}")
    print()


def print_group_details(groups: Dict[str, EdgeGroup]):
    """打印分组详情"""
    print("\n[Edge Groups Detail]")
    print("-" * 70)

    for group_id, group in groups.items():
        print(f"  {group_id}:")
        print(f"    - Edges: {', '.join(group.edge_ids)}")
        print(f"    - Centroid: ({group.centroid[0]:.1f}, {group.centroid[1]:.1f})")
        print(f"    - Inspect length: {group.total_inspect_length:.1f}px")
        if group.bbox:
            print(f"    - BBox: ({group.bbox[0]:.1f}, {group.bbox[1]:.1f}) -> "
                  f"({group.bbox[2]:.1f}, {group.bbox[3]:.1f})")
    print()


# =====================================================
# 优化的分组感知连续航迹规划器
# =====================================================
# 优化目标：
#   1. 优化 group visit order（基于候选方案评估）
#   2. 优化 group entry/exit strategy
# 策略：
#   - 为每个 group 生成多个候选 entry/exit 方案
#   - 基于当前 mission 末端选择最优 group + entry/exit 组合
# =====================================================

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Literal


@dataclass
class GroupPlanCandidate:
    """
    分组规划候选方案
    """
    group_id: str
    start_edge_id: str
    start_direction: str
    entry_point: Tuple[float, float]
    exit_point: Tuple[float, float]
    edge_visit_order: List[str] = field(default_factory=list)
    segments: List[MissionSegment] = field(default_factory=list)
    internal_cost: float = 0.0
    total_cost: float = 0.0  # internal + connect_from_current

    def summary(self) -> str:
        """摘要"""
        dir_mark = '+' if self.start_direction == 'forward' else '-'
        return (f"{self.group_id}[{self.start_edge_id}{dir_mark}], "
                f"entry={self.entry_point[:2]}, exit={self.exit_point[:2]}, "
                f"cost={self.total_cost:.1f}px")


def plan_group_from_start_edge(
    topo_graph: TopoGraph,
    edge_task_map: dict,
    group_edge_ids: List[str],
    start_edge_id: str,
    start_direction: str,
    adjacency: dict
) -> GroupPlanCandidate:
    """
    从指定起始边规划 group 内路径

    Args:
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        group_edge_ids: 组内边ID列表
        start_edge_id: 起始边ID
        start_direction: 起始方向
        adjacency: 边邻接表

    Returns:
        GroupPlanCandidate: 组内规划候选方案
    """
    segments = []
    unvisited = set(group_edge_ids)

    if start_edge_id not in unvisited:
        return None

    # 获取起始边的 geometry
    start_edge = edge_task_map[start_edge_id]
    start_geo = get_edge_geometry_with_direction(start_edge, start_direction)
    entry_point = start_geo[0]

    # 添加起始 inspect segment
    segments.append(MissionSegment(
        type='inspect',
        edge_id=start_edge_id,
        geometry=start_geo,
        length=start_edge.len2d,
        direction=start_direction
    ))

    unvisited.remove(start_edge_id)
    current_edge_id = start_edge_id
    current_direction = start_direction
    internal_cost = start_edge.len2d

    # 贪心规划剩余 edges
    while unvisited:
        unvisited_neighbors = [e for e in adjacency.get(current_edge_id, [])
                              if e in unvisited]
        candidates = unvisited_neighbors if unvisited_neighbors else list(unvisited)

        best_plan = None
        min_cost = float('inf')

        for candidate_id in candidates:
            plan = evaluate_transition_with_geometry(
                topo_graph, current_edge_id, current_direction,
                candidate_id, edge_task_map
            )

            if plan is None:
                continue

            if candidate_id in unvisited_neighbors:
                adjusted_cost = plan['connect_length']
            else:
                adjusted_cost = plan['total_incremental_cost']

            if adjusted_cost < min_cost:
                min_cost = adjusted_cost
                best_plan = plan

        if best_plan is None:
            break

        # 添加 connect segment
        segments.append(MissionSegment(
            type='connect',
            from_edge_id=current_edge_id,
            to_edge_id=best_plan['next_edge_id'],
            geometry=best_plan['connect_geometry'],
            length=best_plan['connect_length']
        ))
        internal_cost += best_plan['connect_length']

        # 添加 inspect segment
        next_edge = edge_task_map[best_plan['next_edge_id']]
        next_geo = get_edge_geometry_with_direction(next_edge, best_plan['next_direction'])

        segments.append(MissionSegment(
            type='inspect',
            edge_id=best_plan['next_edge_id'],
            geometry=next_geo,
            length=next_edge.len2d,
            direction=best_plan['next_direction']
        ))
        internal_cost += next_edge.len2d

        unvisited.remove(best_plan['next_edge_id'])
        current_edge_id = best_plan['next_edge_id']
        current_direction = best_plan['next_direction']

    # 获取 exit point（最后一个 segment 的终点）
    exit_point = segments[-1].geometry[-1] if segments else entry_point

    # 构建 visit order
    edge_visit_order = []
    for seg in segments:
        if seg.type == 'inspect':
            dir_mark = '+' if seg.direction == 'forward' else '-'
            edge_visit_order.append(f"{seg.edge_id}{dir_mark}")

    return GroupPlanCandidate(
        group_id=None,  # 由调用者设置
        start_edge_id=start_edge_id,
        start_direction=start_direction,
        entry_point=entry_point,
        exit_point=exit_point,
        edge_visit_order=edge_visit_order,
        segments=segments,
        internal_cost=internal_cost
    )


def generate_group_plan_candidates(
    topo_graph: TopoGraph,
    edge_task_map: dict,
    group: EdgeGroup,
    adjacency: dict
) -> List[GroupPlanCandidate]:
    """
    为一个 group 生成多个候选规划方案

    策略：
    - 对组内每条 edge 的两种方向作为起始，生成候选方案
    - 限制数量避免爆炸（最多 N 个候选）

    Args:
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        group: 边分组
        adjacency: 边邻接表

    Returns:
        List[GroupPlanCandidate]: 候选方案列表
    """
    candidates = []
    max_candidates = min(len(group.edge_ids) * 2, 10)  # 最多10个候选

    for edge_id in group.edge_ids:
        for direction in ['forward', 'reverse']:
            candidate = plan_group_from_start_edge(
                topo_graph, edge_task_map, group.edge_ids,
                edge_id, direction, adjacency
            )

            if candidate is not None:
                candidate.group_id = group.group_id
                candidates.append(candidate)

                if len(candidates) >= max_candidates:
                    break

        if len(candidates) >= max_candidates:
            break

    # 按 internal_cost 排序
    candidates.sort(key=lambda c: c.internal_cost)

    return candidates


def evaluate_group_candidate(
    candidate: GroupPlanCandidate,
    current_end_point: Tuple[float, float]
) -> float:
    """
    评估 group 候选方案（从当前末端到该方案的总代价）

    Args:
        candidate: 组内规划候选方案
        current_end_point: 当前 mission 末端点

    Returns:
        float: 总增量代价（连接 + 内部）
    """
    # 计算从当前末端到 entry 的连接代价
    connect_cost = np.linalg.norm(np.array(candidate.entry_point) -
                                  np.array(current_end_point))

    # 总代价 = 连接代价 + 内部代价
    total_cost = connect_cost + candidate.internal_cost

    candidate.total_cost = total_cost
    return total_cost


def choose_next_group_candidate(
    all_candidates: List[GroupPlanCandidate],
    current_end_point: Tuple[float, float],
    visited_groups: set
) -> Optional[GroupPlanCandidate]:
    """
    从所有未访问 group 的候选方案中选择最优的

    Args:
        all_candidates: 所有候选方案（{group_id: [candidates]}）
        current_end_point: 当前 mission 末端点
        visited_groups: 已访问的 group

    Returns:
        GroupPlanCandidate: 最优候选方案
    """
    best_candidate = None
    min_cost = float('inf')

    for candidate in all_candidates:
        if candidate.group_id in visited_groups:
            continue

        cost = evaluate_group_candidate(candidate, current_end_point)

        if cost < min_cost:
            min_cost = cost
            best_candidate = candidate

    return best_candidate


def build_grouped_continuous_mission_optimized(
    topo_graph: TopoGraph,
    edge_tasks: list,
    groups: Dict[str, EdgeGroup],
    adjacency: dict
) -> GroupedContinuousMission:
    """
    构建优化后的分组连续任务

    策略：
    - 为每个 group 生成候选方案
    - 每一步从当前末端选择最优 group + entry/exit 组合

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        groups: 分组字典
        adjacency: 边邻接表

    Returns:
        GroupedContinuousMission: 优化后的分组连续任务
    """
    print("\n" + "="*60)
    print("[Optimized Grouped Planner] 开始构建优化分组航迹...")
    print("="*60)

    mission = GroupedContinuousMission()
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    # 添加 groups
    for group in groups.values():
        mission.add_group(group)

    # 为每个 group 生成候选方案
    print("\n[Candidate Generation] 为每个 group 生成候选方案...")
    all_candidates = []  # List[GroupPlanCandidate]
    group_candidates = {}  # {group_id: [candidates]}

    total_candidates = 0
    for group_id, group in groups.items():
        candidates = generate_group_plan_candidates(
            topo_graph, edge_task_map, group, adjacency
        )
        group_candidates[group_id] = candidates
        all_candidates.extend(candidates)
        total_candidates += len(candidates)
        print(f"  {group_id}: {len(candidates)} 个候选")
        for i, cand in enumerate(candidates[:3]):  # 只显示前3个
            print(f"    - 候选 {i+1}: {cand.summary()}")
        if len(candidates) > 3:
            print(f"    ... (共 {len(candidates)} 个)")

    print(f"\n  总候选数: {total_candidates}")

    # 选择起始 group（选择 inspect 长度最大的 group 的最优候选）
    start_group_id = max(groups.items(),
                        key=lambda x: x[1].total_inspect_length)[0]
    start_candidates = group_candidates[start_group_id]
    start_candidate = start_candidates[0]  # 已按 internal_cost 排序

    print(f"\n[Start Group] {start_group_id}")
    print(f"  起始方案: {start_candidate.summary()}")

    # 添加起始 group 的 segments
    current_end_point = start_candidate.entry_point
    visited_groups = {start_group_id}

    for segment in start_candidate.segments:
        mission.segments.append(segment)
        mission.total_length += segment.length

        if segment.type == 'inspect':
            mission.inspect_length += segment.length
            if mission.start_edge_id is None:
                mission.start_edge_id = segment.edge_id

            direction_mark = '+' if segment.direction == 'forward' else '-'
            mission.visit_order.append(f"{segment.edge_id}{direction_mark}")
        else:
            mission.connect_length += segment.length
            mission.intra_group_connect_length += segment.length

        mission.full_path.extend(segment.geometry)

    current_end_point = start_candidate.exit_point
    mission.group_visit_order.append(start_group_id)

    print(f"  完成组内 {len([s for s in start_candidate.segments if s.type == 'inspect'])} 条边")
    print(f"  出口点: ({current_end_point[0]:.1f}, {current_end_point[1]:.1f})")

    # 逐步选择下一个最优 group
    iteration = 1
    while len(visited_groups) < len(groups):
        print(f"\n[Iteration {iteration}] 从当前出口选择下一个 group...")

        # 选择最优候选
        best_candidate = choose_next_group_candidate(
            all_candidates, current_end_point, visited_groups
        )

        if best_candidate is None:
            print("  [WARN] 无法找到下一个 group")
            break

        group_id = best_candidate.group_id
        print(f"  选择 {group_id}")
        print(f"  方案: {best_candidate.summary()}")

        # 生成从当前末端到新 group entry 的 connect segment（沿拓扑图）
        connect_geo, connect_len = generate_connection_segment_along_topo(
            current_end_point, best_candidate.entry_point,
            topo_graph, edge_task_map
        )

        # 对 connect geometry 进行插值，生成密集路径点
        connect_geo_dense = interpolate_geometry(connect_geo, step_size=10.0, min_points=20)
        print(f"  [插值] Connect: {len(connect_geo)}点 -> {len(connect_geo_dense)}点")

        # 添加 inter-group connect
        connect_segment = MissionSegment(
            type='connect',
            from_edge_id=f"prev_exit",
            to_edge_id=f"{group_id}_entry",
            geometry=connect_geo_dense,
            length=connect_len
        )
        connect_segment.is_inter_group = True

        mission.segments.append(connect_segment)
        mission.total_length += connect_len
        mission.connect_length += connect_len
        mission.inter_group_connect_length += connect_len
        mission.full_path.extend(connect_geo_dense)

        # 添加新 group 的 segments
        for segment in best_candidate.segments:
            mission.segments.append(segment)
            mission.total_length += segment.length

            if segment.type == 'inspect':
                mission.inspect_length += segment.length

                direction_mark = '+' if segment.direction == 'forward' else '-'
                mission.visit_order.append(f"{segment.edge_id}{direction_mark}")
            else:
                mission.connect_length += segment.length
                mission.intra_group_connect_length += segment.length

            mission.full_path.extend(segment.geometry)

        visited_groups.add(group_id)
        mission.group_visit_order.append(group_id)
        current_end_point = best_candidate.exit_point

        print(f"  完成组内 {len([s for s in best_candidate.segments if s.type == 'inspect'])} 条边")
        print(f"  出口点: ({current_end_point[0]:.1f}, {current_end_point[1]:.1f})")

        iteration += 1

    print(f"\n[Optimized Grouped Planner] 航迹构建完成:")
    print(f"  - 总段数: {len(mission.segments)}")
    print(f"  - 访问边数: {len(mission.visit_order)}")
    print(f"  - 总长度: {mission.total_length:.1f}px")
    print(f"  - 巡检长度: {mission.inspect_length:.1f}px")
    print(f"  - 连接长度: {mission.connect_length:.1f}px")
    print(f"    - 组内连接: {mission.intra_group_connect_length:.1f}px")
    print(f"    - 组间连接: {mission.inter_group_connect_length:.1f}px")

    return mission


def visualize_grouped_topo_plan_optimized(
    topo_graph: TopoGraph,
    edge_tasks: list,
    mission: GroupedContinuousMission,
    groups: Dict[str, EdgeGroup],
    output_path: str
):
    """
    可视化优化后的分组拓扑规划结果

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        mission: 分组连续任务
        groups: 分组字典
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(16, 16))

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'purple'
            marker = 's'
            size = 50
        elif node.kind == 'split':
            color = 'darkorange'
            marker = '^'
            size = 60
        else:
            color = 'gray'
            marker = 'o'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=3,
                  edgecolors='black', linewidths=1)

    # 构建映射
    inspect_count = 0
    visit_order_map = {}
    edge_to_group = {}

    for group_id, group in groups.items():
        for eid in group.edge_ids:
            edge_to_group[eid] = group_id

    for segment in mission.segments:
        if segment.type == 'inspect':
            inspect_count += 1
            visit_order_map[segment.edge_id] = inspect_count

    # 为每个 group 分配颜色
    group_colors_map = {}
    for i, gid in enumerate(mission.group_visit_order):
        group_colors_map[gid] = plt.cm.tab10(i / len(mission.group_visit_order))

    # 绘制 segments
    for segment in mission.segments:
        if segment.type == 'inspect':
            visit_idx = visit_order_map.get(segment.edge_id, 0)
            group_id = edge_to_group.get(segment.edge_id, 'Unknown')
            color = group_colors_map.get(group_id, 'gray')

            geometry = np.array(segment.geometry)
            ax.plot(geometry[:, 0], geometry[:, 1], color=color,
                   linewidth=4, alpha=0.9, zorder=2)

            # 显示访问序号和方向
            mid_idx = len(geometry) // 2
            mid_x, mid_y = geometry[mid_idx]
            dir_mark = '→' if segment.direction == 'forward' else '←'
            ax.text(mid_x, mid_y, f"{visit_idx}{dir_mark}",
                   fontsize=9, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                            edgecolor=color, linewidth=2, alpha=0.9),
                   zorder=5)

        else:  # connect
            geometry = np.array(segment.geometry)
            is_inter = hasattr(segment, 'is_inter_group') and segment.is_inter_group

            if is_inter:
                ax.plot(geometry[:, 0], geometry[:, 1], color='red',
                       linewidth=2.5, alpha=0.7, linestyle='--', zorder=1)
            else:
                ax.plot(geometry[:, 0], geometry[:, 1], color='gray',
                       linewidth=1.5, alpha=0.5, linestyle=':', zorder=1)

            if len(geometry) >= 2:
                mid_idx = len(geometry) // 2
                mid_x, mid_y = geometry[mid_idx]

                if is_inter:
                    label = f"INTER\n{segment.length:.0f}px"
                    bbox_color = 'red'
                else:
                    label = f"{segment.length:.0f}px"
                    bbox_color = 'lightgray'

                ax.text(mid_x, mid_y, label,
                       fontsize=6 if not is_inter else 7,
                       ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor=bbox_color,
                                alpha=0.7, edgecolor='black'),
                       zorder=4)

    # 标记 group entry/exit 点
    for i, group_id in enumerate(mission.group_visit_order):
        group = groups[group_id]

        # 找到该 group 的第一个和最后一个 inspect segment
        group_segments = [s for s in mission.segments if s.type == 'inspect'
                         and s.edge_id in group.edge_ids]

        if group_segments:
            # Entry point
            first_seg = group_segments[0]
            if first_seg.geometry:
                entry_point = first_seg.geometry[0]
                ax.scatter(entry_point[0], entry_point[1],
                          c='cyan', marker='o', s=80, zorder=6,
                          edgecolors='black', linewidths=1.5,
                          label='Group Entry' if i == 0 else "")

            # Exit point
            last_seg = group_segments[-1]
            if last_seg.geometry:
                exit_point = last_seg.geometry[-1]
                ax.scatter(exit_point[0], exit_point[1],
                          c='magenta', marker='D', s=80, zorder=6,
                          edgecolors='black', linewidths=1.5,
                          label='Group Exit' if i == 0 else "")

    # 标记全局起点和终点
    if mission.segments:
        first_segment = mission.segments[0]
        if first_segment.geometry:
            start_point = first_segment.geometry[0]
            ax.scatter(start_point[0], start_point[1], c='green', marker='*', s=200,
                      zorder=7, edgecolors='black', linewidths=2,
                      label='Start')

        last_segment = mission.segments[-1]
        if last_segment.geometry:
            end_point = last_segment.geometry[-1]
            ax.scatter(end_point[0], end_point[1], c='red', marker='X', s=200,
                      zorder=7, edgecolors='black', linewidths=2,
                      label='End')

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', linewidth=2.5, linestyle='--', label='Inter-Group Connect'),
        Line2D([0], [0], color='gray', linewidth=1.5, linestyle=':', label='Intra-Group Connect'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
               markersize=15, label='Start'),
        Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
               markersize=10, label='End'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan',
               markersize=8, label='Group Entry'),
        Line2D([0], [0], marker='D', color='w', markerfacecolor='magenta',
               markersize=8, label='Group Exit'),
    ]

    for i, gid in enumerate(mission.group_visit_order[:5]):
        color = group_colors_map.get(gid, 'gray')
        legend_elements.append(
            Line2D([0], [0], color=color, linewidth=4, label=f'{gid}')
        )

    ax.legend(handles=legend_elements, loc='upper right', fontsize=8)

    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0

    ax.set_title(f'Optimized Grouped Mission ({len(mission.groups)} groups, '
                 f'{len(mission.visit_order)} edges, '
                 f'total={mission.total_length:.0f}px, '
                 f'inspect={mission.inspect_length:.0f}px ({100-connect_ratio:.1f}%), '
                 f'connect={mission.connect_length:.0f}px ({connect_ratio:.1f}%))')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 优化分组航迹图已保存: {output_path}")


def visualize_group_entry_exit_debug(
    topo_graph: TopoGraph,
    edge_tasks: list,
    mission: GroupedContinuousMission,
    groups: Dict[str, EdgeGroup],
    output_path: str
):
    """
    可视化 group entry/exit 调试图

    Args:
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        mission: 分组连续任务
        groups: 分组字典
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(16, 16))

    # 绘制拓扑节点
    for node in topo_graph.nodes.values():
        x, y = node.pos2d
        ax.scatter(x, y, c='lightgray', marker='o', s=30, zorder=2,
                  edgecolors='black', linewidths=0.5)

    # 为每个 group 分配颜色
    group_colors_map = {}
    for i, gid in enumerate(mission.group_visit_order):
        group_colors_map[gid] = plt.cm.tab10(i / len(mission.group_visit_order))

    # 绘制每条边（用其 group 颜色）
    edge_to_group = {}
    for group_id, group in groups.items():
        for eid in group.edge_ids:
            edge_to_group[eid] = group_id

    for task in edge_tasks:
        group_id = edge_to_group.get(task.edge_id, 'Unknown')
        color = group_colors_map.get(group_id, 'gray')
        polyline = np.array(task.polyline)
        ax.plot(polyline[:, 0], polyline[:, 1], color=color,
               linewidth=2, alpha=0.6, zorder=1)

    # 绘制 group centroids
    for group in groups.values():
        if group.centroid:
            ax.scatter(group.centroid[0], group.centroid[1],
                      c='yellow', marker='*', s=150, zorder=4,
                      edgecolors='black', linewidths=1)

    # 绘制 entry/exit 点和流向
    for i, group_id in enumerate(mission.group_visit_order):
        group = groups[group_id]
        color = group_colors_map.get(group_id, 'gray')

        # 找到该 group 的 entry 和 exit
        group_segments = [s for s in mission.segments if s.type == 'inspect'
                         and s.edge_id in group.edge_ids]

        if group_segments:
            first_seg = group_segments[0]
            last_seg = group_segments[-1]

            if first_seg.geometry and last_seg.geometry:
                entry_point = first_seg.geometry[0]
                exit_point = last_seg.geometry[-1]

                # 绘制 entry 点
                ax.scatter(entry_point[0], entry_point[1],
                          c='cyan', marker='o', s=100, zorder=5,
                          edgecolors='black', linewidths=2)

                # 绘制 exit 点
                ax.scatter(exit_point[0], exit_point[1],
                          c='magenta', marker='D', s=100, zorder=5,
                          edgecolors='black', linewidths=2)

                # 绘制从 entry 到 exit 的虚线箭头（表示大致流向）
                ax.annotate('', xy=exit_point, xytext=entry_point,
                          arrowprops=dict(arrowstyle='->', color=color,
                                        lw=2, alpha=0.5, linestyle='--'),
                          zorder=3)

                # 标注 group 序号
                mid_x = (entry_point[0] + exit_point[0]) / 2
                mid_y = (entry_point[1] + exit_point[1]) / 2
                ax.text(mid_x, mid_y, f"G{i+1}",
                       fontsize=12, ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                                edgecolor=color, linewidth=2, alpha=0.9),
                       zorder=6)

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='*', color='w', markerfacecolor='yellow',
               markersize=12, label='Group Centroid'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan',
               markersize=8, label='Entry Point'),
        Line2D([0], [0], marker='D', color='w', markerfacecolor='magenta',
               markersize=8, label='Exit Point'),
    ]

    for i, gid in enumerate(mission.group_visit_order[:5]):
        color = group_colors_map.get(gid, 'gray')
        legend_elements.append(
            Line2D([0], [0], color=color, linewidth=2, label=f'G{i+1}: {gid}')
        )

    ax.legend(handles=legend_elements, loc='upper right', fontsize=8)

    ax.set_title(f'Group Entry/Exit Debug ({len(mission.groups)} groups)')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] Entry/Exit 调试图已保存: {output_path}")


def print_optimized_mission_statistics(mission: GroupedContinuousMission,
                                       old_mission: GroupedContinuousMission = None):
    """打印优化任务统计信息"""
    print("\n[Optimized Mission Statistics]")
    print("-" * 70)

    n_inspect = sum(1 for s in mission.segments if s.type == 'inspect')
    n_connect = sum(1 for s in mission.segments if s.type == 'connect')
    n_inter = sum(1 for s in mission.segments if s.type == 'connect' and
                  hasattr(s, 'is_inter_group') and s.is_inter_group)
    n_intra = n_connect - n_inter

    connect_ratio = mission.connect_length / mission.total_length * 100 if mission.total_length > 0 else 0
    inspect_ratio = 100 - connect_ratio

    print(f"  总段数: {len(mission.segments)}")
    print(f"    - Inspect: {n_inspect}")
    print(f"    - Connect: {n_connect} (intra={n_intra}, inter={n_inter})")
    print()
    print(f"  Groups: {len(mission.groups)}")
    print(f"  Group visit order: {' -> '.join(mission.group_visit_order)}")
    print()
    print(f"  总长度: {mission.total_length:.1f}px")
    print(f"    - Inspect: {mission.inspect_length:.1f}px ({inspect_ratio:.1f}%)")
    print(f"    - Connect: {mission.connect_length:.1f}px ({connect_ratio:.1f}%)")
    print(f"      - Intra-group: {mission.intra_group_connect_length:.1f}px")
    print(f"      - Inter-group: {mission.inter_group_connect_length:.1f}px")
    print()

    if old_mission is not None:
        print(f"  [对比 step9_2 版本]")
        print(f"    旧版总长度: {old_mission.total_length:.1f}px")
        print(f"    新版总长度: {mission.total_length:.1f}px")
        print(f"    变化: {mission.total_length - old_mission.total_length:+.1f}px "
              f"({100*(mission.total_length-old_mission.total_length)/old_mission.total_length:+.1f}%)")
        print()

        print(f"    旧版 connect: {old_mission.connect_length:.1f}px")
        print(f"    新版 connect: {mission.connect_length:.1f}px")
        print(f"    变化: {mission.connect_length - old_mission.connect_length:+.1f}px "
              f"({100*(mission.connect_length-old_mission.connect_length)/old_mission.connect_length:+.1f}%)")
        print()

        print(f"    旧版 inter-group: {old_mission.inter_group_connect_length:.1f}px")
        print(f"    新版 inter-group: {mission.inter_group_connect_length:.1f}px")
        print(f"    变化: {mission.inter_group_connect_length - old_mission.inter_group_connect_length:+.1f}px")
        print()

        print(f"    旧版 inspect ratio: {100*old_mission.inspect_length/old_mission.total_length:.1f}%")
        print(f"    新版 inspect ratio: {inspect_ratio:.1f}%")
        print(f"    提升: {inspect_ratio - 100*old_mission.inspect_length/old_mission.total_length:+.1f}%")
        print()

    print(f"  访问顺序: {' -> '.join(mission.visit_order)}")
    print()


def print_group_entry_exit_summary(mission: GroupedContinuousMission,
                                   groups: Dict[str, EdgeGroup]):
    """打印每个 group 的 entry/exit 摘要"""
    print("\n[Group Entry/Exit Summary]")
    print("-" * 70)

    for i, group_id in enumerate(mission.group_visit_order):
        group = groups[group_id]

        # 找到该 group 的 entry 和 exit
        group_segments = [s for s in mission.segments if s.type == 'inspect'
                         and s.edge_id in group.edge_ids]

        if group_segments:
            first_seg = group_segments[0]
            last_seg = group_segments[-1]

            entry_point = first_seg.geometry[0] if first_seg.geometry else None
            exit_point = last_seg.geometry[-1] if last_seg.geometry else None

            print(f"  {group_id} (第 {i+1} 个访问):")
            print(f"    - 边数: {len(group.edge_ids)}")
            print(f"    - Entry: {first_seg.edge_id} ({first_seg.direction}) "
                  f"@ ({entry_point[0]:.1f}, {entry_point[1]:.1f})")
            print(f"    - Exit: {last_seg.edge_id} ({last_seg.direction}) "
                  f"@ ({exit_point[0]:.1f}, {exit_point[1]:.1f})")
            print()


# =====================================================
# JSON 导出功能
# =====================================================

import json
import os
from datetime import datetime
from typing import Any


def export_grouped_mission_to_json(
    mission: GroupedContinuousMission,
    edge_tasks: List[EdgeTask],
    line_inspection_points_by_line: Dict[str, List],
    output_path: str = "result/latest/mission_output.json",
    terrain_3d: Optional[np.ndarray] = None
) -> str:
    """
    将分组连续任务导出为标准 JSON 格式

    Args:
        mission: 分组连续任务对象
        edge_tasks: 边任务列表
        line_inspection_points_by_line: {line_id: [巡检点列表]}
        output_path: 输出文件路径
        terrain_3d: 3D地形数组（可选，用于生成3D坐标）

    Returns:
        str: 输出文件的完整路径
    """
    print(f"[JSON Export] 导出任务到 JSON...")
    print(f"  输出路径: {output_path}")

    # 如果 mission 没有 edge_to_group 属性，重新构建
    if not hasattr(mission, 'edge_to_group') or not mission.edge_to_group:
        mission.edge_to_group = {}
        for group_id, group in mission.groups.items():
            for edge_id in group.edge_ids:
                mission.edge_to_group[edge_id] = group_id

    # 1. 元信息
    metadata = {
        "version": "1.0.0",
        "planner_name": "Step9_2_GroupedContinuousPlanner",
        "timestamp": datetime.now().isoformat(),
        "export_date": datetime.now().strftime("%Y-%m-%d"),
        "export_time": datetime.now().strftime("%H:%M:%S")
    }

    # 2. 统计信息
    inspect_ratio = (mission.inspect_length / mission.total_length * 100
                     if mission.total_length > 0 else 0)

    # 统计巡检点总数
    total_inspection_points = 0
    for task in edge_tasks:
        total_inspection_points += task.num_points

    stats = {
        "total_length": float(round(mission.total_length, 2)),
        "inspect_length": float(round(mission.inspect_length, 2)),
        "connect_length": float(round(mission.connect_length, 2)),
        "inspect_ratio": float(round(inspect_ratio, 2)),
        "num_groups": int(len(mission.groups)),
        "num_edges": int(len(mission.visit_order)),
        "num_segments": int(len(mission.segments)),
        "num_inspection_points": int(total_inspection_points),
        "intra_group_connect_length": float(round(mission.intra_group_connect_length, 2)),
        "inter_group_connect_length": float(round(mission.inter_group_connect_length, 2))
    }

    # 3. Groups 信息
    groups_data = []
    for group_id in mission.group_visit_order:
        group = mission.groups.get(group_id)
        if group:
            group_data = {
                "group_id": group.group_id,
                "edge_ids": list(group.edge_ids),
                "total_inspect_length": float(round(group.total_inspect_length, 2))
            }

            # 处理 centroid（可能是 tuple 或 numpy array）
            if group.centroid is not None and len(group.centroid) > 0:
                group_data["centroid"] = {
                    "x": float(round(group.centroid[0], 2)),
                    "y": float(round(group.centroid[1], 2))
                }
            else:
                group_data["centroid"] = None

            # 处理 bbox（可能是 tuple 或 numpy array）
            if group.bbox is not None and len(group.bbox) >= 4:
                group_data["bbox"] = {
                    "min_x": float(round(group.bbox[0], 2)),
                    "min_y": float(round(group.bbox[1], 2)),
                    "max_x": float(round(group.bbox[2], 2)),
                    "max_y": float(round(group.bbox[3], 2))
                }
            else:
                group_data["bbox"] = None

            groups_data.append(group_data)

    # 4. 访问顺序
    # 从 segments 构建 direction_map
    direction_map = {}
    for segment in mission.segments:
        if segment.type == 'inspect' and segment.edge_id:
            direction_map[segment.edge_id] = segment.direction

    visit_order = {
        "edge_visit_order": mission.visit_order,
        "edge_direction": direction_map,
        "group_visit_order": mission.group_visit_order
    }

    # 5. Mission Segments
    segments_data = []
    for i, segment in enumerate(mission.segments):
        # 确保 geometry 被正确处理
        geometry_2d = []
        if segment.geometry:
            for p in segment.geometry:
                geometry_2d.append([float(round(p[0], 2)), float(round(p[1], 2))])

        segment_data = {
            "segment_id": f"seg_{i:04d}",
            "type": segment.type,
            "edge_id": segment.edge_id,
            "from_edge_id": segment.from_edge_id,
            "to_edge_id": segment.to_edge_id,
            "length": float(round(segment.length, 2)),
            "direction": segment.direction,
            "geometry_2d": geometry_2d
        }

        # 添加 group_id
        if segment.edge_id and segment.edge_id in mission.edge_to_group:
            segment_data["group_id"] = mission.edge_to_group[segment.edge_id]
        else:
            segment_data["group_id"] = None

        # 添加 3D 几何（如果提供 terrain）
        if terrain_3d is not None and segment.geometry:
            geometry_3d = []
            for x, y in segment.geometry:
                h_idx, w_idx = int(y), int(x)
                if 0 <= h_idx < terrain_3d.shape[0] and 0 <= w_idx < terrain_3d.shape[1]:
                    z = terrain_3d[h_idx, w_idx, 2]
                    geometry_3d.append([float(round(x, 2)), float(round(y, 2)), float(round(z, 2))])
                else:
                    geometry_3d.append([float(round(x, 2)), float(round(y, 2)), 0.0])
            segment_data["geometry_3d"] = geometry_3d
        else:
            segment_data["geometry_3d"] = None

        segments_data.append(segment_data)

    # 6. Inspection Points - 需要从 edge_tasks 和 line_inspection_points 构建
    inspection_points_data = []
    point_id_counter = 0

    # 为每个 edge 获取其巡检点
    edge_to_inspection_points = {}  # {edge_id: [points]}

    # 首先从 edge_tasks 获取已有的 inspection_points
    for task in edge_tasks:
        if task.inspection_points:
            edge_to_inspection_points[task.edge_id] = task.inspection_points

    # 根据 visit_order 为每个点分配访问顺序
    visit_order_counter = 0

    for edge_id_with_dir in mission.visit_order:
        # 去除方向标记（+/-）
        if edge_id_with_dir.endswith('+'):
            edge_id = edge_id_with_dir[:-1]
            direction = 'forward'
        elif edge_id_with_dir.endswith('-'):
            edge_id = edge_id_with_dir[:-1]
            direction = 'reverse'
        else:
            edge_id = edge_id_with_dir
            direction = direction_map.get(edge_id_with_dir, 'forward')

        points = edge_to_inspection_points.get(edge_id, [])

        # 如果 direction_map 中有相反的方向，则反转
        if direction == 'reverse':
            points = list(reversed(points))

        for point in points:
            point_id_counter += 1
            visit_order_counter += 1

            # 处理 LineInspectionPoint 对象或字典
            if hasattr(point, 'pixel_position'):
                # LineInspectionPoint dataclass
                point_type = point.point_type
                pixel_pos = point.pixel_position
                pos_3d = point.position_3d
                line_id = point.line_id
                priority = point.priority
                image_ref = point.image_path
                detection_result = point.detection_result
                status = point.status
                source_reason = point.source_reason
            elif isinstance(point, dict):
                # 字典格式
                point_type = point.get('type', point.get('point_type', 'unknown'))
                pixel_pos = point.get('pixel_position', point.get('pos2d', (0, 0)))
                pos_3d = point.get('position_3d')
                line_id = point.get('line_id', '')
                priority = point.get('priority', 'normal')
                image_ref = point.get('image_path')
                detection_result = point.get('detection_result')
                status = point.get('status', 'uninspected')
                source_reason = point.get('source_reason', '')
            else:
                # 未知格式，跳过
                continue

            # 确保 pixel_pos 是元组
            if not isinstance(pixel_pos, (tuple, list)) or len(pixel_pos) < 2:
                continue

            # 获取 3D 位置
            if pos_3d is None and terrain_3d is not None:
                x, y = pixel_pos
                h_idx, w_idx = int(y), int(x)
                if 0 <= h_idx < terrain_3d.shape[0] and 0 <= w_idx < terrain_3d.shape[1]:
                    z = terrain_3d[h_idx, w_idx, 2]
                    pos_3d = [round(x, 2), round(y, 2), round(z, 2)]

            # 获取 group_id
            group_id = mission.edge_to_group.get(edge_id) if hasattr(mission, 'edge_to_group') else None

            point_data = {
                "point_id": f"IP_{point_id_counter:05d}",
                "edge_id": edge_id,
                "group_id": group_id,
                "line_id": line_id,
                "point_type": point_type,
                "pixel_position": [round(float(pixel_pos[0]), 2), round(float(pixel_pos[1]), 2)],
                "position_3d": pos_3d if pos_3d is None else [float(x) for x in pos_3d],
                "visit_order": int(visit_order_counter),
                "priority": priority,
                "image_ref": image_ref,
                "detection_result": detection_result,
                "status": status,
                "source_reason": source_reason
            }
            inspection_points_data.append(point_data)

    # 7. Full Path
    full_path_2d = [[float(round(p[0], 2)), float(round(p[1], 2))] for p in (mission.full_path or [])]

    full_path_3d = None
    if terrain_3d is not None:
        full_path_3d = []
        for x, y in (mission.full_path or []):
            h_idx, w_idx = int(y), int(x)
            if 0 <= h_idx < terrain_3d.shape[0] and 0 <= w_idx < terrain_3d.shape[1]:
                z = terrain_3d[h_idx, w_idx, 2]
                full_path_3d.append([float(round(x, 2)), float(round(y, 2)), float(round(z, 2))])
            else:
                full_path_3d.append([float(round(x, 2)), float(round(y, 2)), 0.0])

    full_path_data = {
        "full_path_2d": full_path_2d,
        "full_path_3d": full_path_3d,
        "num_waypoints": len(full_path_2d)
    }

    # 组装完整 JSON
    output_data = {
        "metadata": metadata,
        "statistics": stats,
        "groups": groups_data,
        "visit_order": visit_order,
        "segments": segments_data,
        "inspection_points": inspection_points_data,
        "full_path": full_path_data
    }

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    # 写入 JSON 文件
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"  [完成] JSON 已导出: {output_path}")
    print(f"  - Groups: {len(groups_data)}")
    print(f"  - Segments: {len(segments_data)}")
    print(f"  - Inspection Points: {len(inspection_points_data)}")
    print(f"  - Waypoints: {len(full_path_2d)}")

    return output_path


def load_mission_json_for_ui(json_path: str) -> Dict[str, Any]:
    """
    从 JSON 文件加载任务数据用于 UI 显示

    Args:
        json_path: JSON 文件路径

    Returns:
        Dict: 包含所有任务数据的字典
    """
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data



