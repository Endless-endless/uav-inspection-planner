"""
=====================================================
Weather-Aware 3D A* Path Planning Algorithm
=====================================================

This module implements a three-dimensional A* search algorithm for UAV path
planning with weather-aware cost modeling.

Algorithm Formulation:
----------------------
The cost function is defined as:

    F(n) = G(n) + w * H(n)

    G(n) = C_d + C_h + λ_t * C_t + λ_w * C_w

where:
    - F(n): Total estimated cost from start to goal via node n
    - G(n): Actual cost from start to node n
    - H(n): Heuristic estimate from node n to goal
    - w: Heuristic weight (w ≥ 1.0)
    - C_d: Distance cost (Euclidean step cost)
    - C_h: Height cost (terrain-based penalty)
    - C_t: Turn cost (penalty for direction changes)
    - C_w: Weather cost (wind-induced penalty)
    - λ_t: Turn weight parameter
    - λ_w: Wind weight parameter

References:
-----------
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for
  the heuristic determination of minimum cost paths.
"""

import math
import heapq
import numpy as np
import time
from .node import Node

# Weather module
from weather.wind_model import WindField, NonUniformWindField, compute_wind_cost_physics, compute_wind_cost_weighted
from config.settings import (
    WIND_VECTOR, WIND_COST_WEIGHT, USE_SQUARED_WIND_COST,
    USE_NON_UNIFORM_WIND, WIND_COST_METHOD, DRONE_AIRSPEED
)


class AStar3D:
    """
    Weather-Aware 3D A* Path Planner

    Implements the A* search algorithm for 3D environments with:
    - Euclidean distance heuristic
    - Turn penalty for path smoothness
    - Weather-aware cost modeling

    Cost Function:
        F(n) = G(n) + w * H(n)
        G(n) = C_d + C_h + λ_t * C_t + C_w

    Weather Cost Methods:
    - Physics-based: C_w = 1 / (v_drone + dot(W, d))  [无需调参]
    - Weighted: C_w = λ_w * ||W|| * (1 - cos(θ))   [需调参]
    """

    def __init__(self, grid_map, weight=1.0, turn_weight=0.3, max_turn_angle=None,
                 use_heapq=True, search_radius=None, dist_map=None):
        """
        Initialize the A* planner.

        Args:
            grid_map: 3D grid map for collision detection and neighbor generation
            weight: Heuristic weight w (default: 1.0)
                - w = 1.0: Standard A* (optimality guaranteed)
                - w > 1.0: Weighted A* (faster search, suboptimal)
            turn_weight: Turn weight λ_t (default: 0.3)
                - Higher values produce smoother paths
            max_turn_angle: Maximum allowable turn angle in radians (default: None)
                - Constrains UAV maneuverability
            use_heapq: Use heapq for priority queue (default: True, ~10x faster)
            search_radius: Limit search to specific radius around path
            dist_map: Distance map for constraining search near powerlines
        """
        self.grid_map = grid_map
        self.expanded_nodes = 0

        # Heuristic weight (w)
        self.weight = weight

        # Turn weight (λ_t)
        self.turn_weight = turn_weight

        # Maximum turn angle constraint
        self.max_turn_angle = max_turn_angle

        # Optimization settings
        self.use_heapq = use_heapq
        self.search_radius = search_radius
        self.dist_map = dist_map
        self.max_iterations = 500000
        self.early_exit_threshold = 5.0
        self.cost_pruning_ratio = 2.0

        # Weather model initialization (根据配置选择风场类型)
        if USE_NON_UNIFORM_WIND:
            # 非均匀风场：区域差异风场，强制路径绕行
            self.wind_field = NonUniformWindField(WIND_VECTOR, high_wind_multiplier=3.0)
            self.is_non_uniform = True
        else:
            # 均匀风场：全局统一风速
            self.wind_field = WindField(WIND_VECTOR)
            self.is_non_uniform = False

        # Cost statistics for analysis
        self.total_distance_cost = 0.0
        self.total_height_cost = 0.0
        self.total_turn_cost = 0.0
        self.total_weather_cost = 0.0

    def _grid_to_world(self, grid_pos):
        """
        将栅格坐标转换为世界坐标（米）

        Args:
            grid_pos: 栅格坐标 (x, y, z)

        Returns:
            tuple: 世界坐标 (wx, wy, wz) 单位：米
        """
        res = self.grid_map.resolution
        return (
            self.grid_map.b_min[0] + grid_pos[0] * res,
            self.grid_map.b_min[1] + grid_pos[1] * res,
            self.grid_map.b_min[2] + grid_pos[2] * res
        )

    def heuristic(self, a, b):
        """
        Compute Euclidean distance heuristic between two 3D points.

        Args:
            a: Source coordinates (x, y, z)
            b: Target coordinates (x, y, z)

        Returns:
            float: Euclidean distance H(n)
        """
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def compute_turn_penalty(self, parent, current, neighbor):
        """
        Compute turn penalty based on angle between consecutive motion vectors.

        The turn cost C_t is normalized to [0, 1] and squared to emphasize
        sharp turns.

        Args:
            parent: Previous node position
            current: Current node position
            neighbor: Next node position

        Returns:
            tuple: (penalty, angle_radians)
                - penalty: Normalized turn penalty in [0, 1]
                - angle_radians: Absolute turn angle
        """
        if parent is None:
            return 0, 0

        # Motion vectors
        v1 = np.array(current) - np.array(parent)
        v2 = np.array(neighbor) - np.array(current)

        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)

        if norm1 == 0 or norm2 == 0:
            return 0, 0

        # Cosine of angle between vectors
        cos_angle = np.dot(v1, v2) / (norm1 * norm2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        # Angle in radians
        angle = np.arccos(cos_angle)

        # Normalize to [0, 1] and square for penalty
        normalized = angle / math.pi
        penalty = normalized ** 2

        return penalty, angle

    def _should_prune(self, cost, best_cost):
        """
        早期剪枝：如果当前代价远超已知最优解，剪枝

        Args:
            cost: 当前节点代价
            best_cost: 已知最优代价

        Returns:
            bool: 是否应该剪枝
        """
        if best_cost > 0 and cost > best_cost * self.cost_pruning_ratio:
            return True
        return False

    def plan(self, start, goal):
        """
        Compute optimal path from start to goal using A* search.

        Args:
            start: Start coordinates (x, y, z)
            goal: Goal coordinates (x, y, z)

        Returns:
            list: Path as list of (x, y, z) tuples, or None if no path found
        """
        # Reset node counter and cost statistics
        self.expanded_nodes = 0
        self.total_distance_cost = 0.0
        self.total_height_cost = 0.0
        self.total_turn_cost = 0.0
        self.total_weather_cost = 0.0

        # Debug output
        print(f"[DEBUG] A* search started: start={start}, goal={goal}")
        wind_mode = "Non-Uniform" if self.is_non_uniform else "Uniform"

        if WIND_COST_METHOD == 'physics':
            print(f"[DEBUG] Wind: {wind_mode} field, {WIND_VECTOR}")
            print(f"[DEBUG] Cost function: Physics-based (v_drone={DRONE_AIRSPEED} m/s)")
            print(f"[DEBUG] Method: C_w = 1 / (v_drone + dot(W, d)) - No tuning needed!")
        else:
            cost_mode = "Squared" if USE_SQUARED_WIND_COST else "Linear"
            print(f"[DEBUG] Wind: {wind_mode} field, {WIND_VECTOR}, λ_w={WIND_COST_WEIGHT}")
            print(f"[DEBUG] Cost function: Weighted method ({cost_mode}) - Requires tuning")

        # 如果是非均匀风场，显示风场详情
        if self.is_non_uniform:
            print(f"[DEBUG] Non-uniform wind field zones:")
            print(f"  - x < 0:    Low wind zone (30% of base)")
            print(f"  - 0 ≤ x < 10: Medium wind zone (100% of base)")
            print(f"  - x ≥ 10:   High wind zone (300% of base)")

        start_time = time.time()
        last_report_time = start_time
        report_interval = 10000  # Report every 10000 expanded nodes

        # Initialize start node
        start_node = Node(start)
        start_node.heuristic_cost = self.heuristic(start, goal)
        start_node.total_cost = start_node.heuristic_cost

        # 根据优化设置选择数据结构
        if self.use_heapq:
            # 使用heapq优先队列（优化版本）
            open_heap = []
            # 格式: (f_cost, g_cost, position, node)
            heapq.heappush(open_heap, (start_node.total_cost, 0, start, start_node))
            open_dict = {start: start_node}  # 用于快速查找
            closed_set = set()
        else:
            # 原始版本：字典+min
            open_list = {start: start_node}
            closed_list = {}

        # 最优代价记录（用于剪枝）
        best_cost_to_goal = float('inf')

        # 计算直线距离用于提前终止判断
        straight_distance = self.heuristic(start, goal)

        while (open_heap if self.use_heapq else open_list):

            # Select node with minimum F(n) = G(n) + w * H(n)
            if self.use_heapq:
                # 从堆中取出最小F值节点
                while open_heap:
                    f_cost, g_cost, pos, current = heapq.heappop(open_heap)
                    if pos not in closed_set:
                        break
                else:
                    break  # 堆空了
            else:
                # 原始版本：O(n)查找
                current = min(open_list.values(), key=lambda n: n.total_cost)

            self.expanded_nodes += 1

            # Debug progress report
            if self.expanded_nodes % report_interval == 0:
                elapsed = time.time() - start_time
                mode = "Optimized" if self.use_heapq else "Standard"
                print(f"[DEBUG] {mode} A* progress: {self.expanded_nodes} nodes expanded in {elapsed:.2f}s")

            # 早期剪枝检查
            if self.use_heapq and self._should_prune(current.cost, best_cost_to_goal):
                continue

            # Check if goal reached
            current_pos = current.position if self.use_heapq else current.position

            if current_pos == goal:
                elapsed = time.time() - start_time
                elapsed = time.time() - start_time
                mode = "Optimized" if self.use_heapq else "Standard"
                print(f"[DEBUG] {mode} A* goal reached: {self.expanded_nodes} nodes in {elapsed:.3f}s")
                best_cost_to_goal = current.cost

                # 输出cost统计信息
                total_cost = self.total_distance_cost + self.total_height_cost + self.total_turn_cost + self.total_weather_cost
                if total_cost > 0:
                    distance_ratio = (self.total_distance_cost / total_cost) * 100
                    height_ratio = (self.total_height_cost / total_cost) * 100
                    turn_ratio = (self.total_turn_cost / total_cost) * 100
                    weather_ratio = (self.total_weather_cost / total_cost) * 100

                    print(f"[DEBUG] Cost Statistics:")
                    print(f"  - Distance Cost (C_d): {self.total_distance_cost:.2f} ({distance_ratio:.1f}%)")
                    print(f"  - Height Cost (C_h): {self.total_height_cost:.2f} ({height_ratio:.1f}%)")
                    print(f"  - Turn Cost (λ_t·C_t): {self.total_turn_cost:.2f} ({turn_ratio:.1f}%)")
                    print(f"  - Weather Cost (λ_w·C_w): {self.total_weather_cost:.2f} ({weather_ratio:.1f}%)")
                    print(f"  - Total Cost: {total_cost:.2f}")

                    # 计算纯weather cost占比（不含λ_w权重）
                    if WIND_COST_WEIGHT > 0 and self.total_weather_cost > 0:
                        base_weather_cost = self.total_weather_cost / WIND_COST_WEIGHT
                        weather_impact_ratio = (base_weather_cost / (self.total_distance_cost + base_weather_cost)) * 100
                        print(f"[DEBUG] Weather Impact Ratio (C_w / (C_d + C_w)): {weather_impact_ratio:.1f}%")

                # Reconstruct path by backtracking
                path = []
                while current:
                    path.append(current.position)
                    current = current.parent

                print(f"[DEBUG] Path found with {len(path)} waypoints")
                return path[::-1]

            # Move current node from open to closed list
            if self.use_heapq:
                closed_set.add(current_pos)
            else:
                del open_list[current.position]
                closed_list[current.position] = current

            # 检查最大迭代次数
            if self.expanded_nodes > self.max_iterations:
                print(f"[WARN] Maximum iterations ({self.max_iterations}) reached")
                break

            # 提前终止检查（如果很接近目标）
            if self.use_heapq:
                dist_to_goal_now = self.heuristic(current_pos, goal)
                if dist_to_goal_now <= 3.0 and self.grid_map.is_valid(goal):
                    elapsed = time.time() - start_time
                    print(f"[DEBUG] Early exit: very close to goal")

                    # 构建路径
                    path = []
                    temp = current
                    while temp:
                        path.append(temp.position)
                        temp = temp.parent
                    path.append(goal)

                    print(f"[DEBUG] Optimized A* goal reached: {self.expanded_nodes} nodes in {elapsed:.3f}s")
                    print(f"[DEBUG] Path found with {len(path)} waypoints")
                    return path[::-1]

            # Expand neighbors
            current_pos_for_neighbors = current_pos if self.use_heapq else current.position
            for neighbor in self.grid_map.get_neighbors(current_pos_for_neighbors):

                # 跳过已访问节点
                if self.use_heapq:
                    if neighbor in closed_set:
                        continue
                else:
                    if neighbor in closed_list:
                        continue

                # 搜索范围限制（电网附近）
                if self.use_heapq and self.search_radius is not None:
                    # 简单的范围检查：只在起点和终点附近搜索
                    dist_to_start = self.heuristic(neighbor, start)
                    dist_to_goal = self.heuristic(neighbor, goal)
                    if dist_to_start > self.search_radius and dist_to_goal > self.search_radius:
                        continue

                neighbor_node = Node(neighbor, current)

                # ==============================================
                # Distance Cost (C_d)
                # ==============================================
                # Euclidean distance between adjacent nodes

                C_d = self.heuristic(current.position, neighbor)

                # ==============================================
                # Height Cost (C_h)
                # ==============================================
                # Penalty based on terrain height (encourages low-altitude flight)

                if hasattr(self.grid_map, 'height_map'):
                    x, y, z = neighbor
                    h, w = self.grid_map.height_map.shape
                    # 边界检查
                    if 0 <= y < h and 0 <= x < w:
                        height = self.grid_map.height_map[y, x]
                        C_h = height * 0.3  # 高度代价系数
                    else:
                        C_h = 0
                else:
                    C_h = 0

                tentative_cost = current.cost + C_d + C_h

                # ==============================================
                # Turn Cost (C_t)
                # ==============================================
                # Penalty based on angle between motion vectors

                turn_penalty, angle = self.compute_turn_penalty(
                    current.parent.position if current.parent else None,
                    current.position,
                    neighbor
                )

                # Apply maximum turn angle constraint
                if self.max_turn_angle is not None:
                    if angle > self.max_turn_angle:
                        continue

                # Weighted turn cost: λ_t * C_t
                C_t = self.turn_weight * turn_penalty

                # ==============================================
                # Weather Cost (C_w)
                # ==============================================
                # Wind-induced cost based on motion-wind alignment

                move_vector = (
                    neighbor[0] - current.position[0],
                    neighbor[1] - current.position[1],
                    neighbor[2] - current.position[2]
                )

                # 获取当前位置的风向量（非均匀风场会根据位置返回不同的风）
                current_world_pos = self._grid_to_world(current.position)
                wind_vector = self.wind_field.get_wind_vector(current_world_pos)

                # 根据配置选择风场成本计算方法
                if WIND_COST_METHOD == 'physics':
                    # 物理驱动方法：无需调参
                    # C_w = 1 / (v_drone + dot(W, d))
                    C_w = compute_wind_cost_physics(
                        move_vector,
                        wind_vector,
                        drone_speed=DRONE_AIRSPEED
                    )
                else:  # 'weighted'
                    # 权重方法：需要调参（旧方法，保留兼容性）
                    C_w = compute_wind_cost_weighted(
                        move_vector,
                        wind_vector,
                        alpha=WIND_COST_WEIGHT,
                        use_squared=USE_SQUARED_WIND_COST
                    )

                # 累计统计信息
                self.total_distance_cost += C_d
                self.total_height_cost += C_h
                self.total_turn_cost += C_t
                self.total_weather_cost += C_w

                # ==============================================
                # Total G Cost: G(n) = C_d + C_h + λ_t * C_t + λ_w * C_w
                # ==============================================

                neighbor_node.cost = tentative_cost + C_t + C_w

                # ==============================================
                # Heuristic Cost H(n)
                # ==============================================

                neighbor_node.heuristic_cost = \
                    self.heuristic(neighbor, goal)

                # ==============================================
                # Total F Cost: F(n) = G(n) + w * H(n)
                # ==============================================

                neighbor_node.total_cost = (
                        neighbor_node.cost +
                        self.weight * neighbor_node.heuristic_cost
                )

                # Update open list if better path found
                if self.use_heapq:
                    existing_cost = open_dict.get(neighbor)
                    if existing_cost is None or neighbor_node.cost < existing_cost.cost:
                        heapq.heappush(open_heap, (neighbor_node.total_cost, neighbor_node.cost, neighbor, neighbor_node))
                        open_dict[neighbor] = neighbor_node
                        # 更新统计
                        if existing_cost is None:
                            self.total_distance_cost += C_d
                            self.total_height_cost += C_h
                            self.total_turn_cost += C_t
                            self.total_weather_cost += C_w
                else:
                    if neighbor not in open_list or neighbor_node.cost < open_list[neighbor].cost:
                        open_list[neighbor] = neighbor_node

        elapsed = time.time() - start_time
        print(f"[DEBUG] A* search failed: {self.expanded_nodes} nodes expanded in {elapsed:.3f}s")
        return None
