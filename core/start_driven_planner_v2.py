"""
起点驱动任务重规划核心模块（优化版 V2）

改进：
1. 智能起点候选选择（考虑拓扑端点、大group优先）
2. 路径生成后增加2-opt局部优化
3. 更强的代价函数（connect_length优先）
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import json
from pathlib import Path
import random


def analyze_edge_topology(baseline_data: Dict) -> Dict[str, Dict]:
    """
    分析边的拓扑特性

    Returns:
        edge_info: {
            edge_id: {
                'is_endpoint': bool,
                'group_id': str,
                'group_size': int,
                'group_length': float,
                'position': [x, y],
                'degree': int  # 连接其他边的数量
            }
        }
    """
    segments = baseline_data.get('segments', [])
    groups = baseline_data.get('groups', [])
    groups_map = {g['group_id']: g for g in groups}

    # 构建边连接图
    edge_connections = {}
    for seg in segments:
        if seg['type'] == 'inspect':
            edge_id = seg['edge_id']
            from_id = seg.get('from_edge_id')
            to_id = seg.get('to_edge_id')

            if edge_id not in edge_connections:
                edge_connections[edge_id] = set()

            if from_id and from_id != edge_id:
                edge_connections[edge_id].add(from_id)
            if to_id and to_id != edge_id:
                edge_connections[edge_id].add(to_id)

    # 构建边信息
    edge_info = {}
    for seg in segments:
        if seg['type'] == 'inspect':
            edge_id = seg['edge_id']
            geometry = seg['geometry_2d']
            position = np.array(geometry[0])

            # 找到所属group
            group_id = None
            group_size = 0
            group_length = 0
            for g in groups:
                if edge_id in g['edge_ids']:
                    group_id = g['group_id']
                    group_size = len(g['edge_ids'])
                    group_length = g['total_inspect_length']
                    break

            edge_info[edge_id] = {
                'is_endpoint': len(edge_connections.get(edge_id, set())) <= 1,
                'group_id': group_id,
                'group_size': group_size,
                'group_length': group_length,
                'position': position,
                'degree': len(edge_connections.get(edge_id, set()))
            }

    return edge_info


def generate_smart_start_candidates(
    start_x: float,
    start_y: float,
    edge_geometries: Dict[str, List[List[float]]],
    edge_info: Dict[str, Dict],
    max_candidates: int = 8
) -> List[Dict]:
    """
    智能生成候选起点（综合考虑拓扑、分组、距离）

    优先级策略：
    1. 拓扑端点边（degree<=1）
    2. 大group的边（group_size更大的优先）
    3. 距离较近的边

    Returns:
        候选列表，每个包含：
        - edge_id, direction, position, distance
        - priority_score: 优先级分数
        - type: 候选类型
    """
    start_point = np.array([start_x, start_y])
    candidates = []

    # 为每个边生成候选
    for edge_id, geometry in edge_geometries.items():
        info = edge_info.get(edge_id, {})
        group_size = info.get('group_size', 0)
        group_length = info.get('group_length', 0)
        is_endpoint = info.get('is_endpoint', True)  # 默认为True

        # 候选1：边的起点
        start_pos = np.array(geometry[0])
        dist = np.linalg.norm(start_pos - start_point)
        priority = compute_priority_score(dist, is_endpoint, group_size, group_length)
        candidates.append({
            'type': 'edge_start',
            'edge_id': edge_id,
            'direction': 'forward',
            'position': tuple(geometry[0]),
            'distance': dist,
            'priority_score': priority,
            'is_endpoint': is_endpoint,
            'group_size': group_size
        })

        # 候选2：边的终点
        end_pos = np.array(geometry[-1])
        dist = np.linalg.norm(end_pos - start_point)
        priority = compute_priority_score(dist, is_endpoint, group_size, group_length)
        candidates.append({
            'type': 'edge_end',
            'edge_id': edge_id,
            'direction': 'reverse',
            'position': tuple(geometry[-1]),
            'distance': dist,
            'priority_score': priority,
            'is_endpoint': is_endpoint,
            'group_size': group_size
        })

        # 候选3：边的中间点（仅对长边）
        if len(geometry) >= 10:  # 至少10个点才考虑中间
            for frac, label in [(0.25, 'mid_25'), (0.5, 'mid_50'), (0.75, 'mid_75')]:
                idx = int(len(geometry) * frac)
                pos = np.array(geometry[idx])
                dist = np.linalg.norm(pos - start_point)
                priority = compute_priority_score(dist, is_endpoint, group_size, group_length) * 0.9  # 中间点略低优先
                candidates.append({
                    'type': f'edge_{label}',
                    'edge_id': edge_id,
                    'direction': 'forward',
                    'position': tuple(geometry[idx]),
                    'distance': dist,
                    'priority_score': priority,
                    'is_endpoint': is_endpoint,
                    'group_size': group_size
                })

    # 按优先级排序，然后按距离
    candidates.sort(key=lambda c: (-c['priority_score'], c['distance']))

    # 取前N个，并确保多样性
    selected = []
    seen_edges = set()
    seen_groups = set()

    for cand in candidates:
        # 确保不重复选同一边的同方向
        key = (cand['edge_id'], cand['direction'])
        if key in seen_edges:
            continue

        # 确保从不同group选择
        group_id = edge_info.get(cand['edge_id'], {}).get('group_id')
        if group_id in seen_groups and len(seen_groups) < len(edge_info):
            # 如果还有未选择的group，跳过已选group的边
            continue

        selected.append(cand)
        seen_edges.add(key)
        if group_id:
            seen_groups.add(group_id)

        if len(selected) >= max_candidates:
            break

    return selected


def compute_priority_score(
    distance: float,
    is_endpoint: bool,
    group_size: int,
    group_length: float
) -> float:
    """
    计算优先级分数

    距离越近分数越高，端点边加分，大group加分
    """
    # 基础分数：距离的倒数（归一化到0-100）
    dist_score = 100000 / (distance + 100)

    # 端点边加分
    endpoint_bonus = 50 if is_endpoint else 0

    # Group大小加分
    size_bonus = group_size * 10

    # Group长度加分
    length_bonus = group_length / 10

    return dist_score + endpoint_bonus + size_bonus + length_bonus


def two_opt_local_search(
    visit_order: List[str],
    edge_geometries: Dict[str, List[List[float]]],
    max_iterations: int = 100
) -> Tuple[List[str], float]:
    """
    2-opt局部搜索优化路径

    尝试交换路径中的两条边来减少总连接长度
    """
    best_order = visit_order.copy()
    best_cost = compute_path_cost(best_order, edge_geometries)

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        # 尝试所有可能的2-opt交换
        for i in range(1, len(best_order) - 1):
            for j in range(i + 1, len(best_order)):
                # 交换i和j之间的边
                new_order = best_order[:i] + list(reversed(best_order[i:j+1])) + best_order[j+1:]
                new_cost = compute_path_cost(new_order, edge_geometries)

                if new_cost < best_cost:
                    best_order = new_order
                    best_cost = new_cost
                    improved = True

        # 尝试交换相邻边
        for i in range(1, len(best_order) - 1):
            new_order = best_order[:i] + [best_order[i+1], best_order[i]] + best_order[i+2:]
            new_cost = compute_path_cost(new_order, edge_geometries)

            if new_cost < best_cost:
                best_order = new_order
                best_cost = new_cost
                improved = True

    return best_order, best_cost


def compute_path_cost(
    visit_order: List[str],
    edge_geometries: Dict[str, List[List[float]]]
) -> float:
    """计算路径的总连接长度"""
    total_cost = 0.0
    for i in range(len(visit_order) - 1):
        geom1 = edge_geometries[visit_order[i]]
        geom2 = edge_geometries[visit_order[i + 1]]
        total_cost += np.linalg.norm(np.array(geom2[0]) - np.array(geom1[-1]))
    return total_cost


def generate_path_forward_greedy(
    start_edge_id: str,
    start_direction: str,
    start_position: Tuple[float, float],
    edge_geometries: Dict[str, List[List[float]]]
) -> Tuple[List[str], float]:
    """
    正向贪心：从起点向外扩展

    Returns:
        (edge_visit_order, total_cost)
    """
    visit_order = [start_edge_id]
    visited = {start_edge_id}
    current_end_pos = start_position
    total_cost = 0.0

    while len(visited) < len(edge_geometries):
        best_next = None
        best_dist = float('inf')

        for edge_id, geometry in edge_geometries.items():
            if edge_id in visited:
                continue

            start_pos = np.array(geometry[0])
            dist = np.linalg.norm(start_pos - np.array(current_end_pos))

            if dist < best_dist:
                best_dist = dist
                best_next = edge_id

        if best_next is None:
            break

        next_geometry = edge_geometries[best_next]
        connect_dist = np.linalg.norm(np.array(next_geometry[0]) - np.array(current_end_pos))
        total_cost += connect_dist

        visit_order.append(best_next)
        visited.add(best_next)
        current_end_pos = next_geometry[-1]

    return visit_order, total_cost


def generate_path_backward_greedy(
    start_edge_id: str,
    edge_geometries: Dict[str, List[List[float]]]
) -> Tuple[List[str], float]:
    """
    反向贪心：从终点向回规划

    Returns:
        (edge_visit_order, total_cost)
    """
    # 反向：从所有边中选择终点最远的作为起点
    visit_order = []
    visited = set()

    start_geometry = edge_geometries[start_edge_id]
    farthest_edge = None
    max_dist = 0

    for edge_id, geometry in edge_geometries.items():
        if edge_id == start_edge_id:
            continue
        dist = np.linalg.norm(np.array(geometry[-1]) - np.array(start_geometry[0]))
        if dist > max_dist:
            max_dist = dist
            farthest_edge = edge_id

    if farthest_edge is None:
        return [start_edge_id], 0.0

    visit_order = [farthest_edge]
    visited.add(farthest_edge)
    current_end = edge_geometries[farthest_edge][0]

    while len(visited) < len(edge_geometries):
        best_prev = None
        best_dist = float('inf')

        for edge_id, geometry in edge_geometries.items():
            if edge_id in visited:
                continue

            end_pos = np.array(geometry[-1])
            dist = np.linalg.norm(end_pos - np.array(current_end))

            if dist < best_dist:
                best_dist = dist
                best_prev = edge_id

        if best_prev is None:
            break

        visit_order.insert(0, best_prev)
        visited.add(best_prev)
        current_end = edge_geometries[best_prev][-1]

    total_cost = 0.0
    for i in range(len(visit_order) - 1):
        geom1 = edge_geometries[visit_order[i]]
        geom2 = edge_geometries[visit_order[i + 1]]
        total_cost += np.linalg.norm(np.array(geom2[0]) - np.array(geom1[-1]))

    return visit_order, total_cost


def generate_path_mst_based(
    start_edge_id: str,
    edge_geometries: Dict[str, List[List[float]]]
) -> Tuple[List[str], float]:
    """
    基于最小生成树（MST）的路径生成

    Returns:
        (edge_visit_order, total_cost)
    """
    edge_ids = list(edge_geometries.keys())

    # 计算所有边之间的距离
    dist_matrix = {}
    for i, e1 in enumerate(edge_ids):
        for j, e2 in enumerate(edge_ids):
            if i >= j:
                continue
            geom1 = edge_geometries[e1]
            geom2 = edge_geometries[e2]
            # 边e1终点到边e2起点的距离
            dist = np.linalg.norm(np.array(geom2[0]) - np.array(geom1[-1]))
            dist_matrix[(e1, e2)] = dist
            dist_matrix[(e2, e1)] = dist

    # Prim算法构建MST
    mst_edges = []
    in_mst = {start_edge_id}
    candidates = []

    # 初始化候选边
    for e in edge_ids:
        if e != start_edge_id:
            candidates.append((start_edge_id, e, dist_matrix[(start_edge_id, e)]))

    while len(in_mst) < len(edge_ids):
        # 选择最短的候选边
        candidates.sort(key=lambda x: x[2])
        selected = None
        for u, v, w in candidates:
            if v not in in_mst:
                selected = (u, v)
                break

        if selected is None:
            break

        u, v = selected
        mst_edges.append((u, v))
        in_mst.add(v)

        # 添加新的候选边
        for e in edge_ids:
            if e not in in_mst:
                candidates.append((v, e, dist_matrix[(v, e)]))

    # 从MST构建路径（从start_edge_id开始DFS遍历）
    adj = {e: [] for e in edge_ids}
    for u, v in mst_edges:
        adj[u].append(v)
        adj[v].append(u)

    visit_order = []
    visited = set()

    def dfs(edge_id):
        visit_order.append(edge_id)
        visited.add(edge_id)
        for neighbor in adj[edge_id]:
            if neighbor not in visited:
                dfs(neighbor)

    dfs(start_edge_id)

    total_cost = compute_path_cost(visit_order, edge_geometries)
    return visit_order, total_cost


def generate_multiple_paths_for_candidate(
    candidate: Dict,
    edge_geometries: Dict[str, List[List[float]]]
) -> List[Tuple[List[str], float, str]]:
    """
    为单个候选起点生成多条路径

    Returns:
        [(visit_order, cost, strategy_name), ...]
    """
    paths = []

    # 策略1：正向贪心
    try:
        order1, cost1 = generate_path_forward_greedy(
            candidate['edge_id'],
            candidate['direction'],
            candidate['position'],
            edge_geometries
        )
        paths.append((order1, cost1, 'forward_greedy'))
    except:
        pass

    # 策略2：反向贪心
    try:
        order2, cost2 = generate_path_backward_greedy(
            candidate['edge_id'],
            edge_geometries
        )
        paths.append((order2, cost2, 'backward_greedy'))
    except:
        pass

    # 策略3：MST-based
    try:
        order3, cost3 = generate_path_mst_based(
            candidate['edge_id'],
            edge_geometries
        )
        paths.append((order3, cost3, 'mst_based'))
    except:
        pass

    # 策略4：随机扰动（多次尝试，取最优）
    best_random_order = None
    best_random_cost = float('inf')

    for _ in range(5):
        try:
            edge_ids = list(edge_geometries.keys())
            if candidate['edge_id'] in edge_ids:
                edge_ids.remove(candidate['edge_id'])
            random.shuffle(edge_ids)
            order = [candidate['edge_id']] + edge_ids
            cost = compute_path_cost(order, edge_geometries)

            if cost < best_random_cost:
                best_random_cost = cost
                best_random_order = order
        except:
            pass

    if best_random_order is not None:
        paths.append((best_random_order, best_random_cost, 'random_perturbation'))

    return paths


def optimize_path_with_2opt(
    visit_order: List[str],
    edge_geometries: Dict[str, List[List[float]]]
) -> Tuple[List[str], float]:
    """
    对路径进行2-opt优化

    Returns:
        (optimized_order, new_cost)
    """
    return two_opt_local_search(visit_order, edge_geometries, max_iterations=50)


def plan_from_start_point_v2(
    start_x: float,
    start_y: float,
    baseline_data: Optional[Dict] = None
) -> Optional[Dict]:
    """
    从起点坐标规划任务（优化版 V2）

    Args:
        start_x: 起点X坐标
        start_y: 起点Y坐标
        baseline_data: 基线数据（用于比较）

    Returns:
        新任务数据（JSON格式）
    """
    print(f"\n{'='*60}")
    print(f"起点驱动任务重规划（优化版 V2）")
    print(f"{'='*60}")
    print(f"输入起点: ({start_x}, {start_y})")

    if baseline_data is None:
        print("错误：缺少基线数据")
        return None

    # 从基线中提取数据
    baseline_segments = baseline_data.get('segments', [])
    baseline_visit_order = baseline_data.get('visit_order', {}).get('edge_visit_order', [])

    # 提取edge几何信息
    edge_geometries = {}
    for seg in baseline_segments:
        if seg['type'] == 'inspect':
            edge_id = seg['edge_id']
            if edge_id and edge_id not in edge_geometries:
                edge_geometries[edge_id] = seg['geometry_2d']

    print(f"基线包含 {len(baseline_segments)} 个segments, {len(baseline_visit_order)} 条边")
    print(f"提取了 {len(edge_geometries)} 条边的几何信息")

    # 分析边拓扑特性
    edge_info = analyze_edge_topology(baseline_data)
    print(f"拓扑分析: {len(edge_info)} 条边")

    # =====================================================
    # Step 1: 智能生成候选起点
    # =====================================================
    print(f"\n[Step 1] 智能生成候选起点...")
    start_candidates = generate_smart_start_candidates(
        start_x, start_y, edge_geometries, edge_info,
        max_candidates=8
    )

    print(f"  生成了 {len(start_candidates)} 个候选起点:")
    for i, cand in enumerate(start_candidates):
        endpoint_mark = "[端点]" if cand.get('is_endpoint') else ""
        group_info = f"G{cand.get('group_size', 0)}" if cand.get('group_size') else "G0"
        print(f"    {i+1}. {cand['type']} | {cand['edge_id']} ({cand['direction']}) | {group_info} {endpoint_mark} | dist={cand['distance']:.1f}px | score={cand['priority_score']:.1f}")

    # =====================================================
    # Step 2: 为每个候选生成多条路径
    # =====================================================
    print(f"\n[Step 2] 为每个候选生成多条路径...")

    all_candidate_paths = []

    for i, candidate in enumerate(start_candidates):
        print(f"\n  候选 {i+1}: {candidate['type']} | {candidate['edge_id']}")

        # 生成多条路径
        paths = generate_multiple_paths_for_candidate(candidate, edge_geometries)

        print(f"    生成了 {len(paths)} 条路径:")
        for j, (order, cost, strategy) in enumerate(paths):
            connect_count = len(order) - 1
            print(f"      {j+1}. {strategy}: Connect={cost:.1f}px, 段数={connect_count}")

            all_candidate_paths.append({
                'candidate_index': i,
                'candidate_info': candidate,
                'visit_order': order,
                'connect_cost': cost,
                'strategy': strategy
            })

    if not all_candidate_paths:
        print("\n[失败] 无法生成任何路径")
        return None

    print(f"\n总共生成了 {len(all_candidate_paths)} 个候选方案")

    # =====================================================
    # Step 3: 2-opt局部优化
    # =====================================================
    print(f"\n[Step 3] 2-opt局部优化...")

    optimized_paths = []
    for p in all_candidate_paths:
        original_order = p['visit_order']
        original_cost = p['connect_cost']

        # 跳过已经很优的路径
        if len(original_order) <= 3:
            optimized_paths.append(p)
            continue

        # 执行2-opt优化
        optimized_order, optimized_cost = optimize_path_with_2opt(
            original_order, edge_geometries
        )

        # 如果有改进，更新
        if optimized_cost < original_cost * 0.99:  # 至少改进1%
            print(f"    {p['strategy']}: {original_cost:.1f}px -> {optimized_cost:.1f}px (改进{(1-optimized_cost/original_cost)*100:.1f}%)")
            optimized_paths.append({
                'candidate_index': p['candidate_index'],
                'candidate_info': p['candidate_info'],
                'visit_order': optimized_order,
                'connect_cost': optimized_cost,
                'strategy': f"{p['strategy']}_2opt"
            })
        else:
            optimized_paths.append(p)

    print(f"  优化完成: {len(optimized_paths)} 个方案")

    # =====================================================
    # Step 4: 评分并选择最优方案
    # =====================================================
    print(f"\n[Step 4] 评分并选择最优方案...")

    # 按connect_cost排序
    optimized_paths.sort(key=lambda p: p['connect_cost'])

    # 显示top 5
    print(f"  Top 5 方案:")
    for i, p in enumerate(optimized_paths[:5]):
        connect_count = len(p['visit_order']) - 1
        print(f"    {i+1}. {p['strategy']}: Connect={p['connect_cost']:.1f}px, 段数={connect_count}")

    # 选择最优
    best_path = optimized_paths[0]

    print(f"\n  最优方案: {best_path['strategy']}")
    print(f"    Connect: {best_path['connect_cost']:.1f}px")
    print(f"    访问顺序: {' -> '.join(best_path['visit_order'][:5])}...")

    # =====================================================
    # Step 5: 重建完整任务数据
    # =====================================================
    print(f"\n[Step 5] 重建任务数据...")

    new_visit_order = best_path['visit_order']

    # 重建segments
    new_segments = []
    total_connect = 0
    total_inspect = 0

    for i, edge_id in enumerate(new_visit_order):
        # 添加inspect segment
        geometry = edge_geometries[edge_id]
        inspect_len = sum(np.linalg.norm(np.array(geometry[j+1]) - np.array(geometry[j]))
                        for j in range(len(geometry)-1))

        total_inspect += inspect_len

        new_segments.append({
            "segment_id": f"seg_{i*2:04d}",
            "type": "inspect",
            "edge_id": edge_id,
            "from_edge_id": new_visit_order[i-1] if i > 0 else None,
            "to_edge_id": edge_id,
            "geometry_2d": geometry,
            "length": round(inspect_len, 2),
            "direction": "forward",
            "geometry_3d": None,
            "group_id": None
        })

        # 添加connect segment（如果不是第一条边）
        if i > 0:
            prev_geometry = edge_geometries[new_visit_order[i-1]]
            connect_geo = [prev_geometry[-1], geometry[0]]
            connect_len = np.linalg.norm(np.array(geometry[0]) - np.array(prev_geometry[-1]))

            total_connect += connect_len

            new_segments.append({
                "segment_id": f"seg_{i*2-1:04d}",
                "type": "connect",
                "edge_id": None,
                "from_edge_id": new_visit_order[i-1],
                "to_edge_id": edge_id,
                "geometry_2d": connect_geo,
                "length": round(connect_len, 2),
                "geometry_3d": None,
                "group_id": None
            })

    # 复制基线的其他信息
    new_mission_data = {
        "metadata": baseline_data.get("metadata", {}).copy(),
        "statistics": {},
        "groups": baseline_data.get("groups", []).copy(),
        "visit_order": {
            "edge_visit_order": new_visit_order,
            "edge_direction": {},
            "group_visit_order": baseline_data.get("visit_order", {}).get("group_visit_order", [])
        },
        "segments": new_segments,
        "inspection_points": baseline_data.get("inspection_points", []).copy(),
        "full_path_2d": [],
        "full_path_3d": []
    }

    # 更新统计信息
    new_mission_data["statistics"] = {
        "total_length": round(total_inspect + total_connect, 2),
        "inspect_length": round(total_inspect, 2),
        "connect_length": round(total_connect, 2),
        "inspect_ratio": round((total_inspect / (total_inspect + total_connect) * 100) if (total_inspect + total_connect) > 0 else 0, 2),
        "num_groups": baseline_data.get("statistics", {}).get("num_groups", 0),
        "num_edges": len(new_visit_order),
        "num_segments": len(new_segments),
        "num_inspection_points": baseline_data.get("statistics", {}).get("num_inspection_points", 0),
        "intra_group_connect_length": baseline_data.get("statistics", {}).get("intra_group_connect_length", 0),
        "inter_group_connect_length": round(total_connect, 2)
    }

    new_stats = new_mission_data["statistics"]
    print(f"\n新方案统计:")
    print(f"  Total: {new_stats['total_length']:.1f}px")
    print(f"  Inspect: {new_stats['inspect_length']:.1f}px")
    print(f"  Connect: {new_stats['connect_length']:.1f}px ({new_stats['connect_length'] / new_stats['total_length'] * 100:.1f}%)")

    return new_mission_data
