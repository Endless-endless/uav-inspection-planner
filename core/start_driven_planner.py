"""
起点驱动任务重规划核心模块

功能：
1. 将用户输入的起点映射到任务系统
2. 生成多个候选接入方案
3. 为每个方案重新生成任务顺序
4. 与基线比较，选择最优方案
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import json
from pathlib import Path
from scipy.ndimage import gaussian_filter

from core.topo import TopoGraph, TopoNode
from core.topo_task import EdgeTask
from core.topo_plan import (
    GroupedContinuousMission,
    EdgeGroup,
    build_edge_adjacency_simple,
    compute_edge_centroids,
    group_edges_spatially,
    build_grouped_continuous_mission,
    order_groups_greedy,
    export_grouped_mission_to_json
)


def get_node_xy(node) -> Tuple[float, float]:
    """
    兼容多种TopoNode坐标字段

    优先级：pos2d > pos > x/y > position > coord > coords > point > xy
    """
    # 优先：pos2d (当前TopoNode使用)
    if hasattr(node, "pos2d") and node.pos2d is not None:
        return float(node.pos2d[0]), float(node.pos2d[1])

    # 兼容：x/y 分开字段
    if hasattr(node, "x") and hasattr(node, "y"):
        return float(node.x), float(node.y)

    # 兼容：其他可能的字段名
    for attr in ["pos", "position", "coord", "coords", "point", "xy"]:
        if hasattr(node, attr):
            value = getattr(node, attr)
            if value is None:
                continue
            if isinstance(value, (list, tuple)) and len(value) >= 2:
                return float(value[0]), float(value[1])
            if hasattr(value, "__len__") and len(value) >= 2:
                return float(value[0]), float(value[1])
            if hasattr(value, "x") and hasattr(value, "y"):
                return float(value.x), float(value.y)

    # 兼容：dict类型
    if isinstance(node, dict):
        if "x" in node and "y" in node:
            return float(node["x"]), float(node["y"])
        for key in ["pos2d", "pos", "position", "coord", "coords", "point", "xy"]:
            if key in node:
                value = node[key]
                if isinstance(value, (list, tuple)) and len(value) >= 2:
                    return float(value[0]), float(value[1])

    raise AttributeError(f"Cannot extract x/y from node: {node}")


def find_nearest_access_points(
    start_x: float,
    start_y: float,
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    max_candidates: int = 5
) -> List[Dict]:
    """
    找到最近的几个接入点

    返回候选列表，每个候选包含：
    - type: 'node' | 'edge_endpoint'
    - node_id / edge_id
    - position: (x, y)
    - distance: 距离用户起点的距离
    """
    start_point = np.array([start_x, start_y])
    candidates = []

    # 候选1：最近端点节点（deg=1的节点）
    for node_id, node in topo_graph.nodes.items():
        if node.deg == 1:  # 端点
            nx, ny = get_node_xy(node)
            node_pos = np.array([nx, ny])
            dist = np.linalg.norm(node_pos - start_point)
            candidates.append({
                'type': 'endpoint_node',
                'node_id': node_id,
                'position': (nx, ny),
                'distance': dist,
                'priority': 1  # 高优先级
            })

    # 候选2：边的端点
    for edge in edge_tasks:
        # 起点
        start_pos = np.array(edge.polyline[0])
        dist_start = np.linalg.norm(start_pos - start_point)
        candidates.append({
            'type': 'edge_start',
            'edge_id': edge.edge_id,
            'position': tuple(edge.polyline[0]),
            'direction': 'forward',
            'distance': dist_start,
            'priority': 2
        })

        # 终点
        end_pos = np.array(edge.polyline[-1])
        dist_end = np.linalg.norm(end_pos - start_point)
        candidates.append({
            'type': 'edge_end',
            'edge_id': edge.edge_id,
            'position': tuple(edge.polyline[-1]),
            'direction': 'reverse',
            'distance': dist_end,
            'priority': 2
        })

    # 候选3：最近的拓扑节点（包括split节点）
    for node_id, node in topo_graph.nodes.items():
        nx, ny = get_node_xy(node)
        node_pos = np.array([nx, ny])
        dist = np.linalg.norm(node_pos - start_point)
        if dist < 200:  # 只考虑200px内的节点
            candidates.append({
                'type': 'topo_node',
                'node_id': node_id,
                'position': (nx, ny),
                'distance': dist,
                'priority': 3
            })

    # 按距离和优先级排序
    candidates.sort(key=lambda c: (c['priority'], c['distance']))

    return candidates[:max_candidates]


def plan_mission_from_access_point(
    start_x: float,
    start_y: float,
    access_point: Dict,
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask],
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: Dict
) -> Optional[GroupedContinuousMission]:
    """
    从指定接入点规划任务（真实起点驱动版本）

    Args:
        start_x: 用户输入的起点X坐标
        start_y: 用户输入的起点Y坐标
        access_point: 接入点信息
        topo_graph: 拓扑图
        edge_tasks: 边任务列表
        edge_task_map: 边任务映射
        groups: 分组信息
        adjacency: 邻接表

    Returns:
        规划的任务
    """
    # 确定起始边和方向
    start_edge_id = None
    start_direction = 'forward'

    if access_point['type'] in ['edge_start', 'edge_end']:
        # 直接从边开始
        start_edge_id = access_point['edge_id']
        start_direction = access_point['direction']

    elif access_point['type'] == 'endpoint_node':
        # 找到连接此节点的边
        node_id = access_point['node_id']
        for edge in edge_tasks:
            if edge.u == node_id or edge.v == node_id:
                start_edge_id = edge.edge_id
                # 确定方向：从节点出发
                if edge.u == node_id:
                    start_direction = 'forward'
                else:
                    start_direction = 'reverse'
                break

    elif access_point['type'] == 'topo_node':
        # 找到最近的边
        node_id = access_point['node_id']
        min_dist = float('inf')
        for edge in edge_tasks:
            # 检查边是否连接此节点
            if edge.u == node_id:
                dist = np.linalg.norm(np.array(edge.polyline[0]) -
                                     np.array([access_point['position'][0], access_point['position'][1]]))
                if dist < min_dist:
                    min_dist = dist
                    start_edge_id = edge.edge_id
                    start_direction = 'forward'
            elif edge.v == node_id:
                dist = np.linalg.norm(np.array(edge.polyline[-1]) -
                                     np.array([access_point['position'][0], access_point['position'][1]]))
                if dist < min_dist:
                    min_dist = dist
                    start_edge_id = edge.edge_id
                    start_direction = 'reverse'

    if start_edge_id is None:
        return None

    # 使用真实起点驱动的分组规划逻辑
    mission = _plan_greedy_from_start_edge(
        start_x, start_y,
        access_point,
        start_edge_id, start_direction,
        topo_graph, edge_task_map, groups, adjacency
    )

    return mission


def _plan_greedy_from_start_edge(
    start_x: float,
    start_y: float,
    access_point: Dict,
    start_edge_id: str,
    start_direction: str,
    topo_graph: TopoGraph,
    edge_task_map: Dict[str, EdgeTask],
    groups: Dict[str, EdgeGroup],
    adjacency: Dict
) -> GroupedContinuousMission:
    """
    从指定起始边开始贪心规划（真实起点驱动版本）

    特点：
    1. 第一段segment是从用户输入点到接入点的connect路径
    2. 然后从接入点进入第一条巡检边
    3. 后续使用最近邻策略规划剩余边

    Args:
        start_x: 用户输入的起点X坐标
        start_y: 用户输入的起点Y坐标
        access_point: 接入点信息
        start_edge_id: 起始边ID
        start_direction: 起始方向
        topo_graph: 拓扑图
        edge_task_map: 边任务映射
        groups: 分组信息
        adjacency: 邻接表

    Returns:
        规划的任务
    """
    mission = GroupedContinuousMission()

    # 添加所有groups
    for group in groups.values():
        mission.add_group(group)

    # 访问过的边
    visited = set()
    visit_order = []

    # ========== 第一步：添加从用户输入点到接入点的首段connect ==========
    access_pos = access_point['position']
    # 确保 access_pos 是 (float, float) 格式
    ax, ay = get_point_xy(access_pos)
    access_pos = (ax, ay)

    # 计算用户输入点到接入点的距离
    initial_connect_len = np.linalg.norm(np.array([ax, ay]) - np.array([start_x, start_y]))

    # 如果用户输入点和接入点不重合，添加首段connect
    if initial_connect_len > 1e-6:
        mission.segments.append({
            'type': 'connect',
            'from_edge_id': None,
            'to_edge_id': start_edge_id,
            'geometry': [(start_x, start_y), access_pos],
            'length': initial_connect_len
        })

    # 当前位置设为接入点
    current_end_pos = access_pos

    # ========== 第二步：添加第一条巡检边 ==========

    # 获取起始边的几何（按方向）
    start_edge = edge_task_map[start_edge_id]
    start_geo = _get_edge_geometry_with_direction(start_edge, start_direction)

    # 计算从接入点到起始边起点的connect（如果需要）
    # 注意：接入点可能在边的起点，也可能在终点或其他位置
    # 这里简化处理：直接从接入点连接到边的起点
    edge_start_pos = start_geo[0]
    edge_start_x, edge_start_y = get_point_xy(edge_start_pos)

    connect_to_edge_len = np.linalg.norm(np.array([edge_start_x, edge_start_y]) - np.array(current_end_pos))

    if connect_to_edge_len > 1e-6:
        mission.segments.append({
            'type': 'connect',
            'from_edge_id': None,
            'to_edge_id': start_edge_id,
            'geometry': [current_end_pos, (edge_start_x, edge_start_y)],
            'length': connect_to_edge_len
        })

    # 添加第一条inspect段
    mission.segments.append({
        'type': 'inspect',
        'edge_id': start_edge_id,
        'from_edge_id': None,
        'to_edge_id': start_edge_id,
        'geometry': start_geo,
        'length': start_edge.len2d,
        'direction': start_direction
    })
    visit_order.append(start_edge_id)
    visited.add(start_edge_id)

    current_end_pos = start_geo[-1]

    # ========== 第三步：贪心选择后续边 ==========
    while len(visited) < len(edge_task_map):
        # 找最近的未访问边
        best_next = None
        best_dist = float('inf')
        best_direction = 'forward'

        for edge_id, edge in edge_task_map.items():
            if edge_id in visited:
                continue

            # 尝试两个方向
            for direction in ['forward', 'reverse']:
                geo = _get_edge_geometry_with_direction(edge, direction)
                start_pos = geo[0]
                dist = np.linalg.norm(np.array(start_pos) - np.array(current_end_pos))

                if dist < best_dist:
                    best_dist = dist
                    best_next = edge_id
                    best_direction = direction

        if best_next is None:
            break

        # 生成connect段
        next_edge = edge_task_map[best_next]
        next_geo = _get_edge_geometry_with_direction(next_edge, best_direction)

        # 简化的直线connect
        connect_geo = [current_end_pos, next_geo[0]]
        connect_len = np.linalg.norm(np.array(next_geo[0]) - np.array(current_end_pos))

        mission.segments.append({
            'type': 'connect',
            'from_edge_id': start_edge_id,
            'to_edge_id': best_next,
            'geometry': connect_geo,
            'length': connect_len
        })

        # 添加inspect段
        mission.segments.append({
            'type': 'inspect',
            'edge_id': best_next,
            'from_edge_id': start_edge_id,
            'to_edge_id': best_next,
            'geometry': next_geo,
            'length': next_edge.len2d,
            'direction': best_direction
        })

        visit_order.append(best_next)
        visited.add(best_next)

        start_edge_id = best_next
        current_direction = best_direction
        current_end_pos = next_geo[-1]

    # ========== 第四步：统计 ==========
    inspect_len = sum(s['length'] for s in mission.segments if s['type'] == 'inspect')
    connect_len = sum(s['length'] for s in mission.segments if s['type'] == 'connect')

    mission.visit_order = visit_order
    mission.total_length = inspect_len + connect_len
    mission.inspect_length = inspect_len
    mission.connect_length = connect_len

    return mission


def _get_edge_geometry_with_direction(edge: EdgeTask, direction: str) -> List[Tuple[float, float]]:
    """获取指定方向的边几何"""
    if direction == 'forward':
        return edge.polyline.copy()
    else:
        return list(reversed(edge.polyline.copy()))


def plan_from_start_point(
    start_x: float,
    start_y: float,
    baseline_data: Optional[Dict] = None
) -> Optional[Dict]:
    """
    从起点坐标规划任务（全局重规划版本）

    【重要】这是真正的"全局起点驱动重规划"，不是"接入式重规划"

    区别：
    - 旧版本：找到最近接入点，从接入点进入已有路径框架
    - 新版本：以输入点为真正的全局起点，重新规划整个任务顺序

    特点：
    1. 每次输入新起点都会重新生成全局路径
    2. 第一条边、访问顺序、边方向都可能完全不同
    3. 不是只改头部接入，而是真正的全局重排序

    Args:
        start_x: 起点X坐标
        start_y: 起点Y坐标
        baseline_data: 基线数据（仅用于展示对比，不影响规划结果）

    Returns:
        新任务数据（JSON格式）
    """
    print(f"\n{'='*60}")
    print(f"全局起点驱动任务重规划（新版本）")
    print(f"{'='*60}")
    print(f"输入起点: ({start_x}, {start_y})")
    print(f"说明：这是真正的全局重规划，不是接入式修补")

    # 加载必要的模块
    from planner.powerline_planner_v3_final import PowerlinePlannerV3

    # 创建临时planner来获取数据
    planner = PowerlinePlannerV3('data/test.png')

    # 生成地形数据
    from scipy.ndimage import gaussian_filter
    np.random.seed(0)

    # 兼容多种planner属性
    img_obj = None
    for attr in ["img", "image", "raw_img", "original_image", "binary_img", "mask"]:
        if hasattr(planner, attr):
            img_obj = getattr(planner, attr)
            if img_obj is not None:
                break

    if img_obj is not None and hasattr(img_obj, "shape"):
        h, w = img_obj.shape[:2]
    else:
        h, w = 960, 916

    terrain_raw = np.random.rand(h, w)
    terrain_raw = gaussian_filter(terrain_raw, sigma=20)
    terrain_raw = (terrain_raw - terrain_raw.min()) / (terrain_raw.max() - terrain_raw.min()) * 60
    terrain_raw = terrain_raw.astype(np.float32)

    # 执行必要的步骤来获取拓扑图和边任务
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_continuous_path()
    planner.step4_extract_independent_lines()
    planner.step5_generate_line_inspection_points()
    planner.step6_map_line_points_to_3d(terrain_raw)
    planner.step7_5_build_topo()
    planner.step8_5_build_edge_tasks()

    topo_graph = planner.topo_graph
    edge_tasks = planner.edge_tasks
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    print(f"拓扑图: {len(topo_graph.nodes)} 个节点, {len(topo_graph.edges)} 条边")
    print(f"边任务: {len(edge_tasks)} 个")

    # 创建虚拟分组（保持兼容性）
    groups = {}
    try:
        centroids = compute_edge_centroids(edge_tasks)
    except Exception:
        centroids = {task.edge_id: (task.polyline[0][0], task.polyline[0][1])
                     for task in edge_tasks}

    for task in edge_tasks:
        groups[f"Group_{task.edge_id}"] = EdgeGroup(
            group_id=f"Group_{task.edge_id}",
            edge_ids=[task.edge_id],
            centroid=centroids.get(task.edge_id, (task.polyline[0][0], task.polyline[0][1])),
            bbox=(
                min(p[0] for p in task.polyline),
                min(p[1] for p in task.polyline),
                max(p[0] for p in task.polyline),
                max(p[1] for p in task.polyline)
            ),
            total_inspect_length=task.len2d
        )

    print(f"\n[全局重规划开始]")
    print(f"算法：最近邻贪心 + 双向方向选择")
    print(f"当前起点: ({start_x}, {start_y})")

    # ========== 核心调用：全局重规划 ==========
    # 这里调用的是新的全局重规划函数，而不是旧的接入式逻辑
    mission = plan_global_mission_from_start_point(
        start_x=start_x,
        start_y=start_y,
        topo_graph=topo_graph,
        edge_tasks=edge_tasks
    )

    if mission is None:
        print(f"\n[失败] 全局重规划失败")
        return None

    print(f"\n[全局重规划完成]")
    print(f"  总长度: {mission.total_length:.1f}px")
    print(f"  巡检长度: {mission.inspect_length:.1f}px")
    print(f"  连接长度: {mission.connect_length:.1f}px")
    print(f"  访问顺序: {' -> '.join(mission.visit_order[:8])}...")

    # 如果有基线数据，显示对比（但不影响采用）
    if baseline_data:
        baseline_stats = baseline_data.get('statistics', {})
        baseline_total = baseline_stats.get('total_length', 0)
        baseline_connect = baseline_stats.get('connect_length', 0)
        print(f"\n[基线对比]（仅供参考）")
        print(f"  基线总长度: {baseline_total:.1f}px")
        print(f"  新方案总长度: {mission.total_length:.1f}px")
        print(f"  差异: {mission.total_length - baseline_total:+.1f}px")

    # 转换为JSON格式
    mission_data = _convert_mission_to_json_format(
        mission, edge_tasks, groups, planner.line_inspection_points_by_line
    )

    return mission_data


def get_point_xy(point) -> Tuple[float, float]:
    """
    安全获取点的坐标（兼容多种数据类型）

    优先级：
    1. tuple/list: [x, y] 或 (x, y)
    2. dict: {"x":..., "y":...}, {"position":[x, y]}, {"pixel_position":[x, y]}
    3. 对象属性:
       - point.x / point.y
       - point.px / point.py
       - point.position -> [x, y]
       - point.pixel_position -> (x, y)    ← LineInspectionPoint
       - point.position_3d -> (x, y, z)    ← LineInspectionPoint

    Args:
        point: 点对象（多种类型）

    Returns:
        Tuple[float, float]: (x, y) 坐标
        如果无法解析，返回 (0.0, 0.0) 并打印警告
    """
    # ========== 1. tuple/list: [x, y] 或 (x, y) ==========
    if isinstance(point, (list, tuple)):
        if len(point) >= 2:
            try:
                return float(point[0]), float(point[1])
            except (TypeError, ValueError):
                pass

    # ========== 2. dict 类型 ==========
    if isinstance(point, dict):
        # 尝试直接 x, y
        if "x" in point and "y" in point:
            return float(point["x"]), float(point["y"])

        # 尝试 position 字段
        if "position" in point:
            pos = point["position"]
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                return float(pos[0]), float(pos[1])

        # 尝试 pixel_position 字段（LineInspectionPoint）
        if "pixel_position" in point:
            pos = point["pixel_position"]
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                return float(pos[0]), float(pos[1])

        # 尝试 position_3d 字段（LineInspectionPoint，取 x, y）
        if "position_3d" in point:
            pos = point["position_3d"]
            if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                return float(pos[0]), float(pos[1])

        # 尝试其他可能的字段名
        for key in ["pos", "coord", "coords", "xy", "pixel_pos"]:
            if key in point:
                val = point[key]
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    return float(val[0]), float(val[1])

    # ========== 3. 对象属性 ==========

    # 3.1 尝试 x, y
    if hasattr(point, "x") and hasattr(point, "y"):
        return float(point.x), float(point.y)

    # 3.2 尝试 px, py
    if hasattr(point, "px") and hasattr(point, "py"):
        return float(point.px), float(point.py)

    # 3.3 尝试 position 属性
    if hasattr(point, "position"):
        pos = point.position
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            return float(pos[0]), float(pos[1])
        if hasattr(pos, "x") and hasattr(pos, "y"):
            return float(pos.x), float(pos.y)

    # 3.4 尝试 pixel_position 属性（LineInspectionPoint）★
    if hasattr(point, "pixel_position"):
        pos = point.pixel_position
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            return float(pos[0]), float(pos[1])
        if hasattr(pos, "x") and hasattr(pos, "y"):
            return float(pos.x), float(pos.y)

    # 3.5 尝试 position_3d 属性（LineInspectionPoint，取 x, y）★
    if hasattr(point, "position_3d"):
        pos = point.position_3d
        if isinstance(pos, (list, tuple)) and len(pos) >= 3:
            return float(pos[0]), float(pos[1])
        if hasattr(pos, "x") and hasattr(pos, "y"):
            return float(pos.x), float(pos.y)

    # 3.6 尝试其他通用属性名
    for attr in ["pos", "coord", "coords", "xy", "point2d", "p2d", "pixel_pos"]:
        if hasattr(point, attr):
            val = getattr(point, attr)
            if isinstance(val, (list, tuple)) and len(val) >= 2:
                return float(val[0]), float(val[1])
            if hasattr(val, "x") and hasattr(val, "y"):
                return float(val.x), float(val.y)

    # ========== 所有尝试都失败，打印警告并返回默认值 ==========
    print(f"[WARN] 无法读取坐标 - 类型: {type(point)}, 内容: {point}")
    return (0.0, 0.0)



def _convert_mission_to_json_format(
    mission: GroupedContinuousMission,
    edge_tasks: List[EdgeTask],
    groups: Dict[str, EdgeGroup],
    line_inspection_points_by_line: Dict
) -> Dict:
    """将任务对象转换为JSON格式"""

    # 准备segments数据
    segments_data = []
    full_path_2d = []  # 收集完整的2D路径

    for i, seg in enumerate(mission.segments):
        seg_type = seg.get('type')
        geometry = seg.get('geometry', [])

        # 使用安全函数读取几何点坐标
        geometry_2d = []
        for p in geometry:
            px, py = get_point_xy(p)
            # get_point_xy 已经处理了错误情况，并返回 (0.0, 0.0)
            # 如果是 (0.0, 0.0) 可能是真正的原点，也可能是解析失败
            # 这里我们仍然添加它，除非有特殊需求过滤掉
            geometry_2d.append([px, py])
            full_path_2d.append([px, py])

        segment_data = {
            "segment_id": f"seg_{i:04d}",
            "type": seg_type,
            "edge_id": seg.get('edge_id'),
            "from_edge_id": seg.get('from_edge_id'),
            "to_edge_id": seg.get('to_edge_id'),
            "length": float(seg.get('length', 0)),
            "direction": seg.get('direction', 'forward'),
            "geometry_2d": geometry_2d,
            "geometry_3d": None,
            "group_id": None
        }

        segments_data.append(segment_data)

    # 统计信息
    inspect_len = sum(s['length'] for s in segments_data if s['type'] == 'inspect')
    connect_len = sum(s['length'] for s in segments_data if s['type'] == 'connect')
    total_len = inspect_len + connect_len

    # 访问顺序
    visit_order = mission.visit_order

    # Group顺序（简化）
    group_visit_order = list(groups.keys())

    # 统计巡检点
    total_inspection_points = sum(task.num_points for task in edge_tasks)

    statistics = {
        "total_length": round(total_len, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "inspect_ratio": round((inspect_len / total_len * 100) if total_len > 0 else 0, 2),
        "num_groups": len(groups),
        "num_edges": len(visit_order),
        "num_segments": len(segments_data),
        "num_inspection_points": total_inspection_points,
        "intra_group_connect_length": 0.0,
        "inter_group_connect_length": connect_len
    }

    # Inspection points（使用安全函数读取坐标）
    inspection_points_data = []
    point_id = 0
    for edge_id in visit_order:
        edge = next((e for e in edge_tasks if e.edge_id == edge_id), None)
        if edge and hasattr(edge, 'inspection_points') and edge.inspection_points:
            for point in edge.inspection_points:
                px, py = get_point_xy(point)
                # get_point_xy 已经处理了错误情况，并返回 (0.0, 0.0)
                # 只有在非零坐标时才添加（过滤掉解析失败的点）
                if px != 0.0 or py != 0.0:
                    point_id += 1
                    inspection_points_data.append({
                        "point_id": f"point_{point_id:04d}",
                        "position": [px, py],
                        "edge_id": edge_id,
                        "type": "sample"
                    })

    # Groups信息（使用安全函数读取centroid）
    groups_data = []
    for group_id, group in groups.items():
        centroid_dict = None
        if group.centroid:
            cx, cy = get_point_xy(group.centroid)
            # 只有在非零坐标时才设置 centroid（过滤掉解析失败的点）
            if cx != 0.0 or cy != 0.0:
                centroid_dict = {"x": round(cx, 2), "y": round(cy, 2)}

        group_data = {
            "group_id": group.group_id,
            "edge_ids": list(group.edge_ids),
            "total_inspect_length": round(group.total_inspect_length, 2),
            "centroid": centroid_dict
        }
        groups_data.append(group_data)

    return {
        "metadata": {
            "version": "1.0.0",
            "planner_name": "StartDrivenPlanner",
            "start_point_method": "user_input"
        },
        "statistics": statistics,
        "groups": groups_data,
        "visit_order": {
            "edge_visit_order": visit_order,
            "edge_direction": {},
            "group_visit_order": group_visit_order
        },
        "segments": segments_data,
        "inspection_points": inspection_points_data,
        "full_path_2d": full_path_2d,
        "full_path_3d": []
    }


# =====================================================
# 全局起点驱动重规划（新实现）
# =====================================================

def plan_global_mission_from_start_point(
    start_x: float,
    start_y: float,
    topo_graph: TopoGraph,
    edge_tasks: List[EdgeTask]
) -> GroupedContinuousMission:
    """
    真正的全局起点驱动重规划（彻底独立版本）

    【重要声明】
    此函数完全独立地生成路径，不依赖任何旧路径骨架：
    - 不使用 baseline 的 visit_order
    - 不使用 groups 的既有顺序
    - 不使用任何预定义的访问框架
    - 每一步都从所有未访问边中独立选择

    算法：增强的最近邻贪心 + 双向方向选择 + 详细决策日志
    - 起点 = 用户输入点
    - 每步评估所有未访问边 × 两个方向
    - 选择增量代价最小的候选
    - 更新位置，重复

    Args:
        start_x: 用户输入的起点X坐标
        start_y: 用户输入的起点Y坐标
        topo_graph: 拓扑图（仅用于获取结构，不影响顺序）
        edge_tasks: 待巡检的边任务列表

    Returns:
        GroupedContinuousMission: 全局重规划后的任务
    """
    mission = GroupedContinuousMission()

    # ========== 初始化状态（完全独立，无旧路径依赖）==========
    visited = set()  # 已访问的边ID
    visit_order = []  # 访问顺序
    edge_direction_map = {}  # 每条边选择的方向

    # 当前位置：从用户输入点开始
    current_pos = (float(start_x), float(start_y))

    # 边任务映射
    edge_task_map = {task.edge_id: task for task in edge_tasks}

    # 创建虚拟分组（每条边一个组，仅用于兼容性，不影响顺序）
    for task in edge_tasks:
        group = EdgeGroup(
            group_id=f"Group_{task.edge_id}",
            edge_ids=[task.edge_id],
            centroid=(task.polyline[0][0], task.polyline[0][1]),
            bbox=(
                min(p[0] for p in task.polyline),
                min(p[1] for p in task.polyline),
                max(p[0] for p in task.polyline),
                max(p[1] for p in task.polyline)
            ),
            total_inspect_length=task.len2d
        )
        mission.add_group(group)

    print(f"\n[全局重规划决策过程]")
    print(f"初始位置: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
    print(f"待巡检边数: {len(edge_tasks)}")

    # ========== 主循环：贪心选择下一条边 ==========
    step = 0
    while len(visited) < len(edge_tasks):
        step += 1
        print(f"\n--- 第 {step} 步决策 ---")
        print(f"当前位置: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
        print(f"已访问: {len(visited)}/{len(edge_tasks)} 条边")

        best_edge_id = None
        best_direction = None
        best_cost = float('inf')
        best_connect_len = float('inf')
        best_start_pos = None
        best_end_pos = None

        # 收集所有候选边及其代价
        candidates = []

        # 遍历所有未访问的边，评估两个方向
        for edge_id, edge in edge_task_map.items():
            if edge_id in visited:
                continue

            # ========== 评估 forward 方向 ==========
            fx, fy = get_point_xy(edge.polyline[0])
            forward_start_pos = (fx, fy)
            forward_connect = np.linalg.norm(np.array([fx, fy]) - np.array(current_pos))
            forward_total = forward_connect + edge.len2d

            candidates.append({
                'edge_id': edge_id,
                'direction': 'forward',
                'connect_len': forward_connect,
                'inspect_len': edge.len2d,
                'total_cost': forward_total,
                'start_pos': forward_start_pos,
                'end_pos': edge.polyline[-1]
            })

            # ========== 评估 reverse 方向 ==========
            rx, ry = get_point_xy(edge.polyline[-1])
            reverse_start_pos = (rx, ry)
            reverse_connect = np.linalg.norm(np.array([rx, ry]) - np.array(current_pos))
            reverse_total = reverse_connect + edge.len2d

            candidates.append({
                'edge_id': edge_id,
                'direction': 'reverse',
                'connect_len': reverse_connect,
                'inspect_len': edge.len2d,
                'total_cost': reverse_total,
                'start_pos': reverse_start_pos,
                'end_pos': edge.polyline[0]
            })

        # 按总代价排序，打印前5个候选
        candidates.sort(key=lambda x: x['total_cost'])
        print(f"候选边评估（按代价排序，Top 5）:")
        for i, c in enumerate(candidates[:5]):
            print(f"  {i+1}. {c['edge_id']} ({c['direction']}) - "
                  f"总代价={c['total_cost']:.1f}px "
                  f"(connect={c['connect_len']:.1f}, inspect={c['inspect_len']:.1f})")

        # 选择代价最小的候选
        best = candidates[0]
        best_edge_id = best['edge_id']
        best_direction = best['direction']
        best_connect_len = best['connect_len']
        best_cost = best['total_cost']
        best_start_pos = best['start_pos']
        best_end_pos = best['end_pos']

        print(f"选择: {best_edge_id} ({best_direction})")
        print(f"理由: 最小总代价 = {best_cost:.1f}px")

        # ========== 添加 segment ==========
        best_edge = edge_task_map[best_edge_id]

        # 1. 如果需要，添加 connect 段
        if best_connect_len > 1e-6:
            mission.segments.append({
                'type': 'connect',
                'from_edge_id': visit_order[-1] if visit_order else None,
                'to_edge_id': best_edge_id,
                'geometry': [current_pos, best_start_pos],
                'length': best_connect_len
            })
            print(f"添加 connect 段: 长度={best_connect_len:.1f}px")

        # 2. 添加 inspect 段
        if best_direction == 'forward':
            inspect_geo = best_edge.polyline.copy()
        else:
            inspect_geo = list(reversed(best_edge.polyline.copy()))

        mission.segments.append({
            'type': 'inspect',
            'edge_id': best_edge_id,
            'from_edge_id': visit_order[-1] if visit_order else None,
            'to_edge_id': best_edge_id,
            'geometry': inspect_geo,
            'length': best_edge.len2d,
            'direction': best_direction
        })
        print(f"添加 inspect 段: {best_edge_id}, 方向={best_direction}, 长度={best_edge.len2d:.1f}px")

        # 3. 更新状态
        visit_order.append(best_edge_id)
        edge_direction_map[best_edge_id] = best_direction
        visited.add(best_edge_id)

        # 4. 更新当前位置为这条边的终点
        ex, ey = get_point_xy(best_end_pos)
        current_pos = (ex, ey)

    # ========== 统计 ==========
    inspect_len = sum(s['length'] for s in mission.segments if s['type'] == 'inspect')
    connect_len = sum(s['length'] for s in mission.segments if s['type'] == 'connect')

    mission.visit_order = visit_order
    mission.total_length = inspect_len + connect_len
    mission.inspect_length = inspect_len
    mission.connect_length = connect_len

    print(f"\n[全局重规划完成]")
    print(f"最终访问顺序: {' -> '.join(visit_order)}")
    print(f"总长度: {mission.total_length:.1f}px (巡检={inspect_len:.1f}, 连接={connect_len:.1f})")

    return mission
