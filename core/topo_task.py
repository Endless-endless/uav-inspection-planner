"""
=====================================================
拓扑任务建模层
=====================================================

功能：
- 将巡检点映射到拓扑边
- 构建基于拓扑边的任务单元（EdgeTask）
- 为拓扑级路径规划奠定基础

EdgeTask：
- 基于 TopoEdge 的任务单元
- 包含该边的所有巡检点
- 可用于路径规划和任务调度

设计原则：
- 每条 EdgeTask 是一个独立的巡检任务
- 巡检点不遗漏、不重复
- 支持任务优先级和约束
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import numpy as np
from scipy.spatial.distance import euclidean


# =====================================================
# 数据结构
# =====================================================

@dataclass
class EdgeTask:
    """
    边任务：基于拓扑边的巡检任务单元

    属性：
        edge_id: 拓扑边ID
        u: 起始节点ID
        v: 结束节点ID
        line_id: 所属线路ID
        polyline: 2D几何（骨架段）
        len2d: 长度
        inspection_points: 该边的巡检点列表
        num_points: 巡检点数量
        is_straight: 是否近似直线
    """
    edge_id: str                          # 拓扑边ID
    u: str                                # 起始节点ID
    v: str                                # 结束节点ID
    line_id: str                          # 所属线路ID
    polyline: List[Tuple[float, float]]   # 2D几何
    len2d: float                          # 长度
    inspection_points: List[Dict] = field(default_factory=list)  # 巡检点列表
    num_points: int = 0                    # 巡检点数量
    is_straight: bool = True              # 是否近似直线
    split_reason: Optional[str] = None     # 切分原因

    # 任务属性（后续扩展）
    priority: int = 0                     # 优先级
    estimated_time: float = 0.0           # 预估执行时间
    meta: Dict = field(default_factory=dict)


# =====================================================
# 点到边的映射
# =====================================================

def map_points_to_edges(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> Dict[str, List[Dict]]:
    """
    将巡检点映射到对应的拓扑边

    原理：
    - 每个巡检点属于某条线路
    - 每条线路被切分为多条拓扑边
    - 根据点的位置，将其分配到对应的边上

    Args:
        topo_graph: 拓扑图
        line_inspection_points_by_line: {line_id: [巡检点列表]}

    Returns:
        Dict[str, List[Dict]]: {edge_id: [该边上的巡检点列表]}
    """
    print("[点边映射] 开始将巡检点映射到拓扑边...")

    edge_points = {}  # {edge_id: [points]}

    # 遍历所有拓扑边
    for edge_id, edge in topo_graph.edges.items():
        line_id = edge.line_id

        # 获取该线路的所有巡检点
        if line_id not in line_inspection_points_by_line:
            continue

        points = line_inspection_points_by_line[line_id]
        if not points:
            continue

        # 获取该边的polyline（骨架点）
        polyline = np.array(edge.polyline)

        # 为每条边分配点
        edge_points[edge_id] = []

        for point in points:
            # 获取点的2D位置 (LineInspectionPoint dataclass)
            if hasattr(point, 'pixel_position'):
                pt_pos = point.pixel_position
            elif isinstance(point, dict):
                pt_pos = point.get('pos2d', point.get('position', None))
            else:
                pt_pos = None

            if pt_pos is None:
                continue

            # 计算点到polyline的距离
            distances = []
            for i in range(len(polyline) - 1):
                # 计算点到线段的距离
                p1 = polyline[i]
                p2 = polyline[i + 1]
                dist = point_to_line_segment_distance(pt_pos, p1, p2)
                distances.append((dist, i))

            if distances:
                # 找最近的线段
                min_dist, _ = min(distances)

                # 如果距离合理（比如 < 10px），认为该点属于这条边
                if min_dist < 10.0:
                    edge_points[edge_id].append(point)

        # 如果该边没有点，检查端点
        if not edge_points[edge_id]:
            # 检查该边的端点是否有巡检点
            u_node = topo_graph.get_node(edge.u)
            v_node = topo_graph.get_node(edge.v)

            # 检查line_id对应的巡检点
            for point in points:
                if hasattr(point, 'pixel_position'):
                    pt_pos = point.pixel_position
                elif isinstance(point, dict):
                    pt_pos = point.get('pos2d', point.get('position', None))
                else:
                    continue

                if pt_pos is None:
                    continue

                # 检查是否是端点
                u_dist = euclidean(pt_pos, u_node.pos2d)
                v_dist = euclidean(pt_pos, v_node.pos2d)

                if u_dist < 10.0 or v_dist < 10.0:
                    edge_points[edge_id].append(point)
                    break

    # 统计
    total_mapped = sum(len(pts) for pts in edge_points.values())
    print(f"  [点边映射] 完成: {len(edge_points)} 条边, {total_mapped} 个巡检点")

    return edge_points


def point_to_line_segment_distance(point, line_start, line_end):
    """
    计算点到线段的垂直距离

    Args:
        point: 点 (x, y)
        line_start: 线段起点 (x, y)
        line_end: 线段终点 (x, y)

    Returns:
        float: 距离
    """
    point = np.array(point)
    start = np.array(line_start)
    end = np.array(line_end)

    line_vec = end - start
    point_vec = point - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-6:
        return np.linalg.norm(point_vec)

    # 投影
    t = np.dot(point_vec, line_vec) / (line_len ** 2)
    t = np.clip(t, 0, 1)

    # 最近点
    closest = start + t * line_vec

    return np.linalg.norm(point - closest)


# =====================================================
# 边任务构建
# =====================================================

def build_edge_tasks(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> List[EdgeTask]:
    """
    为每条拓扑边构建 EdgeTask

    Args:
        topo_graph: 拓扑图
        line_inspection_points_by_line: {line_id: [巡检点列表]}

    Returns:
        List[EdgeTask]: 边任务列表
    """
    print("[边任务构建] 开始构建边任务...")

    # 先映射点到边
    edge_points = map_points_to_edges(topo_graph, line_inspection_points_by_line)

    # 构建EdgeTask列表
    edge_tasks = []

    for edge_id, edge in topo_graph.edges.items():
        points = edge_points.get(edge_id, [])

        task = EdgeTask(
            edge_id=edge_id,
            u=edge.u,
            v=edge.v,
            line_id=edge.line_id,
            polyline=edge.polyline,
            len2d=edge.len2d,
            inspection_points=points,
            num_points=len(points),
            is_straight=edge.is_straight,
            split_reason=edge.split_reason
        )

        edge_tasks.append(task)

    print(f"  [边任务构建] 完成: {len(edge_tasks)} 个任务")

    return edge_tasks


def summarize_edge_tasks(edge_tasks: List[EdgeTask]) -> Dict:
    """
    统计边任务信息

    Args:
        edge_tasks: 边任务列表

    Returns:
        Dict: 统计信息
    """
    total_edges = len(edge_tasks)
    total_points = sum(task.num_points for task in edge_tasks)
    total_length = sum(task.len2d for task in edge_tasks)

    # 按线路统计
    line_split_info = {}
    for task in edge_tasks:
        line_id = task.line_id
        line_split_info[line_id] = line_split_info.get(line_id, 0) + 1

    # 点数分布
    points_distribution = {}
    for task in edge_tasks:
        points_distribution[task.edge_id] = task.num_points

    # 直线/曲线分布
    straight_edges = sum(1 for task in edge_tasks if task.is_straight)
    curved_edges = total_edges - straight_edges

    stats = {
        'total_edges': total_edges,
        'total_points': total_points,
        'total_length': total_length,
        'avg_points_per_edge': total_points / total_edges if total_edges > 0 else 0,
        'avg_length': total_length / total_edges if total_edges > 0 else 0,
        'line_split_info': line_split_info,
        'points_distribution': points_distribution,
        'straight_edges': straight_edges,
        'curved_edges': curved_edges
    }

    return stats


# =====================================================
# 可视化
# =====================================================

def visualize_edge_numbers(topo_graph, line_inspection_points_by_line: Dict[str, List],
                            output_path: str):
    """
    可视化拓扑边及其巡检点

    Args:
        topo_graph: 拓扑图
        line_inspection_points_by_line: {line_id: [巡检点列表]}
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(14, 14))

    # 绘制拓扑边
    for edge in topo_graph.edges.values():
        polyline = np.array(edge.polyline)
        color = 'blue' if edge.is_straight else 'green'
        ax.plot(polyline[:, 0], polyline[:, 1], color=color, linewidth=2, alpha=0.7)

        # 显示边编号
        mid_idx = len(polyline) // 2
        mid_x, mid_y = polyline[mid_idx]
        edge_num = edge.id.split('_')[-1]
        ax.text(mid_x, mid_y, edge_num, fontsize=8, ha='center', va='center',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # 绘制巡检点
    for line_id, points in line_inspection_points_by_line.items():
        for point in points:
            # 获取位置 (LineInspectionPoint dataclass)
            if hasattr(point, 'pixel_position'):
                pos = point.pixel_position
                kind = point.point_type if hasattr(point, 'point_type') else 'middle'
            elif isinstance(point, dict):
                pos = point.get('pos2d', point.get('position', None))
                kind = point.get('kind', 'middle')
            else:
                continue

            if pos is None:
                continue

            if kind == 'endpoint' or kind == 'start':
                color = 'red'
                marker = 'o'
                size = 30
            elif kind == 'end':
                color = 'blue'
                marker = 'o'
                size = 30
            else:  # middle / turning / sample
                color = 'orange'
                marker = '.'
                size = 10

            ax.scatter(pos[0], pos[1], c=color, marker=marker, s=size, zorder=5)

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
            marker = 'x'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=6,
                  edgecolors='black', linewidths=1)

    # 图例
    patches = [
        mpatches.Patch(color='blue', label='Straight Edge'),
        mpatches.Patch(color='green', label='Curved Edge'),
        mpatches.Patch(color='red', label='Endpoint/Turning Point'),
        mpatches.Patch(color='orange', label='Sample Point'),
        mpatches.Patch(color='purple', label='Endpoint Node'),
        mpatches.Patch(color='darkorange', label='Split Node')
    ]
    ax.legend(handles=patches, loc='upper right')

    ax.set_title('Topology Edges with Inspection Points')
    ax.set_aspect('equal')
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 边任务图已保存: {output_path}")


def visualize_edge_task_summary(edge_tasks: List[EdgeTask], output_path: str):
    """
    可视化边任务统计信息

    Args:
        edge_tasks: 边任务列表
        output_path: 输出路径
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 子图1: 每条边的点数分布
    ax1 = axes[0]
    edge_ids = [task.edge_id.split('_')[-1] for task in edge_tasks]
    point_counts = [task.num_points for task in edge_tasks]

    colors = ['green' if task.is_straight else 'orange' for task in edge_tasks]
    ax1.bar(range(len(edge_ids)), point_counts, color=colors, alpha=0.7)
    ax1.set_xlabel('Edge ID')
    ax1.set_ylabel('Number of Inspection Points')
    ax1.set_title('Inspection Points per Edge')
    ax1.set_xticks(range(len(edge_ids)))
    ax1.set_xticklabels(edge_ids, rotation=45, ha='right')

    # 子图2: 边长度分布
    ax2 = axes[1]
    edge_lengths = [task.len2d for task in edge_tasks]
    ax2.bar(range(len(edge_ids)), edge_lengths, color=colors, alpha=0.7)
    ax2.set_xlabel('Edge ID')
    ax2.set_ylabel('Length (px)')
    ax2.set_title('Edge Length Distribution')
    ax2.set_xticks(range(len(edge_ids)))
    ax2.set_xticklabels(edge_ids, rotation=45, ha='right')

    # 图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', alpha=0.7, label='Straight Edge'),
        Patch(facecolor='orange', alpha=0.7, label='Curved Edge')
    ]
    ax2.legend(handles=legend_elements, loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 统计图已保存: {output_path}")
