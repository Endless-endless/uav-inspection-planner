"""
=====================================================
拓扑建模层 - 保守版本（Phase 1.1）
=====================================================

功能：
- 从独立线路提取拓扑节点
- 根据保守切分规则将线路分割为拓扑边
- 构建节点-边图结构

设计原则（保守）：
- 节点少而准，edge有任务意义
- 只在有意义的地方切分
- 图结构清晰可解释

节点类型：
- endpoint: 线路端点
- split: 保守切分节点（长度/角度）

边切分规则（保守）：
- 长度阈值：累积路径长度超过 EDGE_MAX_LEN = 600px
- 角度阈值：相邻段角度变化超过 ANGLE_THRESH = 60°
- 最小边长：切分产生的边长度必须 >= EDGE_MIN_LEN = 150px
- 角度去重：局部窗口内只保留角度变化最大的切分点
- 切分优先级：端点 → 强角度切分点 → 长度切分点

本次实现不包含：
- 曲率极大值切分（已取消）
- T型连接/junction合并（Phase 1.2再做）
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import numpy as np
from scipy.spatial.distance import euclidean


# =====================================================
# 常量定义（保守版本）
# =====================================================

EDGE_MAX_LEN = 600.0   # 边的最大长度（像素）- 保守：600px
EDGE_MIN_LEN = 150.0   # 边的最小长度（像素）- 新增：避免短边
ANGLE_THRESH = 60.0    # 角度切分阈值（度）- 保守：60°
ANGLE_WINDOW = 10      # 角度去重窗口大小（骨架点数）


# =====================================================
# 数据结构
# =====================================================

@dataclass
class TopoNode:
    """拓扑节点"""
    id: str                          # 唯一标识
    kind: str                        # endpoint / split
    pos2d: Tuple[float, float]       # (x, y)
    pos3d: Tuple[float, float, float] # (x, y, z)
    deg: int                         # 度数（连接的边数）
    line_ids: List[str]              # 属于哪些线路
    pt_indices: Dict[str, int]       # {line_id: 该线路中的骨架点索引}
    meta: Dict = field(default_factory=dict)

    # 保守版本新增字段
    angle_change: float = 0.0        # 如果是split点，记录角度变化量（度）
    split_reason: str = None         # 切分原因：'length'/'angle'


@dataclass
class TopoEdge:
    """拓扑边"""
    id: str                          # 唯一标识
    u: str                           # 起始节点ID
    v: str                           # 结束节点ID
    line_id: str                     # 所属线路ID
    polyline: List[Tuple[float, float]] # 2D几何（从u到v的骨架段）
    len2d: float                     # 长度
    pts_idx: List[int] = field(default_factory=list)  # 该边对应的巡检点索引
    meta: Dict = field(default_factory=dict)

    # 保守版本新增字段
    is_straight: bool = True         # 该边是否近似直线
    split_reason: str = None         # 切分原因：'length'/'angle'


@dataclass
class TopoGraph:
    """拓扑图"""
    nodes: Dict[str, TopoNode] = field(default_factory=dict)
    edges: Dict[str, TopoEdge] = field(default_factory=dict)
    adj: Dict[str, List[str]] = field(default_factory=dict)  # 邻接表

    def add_node(self, node: TopoNode):
        """添加节点"""
        self.nodes[node.id] = node
        if node.id not in self.adj:
            self.adj[node.id] = []

    def add_edge(self, edge: TopoEdge):
        """添加边"""
        self.edges[edge.id] = edge
        # 更新邻接表
        if edge.u not in self.adj:
            self.adj[edge.u] = []
        if edge.v not in self.adj:
            self.adj[edge.v] = []
        if edge.v not in self.adj[edge.u]:
            self.adj[edge.u].append(edge.v)
        if edge.u not in self.adj[edge.v]:
            self.adj[edge.v].append(edge.u)

    def get_node(self, node_id: str) -> Optional[TopoNode]:
        """获取节点"""
        return self.nodes.get(node_id)

    def get_edge(self, edge_id: str) -> Optional[TopoEdge]:
        """获取边"""
        return self.edges.get(edge_id)

    def get_neighbors(self, node_id: str) -> List[str]:
        """获取邻居节点"""
        return self.adj.get(node_id, [])

    def get_edges_at_node(self, node_id: str) -> List[TopoEdge]:
        """获取连接到某节点的所有边"""
        edges = []
        for edge in self.edges.values():
            if edge.u == node_id or edge.v == node_id:
                edges.append(edge)
        return edges


# =====================================================
# 节点检测（保守版本）
# =====================================================

def detect_endpoint_nodes(lines) -> List[TopoNode]:
    """
    检测线路端点节点

    Args:
        lines: 独立线路列表（IndependentLine对象）

    Returns:
        List[TopoNode]: 端点节点列表
    """
    nodes = []
    for line in lines:
        # 起点节点
        start_pos = line.ordered_pixels[0]
        nodes.append(TopoNode(
            id=f"{line.id}_start",
            kind="endpoint",
            pos2d=start_pos,
            pos3d=(start_pos[0], start_pos[1], 0.0),  # 临时Z，后续更新
            deg=1,  # 暂时设为1，后续根据实际连接更新
            line_ids=[line.id],
            pt_indices={line.id: 0}
        ))

        # 终点节点
        end_pos = line.ordered_pixels[-1]
        nodes.append(TopoNode(
            id=f"{line.id}_end",
            kind="endpoint",
            pos2d=end_pos,
            pos3d=(end_pos[0], end_pos[1], 0.0),
            deg=1,
            line_ids=[line.id],
            pt_indices={line.id: len(line.ordered_pixels) - 1}
        ))

    print(f"  [端点检测] 提取了 {len(nodes)} 个端点节点")
    return nodes


def compute_angle_change(points, idx: int) -> float:
    """
    计算某个骨架点处的角度变化量

    Args:
        points: 骨架点列表
        idx: 要计算的点索引

    Returns:
        float: 角度变化量（度），越大表示越弯曲
    """
    if idx <= 0 or idx >= len(points) - 1:
        return 0.0

    v1 = np.array(points[idx - 1]) - np.array(points[idx])
    v2 = np.array(points[idx + 1]) - np.array(points[idx])

    # 避免零向量
    if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
        return 0.0

    # 计算夹角
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cos_angle))

    # 返回角度变化量（180 - angle）
    return abs(180.0 - angle_deg)


def detect_angle_split_points(line, angle_thresh: float = ANGLE_THRESH,
                               window: int = ANGLE_WINDOW) -> List[Tuple[int, float]]:
    """
    检测角度切分点（带去重）

    Args:
        line: 独立线路
        angle_thresh: 角度阈值
        window: 去重窗口大小

    Returns:
        List[Tuple[int, float]]: [(索引, 角度变化量)]
    """
    points = line.ordered_pixels
    if len(points) < 3:
        return []

    # 1. 计算所有点的角度变化
    angle_changes = []
    for i in range(1, len(points) - 1):
        angle_change = compute_angle_change(points, i)
        angle_changes.append((i, angle_change))

    # 2. 筛选超过阈值的点
    candidates = [(i, ang) for i, ang in angle_changes if ang > angle_thresh]

    if not candidates:
        return []

    # 3. 去重：在局部窗口内只保留角度变化最大的点
    deduped = []
    i = 0
    while i < len(candidates):
        idx, angle = candidates[i]

        # 找到窗口内的所有候选点
        window_candidates = [(idx, angle)]
        j = i + 1
        while j < len(candidates) and candidates[j][0] - idx <= window:
            window_candidates.append(candidates[j])
            j += 1

        # 只保留角度变化最大的
        window_candidates.sort(key=lambda x: x[1], reverse=True)
        deduped.append(window_candidates[0])

        # 跳过窗口内的其他点
        i = j

    return deduped


def detect_length_split_points(line, max_len: float = EDGE_MAX_LEN) -> List[int]:
    """
    检测长度切分点

    Args:
        line: 独立线路
        max_len: 长度阈值

    Returns:
        List[int]: 切分点索引列表
    """
    points = line.ordered_pixels
    split_indices = []

    accum_len = 0.0
    for i in range(1, len(points)):
        seg_len = euclidean(points[i-1], points[i])
        accum_len += seg_len

        if accum_len > max_len:
            split_indices.append(i)
            accum_len = 0.0

    return split_indices


def filter_by_min_edge_length(line, split_indices: List[int],
                              min_len: float = EDGE_MIN_LEN) -> List[int]:
    """
    过滤切分点，确保产生的边长度 >= min_len

    Args:
        line: 独立线路
        split_indices: 切分点索引列表
        min_len: 最小边长

    Returns:
        List[int]: 过滤后的切分点索引列表
    """
    if not split_indices:
        return []

    points = line.ordered_pixels
    filtered = [0]  # 起点（索引0）

    for idx in split_indices:
        # 计算从上一个保留点到当前点的长度
        last_idx = filtered[-1]
        edge_len = 0.0
        for i in range(last_idx + 1, idx + 1):
            edge_len += euclidean(points[i-1], points[i])

        # 只有边长度 >= min_len 时才保留该切分点
        if edge_len >= min_len:
            filtered.append(idx)

    # 检查最后一条边（从最后一个切分点到终点）
    last_idx = filtered[-1]
    edge_len = 0.0
    for i in range(last_idx + 1, len(points)):
        edge_len += euclidean(points[i-1], points[i])

    # 如果最后一条边 < min_len，移除最后一个切分点
    if edge_len < min_len and len(filtered) > 1:
        filtered.pop()

    return filtered[1:]  # 返回切分点索引，不包含起点


def detect_split_nodes(line,
                      max_len: float = EDGE_MAX_LEN,
                      min_len: float = EDGE_MIN_LEN,
                      angle_thresh: float = ANGLE_THRESH,
                      angle_window: int = ANGLE_WINDOW) -> List[TopoNode]:
    """
    在单条线路内部检测切分节点（保守版本）

    切分优先级：
    1. 先确定强角度切分点（> angle_thresh，局部去重）
    2. 再补充长度切分点（但需满足最小边长约束）

    Args:
        line: 独立线路（IndependentLine对象）
        max_len: 长度切分阈值
        min_len: 最小边长约束
        angle_thresh: 角度切分阈值（度）
        angle_window: 角度去重窗口

    Returns:
        List[TopoNode]: 切分节点列表
    """
    points = line.ordered_pixels
    if len(points) < 3:
        return []

    # ===== 优先级1: 检测强角度切分点 =====
    angle_split_points = detect_angle_split_points(line, angle_thresh, angle_window)

    # ===== 优先级2: 检测长度切分点 =====
    length_split_indices = detect_length_split_points(line, max_len)

    # ===== 合并切分点并应用最小边长约束 =====
    all_split_indices = set()

    # 先添加角度切分点
    for idx, _ in angle_split_points:
        all_split_indices.add(idx)

    # 再添加长度切分点
    for idx in length_split_indices:
        all_split_indices.add(idx)

    # 转换为排序列表
    sorted_splits = sorted(all_split_indices)

    # 应用最小边长约束过滤
    filtered_splits = filter_by_min_edge_length(line, sorted_splits, min_len)

    # ===== 创建切分节点 =====
    split_nodes = []

    for idx in filtered_splits:
        # 计算角度变化量
        angle_change = compute_angle_change(points, idx)

        # 判断切分原因
        split_reason = "length"
        for angle_idx, ang in angle_split_points:
            if angle_idx == idx:
                split_reason = "angle"
                break

        node = TopoNode(
            id=f"{line.id}_split_{idx}",
            kind="split",
            pos2d=points[idx],
            pos3d=(points[idx][0], points[idx][1], 0.0),
            deg=2,  # 切分节点度数始终为2
            line_ids=[line.id],
            pt_indices={line.id: idx},
            angle_change=angle_change,
            split_reason=split_reason
        )
        split_nodes.append(node)

    if split_nodes:
        print(f"  [切分节点] {line.id}: 检测到 {len(split_nodes)} 个切分点")
        reason_count = {"angle": 0, "length": 0}
        for node in split_nodes:
            reason_count[node.split_reason] += 1
        print(f"    - 角度切分: {reason_count['angle']}, 长度切分: {reason_count['length']}")

    return split_nodes


def detect_topo_nodes(lines, skeleton=None) -> List[TopoNode]:
    """
    检测所有拓扑节点（保守版本）

    Args:
        lines: 独立线路列表
        skeleton: 骨架图像（本次不使用）

    Returns:
        List[TopoNode]: 所有拓扑节点
    """
    print("[拓扑节点] 开始检测（保守版本）...")

    nodes = []

    # (1) 添加所有线路的端点
    endpoints = detect_endpoint_nodes(lines)
    nodes.extend(endpoints)

    # (2) 在每条线路内部检测切分节点
    print("[拓扑节点] 检测切分节点（优先级：角度 > 长度，最小边长={}px）...".format(int(EDGE_MIN_LEN)))
    for line in lines:
        split_nodes = detect_split_nodes(line)
        nodes.extend(split_nodes)

    # 保守版本：暂不做T型连接（Phase 1.2再做）

    print(f"[拓扑节点] 检测完成（去重前）: {len(nodes)} 个节点")
    print(f"  - endpoint: {len([n for n in nodes if n.kind == 'endpoint'])}")
    print(f"  - split: {len([n for n in nodes if n.kind == 'split'])}")

    return nodes


def merge_duplicate_nodes(nodes: List[TopoNode],
                          thresh: float = 25.0) -> Tuple[List[TopoNode], Dict[str, str]]:
    """
    合并重复的拓扑节点（空间聚类方法）

    对所有节点进行空间聚类，距离阈值内的节点合并为一个。

    合并规则：
    - pos2d / pos3d：取平均
    - kind：优先级选择（junction > split > endpoint）
    - line_ids：合并去重
    - pt_indices：合并（保留所有线路的位置索引）
    - 约束：不合并同一线路的两个端点（即使距离很近）

    Args:
        nodes: 所有拓扑节点
        thresh: 距离阈值（像素）

    Returns:
        Tuple[List[TopoNode], Dict[str, str]]: (去重后的节点列表, {旧节点ID: 新节点ID})
    """
    print(f"[节点聚类] 开始空间聚类合并（距离阈值={thresh}px）...")

    if not nodes:
        return [], {}

    # kind优先级
    kind_priority = {"junction": 3, "split": 2, "endpoint": 1}

    # 用于记录已处理的节点索引
    processed = set()
    merged_nodes = []
    old_to_new_id = {}

    cluster_id = 0

    for i, node1 in enumerate(nodes):
        if i in processed:
            continue

        # 找到所有距离相近的节点（无论类型）
        cluster = [node1]
        cluster_indices = [i]

        for j, node2 in enumerate(nodes):
            if j <= i or j in processed:
                continue

            dist = euclidean(node1.pos2d, node2.pos2d)

            if dist < thresh:
                # 检查是否会合并同一线路的两个端点
                conflict = False
                for n1 in cluster:
                    # n1和n2都是endpoint，且有共同的line_id
                    if n1.kind == 'endpoint' and node2.kind == 'endpoint':
                        if set(n1.line_ids) & set(node2.line_ids):
                            # 检查pt_indices是否不同（一个是start，一个是end）
                            if n1.pt_indices != node2.pt_indices:
                                conflict = True
                                break

                if not conflict:
                    cluster.append(node2)
                    cluster_indices.append(j)

        # 如果簇中有多个节点，合并它们
        if len(cluster) > 1:
            # 计算平均位置
            avg_x = sum(n.pos2d[0] for n in cluster) / len(cluster)
            avg_y = sum(n.pos2d[1] for n in cluster) / len(cluster)
            avg_z = sum(n.pos3d[2] for n in cluster) / len(cluster)

            # 合并line_ids（去重）
            merged_line_ids = []
            for n in cluster:
                for line_id in n.line_ids:
                    if line_id not in merged_line_ids:
                        merged_line_ids.append(line_id)

            # 合并pt_indices（每个line_id保留所有索引）
            merged_pt_indices = {}
            for n in cluster:
                for line_id, pt_idx in n.pt_indices.items():
                    if line_id not in merged_pt_indices:
                        merged_pt_indices[line_id] = []
                    # 避免重复添加相同的索引
                    if pt_idx not in merged_pt_indices[line_id]:
                        merged_pt_indices[line_id].append(pt_idx)

            # 如果某个line_id只有一个索引，转换为单个值
            for line_id in list(merged_pt_indices.keys()):
                if len(merged_pt_indices[line_id]) == 1:
                    merged_pt_indices[line_id] = merged_pt_indices[line_id][0]

            # 选择优先级最高的kind
            merged_kind = max((n.kind for n in cluster), key=lambda k: kind_priority.get(k, 0))

            # 生成新的节点ID
            new_id = f"merged_{cluster_id}"
            cluster_id += 1

            # 创建合并后的节点
            merged_node = TopoNode(
                id=new_id,
                kind=merged_kind,
                pos2d=(avg_x, avg_y),
                pos3d=(avg_x, avg_y, avg_z),
                deg=0,  # 后续重新计算
                line_ids=merged_line_ids,
                pt_indices=merged_pt_indices
            )

            # 保留额外属性
            for n in cluster:
                if hasattr(n, 'angle_change') and n.angle_change > 0:
                    merged_node.angle_change = max(merged_node.angle_change, n.angle_change)
                if hasattr(n, 'split_reason') and n.split_reason:
                    if not merged_node.split_reason or merged_node.split_reason == 'length':
                        merged_node.split_reason = n.split_reason

            merged_nodes.append(merged_node)

            # 记录ID映射
            for n in cluster:
                old_to_new_id[n.id] = new_id

            # 标记为已处理
            for idx in cluster_indices:
                processed.add(idx)

            # 统计簇中的节点类型
            kind_count = {}
            for n in cluster:
                kind_count[n.kind] = kind_count.get(n.kind, 0) + 1

            print(f"  [合并簇] {len(cluster)} 个节点 -> {new_id} "
                  f"(kind={merged_kind}, {kind_count}, lines={len(merged_line_ids)})")
        else:
            # 单独的节点，直接保留
            merged_nodes.append(node1)
            old_to_new_id[node1.id] = node1.id
            processed.add(i)

    print(f"[节点聚类] 完成: {len(nodes)} -> {len(merged_nodes)} 个节点")
    print(f"  - endpoint: {len([n for n in merged_nodes if n.kind == 'endpoint'])}")
    print(f"  - split: {len([n for n in merged_nodes if n.kind == 'split'])}")

    return merged_nodes, old_to_new_id


def update_edges_after_merge(edges: List[TopoEdge],
                              old_to_new_id: Dict[str, str]) -> List[TopoEdge]:
    """
    节点合并后更新边的端点引用

    Args:
        edges: 原始边列表
        old_to_new_id: {旧节点ID: 新节点ID}

    Returns:
        List[TopoEdge]: 更新后的边列表
    """
    print("[边更新] 更新边的端点引用...")

    updated_edges = []
    removed_count = 0

    for edge in edges:
        new_u = old_to_new_id.get(edge.u, edge.u)
        new_v = old_to_new_id.get(edge.v, edge.v)

        # 如果边的两端变成了同一个节点，移除这条边
        if new_u == new_v:
            removed_count += 1
            continue

        # 创建更新后的边
        updated_edge = TopoEdge(
            id=edge.id,
            u=new_u,
            v=new_v,
            line_id=edge.line_id,
            polyline=edge.polyline,
            len2d=edge.len2d,
            pts_idx=edge.pts_idx,
            is_straight=edge.is_straight,
            split_reason=edge.split_reason
        )
        updated_edges.append(updated_edge)

    if removed_count > 0:
        print(f"  [边更新] 移除了 {removed_count} 条自环边")

    return updated_edges


# =====================================================
# 边切分
# =====================================================

def split_line_to_edges(line, nodes_on_line: List[TopoNode],
                        min_len: float = EDGE_MIN_LEN) -> List[TopoEdge]:
    """
    将单条线路按节点切分为多条边

    Args:
        line: 独立线路（IndependentLine对象）
        nodes_on_line: 该线路上的所有节点（按位置排序）
        min_len: 最小边长约束（仅对被切分的边生效）

    Returns:
        List[TopoEdge]: 拓扑边列表
    """
    if len(nodes_on_line) < 2:
        return []

    edges = []
    points = line.ordered_pixels
    line_len = len(points)

    # 按骨架点索引排序
    def get_sort_key(node):
        idx = node.pt_indices.get(line.id, 0)
        # 如果idx是列表（合并后的节点），需要根据节点类型选择正确的索引
        if isinstance(idx, list):
            # 对于合并节点，优先选择端点索引（0 或 len-1）
            # 如果列表中有 0，说明包含起点，返回 0
            # 如果列表中有 len-1，说明包含终点，返回 len-1
            # 否则返回平均值（用于 split 节点）
            if 0 in idx:
                return 0
            elif (line_len - 1) in idx:
                return line_len - 1
            else:
                # 对于纯 split 节点合并，返回中间值
                return sum(idx) / len(idx)
        return idx

    nodes_on_line.sort(key=get_sort_key)

    # 如果只有端点节点（没有切分），创建一条完整的边
    if len(nodes_on_line) == 2:
        u_node, v_node = nodes_on_line[0], nodes_on_line[1]
        if u_node.kind == 'endpoint' and v_node.kind == 'endpoint':
            # 获取索引
            u_idx_list = u_node.pt_indices.get(line.id, 0)
            v_idx_list = v_node.pt_indices.get(line.id, line_len - 1)

            # 处理列表情况（合并后的节点）
            if isinstance(u_idx_list, list):
                # 对于起点，选择 0 或最小索引
                if 0 in u_idx_list:
                    u_idx = 0
                else:
                    u_idx = min(u_idx_list)
            else:
                u_idx = u_idx_list

            if isinstance(v_idx_list, list):
                # 对于终点，选择 line_len-1 或最大索引
                if (line_len - 1) in v_idx_list:
                    v_idx = line_len - 1
                else:
                    v_idx = max(v_idx_list)
            else:
                v_idx = v_idx_list

            if u_idx < v_idx:
                edge_polyline = points[u_idx:v_idx+1]

                # 计算长度
                edge_len = 0.0
                for j in range(1, len(edge_polyline)):
                    edge_len += euclidean(edge_polyline[j-1], edge_polyline[j])

                # 判断是否近似直线
                is_straight = True
                if len(edge_polyline) > 2:
                    start = np.array(edge_polyline[0])
                    end = np.array(edge_polyline[-1])
                    vec = end - start
                    vec_len = np.linalg.norm(vec)

                    if vec_len > 1e-6:
                        max_dev = 0.0
                        for pt in edge_polyline[1:-1]:
                            pt_arr = np.array(pt)
                            dev = np.linalg.norm(np.cross(vec, pt_arr - start)) / vec_len
                            max_dev = max(max_dev, dev)
                        is_straight = max_dev < 5.0

                edge = TopoEdge(
                    id=f"{line.id}_edge_0",
                    u=u_node.id,
                    v=v_node.id,
                    line_id=line.id,
                    polyline=edge_polyline,
                    len2d=edge_len,
                    is_straight=is_straight,
                    split_reason=None
                )
                edges.append(edge)
            return edges

    # 有切分节点的情况，应用最小边长约束
    for i in range(len(nodes_on_line) - 1):
        u_node = nodes_on_line[i]
        v_node = nodes_on_line[i + 1]

        # 获取polyline段（从u到v的骨架点）
        u_idx_list = u_node.pt_indices.get(line.id)
        v_idx_list = v_node.pt_indices.get(line.id)

        # 处理列表情况（合并后的节点）
        if isinstance(u_idx_list, list):
            # 对于起点侧，优先选择端点索引（0），否则选最大值
            if 0 in u_idx_list:
                u_idx = 0
            else:
                u_idx = max(u_idx_list)
        else:
            u_idx = u_idx_list

        if isinstance(v_idx_list, list):
            # 对于终点侧，优先选择端点索引（line_len-1），否则选最小值
            if (line_len - 1) in v_idx_list:
                v_idx = line_len - 1
            else:
                v_idx = min(v_idx_list)
        else:
            v_idx = v_idx_list

        if u_idx is None or v_idx is None or u_idx >= v_idx:
            continue  # 跳过无效的边

        edge_polyline = points[u_idx:v_idx+1]

        # 计算长度
        edge_len = 0.0
        for j in range(1, len(edge_polyline)):
            edge_len += euclidean(edge_polyline[j-1], edge_polyline[j])

        # 最小边长约束（仅对切分产生的边生效）
        if edge_len < min_len:
            continue  # 跳过太短的边

        # 判断是否近似直线
        is_straight = True
        if len(edge_polyline) > 2:
            # 计算首尾向量与中间点的偏差
            start = np.array(edge_polyline[0])
            end = np.array(edge_polyline[-1])
            vec = end - start
            vec_len = np.linalg.norm(vec)

            if vec_len > 1e-6:
                max_dev = 0.0
                for pt in edge_polyline[1:-1]:
                    pt_arr = np.array(pt)
                    # 计算点到直线的距离
                    dev = np.linalg.norm(np.cross(vec, pt_arr - start)) / vec_len
                    max_dev = max(max_dev, dev)

                # 如果最大偏差 > 5px，认为不是直线
                is_straight = max_dev < 5.0

        # 确定切分原因
        split_reason = None
        if v_node.kind == "split":
            split_reason = v_node.split_reason

        edge = TopoEdge(
            id=f"{line.id}_edge_{i}",
            u=u_node.id,
            v=v_node.id,
            line_id=line.id,
            polyline=edge_polyline,
            len2d=edge_len,
            is_straight=is_straight,
            split_reason=split_reason
        )
        edges.append(edge)

    return edges


def split_lines_to_edges(lines, nodes: List[TopoNode],
                         min_len: float = EDGE_MIN_LEN) -> List[TopoEdge]:
    """
    将所有线路切分为拓扑边

    Args:
        lines: 独立线路列表
        nodes: 所有拓扑节点
        min_len: 最小边长约束

    Returns:
        List[TopoEdge]: 所有拓扑边
    """
    print("[拓扑边] 开始切分线路...")

    all_edges = []

    for line in lines:
        # 找到该线路上的所有节点
        nodes_on_line = []
        for node in nodes:
            if line.id in node.line_ids:
                nodes_on_line.append(node)

        if len(nodes_on_line) < 2:
            print(f"  [警告] {line.id} 节点数不足: {len(nodes_on_line)}")
            continue

        # 切分
        edges = split_line_to_edges(line, nodes_on_line, min_len)
        all_edges.extend(edges)

        if len(edges) > 1:
            print(f"  [切分] {line.id}: {len(edges)} 条边")
        elif len(edges) == 1:
            pass  # 不切分的线路不打印

    print(f"[拓扑边] 总计 {len(all_edges)} 条边")
    return all_edges


# =====================================================
# 图构建
# =====================================================

def build_topo_graph(nodes: List[TopoNode], edges: List[TopoEdge]) -> TopoGraph:
    """
    构建拓扑图

    Args:
        nodes: 所有拓扑节点
        edges: 所有拓扑边

    Returns:
        TopoGraph: 拓扑图
    """
    print("[拓扑图] 开始构建...")

    graph = TopoGraph()

    # 添加节点
    for node in nodes:
        graph.add_node(node)

    # 添加边
    for edge in edges:
        graph.add_edge(edge)

    # 更新节点度数
    for node_id in graph.nodes:
        node = graph.nodes[node_id]
        node.deg = len(graph.get_neighbors(node_id))

    # 统计
    straight_edges = sum(1 for e in edges if e.is_straight)
    curved_edges = len(edges) - straight_edges

    print(f"[拓扑图] 构建完成:")
    print(f"  - 节点: {len(graph.nodes)}")
    print(f"  - 边: {len(graph.edges)}")
    print(f"  - 直线边: {straight_edges}, 曲线边: {curved_edges}")
    print(f"  - 连通分量: {count_connected_components(graph)}")

    return graph


def count_connected_components(graph: TopoGraph) -> int:
    """
    计算图的连通分量数

    Args:
        graph: 拓扑图

    Returns:
        int: 连通分量数
    """
    visited = set()
    components = 0

    for node_id in graph.nodes:
        if node_id not in visited:
            # BFS遍历
            queue = [node_id]
            visited.add(node_id)
            while queue:
                curr = queue.pop(0)
                for neighbor in graph.get_neighbors(curr):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            components += 1

    return components


# =====================================================
# 辅助函数
# =====================================================

def update_node_3d(nodes: List[TopoNode], height_map: np.ndarray):
    """
    使用高程图更新节点的3D坐标

    Args:
        nodes: 拓扑节点列表
        height_map: 高程图
    """
    h, w = height_map.shape

    for node in nodes:
        x, y = int(node.pos2d[0]), int(node.pos2d[1])
        if 0 <= x < w and 0 <= y < h:
            z = height_map[y, x]
            node.pos3d = (node.pos2d[0], node.pos2d[1], z)


def visualize_topo_graph(graph: TopoGraph, lines, output_path: str,
                        show_edge_numbers: bool = True):
    """
    可视化拓扑图

    Args:
        graph: 拓扑图
        lines: 原始线路（用于背景）
        output_path: 输出路径
        show_edge_numbers: 是否显示边编号
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    fig, ax = plt.subplots(figsize=(14, 14))

    # 绘制原始线路作为背景
    for line in lines:
        points = np.array(line.ordered_pixels)
        ax.plot(points[:, 0], points[:, 1], 'lightgray', linewidth=1, alpha=0.5)

    # 绘制边
    for edge in graph.edges.values():
        polyline = np.array(edge.polyline)
        color = 'blue' if edge.is_straight else 'green'
        ax.plot(polyline[:, 0], polyline[:, 1], color=color, linewidth=1.5, alpha=0.7)

        # 显示边编号
        if show_edge_numbers:
            mid_idx = len(polyline) // 2
            mid_x, mid_y = polyline[mid_idx]
            edge_num = edge.id.split('_')[-1]
            ax.text(mid_x, mid_y, edge_num, fontsize=7, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6))

    # 绘制节点
    for node in graph.nodes.values():
        x, y = node.pos2d
        if node.kind == 'endpoint':
            color = 'red'
            marker = 'o'
            size = 80
        elif node.kind == 'split':
            # 根据切分原因区分颜色
            if node.split_reason == 'angle':
                color = 'orange'  # 角度切分
            else:
                color = 'yellow'  # 长度切分
            marker = 's'
            size = 60
        else:
            color = 'gray'
            marker = 'x'
            size = 30

        ax.scatter(x, y, c=color, marker=marker, s=size, zorder=5,
                  edgecolors='black', linewidths=0.5)

    # 图例
    patches = [
        mpatches.Patch(color='red', label='Endpoint'),
        mpatches.Patch(color='orange', label='Split (Angle)'),
        mpatches.Patch(color='yellow', label='Split (Length)'),
        mpatches.Patch(color='blue', label='Straight Edge'),
        mpatches.Patch(color='green', label='Curved Edge')
    ]
    ax.legend(handles=patches, loc='upper right')

    ax.set_title(f'Topology Graph (Conservative v1.1)\nNodes: {len(graph.nodes)}, Edges: {len(graph.edges)}')
    ax.set_aspect('equal')
    ax.invert_yaxis()  # 图像坐标系

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [可视化] 拓扑图已保存: {output_path}")


def compute_topo_stats(graph: TopoGraph, lines) -> Dict:
    """
    计算拓扑统计信息

    Args:
        graph: 拓扑图
        lines: 原始线路列表

    Returns:
        Dict: 统计信息
    """
    # 节点类型统计
    node_kinds = {}
    for node in graph.nodes.values():
        node_kinds[node.kind] = node_kinds.get(node.kind, 0) + 1

    # 节点度数统计
    degree_dist = {}
    for node in graph.nodes.values():
        deg = node.deg
        degree_dist[deg] = degree_dist.get(deg, 0) + 1

    # 边统计
    straight_edges = sum(1 for e in graph.edges.values() if e.is_straight)
    curved_edges = len(graph.edges) - straight_edges

    # 边长度统计
    edge_lengths = [e.len2d for e in graph.edges.values()]
    avg_edge_len = np.mean(edge_lengths) if edge_lengths else 0
    min_edge_len = min(edge_lengths) if edge_lengths else 0
    max_edge_len = max(edge_lengths) if edge_lengths else 0

    # 按线路统计边数
    edges_by_line = {}
    for edge in graph.edges.values():
        edges_by_line[edge.line_id] = edges_by_line.get(edge.line_id, 0) + 1

    # 切分原因统计
    split_reasons = {}
    for node in graph.nodes.values():
        if node.kind == 'split' and node.split_reason:
            split_reasons[node.split_reason] = split_reasons.get(node.split_reason, 0) + 1

    stats = {
        'total_nodes': len(graph.nodes),
        'total_edges': len(graph.edges),
        'node_kinds': node_kinds,
        'degree_dist': degree_dist,
        'straight_edges': straight_edges,
        'curved_edges': curved_edges,
        'avg_edge_length': avg_edge_len,
        'min_edge_length': min_edge_len,
        'max_edge_length': max_edge_len,
        'edges_by_line': edges_by_line,
        'split_reasons': split_reasons,
        'num_lines': len(lines),
        'avg_edges_per_line': len(graph.edges) / len(lines) if lines else 0
    }

    return stats
