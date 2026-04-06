"""
独立线路识别模块

功能：
1. 从 skeleton 中提取多条独立红色电网线路
2. 每条线路保持独立，不自动连接
3. 将单条连通域排序成有序 polyline
4. 提供线路统计与可视化

特点：
- 基于 skeleton 提取独立连通域
- 8 邻域连通性判断
- 自动过滤噪声线段
- 端点检测与路径排序
"""

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import label
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Set
from collections import defaultdict, deque
import os


@dataclass
class IndependentLine:
    """
    独立线路数据结构

    Attributes:
        id: 线路唯一标识（如 L_001, L_002）
        raw_pixels: 原始骨架像素集合 [(x, y), ...]
        ordered_pixels: 排序后的有序折线 [(x, y), ...]
        length_2d: 2D 长度（像素距离）
        bbox: 边界框 (min_x, min_y, max_x, max_y)
        component_size: 连通域大小（像素数）
        endpoints: 端点列表 [(x, y), ...]
        is_valid: 是否为有效线路（非噪声）
    """
    id: str
    raw_pixels: List[Tuple[int, int]]
    ordered_pixels: List[Tuple[int, int]] = field(default_factory=list)
    length_2d: float = 0.0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    component_size: int = 0
    endpoints: List[Tuple[int, int]] = field(default_factory=list)
    is_valid: bool = True

    def __post_init__(self):
        """初始化后处理：计算 bbox 和 component_size"""
        if self.raw_pixels:
            x_coords = [p[0] for p in self.raw_pixels]
            y_coords = [p[1] for p in self.raw_pixels]
            self.bbox = (min(x_coords), min(y_coords), max(x_coords), max(y_coords))
            self.component_size = len(self.raw_pixels)

    def compute_length_2d(self) -> float:
        """计算 2D 长度"""
        if not self.ordered_pixels:
            return 0.0

        length = 0.0
        for i in range(1, len(self.ordered_pixels)):
            x1, y1 = self.ordered_pixels[i-1]
            x2, y2 = self.ordered_pixels[i]
            length += np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        self.length_2d = length
        return length


def extract_independent_line_components(
    skeleton: np.ndarray,
    min_pixels: int = 20
) -> List[IndependentLine]:
    """
    从 skeleton 中提取所有独立连通域

    Args:
        skeleton: 骨架图像（二值，0/255 或 0/1）
        min_pixels: 最小像素数阈值，小于此值的连通域视为噪声

    Returns:
        List[IndependentLine]: 独立线路列表（未排序）
    """
    print("[独立线路] 提取连通域...")

    # 确保是二值图像
    binary = (skeleton > 0).astype(np.uint8)

    # 使用 8 邻域标记连通域
    structure = np.ones((3, 3), dtype=np.uint8)
    labeled, num_features = label(binary, structure=structure)

    print(f"  检测到 {num_features} 个连通域")

    # 提取每个连通域的像素
    lines = []
    line_id = 0

    for feature_idx in range(1, num_features + 1):
        component_mask = (labeled == feature_idx)
        pixels = np.argwhere(component_mask)

        if len(pixels) < min_pixels:
            print(f"  [过滤] 连通域 {feature_idx}: {len(pixels)} 像素 (小于阈值 {min_pixels})")
            continue

        # 转换为 (x, y) 格式
        raw_pixels = [(int(col), int(row)) for row, col in pixels]

        # 生成线路 ID
        line_id_str = f"L_{line_id:03d}"
        line_id += 1

        line = IndependentLine(
            id=line_id_str,
            raw_pixels=raw_pixels
        )
        lines.append(line)

        print(f"  [提取] {line.id}: {len(raw_pixels)} 像素")

    print(f"  [结果] 有效线路: {len(lines)} 条")
    return lines


def detect_endpoints(
    pixels: Set[Tuple[int, int]],
    skeleton_shape: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """
    检测连通域的端点（度为1的点）

    Args:
        pixels: 像素集合 {(x, y), ...}
        skeleton_shape: 骨架图像形状 (height, width)

    Returns:
        List[Tuple[int, int]]: 端点列表 [(x, y), ...]
    """
    # 8 邻域方向
    neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

    degrees = {}
    for x, y in pixels:
        degree = 0
        for dy, dx in neighbors:
            nx, ny = x + dx, y + dy
            if (nx, ny) in pixels:
                degree += 1
        degrees[(x, y)] = degree

    # 端点是度为 1 的点
    endpoints = [point for point, deg in degrees.items() if deg == 1]

    # 如果没有端点（闭环），选择两个最远点
    if not endpoints and len(pixels) >= 2:
        pixel_list = list(pixels)
        max_dist = 0
        pair = (pixel_list[0], pixel_list[1])

        for i in range(len(pixel_list)):
            for j in range(i + 1, min(i + 100, len(pixel_list))):  # 限制搜索范围
                p1 = pixel_list[i]
                p2 = pixel_list[j]
                dist = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
                if dist > max_dist:
                    max_dist = dist
                    pair = (p1, p2)

        endpoints = [pair[0], pair[1]]

    return endpoints


def order_component_as_polyline(
    line: IndependentLine
) -> List[Tuple[int, int]]:
    """
    将单个连通域排序成有序 polyline

    策略：
    1. 检测端点
    2. 从一个端点开始，沿连通域走到另一个端点
    3. 处理分支点（优先选未访问的邻居）

    Args:
        line: IndependentLine 对象

    Returns:
        List[Tuple[int, int]]: 排序后的像素序列 [(x, y), ...]
    """
    if not line.raw_pixels:
        return []

    pixel_set = set(line.raw_pixels)
    height, width = 3000, 3000  # 默认值，实际使用 bbox 可能更好

    # 检测端点
    endpoints = detect_endpoints(pixel_set, (height, width))
    line.endpoints = endpoints

    if not endpoints:
        # 无端点情况：按任意顺序返回
        return line.raw_pixels.copy()

    # 从第一个端点开始遍历
    start_point = endpoints[0]

    # 8 邻域
    neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

    # BFS/DFS 遍历，但优先走"直路"
    visited = set()
    ordered = []
    queue = deque([start_point])

    while queue:
        point = queue.popleft()
        if point in visited:
            continue

        visited.add(point)
        ordered.append(point)

        x, y = point
        neighbors_list = []

        for dy, dx in neighbors:
            nx, ny = x + dx, y + dy
            if (nx, ny) in pixel_set and (nx, ny) not in visited:
                neighbors_list.append((nx, ny))

        # 优先选择度较小的邻居（即更可能是路径延续而非分支）
        neighbors_list.sort(key=lambda p: len([
            n for n in [(p[0]+dx, p[1]+dy) for dy, dx in neighbors]
            if n in pixel_set
        ]))

        for neighbor in neighbors_list:
            if neighbor not in visited:
                queue.appendleft(neighbor)  # BFS，但优先处理新加入的点

    # 如果遍历后还有未访问的点（分支），尝试追加
    unvisited = pixel_set - visited
    if unvisited:
        # 简单策略：按距离最近追加
        for up in unvisited:
            if ordered:
                # 找到最近的已访问点
                distances = [(up[0]-p[0])**2 + (up[1]-p[1])**2 for p in ordered]
                min_dist_idx = min(range(len(distances)), key=distances.__getitem__)
                # 在该位置后插入（简化处理：直接追加）
                ordered.append(up)

    line.ordered_pixels = ordered
    line.compute_length_2d()

    return ordered


def extract_independent_lines_from_skeleton(
    skeleton: np.ndarray,
    min_pixels: int = 20,
    sort_polylines: bool = True
) -> List[IndependentLine]:
    """
    从 skeleton 中提取多条独立线路（完整流程）

    步骤：
    1. 提取所有独立连通域
    2. 过滤噪声线段
    3. 对每条线路排序成 polyline
    4. 计算统计信息

    Args:
        skeleton: 骨架图像（二值）
        min_pixels: 最小像素数阈值
        sort_polylines: 是否对 polyline 排序

    Returns:
        List[IndependentLine]: 独立线路列表
    """
    print("[独立线路] 开始提取...")

    # 提取连通域
    lines = extract_independent_line_components(skeleton, min_pixels)

    # 排序 polyline
    if sort_polylines:
        print("[独立线路] 排序 polyline...")
        for line in lines:
            line.ordered_pixels = order_component_as_polyline(line)
            print(f"  {line.id}: {len(line.ordered_pixels)} 有序点, 长度 {line.length_2d:.1f}px")

    # 按长度排序（长线在前）
    lines.sort(key=lambda l: l.length_2d, reverse=True)

    print(f"[独立线路] 完成，共 {len(lines)} 条有效线路")

    return lines


def save_independent_lines_visualization(
    lines: List[IndependentLine],
    original_image_path: str,
    output_path: str = "result/step4_independent_lines.png"
):
    """
    保存独立线路的可视化图像

    每条线路用不同颜色标注

    Args:
        lines: 独立线路列表
        original_image_path: 原始图像路径（作为背景）
        output_path: 输出路径
    """
    import os
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    # 加载原始图像
    from PIL import Image
    try:
        base_img = Image.open(original_image_path).convert("RGB")
    except:
        # 如果原始图像不可用，创建空白图像
        max_x = max([max([p[0] for p in line.raw_pixels]) for line in lines] + [100])
        max_y = max([max([p[1] for p in line.raw_pixels]) for line in lines] + [100])
        base_img = Image.new("RGB", (max_x + 50, max_y + 50), (255, 255, 255))

    draw = ImageDraw.Draw(base_img)

    # 为每条线路生成颜色
    colors = [
        (255, 0, 0),    # 红
        (0, 255, 0),    # 绿
        (0, 0, 255),    # 蓝
        (255, 255, 0),  # 黄
        (255, 0, 255),  # 品红
        (0, 255, 255),  # 青
        (128, 0, 0),    # 深红
        (0, 128, 0),    # 深绿
    ]

    # 绘制每条线路
    for i, line in enumerate(lines):
        color = colors[i % len(colors)]

        # 绘制线路
        for x, y in line.ordered_pixels:
            draw.point((x, y), fill=color)

        # 标注端点
        for x, y in line.endpoints:
            draw.ellipse([x-3, y-3, x+3, y+3], fill=(255, 255, 255), outline=color)

    base_img.save(output_path)
    print(f"  [保存] {output_path}")


def get_lines_statistics(lines: List[IndependentLine]) -> Dict:
    """
    获取线路统计信息

    Args:
        lines: 独立线路列表

    Returns:
        Dict: 统计信息字典
    """
    if not lines:
        return {
            'total_lines': 0,
            'total_length': 0.0,
            'avg_length': 0.0,
            'max_length': 0.0,
            'min_length': 0.0,
            'total_points': 0,
            'lines_detail': []
        }

    lengths = [line.length_2d for line in lines]
    points = [len(line.ordered_pixels) for line in lines]

    return {
        'total_lines': len(lines),
        'total_length': sum(lengths),
        'avg_length': np.mean(lengths),
        'max_length': max(lengths),
        'min_length': min(lengths),
        'total_points': sum(points),
        'lines_detail': [
            {
                'id': line.id,
                'length': line.length_2d,
                'points': len(line.ordered_pixels),
                'endpoints': line.endpoints
            }
            for line in lines
        ]
    }


# 便捷函数：从 PowerlinePlannerV3 对象中提取独立线路
def extract_lines_from_planner(planner, min_pixels: int = 20) -> List[IndependentLine]:
    """
    从现有规划器对象中提取独立线路

    Args:
        planner: PowerlinePlannerV3 实例
        min_pixels: 最小像素数阈值

    Returns:
        List[IndependentLine]: 独立线路列表
    """
    if planner.skeleton is None:
        planner.step3_skeletonize()

    return extract_independent_lines_from_skeleton(planner.skeleton, min_pixels)
