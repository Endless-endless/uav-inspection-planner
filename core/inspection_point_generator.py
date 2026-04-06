"""
多独立线路巡检点生成模块

功能：
1. 对每条独立线路生成巡检点
2. 支持端点、拐点、采样点检测
3. 映射到 3D 高程
4. 按线路分组管理巡检点

特点：
- 兼容单路径模式
- 预留图像采集字段
- 结构化巡检点数据
"""

import numpy as np
from PIL import Image, ImageDraw
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import os


@dataclass
class LineInspectionPoint:
    """
    线路巡检点数据结构（多独立线路版本）

    Attributes:
        id: 巡检点唯一标识（如 L_001_P_001）
        line_id: 所属线路ID（如 L_001）
        point_type: 点类型（endpoint / turning / sample）
        pixel_position: 2D 像素坐标 (x, y)
        position_3d: 3D 坐标 (x, y, z) 或 None
        line_index: 在线路 polyline 中的索引
        priority: 优先级（high / normal）
        image_path: 采集的图像路径（预留字段）
        detection_result: 检测结果（预留字段）
        status: 状态（uninspected / inspected）
        source_reason: 生成原因说明
    """
    id: str
    line_id: str
    point_type: str  # endpoint / turning / sample
    pixel_position: Tuple[float, float]
    position_3d: Optional[Tuple[float, float, float]] = None
    line_index: int = 0
    priority: str = "normal"
    image_path: Optional[str] = None
    detection_result: Optional[dict] = None
    status: str = "uninspected"
    source_reason: str = ""

    def to_legacy_dict(self) -> dict:
        """
        转换为旧可视化兼容的字典格式

        Returns:
            dict: 兼容旧接口的字典
        """
        return {
            'position': self.position_3d if self.position_3d else (self.pixel_position[0], self.pixel_position[1], 0),
            'index': self.line_index,
            'is_photo': self.point_type in ('endpoint', 'turning'),
            'priority': self.priority,
            'gimbal_angle': -30,
            'action': 'photo' if self.point_type in ('endpoint', 'turning') else 'fly',
            'cost': 0,
            'reason': self.source_reason
        }


def detect_turning_points(
    polyline: List[Tuple[int, int]],
    angle_threshold_deg: float = 25.0
) -> List[int]:
    """
    检测 polyline 中的明显拐点

    Args:
        polyline: 有序折线 [(x, y), ...]
        angle_threshold_deg: 角度阈值（度），小于此值不认为是拐点

    Returns:
        List[int]: 拐点索引列表
    """
    if len(polyline) < 3:
        return []

    turning_indices = []
    angle_threshold_rad = np.radians(angle_threshold_deg)

    for i in range(1, len(polyline) - 1):
        # 计算相邻向量的夹角
        p0 = np.array(polyline[i - 1])
        p1 = np.array(polyline[i])
        p2 = np.array(polyline[i + 1])

        v1 = p1 - p0
        v2 = p2 - p1

        # 避免零向量
        if np.linalg.norm(v1) < 1 or np.linalg.norm(v2) < 1:
            continue

        # 计算夹角
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # 如果夹角大于阈值，认为是拐点
        if angle > angle_threshold_rad:
            turning_indices.append(i)

    # 去重：避免过密拐点
    if len(turning_indices) > 1:
        filtered = [turning_indices[0]]
        for idx in turning_indices[1:]:
            if idx - filtered[-1] > 5:  # 至少间隔5个点
                filtered.append(idx)
        turning_indices = filtered

    return turning_indices


def sample_points_along_polyline(
    polyline: List[Tuple[int, int]],
    spacing: float = 80.0
) -> List[Tuple[int, float, float]]:
    """
    沿折线等间距采样点

    Args:
        polyline: 有序折线 [(x, y), ...]
        spacing: 采样间距（像素）

    Returns:
        List[Tuple[int, float, float]]: (索引, 累积距离, 采样位置) 的列表
    """
    if len(polyline) < 2:
        return []

    samples = []
    cumulative_dist = 0.0
    target_dist = spacing

    for i in range(1, len(polyline)):
        p0 = np.array(polyline[i - 1])
        p1 = np.array(polyline[i])
        segment_dist = np.linalg.norm(p1 - p0)

        while cumulative_dist + segment_dist >= target_dist:
            # 在此线段上采样
            remain = target_dist - cumulative_dist
            t = remain / segment_dist if segment_dist > 0 else 0
            sample_pos = p0 + t * (p1 - p0)

            samples.append((i, target_dist, (float(sample_pos[0]), float(sample_pos[1]))))

            target_dist += spacing

        cumulative_dist += segment_dist

    return samples


def generate_line_inspection_points(
    line,  # IndependentLine 对象
    terrain: Optional[np.ndarray] = None,
    flight_height: float = 25.0,
    spacing: float = 100.0,
    angle_threshold_deg: float = 25.0,
    max_points_per_line: int = 50
) -> List[LineInspectionPoint]:
    """
    对单条独立线路生成巡检点

    规则：
    - 起点、终点必设（endpoint，high priority）
    - 明显拐点必设（turning，high priority）
    - 长线段等距补点（sample，normal priority）
    - 限制每条线最大点数，避免过密

    Args:
        line: IndependentLine 对象
        terrain: 地形高度图（可选）
        flight_height: 飞行高度（米）
        spacing: 采样间距（像素）
        angle_threshold_deg: 拐点角度阈值
        max_points_per_line: 每条线最大点数限制

    Returns:
        List[LineInspectionPoint]: 巡检点列表
    """
    if not line.ordered_pixels:
        return []

    polyline = line.ordered_pixels
    points = []
    point_count = 0
    used_indices = set()  # 记录已使用的索引，避免重复点

    # 1. 端点（起点和终点）- 最高优先级
    if len(polyline) >= 2:
        # 起点
        start_pos = polyline[0]
        start_pos_3d = _map_to_3d(start_pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="endpoint",
            pixel_position=(float(start_pos[0]), float(start_pos[1])),
            position_3d=start_pos_3d,
            line_index=0,
            priority="high",
            status="uninspected",
            source_reason="start_endpoint"
        ))
        used_indices.add(0)
        point_count += 1

        # 终点
        end_pos = polyline[-1]
        end_pos_3d = _map_to_3d(end_pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="endpoint",
            pixel_position=(float(end_pos[0]), float(end_pos[1])),
            position_3d=end_pos_3d,
            line_index=len(polyline) - 1,
            priority="high",
            status="uninspected",
            source_reason="end_endpoint"
        ))
        used_indices.add(len(polyline) - 1)
        point_count += 1

    # 2. 拐点检测 - 高优先级，确保保留
    turning_indices = detect_turning_points(polyline, angle_threshold_deg)
    added_turning = 0
    for idx in turning_indices:
        # 跳过起点和终点附近的拐点
        if idx < 5 or idx > len(polyline) - 5:
            continue

        # 避免重复添加
        if idx in used_indices:
            continue

        pos = polyline[idx]
        pos_3d = _map_to_3d(pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="turning",
            pixel_position=(float(pos[0]), float(pos[1])),
            position_3d=pos_3d,
            line_index=idx,
            priority="high",
            status="uninspected",
            source_reason=f"turning_point_angle_{angle_threshold_deg}"
        ))
        used_indices.add(idx)
        added_turning += 1
        point_count += 1

    # 3. 等距采样点 - 普通优先级，受最大点数限制
    remaining_quota = max_points_per_line - len(points)
    samples = sample_points_along_polyline(polyline, spacing)
    added_samples = 0
    for idx, dist, pos in samples:
        # 达到最大点数限制
        if added_samples >= remaining_quota:
            break

        # 避免与端点重复
        if idx < 5 or idx > len(polyline) - 5:
            continue

        # 避免与拐点重复
        if idx in used_indices:
            continue

        # 避免与已有采样点位置重复
        is_duplicate = any(abs(pos[0] - p.pixel_position[0]) < 3 and
                          abs(pos[1] - p.pixel_position[1]) < 3
                          for p in points if p.point_type == "sample")
        if is_duplicate:
            continue

        pos_3d = _map_to_3d(pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="sample",
            pixel_position=pos,
            position_3d=pos_3d,
            line_index=idx,
            priority="normal",
            status="uninspected",
            source_reason=f"sample_spacing_{spacing}px"
        ))
        used_indices.add(idx)
        added_samples += 1
        point_count += 1

    # 按线路索引排序
    points.sort(key=lambda p: p.line_index)

    print(f"  {line.id}: {len(points)} 个巡检点 (端点: 2, 拐点: {added_turning}, 采样: {added_samples})")

    return points


def _map_to_3d(
    pixel_pos: Tuple[float, int],
    terrain: Optional[np.ndarray],
    flight_height: float
) -> Tuple[float, float, float]:
    """
    将 2D 像素坐标映射到 3D

    Args:
        pixel_pos: (x, y) 像素坐标（可以是整数或浮点数）
        terrain: 地形高度图（可选）
        flight_height: 飞行高度（米）

    Returns:
        Tuple[float, float, float]: (x, y, z) 3D 坐标
    """
    x, y = pixel_pos

    # 安全转换浮点坐标为整数索引
    x_int = int(round(x))
    y_int = int(round(y))

    if terrain is not None:
        # 检查 terrain 是否为二维数组
        if terrain.ndim != 2:
            raise ValueError(f"terrain 必须是二维数组，当前维度: {terrain.ndim}")

        # 边界检查与裁剪
        h, w = terrain.shape
        x_clipped = max(0, min(x_int, w - 1))
        y_clipped = max(0, min(y_int, h - 1))

        terrain_h = float(terrain[y_clipped, x_clipped])
        z = terrain_h + flight_height
    else:
        # 优雅退化：没有地形数据时使用默认高度
        z = flight_height

    return (float(x), float(y), z)


def generate_all_inspection_points(
    lines: List,
    terrain: Optional[np.ndarray] = None,
    flight_height: float = 25.0,
    spacing: float = 100.0,
    angle_threshold_deg: float = 25.0,
    max_points_per_line: int = 50
) -> Tuple[List[LineInspectionPoint], Dict[str, List[LineInspectionPoint]]]:
    """
    批量生成所有线路的巡检点

    Args:
        lines: IndependentLine 列表
        terrain: 地形高度图（可选）
        flight_height: 飞行高度（米）
        spacing: 采样间距（像素）
        angle_threshold_deg: 拐点角度阈值
        max_points_per_line: 每条线最大点数限制

    Returns:
        Tuple[List, Dict]: (所有巡检点, 按线路分组的巡检点字典)
    """
    print("[巡检点生成] 批量生成...")

    all_points = []
    points_by_line = {}

    for line in lines:
        line_points = generate_line_inspection_points(
            line, terrain, flight_height, spacing, angle_threshold_deg, max_points_per_line
        )
        all_points.extend(line_points)
        points_by_line[line.id] = line_points

    # 统计
    total_points = len(all_points)
    endpoint_count = sum(1 for p in all_points if p.point_type == "endpoint")
    turning_count = sum(1 for p in all_points if p.point_type == "turning")
    sample_count = sum(1 for p in all_points if p.point_type == "sample")

    print(f"[巡检点生成] 完成，总计 {total_points} 个点")
    print(f"  - 端点: {endpoint_count}")
    print(f"  - 拐点: {turning_count}")
    print(f"  - 采样点: {sample_count}")

    return all_points, points_by_line


def save_inspection_points_visualization(
    lines: List,
    points_by_line: Dict[str, List[LineInspectionPoint]],
    original_image_path: str,
    output_path: str = "result/step5_line_inspection_points.png"
):
    """
    保存巡检点可视化图像

    Args:
        lines: IndependentLine 列表
        points_by_line: 按线路分组的巡检点
        original_image_path: 原始图像路径
        output_path: 输出路径
    """
    import os
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    # 加载原始图像
    from PIL import Image
    try:
        base_img = Image.open(original_image_path).convert("RGB")
    except:
        max_x = max([max([p[0] for p in line.raw_pixels]) for line in lines] + [100])
        max_y = max([max([p[1] for p in line.raw_pixels]) for line in lines] + [100])
        base_img = Image.new("RGB", (max_x + 50, max_y + 50), (255, 255, 255))

    draw = ImageDraw.Draw(base_img)

    # 颜色方案
    endpoint_color = (255, 0, 0)      # 红色：端点
    turning_color = (0, 255, 0)       # 绿色：拐点
    sample_color = (0, 0, 255)        # 蓝色：采样点

    # 绘制巡检点
    for line_id, points in points_by_line.items():
        for point in points:
            x, y = point.pixel_position

            # 根据类型选择颜色
            if point.point_type == "endpoint":
                color = endpoint_color
                radius = 5
            elif point.point_type == "turning":
                color = turning_color
                radius = 4
            else:  # sample
                color = sample_color
                radius = 3

            # 绘制点
            draw.ellipse([x-radius, y-radius, x+radius, y+radius], fill=color)

    # 保存
    base_img.save(output_path)
    print(f"  [保存] {output_path}")


def get_inspection_points_statistics(
    all_points: List[LineInspectionPoint],
    points_by_line: Dict[str, List[LineInspectionPoint]]
) -> Dict:
    """
    获取巡检点统计信息

    Args:
        all_points: 所有巡检点
        points_by_line: 按线路分组的巡检点

    Returns:
        Dict: 统计信息
    """
    endpoint_count = sum(1 for p in all_points if p.point_type == "endpoint")
    turning_count = sum(1 for p in all_points if p.point_type == "turning")
    sample_count = sum(1 for p in all_points if p.point_type == "sample")
    high_priority_count = sum(1 for p in all_points if p.priority == "high")

    lines_summary = []
    for line_id, points in points_by_line.items():
        line_endpoint = sum(1 for p in points if p.point_type == "endpoint")
        line_turning = sum(1 for p in points if p.point_type == "turning")
        line_sample = sum(1 for p in points if p.point_type == "sample")

        lines_summary.append({
            'line_id': line_id,
            'total_points': len(points),
            'endpoint': line_endpoint,
            'turning': line_turning,
            'sample': line_sample
        })

    return {
        'total_points': len(all_points),
        'total_lines': len(points_by_line),
        'endpoint_count': endpoint_count,
        'turning_count': turning_count,
        'sample_count': sample_count,
        'high_priority_count': high_priority_count,
        'lines_summary': lines_summary
    }


def convert_line_points_to_legacy_format(
    points_by_line: Dict[str, List[LineInspectionPoint]]
) -> List[dict]:
    """
    将多线路巡检点转换为旧可视化兼容格式

    Args:
        points_by_line: 按线路分组的巡检点

    Returns:
        List[dict]: 兼容旧接口的字典列表
    """
    all_legacy = []

    for line_id, points in points_by_line.items():
        for point in points:
            all_legacy.append(point.to_legacy_dict())

    # 按索引排序
    all_legacy.sort(key=lambda d: d['index'])

    return all_legacy


def merge_line_points_to_single_path(
    points_by_line: Dict[str, List[LineInspectionPoint]],
    sort_by_line_length: bool = True
) -> List[LineInspectionPoint]:
    """
    将多线路巡检点合并为单一路径（用于兼容旧单路径模式）

    Args:
        points_by_line: 按线路分组的巡检点
        sort_by_line_length: 是否按线路长度排序后合并

    Returns:
        List[LineInspectionPoint]: 合并后的巡检点列表
    """
    # 获取每条线路的巡检点
    line_items = list(points_by_line.items())

    # 按线路长度排序（长线优先）
    if sort_by_line_length:
        line_items.sort(key=lambda item: len(item[1]), reverse=True)

    # 合并
    merged = []
    index_counter = 0

    for line_id, points in line_items:
        for point in points:
            # 更新索引
            point.line_index = index_counter
            index_counter += 1
            merged.append(point)

    return merged
