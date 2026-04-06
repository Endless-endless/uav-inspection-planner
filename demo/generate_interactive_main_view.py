"""
正式主展示页生成器 - 交互式版本
基于 data/test.png 的真实地图 + Plotly 交互式可视化

底图：
- data/test.png：正式主展示底图（本脚本使用）

输出：
- result/latest/main_view_interactive.html（真正的交互式主展示页）

特点：
- 使用 Plotly layout.images 显示真实地图
- 路径可交互（hover显示信息）
- inspection points 可交互（hover显示详情）
- 图层控制（sample points开关）
- 系统界面风格（不是报告页）
"""

import os
import sys
import numpy as np
import json
from PIL import Image
from typing import List, Tuple
from pathlib import Path

# Plotly 容错处理
try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    print("[WARN] plotly 未安装，交互式可视化功能将不可用")
    go = None
    make_subplots = None
    PLOTLY_AVAILABLE = False


def get_result_image_path(point_id: str) -> str:
    """
    获取巡检点结果图片路径

    检查 result/latest/inspection_images/IP_xxxx.jpg 是否存在

    Args:
        point_id: 巡检点ID，格式如 "IP_0001"

    Returns:
        图片相对路径（用于网页引用），如果不存在则返回 None
        注意：返回的是相对于 result/latest/ 的路径
    """
    # HTML 文件在 result/latest/main_view_interactive.html
    # 图片在 result/latest/inspection_images/
    # 所以相对路径是 inspection_images/{point_id}.jpg
    rel_path = f"inspection_images/{point_id}.jpg"
    abs_path = Path("result/latest") / rel_path

    if abs_path.exists():
        return rel_path
    return None


def rdp_simplify(points: List[Tuple[float, float]], epsilon: float = 1.0) -> List[Tuple[float, float]]:
    """
    Ramer-Douglas-Peucker 折线简化算法
    用于去除路径中的小毛刺和微小抖动，保留主拐点

    Args:
        points: 原始点列表 [(x, y), ...]
        epsilon: 简化容差（像素），越小越保留细节

    Returns:
        简化后的点列表
    """
    if len(points) <= 2:
        return points[:]

    # 找到起点和终点
    start, end = points[0], points[-1]

    # 计算所有点到起终点连线的垂直距离
    def perpendicular_distance(point, line_start, line_end):
        """计算点到线段的垂直距离"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end

        # 线段长度
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return np.sqrt((x0 - x1)**2 + (y0 - y1)**2)

        # 垂直距离公式
        return abs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1) / np.sqrt(dx**2 + dy**2)

    # 找到距离最大的点
    max_dist = 0
    max_idx = 0
    for i, point in enumerate(points[1:-1], 1):
        dist = perpendicular_distance(point, start, end)
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    # 如果最大距离大于容差，递归简化
    if max_dist > epsilon:
        # 递归处理左右两部分
        left = rdp_simplify(points[:max_idx+1], epsilon)
        right = rdp_simplify(points[max_idx:], epsilon)
        # 合并（去除重复的中间点）
        return left[:-1] + right
    else:
        # 直接返回起终点
        return [start, end]


def lighten_geometry_by_resampling(points: List[Tuple[float, float]],
                                    target_spacing: float = 15.0) -> List[Tuple[float, float]]:
    """
    通过重新采样让路径更平滑（保留主拐点，去除小抖动）

    Args:
        points: 原始点列表
        target_spacing: 目标点间距（像素）

    Returns:
        重采样后的点列表
    """
    if len(points) <= 2:
        return points[:]

    result = [points[0]]
    accumulated_dist = 0

    for i in range(1, len(points)):
        x0, y0 = points[i-1]
        x1, y1 = points[i]
        dist = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)

        accumulated_dist += dist

        if accumulated_dist >= target_spacing:
            result.append(points[i])
            accumulated_dist = 0

    # 确保包含终点
    if result[-1] != points[-1]:
        result.append(points[-1])

    return result


def load_mission_data(mission_json_path='result/latest/mission_output.json'):
    """加载任务数据"""
    with open(mission_json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 提取路径
    full_path_dict = data.get('full_path', {})
    path_2d = full_path_dict.get('full_path_2d', [])
    path_3d = full_path_dict.get('full_path_3d', [])

    # 提取巡检点
    inspection_points = data.get('inspection_points', [])

    # 提取 segments
    segments = data.get('segments', [])

    # 如果 path_2d 为空，从 segments 构建
    if not path_2d and segments:
        path_2d = []
        for seg in segments:
            geo = seg.get('geometry_2d', [])
            if geo:
                path_2d.extend(geo)
            # 段之间断开（添加 None）
            path_2d.append(None)
        # 移除最后的 None
        if path_2d and path_2d[-1] is None:
            path_2d.pop()

    # 统计信息
    statistics = data.get('statistics', {})

    # 天气信息
    weather = data.get('weather', {})

    return path_2d, path_3d, inspection_points, segments, statistics, weather


def create_interactive_main_view(
    map_image_path='data/test.png',
    mission_json_path='result/latest/mission_output.json',
    output_html_path='result/latest/main_view_interactive.html',
    weather=None
):
    """
    创建交互式主展示页

    使用 Plotly 的 layout.images 方式显示真实地图
    确保地图的真实内容清晰可见，不是紫色平面

    Args:
        map_image_path: 地图图片路径
        mission_json_path: 任务 JSON 路径
        output_html_path: 输出 HTML 路径
        weather: 可选的天气信息字典，如果提供则直接使用，不从 JSON 读取
    """

    # 检查 Plotly 是否可用
    if not PLOTLY_AVAILABLE:
        print("[ERROR] Plotly 未安装，无法生成交互式可视化页面")
        print("[解决方法] pip install plotly")
        return None

    print("="*70)
    print("生成交互式主展示页")
    print("="*70)

    # =====================================================
    # 加载数据
    # =====================================================
    print("\n[INFO] 加载数据...")

    path_2d, path_3d, inspection_points, segments, statistics, _ = load_mission_data(mission_json_path)

    # 如果提供了 weather 参数，使用它；否则使用从 JSON 加载的天气（可能为空）
    if weather is not None:
        print(f"[INFO] 使用传入的天气信息: {weather.get('label', 'unknown')}")
    else:
        # 尝试从 JSON 加载天气信息（兼容旧代码）
        _, _, _, _, _, weather = load_mission_data(mission_json_path)
        if not weather:
            print("[WARN] 未提供天气信息参数，且 JSON 中也无天气数据")

    print(f"[INFO] 路径点数: {len(path_2d)}")
    print(f"[INFO] 巡检点数: {len(inspection_points)}")
    print(f"[INFO] 段数: {len(segments)}")

    # =====================================================
    # 生成 Inspection Sampling Points（带完整元数据）
    # =====================================================
    # 在 inspect segments 上按固定间距生成采样点
    # 这些点是后续"点击交互 + 照片绑定"的锚点
    inspection_spacing_px = 70  # 可配置参数：采样间距（像素）
    inspection_points_with_metadata = []  # 存储带元数据的巡检点

    point_counter = 0  # 全局巡检点序号

    for seg in segments:
        if seg.get('type') == 'inspect':
            geo = seg.get('geometry_2d', [])
            if len(geo) < 2:
                continue

            seg_id = seg.get('segment_id')
            edge_id = seg.get('edge_id')
            group_id = seg.get('group_id')

            # 添加起点作为巡检点
            start_point = geo[0]
            point_id = f'IP_{point_counter:04d}'
            inspection_points_with_metadata.append({
                'point_id': point_id,
                'segment_id': seg_id,
                'edge_id': edge_id,
                'group_id': group_id,
                'sequence_in_path': point_counter,
                'position': list(start_point),
                'type': 'sample',
                'status': 'uninspected',
                'result_image': get_result_image_path(point_id),  # 绑定图片路径
                'result_text': '',
                'photo_path': get_result_image_path(point_id),  # 同步到 photo_path
                'description': f'Start of {edge_id}'
            })
            point_counter += 1

            # 沿着整条线按固定间距采样
            accumulated_dist = 0  # 累计距离
            last_sample_dist = 0  # 上一次采样的距离位置

            for i in range(len(geo) - 1):
                p1 = geo[i]
                p2 = geo[i + 1]
                import math
                segment_dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                accumulated_dist += segment_dist

                # 检查是否需要在这段上插入采样点
                while accumulated_dist >= last_sample_dist + inspection_spacing_px:
                    # 计算采样点在这段上的位置
                    target_dist = last_sample_dist + inspection_spacing_px
                    dist_into_segment = target_dist - (accumulated_dist - segment_dist)
                    ratio = dist_into_segment / segment_dist if segment_dist > 0 else 0

                    sample_x = p1[0] + (p2[0] - p1[0]) * ratio
                    sample_y = p1[1] + (p2[1] - p1[1]) * ratio

                    point_id = f'IP_{point_counter:04d}'
                    inspection_points_with_metadata.append({
                        'point_id': point_id,
                        'segment_id': seg_id,
                        'edge_id': edge_id,
                        'group_id': group_id,
                        'sequence_in_path': point_counter,
                        'position': [sample_x, sample_y],
                        'type': 'sample',
                        'status': 'uninspected',
                        'result_image': get_result_image_path(point_id),  # 绑定图片路径
                        'result_text': '',
                        'photo_path': get_result_image_path(point_id),  # 同步到 photo_path
                        'description': f'Inspection point on {edge_id}'
                    })
                    point_counter += 1
                    last_sample_dist = target_dist

    print(f"[INFO] 生成的 inspection sampling points: {len(inspection_points_with_metadata)}")
    print(f"[INFO] 间距: {inspection_spacing_px}px")

    # =====================================================
    # 加载地图并获取尺寸
    # =====================================================
    print(f"\n[INFO] 加载地图: {map_image_path}")

    map_img = Image.open(map_image_path)
    map_width, map_height = map_img.size

    print(f"[INFO] 地图尺寸: {map_width} x {map_height}")
    print(f"[INFO] 坐标系: origin='upper' (图像坐标系)")

    # 将图片转为 base64（用于嵌入 HTML）
    import base64
    from io import BytesIO

    buffered = BytesIO()
    map_img.save(buffered, format="PNG")
    img_base64 = base64.b64encode(buffered.getvalue()).decode()
    img_data_uri = f"data:image/png;base64,{img_base64}"

    # =====================================================
    # 创建 Plotly 图形
    # =====================================================
    print("\n[INFO] 创建交互式图形...")

    fig = go.Figure()

    # =====================================================
    # 关键：使用 layout.images 添加真实地图作为背景
    # =====================================================
    # 这样可以确保地图的真实内容清晰可见，不是紫色平面
    fig.add_layout_image(
        dict(
            source=img_data_uri,  # 使用 base64 编码的图片
            xref="x",
            yref="y",
            x=0,
            y=map_height,
            sizex=map_width,
            sizey=map_height,
            sizing="stretch",
            layer="below",  # 放在所有图层下面
            opacity=1.0
        )
    )

    # =====================================================
    # 叠加路径 - 单条主路径，内部按段类型切换线型
    # =====================================================
    # Inspect 段: 实线
    # Connect 段: dashed
    # 整条路径连续，只在不同段之间断开（用 None 分隔）

    if segments:
        # 按 segments 顺序构建主路径
        main_path_x = []
        main_path_y = []
        main_path_text = []

        for seg in segments:
            geo = seg.get('geometry_2d', [])
            if not geo:
                continue

            seg_type = seg.get('type')
            seg_id = seg.get('segment_id')
            edge_id = seg.get('edge_id', 'N/A')
            length = seg.get('length', 0)

            x_coords = [p[0] for p in geo]
            y_coords = [p[1] for p in geo]

            # 为每个点创建 hover 信息
            hover = [f"{seg_type.upper()} {seg_id}<br>Edge: {edge_id}<br>长度: {length:.1f}px<br>坐标: ({x:.1f}, {y:.1f})"
                    for x, y in zip(x_coords, y_coords)]

            # 添加当前段的数据
            main_path_x.extend(x_coords)
            main_path_y.extend(y_coords)
            main_path_text.extend(hover)

            # 段之间断开（添加 None）
            main_path_x.append(None)
            main_path_y.append(None)
            main_path_text.append(None)

        # 移除最后的 None
        if main_path_x and main_path_x[-1] is None:
            main_path_x.pop()
            main_path_y.pop()
            main_path_text.pop()

        # 单条主路径：使用统一的颜色，内部按段类型切换线型
        # 由于 Plotly 限制，需要用两条 trace 来实现不同线型，
        # 但通过 None 断开确保视觉上连续
        inspect_x, inspect_y, inspect_text = [], [], []
        connect_x, connect_y, connect_text = [], [], []

        for seg in segments:
            geo = seg.get('geometry_2d', [])
            if not geo:
                continue

            seg_type = seg.get('type')
            seg_id = seg.get('segment_id')
            edge_id = seg.get('edge_id', 'N/A')
            length = seg.get('length', 0)

            # 对 inspect 段应用轻度 RDP 简化去除毛刺
            # connect 段不处理，保留原始密集点
            if seg_type == 'inspect':
                # 使用小容差（2.0px）只去除微小抖动，保留主拐点
                simplified_geo = rdp_simplify(geo, epsilon=2.0)
                # 确保至少保留起终点
                if len(simplified_geo) < 2:
                    simplified_geo = [geo[0], geo[-1]] if len(geo) >= 2 else geo
                # 如果简化后点数太少，稍微放松容差重试
                if len(simplified_geo) < 5 and len(geo) > 10:
                    simplified_geo = rdp_simplify(geo, epsilon=1.0)
                geo = simplified_geo

            x_coords = [p[0] for p in geo]
            y_coords = [p[1] for p in geo]

            hover = [f"{seg_type.upper()} {seg_id}<br>Edge: {edge_id}<br>长度: {length:.1f}px<br>坐标: ({x:.1f}, {y:.1f})"
                    for x, y in zip(x_coords, y_coords)]

            if seg_type == 'inspect':
                inspect_x.extend(x_coords + [None])
                inspect_y.extend(y_coords + [None])
                inspect_text.extend(hover + [None])
            else:  # connect
                connect_x.extend(x_coords + [None])
                connect_y.extend(y_coords + [None])
                connect_text.extend(hover + [None])

        # 移除最后的 None
        for lst in [inspect_x, inspect_y, inspect_text, connect_x, connect_y, connect_text]:
            if lst and lst[-1] is None:
                lst.pop()

        # 绘制 inspect 段（蓝色实线）
        if inspect_x:
            fig.add_trace(go.Scatter(
                x=inspect_x,
                y=inspect_y,
                mode='lines',
                line=dict(
                    color='#1f77b4',  # 蓝色
                    width=4,
                    dash='solid'  # 实线
                ),
                name='沿电网巡检 (Inspect)',
                hovertext=inspect_text,
                hoverinfo='text',
                legendgroup='path'
            ))

        # 绘制 connect 段（orangedashed）
        if connect_x:
            fig.add_trace(go.Scatter(
                x=connect_x,
                y=connect_y,
                mode='lines',
                line=dict(
                    color='#ff6600',  # orange
                    width=3,
                    dash='dash'  # dashed
                ),
                name='电网间转场 (Connect)',
                hovertext=connect_text,
                hoverinfo='text',
                legendgroup='path'
            ))

    # =====================================================
    # 叠加 inspection points（可交互，支持 hover）
    # =====================================================
    if inspection_points:
        # 按类型分类
        endpoint_points = [p for p in inspection_points if p.get('point_type') == 'endpoint']
        turning_points = [p for p in inspection_points if p.get('point_type') == 'turning']
        sample_points = [p for p in inspection_points if p.get('point_type') == 'sample']

        # 终点（red菱形）
        if endpoint_points:
            end_x = [p['pixel_position'][0] for p in endpoint_points]
            end_y = [p['pixel_position'][1] for p in endpoint_points]

            # 创建 hover 信息
            end_hover = []
            for p in endpoint_points:
                hover = f"类型: {p.get('point_type', 'unknown')}<br>"
                hover += f"ID: {p.get('point_id', 'N/A')}<br>"
                hover += f"边ID: {p.get('edge_id', 'N/A')}<br>"
                hover += f"访问顺序: {p.get('visit_order', 'N/A')}<br>"
                hover += f"坐标: ({p['pixel_position'][0]:.1f}, {p['pixel_position'][1]:.1f})"
                end_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=end_x,
                y=end_y,
                mode='markers',
                marker=dict(
                    color='red',
                    size=14,
                    symbol='diamond',
                    line=dict(color='white', width=2)
                ),
                name=f'巡检终点 ({len(endpoint_points)}个)',
                hovertext=end_hover,
                hoverinfo='text',
                visible=True  # 默认显示
            ))

        # 转向点（orange圆形）- 默认隐藏
        if turning_points:
            turn_x = [p['pixel_position'][0] for p in turning_points]
            turn_y = [p['pixel_position'][1] for p in turning_points]

            # 创建 hover 信息
            turn_hover = []
            for p in turning_points:
                hover = f"类型: {p.get('point_type', 'unknown')}<br>"
                hover += f"ID: {p.get('point_id', 'N/A')}<br>"
                hover += f"边ID: {p.get('edge_id', 'N/A')}<br>"
                hover += f"访问顺序: {p.get('visit_order', 'N/A')}<br>"
                hover += f"坐标: ({p['pixel_position'][0]:.1f}, {p['pixel_position'][1]:.1f})"
                turn_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=turn_x,
                y=turn_y,
                mode='markers',
                marker=dict(
                    color='orange',
                    size=10,
                    symbol='circle',
                    line=dict(color='white', width=1.5)
                ),
                name=f'转向点 ({len(turning_points)}个) - 点击图例显示',
                hovertext=turn_hover,
                hoverinfo='text',
                visible=False  # 默认隐藏
            ))

        # 采样点（青色小点，默认隐藏，可通过图例显示）
        if sample_points:
            sample_x = [p['pixel_position'][0] for p in sample_points]
            sample_y = [p['pixel_position'][1] for p in sample_points]

            # 创建 hover 信息
            sample_hover = []
            for p in sample_points:
                hover = f"类型: {p.get('point_type', 'unknown')}<br>"
                hover += f"ID: {p.get('point_id', 'N/A')}<br>"
                hover += f"边ID: {p.get('edge_id', 'N/A')}<br>"
                hover += f"访问顺序: {p.get('visit_order', 'N/A')}<br>"
                hover += f"坐标: ({p['pixel_position'][0]:.1f}, {p['pixel_position'][1]:.1f})"
                sample_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=sample_x,
                y=sample_y,
                mode='markers',
                marker=dict(
                    color='cyan',
                    size=5,
                    symbol='circle',
                    opacity=0.6
                ),
                name=f'原始采样点 ({len(sample_points)}个) - 点击图例显示',
                hovertext=sample_hover,
                hoverinfo='text',
                visible=False  # 默认隐藏
            ))

    # =====================================================
    # 显示 Inspection Sampling Points（带完整元数据的巡检点）
    # 这些是用于后续"点击交互 + 照片绑定"的锚点
    # =====================================================
    if inspection_points_with_metadata:
        sample_x = [p['position'][0] for p in inspection_points_with_metadata]
        sample_y = [p['position'][1] for p in inspection_points_with_metadata]

        # 创建详细的 hover 信息（包含所有元数据）
        sample_hover = []
        for p in inspection_points_with_metadata:
            hover = (f"<b>巡检点: {p['point_id']}</b><br>"
                     f"类型: 巡检采样点<br>"
                     f"边ID: {p['edge_id']}<br>"
                     f"段ID: {p['segment_id']}<br>"
                     f"分组ID: {p['group_id']}<br>"
                     f"路径序号: {p['sequence_in_path']}<br>"
                     f"坐标: ({p['position'][0]:.1f}, {p['position'][1]:.1f})<br>"
                     f"<i>点击可查看巡检照片（待绑定）</i>")
            sample_hover.append(hover)

        # 创建 customdata（包含完整元数据，供后续 click 事件使用）
        custom_data = [
            {
                'point_id': p['point_id'],
                'segment_id': p['segment_id'],
                'edge_id': p['edge_id'],
                'group_id': p['group_id'],
                'sequence': p['sequence_in_path'],
                'position': p['position'],  # ← 新增：坐标
                'type': p.get('type', 'sample'),  # ← 新增：类型
                'status': p.get('status', 'uninspected'),  # ← 新增：状态
                'result_image': p.get('result_image', None),  # ← 新增：结果图片
                'result_text': p.get('result_text', ''),  # ← 新增：结果文本
                'photo_path': p.get('photo_path', '')
            }
            for p in inspection_points_with_metadata
        ]

        fig.add_trace(go.Scatter(
            x=sample_x,
            y=sample_y,
            mode='markers',
            marker=dict(
                color='#ff7f0e',  # orange
                size=12,
                symbol='circle',
                line=dict(color='white', width=2),
                opacity=1.0
            ),
            name=f'巡检点 ({len(sample_x)}个) - {inspection_spacing_px}px间距',
            hovertext=sample_hover,
            hoverinfo='text',
            customdata=custom_data,
            visible=True,  # 默认显示
            # 启用点击事件（用于查看图片）
            meta=[str(i) for i in range(len(sample_x))]  # 点索引
        ))

    # =====================================================
    # 标记动态起点、接入点和终点
    # =====================================================
    if path_2d:
        # 原始用户输入点（黄色星形 - 显示用户真实输入位置）
        # 初始隐藏，用户输入后显示
        fig.add_trace(go.Scatter(
            x=[path_2d[0][0]],  # 初始占位
            y=[path_2d[0][1]],
            mode='markers+text',
            marker=dict(
                color='#FFD700',  # 金黄色
                size=22,
                symbol='star',
                line=dict(color='orange', width=2)
            ),
            name='用户输入点',
            text=['输入'],
            textposition='middle center',
            textfont=dict(size=9, color='white', family='Arial Black'),
            hovertext=['用户输入点<br>（原始坐标）'],
            hoverinfo='text',
            visible=False  # 初始隐藏
        ))

        # 系统接入点（蓝色圆圈 - 系统选择的最近路径点）
        start_x, start_y = path_2d[0][0], path_2d[0][1]
        fig.add_trace(go.Scatter(
            x=[start_x],
            y=[start_y],
            mode='markers+text',
            marker=dict(
                color='#0066CC',
                size=18,
                symbol='circle',
                line=dict(color='white', width=2)
            ),
            name='接入点',
            text=['接入'],
            textposition='middle center',
            textfont=dict(size=9, color='white', family='Arial Black'),
            hovertext=[f"系统接入点<br>坐标: ({start_x:.1f}, {start_y:.1f})"],
            hoverinfo='text'
        ))

        # 起点（绿色方块）- 偏移避免被用户输入点遮挡
        start_x, start_y = path_2d[0][0], path_2d[0][1]
        fig.add_trace(go.Scatter(
            x=[start_x + 15],  # 向右偏移 15 像素
            y=[start_y + 15],  # 向上偏移 15 像素
            mode='markers+text',
            marker=dict(
                color='#28a745',
                size=16,
                symbol='square',
                line=dict(color='white', width=2)
            ),
            name='起点',
            text=['起'],
            textposition='middle center',
            textfont=dict(size=10, color='white', family='Arial Black'),
            hovertext=[f"起点<br>坐标: ({start_x:.1f}, {start_y:.1f})"],
            hoverinfo='text'
        ))

        # 终点（red方块）
        end_x, end_y = path_2d[-1][0], path_2d[-1][1]
        fig.add_trace(go.Scatter(
            x=[end_x],
            y=[end_y],
            mode='markers+text',
            marker=dict(
                color='red',
                size=16,
                symbol='square',
                line=dict(color='white', width=2)
            ),
            name='终点',
            text=['终'],
            textposition='middle center',
            textfont=dict(size=10, color='white', family='Arial Black'),
            hovertext=[f"终点<br>坐标: ({end_x:.1f}, {end_y:.1f})"],
            hoverinfo='text'
        ))

    # =====================================================
    # 设置布局
    # =====================================================
    # 设置坐标轴范围（匹配地图尺寸）
    # 注意：y轴使用反转，因为图像坐标系 y轴向下
    fig.update_xaxes(
        range=[0, map_width],
        showgrid=True,
        gridwidth=1,
        gridcolor='rgba(128, 128, 128, 0.3)',
        showline=True,
        linewidth=2,
        linecolor='black'
    )

    fig.update_yaxes(
        range=[map_height, 0],  # 反转 y 轴，匹配图像坐标系
        showgrid=True,
        gridwidth=1,
        gridcolor='rgba(128, 128, 128, 0.3)',
        showline=True,
        linewidth=2,
        linecolor='black'
    )

    # 更新布局
    fig.update_layout(
        title=dict(
            text='UAV 电网巡检 - 交互式主展示页',
            x=0.5,
            xanchor='center',
            font=dict(size=20)
        ),
        xaxis_title='X (像素)',
        yaxis_title='Y (像素)',
        hovermode='closest',
        showlegend=True,
        legend=dict(
            x=1.02,
            y=1,
            xanchor='left',
            yanchor='top',
            bgcolor='rgba(255, 255, 255, 0.8)',
            bordercolor='gray',
            borderwidth=1
        ),
        # 设置宽度和高度
        width=1400,
        height=900,
        # 系统界面风格的背景色
        plot_bgcolor='white',
        paper_bgcolor='#f0f0f0',
        # 添加边距
        margin=dict(l=80, r=200, t=100, b=80)
    )

    # =====================================================
    # 添加统计信息注释
    # =====================================================
    if statistics:
        stats_text = (
            f"<b>任务统计</b><br>"
            f"-------------------<br>"
            f"总长度: {statistics.get('total_length', 0):.1f} px<br>"
            f"巡检长度: {statistics.get('inspect_length', 0):.1f} px<br>"
            f"连接长度: {statistics.get('connect_length', 0):.1f} px<br>"
            f"巡检点: {statistics.get('num_inspection_points', 0)} 个<br>"
            f"分组数: {statistics.get('num_groups', 0)} 个"
        )

        fig.add_annotation(
            text=stats_text,
            xref='paper',
            yref='paper',
            x=1.02,
            y=0.5,
            xanchor='left',
            yanchor='middle',
            showarrow=False,
            font=dict(size=12, color='black'),
            bgcolor='rgba(255, 255, 255, 0.9)',
            bordercolor='blue',
            borderwidth=2,
            borderpad=10
        )

    # =====================================================
    # 添加 UAV 标记（用于模拟飞行）
    # =====================================================
    # 初始位置设在起点，初始隐藏
    if path_2d:
        uav_start = path_2d[0]
        fig.add_trace(go.Scatter(
            x=[uav_start[0]],
            y=[uav_start[1]],
            mode='markers',
            marker=dict(
                color='red',
                size=20,
                symbol='triangle-up',
                line=dict(color='white', width=2),
                opacity=1.0
            ),
            name='UAV 飞行器',
            hoverinfo='text',
            hovertext=['UAV 飞行器<br>状态: 待命'],
            showlegend=False
        ))


    # =====================================================
    # 保存为 HTML
    # =====================================================
    print(f"\n[INFO] 保存交互式主展示页...")

    # 使用 config 启用一些交互选项
    config = {
        'displayModeBar': True,
        'displaylogo': False,
        'modeBarButtonsToRemove': ['lasso2d', 'select2d']
    }

    fig.write_html(
        output_html_path,
        config=config,
        include_plotlyjs='cdn'
    )

    # =====================================================
    # 添加天气信息卡片和模拟飞行控制面板
    # =====================================================
    print(f"[INFO] 添加天气信息卡片...")

    # 读取生成的 HTML
    with open(output_html_path, 'r', encoding='utf-8') as f:
        html_content = f.read()

    # 生成天气信息卡片 HTML（完全静态，不依赖任何变量）
    weather_card_html = generate_weather_card_html()

    # 生成飞行控制面板 HTML
    flight_control_html = generate_flight_control_code(path_2d, segments, inspection_points_with_metadata, weather, statistics)

    # 在 </body> 标签之前插入天气卡片和飞行控制面板
    html_content = html_content.replace('</body>', weather_card_html + flight_control_html + '</body>')

    # 写回文件
    with open(output_html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"[INFO] 交互式主展示页已保存: {output_html_path}")

    return output_html_path



def generate_weather_card_html():
    """
    生成天气信息卡片的静态 HTML（最终稳定版：完全写死，不依赖任何变量）

    Returns:
        str: HTML 代码（纯静态，包含内联 CSS）
    """
    return """
    <div id="weather-card-static" style="
        position: fixed;
        right: 20px;
        bottom: 20px;
        width: 175px;
        background: rgba(255, 255, 255, 0.96);
        border: 1px solid #ddd;
        border-radius: 8px;
        box-shadow: 0 3px 8px rgba(0,0,0,0.1);
        padding: 10px 12px;
        z-index: 900;
        font-family: Arial, sans-serif;
        font-size: 12px;
        color: #333;
        pointer-events: auto;
    ">
        <div style="
            display: flex;
            align-items: center;
            margin-bottom: 8px;
            padding-bottom: 6px;
            border-bottom: 1px solid #e0e0e0;
        ">
            <span style="font-size: 14px; margin-right: 4px;">🌤️</span>
            <span style="font-weight: bold; font-size: 13px;">天气信息</span>
        </div>
        <div style="display: flex; flex-direction: column; gap: 4px;">
            <div style="display: flex; justify-content: space-between;">
                <span style="color: #666;">天气</span>
                <span style="font-weight: bold;">强逆风</span>
            </div>
            <div style="display: flex; justify-content: space-between;">
                <span style="color: #666;">场景</span>
                <span style="font-weight: bold;">headwind_strong</span>
            </div>
            <div style="display: flex; justify-content: space-between;">
                <span style="color: #666;">风速</span>
                <span style="font-weight: bold;">10.0 m/s</span>
            </div>
            <div style="display: flex; justify-content: space-between;">
                <span style="color: #666;">风向</span>
                <span style="font-weight: bold;">180°</span>
            </div>
            <div style="display: flex; justify-content: space-between;">
                <span style="color: #666;">风险</span>
                <span style="
                    font-weight: bold;
                    color: #dc3545;
                    background: rgba(0,0,0,0.05);
                    padding: 1px 6px;
                    border-radius: 3px;
                    font-size: 11px;
                ">高</span>
            </div>
        </div>
    </div>
    """


def generate_flight_control_code(path_2d, segments, inspection_points_with_metadata, weather, statistics):
    """生成飞行控制面板的 HTML 和 JavaScript 代码 - 修复版"""
    import json

    image_width = 916
    image_height = 960

    if path_2d:
        # 过滤掉 None 值
        valid_path_2d = [p for p in path_2d if p is not None]
        if valid_path_2d:
            x_coords = [p[0] for p in valid_path_2d]
            y_coords = [p[1] for p in valid_path_2d]
            x_min, x_max = int(min(x_coords)), int(max(x_coords))
            y_min, y_max = int(min(y_coords)), int(max(y_coords))
        else:
            x_min, x_max = 0, image_width
            y_min, y_max = 0, image_height
    else:
        x_min, x_max = 0, image_width
        y_min, y_max = 0, image_height

    # 构建 segment 索引映射（用于分段速度控制）
    segment_map_json = "[]"
    if segments:
        seg_map = []
        for seg in segments:
            geo = seg.get('geometry_2d', [])
            seg_len = len(geo)
            for _ in range(seg_len):
                seg_map.append({
                    'type': seg.get('type', 'unknown'),
                    'segment_id': seg.get('segment_id', ''),
                    'edge_id': seg.get('edge_id', ''),
                    'group_id': seg.get('group_id', '')
                })
        segment_map_json = json.dumps(seg_map)

    # 嵌入当前 mission 数据供其他功能使用
    # 天气卡片现在是静态 HTML，不在这里传递天气数据

    # 预先序列化数据，避免 f-string 中的编码问题
    flight_path_json = json.dumps(path_2d, ensure_ascii=True)
    segments_data_json = json.dumps(segments, ensure_ascii=True)
    # 使用 default=str 来处理可能包含特殊对象的 inspection_points_with_metadata
    inspection_points_data_json = json.dumps(inspection_points_with_metadata, ensure_ascii=True, default=str)

    return f"""
    <script>
    const flightPath = {flight_path_json};
    const segmentsData = {segments_data_json};
    const inspectionPointsData = {inspection_points_data_json};
    const segmentIndexMap = {segment_map_json};

    // Current mission data (weather card is now static HTML, not passed here)
    const currentMissionData = {{}};

    // Filter out None values, create continuous playback path
    let continuousFlightPath = flightPath.filter(pt => pt !== null && pt !== undefined);

    // Mission start point (user input point or path start point)
    const missionStartPoint = continuousFlightPath[0] || [500, 500];

    // Add path from user input start point to first inspection point
    function interpolatePath(p1, p2, steps) {{
        const points = [];
        for (let i = 0; i <= steps; i++) {{
            const t = i / steps;
            points.push([
                p1[0] + (p2[0] - p1[0]) * t,
                p1[1] + (p2[1] - p1[1]) * t
            ]);
        }}
        return points;
    }}

    // If there are inspection points, prepend path from start point to first inspection point
    if (inspectionPointsData.length > 0) {{
        const firstInspectionPoint = inspectionPointsData[0].position;
        if (firstInspectionPoint && firstInspectionPoint.length >= 2) {{
            const startToFirstPath = interpolatePath(missionStartPoint, firstInspectionPoint, 30);
            continuousFlightPath = [...startToFirstPath, ...continuousFlightPath];
        }}
    }}

    const IMAGE_INFO = {{
        width: {image_width}, height: {image_height},
        x_min: {x_min}, x_max: {x_max},
        y_min: {y_min}, y_max: {y_max}
    }};

    let plotlyDiv = null;
    let uavTraceIndex = -1;
    let startPointTraceIndex = -1;
    let connectTraceIndex = -1;  // mission connect segment trace index

    let flightState = {{
        isFlying: false, isPaused: false, currentIndex: 0, timer: null,
        inspectSpeed: 35, connectSpeed: 50
    }};

    let panelExpanded = false;

    // Inspection playback state section
    const INSPECTION_DWELL_TIME = 2500;  // Dwell time at inspection points (ms)
    const INSPECTION_FRAME_INTERVAL = 30;  // Playback animation frame interval in milliseconds
    let inspectionPlayState = {{
        isPlaying: false, isPaused: false, currentPathIndex: 0, timer: null,
        inspectionTraceIndex: -1,  // Inspection point trace index
        playbackMarkerIndex: -1,   // Playback indicator trace index
        playbackLineIndex: -1,     // Inspection playback auxiliary line trace index
        isMoving: false,           // Whether moving
        inspectionPointPathIndices: []  // Path index mapping for each inspection point
    }};
    let isInspectionMode = false;  // Mode: false for flight mode, true for inspection playback mode

    function toggleControlPanel() {{
        const panel = document.getElementById('flightControlPanel');
        const btn = document.getElementById('panelToggleBtn');
        const content = document.getElementById('panelContent');
        if (panelExpanded) {{
            panel.style.width = '56px'; content.style.display = 'none';
            btn.innerHTML = '◀'; btn.title = '展开';
        }} else {{
            panel.style.width = '320px'; content.style.display = 'block';
            btn.innerHTML = '▶'; btn.title = '收起';
        }}
        panelExpanded = !panelExpanded;
    }}

    function resetToCleanInitialState() {{
        // Reset all runtime states to clean initial state
        console.log('Resetting to clean initial state...');

        // Reset input fields to empty
        const inputX = document.getElementById('inputX');
        const inputY = document.getElementById('inputY');
        if (inputX) inputX.value = '';
        if (inputY) inputY.value = '';

        // Reset playback states
        flightState.isFlying = false;
        flightState.isPaused = false;
        flightState.currentIndex = 0;

        inspectionPlayState.isPlaying = false;
        inspectionPlayState.isPaused = false;
        inspectionPlayState.currentPathIndex = 0;
        inspectionPlayState.isMoving = false;

        // Reset status displays
        const statusElements = ['flightStatus', 'currentStatus', 'currentCoord', 'currentType'];
        statusElements.forEach(id => {{
            const el = document.getElementById(id);
            if (el) {{
                if (id === 'flightStatus' || id === 'currentStatus') {{
                    el.textContent = '未开始';
                }} else if (id === 'currentCoord') {{
                    el.textContent = '-';
                }} else if (id === 'currentType') {{
                    el.textContent = '-';
                }}
            }}
        }});

        // Clear any runtime annotations
        if (plotlyDiv && plotlyDiv.layout) {{
            const currentAnnotations = plotlyDiv.layout.annotations || [];
            if (currentAnnotations.length > 0) {{
                Plotly.relayout(plotlyDiv, {{ annotations: [] }});
            }}
        }}

        // Reset UAV marker position to initial state (if exists)
        if (uavTraceIndex !== -1 && plotlyDiv) {{
            try {{
                const uavData = plotlyDiv.data[uavTraceIndex];
                if (uavData && uavData.x && uavData.x.length > 0) {{
                    const initialX = uavData.x[0];
                    const initialY = uavData.y[0];
                    Plotly.restyle(plotlyDiv, {{
                        x: [[initialX]],
                        y: [[initialY]]
                    }}, [uavTraceIndex]);
                }}
            }} catch (e) {{
                console.warn('Failed to reset UAV marker:', e);
            }}
        }}

        // Reset inspection point highlights
        if (inspectionPlayState.inspectionTraceIndex !== -1 && plotlyDiv) {{
            try {{
                const n = plotlyDiv.data[inspectionPlayState.inspectionTraceIndex].marker.size.length;
                Plotly.restyle(plotlyDiv, {{
                    'marker.size': [new Array(n).fill(12)],
                    'marker.color': [new Array(n).fill('#ff7f0e')]
                }}, [inspectionPlayState.inspectionTraceIndex]);
            }} catch (e) {{
                console.warn('Failed to reset inspection point highlights:', e);
            }}
        }}

        // Reset playback marker (if exists)
        if (inspectionPlayState.playbackMarkerIndex !== -1 && plotlyDiv) {{
            try {{
                Plotly.restyle(plotlyDiv, {{
                    x: [[missionStartPoint[0]]],
                    y: [[missionStartPoint[1]]]
                }}, [inspectionPlayState.playbackMarkerIndex]);
            }} catch (e) {{
                console.warn('Failed to reset playback marker:', e);
            }}
        }}

        console.log('Clean initial state restored');
    }}

    function initializeFlightSystem() {{
        const divs = document.getElementsByClassName('plotly-graph-div');
        if (divs.length > 0) {{
            plotlyDiv = divs[0];
            for (let i = 0; i < plotlyDiv.data.length; i++) {{
                if (plotlyDiv.data[i].name === 'UAV 飞行器') uavTraceIndex = i;
                else if (plotlyDiv.data[i].name === '当前起点') startPointTraceIndex = i;
                else if (plotlyDiv.data[i].name && plotlyDiv.data[i].name.includes('电网间转场')) {{
                    connectTraceIndex = i;
                }}
                else if (plotlyDiv.data[i].name && plotlyDiv.data[i].name.includes('巡检点')) {{
                    inspectionPlayState.inspectionTraceIndex = i;
                }}
            }}
            // Add start point marker (with offset to avoid overlap)
            addStartPointMarker();
            // Add playback indicator marker (small plane)
            addInspectionPlaybackMarker();
            // Add inspection playback auxiliary line (with interpolation)
            addInspectionPlaybackLine();
            // Build path index mapping for each inspection point
            buildInspectionPointPathIndexMapping();
            console.log('UAV trace:', uavTraceIndex, 'Start point trace:', startPointTraceIndex, 'Connect trace:', connectTraceIndex, 'Inspection trace:', inspectionPlayState.inspectionTraceIndex, 'Playback marker:', inspectionPlayState.playbackMarkerIndex);
        }}
    }}

    function addStartPointMarker() {{
        // Add start point marker trace (with offset to avoid overlapping with user input point)
        // This function can be called both during initial load and after replan
        if (!plotlyDiv || !missionStartPoint) return;

        const startMarkerData = {{
            x: [missionStartPoint[0] + 15],  // Offset right by 15px
            y: [missionStartPoint[1] + 15],  // Offset up by 15px
            mode: 'markers+text',
            marker: {{
                color: '#28a745',  // Green
                size: 16,
                symbol: 'square',
                line: {{ color: 'white', width: 2 }}
            }},
            name: '起点',
            text: ['起'],
            textposition: 'middle center',
            textfont: {{ size: 10, color: 'white', family: 'Arial Black' }},
            hovertext: [`起点<br>坐标: (${{missionStartPoint[0].toFixed(1)}}, ${{missionStartPoint[1].toFixed(1)}})`],
            hoverinfo: 'text',
            showlegend: false
        }};

        Plotly.addTraces(plotlyDiv, [startMarkerData]);
    }}

    function buildInspectionPointPathIndexMapping() {{
        // find nearest index in real path for each inspection point
        inspectionPlayState.inspectionPointPathIndices = [];

        for (const point of inspectionPointsData) {{
            if (!point.position || point.position.length < 2) {{
                inspectionPlayState.inspectionPointPathIndices.push(-1);
                continue;
            }}

            const px = point.position[0];
            const py = point.position[1];

            // find nearest point in continuousFlightPath
            let minDist = Infinity;
            let nearestIdx = -1;

            for (let i = 0; i < continuousFlightPath.length; i++) {{
                const pathPt = continuousFlightPath[i];
                const dist = Math.sqrt((pathPt[0] - px) ** 2 + (pathPt[1] - py) ** 2);
                if (dist < minDist) {{
                    minDist = dist;
                    nearestIdx = i;
                }}
            }}

            // map continuousFlightPath index back to original flightPath index
            // find point with same coordinates in flightPath
            if (nearestIdx >= 0) {{
                const targetPt = continuousFlightPath[nearestIdx];
                let originalIdx = -1;
                for (let i = 0; i < flightPath.length; i++) {{
                    const pt = flightPath[i];
                    if (pt && pt[0] === targetPt[0] && pt[1] === targetPt[1]) {{
                        originalIdx = i;
                        break;
                    }}
                }}
                inspectionPlayState.inspectionPointPathIndices.push(originalIdx);
            }} else {{
                inspectionPlayState.inspectionPointPathIndices.push(-1);
            }}
        }}

        console.log('巡检点路径索引映射:', inspectionPlayState.inspectionPointPathIndices);
    }}

    function addInspectionPlaybackMarker() {{
        if (!plotlyDiv || inspectionPointsData.length === 0) return;

        // create playback indicator trace
        const markerData = {{
            x: [inspectionPointsData[0].position[0]],
            y: [inspectionPointsData[0].position[1]],
            mode: 'markers',
            marker: {{
                color: '#ff0000',  // red
                size: 18,
                symbol: 'triangle-up',  // triangle (small plane)
                line: {{ color: 'white', width: 2 }}
            }},
            name: '巡检播放指示',
            hoverinfo: 'none',
            showlegend: false
        }};

        Plotly.addTraces(plotlyDiv, [markerData]);
        inspectionPlayState.playbackMarkerIndex = plotlyDiv.data.length - 1;
    }}

    function addInspectionPlaybackLine() {{
        // Create playback auxiliary line with interpolation from start point to first inspection point
        if (!plotlyDiv || inspectionPointsData.length === 0) return;

        // Build interpolated path from start point to first inspection point
        function interpolatePath(p1, p2, steps) {{
            const points = [];
            for (let i = 0; i <= steps; i++) {{
                const t = i / steps;
                points.push([
                    p1[0] + (p2[0] - p1[0]) * t,
                    p1[1] + (p2[1] - p1[1]) * t
                ]);
            }}
            return points;
        }}

        const lineX = [];
        const lineY = [];

        // Add interpolated path from mission start point to first inspection point
        if (inspectionPointsData.length > 0 && inspectionPointsData[0].position) {{
            const firstInspectionPoint = inspectionPointsData[0].position;
            const interpolatedPath = interpolatePath(missionStartPoint, firstInspectionPoint, 30);

            for (const pt of interpolatedPath) {{
                lineX.push(pt[0]);
                lineY.push(pt[1]);
            }}
        }} else {{
            // If no inspection points, just use start point
            lineX.push(missionStartPoint[0]);
            lineY.push(missionStartPoint[1]);
        }}

        // Add all inspection points
        for (const p of inspectionPointsData) {{
            lineX.push(p.position[0]);
            lineY.push(p.position[1]);
        }}

        // Create playback auxiliary line trace
        const lineData = {{
            x: lineX,
            y: lineY,
            mode: 'lines',
            line: {{
                color: '#ff6600',  // orange
                width: 2,
                dash: 'dash'  // dashed
            }},
            name: '巡检播放辅助线',
            hoverinfo: 'none',
            showlegend: false,
            visible: false
        }};

        Plotly.addTraces(plotlyDiv, [lineData]);
        inspectionPlayState.playbackLineIndex = plotlyDiv.data.length - 1;
        console.log('巡检播放辅助线已创建（包含起点到首个巡检点的插值路径），索引:', inspectionPlayState.playbackLineIndex);
    }}

    function getSegmentTypeAtIndex(idx) {{
        if (idx >= 0 && idx < segmentIndexMap.length) {{
            return segmentIndexMap[idx].type;
        }}
        return 'unknown';
    }}

    function getCurrentSpeed(idx) {{
        return getSegmentTypeAtIndex(idx) === 'connect' ? flightState.connectSpeed : flightState.inspectSpeed;
    }}

    function updateDynamicStartPoint(x, y) {{
        if (plotlyDiv && startPointTraceIndex !== -1) {{
            Plotly.restyle(plotlyDiv, {{ x: [[x]], y: [[y]] }}, [startPointTraceIndex]);
        }}
    }}

    function findNearestPathIndex(x, y) {{
        let minDist = Infinity, nearestIdx = 0;
        for (let i = 0; i < flightPath.length; i++) {{
            const dx = flightPath[i][0] - x, dy = flightPath[i][1] - y;
            const dist = dx*dx + dy*dy;
            if (dist < minDist) {{ minDist = dist; nearestIdx = i; }}
        }}
        return nearestIdx;
    }}

    function validateInput(x, y) {{
        const errs = [];
        if (isNaN(x) || x === '') errs.push('X坐标不能为空');
        else if (x < IMAGE_INFO.x_min || x > IMAGE_INFO.x_max) errs.push('X超出范围');
        if (isNaN(y) || y === '') errs.push('Y坐标不能为空');
        else if (y < IMAGE_INFO.y_min || y > IMAGE_INFO.y_max) errs.push('Y超出范围');
        return errs;
    }}

    function startFlight() {{
        if (!plotlyDiv || uavTraceIndex === -1) {{ alert('系统未初始化'); return; }}
        const x = parseFloat(document.getElementById('inputX').value);
        const y = parseFloat(document.getElementById('inputY').value);
        const errs = validateInput(x, y);
        if (errs.length > 0) {{ alert('输入错误:\\n' + errs.join('\\n')); return; }}

        updateDynamicStartPoint(x, y);
        if (flightState.timer) {{ clearInterval(flightState.timer); flightState.timer = null; }}

        const nearestIdx = findNearestPathIndex(x, y);
        flightState.currentIndex = nearestIdx;
        flightState.isFlying = true;
        flightState.isPaused = false;

        updateFlightDisplay();
        updateStatus('飞行中');
        flightState.timer = setInterval(flightStep, getCurrentSpeed(nearestIdx));
    }}

    function flightStep() {{
        if (!flightState.isFlying || flightState.isPaused) return;
        flightState.currentIndex++;

        if (flightState.currentIndex >= flightPath.length) {{
            stopFlight(); updateStatus('飞行完成');
            alert('✅ 飞行完成！共 ' + flightState.currentIndex + ' 点');
            return;
        }}
        updateFlightDisplay();

        const newSpeed = getCurrentSpeed(flightState.currentIndex);
        clearInterval(flightState.timer);
        flightState.timer = setInterval(flightStep, newSpeed);
    }}

    function updateFlightDisplay() {{
        if (!plotlyDiv || uavTraceIndex === -1) return;
        const idx = flightState.currentIndex, pt = flightPath[idx];

        Plotly.restyle(plotlyDiv, {{ x: [[pt[0]]], y: [[pt[1]]] }}, [uavTraceIndex]);

        document.getElementById('currentStatus').textContent = `${{idx}} / ${{flightPath.length}}`;
        document.getElementById('currentCoord').textContent = `(${{pt[0].toFixed(1)}}, ${{pt[1].toFixed(1)}})`;
        document.getElementById('currentType').textContent = getSegmentTypeAtIndex(idx) === 'connect' ? '连接' : '巡检';
    }}

    function pauseFlight() {{
        if (flightState.isFlying && !flightState.isPaused) {{
            flightState.isPaused = true; updateStatus('已暂停');
        }}
    }}

    function resumeFlight() {{
        if (flightState.isFlying && flightState.isPaused) {{
            flightState.isPaused = false; updateStatus('飞行中');
            const speed = getCurrentSpeed(flightState.currentIndex);
            clearInterval(flightState.timer);
            flightState.timer = setInterval(flightStep, speed);
        }}
    }}

    function stopFlight() {{
        if (flightState.timer) {{ clearInterval(flightState.timer); flightState.timer = null; }}
        flightState.isFlying = false; flightState.isPaused = false; flightState.currentIndex = 0;

        if (plotlyDiv && uavTraceIndex !== -1 && flightPath.length > 0) {{
            Plotly.restyle(plotlyDiv, {{ x: [[flightPath[0][0]]], y: [[flightPath[0][1]]] }}, [uavTraceIndex]);
        }}
        updateStatus('未开始');
        document.getElementById('currentStatus').textContent = '-';
        document.getElementById('currentCoord').textContent = '-';
        document.getElementById('currentType').textContent = '-';
    }}

    function startInspectionPlayback() {{
        if (inspectionPointsData.length === 0) {{
            alert('没有巡检点数据');
            return;
        }}

        if (inspectionPlayState.timer) {{
            clearTimeout(inspectionPlayState.timer);
            inspectionPlayState.timer = null;
        }}

        for (let i = 0; i < inspectionPointsData.length; i++) {{
            delete inspectionPlayState[`triggered_${{i}}`];
        }}

        inspectionPlayState.currentPathIndex = 0;
        inspectionPlayState.isPlaying = true;
        inspectionPlayState.isPaused = false;
        inspectionPlayState.isMoving = false;

        if (inspectionPlayState.playbackMarkerIndex !== -1 && continuousFlightPath.length > 0) {{
            const firstPathPoint = continuousFlightPath[0];
            Plotly.restyle(plotlyDiv, {{
                x: [[firstPathPoint[0]]],
                y: [[firstPathPoint[1]]]
            }}, [inspectionPlayState.playbackMarkerIndex]);
        }}

        startPathFollowing();

        updateStatus('巡检播放中');
        document.getElementById('currentType').textContent = '自动播放';
    }}

    function startPathFollowing() {{
        if (!inspectionPlayState.isPlaying || inspectionPlayState.isPaused) return;

        inspectionPlayState.isMoving = true;

        inspectionPlayState.timer = setInterval(function() {{
            if (!inspectionPlayState.isPlaying || inspectionPlayState.isPaused) return;

            inspectionPlayState.currentPathIndex++;

            while (inspectionPlayState.currentPathIndex < flightPath.length &&
                   (flightPath[inspectionPlayState.currentPathIndex] === null ||
                    flightPath[inspectionPlayState.currentPathIndex] === undefined)) {{
                inspectionPlayState.currentPathIndex++;
            }}

            if (inspectionPlayState.currentPathIndex >= flightPath.length) {{
                stopInspectionPlayback();
                alert('✅ 巡检播放完成！');
                return;
            }}

            const currentPathPoint = flightPath[inspectionPlayState.currentPathIndex];

            if (inspectionPlayState.playbackMarkerIndex !== -1) {{
                Plotly.restyle(plotlyDiv, {{
                    x: [[currentPathPoint[0]]],
                    y: [[currentPathPoint[1]]]
                }}, [inspectionPlayState.playbackMarkerIndex]);
            }}

            document.getElementById('currentCoord').textContent =
                `(${{currentPathPoint[0].toFixed(1)}}, ${{currentPathPoint[1].toFixed(1)}})`;

            checkAndTriggerInspectionPoint();

        }}, INSPECTION_FRAME_INTERVAL);
    }}

    function checkAndTriggerInspectionPoint() {{
        const currentPathIdx = inspectionPlayState.currentPathIndex;

        for (let i = 0; i < inspectionPlayState.inspectionPointPathIndices.length; i++) {{
            const inspectionPathIdx = inspectionPlayState.inspectionPointPathIndices[i];

            if (Math.abs(currentPathIdx - inspectionPathIdx) <= 2) {{
                if (!inspectionPlayState[`triggered_${{i}}`]) {{
                    triggerInspectionPoint(i);
                }}
                break;
            }}
        }}
    }}

    function triggerInspectionPoint(pointIdx) {{
        if (pointIdx < 0 || pointIdx >= inspectionPointsData.length) return;

        inspectionPlayState[`triggered_${{pointIdx}}`] = true;

        document.getElementById('currentStatus').textContent =
            `${{pointIdx + 1}} / ${{inspectionPointsData.length}}`;

        const wasPaused = inspectionPlayState.isPaused;
        inspectionPlayState.isPaused = true;

        arriveAtInspectionPoint(pointIdx);

        setTimeout(function() {{
            if (inspectionPlayState.isPlaying) {{
                inspectionPlayState.isPaused = wasPaused;
            }}
        }}, INSPECTION_DWELL_TIME);
    }}

    function arriveAtInspectionPoint(idx) {{
        const pointData = inspectionPointsData[idx];

        updateInspectionPointPanel(pointData);

        highlightInspectionPoint(idx);

        if (pointData.position) {{
            document.getElementById('currentCoord').textContent =
                `(${{pointData.position[0].toFixed(1)}}, ${{pointData.position[1].toFixed(1)}})`;

            // Add a temporary annotation at the inspection point location
            const annotationText = pointData.point_id || `IP_${{idx}}`;
            const newAnnotation = {{
                x: pointData.position[0],
                y: pointData.position[1],
                xref: 'x',
                yref: 'y',
                text: `<b>${{annotationText}}</b>`,
                showarrow: true,
                arrowhead: 2,
                arrowsize: 1,
                arrowwidth: 2,
                arrowcolor: '#ff7f0e',
                ax: 0,
                ay: -40,
                font: {{ size: 11, color: '#ff7f0e' }},
                bgcolor: 'rgba(255, 255, 255, 0.9)',
                bordercolor: '#ff7f0e',
                borderwidth: 1,
                borderpad: 4,
                opacity: 0.9
            }};

            const currentLayout = plotlyDiv.layout;
            const currentAnnotations = currentLayout.annotations || [];

            Plotly.relayout(plotlyDiv, {{
                annotations: [...currentAnnotations, newAnnotation]
            }});

            // Remove the annotation after dwell time
            setTimeout(() => {{
                const updatedLayout = plotlyDiv.layout;
                const updatedAnnotations = updatedLayout.annotations || [];
                const filteredAnnotations = updatedAnnotations.slice(0, -1);
                Plotly.relayout(plotlyDiv, {{ annotations: filteredAnnotations }});
            }}, INSPECTION_DWELL_TIME);
        }}
    }}

    function highlightInspectionPoint(currentIndex) {{
        if (!plotlyDiv || inspectionPlayState.inspectionTraceIndex === -1) return;

        const trace = plotlyDiv.data[inspectionPlayState.inspectionTraceIndex];
        if (!trace || !trace.marker) return;

        const n = inspectionPointsData.length;
        const sizes = new Array(n).fill(12);
        const colors = new Array(n).fill('#ff7f0e');

        sizes[currentIndex] = 20;
        colors[currentIndex] = '#ff0000';  // red

        Plotly.restyle(plotlyDiv, {{
            'marker.size': [sizes],
            'marker.color': [colors]
        }}, [inspectionPlayState.inspectionTraceIndex]);
    }}

    function pauseInspectionPlayback() {{
        if (inspectionPlayState.isPlaying && !inspectionPlayState.isPaused) {{
            inspectionPlayState.isPaused = true;
            updateStatus('已暂停');
        }}
    }}

    function resumeInspectionPlayback() {{
        if (inspectionPlayState.isPlaying && inspectionPlayState.isPaused) {{
            inspectionPlayState.isPaused = false;
            updateStatus('巡检播放中');
        }}
    }}

    function stopInspectionPlayback() {{
        if (inspectionPlayState.timer) {{
            clearTimeout(inspectionPlayState.timer);
            inspectionPlayState.timer = null;
        }}

        inspectionPlayState.isPlaying = false;
        inspectionPlayState.isPaused = false;
        inspectionPlayState.currentPathIndex = 0;
        inspectionPlayState.isMoving = false;

        for (let i = 0; i < inspectionPointsData.length; i++) {{
            delete inspectionPlayState[`triggered_${{i}}`];
        }}

        if (plotlyDiv && inspectionPlayState.inspectionTraceIndex !== -1) {{
            const n = inspectionPointsData.length;
            Plotly.restyle(plotlyDiv, {{
                'marker.size': [new Array(n).fill(12)],
                'marker.color': [new Array(n).fill('#ff7f0e')]
            }}, [inspectionPlayState.inspectionTraceIndex]);
        }}

        if (plotlyDiv && inspectionPlayState.playbackMarkerIndex !== -1 && continuousFlightPath.length > 0) {{
            const firstPathPoint = continuousFlightPath[0];
            Plotly.restyle(plotlyDiv, {{
                x: [[firstPathPoint[0]]],
                y: [[firstPathPoint[1]]]
            }}, [inspectionPlayState.playbackMarkerIndex]);
        }}

        closeInspectionPointInfo();

        updateStatus('未开始');
        document.getElementById('currentStatus').textContent = '-';
        document.getElementById('currentCoord').textContent = '-';
        document.getElementById('currentType').textContent = '-';
    }}

    function togglePlayMode() {{
        const checkbox = document.getElementById('inspectionModeCheckbox');
        isInspectionMode = checkbox.checked;

        const startBtn = document.querySelector('.btn-start');
        const hintEl = document.querySelector('.hint-text');

        if (isInspectionMode) {{
            if (startBtn) startBtn.title = '开始巡检播放';
            if (hintEl) hintEl.textContent = '🎯 巡检播放模式 | 按巡检点顺序播放 | 每点停留 2.5秒';

            if (plotlyDiv && connectTraceIndex !== -1) {{
                Plotly.restyle(plotlyDiv, {{ visible: false }}, [connectTraceIndex]);
            }}
            if (plotlyDiv && inspectionPlayState.playbackLineIndex !== -1) {{
                Plotly.restyle(plotlyDiv, {{ visible: true }}, [inspectionPlayState.playbackLineIndex]);
            }}
        }} else {{
            if (startBtn) startBtn.title = '开始飞行';
            if (hintEl) hintEl.textContent = '⚡ 连接段速度更快 | 🔍 巡检段流畅 | 🔄 从起点重规划';

            if (plotlyDiv && connectTraceIndex !== -1) {{
                Plotly.restyle(plotlyDiv, {{ visible: true }}, [connectTraceIndex]);
            }}
            if (plotlyDiv && inspectionPlayState.playbackLineIndex !== -1) {{
                Plotly.restyle(plotlyDiv, {{ visible: false }}, [inspectionPlayState.playbackLineIndex]);
            }}
        }}
    }}

    const originalStartFlight = startFlight;
    const originalPauseFlight = pauseFlight;
    const originalResumeFlight = resumeFlight;
    const originalStopFlight = stopFlight;

    startFlight = function() {{
        if (isInspectionMode) {{
            startInspectionPlayback();
        }} else {{
            originalStartFlight();
        }}
    }};

    pauseFlight = function() {{
        if (isInspectionMode) {{
            pauseInspectionPlayback();
        }} else {{
            originalPauseFlight();
        }}
    }};

    resumeFlight = function() {{
        if (isInspectionMode) {{
            resumeInspectionPlayback();
        }} else {{
            originalResumeFlight();
        }}
    }};

    stopFlight = function() {{
        if (isInspectionMode) {{
            stopInspectionPlayback();
        }} else {{
            originalStopFlight();
        }}
    }};

    function updateStatus(status) {{
        const el = document.getElementById('flightStatus');
        el.textContent = status;
        el.style.color = status === '飞行中' ? '#4CAF50' : status === '已暂停' ? '#ff9800' : '#999';
    }}

    function updateReplanStatus(msg, isError = false) {{
        const el = document.getElementById('replanStatus');
        if (el) {{
            el.textContent = msg;
            el.style.color = isError ? '#f44336' : '#4CAF50';
        }}
    }}

    async function replanFromStartPoint() {{
        try {{
            const xInput = document.getElementById('inputX');
            const yInput = document.getElementById('inputY');

            if (!xInput || !yInput) {{
                throw new Error('找不到 X/Y 输入框');
            }}

            const x = Number(xInput.value);
            const y = Number(yInput.value);

            if (!Number.isFinite(x) || !Number.isFinite(y)) {{
                throw new Error('请输入合法的 X/Y 坐标');
            }}

            updateReplanStatus('正在发送重规划请求...');

            const resp = await fetch('http://127.0.0.1:8000/replan', {{
                method: 'POST',
                headers: {{ 'Content-Type': 'application/json' }},
                body: JSON.stringify({{ x, y }})
            }});

            const result = await resp.json();

            console.log('replan result:', result);

            if (!resp.ok) {{
                throw new Error(result?.detail || result?.message || '后端返回失败');
            }}

            updateReplanStatus('重规划请求成功');

            if (result.is_adopted) {{
                console.log('新方案已被采纳，正在加载数据...');
                await reloadMissionDataAfterReplan();
            }} else {{
                console.log('新方案未被采纳:', result.comparison?.reason);
            }}

        }} catch (err) {{
            console.error('replan failed:', err);
            updateReplanStatus('重规划失败', true);
            alert('重规划失败: ' + err.message + '\\n\\n请确保服务已启动: python scripts/replan_service.py');
        }}
    }}

    async function reloadMissionDataAfterReplan() {{
        try {{
            updateReplanStatus('正在加载新规划结果...');

            stopInspectionPlayback();

            const resp = await fetch('result/latest/mission_output.json?t=' + Date.now());
            if (!resp.ok) {{
                throw new Error('加载新规划数据失败');
            }}

            const newMissionData = await resp.json();
            console.log('新规划数据:', newMissionData);

            const newSegments = newMissionData.segments || [];
            const newInspectionPointsWithMetadata = [];

            let pointCounter = 0;
            const inspectionSpacingPx = 70;

            for (const seg of newSegments) {{
                if (seg.type === 'inspect') {{
                    const geo = seg.geometry_2d || [];
                    if (geo.length < 2) continue;

                    const segId = seg.segment_id;
                    const edgeId = seg.edge_id;
                    const groupId = seg.group_id;

                    const startPoint = geo[0];
                    const pointId = `IP_${{pointCounter.toString().padStart(4, '0')}}`;
                    newInspectionPointsWithMetadata.push({{
                        point_id: pointId,
                        segment_id: segId,
                        edge_id: edgeId,
                        group_id: groupId,
                        sequence_in_path: pointCounter,
                        position: [startPoint[0], startPoint[1]],
                        type: 'sample',
                        status: 'uninspected',
                        result_image: await checkInspectionImageExists(pointId),
                        result_text: '',
                        photo_path: await checkInspectionImageExists(pointId),
                        description: `Start of ${{edgeId}}`
                    }});
                    pointCounter++;

                    let accumulatedDist = 0;
                    let lastSampleDist = 0;

                    for (let i = 0; i < geo.length - 1; i++) {{
                        const p1 = geo[i];
                        const p2 = geo[i + 1];
                        const segmentDist = Math.sqrt(Math.pow(p2[0] - p1[0], 2) + Math.pow(p2[1] - p1[1], 2));
                        accumulatedDist += segmentDist;

                        while (accumulatedDist >= lastSampleDist + inspectionSpacingPx) {{
                            const targetDist = lastSampleDist + inspectionSpacingPx;
                            const distIntoSegment = targetDist - (accumulatedDist - segmentDist);
                            const ratio = segmentDist > 0 ? distIntoSegment / segmentDist : 0;

                            const sampleX = p1[0] + (p2[0] - p1[0]) * ratio;
                            const sampleY = p1[1] + (p2[1] - p1[1]) * ratio;

                            const samplePointId = `IP_${{pointCounter.toString().padStart(4, '0')}}`;
                            newInspectionPointsWithMetadata.push({{
                                point_id: samplePointId,
                                segment_id: segId,
                                edge_id: edgeId,
                                group_id: groupId,
                                sequence_in_path: pointCounter,
                                position: [sampleX, sampleY],
                                type: 'sample',
                                status: 'uninspected',
                                result_image: await checkInspectionImageExists(samplePointId),
                                result_text: '',
                                photo_path: await checkInspectionImageExists(samplePointId),
                                description: `Inspection point on ${{edgeId}}`
                            }});
                            pointCounter++;
                            lastSampleDist = targetDist;
                        }}
                    }}
                }}
            }}

            console.log('生成的新巡检点数据:', newInspectionPointsWithMetadata.length, '个点');

            inspectionPointsData.length = 0;
            inspectionPointsData.push(...newInspectionPointsWithMetadata);

            // Rebuild continuousFlightPath with interpolation after replan
            function interpolatePath(p1, p2, steps) {{
                const points = [];
                for (let i = 0; i <= steps; i++) {{
                    const t = i / steps;
                    points.push([
                        p1[0] + (p2[0] - p1[0]) * t,
                        p1[1] + (p2[1] - p1[1]) * t
                    ]);
                }}
                return points;
            }}

            // Rebuild flightPath from new segments (filter null values)
            let newFlightPath = [];
            for (const seg of newSegments) {{
                if (seg.geometry_2d) {{
                    for (const pt of seg.geometry_2d) {{
                        newFlightPath.push([pt[0], pt[1]]);
                    }}
                    newFlightPath.push(null);  // Segment separator
                }}
            }}
            // Remove last null if exists
            if (newFlightPath.length > 0 && newFlightPath[newFlightPath.length - 1] === null) {{
                newFlightPath.pop();
            }}

            // Filter and create continuous path
            let newContinuousFlightPath = newFlightPath.filter(pt => pt !== null && pt !== undefined);

            // Add interpolated path from mission start point to first inspection point
            if (newInspectionPointsWithMetadata.length > 0) {{
                const firstInspectionPoint = newInspectionPointsWithMetadata[0].position;
                if (firstInspectionPoint && firstInspectionPoint.length >= 2) {{
                    const startToFirstPath = interpolatePath(missionStartPoint, firstInspectionPoint, 30);
                    newContinuousFlightPath = [...startToFirstPath, ...newContinuousFlightPath];
                }}
            }}

            continuousFlightPath = newContinuousFlightPath;

            await reinitializeInspectionPlaybackSystem(newInspectionPointsWithMetadata);

            currentMissionData = newMissionData;


            updateReplanStatus('新规划结果已加载');
            alert('✅ 新规划结果已加载！共 ' + newInspectionPointsWithMetadata.length + ' 个巡检点');

        }} catch (err) {{
            console.error('reload mission data failed:', err);
            updateReplanStatus('正在刷新页面以加载新规划结果...', false);
            setTimeout(() => location.reload(), 500);
        }}
    }}

    async function checkInspectionImageExists(pointId) {{
        try {{
            const imagePath = `result/latest/inspection_images/${{pointId}}.jpg`;
            const resp = await fetch(imagePath + '?t=' + Date.now(), {{ method: 'HEAD' }});
            if (resp.ok) {{
                return `inspection_images/${{pointId}}.jpg`;
            }}
            return null;
        }} catch (e) {{
            return null;
        }}
    }}

    async function reinitializeInspectionPlaybackSystem(newInspectionPoints) {{
        if (!plotlyDiv) return;

        if (inspectionPlayState.playbackMarkerIndex !== -1) {{
            try {{
                Plotly.deleteTraces(plotlyDiv, inspectionPlayState.playbackMarkerIndex);
            }} catch (e) {{
                console.warn('删除旧播放指示点失败:', e);
            }}
        }}

        if (inspectionPlayState.playbackLineIndex !== -1) {{
            try {{
                Plotly.deleteTraces(plotlyDiv, inspectionPlayState.playbackLineIndex);
            }} catch (e) {{
                console.warn('删除旧播放辅助线失败:', e);
            }}
        }}

        // Re-add start point marker (with offset to avoid overlap)
        addStartPointMarker();

        // Add playback indicator marker (small plane)
        addInspectionPlaybackMarker();

        // Add inspection playback auxiliary line (with interpolation)
        addInspectionPlaybackLine();

        buildInspectionPointPathIndexMapping();

        for (let i = 0; i < plotlyDiv.data.length; i++) {{
            if (plotlyDiv.data[i].name && plotlyDiv.data[i].name.includes('巡检点')) {{
                inspectionPlayState.inspectionTraceIndex = i;

                const sampleX = newInspectionPoints.map(p => p.position[0]);
                const sampleY = newInspectionPoints.map(p => p.position[1]);

                const newCustomData = newInspectionPoints.map(p => ({{
                    point_id: p.point_id,
                    segment_id: p.segment_id,
                    edge_id: p.edge_id,
                    group_id: p.group_id,
                    sequence: p.sequence_in_path,
                    position: p.position,
                    type: p.type || 'sample',
                    status: p.status || 'uninspected',
                    result_image: p.result_image,
                    result_text: p.result_text || '',
                    photo_path: p.photo_path || ''
                }}));

                Plotly.restyle(plotlyDiv, {{
                    x: [sampleX],
                    y: [sampleY],
                    customdata: [newCustomData]
                }}, [i]);

                console.log('巡检点 trace 已更新:', i);
                break;
            }}
        }}

        inspectionPlayState.currentIndex = 0;
        inspectionPlayState.isPlaying = false;
        inspectionPlayState.isPaused = false;
        inspectionPlayState.isMoving = false;
        inspectionPlayState.arrivedAtPoint = false;

        if (inspectionPlayState.timer) {{
            clearTimeout(inspectionPlayState.timer);
            inspectionPlayState.timer = null;
        }}
        if (inspectionPlayState.moveTimer) {{
            clearTimeout(inspectionPlayState.moveTimer);
            inspectionPlayState.moveTimer = null;
        }}

        closeInspectionPointInfo();

        document.getElementById('currentStatus').textContent = '-';
        document.getElementById('currentCoord').textContent = '-';
        document.getElementById('currentType').textContent = '-';

        console.log('巡检播放系统已重新初始化');
    }}

    function setupInspectionPointClickHandler() {{
        const plotlyDivs = document.getElementsByClassName('plotly-graph-div');
        if (plotlyDivs.length === 0) {{
            console.error('未找到 Plotly 图表 div');
            return;
        }}

        const plotlyDiv = plotlyDivs[0];
        console.log('绑定点击事件到:', plotlyDiv);

        plotlyDiv.on('plotly_click', function(data) {{
            console.log('plotly_click 触发');
            if (!data.points || data.points.length === 0) {{
                console.log('没有点击到点');
                return;
            }}

            const point = data.points[0];
            const traceName = point.data.name;
            console.log('点击的 trace:', traceName);

            if (traceName && traceName.includes('巡检点')) {{
                console.log('这是巡检点点击!');
                const customData = point.data.customdata;
                const idx = point.pointNumber;
                console.log('customdata:', customData);
                console.log('idx:', idx);

                if (customData && customData[idx]) {{
                    const pointData = customData[idx];
                    console.log('巡检点数据:', pointData);

                    updateInspectionPointPanel(pointData);
                }} else {{
                    console.log('customdata 或 idx 无效');
                }}
            }}
        }});
    }}

    function updateInspectionPointPanel(pointData) {{
        console.log('更新信息面板:', pointData);

        const infoElements = {{
            'pointInfo_id': pointData.point_id || 'N/A',
            'pointInfo_segment': pointData.segment_id || 'N/A',
            'pointInfo_edge': pointData.edge_id || 'N/A',
            'pointInfo_group': pointData.group_id || '未指定',
            'pointInfo_sequence': pointData.sequence || 'N/A',
            'pointInfo_coord': pointData.position ?
                `(${{pointData.position[0].toFixed(1)}}, ${{pointData.position[1].toFixed(1)}})` : 'N/A',
            'pointInfo_type': pointData.type || 'sample',
            'pointInfo_status': getStatusText(pointData.status || 'uninspected')
        }};

        for (const [id, value] of Object.entries(infoElements)) {{
            const el = document.getElementById(id);
            if (el) {{
                el.textContent = value;
            }}
        }}

        const statusElement = document.getElementById('pointInfo_status');
        if (statusElement) {{
            statusElement.className = 'info-value status-' + (pointData.status || 'uninspected');
        }}

        const imgElement = document.getElementById('inspectionPointImage');
        const noImageMsg = document.getElementById('noImageMessage');

        if (pointData.result_image) {{
            console.log('显示图片:', pointData.result_image);
            imgElement.src = pointData.result_image;
            imgElement.style.display = 'block';
            imgElement.onload = function() {{
                noImageMsg.style.display = 'none';
            }};
            imgElement.onerror = function() {{
                console.log('图片加载失败，显示占位消息');
                imgElement.style.display = 'none';
                noImageMsg.style.display = 'block';
            }};
        }} else {{
            console.log('无图片，显示占位消息');
            imgElement.style.display = 'none';
            noImageMsg.style.display = 'block';
        }}

        const panel = document.getElementById('inspectionPointInfoPanel');
        if (panel) {{
            panel.style.display = 'block';
        }}
    }}

    function getStatusText(status) {{
        switch(status) {{
            case 'normal': return '正常';
            case 'abnormal': return '异常';
            case 'uninspected': return '未检测';
            default: return '未检测';
        }}
    }}

    function closeInspectionPointInfo() {{
        document.getElementById('inspectionPointInfoPanel').style.display = 'none';
    }}

    document.addEventListener('DOMContentLoaded', function() {{
        // First, reset to clean initial state
        resetToCleanInitialState();

        // Then, initialize the flight system after a short delay
        setTimeout(initializeFlightSystem, 500);

        // Setup inspection point click handler
        setTimeout(setupInspectionPointClickHandler, 2000);

        console.log('DOMContentLoaded 完成，已恢复干净初始态');
    }});

    function onInspectionPointClick(pointData) {{
        document.getElementById('pointInfo_id').textContent = pointData.point_id || 'N/A';
        document.getElementById('pointInfo_segment').textContent = pointData.segment_id || 'N/A';
        document.getElementById('pointInfo_edge').textContent = pointData.edge_id || 'N/A';
        document.getElementById('pointInfo_group').textContent = pointData.group_id || '未指定';
        document.getElementById('pointInfo_sequence').textContent = pointData.sequence_in_path || 'N/A';
        document.getElementById('pointInfo_type').textContent = pointData.type || 'sample';
        document.getElementById('pointInfo_coord').textContent =
            `(${{pointData.position[0].toFixed(1)}}, ${{pointData.position[1].toFixed(1)}})`;

        const status = pointData.status || 'uninspected';
        const statusElement = document.getElementById('pointInfo_status');
        statusElement.textContent = getStatusText(status);
        statusElement.className = 'info-value status-' + status;

        const imageContainer = document.getElementById('imageContainer');
        const imgElement = document.getElementById('inspectionPointImage');
        const noImageMsg = document.getElementById('noImageMessage');

        if (pointData.result_image) {{
            imgElement.src = pointData.result_image;
            imgElement.style.display = 'block';
            noImageMsg.style.display = 'none';
        }} else {{
            imgElement.style.display = 'none';
            noImageMsg.style.display = 'block';
        }}

        document.getElementById('inspectionPointInfoPanel').style.display = 'block';
    }}

    function getStatusText(status) {{
        switch(status) {{
            case 'normal': return '正常';
            case 'abnormal': return '异常';
            case 'uninspected': return '未检测';
            default: return '未检测';
        }}
    }}

    function closeInspectionPointInfo() {{
        document.getElementById('inspectionPointInfoPanel').style.display = 'none';
    }}
    </script>

    <style>
    #flightControlPanel {{
        position: fixed;
        top: 100px;
        right: 0;
        width: 50px;
        background: rgba(255, 255, 255, 0.95);
        border: 2px solid #1f77b4;
        border-left: none;
        border-radius: 8px 0 0 8px;
        padding: 7px;
        box-shadow: -2px 2px 12px rgba(0,0,0,0.15);
        z-index: 1001;
        transition: width 0.3s ease;
        box-sizing: border-box;
    }}

    #panelToggleBtn {{
        width: 36px;
        height: 36px;
        margin: 0 auto;
        background: #1f77b4;
        color: white;
        border: none;
        border-radius: 4px;
        cursor: pointer;
        font-size: 18px;
        display: flex;
        align-items: center;
        justify-content: center;
        box-sizing: border-box;
    }}

    #panelToggleBtn:hover {{
        background: #1452a2;
    }}


    #panelContent {{ padding: 0 15px 15px 15px; display: none; }}
    #flightControlPanel h3 {{
        margin: 0 0 15px 0; font-size: 16px; color: #1f77b4;
        border-bottom: 2px solid #1f77b4; padding-bottom: 8px;
    }}
    .coord-info {{
        background: #e3f2fd; border-radius: 4px; padding: 8px; margin-bottom: 15px;
        font-size: 11px; color: #0d47a1;
    }}
    .coord-info-row {{ display: flex; justify-content: space-between; margin: 3px 0; }}
    .control-group {{ margin-bottom: 12px; }}
    .control-group label {{
        display: inline-block; width: 30px; font-weight: bold;
        font-size: 13px; color: #555;
    }}
    .control-group input {{
        width: 100px; padding: 6px; border: 1px solid #ccc;
        border-radius: 4px; font-size: 13px;
    }}
    .button-group {{ display: flex; gap: 6px; margin: 12px 0; }}
    .button-group button {{
        flex: 1; padding: 10px; border: none; border-radius: 5px;
        cursor: pointer; font-weight: bold; font-size: 13px;
    }}
    .btn-start {{ background: #4CAF50; color: white; }}
    .btn-pause {{ background: #ff9800; color: white; }}
    .btn-resume {{ background: #2196F3; color: white; }}
    .btn-reset {{ background: #f44336; color: white; }}
    .btn-replan {{ background: #9C27B0; color: white; }}
    .status-display {{
        background: #f8f9fa; border-radius: 5px; padding: 10px;
        margin-top: 12px; font-size: 12px;
    }}
    .status-item {{ margin: 4px 0; display: flex; justify-content: space-between; }}
    .status-item span:first-child {{ font-weight: bold; color: #555; font-size: 11px; }}
    .status-item span:last-child {{ color: #333; font-size: 11px; }}
    #flightStatus {{ font-weight: bold; font-size: 13px; }}
    .hint-text {{ font-size: 10px; color: #999; margin-top: 8px; line-height: 1.4; }}
    #inspectionPointInfoPanel {{
        position: fixed; top: 120px; left: 24px;
        background: rgba(255, 255, 255, 0.95);
        border: 2px solid #ff7f0e; border-radius: 8px;
        padding: 15px; width: 300px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        z-index: 9999; display: none;
        max-height: 70vh; overflow-y: auto;
    }}
    #inspectionPointInfoPanel h3 {{
        margin: 0 0 10px 0; font-size: 16px; color: #ff7f0e;
        border-bottom: 2px solid #ff7f0e; padding-bottom: 5px;
        display: flex; justify-content: space-between; align-items: center;
    }}
    .close-btn {{ background: none; border: none; font-size: 20px; cursor: pointer; color: #999; }}
    .close-btn:hover {{ color: #333; }}
    .info-row {{ display: flex; justify-content: space-between; margin: 8px 0; font-size: 12px; border-bottom: 1px solid #eee; padding-bottom: 5px; }}
    .info-label {{ font-weight: bold; color: #555; }}
    .info-value {{ color: #333; }}
    .status-uninspected {{ color: #999; }}
    .status-normal {{ color: #4CAF50; font-weight: bold; }}
    .status-abnormal {{ color: #f44336; font-weight: bold; }}
    #imageContainer {{ margin-top: 15px; text-align: center; border: 2px dashed #ddd; border-radius: 4px; padding: 15px; min-height: 120px; display: flex; align-items: center; justify-content: center; background: #fafafa; }}
    #inspectionPointImage {{ max-width: 100%; max-height: 150px; object-fit: contain; }}
    #noImageMessage {{ color: #999; font-size: 13px; text-align: center; }}
    </style>

    <div id="flightControlPanel">
        <button id="panelToggleBtn" onclick="toggleControlPanel()" title="展开">◀</button>
        <div id="panelContent">
            <h3>🚁 模拟飞行控制</h3>

            <!-- 模式切换 -->
            <div style="margin-bottom: 12px; padding: 8px; background: #e3f2fd; border-radius: 4px;">
                <label style="display: flex; align-items: center; cursor: pointer; font-size: 13px; color: #0d47a1;">
                    <input type="checkbox" id="inspectionModeCheckbox" onchange="togglePlayMode()" style="margin-right: 8px;">
                    <span>🎯 巡检播放模式（自动遍历巡检点）</span>
                </label>
            </div>

            <div class="coord-info">
                <div class="coord-info-row"><span>X: {x_min} ~ {x_max}</span></div>
                <div class="coord-info-row"><span>Y: {y_min} ~ {y_max}</span></div>
            </div>
            <div class="control-group">
                <label>X:</label>
                <input type="number" id="inputX" min="{x_min}" max="{x_max}" step="1" placeholder="X">
            </div>
            <div class="control-group">
                <label>Y:</label>
                <input type="number" id="inputY" min="{y_min}" max="{y_max}" step="1" placeholder="Y">
            </div>
            <div class="button-group">
                <button class="btn-start" onclick="startFlight()">开始</button>
                <button class="btn-pause" onclick="pauseFlight()">暂停</button>
            </div>
            <div class="button-group">
                <button class="btn-resume" onclick="resumeFlight()">继续</button>
                <button class="btn-reset" onclick="stopFlight()">重置</button>
            </div>
            <div class="button-group">
                <button class="btn-replan" onclick="replanFromStartPoint()">🔄 重规划</button>
            </div>
            <div class="status-display">
                <div class="status-item"><span>状态:</span><span id="flightStatus">未开始</span></div>
                <div class="status-item"><span>索引:</span><span id="currentStatus">-</span></div>
                <div class="status-item"><span>坐标:</span><span id="currentCoord">-</span></div>
                <div class="status-item"><span>类型:</span><span id="currentType">-</span></div>
                <div class="status-item"><span>重规划:</span><span id="replanStatus">-</span></div>
            </div>
            <p class="hint-text">⚡ 连接段速度更快 | 🔍 巡检段流畅 | 🔄 从起点重规划</p>
        </div>
    </div>

    <div id="inspectionPointInfoPanel">
        <h3>📍 巡检点信息<button class="close-btn" onclick="closeInspectionPointInfo()">&times;</button></h3>
        <div class="info-row"><span class="info-label">ID:</span><span class="info-value" id="pointInfo_id">-</span></div>
        <div class="info-row"><span class="info-label">段:</span><span class="info-value" id="pointInfo_segment">-</span></div>
        <div class="info-row"><span class="info-label">边:</span><span class="info-value" id="pointInfo_edge">-</span></div>
        <div class="info-row"><span class="info-label">分组:</span><span class="info-value" id="pointInfo_group">-</span></div>
        <div class="info-row"><span class="info-label">序号:</span><span class="info-value" id="pointInfo_sequence">-</span></div>
        <div class="info-row"><span class="info-label">类型:</span><span class="info-value" id="pointInfo_type">-</span></div>
        <div class="info-row"><span class="info-label">坐标:</span><span class="info-value" id="pointInfo_coord">-</span></div>
        <div class="info-row"><span class="info-label">状态:</span><span class="info-value" id="pointInfo_status">-</span></div>
        <div id="imageContainer">
            <img id="inspectionPointImage" style="display: none;">
            <div id="noImageMessage">📷 暂无巡检图片</div>
        </div>
    </div>
    """
def main():
    """主函数"""
    # 检查 Plotly 是否可用
    if not PLOTLY_AVAILABLE:
        print("="*70)
        print("UAV 电网巡检 - 交互式主展示页生成")
        print("="*70)
        print()
        print("[ERROR] Plotly 未安装，无法生成交互式可视化页面")
        print()
        print("[解决方法]")
        print("  pip install plotly")
        print()
        return None

    print("="*70)
    print("UAV 电网巡检 - 交互式主展示页生成")
    print("="*70)
    print()
    print("[页面类型]")
    print("  - 真正的交互式主展示页")
    print("  - 使用 Plotly layout.images 显示真实地图")
    print("  - 路径和 inspection points 支持 hover 交互")
    print("  - 图层控制（sample points 可通过图例显示/隐藏）")
    print()
    print("[底图信息]")
    print("  - 底图文件: data/test.png")
    print("  - 显示方式: layout.images (真实地图，非紫色平面)")
    print()

    # 创建输出目录
    os.makedirs('result/latest', exist_ok=True)

    # =====================================================
    # 生成交互式主展示页
    # =====================================================
    create_interactive_main_view(
        map_image_path='data/test.png',
        mission_json_path='result/latest/mission_output.json',
        output_html_path='result/latest/main_view_interactive.html'
    )

    # =====================================================
    # 完成
    # =====================================================
    print("\n" + "="*70)
    print("生成完成！")
    print("="*70)
    print()
    print("[输出文件]")
    print("  result/latest/main_view_interactive.html")
    print()
    print("[页面特性]")
    print("  [OK] 真实地图底图（data/test.png）")
    print("  [OK] 路径可交互（hover 显示信息）")
    print("  [OK] inspection points 可交互（hover 显示详情）")
    print("  [OK] 图层控制（sample points 可通过图例开关）")
    print("  [OK] 系统界面风格（非报告页）")
    print()
    print("[如何使用]")
    print("  1. 在浏览器中打开: result/latest/main_view_interactive.html")
    print("  2. 鼠标 hover 路径查看路径点信息")
    print("  3. 鼠标 hover inspection points 查看点详情")
    print("  4. 点击右侧图例显示/隐藏 sample points")
    print("  5. 使用工具栏缩放、平移、选择区域")
    print()


if __name__ == "__main__":
    main()
