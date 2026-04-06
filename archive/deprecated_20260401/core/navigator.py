"""
UAV导航模块

功能：
1. 从输入点导航到最近巡检点
2. 构建完整执行路径（接入段 + 巡检段）
3. 可视化导航路径
4. 创建执行动画
"""

import numpy as np
from typing import List, Dict, Tuple, Optional


def find_nearest_inspection_point(input_pos, inspection_points):
    """
    找到距离输入点最近的巡检点

    Args:
        input_pos: 输入点坐标 (x, y, z)
        inspection_points: 巡检任务点列表 (List[InspectionPoint])

    Returns:
        nearest_point: 最近的巡检点对象
        nearest_index: 最近巡检点在原路径中的索引
        distance: 距离（米）
    """
    if not inspection_points:
        return None, -1, float('inf')

    # 只考虑拍照点
    photo_points = [p for p in inspection_points if p.is_photo]

    if not photo_points:
        return None, -1, float('inf')

    input_pos_array = np.array(input_pos)
    min_distance = float('inf')
    nearest_point = None
    nearest_index = -1

    # 遍历所有拍照点，找最近的一个
    for point in photo_points:
        point_pos = np.array(point.position)
        distance = np.linalg.norm(input_pos_array - point_pos)

        if distance < min_distance:
            min_distance = distance
            nearest_point = point
            nearest_index = point.index

    return nearest_point, nearest_index, min_distance


def generate_connection_segment(input_pos, target_pos, num_points=8):
    """
    生成从输入点到目标点的平滑连接段（线性插值）

    Args:
        input_pos: 输入点坐标 (x, y, z)
        target_pos: 目标点坐标 (x, y, z)
        num_points: 插值点数量（默认8）

    Returns:
        连接段路径点列表（不包含input_pos，包含target_pos）
    """
    input_array = np.array(input_pos)
    target_array = np.array(target_pos)

    # 线性插值生成中间点
    segment_points = []
    for i in range(1, num_points + 1):
        t = i / (num_points + 1)
        interpolated = input_array + t * (target_array - input_array)
        segment_points.append(tuple(interpolated))

    # 最后添加目标点
    segment_points.append(tuple(target_array))

    return segment_points


def get_remaining_inspection_points(inspection_points, start_index):
    """
    获取从指定索引开始的剩余巡检点

    Args:
        inspection_points: 巡检任务点列表
        start_index: 起始索引（包含）

    Returns:
        剩余巡检点列表
    """
    return [p for p in inspection_points if p.index >= start_index]


def get_path_from_index(path_3d, start_index):
    """
    获取从指定索引开始的路径子段

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        start_index: 起始索引（包含）

    Returns:
        路径子段
    """
    if start_index < 0:
        start_index = 0
    if start_index >= len(path_3d):
        return []

    return path_3d[start_index:]


def navigate_from_input(input_pos, path_3d, inspection_points, connection_points=8):
    """
    从输入点导航到最近巡检点，并构建完整执行路径

    Args:
        input_pos: 输入点坐标 (x, y, z)
        path_3d: 完整3D路径 [(x, y, z), ...]
        inspection_points: 巡检任务点列表
        connection_points: 连接段插值点数量

    Returns:
        Dict: {
            "nearest_point": 最近的巡检点对象,
            "nearest_index": 最近巡检点索引,
            "distance": 距离（米）,
            "connection_segment": 接入段路径（输入点→最近巡检点）,
            "remaining_path": 剩余巡检路径,
            "full_execution_path": 完整执行路径（接入段+巡检段）,
            "remaining_points": 剩余巡检点列表,
            "remaining_photo_count": 剩余拍照点数量
        }
    """
    # 1. 找最近巡检点
    nearest_point, nearest_index, distance = find_nearest_inspection_point(
        input_pos, inspection_points
    )

    if nearest_point is None:
        return {
            "nearest_point": None,
            "nearest_index": -1,
            "distance": float('inf'),
            "connection_segment": [],
            "remaining_path": [],
            "full_execution_path": [],
            "remaining_points": [],
            "remaining_photo_count": 0
        }

    # 2. 生成接入段（输入点 → 最近巡检点）
    target_pos = nearest_point.position
    connection_segment = generate_connection_segment(
        input_pos, target_pos, num_points=connection_points
    )

    # 3. 获取剩余巡检路径
    remaining_path = get_path_from_index(path_3d, nearest_index)

    # 4. 构建完整执行路径（接入段 + 巡检段）
    # 注意：不重复加入最近巡检点（connection_segment已包含）
    full_execution_path = connection_segment + remaining_path[1:] if remaining_path else connection_segment

    # 5. 获取剩余巡检点
    remaining_points = get_remaining_inspection_points(
        inspection_points, nearest_index
    )

    # 6. 统计剩余拍照点数量
    remaining_photo_count = sum(1 for p in remaining_points if p.is_photo)

    return {
        "nearest_point": nearest_point,
        "nearest_index": nearest_index,
        "distance": distance,
        "connection_segment": connection_segment,
        "remaining_path": remaining_path,
        "full_execution_path": full_execution_path,
        "remaining_points": remaining_points,
        "remaining_photo_count": remaining_photo_count
    }


def create_navigation_visualization(input_pos, path_3d, inspection_points,
                                    output_file="output/navigation.html",
                                    connection_points=8):
    """
    创建导航可视化HTML（统一展示完整执行路径）

    Args:
        input_pos: 输入点坐标 (x, y, z)
        path_3d: 完整3D路径
        inspection_points: 巡检任务点列表
        output_file: 输出HTML文件路径
        connection_points: 连接段插值点数量
    """
    import plotly.graph_objects as go
    import os

    print("[导航可视化] 创建导航视图...")

    # 获取导航信息
    nav_info = navigate_from_input(input_pos, path_3d, inspection_points, connection_points)

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 创建图形
    fig = go.Figure()

    # 1. 完整原始路径（灰色虚线，淡化显示）
    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='lightgray', width=2, dash='dot'),
        name='Original Path',
        hovertemplate='Original<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 2. 完整执行路径（绿色加粗，高亮显示）
    # 这是UAV实际飞行的路径：输入点 → 接入段 → 巡检段
    if nav_info["full_execution_path"]:
        exec_x = [p[0] for p in nav_info["full_execution_path"]]
        exec_y = [p[1] for p in nav_info["full_execution_path"]]
        exec_z = [p[2] for p in nav_info["full_execution_path"]]

        fig.add_trace(go.Scatter3d(
            x=exec_x,
            y=exec_y,
            z=exec_z,
            mode='lines',
            line=dict(color='green', width=5),
            name='Full Execution Path',
            hovertemplate='Execution<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # 3. 输入点（紫色圆点）
    fig.add_trace(go.Scatter3d(
        x=[input_pos[0]],
        y=[input_pos[1]],
        z=[input_pos[2]],
        mode='markers',
        marker=dict(size=12, color='purple', symbol='circle',
                   line=dict(color='white', width=2)),
        name='Input Point (Start)',
        hovertemplate=f'Input Point<br>X: {input_pos[0]:.0f}<br>Y: {input_pos[1]:.0f}<br>Z: {input_pos[2]:.1f}m<extra></extra>'
    ))

    # 4. 最近巡检点（红色大钻石，接入点）
    if nav_info["nearest_point"]:
        np_pos = nav_info["nearest_point"].position
        fig.add_trace(go.Scatter3d(
            x=[np_pos[0]],
            y=[np_pos[1]],
            z=[np_pos[2]],
            mode='markers',
            marker=dict(size=15, color='red', symbol='diamond',
                       line=dict(color='yellow', width=2)),
            name='Nearest Inspection Point',
            hovertemplate=f'Nearest Point (index {nav_info["nearest_index"]})<br>X: {np_pos[0]:.0f}<br>Y: {np_pos[1]:.0f}<br>Z: {np_pos[2]:.1f}m<br>Connection: {nav_info["distance"]:.1f}m<extra></extra>'
        ))

    # 5. 所有巡检点（黄色小圆点）
    photo_x = [p.position[0] for p in inspection_points if p.is_photo]
    photo_y = [p.position[1] for p in inspection_points if p.is_photo]
    photo_z = [p.position[2] for p in inspection_points if p.is_photo]

    if photo_x:
        fig.add_trace(go.Scatter3d(
            x=photo_x,
            y=photo_y,
            z=photo_z,
            mode='markers',
            marker=dict(size=5, color='yellow', opacity=0.6),
            name='All Inspection Points',
            hovertemplate='Inspection Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # 添加导航信息标注
    info_text = (
        f"<b>Navigation Info</b><br>"
        f"Connection Distance: {nav_info['distance']:.1f} m<br>"
        f"Access Point Index: {nav_info['nearest_index']}<br>"
        f"Remaining Photos: {nav_info['remaining_photo_count']}<br>"
        f"Execution Waypoints: {len(nav_info['full_execution_path'])}<br>"
        f"<b>Path: Input → Connection → Inspection</b>"
    )

    fig.add_annotation(
        text=info_text,
        xref="paper", yref="paper",
        x=0.02, y=0.98,
        showarrow=False,
        font=dict(size=12, color="black"),
        bgcolor="rgba(255,255,255,0.85)",
        bordercolor="green",
        borderwidth=2,
        align="left"
    )

    # 设置布局
    fig.update_layout(
        title=dict(
            text="UAV Navigation - Full Execution Path",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            aspectmode='data',
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1400,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.75, bgcolor="rgba(255,255,255,0.8)")
    )

    # 保存文件
    fig.write_html(output_file, include_plotlyjs='cdn',
                   config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [保存] {output_file}")
    print(f"  [接入] 距离: {nav_info['distance']:.1f}m, 插值点: {len(nav_info['connection_segment'])}")
    print(f"  [起始] 索引: {nav_info['nearest_index']}")
    print(f"  [执行] 完整路径点: {len(nav_info['full_execution_path'])}")
    print(f"  [剩余] 拍照点: {nav_info['remaining_photo_count']}")

    return fig, nav_info


def create_navigation_animation(input_pos, path_3d, inspection_points,
                                output_file="output/navigation_animation.html",
                                connection_points=8,
                                downsample_step=2):
    """
    创建导航执行动画：从输入点起飞 → 接入段 → 巡检段执行

    Args:
        input_pos: 输入点坐标
        path_3d: 完整路径
        inspection_points: 巡检点列表
        output_file: 输出HTML文件
        connection_points: 连接段插值点数量
        downsample_step: 降采样步长（动画性能优化）
    """
    import plotly.graph_objects as go
    import os

    print("[导航动画] 创建执行动画...")

    nav_info = navigate_from_input(input_pos, path_3d, inspection_points, connection_points)

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 获取完整执行路径
    execution_path = nav_info["full_execution_path"]

    # 对执行路径进行降采样（性能优化）
    # 注意：需要保留巡检点
    photo_indices = set(p.index for p in nav_info["remaining_points"] if p.is_photo)

    # 构建降采样路径（保留所有巡检点）
    path_vis = []
    connection_len = len(nav_info["connection_segment"])

    for i, point in enumerate(execution_path):
        # 接入段：全部保留（点数少）
        # 巡检段：降采样但保留巡检点
        if i < connection_len:
            path_vis.append((i, point))
        else:
            # 巡检段：计算原始路径索引
            original_idx = nav_info["nearest_index"] + (i - connection_len)
            if i % downsample_step == 0 or original_idx in photo_indices:
                path_vis.append((original_idx, point))

    path_indices = [item[0] for item in path_vis]
    path_coords = [item[1] for item in path_vis]

    path_x = [p[0] for p in path_coords]
    path_y = [p[1] for p in path_coords]
    path_z = [p[2] for p in path_coords]

    # 创建图形
    fig = go.Figure()

    # 1. 原始完整路径（灰色虚线）
    full_x = [p[0] for p in path_3d]
    full_y = [p[1] for p in path_3d]
    full_z = [p[2] for p in path_3d]

    fig.add_trace(go.Scatter3d(
        x=full_x,
        y=full_y,
        z=full_z,
        mode='lines',
        line=dict(color='lightgray', width=2, dash='dot'),
        name='Original Path'
    ))

    # 2. 完整执行路径（绿色实线）
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='green', width=3),
        name='Execution Path'
    ))

    # 3. UAV初始位置
    uav_trace_index = len(fig.data)
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]],
        y=[path_y[0]],
        z=[path_z[0]],
        mode='markers',
        marker=dict(size=6, color='orange'),
        name='UAV'
    ))

    # 4. 构建动画帧
    frames = []
    photo_frame_count = 0
    connection_len_vis = len(nav_info["connection_segment"])

    for i, (original_idx, point) in enumerate(path_vis):
        x, y, z = point[0], point[1], point[2]

        # 判断当前所在阶段和是否是拍照点
        is_in_connection = i < connection_len_vis
        is_photo = original_idx in photo_indices and not is_in_connection

        # 设置视觉效果
        if is_in_connection:
            # 接入段：橙色正常飞行
            color = "orange"
            size = 5
            text = "Connecting..." if i < connection_len_vis - 1 else "Arrived"
        elif is_photo:
            # 巡检点：红色大尺寸 + 文本
            color = "red"
            size = 8
            text = f"WP {original_idx} - Photo"
        else:
            # 巡检段普通点：橙色
            color = "orange"
            size = 5
            text = f"WP {original_idx}"

        frame = go.Frame(
            data=[
                go.Scatter3d(
                    x=[x],
                    y=[y],
                    z=[z],
                    mode='markers+text',
                    marker=dict(size=size, color=color),
                    text=[text],
                    textposition='top center',
                    textfont=dict(size=10, color='black')
                )
            ],
            traces=[uav_trace_index],
            name=f'Frame {i+1}'
        )

        frames.append(frame)

        # 巡检点停顿效果
        if is_photo:
            frames.append(frame)
            photo_frame_count += 1

    fig.frames = frames

    # 设置布局
    fig.update_layout(
        title=dict(
            text=f"UAV Navigation Execution - {nav_info['remaining_photo_count']} Photos",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            aspectmode='data',
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1200,
        height=800,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
        updatemenus=[
            dict(
                type='buttons',
                showactive=False,
                x=1.0,
                y=1.15,
                xanchor='right',
                yanchor='top',
                direction='left',
                buttons=[
                    dict(
                        label='Play',
                        method='animate',
                        args=[None, {
                            "frame": {"duration": 50, "redraw": False, "fromcurrent": True},
                            "transition": {"duration": 0}
                        }]
                    ),
                    dict(
                        label='Pause',
                        method='animate',
                        args=[[None], {
                            "frame": {"duration": 0, "redraw": False, "fromcurrent": True},
                            "mode": "immediate",
                            "transition": {"duration": 0}
                        }]
                    )
                ]
            )
        ]
    )

    # 保存文件
    fig.write_html(output_file, include_plotlyjs='cdn',
                   config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [保存] {output_file}")
    print(f"  [动画] 帧数: {len(frames)} (含{photo_frame_count}个停顿帧)")
    print(f"  [路径] 接入段: {connection_len_vis}点, 巡检段: {len(path_vis)-connection_len_vis}点")

    return fig, nav_info
