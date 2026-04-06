"""
=====================================================
3D可视化模块 (3D Visualization Module)
=====================================================
功能说明:
    该模块使用Plotly实现3D路径规划结果的可视化。
    支持绘制地图障碍物、规划路径、起终点等元素。

主要功能:
    - 3D交互式地图渲染
    - 障碍物（建筑）显示
    - 路径线条绘制
    - 起终点标记
    - 路径统计信息
    - 地面参考平面
    - 高度热力图效果
=====================================================
"""

import numpy as np
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import os
from config.settings import FIGURES_DIR, FAST_MODE, FAST_MODE_NO_POPUP


def draw_box(fig, x0, x1, y0, y1, z0, z1, color='rgba(200,0,0,0.6)'):
    """
    在3D图中绘制长方体（用于表示建筑/障碍物）

    该函数通过定义长方体的8个顶点和12个三角面来绘制3D长方体。

    参数:
        fig: plotly.graph_objects.Figure - 图形对象
        x0, x1: float - x方向的边界
        y0, y1: float - y方向的边界
        z0, z1: float - z方向的边界
        color: str - 颜色（支持rgba格式）

    顶点编号:
        0: (x0, y0, z0)  1: (x1, y0, z0)
        2: (x1, y1, z0)  3: (x0, y1, z0)
        4: (x0, y0, z1)  5: (x1, y0, z1)
        6: (x1, y1, z1)  7: (x0, y1, z1)
    """
    # 定义长方体的8个顶点
    vertices = np.array([
        [x0, y0, z0],  # 0
        [x1, y0, z0],  # 1
        [x1, y1, z0],  # 2
        [x0, y1, z0],  # 3
        [x0, y0, z1],  # 4
        [x1, y0, z1],  # 5
        [x1, y1, z1],  # 6
        [x0, y1, z1],  # 7
    ])

    # 定义12个三角面（每个面2个三角形）
    faces = [
        [0, 1, 2], [0, 2, 3],  # 底面
        [4, 5, 6], [4, 6, 7],  # 顶面
        [0, 1, 5], [0, 5, 4],  # 前面
        [2, 3, 7], [2, 7, 6],  # 后面
        [1, 2, 6], [1, 6, 5],  # 右面
        [0, 3, 7], [0, 7, 4],  # 左面
    ]

    # 提取三角面索引
    i = [f[0] for f in faces]
    j = [f[1] for f in faces]
    k = [f[2] for f in faces]

    # 添加3D网格到图中
    fig.add_trace(go.Mesh3d(
        x=vertices[:, 0],
        y=vertices[:, 1],
        z=vertices[:, 2],
        i=i, j=j, k=k,
        color=color,
        opacity=0.7,
        flatshading=True
    ))


def visualize(grid, path, save_html=True, html_filename="3d_path_visualization.html",
               show_obstacles=True, show_ground=True, show_info=True):
    """
    可视化3D地图和路径（增强版）

    该函数创建一个完整的3D场景，包括：
    1. 地图中所有建筑物（障碍物）
    2. 规划出的路径（带渐变颜色的平滑曲线）
    3. 起点（绿色大标记）和终点（红色大标记）
    4. 地面参考平面
    5. 路径统计信息

    参数:
        grid: GridMap3D - 3D栅格地图对象
        path: list - 路径点列表，每个元素为 (x, y, z) 栅格索引
        save_html: bool - 是否将可视化保存为HTML文件，默认为True
        html_filename: str - 保存的HTML文件名，默认为"3d_path_visualization.html"
        show_obstacles: bool - 是否显示障碍物
        show_ground: bool - 是否显示地面平面
        show_info: bool - 是否显示统计信息
    """
    # 创建图形对象
    fig = go.Figure()

    res = grid.resolution

    # ===== 绘制地面参考平面 =====
    if show_ground:
        # 创建地面网格
        ground_x = [grid.b_min[0], grid.b_max[0], grid.b_max[0], grid.b_min[0], grid.b_min[0]]
        ground_y = [grid.b_min[1], grid.b_min[1], grid.b_max[1], grid.b_max[1], grid.b_min[1]]
        ground_z = [grid.b_min[2]] * 5

        fig.add_trace(go.Scatter3d(
            x=ground_x,
            y=ground_y,
            z=ground_z,
            mode='lines',
            line=dict(width=2, color='lightgray'),
            name='地面边界',
            showlegend=False
        ))

        # 地面填充（半透明）
        if hasattr(grid, 'b_min'):
            xx, yy = np.meshgrid(
                [grid.b_min[0], grid.b_max[0]],
                [grid.b_min[1], grid.b_max[1]]
            )
            zz = np.full_like(xx, grid.b_min[2])

            fig.add_trace(go.Surface(
                x=xx,
                y=yy,
                z=zz,
                colorscale=[[0, 'rgba(240,240,240,0.3)'], [1, 'rgba(240,240,240,0.3)']],
                showscale=False,
                showlegend=False,
                name='地面'
            ))

    # ===== 绘制建筑物/障碍物 =====
    if show_obstacles:
        # 兼容两种地图类型
        if hasattr(grid, "json_world"):
            # GridMap3D: 从 JSON 加载建筑物墙
            for wall in grid.json_world.get("walls", []):
                x0, y0, z0 = wall["plane"]["start"]
                x1, y1, z1 = wall["plane"]["stop"]
                # 使用更现代的颜色
                draw_box(fig, x0, x1, y0, y1, z0, z1, color='rgba(220, 50, 50, 0.5)')
        elif hasattr(grid, "height_map"):
            # HeightMap3D: 绘制地形表面
            # 创建网格坐标
            h, w = grid.height_map.shape
            x_coords = np.arange(w) * res + grid.b_min[0]
            y_coords = np.arange(h) * res + grid.b_min[1]
            X, Y = np.meshgrid(x_coords, y_coords)
            Z = grid.height_map

            # 绘制地形表面（使用地形高度作为颜色）
            fig.add_trace(go.Surface(
                x=X,
                y=Y,
                z=Z,
                colorscale='Earth',
                colorbar=dict(title='高度 (m)', x=1.02),
                opacity=0.85,
                name='地形',
                showscale=True,
                hovertemplate='X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
            ))

    # ===== 绘制路径 =====
    if path:
        # 将栅格索引转换为物理坐标
        px = [grid.b_min[0] + p[0] * res for p in path]
        py = [grid.b_min[1] + p[1] * res for p in path]
        pz = [grid.b_min[2] + p[2] * res for p in path]

        # 计算路径统计信息
        path_length = 0
        for i in range(1, len(path)):
            dx = (path[i][0] - path[i-1][0]) * res
            dy = (path[i][1] - path[i-1][1]) * res
            dz = (path[i][2] - path[i-1][2]) * res
            path_length += np.sqrt(dx**2 + dy**2 + dz**2)

        min_height = min(pz) if pz else 0
        max_height = max(pz) if pz else 0
        avg_height = np.mean(pz) if pz else 0

        # 创建颜色渐变（基于高度）
        colors = px
        for i in range(len(pz)):
            # 根据高度归一化颜色
            height_ratio = (pz[i] - min_height) / (max_height - min_height + 0.001)
            colors[i] = height_ratio

        # 添加3D路径线（带颜色渐变）
        fig.add_trace(go.Scatter3d(
            x=px,
            y=py,
            z=pz,
            mode='lines+markers',
            line=dict(
                width=4,
                color=px,  # 使用x坐标创建渐变效果
                colorscale='Viridis',  # 现代配色方案
                showscale=False
            ),
            marker=dict(
                size=3,
                color=px,
                colorscale='Viridis',
                opacity=0.6
            ),
            name=f'规划路径 ({len(path)}点)',
            hovertemplate='X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # ===== 绘制起点（绿色球标记）=====
        fig.add_trace(go.Scatter3d(
            x=[px[0]],
            y=[py[0]],
            z=[pz[0]],
            mode='markers',
            marker=dict(
                size=5,
                color='rgb(0, 200, 100)',
                symbol='circle'
            ),
            name='起点',
            hovertemplate='起点<br>X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # ===== 绘制终点（红色球标记）=====
        fig.add_trace(go.Scatter3d(
            x=[px[-1]],
            y=[py[-1]],
            z=[pz[-1]],
            mode='markers',
            marker=dict(
                size=5,
                color='rgb(220, 50, 50)',
                symbol='circle'
            ),
            name='终点',
            hovertemplate='终点<br>X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 添加路径方向箭头（每隔一段距离）
        arrow_interval = max(1, len(path) // 15)
        for i in range(0, len(path) - arrow_interval, arrow_interval):
            if i + arrow_interval < len(path):
                # 箭头位置
                idx = i + arrow_interval // 2
                fig.add_trace(go.Scatter3d(
                    x=[px[idx]],
                    y=[py[idx]],
                    z=[pz[idx]],
                    mode='markers',
                    marker=dict(
                        size=8,
                        color='rgba(100, 100, 255, 0.5)',
                        symbol='diamond'
                    ),
                    showlegend=False,
                    hoverinfo='skip'
                ))

    # ===== 设置场景布局 =====
    title_text = "UAV 3D Inspection Path Planning"
    if path and show_info:
        title_text += f"<br><sup>Path Length: {path_length:.1f}m | Waypoints: {len(path)} | Altitude: {min_height:.1f}-{max_height:.1f}m</sup>"

    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=0.8),  # 优化视角避免遮挡
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(
                    text='X (m)',
                    font=dict(size=14, color='rgb(50, 50, 50)')
                )
            ),
            yaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(
                    text='Y (m)',
                    font=dict(size=14, color='rgb(50, 50, 50)')
                )
            ),
            zaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(
                    text='Z (m)',
                    font=dict(size=14, color='rgb(50, 50, 50)')
                )
            )
        ),
        width=1200,
        height=900,
        margin=dict(l=0, r=0, t=40, b=0),  # 避免边缘裁剪
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.7)"  # 避免图例重叠
        ),
        title=dict(
            text=title_text,
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        font=dict(family="Arial, sans-serif"),
        paper_bgcolor='white',
        plot_bgcolor='white'
    )

    # ===== 保存HTML文件 =====
    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn', config={'displayModeBar': True, 'displaylogo': False})
        print(f"[INFO] 3D可视化已保存: {html_path}")

    # 显示交互式图形（FAST_MODE下可跳过弹窗）
    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()
    else:
        print(f"[FAST_MODE] 可视化已保存，跳过弹窗显示")

    return fig


def visualize_multi_goal(grid, path, inspection_points, save_html=True,
                         html_filename="3d_multi_goal_visualization.html"):
    """
    可视化多目标巡检路径（增强版）

    该函数在visualize的基础上，额外标注巡检点位置和访问顺序。

    参数:
        grid: GridMap3D - 3D栅格地图对象
        path: list - 完整巡检路径
        inspection_points: list - 巡检点列表（物理坐标）
        save_html: bool - 是否将可视化保存为HTML文件
        html_filename: str - 保存的HTML文件名
    """
    fig = go.Figure()

    res = grid.resolution

    # 绘制建筑物
    for wall in grid.json_world.get("walls", []):
        x0, y0, z0 = wall["plane"]["start"]
        x1, y1, z1 = wall["plane"]["stop"]
        draw_box(fig, x0, x1, y0, y1, z0, z1, color='rgba(220, 50, 50, 0.4)')

    # 绘制路径
    if path:
        px = [grid.b_min[0] + p[0] * res for p in path]
        py = [grid.b_min[1] + p[1] * res for p in path]
        pz = [grid.b_min[2] + p[2] * res for p in path]

        # 路径带颜色渐变
        fig.add_trace(go.Scatter3d(
            x=px, y=py, z=pz,
            mode='lines+markers',
            line=dict(
                width=6,
                color=px,
                colorscale='Plasma',
                showscale=False
            ),
            marker=dict(size=2, color=px, colorscale='Plasma', opacity=0.4),
            name=f'巡检路径 ({len(path)}点)',
            hovertemplate='X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 起点
        fig.add_trace(go.Scatter3d(
            x=[px[0]], y=[py[0]], z=[pz[0]],
            mode='markers',
            marker=dict(size=12, color='rgb(0, 200, 100)', symbol='diamond', line=dict(width=2, color='white')),
            name='起点'
        ))

        # 终点
        fig.add_trace(go.Scatter3d(
            x=[px[-1]], y=[py[-1]], z=[pz[-1]],
            mode='markers',
            marker=dict(size=12, color='rgb(220, 50, 50)', symbol='diamond', line=dict(width=2, color='white')),
            name='终点'
        ))

    # 绘制巡检点（带编号）
    if inspection_points:
        # 找到路径中最接近每个巡检点的位置
        for i, point in enumerate(inspection_points):
            ix, iy, iz = point[0], point[1], point[2]

            # 使用不同颜色区分巡检点
            colors = ['rgb(255, 100, 100)', 'rgb(100, 255, 100)', 'rgb(100, 100, 255)',
                     'rgb(255, 255, 100)', 'rgb(255, 100, 255)', 'rgb(100, 255, 255)']
            color = colors[i % len(colors)]

            fig.add_trace(go.Scatter3d(
                x=[ix], y=[iy], z=[iz],
                mode='markers+text',
                marker=dict(size=15, color=color, symbol='circle', line=dict(width=2, color='white')),
                text=[f'{i+1}'],
                textfont=dict(size=14, color='white'),
                textposition='top center',
                name=f'巡检点{i+1}',
                hovertemplate=f'巡检点{i+1}<br>X: {ix:.1f}m<br>Y: {iy:.1f}m<br>Z: {iz:.1f}m<extra></extra>'
            ))

    title_text = f"多目标巡检路径规划 ({len(inspection_points)}个巡检点)"
    if path:
        path_length = 0
        for i in range(1, len(path)):
            dx = (path[i][0] - path[i-1][0]) * res
            dy = (path[i][1] - path[i-1][1]) * res
            dz = (path[i][2] - path[i-1][2]) * res
            path_length += np.sqrt(dx**2 + dy**2 + dz**2)
        title_text += f"<br><sup>总长度: {path_length:.1f}m | 航点: {len(path)}</sup>"

    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(eye=dict(x=1.8, y=1.8, z=1.2)),
            xaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='X (米)'),
            yaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='Y (米)'),
            zaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='Z (米)')
        ),
        width=1200,
        height=900,
        title=dict(text=title_text, font=dict(size=16)),
        font=dict(family="Arial"),
        paper_bgcolor='white'
    )

    # 保存HTML文件
    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn', config={'displayModeBar': True, 'displaylogo': False})
        print(f"[INFO] 多目标巡检3D可视化已保存: {html_path}")

    # 显示交互式图形（FAST_MODE下可跳过弹窗）
    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()
    else:
        print(f"[FAST_MODE] 可视化已保存，跳过弹窗显示")

    return fig


def visualize_path_comparison(grid, paths, labels, colors, save_html=True,
                             html_filename="3d_path_comparison.html"):
    """
    可视化多条路径对比（用于对比不同算法或参数的结果）

    参数:
        grid: GridMap3D - 3D栅格地图对象
        paths: list - 路径列表 [[(x,y,z), ...], ...]
        labels: list - 路径标签列表
        colors: list - 路径颜色列表
        save_html: bool - 是否保存HTML
        html_filename: str - 保存文件名
    """
    fig = go.Figure()

    res = grid.resolution

    # 绘制建筑物（半透明）
    for wall in grid.json_world.get("walls", []):
        x0, y0, z0 = wall["plane"]["start"]
        x1, y1, z1 = wall["plane"]["stop"]
        draw_box(fig, x0, x1, y0, y1, z0, z1, color='rgba(200, 200, 200, 0.2)')

    # 绘制每条路径
    for path_idx, (path, label, color) in enumerate(zip(paths, labels, colors)):
        if path:
            px = [grid.b_min[0] + p[0] * res for p in path]
            py = [grid.b_min[1] + p[1] * res for p in path]
            pz = [grid.b_min[2] + p[2] * res for p in path]

            # 计算路径长度
            length = 0
            for i in range(1, len(path)):
                dx = (path[i][0] - path[i-1][0]) * res
                dy = (path[i][1] - path[i-1][1]) * res
                dz = (path[i][2] - path[i-1][2]) * res
                length += np.sqrt(dx**2 + dy**2 + dz**2)

            fig.add_trace(go.Scatter3d(
                x=px, y=py, z=pz,
                mode='lines',
                line=dict(width=6, color=color),
                name=f'{label} ({length:.1f}m, {len(path)}点)',
                hovertemplate=f'{label}<br>X: %{{x:.1f}}m<br>Y: %{{y:.1f}}m<br>Z: %{{z:.1f}}m<extra></extra>'
            ))

            # 标记起点（只显示一次）
            if path_idx == 0:
                fig.add_trace(go.Scatter3d(
                    x=[px[0]], y=[py[0]], z=[pz[0]],
                    mode='markers',
                    marker=dict(size=12, color='rgb(0, 200, 100)', symbol='diamond'),
                    name='起点', showlegend=False
                ))

    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(eye=dict(x=1.8, y=1.8, z=1.2)),
            xaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='X (米)'),
            yaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='Y (米)'),
            zaxis=dict(backgroundcolor="rgb(245, 245, 245)", gridcolor="white", title='Z (米)')
        ),
        width=1200,
        height=900,
        title="路径对比 - 不同算法/参数",
        paper_bgcolor='white'
    )

    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn')
        print(f"[INFO] 路径对比3D可视化已保存: {html_path}")

    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()

    return fig


def visualize_with_height_profile(grid, path, save_html=True,
                                  html_filename="3d_with_height_profile.html"):
    """
    创建双子图：3D路径 + 高度剖面图

    参数:
        grid: GridMap3D - 3D栅格地图对象
        path: list - 路径点列表
        save_html: bool - 是否保存HTML
        html_filename: str - 保存文件名
    """
    # 创建双子图
    fig = make_subplots(
        rows=1, cols=2,
        column_widths=[0.6, 0.4],
        specs=[[{'type': 'scatter3d'}, {'type': 'scatter'}]],
        horizontal_spacing=0.05
    )

    res = grid.resolution

    # ===== 左图：3D可视化 =====
    # 绘制障碍物
    for wall in grid.json_world.get("walls", []):
        x0, y0, z0 = wall["plane"]["start"]
        x1, y1, z1 = wall["plane"]["stop"]
        draw_box(fig, x0, x1, y0, y1, z0, z1, color='rgba(220, 50, 50, 0.3)')

    # 绘制3D路径
    if path:
        px = [grid.b_min[0] + p[0] * res for p in path]
        py = [grid.b_min[1] + p[1] * res for p in path]
        pz = [grid.b_min[2] + p[2] * res for p in path]

        fig.add_trace(
            go.Scatter3d(
                x=px, y=py, z=pz,
                mode='lines',
                line=dict(width=6, color='royalblue'),
                name='3D路径'
            ),
            row=1, col=1
        )

        fig.add_trace(
            go.Scatter3d(
                x=[px[0]], y=[py[0]], z=[pz[0]],
                mode='markers',
                marker=dict(size=10, color='green'),
                name='起点',
                showlegend=False
            ),
            row=1, col=1
        )

        fig.add_trace(
            go.Scatter3d(
                x=[px[-1]], y=[py[-1]], z=[pz[-1]],
                mode='markers',
                marker=dict(size=10, color='red'),
                name='终点',
                showlegend=False
            ),
            row=1, col=1
        )

        # ===== 右图：高度剖面 =====
        # 计算累积距离
        distances = [0]
        for i in range(1, len(path)):
            dx = (path[i][0] - path[i-1][0]) * res
            dy = (path[i][1] - path[i-1][1]) * res
            dz = (path[i][2] - path[i-1][2]) * res
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            distances.append(distances[-1] + dist)

        fig.add_trace(
            go.Scatter(
                x=distances,
                y=pz,
                mode='lines+markers',
                line=dict(color='royalblue', width=2),
                marker=dict(size=4),
                name='高度剖面',
                fill='tozeroy',
                fillcolor='rgba(100, 150, 255, 0.2)'
            ),
            row=1, col=2
        )

        fig.add_trace(
            go.Scatter(
                x=[distances[0]],
                y=[pz[0]],
                mode='markers',
                marker=dict(size=12, color='green'),
                name='起点',
                showlegend=False
            ),
            row=1, col=2
        )

        fig.add_trace(
            go.Scatter(
                x=[distances[-1]],
                y=[pz[-1]],
                mode='markers',
                marker=dict(size=12, color='red'),
                name='终点',
                showlegend=False
            ),
            row=1, col=2
        )

    # 更新布局
    fig.update_layout(
        title_text="3D路径 + 高度剖面",
        paper_bgcolor='white',
        height=600,
        width=1400
    )

    fig.update_scenes(
        xaxis=dict(title='X (米)'),
        yaxis=dict(title='Y (米)'),
        zaxis=dict(title='Z (米)'),
        aspectmode='data',
        camera=dict(eye=dict(x=1.5, y=1.5, z=1.2)),
        row=1, col=1
    )

    fig.update_xaxes(title_text="距离 (米)", row=1, col=2)
    fig.update_yaxes(title_text="高度 (米)", row=1, col=2)

    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn')
        print(f"[INFO] 3D+高度剖面可视化已保存: {html_path}")

    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()

    return fig


def create_2d_top_view(grid, path, save_html=True, html_filename="2d_top_view.html"):
    """
    创建2D俯视图（投影到XY平面）

    参数:
        grid: GridMap3D - 3D栅格地图对象
        path: list - 路径点列表
        save_html: bool - 是否保存HTML
        html_filename: str - 保存文件名
    """
    fig = go.Figure()

    res = grid.resolution

    # 绘制障碍物的俯视图轮廓
    for wall in grid.json_world.get("walls", []):
        x0, y0, z0 = wall["plane"]["start"]
        x1, y1, z1 = wall["plane"]["stop"]

        # 创建障碍物的底面矩形
        obs_x = [x0, x1, x1, x0, x0]
        obs_y = [y0, y0, y1, y1, y0]

        fig.add_trace(go.Scatter(
            x=obs_x, y=obs_y,
            mode='lines',
            line=dict(color='red', width=1),
            fill='toself',
            fillcolor='rgba(255, 100, 100, 0.3)',
            showlegend=False,
            hoverinfo='skip'
        ))

    # 绘制路径（俯视图）
    if path:
        px = [grid.b_min[0] + p[0] * res for p in path]
        py = [grid.b_min[1] + p[1] * res for p in path]

        # 路径颜色基于高度
        pz = [grid.b_min[2] + p[2] * res for p in path]

        fig.add_trace(go.Scatter(
            x=px, y=py,
            mode='lines+markers',
            line=dict(width=4, color='royalblue'),
            marker=dict(
                size=5,
                color=pz,
                colorscale='Viridis',
                showscale=True,
                colorbar=dict(title='高度 (米)', x=1.1)
            ),
            name='路径',
            hovertemplate='X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{marker.color:.1f}m<extra></extra>'
        ))

        fig.add_trace(go.Scatter(
            x=[px[0]], y=[py[0]],
            mode='markers',
            marker=dict(size=15, color='green', symbol='diamond'),
            name='起点'
        ))

        fig.add_trace(go.Scatter(
            x=[px[-1]], y=[py[-1]],
            mode='markers',
            marker=dict(size=15, color='red', symbol='diamond'),
            name='终点'
        ))

    fig.update_layout(
        title="2D俯视图 - 路径投影",
        xaxis_title='X (米)',
        yaxis_title='Y (米)',
        width=900,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='rgb(245, 245, 245)',
        hovermode='closest'
    )

    # 保持等比例
    fig.update_yaxes(scaleanchor="x", scaleratio=1)

    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn')
        print(f"[INFO] 2D俯视图已保存: {html_path}")

    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()

    return fig


def visualize_powerline_inspection(grid, path_3d, waypoints, powerline_pixels=None,
                                   save_html=True, html_filename="powerline_inspection_3d.html"):
    """
    电网巡检专用3D可视化

    显示:
    1. 地形表面 (terrain surface)
    2. 原始电网 (红色线)
    3. 巡检路径 (蓝色线)
    4. 航点 (绿色点)

    Args:
        grid: HeightMap3D - 3D栅格地图对象
        path_3d: list - 3D路径 [(x, y, z), ...]
        waypoints: list - 航点列表 [(x, y), ...] 2D坐标
        powerline_pixels: list - 原始电网像素坐标 [(x, y), ...] (可选)
        save_html: bool - 是否保存HTML
        html_filename: str - 保存文件名
    """
    fig = go.Figure()

    res = grid.resolution

    # ===== 1. 绘制地形表面 =====
    if hasattr(grid, 'height_map'):
        h, w = grid.height_map.shape
        x_coords = np.arange(w) * res + grid.b_min[0]
        y_coords = np.arange(h) * res + grid.b_min[1]
        X, Y = np.meshgrid(x_coords, y_coords)
        Z = grid.height_map

        fig.add_trace(go.Surface(
            x=X,
            y=Y,
            z=Z,
            colorscale='Earth',
            colorbar=dict(title='地形高度 (m)', x=1.02),
            opacity=0.9,
            name='地形',
            showscale=True,
            hovertemplate='X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # ===== 2. 绘制原始电网 (红色线) =====
    if powerline_pixels:
        # 将2D像素坐标转换为3D世界坐标
        pl_x = [p[0] * res + grid.b_min[0] for p in powerline_pixels]
        pl_y = [p[1] * res + grid.b_min[1] for p in powerline_pixels]
        # 电网在地形上方，使用地形高度+小偏移
        pl_z = []
        for x, y in powerline_pixels:
            h_idx, w_idx = int(y), int(x)
            if 0 <= h_idx < h and 0 <= w_idx < w:
                terrain_z = grid.height_map[h_idx, w_idx]
            else:
                terrain_z = 0
            pl_z.append(terrain_z + 2)  # 电网离地2米

        fig.add_trace(go.Scatter3d(
            x=pl_x,
            y=pl_y,
            z=pl_z,
            mode='lines',
            line=dict(width=3, color='rgb(255, 50, 50)'),
            name='原始电网',
            hovertemplate='电网<br>X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # ===== 3. 绘制巡检路径 (蓝色线) =====
    if path_3d:
        path_x = [p[0] * res + grid.b_min[0] for p in path_3d]
        path_y = [p[1] * res + grid.b_min[1] for p in path_3d]
        path_z = [p[2] for p in path_3d]  # 已经是世界坐标

        fig.add_trace(go.Scatter3d(
            x=path_x,
            y=path_y,
            z=path_z,
            mode='lines',
            line=dict(width=5, color='rgb(50, 100, 255)'),
            name=f'巡检路径 ({len(path_3d)}点)',
            hovertemplate='路径<br>X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 起点标记
        fig.add_trace(go.Scatter3d(
            x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
            mode='markers',
            marker=dict(size=8, color='rgb(0, 200, 100)', symbol='diamond'),
            name='起点'
        ))

        # 终点标记
        fig.add_trace(go.Scatter3d(
            x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
            mode='markers',
            marker=dict(size=8, color='rgb(255, 100, 0)', symbol='diamond'),
            name='终点'
        ))

    # ===== 4. 绘制航点 (绿色点) =====
    if waypoints:
        wp_x = [wp[0] * res + grid.b_min[0] for wp in waypoints]
        wp_y = [wp[1] * res + grid.b_min[1] for wp in waypoints]
        # 航点高度从路径中插值或使用默认高度
        wp_z = []
        for i, (x, y) in enumerate(waypoints):
            # 找到路径中最接近的点
            if path_3d:
                min_dist = float('inf')
                closest_z = grid.b_min[2] + 20  # 默认高度
                for p in path_3d:
                    dist = (p[0] - x)**2 + (p[1] - y)**2
                    if dist < min_dist:
                        min_dist = dist
                        closest_z = p[2]
                wp_z.append(closest_z)
            else:
                wp_z.append(grid.b_min[2] + 20)

        fig.add_trace(go.Scatter3d(
            x=wp_x,
            y=wp_y,
            z=wp_z,
            mode='markers',
            marker=dict(
                size=4,
                color='rgb(0, 200, 0)',
                symbol='circle',
                line=dict(width=1, color='white')
            ),
            name=f'航点 ({len(waypoints)}个)',
            hovertemplate='航点<br>X: %{x:.1f}m<br>Y: %{y:.1f}m<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # 计算路径统计
    path_length = 0
    if path_3d:
        for i in range(1, len(path_3d)):
            dx = (path_3d[i][0] - path_3d[i-1][0]) * res
            dy = (path_3d[i][1] - path_3d[i-1][1]) * res
            dz = (path_3d[i][2] - path_3d[i-1][2])
            path_length += np.sqrt(dx**2 + dy**2 + dz**2)

        min_z = min(p[2] for p in path_3d)
        max_z = max(p[2] for p in path_3d)

        title_text = (f"UAV电网巡检路径规划<br>"
                     f"<sup>路径长度: {path_length:.1f}m | 航点: {len(path_3d)} | "
                     f"飞行高度: {min_z:.1f}-{max_z:.1f}m</sup>")
    else:
        title_text = "UAV电网巡检路径规划"

    # 设置场景布局
    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.3, y=1.3, z=0.9),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='X (m)', font=dict(size=14))
            ),
            yaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='Y (m)', font=dict(size=14))
            ),
            zaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='Z (m)', font=dict(size=14))
            )
        ),
        width=1200,
        height=900,
        margin=dict(l=0, r=0, t=50, b=0),
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.8)"
        ),
        title=dict(
            text=title_text,
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        font=dict(family="Arial, sans-serif"),
        paper_bgcolor='white'
    )

    # 保存HTML
    if save_html:
        os.makedirs(FIGURES_DIR, exist_ok=True)
        html_path = os.path.join(FIGURES_DIR, html_filename)
        fig.write_html(html_path, include_plotlyjs='cdn',
                      config={'displayModeBar': True, 'displaylogo': False})
        print(f"[INFO] 电网巡检3D可视化已保存: {html_path}")

    # 显示
    if not (FAST_MODE and FAST_MODE_NO_POPUP):
        fig.show()
    else:
        print(f"[FAST_MODE] 可视化已保存，跳过弹窗显示")

    return fig


__all__ = [
    'visualize',
    'visualize_multi_goal',
    'visualize_path_comparison',
    'visualize_with_height_profile',
    'create_2d_top_view',
    'visualize_powerline_inspection'
]
