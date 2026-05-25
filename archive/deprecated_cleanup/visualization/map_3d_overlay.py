"""
=====================================================
3D路径叠加到真实地图
=====================================================
核心功能：将3D路径显示在真实地图背景上
=====================================================
"""

import numpy as np
import plotly.graph_objects as go
from PIL import Image
import os


def plot_3d_path_on_map(
    map_image_path,
    path_3d,
    map_bounds,
    output_path="figures/3d_on_map.html"
):
    """
    将3D路径叠加到真实地图上

    参数:
        map_image_path: 地图图片路径
        path_3d: 路径点列表 [(x, y, z), ...]，单位：米
        map_bounds: 地图覆盖的物理范围 (min_x, max_x, min_y, max_y)，单位：米
        output_path: 输出HTML路径
    """
    # 加载地图
    img = Image.open(map_image_path)
    img_array = np.array(img)
    height, width = img_array.shape[:2]

    min_x, max_x, min_y, max_y = map_bounds

    # 创建图形
    fig = go.Figure()

    # 地面网格
    x_phys = np.linspace(min_x, max_x, width)
    y_phys = np.linspace(min_y, max_y, height)
    X, Y = np.meshgrid(x_phys, y_phys)
    Z = np.zeros_like(X)

    # 添加地图纹理
    fig.add_trace(go.Surface(
        x=X, y=Y, z=Z,
        surfacecolor=np.flipud(img_array),
        colorscale='Viridis',
        showscale=False,
        opacity=0.9,
        name='地图'
    ))

    # 绘制路径
    if path_3d:
        px = [p[0] for p in path_3d]
        py = [p[1] for p in path_3d]
        pz = [p[2] for p in path_3d]

        # 路径线
        fig.add_trace(go.Scatter3d(
            x=px, y=py, z=pz,
            mode='lines',
            line=dict(width=5, color='blue'),
            name='路径'
        ))

        # 起点
        fig.add_trace(go.Scatter3d(
            x=[px[0]], y=[py[0]], z=[pz[0]],
            mode='markers',
            marker=dict(size=12, color='green', symbol='diamond'),
            name='起点'
        ))

        # 终点
        fig.add_trace(go.Scatter3d(
            x=[px[-1]], y=[py[-1]], z=[pz[-1]],
            mode='markers',
            marker=dict(size=12, color='red', symbol='diamond'),
            name='终点'
        ))

    # 布局
    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.2)),
            xaxis=dict(title='X (米)'),
            yaxis=dict(title='Y (米)'),
            zaxis=dict(title='Z (米)')
        ),
        width=1200,
        height=900,
        title='3D路径 - 叠加真实地图',
        paper_bgcolor='white'
    )

    # 保存
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    fig.write_html(output_path)

    return fig
