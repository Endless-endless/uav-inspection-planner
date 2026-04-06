"""
=====================================================
地图路径叠加显示模块 (Map Overlay Visualization)
=====================================================
功能说明:
    将 UAV 路径规划结果叠加显示在真实地图图片上。
    支持 2D 投影显示，忽略高度信息。

主要功能:
    - 在地图背景上绘制路径
    - 标记起点和终点
    - 支持路径平滑
    - 坐标自动转换

坐标转换说明:
    - 路径坐标: (x, y, z) - 栅格索引
    - 图像坐标: (row, col) - 像素索引
    - 转换关系: col=x, row=y (图像y轴向下)
=====================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from typing import Tuple, List, Optional
from PIL import Image


def plot_path_on_real_map(
    map_image_path: str,
    path: List[Tuple[int, int, int]],
    resolution: float = 1.0,
    output_path: str = "path_on_map.png",
    start_marker: bool = True,
    end_marker: bool = True,
    path_color: str = 'red',
    path_width: float = 2.5,
    smooth_path: bool = False,
    show_grid: bool = False,
    dpi: int = 150,
    figsize: Optional[Tuple[int, int]] = None
) -> str:
    """
    将路径叠加显示在真实地图上

    参数:
        map_image_path: 地图图片路径
        path: 路径点列表 [(x, y, z), ...]，栅格索引
        resolution: 栅格分辨率（米/栅格），用于计算距离
        output_path: 输出图片路径
        start_marker: 是否显示起点标记
        end_marker: 是否显示终点标记
        path_color: 路径颜色
        path_width: 路径线宽
        smooth_path: 是否平滑路径（使用样条插值）
        show_grid: 是否显示网格
        dpi: 输出图片分辨率
        figsize: 图片尺寸 (width, height)，None则自动

    返回:
        str: 保存的图片路径

    坐标转换说明:
        路径坐标 (x, y, z) → 图像坐标 (col, row)
        - col = x (图像x轴向右)
        - row = height - y (图像y轴向下，需要翻转)

        或者使用 matplotlib 的 imshow(origin='upper')，
        则: row = y, col = x (直接映射)
    """
    # 加载地图图片
    print(f"[INFO] 加载地图: {map_image_path}")
    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)

    img_height, img_width = map_array.shape[:2]
    print(f"[INFO] 地图尺寸: {img_width} x {img_height}")

    # 设置图片大小
    if figsize is None:
        # 根据地图比例设置
        aspect = img_width / img_height
        if aspect > 1:
            figsize = (12, 12 / aspect)
        else:
            figsize = (12 * aspect, 12)

    # 创建图形
    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    # 显示地图背景
    ax.imshow(map_array, origin='upper')

    # 提取路径坐标（忽略z，只取x, y）
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        path_z = [p[2] for p in path]

        # 计算路径统计
        from scipy.interpolate import splprep, splev
        if smooth_path and len(path) > 3:
            # 使用B样条平滑路径
            try:
                # 参数化
                t = np.linspace(0, 1, len(path_x))

                # 样条拟合
                tck, u = splprep([path_x, path_y], s=0, k=min(3, len(path) - 1))
                unew = np.linspace(0, 1, len(path_x) * 3)
                smooth_x, smooth_y = splev(unew, tck)

                plot_x = smooth_x
                plot_y = smooth_y
                print(f"[INFO] 路径已平滑: {len(path)} → {len(plot_x)} 点")
            except Exception as e:
                print(f"[WARNING] 平滑失败: {e}，使用原始路径")
                plot_x = path_x
                plot_y = path_y
        else:
            plot_x = path_x
            plot_y = path_y

        # 绘制路径
        # 注意：plt.plot的参数顺序是(x, y)
        # 但imshow的坐标系中，x是横轴(col)，y是纵轴(row)
        # 由于我们设置了origin='upper'，y轴向下，所以直接映射即可
        ax.plot(plot_x, plot_y,
                color=path_color,
                linewidth=path_width,
                alpha=0.8,
                label='规划路径')

        # 绘制路径方向箭头（每隔一段距离）
        arrow_interval = max(1, len(path) // 20)
        for i in range(0, len(path) - 1, arrow_interval):
            x1, y1 = path[i][0], path[i][1]
            x2, y2 = path[i + 1][0], path[i + 1][1]
            dx, dy = x2 - x1, y2 - y1
            if abs(dx) > 0.1 or abs(dy) > 0.1:  # 避免零长度箭头
                ax.arrow(x1, y1, dx * 0.3, dy * 0.3,
                        head_width=2, head_length=2,
                        fc=path_color, ec=path_color, alpha=0.5)

        # 绘制起点
        if start_marker:
            start_x, start_y = path[0][0], path[0][1]
            ax.plot(start_x, start_y, 'go',
                   markersize=12,
                   markeredgecolor='white',
                   markeredgewidth=2,
                   label='起点',
                   zorder=5)

        # 绘制终点
        if end_marker:
            end_x, end_y = path[-1][0], path[-1][1]
            ax.plot(end_x, end_y, 'bo',
                   markersize=12,
                   markeredgecolor='white',
                   markeredgewidth=2,
                   label='终点',
                   zorder=5)

        # 路径统计信息
        if resolution > 0:
            # 计算路径长度（米）
            length = 0
            for i in range(len(path) - 1):
                dx = (path[i+1][0] - path[i][0]) * resolution
                dy = (path[i+1][1] - path[i][1]) * resolution
                dz = (path[i+1][2] - path[i][2]) * resolution
                length += np.sqrt(dx**2 + dy**2 + dz**2)

            # 计算平均高度
            avg_height = np.mean(path_z) * resolution

            # 显示统计信息
            info_text = f'路径长度: {length:.1f}m\n'
            info_text += f'航点数: {len(path)}\n'
            info_text += f'平均高度: {avg_height:.1f}m\n'
            info_text += f'分辨率: {resolution}m/栅格'

            # 在图上添加文本框
            props = dict(boxstyle='round', facecolor='white', alpha=0.8)
            ax.text(0.02, 0.98, info_text,
                   transform=ax.transAxes,
                   fontsize=10,
                   verticalalignment='top',
                   bbox=props)

    # 显示网格（可选）
    if show_grid:
        ax.grid(True, alpha=0.3, linestyle='--')

    # 设置坐标轴
    ax.set_xlabel('X (栅格索引)', fontsize=12)
    ax.set_ylabel('Y (栅格索引)', fontsize=12)
    ax.set_title('UAV 路径规划结果 - 地图叠加', fontsize=14, fontweight='bold')

    # 添加图例
    if path and (start_marker or end_marker):
        ax.legend(loc='upper right', fontsize=10)

    # 设置坐标轴范围（确保路径可见）
    if path:
        all_x = [p[0] for p in path]
        all_y = [p[1] for p in path]
        ax.set_xlim(min(all_x) - 5, max(all_x) + 5)
        ax.set_ylim(min(all_y) - 5, max(all_y) + 5)

    plt.tight_layout()

    # 保存图片
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight')
    print(f"[INFO] 结果已保存: {output_path}")

    plt.close()

    return output_path


def plot_path_on_real_map_with_coords(
    map_image_path: str,
    path: List[Tuple[int, int, int]],
    b_min: Tuple[float, float, float],
    resolution: float = 1.0,
    output_path: str = "path_on_map_coords.png",
    show_coords: bool = True,
    **kwargs
) -> str:
    """
    将路径叠加显示在真实地图上（带物理坐标标注）

    参数:
        map_image_path: 地图图片路径
        path: 路径点列表 [(x, y, z), ...]，栅格索引
        b_min: 边界最小值 (x_min, y_min, z_min)，单位：米
        resolution: 栅格分辨率
        output_path: 输出图片路径
        show_coords: 是否显示物理坐标刻度
        **kwargs: 传递给 plot_path_on_real_map 的其他参数

    返回:
        str: 保存的图片路径
    """
    import matplotlib.pyplot as plt
    from PIL import Image

    # 加载地图
    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)

    img_height, img_width = map_array.shape[:2]

    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)

    # 显示地图背景
    ax.imshow(map_array, origin='upper')

    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]

        # 绘制路径
        ax.plot(path_x, path_y,
                color='red',
                linewidth=2.5,
                alpha=0.8,
                label='规划路径')

        # 绘制起点和终点
        ax.plot(path_x[0], path_y[0], 'go',
               markersize=12,
               markeredgecolor='white',
               markeredgewidth=2,
               label='起点',
               zorder=5)
        ax.plot(path_x[-1], path_y[-1], 'bo',
               markersize=12,
               markeredgecolor='white',
               markeredgewidth=2,
               label='终点',
               zorder=5)

    # 设置坐标轴（显示物理坐标）
    if show_coords and b_min is not None:
        # 创建物理坐标刻度
        x_min, y_min, _ = b_min

        # 计算刻度位置和标签
        x_ticks = np.arange(0, img_width, max(1, img_width // 10))
        y_ticks = np.arange(0, img_height, max(1, img_height // 10))

        x_labels = [f"{x_min + x * resolution:.0f}" for x in x_ticks]
        y_labels = [f"{y_min + y * resolution:.0f}" for y in y_ticks]

        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.set_xticklabels(x_labels, rotation=45, ha='right')
        ax.set_yticklabels(y_labels)

        ax.set_xlabel('X (米)', fontsize=12)
        ax.set_ylabel('Y (米)', fontsize=12)
    else:
        ax.set_xlabel('X (栅格索引)', fontsize=12)
        ax.set_ylabel('Y (栅格索引)', fontsize=12)

    ax.set_title('UAV 路径规划结果 - 地图叠加（物理坐标）',
                fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 结果已保存: {output_path}")
    plt.close()

    return output_path


def plot_multiple_paths_on_map(
    map_image_path: str,
    paths: List[Tuple[List[Tuple[int, int, int]], str, str]],
    resolution: float = 1.0,
    output_path: str = "multiple_paths_on_map.png",
    **kwargs
) -> str:
    """
    在同一地图上绘制多条路径（用于对比）

    参数:
        map_image_path: 地图图片路径
        paths: 路径列表，每个元素为 (path, label, color)
            - path: 路径点列表
            - label: 路径标签
            - color: 路径颜色
        resolution: 栅格分辨率
        output_path: 输出图片路径
        **kwargs: 其他参数

    返回:
        str: 保存的图片路径

    示例:
        paths = [
            (path1, "A* (w=1.0)", "blue"),
            (path2, "A* (w=1.5)", "red"),
            (path3, "A* (w=2.0)", "green"),
        ]
        plot_multiple_paths_on_map("map.png", paths)
    """
    from PIL import Image

    # 加载地图
    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)

    img_height, img_width = map_array.shape[:2]

    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)

    # 显示地图背景
    ax.imshow(map_array, origin='upper')

    # 绘制每条路径
    for path, label, color in paths:
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]

            ax.plot(path_x, path_y,
                    color=color,
                    linewidth=2,
                    alpha=0.7,
                    label=label)

            # 标记起点
            ax.plot(path_x[0], path_y[0], 'o',
                   color=color,
                   markersize=8,
                   markeredgecolor='white',
                   markeredgewidth=1.5,
                   zorder=5)

    # 设置坐标轴
    ax.set_xlabel('X (栅格索引)', fontsize=12)
    ax.set_ylabel('Y (栅格索引)', fontsize=12)
    ax.set_title('多路径对比 - 地图叠加', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 结果已保存: {output_path}")
    plt.close()

    return output_path


def create_animation_from_paths(
    map_image_path: str,
    path: List[Tuple[int, int, int]],
    output_path: str = "path_animation.gif",
    resolution: float = 1.0,
    interval: int = 100,
    marker_size: int = 15,
    fps: int = 10
) -> str:
    """
    创建路径动画（UAV沿路径移动）

    参数:
        map_image_path: 地图图片路径
        path: 路径点列表
        output_path: 输出GIF路径
        resolution: 栅格分辨率
        interval: 帧间隔（毫秒）
        marker_size: 标记大小
        fps: 帧率

    返回:
        str: 保存的GIF路径
    """
    try:
        from PIL import Image
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("[ERROR] 需要安装 matplotlib 和 Pillow")
        return None

    # 加载地图
    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)

    # 创建图形
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(map_array, origin='upper')

    # 初始化标记
    marker, = ax.plot([], [], 'ro',
                     markersize=marker_size,
                     markeredgecolor='white',
                     markeredgewidth=2,
                     zorder=5)

    # 绘制完整路径（半透明）
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'b--',
               linewidth=1, alpha=0.3)

        # 标记起点和终点
        ax.plot(path_x[0], path_y[0], 'go',
               markersize=10, label='起点')
        ax.plot(path_x[-1], path_y[-1], 'bo',
               markersize=10, label='终点')

    ax.set_title('UAV 路径动画', fontsize=14, fontweight='bold')
    ax.legend()

    def init():
        marker.set_data([], [])
        return marker,

    def update(frame):
        if frame < len(path):
            x, y = path[frame][0], path[frame][1]
            marker.set_data([x], [y])
        return marker,

    # 创建动画
    anim = FuncAnimation(fig, update,
                        frames=len(path),
                        init_func=init,
                        blit=True,
                        interval=interval)

    # 保存为GIF
    anim.save(output_path, writer='pillow', fps=fps)
    print(f"[INFO] 动画已保存: {output_path}")
    plt.close()

    return output_path


# =====================================================
# 便捷函数
# =====================================================

def overlay_path(map_path: str, path: List[tuple], **kwargs):
    """简洁的路径叠加函数"""
    return plot_path_on_real_map(map_path, path, **kwargs)


def overlay_path_with_height_profile(
    map_image_path: str,
    path: List[Tuple[int, int, int]],
    resolution: float = 1.0,
    output_path: str = "path_with_height.png"
) -> str:
    """
    地图叠加 + 高度剖面图（双子图）

    参数:
        map_image_path: 地图图片路径
        path: 路径点列表
        resolution: 栅格分辨率
        output_path: 输出图片路径

    返回:
        str: 保存的图片路径
    """
    from PIL import Image

    # 加载地图
    map_img = Image.open(map_image_path)
    map_array = np.array(map_img)

    # 创建双子图
    fig = plt.figure(figsize=(14, 6))

    # 左图：地图叠加
    ax1 = fig.add_subplot(121)
    ax1.imshow(map_array, origin='upper')

    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        path_z = [p[2] * resolution for p in path]  # 转换为米

        ax1.plot(path_x, path_y, 'r-', linewidth=2, alpha=0.8, label='路径')
        ax1.plot(path_x[0], path_y[0], 'go', markersize=10, label='起点')
        ax1.plot(path_x[-1], path_y[-1], 'bo', markersize=10, label='终点')

    ax1.set_xlabel('X (栅格)', fontsize=11)
    ax1.set_ylabel('Y (栅格)', fontsize=11)
    ax1.set_title('路径俯视图', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.2)

    # 右图：高度剖面
    ax2 = fig.add_subplot(122)

    if path:
        distances = [0]
        for i in range(1, len(path)):
            dx = (path[i][0] - path[i-1][0]) * resolution
            dy = (path[i][1] - path[i-1][1]) * resolution
            dz = (path[i][2] - path[i-1][2]) * resolution
            dist = np.sqrt(dx**2 + dy**2)
            distances.append(distances[-1] + dist)

        ax2.plot(distances, path_z, 'b-', linewidth=2)
        ax2.fill_between(distances, path_z, alpha=0.3)
        ax2.scatter([distances[0]], [path_z[0]], c='green', s=100, zorder=5, label='起点')
        ax2.scatter([distances[-1]], [path_z[-1]], c='blue', s=100, zorder=5, label='终点')

    ax2.set_xlabel('距离 (米)', fontsize=11)
    ax2.set_ylabel('高度 (米)', fontsize=11)
    ax2.set_title('高度剖面', fontsize=12, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 结果已保存: {output_path}")
    plt.close()

    return output_path


__all__ = [
    'plot_path_on_real_map',
    'plot_path_on_real_map_with_coords',
    'plot_multiple_paths_on_map',
    'create_animation_from_paths',
    'overlay_path',
    'overlay_path_with_height_profile'
]
