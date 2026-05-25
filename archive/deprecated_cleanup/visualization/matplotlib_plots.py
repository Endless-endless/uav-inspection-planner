"""
=====================================================
Matplotlib绘图模块 (Matplotlib Plots Module)
=====================================================
功能说明:
    该模块使用Matplotlib创建科研论文级别的图表。
    采用Nature期刊风格的绘图规范。

主要功能:
    - 分辨率实验结果图表
    - 加权A*对比实验图表
    - 多目标巡检结果图表

绘图风格:
    - Times New Roman字体
    - 去除上边框和右边框
    - 柔和的配色方案
    - 适当的网格线
=====================================================
"""

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from config.settings import get_figure_path


def setup_nature_style():
    """
    设置Nature期刊风格的绘图参数
    """
    # 字体设置：Times New Roman是学术论文的标准字体
    mpl.rcParams['font.family'] = 'Times New Roman'
    mpl.rcParams['font.size'] = 11
    mpl.rcParams['axes.labelsize'] = 11
    mpl.rcParams['axes.titlesize'] = 12
    mpl.rcParams['legend.fontsize'] = 10
    mpl.rcParams['xtick.labelsize'] = 10
    mpl.rcParams['ytick.labelsize'] = 10


def remove_top_right_spines(axes):
    """
    去除图表的上边框和右边框（Nature风格核心特征）

    参数:
        axes: matplotlib.axes.Axes, list, or numpy.ndarray - 单个轴对象、轴对象列表或numpy数组
    """
    # Handle numpy arrays by converting to flat list
    if isinstance(axes, np.ndarray):
        axes = axes.flatten()
    elif not isinstance(axes, (list, tuple)):
        axes = [axes]

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)


def plot_resolution_experiment(resolutions, time_list, expanded_list, path_lengths):
    """
    绘制分辨率对比实验结果

    创建一个1x3的子图，展示：
    1. 分辨率 vs 执行时间
    2. 分辨率 vs 扩展节点数
    3. 分辨率 vs 路径长度

    参数:
        resolutions: list - 分辨率列表
        time_list: list - 对应的执行时间列表
        expanded_list: list - 对应的扩展节点数列表
        path_lengths: list - 对应的路径长度列表

    返回:
        tuple - (fig, axes) matplotlib图形对象
    """
    # 设置Nature风格
    setup_nature_style()

    # Nature风格的冷色调配色
    color_time = "#4C72B0"   # 蓝色
    color_nodes = "#DD8452"  # 橙色
    color_length = "#55A868" # 绿色

    # 创建1x3子图
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    # 去除上边框和右边框
    remove_top_right_spines(axes)

    # -------- 执行时间图表 --------
    axes[0].plot(resolutions, time_list,
                 marker='o',
                 markersize=6,
                 linewidth=2,
                 color=color_time)

    axes[0].set_xlabel("Resolution (m)")
    axes[0].set_ylabel("Execution Time (s)")
    axes[0].set_title("Resolution vs Execution Time")
    axes[0].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- 扩展节点数图表 --------
    axes[1].plot(resolutions, expanded_list,
                 marker='s',
                 markersize=6,
                 linewidth=2,
                 color=color_nodes)

    axes[1].set_xlabel("Resolution (m)")
    axes[1].set_ylabel("Expanded Nodes")
    axes[1].set_title("Resolution vs Search Complexity")
    axes[1].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- 路径长度图表 --------
    axes[2].plot(resolutions, path_lengths,
                 marker='^',
                 markersize=6,
                 linewidth=2,
                 color=color_length)

    axes[2].set_xlabel("Resolution (m)")
    axes[2].set_ylabel("Path Length")
    axes[2].set_title("Resolution vs Path Quality")
    axes[2].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    plt.tight_layout()

    # 保存高质量图像
    for path in get_all_figure_paths('resolution_experiment_nature_style'):
        plt.savefig(path, dpi=600, bbox_inches='tight')

    return fig, axes


def plot_weighted_astar_experiment(weights, time_list, expanded_list, path_lengths):
    """
    绘制加权A*对比实验结果

    创建一个1x3的子图，展示：
    1. 权重 vs 执行时间
    2. 权重 vs 扩展节点数
    3. 权重 vs 路径长度

    参数:
        weights: list - 权重列表
        time_list: list - 对应的执行时间列表
        expanded_list: list - 对应的扩展节点数列表
        path_lengths: list - 对应的路径长度列表

    返回:
        tuple - (fig, axes) matplotlib图形对象
    """
    setup_nature_style()

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    remove_top_right_spines(axes)

    # 执行时间
    axes[0].plot(weights, time_list, marker='o', linewidth=2)
    axes[0].set_xlabel("Weight")
    axes[0].set_ylabel("Execution Time (s)")
    axes[0].set_title("Weight vs Execution Time")
    axes[0].grid(True, linestyle=':', alpha=0.6)

    # 扩展节点数
    axes[1].plot(weights, expanded_list, marker='s', linewidth=2)
    axes[1].set_xlabel("Weight")
    axes[1].set_ylabel("Expanded Nodes")
    axes[1].set_title("Weight vs Search Complexity")
    axes[1].grid(True, linestyle=':', alpha=0.6)

    # 路径长度
    axes[2].plot(weights, path_lengths, marker='^', linewidth=2)
    axes[2].set_xlabel("Weight")
    axes[2].set_ylabel("Path Length")
    axes[2].set_title("Weight vs Path Quality")
    axes[2].grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()

    # 保存图像
    for path in get_all_figure_paths('weighted_astar_nature_style'):
        plt.savefig(path, dpi=600, bbox_inches='tight')

    return fig, axes


def plot_turn_penalty_experiment(turn_weights, path_lengths, avg_angles, expanded_nodes, exec_times):
    """
    绘制转弯惩罚对比实验结果

    创建一个2x2的子图，展示：
    1. 转弯权重 vs 路径长度
    2. 转弯权重 vs 平均转角
    3. 转弯权重 vs 扩展节点数
    4. 转弯权重 vs 执行时间

    参数:
        turn_weights: list - 转弯权重列表
        path_lengths: list - 对应的路径长度列表
        avg_angles: list - 对应的平均转角列表
        expanded_nodes: list - 对应的扩展节点数列表
        exec_times: list - 对应的执行时间列表

    返回:
        tuple - (fig, axes) matplotlib图形对象
    """
    setup_nature_style()

    # Nature风格的配色方案
    color_length = "#4C72B0"   # 蓝色 - 路径长度
    color_angle = "#DD8452"    # 橙色 - 平均转角
    color_nodes = "#55A868"    # 绿色 - 扩展节点
    color_time = "#C44E52"     # 红色 - 执行时间

    # 创建2x2子图
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # 去除上边框和右边框
    remove_top_right_spines(axes)

    # -------- 路径长度图表 --------
    axes[0, 0].plot(turn_weights, path_lengths,
                    marker='o',
                    markersize=6,
                    linewidth=2,
                    color=color_length)

    axes[0, 0].set_xlabel("Turn Weight")
    axes[0, 0].set_ylabel("Path Length (m)")
    axes[0, 0].set_title("Turn Weight vs Path Length")
    axes[0, 0].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- 平均转角图表 --------
    axes[0, 1].plot(turn_weights, avg_angles,
                    marker='s',
                    markersize=6,
                    linewidth=2,
                    color=color_angle)

    axes[0, 1].set_xlabel("Turn Weight")
    axes[0, 1].set_ylabel("Average Turn Angle (degrees)")
    axes[0, 1].set_title("Turn Weight vs Path Smoothness")
    axes[0, 1].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- 扩展节点数图表 --------
    axes[1, 0].plot(turn_weights, expanded_nodes,
                    marker='^',
                    markersize=6,
                    linewidth=2,
                    color=color_nodes)

    axes[1, 0].set_xlabel("Turn Weight")
    axes[1, 0].set_ylabel("Expanded Nodes")
    axes[1, 0].set_title("Turn Weight vs Search Complexity")
    axes[1, 0].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- 执行时间图表 --------
    axes[1, 1].plot(turn_weights, exec_times,
                    marker='d',
                    markersize=6,
                    linewidth=2,
                    color=color_time)

    axes[1, 1].set_xlabel("Turn Weight")
    axes[1, 1].set_ylabel("Execution Time (s)")
    axes[1, 1].set_title("Turn Weight vs Execution Time")
    axes[1, 1].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    plt.tight_layout()

    # 保存高质量图像
    for path in get_all_figure_paths('turn_penalty_experiment'):
        plt.savefig(path, dpi=600, bbox_inches='tight')

    return fig, axes


def plot_weather_experiment(wind_speeds, time_list, expanded_list, path_lengths):
    """
    Plot weather impact experiment results.

    Creates a 1x3 subplot showing:
    1. Wind speed vs execution time
    2. Wind speed vs expanded nodes
    3. Wind speed vs path length

    Args:
        wind_speeds: list - Wind speed values in m/s
        time_list: list - Corresponding execution times
        expanded_list: list - Corresponding expanded node counts
        path_lengths: list - Corresponding path lengths

    Returns:
        tuple - (fig, axes) matplotlib figure objects
    """
    setup_nature_style()

    # Nature-style color palette
    color_time = "#4C72B0"   # Blue
    color_nodes = "#DD8452"  # Orange
    color_length = "#55A868" # Green

    # Create 1x3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    remove_top_right_spines(axes)

    # -------- Execution Time Plot --------
    axes[0].plot(wind_speeds, time_list,
                 marker='o',
                 markersize=6,
                 linewidth=2,
                 color=color_time)

    axes[0].set_xlabel("Wind Speed ||W|| (m/s)")
    axes[0].set_ylabel("Execution Time (s)")
    axes[0].set_title("Wind Speed vs Execution Time")
    axes[0].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- Expanded Nodes Plot --------
    axes[1].plot(wind_speeds, expanded_list,
                 marker='s',
                 markersize=6,
                 linewidth=2,
                 color=color_nodes)

    axes[1].set_xlabel("Wind Speed ||W|| (m/s)")
    axes[1].set_ylabel("Expanded Nodes")
    axes[1].set_title("Wind Speed vs Search Complexity")
    axes[1].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    # -------- Path Length Plot --------
    axes[2].plot(wind_speeds, path_lengths,
                 marker='^',
                 markersize=6,
                 linewidth=2,
                 color=color_length)

    axes[2].set_xlabel("Wind Speed ||W|| (m/s)")
    axes[2].set_ylabel("Path Length (m)")
    axes[2].set_title("Wind Speed vs Path Length")
    axes[2].grid(True, linestyle=':', linewidth=0.8, alpha=0.6)

    plt.tight_layout()

    # Save high-quality images
    for path in get_all_figure_paths('weather_experiment'):
        plt.savefig(path, dpi=600, bbox_inches='tight')

    return fig, axes


def get_all_figure_paths(basename):
    """辅助函数：生成所有格式的图形文件路径"""
    from config.settings import FIGURE_FORMATS, FIGURES_DIR
    import os

    paths = []
    for fmt in FIGURE_FORMATS:
        paths.append(os.path.join(FIGURES_DIR, f"{basename}.{fmt}"))
    return paths
