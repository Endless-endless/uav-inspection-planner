"""
=====================================================
可视化模块包 (Visualization Package)
=====================================================
该包包含3D路径规划结果的可视化功能。
"""

from .plotly_viewer import (
    visualize,
    visualize_multi_goal
)
from .matplotlib_plots import (
    plot_resolution_experiment,
    plot_weighted_astar_experiment,
    plot_turn_penalty_experiment
)
from .map_overlay import (
    plot_path_on_real_map,
    plot_path_on_real_map_with_coords,
    plot_multiple_paths_on_map,
    create_animation_from_paths,
    overlay_path,
    overlay_path_with_height_profile
)
from .map_3d_overlay import plot_3d_path_on_map

__all__ = [
    # 3D 可视化
    'visualize',
    'visualize_multi_goal',
    # 实验图表
    'plot_resolution_experiment',
    'plot_weighted_astar_experiment',
    'plot_turn_penalty_experiment',
    # 2D地图叠加
    'plot_path_on_real_map',
    'plot_path_on_real_map_with_coords',
    'plot_multiple_paths_on_map',
    'create_animation_from_paths',
    'overlay_path',
    'overlay_path_with_height_profile',
    # 3D地图叠加
    'plot_3d_path_on_map'
]
