"""
可视化模块包 — 当前主链仅使用 dashboard_map 与 demo 阶段的 map_overlay。
已归档模块见 archive/deprecated_cleanup/visualization/
"""

from .map_overlay import (
    plot_path_on_real_map,
    plot_path_on_real_map_with_coords,
    plot_multiple_paths_on_map,
    create_animation_from_paths,
    overlay_path,
    overlay_path_with_height_profile,
)

__all__ = [
    "plot_path_on_real_map",
    "plot_path_on_real_map_with_coords",
    "plot_multiple_paths_on_map",
    "create_animation_from_paths",
    "overlay_path",
    "overlay_path_with_height_profile",
]
