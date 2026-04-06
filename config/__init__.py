"""
=====================================================
配置模块包 (Configuration Package)
=====================================================
该包包含项目的所有配置参数。
"""

from .settings import (
    PROJECT_ROOT,
    MAP_FILE,
    DEFAULT_RESOLUTION,
    DEFAULT_WEIGHT,
    DEFAULT_TURN_WEIGHT,
    DEFAULT_MAX_TURN_ANGLE,
    START_COORD,
    GOAL_COORD,
    INSPECTION_COORDS,
    RESOLUTION_EXPERIMENT,
    WEIGHT_EXPERIMENT,
    TURN_WEIGHT_EXPERIMENT,
    FIGURES_DIR,
    FIGURE_DPI,
    FIGURE_FORMATS,
    get_map_path,
    get_figure_path,
    get_all_figure_paths
)

__all__ = [
    'PROJECT_ROOT',
    'MAP_FILE',
    'DEFAULT_RESOLUTION',
    'DEFAULT_WEIGHT',
    'DEFAULT_TURN_WEIGHT',
    'DEFAULT_MAX_TURN_ANGLE',
    'START_COORD',
    'GOAL_COORD',
    'INSPECTION_COORDS',
    'RESOLUTION_EXPERIMENT',
    'WEIGHT_EXPERIMENT',
    'TURN_WEIGHT_EXPERIMENT',
    'FIGURES_DIR',
    'FIGURE_DPI',
    'FIGURE_FORMATS',
    'get_map_path',
    'get_figure_path',
    'get_all_figure_paths'
]

