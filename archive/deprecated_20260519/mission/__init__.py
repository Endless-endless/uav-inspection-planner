"""
=====================================================
任务模块包 (Mission Package)
=====================================================
该包包含多目标任务规划的相关功能。
"""

from .multi_goal import (
    coord_to_index,
    raytrace,
    sparsen_path,
    multi_goal_plan,
    find_valid_inspection_point
)
from .tsp_solver import (
    multi_goal_plan_optimal,
    create_closed_loop,
    compute_total_path_length_with_return
)

__all__ = [
    'coord_to_index',
    'raytrace',
    'sparsen_path',
    'multi_goal_plan',
    'find_valid_inspection_point',
    'multi_goal_plan_optimal',
    'create_closed_loop',
    'compute_total_path_length_with_return'
]
