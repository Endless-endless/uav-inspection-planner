"""
=====================================================
分析模块包 (Analysis Package)
=====================================================
该包包含路径性能评估和分析的工具函数。
"""

from .metrics import (
    compute_path_length,
    compute_turn_angle,
    compute_average_turn_angle,
    enforce_turn_constraint
)

__all__ = [
    'compute_path_length',
    'compute_turn_angle',
    'compute_average_turn_angle',
    'enforce_turn_constraint'
]
