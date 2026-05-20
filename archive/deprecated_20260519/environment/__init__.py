"""
=====================================================
环境模块包 (Environment Package)
=====================================================
该包包含3D环境地图的构建和管理功能。

支持的地图类型：
- GridMap3D: 从JSON文件加载的建筑物障碍物地图
- HeightMap3D: 从灰度图生成的地形高度图
"""

from .grid_map import GridMap3D
from .heightmap_loader import HeightMap3D, load_heightmap, load_from_array, create_sample_heightmap

__all__ = [
    'GridMap3D',
    'HeightMap3D',
    'load_heightmap',
    'load_from_array',
    'create_sample_heightmap'
]
