"""
地形生成模块

功能：生成仿真地形用于路径规划测试
"""

import numpy as np
from scipy.ndimage import gaussian_filter


class TerrainGenerator:
    """地形生成器"""

    @staticmethod
    def generate_realistic_terrain(width, height, sigma=20, elevation_scale=60):
        """
        生成真实感地形

        Args:
            width: 地形宽度（像素）
            height: 地形高度（像素）
            sigma: 高斯模糊系数，控制地形平滑度
            elevation_scale: 高程缩放系数（米）

        Returns:
            np.ndarray: 地形高度图 (height, width)
        """
        np.random.seed(0)
        terrain = np.random.rand(height, width)
        terrain = gaussian_filter(terrain, sigma=sigma)
        terrain = (terrain - terrain.min()) / (terrain.max() - terrain.min()) * elevation_scale

        return terrain.astype(np.float32)

    @staticmethod
    def generate_flat_terrain(width, height, elevation=20):
        """
        生成平坦地形

        Args:
            width: 地形宽度
            height: 地形高度
            elevation: 恒定高程（米）

        Returns:
            np.ndarray: 平坦地形
        """
        return np.full((height, width), elevation, dtype=np.float32)
