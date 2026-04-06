"""
能耗评估模块

功能：计算无人机巡检路径的能耗成本
"""

import numpy as np


class EnergyCalculator:
    """能耗计算器"""

    @staticmethod
    def compute_energy_cost(path_3d, wind_speed):
        """
        计算无人机能耗成本

        能耗模型（简化版）：
        - 基础能耗：distance * (1 + 0.1 * wind_speed)
        - 爬升能耗：climb * 2.0

        Args:
            path_3d: 3D路径点 [(x, y, z), ...] 或 numpy数组
            wind_speed: 风速 (m/s)

        Returns:
            dict: {
                'total_energy': 总能耗,
                'avg_energy_per_meter': 每米平均能耗
            }
        """
        if path_3d is None or len(path_3d) < 2:
            return {'total_energy': 0, 'avg_energy_per_meter': 0}

        total_energy = 0
        total_distance = 0

        for i in range(len(path_3d) - 1):
            p1 = path_3d[i]
            p2 = path_3d[i + 1]

            # 计算水平距离
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = np.sqrt(dx**2 + dy**2)

            # 计算爬升高度（只计算正向爬升）
            climb = max(0, p2[2] - p1[2])

            # 能耗模型
            segment_energy = distance * (1 + 0.1 * wind_speed) + climb * 2.0

            total_energy += segment_energy
            total_distance += distance

        avg_energy_per_meter = total_energy / total_distance if total_distance > 0 else 0

        return {
            'total_energy': total_energy,
            'avg_energy_per_meter': avg_energy_per_meter
        }
