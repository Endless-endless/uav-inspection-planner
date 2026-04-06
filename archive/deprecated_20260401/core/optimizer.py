"""
风影响优化模块

功能：基于风向和风速进行路径优化
"""

import numpy as np


class WindOptimizer:
    """风影响路径优化器"""

    def __init__(self, wind_direction=0, wind_speed=5):
        """
        初始化风优化器

        Args:
            wind_direction: 风向角度（0-360°，0=东风）
            wind_speed: 风速（m/s）
        """
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed

    def compute_wind_cost(self, waypoints):
        """
        计算路径的风影响成本

        Args:
            waypoints: 航点列表 [(x, y), ...]

        Returns:
            list: 带风成本的航点 [(x, y, cost), ...]
        """
        if not waypoints:
            return []

        waypoints_with_cost = []
        total_cost = 0
        alpha = 0.05

        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]

            # 计算路径方向
            path_angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if path_angle < 0:
                path_angle += 360

            # 计算风向与路径方向的角度差（有向）
            angle_diff_signed = path_angle - self.wind_direction
            angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
            angle_rad = np.radians(angle_diff_signed)
            cos_theta = np.cos(angle_rad)

            # 强化风惩罚机制
            if cos_theta < 0:
                # 逆风：强惩罚
                segment_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
            else:
                # 顺风：奖励
                segment_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)

            waypoints_with_cost.append((x1, y1, segment_cost))
            total_cost += segment_cost

        # 添加最后一个点
        waypoints_with_cost.append((waypoints[-1][0], waypoints[-1][1], 1.0))

        return waypoints_with_cost

    def optimize_path_with_wind(self, waypoints, width, height, alpha=0.05, beta=0.1):
        """
        基于风的局部路径优化

        Args:
            waypoints: 原始航点 [(x, y), ...]
            width: 图像宽度
            height: 图像高度
            alpha: 风影响系数
            beta: 距离约束系数

        Returns:
            list: 优化后的航点
        """
        if not waypoints:
            return []

        optimized = []
        search_range = 5  # ±5像素

        for idx, (orig_x, orig_y) in enumerate(waypoints):
            # 起终点不优化
            if idx == 0 or idx == len(waypoints) - 1:
                clamped_x = max(0, min(int(orig_x), width - 1))
                clamped_y = max(0, min(int(orig_y), height - 1))
                optimized.append((clamped_x, clamped_y))
                continue

            # 检查原始位置是否在边界内
            if not (0 <= orig_x < width and 0 <= orig_y < height):
                clamped_x = max(0, min(int(orig_x), width - 1))
                clamped_y = max(0, min(int(orig_y), height - 1))
                optimized.append((clamped_x, clamped_y))
                continue

            best_x, best_y = orig_x, orig_y
            best_cost = float('inf')

            # 计算原始位置的成本
            if idx < len(waypoints) - 1:
                next_x, next_y = waypoints[idx + 1]
                path_angle = np.degrees(np.arctan2(next_y - orig_y, next_x - orig_x))
                if path_angle < 0:
                    path_angle += 360
                angle_diff_signed = path_angle - self.wind_direction
                angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
                angle_rad = np.radians(angle_diff_signed)
                cos_theta = np.cos(angle_rad)

                if cos_theta < 0:
                    wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
                else:
                    wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)
                best_cost = 8 * wind_cost

            # 局部搜索
            for dx in range(-search_range, search_range + 1):
                for dy in range(-search_range, search_range + 1):
                    candidate_x = int(orig_x + dx)
                    candidate_y = int(orig_y + dy)

                    # 边界检查
                    if (candidate_x < 0 or candidate_x >= width or
                        candidate_y < 0 or candidate_y >= height):
                        continue

                    # 距离限制
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist > 8:
                        continue

                    dist_cost = beta * dist

                    # 风代价
                    if idx < len(waypoints) - 1:
                        next_x, next_y = waypoints[idx + 1]
                        path_angle = np.degrees(np.arctan2(next_y - candidate_y, next_x - candidate_x))
                        if path_angle < 0:
                            path_angle += 360

                        angle_diff_signed = path_angle - self.wind_direction
                        angle_diff_signed = (angle_diff_signed + 180) % 360 - 180
                        angle_rad = np.radians(angle_diff_signed)
                        cos_theta = np.cos(angle_rad)

                        if cos_theta < 0:
                            wind_cost = 1 + 2 * abs(cos_theta) * self.wind_speed / 10
                        else:
                            wind_cost = max(0.5, 1 - cos_theta * self.wind_speed / 10)

                        segment_cost = dist_cost + 8 * wind_cost

                        if segment_cost < best_cost:
                            best_cost = segment_cost
                            best_x, best_y = candidate_x, candidate_y

            optimized.append((best_x, best_y))

        return optimized
