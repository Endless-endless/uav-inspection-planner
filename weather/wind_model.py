"""
=====================================================
Wind Field Model for UAV Path Planning
=====================================================

This module provides a wind field model for weather-aware UAV path planning.

Wind Cost Formulation:
----------------------
The weather-induced cost component C_w is computed as:

    C_w = α * ||W|| * (1 - cos(θ))

where:
    - α: Wind cost weight parameter
    - W: Wind vector at current position
    - θ: Angle between motion vector and wind vector
    - ||W||: Wind speed magnitude

Properties:
-----------
- C_w ≥ 0: Cost is always non-negative
- Headwind (θ ≈ π): Higher cost (adverse conditions)
- Tailwind (θ ≈ 0): Lower cost (favorable conditions)
- Crosswind (θ ≈ π/2): Moderate cost

References:
-----------
- McGee, T. G., & Hedrick, J. K. (2006). Path planning for UAVs in
  uncertain, adversarial environments.
"""

import numpy as np


class WindField:
    """
    Uniform Wind Field Model

    Represents a spatially uniform wind field characterized by a constant
    wind vector W = (w_x, w_y, w_z).

    The model can be extended to non-uniform fields by overriding
    get_wind_vector() to depend on position.
    """

    def __init__(self, wind_vector=(0.0, 0.0, 0.0)):
        """
        Initialize the wind field.

        Args:
            wind_vector: Wind vector W = (w_x, w_y, w_z) in m/s
        """
        self.wind_vector = np.array(wind_vector, dtype=float)

    def get_wind_vector(self, position=None):
        """
        Retrieve the wind vector at a specified position.

        The current implementation assumes a uniform wind field.
        Subclasses can override this method to implement spatially
        varying wind conditions.

        Args:
            position: 3D coordinates (x, y, z) in meters.
                Unused in uniform field model.

        Returns:
            np.ndarray: Wind vector W = (w_x, w_y, w_z) in m/s
        """
        return self.wind_vector

    def wind_speed(self):
        """
        Compute the magnitude of the wind velocity.

        Returns:
            float: Wind speed ||W|| in m/s
        """
        return np.linalg.norm(self.wind_vector)

    def wind_direction(self):
        """
        Compute the unit wind direction vector.

        Returns:
            np.ndarray: Unit wind vector Ŵ = W / ||W||.
                Returns zeros vector if wind speed is zero.
        """
        speed = self.wind_speed()
        if speed == 0:
            return np.zeros(3)
        return self.wind_vector / speed


class NonUniformWindField:
    """
    Non-Uniform Wind Field Model (Spatially Varying)

    实现空间变化的风场模型，不同区域具有不同的风速和风向。
    这将迫使路径规划算法选择绕开强风区域的路径。

    风场分区设计：
    - 左侧区域 (x < 0): 低风区（顺风/微风）
    - 中间区域 (0 ≤ x < 10): 中等风区
    - 右侧区域 (x ≥ 10): 高风区（强逆风）

    这种设计将诱导路径选择通过左侧区域，即使路径更长。
    """

    def __init__(self, base_wind_vector=(0.0, 0.0, 0.0), high_wind_multiplier=3.0):
        """
        Initialize the non-uniform wind field.

        Args:
            base_wind_vector: Base wind vector W = (w_x, w_y, w_z) in m/s
            high_wind_multiplier: Multiplier for high-wind region (default: 3.0)
                高风区的风速 = base_wind_speed * high_wind_multiplier
        """
        self.base_wind = np.array(base_wind_vector, dtype=float)
        self.high_wind_multiplier = high_wind_multiplier

    def get_wind_vector(self, position=None):
        """
        根据位置返回风向量（空间变化风场）

        风场分区策略：
        - x < 0:      低风区 (base_wind * 0.3) - 安静区域
        - 0 ≤ x < 10: 中风区 (base_wind * 1.0) - 标准区域
        - x ≥ 10:     高风区 (base_wind * 3.0) - 危险区域

        Args:
            position: 3D coordinates (x, y, z) in meters.
                如果为 None，返回基础风向量

        Returns:
            np.ndarray: Wind vector W = (w_x, w_y, w_z) in m/s
        """
        if position is None:
            return self.base_wind

        x, y, z = position

        # 根据x坐标决定风场强度
        if x < 0:
            # 低风区：微风
            multiplier = 0.3
        elif x < 10:
            # 中风区：正常风速
            multiplier = 1.0
        else:
            # 高风区：强风
            multiplier = self.high_wind_multiplier

        return self.base_wind * multiplier

    def wind_speed_at(self, position):
        """
        计算指定位置的风速

        Args:
            position: 3D coordinates (x, y, z) in meters

        Returns:
            float: Wind speed ||W|| in m/s at the specified position
        """
        wind = self.get_wind_vector(position)
        return np.linalg.norm(wind)


# =====================================================
# Wind Cost Function (C_w)
# =====================================================

def compute_wind_cost_weighted(move_vector, wind_vector, alpha=1.0, use_squared=False):
    """
    Compute weather-induced cost using WEIGHTED formulation (Legacy method).

    旧方法：需要手动调节权重 λ_w
    保留此函数用于向后兼容。

    Cost Function:
        C_w = α * ||W||^p * (1 - cos(θ))^q

    Args:
        move_vector: Motion vector V = (v_x, v_y, v_z)
        wind_vector: Wind vector W = (w_x, w_y, w_z) in m/s
        alpha: Wind cost weight parameter (default: 1.0)
        use_squared: If True, use squared form for stronger effect (default: False)

    Returns:
        float: Weather cost C_w ≥ 0
    """
    move_vector = np.array(move_vector, dtype=float)
    move_norm = np.linalg.norm(move_vector)
    wind_norm = np.linalg.norm(wind_vector)

    if move_norm == 0 or wind_norm == 0:
        return 0.0

    move_dir = move_vector / move_norm
    wind_dir = wind_vector / wind_norm
    cos_theta = np.dot(move_dir, wind_dir)

    if use_squared:
        C_w = alpha * (wind_norm ** 2) * ((1 - cos_theta) ** 2)
    else:
        C_w = alpha * wind_norm * (1 - cos_theta)

    return C_w


def compute_wind_cost_physics(move_vector, wind_vector, drone_speed=10.0, epsilon=1e-3):
    """
    物理驱动的风场成本函数（无需调参！）

    =====================================================
    物理模型说明
    =====================================================

    核心思想：用"通过该段路径所需的时间"作为成本

    1. 无人机对空速度：v_drone（恒定，例如 10 m/s）
       - 这是无人机在空气中飞行的速度（相对速度）

    2. 风在运动方向的投影：
       W_proj = dot(W, d_unit)
       - 正值：顺风（风助力）
       - 负值：逆风（风阻碍）
       - 零值：侧风（无影响）

    3. 有效对地速度：
       v_eff = v_drone + W_proj

       示例：
       - 无风:         v_eff = 10 m/s
       - 5 m/s 顺风:   v_eff = 15 m/s  (更快！)
       - 5 m/s 逆风:   v_eff = 5 m/s   (更慢！)
       - 12 m/s 逆风:  v_eff = -2 m/s  (不可能，需要限制)

    4. 时间成本：
       时间 = 距离 / 速度
       Cost ∝ 1 / v_eff

       - 顺风：高 v_eff → 低 cost（快速通过）
       - 逆风：低 v_eff → 高 cost（缓慢通过）

    =====================================================
    处理极端逆风
    =====================================================

    当 |逆风| > v_drone 时，无人机无法前进。
    使用软约束处理：

       v_eff = max(v_drone + W_proj, v_min)

    其中 v_min = v_drone * 0.1（正常速度的10%）

    这自然地为强逆风创建惩罚，无需任何手动权重调节。

    =====================================================
    数值示例（drone_speed = 10 m/s）
    =====================================================

    风速   | 方向    | W_proj | v_eff  | Cost    | 解释
    -------|---------|--------|--------|---------|--------
    0 m/s  | -       | 0      | 10.0   | 0.100   | 正常速度
    5 m/s  | 顺风    | +5     | 15.0   | 0.067   | 加速！
    5 m/s  | 逆风    | -5     | 5.0    | 0.200   | 减速
    8 m/s  | 逆风    | -8     | 2.0    | 0.500   | 大幅减速
    15 m/s | 逆风    | -15    | 1.0    | 1.000   | 极端惩罚！

    =====================================================
    与权重方法的对比
    =====================================================

    权重方法: C_w = λ_w * ||W|| * (1 - cos(θ))
    - 需要为每个场景调参 λ_w
    - 没有物理意义
    - 效果随尺度变化

    物理方法: C_w = 1 / (v_drone + dot(W,d))
    - 无需调参（drone_speed是已知物理量）
    - 直接物理意义（速度的倒数 = 时间成本）
    - 尺度不变的行为

    Args:
        move_vector: 运动向量 V = (v_x, v_y, v_z)（栅格单位）
        wind_vector: 风向量 W = (w_x, w_y, w_z) 单位 m/s
        drone_speed: 无人机对空速度，单位 m/s（默认 10.0）
        epsilon: 防止除零的小常数（默认 1e-3）

    Returns:
        float: 天气成本 C_w ≥ 0（表示时间惩罚）
    """
    move_vector = np.array(move_vector, dtype=float)

    # 计算向量长度
    move_norm = np.linalg.norm(move_vector)
    wind_norm = np.linalg.norm(wind_vector)

    # 无运动时成本为0
    if move_norm == 0:
        return 0.0

    # 计算运动方向的单位向量
    move_dir = move_vector / move_norm

    # 计算风在运动方向的投影
    # W_proj = dot(W, d_unit)
    # - 正值：顺风（风与运动方向相同）
    # - 负值：逆风（风与运动方向相反）
    # - 零值：侧风（风垂直于运动方向）
    W_proj = np.dot(wind_vector, move_dir)

    # 计算有效对地速度
    # v_eff = v_drone + W_proj
    # - 顺风：W_proj > 0, v_eff > v_drone（更快）
    # - 逆风：W_proj < 0, v_eff < v_drone（更慢）
    v_eff = drone_speed + W_proj

    # 处理极端逆风
    # 当 |逆风| > v_drone 时，无人机无法前进
    # 使用软最小值防止除零或负速度
    v_min = drone_speed * 0.1  # 正常速度的 10% 作为最小值
    v_eff = max(v_eff, v_min)

    # 成本与有效速度成反比
    # C_w = 1 / v_eff
    # - 高 v_eff（顺风）→ 低成本
    # - 低 v_eff（逆风）→ 高成本
    C_w = 1.0 / (v_eff + epsilon)

    return C_w


# 默认使用物理驱动方法（可在设置中切换）
compute_wind_cost = compute_wind_cost_physics
