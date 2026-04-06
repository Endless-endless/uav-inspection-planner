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
from typing import Dict, List, Tuple, Optional


# =====================================================
# Weather Scene Configuration
# =====================================================

WEATHER_PROFILES = {
    "calm": {
        "scene": "calm",
        "label": "微风",
        "wind_speed": 2.0,
        "wind_direction": 0.0,
        "gust_factor": 1.1,
        "risk_level": "low",
        "energy_factor": 1.0,
        "description": "微风条件，适合正常巡检"
    },
    "crosswind": {
        "scene": "crosswind",
        "label": "侧风",
        "wind_speed": 6.0,
        "wind_direction": 90.0,
        "gust_factor": 1.2,
        "risk_level": "medium",
        "energy_factor": 1.15,
        "description": "中等侧风，需注意稳定性"
    },
    "headwind_strong": {
        "scene": "headwind_strong",
        "label": "强逆风",
        "wind_speed": 10.0,
        "wind_direction": 180.0,
        "gust_factor": 1.3,
        "risk_level": "high",
        "energy_factor": 1.35,
        "description": "逆风明显，建议谨慎规划连接段"
    },
    "tailwind_efficient": {
        "scene": "tailwind_efficient",
        "label": "顺风高效",
        "wind_speed": 8.0,
        "wind_direction": 0.0,
        "gust_factor": 1.15,
        "risk_level": "low",
        "energy_factor": 0.85,
        "description": "顺风条件，能耗较低"
    },
    "gusty_high_risk": {
        "scene": "gusty_high_risk",
        "label": "强阵风高风险",
        "wind_speed": 12.0,
        "wind_direction": 135.0,
        "gust_factor": 1.5,
        "risk_level": "high",
        "energy_factor": 1.4,
        "description": "强风阵风，高风险建议谨慎规划"
    }
}


def get_weather_profile(scene_name: str) -> Dict:
    """
    获取指定天气场景的配置 profile

    Args:
        scene_name: 天气场景名称 (calm, crosswind, headwind_strong, tailwind_efficient, gusty_high_risk)

    Returns:
        天气 profile 字典，包含 wind_speed, wind_direction, gust_factor, risk_level, energy_factor 等
    """
    scene_name = scene_name.lower().strip()
    if scene_name not in WEATHER_PROFILES:
        print(f"[WARNING] 未知的天气场景 '{scene_name}'，使用默认 'calm' 场景")
        scene_name = "calm"

    profile = WEATHER_PROFILES[scene_name].copy()
    return profile


def list_weather_profiles() -> Dict[str, Dict]:
    """
    列出所有可用的天气场景

    Returns:
        字典，key 为场景名，value 为对应的 profile
    """
    return WEATHER_PROFILES.copy()


def build_wind_vector_from_profile(profile: Dict) -> Tuple[float, float, float]:
    """
    从天气 profile 构建风向量

    Args:
        profile: 天气 profile，包含 wind_speed 和 wind_direction

    Returns:
        (w_x, w_y, w_z) 风向量，单位 m/s
    """
    wind_speed = profile.get("wind_speed", 0.0)
    wind_direction_deg = profile.get("wind_direction", 0.0)

    # 将角度转换为弧度
    wind_direction_rad = np.deg2rad(wind_direction_deg)

    # 计算风向量
    w_x = wind_speed * np.cos(wind_direction_rad)
    w_y = wind_speed * np.sin(wind_direction_rad)
    w_z = 0.0  # 假设水平风

    return (w_x, w_y, w_z)


def estimate_weather_risk(profile: Dict) -> Dict:
    """
    评估天气风险等级

    Args:
        profile: 天气 profile

    Returns:
        包含 risk_level, risk_score, message 的字典
    """
    risk_level = profile.get("risk_level", "unknown")
    gust_factor = profile.get("gust_factor", 1.0)
    wind_speed = profile.get("wind_speed", 0.0)

    # 计算风险评分 (0-100)
    risk_score = 0.0

    # 基础风速风险
    if wind_speed < 3:
        risk_score += 10
    elif wind_speed < 6:
        risk_score += 30
    elif wind_speed < 10:
        risk_score += 50
    else:
        risk_score += 70

    # 阵风因子风险
    if gust_factor > 1.2:
        risk_score += 15
    elif gust_factor > 1.4:
        risk_score += 30

    # 能耗因子风险
    energy_factor = profile.get("energy_factor", 1.0)
    if energy_factor > 1.3:
        risk_score += 10

    # 限制在 0-100
    risk_score = max(0, min(100, risk_score))

    # 生成风险消息
    if risk_level == "low":
        message = "天气条件良好，适合正常巡检"
    elif risk_level == "medium":
        message = "天气条件一般，需注意安全"
    elif risk_level == "high":
        message = "天气条件较差，建议谨慎规划"
    else:
        message = "未知天气条件"

    return {
        "risk_level": risk_level,
        "risk_score": round(risk_score, 2),
        "message": message
    }


def compute_segment_weather_penalty(
    p1: np.ndarray,
    p2: np.ndarray,
    profile: Dict,
    turn_angle_deg: Optional[float] = None
) -> Dict:
    """
    计算路径段在给定天气条件下的惩罚代价细分

    Args:
        p1: 起点 (x, y, z)
        p2: 终点 (x, y, z)
        profile: 天气 profile
        turn_angle_deg: 转向角度（度），None 表示直线

    Returns:
        包含 base_cost, wind_penalty, gust_penalty, risk_penalty, total 的字典
    """
    # 构建运动向量
    move_vector = np.array(p2) - np.array(p1)
    distance = np.linalg.norm(move_vector)

    if distance < 0.001:
        return {
            "base_cost": 0.0,
            "wind_penalty": 0.0,
            "gust_penalty": 0.0,
            "risk_penalty": 0.0,
            "total": 0.0
        }

    # 基础距离代价
    base_cost = distance

    # 构建风向量
    wind_vector = build_wind_vector_from_profile(profile)

    # 计算风向量代价（使用物理驱动方法）
    from weather.wind_model import compute_wind_cost_physics
    wind_cost = compute_wind_cost_physics(move_vector, wind_vector)

    # 转换为惩罚系数
    wind_penalty = wind_cost * distance * 0.1  # 缩放因子

    # 阵风惩罚
    gust_factor = profile.get("gust_factor", 1.0)
    gust_penalty = wind_penalty * (gust_factor - 1.0) * 0.5

    # 风险惩罚
    energy_factor = profile.get("energy_factor", 1.0)
    risk_penalty = base_cost * (energy_factor - 1.0) * 0.2

    # 转向惩罚（如果有转向）
    if turn_angle_deg is not None:
        turn_penalty = abs(turn_angle_deg) / 180.0 * distance * 0.05
    else:
        turn_penalty = 0.0

    # 总惩罚
    total_penalty = base_cost + wind_penalty + gust_penalty + risk_penalty + turn_penalty

    return {
        "base_cost": round(base_cost, 3),
        "wind_penalty": round(wind_penalty, 3),
        "gust_penalty": round(gust_penalty, 3),
        "risk_penalty": round(risk_penalty, 3),
        "turn_penalty": round(turn_penalty, 3),
        "total": round(total_penalty, 3)
    }


def summarize_weather_for_mission(profile: Dict, total_length: float) -> Dict:
    """
    为整个任务生成天气摘要

    Args:
        profile: 天气 profile
        total_length: 任务总长度（像素）

    Returns:
        天气摘要字典
    """
    wind_speed = profile.get("wind_speed", 0.0)
    wind_direction = profile.get("wind_direction", 0.0)
    gust_factor = profile.get("gust_factor", 1.0)
    energy_factor = profile.get("energy_factor", 1.0)

    # 评估风险
    risk_info = estimate_weather_risk(profile)

    # 估算天气惩罚（粗略估计）
    estimated_penalty_per_100px = total_length / 100.0 * (energy_factor - 1.0) * 10.0
    total_weather_penalty = total_length / 100.0 * estimated_penalty_per_100px

    return {
        "scene": profile.get("scene", "unknown"),
        "label": profile.get("label", "未知"),
        "wind_speed": round(wind_speed, 2),
        "wind_direction": round(wind_direction, 2),
        "gust_factor": round(gust_factor, 3),
        "risk_level": risk_info["risk_level"],
        "energy_factor": round(energy_factor, 3),
        "weather_risk_score": risk_info["risk_score"],
        "weather_message": risk_info["message"],
        "estimated_energy_score": round(100 / energy_factor, 2),
        "weather_penalty_total": round(total_weather_penalty, 3)
    }


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
