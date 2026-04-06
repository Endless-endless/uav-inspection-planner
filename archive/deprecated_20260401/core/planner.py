"""
路径规划主流程模块

功能：整合所有模块，提供统一的规划接口
"""

# 导入完整的规划器实现
# 为了保持兼容性，这里直接引用原始实现
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3 as _OriginalPlannerV3

# 导入功能模块
from .vision import PowerlineExtractor
from .optimizer import WindOptimizer
from .energy import EnergyCalculator
from .mission import MissionGenerator
from .terrain import TerrainGenerator


class PowerlinePlannerV3(_OriginalPlannerV3):
    """
    电网巡检路径规划器 V3.0

    整合功能：
    - 电网提取
    - 路径规划
    - 风影响优化
    - 能耗评估
    - 任务生成
    """

    def __init__(self, *args, **kwargs):
        """初始化规划器"""
        super().__init__(*args, **kwargs)

        # 初始化功能模块
        self.energy_calculator = EnergyCalculator()
        self.mission_generator = MissionGenerator()
        self.wind_optimizer = WindOptimizer(
            wind_direction=kwargs.get('wind_direction', 0),
            wind_speed=kwargs.get('wind_speed', 5)
        )


# 便捷函数
def plan_powerline_inspection(
    image_path,
    terrain_raw,
    flight_height=30,
    sample_step=5,
    smooth_window=5,
    use_spline=False,
    add_flight_fluctuation=True,
    tower_interval=20,
    gaussian_sigma=3.0,
    enhance_resolution=False,
    output_file="powerline_inspection.html",
    wind_direction=0,
    wind_speed=5
):
    """
    电网巡检路径规划主函数

    Args:
        image_path: 电网图像路径
        terrain_raw: 原始地形图
        flight_height: 飞行高度（米）
        sample_step: 采样间隔
        smooth_window: 平滑窗口
        use_spline: 是否使用样条平滑
        add_flight_fluctuation: 是否添加飞行波动
        tower_interval: 电塔采样间隔
        gaussian_sigma: 地形高斯模糊sigma
        enhance_resolution: 是否提高地形分辨率
        output_file: 输出文件名
        wind_direction: 风向角度（0-360°）
        wind_speed: 风速（m/s）

    Returns:
        PowerlinePlannerV3: 规划器对象
    """
    # 导入主函数
    from planner.powerline_planner_v3_final import plan_powerline_inspection_v3

    return plan_powerline_inspection_v3(
        image_path=image_path,
        terrain_raw=terrain_raw,
        flight_height=flight_height,
        sample_step=sample_step,
        smooth_window=smooth_window,
        use_spline=use_spline,
        add_flight_fluctuation=add_flight_fluctuation,
        tower_interval=tower_interval,
        gaussian_sigma=gaussian_sigma,
        enhance_resolution=enhance_resolution,
        output_file=output_file,
        wind_direction=wind_direction,
        wind_speed=wind_speed
    )
