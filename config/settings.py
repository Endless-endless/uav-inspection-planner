"""
=====================================================
配置文件模块 (Configuration Settings Module)
=====================================================
功能说明:
    该模块集中管理项目的所有配置参数。
    便于统一调整和实验不同参数组合。

配置项:
    - 地图文件路径
    - 栅格分辨率
    - A*算法参数（权重、转弯惩罚等）
    - 起终点坐标
    - 巡检点坐标
    - 图形输出路径
=====================================================
"""

import os
# 获取项目根目录路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# =====================================================
# 快速运行模式配置 (Fast Mode Configuration)
# =====================================================
# 快速模式开关：True表示使用简化参数进行快速测试
FAST_MODE = True

# 快速模式下的地图规模缩放比例（30%~50%）
FAST_MODE_GRID_SCALE = 0.4

# 快速模式下的分辨率测试范围（跳过高分辨率）
FAST_MODE_RESOLUTIONS = [1.0, 0.8, 0.5]

# 快速模式下的风速测试范围（减少计算量）
FAST_MODE_WIND_SPEEDS = [0, 3, 6]

# 快速模式下的巡检点数量限制
FAST_MODE_MAX_INSPECTION_POINTS = 3

# 快速模式下的可视化设置（True=关闭弹窗，只保存）
FAST_MODE_NO_POPUP = True

# =====================================================
# 地图配置
# =====================================================
# 地图文件相对路径
MAP_FILE = "maps/urban_inspection_v1.world.json"

# 默认栅格分辨率（米）
DEFAULT_RESOLUTION = 0.5

# 默认分辨率（为了向后兼容）
RESOLUTION = DEFAULT_RESOLUTION

# =====================================================
# A*算法参数配置
# =====================================================
# 启发式权重：1.0为标准A*（保证最优性），>1.0为加权A*（加快搜索）
DEFAULT_WEIGHT = 1.2

# 转弯惩罚权重：越大路径越平滑
DEFAULT_TURN_WEIGHT = 5.0

# 最大转角约束（弧度）：None表示无约束
# 例如：math.radians(90) 表示最大转角90度
DEFAULT_MAX_TURN_ANGLE = None

# =====================================================
# 任务坐标配置（物理坐标，单位：米）
# =====================================================
# 单目标任务起终点
START_COORD = (-18, -18, 1.0)
GOAL_COORD = (18, 18, 1.0)

# 多目标巡检点坐标
ALL_INSPECTION_COORDS = [
    (-10, -10, 6),
    (10, -10, 6),
    (-10, 10, 6),
    (10, 10, 6)
]

# 根据FAST_MODE限制巡检点数量
if FAST_MODE:
    INSPECTION_COORDS = ALL_INSPECTION_COORDS[:FAST_MODE_MAX_INSPECTION_POINTS]
    print(f"[CONFIG] FAST_MODE: 巡检点数量限制为 {len(INSPECTION_COORDS)} 个")
else:
    INSPECTION_COORDS = ALL_INSPECTION_COORDS

# =====================================================
# 实验参数配置（根据FAST_MODE动态调整）
# =====================================================
# 分辨率对比实验的分辨率列表（米）
# 性能优化：移除了0.3m分辨率，避免计算过载
if FAST_MODE:
    RESOLUTION_EXPERIMENT = FAST_MODE_RESOLUTIONS
    print(f"[CONFIG] FAST_MODE: 分辨率测试范围 = {RESOLUTION_EXPERIMENT}")
else:
    RESOLUTION_EXPERIMENT = [0.5, 1.0, 2.0]

# 加权A*对比实验的权重列表
if FAST_MODE:
    WEIGHT_EXPERIMENT = [1.0, 1.5, 2.0]
    print(f"[CONFIG] FAST_MODE: 权重测试范围 = {WEIGHT_EXPERIMENT}")
else:
    WEIGHT_EXPERIMENT = [1.0, 1.2, 1.5, 2.0, 3.0]

# 转弯权重对比实验的权重列表
if FAST_MODE:
    TURN_WEIGHT_EXPERIMENT = [0, 5, 10]
    print(f"[CONFIG] FAST_MODE: 转弯权重测试范围 = {TURN_WEIGHT_EXPERIMENT}")
else:
    TURN_WEIGHT_EXPERIMENT = [0, 5, 10, 20]

# 气象影响实验的风速列表（米/秒）
if FAST_MODE:
    WIND_SPEED_EXPERIMENT = FAST_MODE_WIND_SPEEDS
    print(f"[CONFIG] FAST_MODE: 风速测试范围 = {WIND_SPEED_EXPERIMENT}")
else:
    WIND_SPEED_EXPERIMENT = [0, 2, 4, 6, 8]

# =====================================================
# 输出路径配置
# =====================================================
# 图形输出目录
FIGURES_DIR = os.path.join(PROJECT_ROOT, "figures")

# 图形文件格式设置
FIGURE_DPI = 600  # 高分辨率输出
FIGURE_FORMATS = ['png', 'pdf']  # 支持的格式


def get_map_path():
    """获取地图文件的完整路径"""
    return os.path.join(PROJECT_ROOT, MAP_FILE)


def get_figure_path(filename):
    """
    生成图形文件的完整路径

    参数:
        filename: str - 文件名（如 'experiment_result.png'）

    返回:
        str - 图形文件的完整保存路径
    """
    # 确保输出目录存在
    os.makedirs(FIGURES_DIR, exist_ok=True)

    # 分离文件名和扩展名
    name, ext = os.path.splitext(filename)

    # 如果扩展名为空，使用默认格式
    if not ext:
        ext = '.png'

    return os.path.join(FIGURES_DIR, name + ext)


def get_all_figure_paths(basename):
    """
    生成所有格式的图形文件路径

    参数:
        basename: str - 基础文件名（不含扩展名）

    返回:
        list - 所有格式文件的路径列表
    """
    paths = []
    for fmt in FIGURE_FORMATS:
        paths.append(get_figure_path(f"{basename}.{fmt}"))
    return paths


# =====================================================
# Weather Settings
# =====================================================

# 风向量 (wx, wy, wz) 单位 m/s
# 例如 (3,0,0) 表示 x方向 3m/s 风
# 设置为 (0,0,0) 禁用风场
WIND_VECTOR = (5.0, 0.0, 0.0)

# =====================================================
# 天气影响权重配置 (Weather Impact Weight Configuration)
# =====================================================
# LAMBDA_WEATHER: 天气成本权重系数 λ_w
#
# Cost Function: G(n) = C_d + λ_t * C_t + λ_w * C_w
#
# 建议值范围：
#   - λ_w = 0.5 ~ 2.0: 轻微影响（天气因素仅作微调）
#   - λ_w = 10 ~ 30: 中等影响（路径会明显避风）
#   - λ_w = 50 ~ 100: 强烈影响（路径大幅偏转以避开强风区域）
#
# 当前设置：极强天气影响模式（确保天气因素真正影响路径决策）
# 为了验证天气效果，λ_w 必须远大于 turn_weight（通常为0~20）
LAMBDA_WEATHER = 50.0

# 保持向后兼容
WIND_COST_WEIGHT = LAMBDA_WEATHER

# =====================================================
# 风场增强模式配置 (Wind Field Enhancement Configuration)
# =====================================================
# USE_SQUARED_WIND_COST: 是否使用平方放大形式的风场成本函数（仅用于权重方法）
#
# False: C_w = λ_w * ||W|| * (1 - cos(θ))       [线性]
# True:  C_w = λ_w * ||W||^2 * (1 - cos(θ))^2   [平方放大]
#
# 注意：此设置仅在使用权重方法时生效。使用物理驱动方法时无效。
USE_SQUARED_WIND_COST = True

# =====================================================
# 风场成本计算方法选择 (Wind Cost Method Selection)
# =====================================================
# WIND_COST_METHOD: 选择风场成本的计算方法
#
# 'physics': 物理驱动方法（推荐，无需调参）
#    C_w = 1 / (v_drone + dot(W, d))
#    - 基于物理原理（有效速度的倒数 = 时间成本）
#    - 无需手动调节权重
#    - 自动适应风速变化
#
# 'weighted': 权重方法（旧方法，需要调参）
#    C_w = λ_w * ||W|| * (1 - cos(θ))
#    - 需要手动调节 λ_w
#    - 保留用于向后兼容
WIND_COST_METHOD = 'physics'

# =====================================================
# 无人机物理参数 (UAV Physical Parameters)
# =====================================================
# DRONE_AIRSPEED: 无人机的对空速度（单位：m/s）
#
# 这是无人机在空气中飞行的恒定速度。
# 典型值：小型无人机 5~15 m/s，大型无人机 15~30 m/s
#
# 注意：此参数仅在 WIND_COST_METHOD = 'physics' 时使用。
DRONE_AIRSPEED = 10.0  # m/s

# =====================================================
# 风场空间分布配置 (Wind Field Spatial Distribution)
# =====================================================
# USE_NON_UNIFORM_WIND: 是否使用非均匀风场（区域差异风场）
#
# False: 均匀风场（全局风速相同）
# True:  非均匀风场（不同区域风速不同，强制路径绕行）
USE_NON_UNIFORM_WIND = True