# UAV电网巡检路径规划系统 - 项目结构

## 📁 目录结构

```
uav_path_planning_project/
├── core/                      # 🔧 核心模块
│   ├── __init__.py           # 模块导出
│   ├── vision.py             # 电网提取
│   ├── planner.py            # 路径规划主流程
│   ├── optimizer.py          # 风影响优化
│   ├── energy.py             # 能耗评估
│   ├── mission.py            # 巡检任务生成
│   └── terrain.py            # 地形生成
│
├── demo/                      # 🚀 演示入口
│   ├── __init__.py
│   └── demo_main.py          # 主入口文件
│
├── legacy/                    # 📦 归档旧代码
│   ├── old_planners/         # 旧版规划器
│   └── old_demos/            # 旧版演示
│
├── planner/                   # 📐 完整规划器（保持兼容）
│   └── powerline_planner_v3_final.py
│
├── config/                    # ⚙️ 配置
│   ├── __init__.py
│   └── settings.py
│
├── data/                      # 📊 数据
│   ├── map_power1.png
│   ├── map_power2.png
│   └── map_power3.png
│
├── figures/                   # 📈 结果图
│   ├── powerline_v3_*.html
│   └── *.png
│
├── result/                    # 📝 中间结果
│
├── docs/                      # 📚 文档
│
├── analysis/                  # 📉 分析工具
├── environment/               # 🌍 环境模块
├── examples/                  # 💡 示例代码
├── experiments/               # 🔬 实验代码
├── maps/                      # 🗺️ 地图数据
├── mission/                   # 🎯 任务模块
├── utils/                     # 🛠️ 工具函数
├── visualization/             # 🎨 可视化
└── weather/                   # 🌤️ 天气模块
```

## 🔧 核心模块说明

### 1. vision.py - 电网提取模块
```python
from core.vision import PowerlineExtractor

extractor = PowerlineExtractor("data/map_power1.png")
mask = extractor.extract_redline_hsv()
```

**功能**：
- HSV色彩空间提取红色电网
- 形态学操作修复断裂
- 骨架化提取细线路径

### 2. planner.py - 路径规划主流程
```python
from core.planner import plan_powerline_inspection

planner = plan_powerline_inspection(
    image_path="data/map_power1.png",
    terrain_raw=terrain,
    flight_height=25,
    wind_direction=0,
    wind_speed=5
)
```

**功能**：
- 整合所有功能模块
- 提供统一规划接口
- 生成完整巡检方案

### 3. optimizer.py - 风影响优化
```python
from core.optimizer import WindOptimizer

optimizer = WindOptimizer(wind_direction=90, wind_speed=10)
optimized_path = optimizer.optimize_path_with_wind(
    waypoints, width, height
)
```

**功能**：
- 计算风影响成本
- 局部路径优化
- 强化风惩罚机制

### 4. energy.py - 能耗评估
```python
from core.energy import EnergyCalculator

calculator = EnergyCalculator()
energy = calculator.compute_energy_cost(path_3d, wind_speed)
```

**功能**：
- 计算航线总能耗
- 计算每米能耗
- 考虑风速和爬升

### 5. mission.py - 巡检任务生成
```python
from core.mission import MissionGenerator

generator = MissionGenerator()
tasks = generator.generate_inspection_tasks(path_3d, interval=10)
```

**功能**：
- 生成拍照任务点
- 设置相机角度
- 任务ID管理

### 6. terrain.py - 地形生成
```python
from core.terrain import TerrainGenerator

generator = TerrainGenerator()
terrain = generator.generate_realistic_terrain(width, height)
```

**功能**：
- 生成仿真地形
- 高斯平滑处理
- 可配置高程范围

## 🚀 使用方式

### 方式1：使用主入口（推荐）
```bash
python demo/demo_main.py
```

### 方式2：使用核心模块
```python
from core.planner import plan_powerline_inspection
from core.terrain import TerrainGenerator

# 生成地形
terrain = TerrainGenerator.generate_realistic_terrain(2416, 1336)

# 规划路径
planner = plan_powerline_inspection(
    image_path="data/map_power1.png",
    terrain_raw=terrain,
    flight_height=25
)

# 获取结果
stats = planner.stats
print(f"路径长度: {stats['path_length']:.1f}米")
print(f"能耗: {stats['energy_cost']:.1f}单位")
```

### 方式3：使用完整规划器（向后兼容）
```python
from planner.powerline_planner_v3_final import plan_powerline_inspection_v3

planner = plan_powerline_inspection_v3(
    image_path="data/map_power1.png",
    terrain_raw=terrain,
    flight_height=25
)
```

## 📊 系统功能清单

| 功能模块 | 状态 | 说明 |
|----------|------|------|
| 电网提取 | ✅ | HSV色彩空间识别 |
| 路径规划 | ✅ | 沿电网自动生成航线 |
| 风影响优化 | ✅ | 基于风向/风速优化 |
| 能耗评估 | ✅ | 计算航线能耗 |
| 任务生成 | ✅ | 生成拍照任务点 |
| 3D可视化 | ✅ | Plotly交互式图表 |
| 地形生成 | ✅ | 仿真地形生成 |

## 🔄 迁移指南

### 从旧代码迁移到新结构

**旧代码**：
```python
from planner.powerline_planner_v3_final import plan_powerline_inspection_v3
```

**新代码**：
```python
from core.planner import plan_powerline_inspection
# 或
from demo.demo_main import main  # 运行完整演示
```

## 📝 版本历史

- **V3.0** (当前) - 模块化重构，新增能耗和任务生成
- **V2.0** - 风影响优化
- **V1.0** - 基础路径规划

## 🤝 贡献指南

1. 新功能添加到 `core/` 模块
2. 演示代码添加到 `demo/`
3. 保持 `planner/powerline_planner_v3_final.py` 向后兼容
4. 旧代码归档到 `legacy/`
