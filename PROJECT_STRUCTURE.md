# UAV电网巡检路径规划系统 - 项目结构

**更新时间：** 2026-04-06
**项目状态：** 基于拓扑图的全局优化路径规划系统

---

## 📁 目录结构

```
uav_path_planning_project/
├── core/                           # 🔧 核心模块
│   ├── __init__.py                # 模块导出
│   ├── topo.py                    # 🌟 拓扑图建模（节点/边/图构建）
│   ├── topo_task.py               # 🌟 EdgeTask 建模（点边映射）
│   ├── topo_plan.py               # 🌟 拓扑规划逻辑（贪心/分组）
│   ├── topo_global_optimizer.py   # 🌟 全局优化器（模拟退火/综合代价）
│   ├── start_driven_planner.py    # 起点驱动规划（旧版）
│   ├── start_driven_planner_v2.py # 起点驱动规划 v2
│   ├── independent_lines.py       # 独立线路提取
│   ├── inspection_point_generator.py # 巡检点生成
│   └── visualization_enhanced.py  # 可视化增强模块
│
├── planner/                        # 📐 完整规划器
│   ├── powerline_planner_v3_final.py  # 🌟 主规划器（整合所有步骤）
│   ├── powerline_inspector.py     # 巡检器
│   ├── astar3d.py                 # A* 3D 路径规划
│   └── node.py                    # 节点定义
│
├── demo/                           # 🚀 演示入口
│   ├── __init__.py
│   ├── demo_visualization_main.py # 🌟 完整流程入口（推荐）
│   ├── generate_interactive_main_view.py # 生成交互式主展示页
│   ├── generate_main_view_map_based.py # 生成地图叠加页
│   ├── demo_ui_animation.py       # 辅助导航页
│   └── demo_start_point_driven.py # 起点驱动演示
│
├── mission/                        # 🎯 任务模块
│   ├── __init__.py
│   └── multi_goal.py              # 多目标任务
│
├── weather/                        # 🌤️ 天气模块
│   ├── __init__.py
│   └── wind_model.py              # 风况模型
│
├── visualization/                  # 🎨 可视化
│   ├── __init__.py
│   ├── plotly_viewer.py           # Plotly 3D 可视化
│   ├── map_overlay.py             # 地图叠加可视化
│   ├── map_3d_overlay.py          # 3D 地图叠加
│   ├── matplotlib_plots.py        # Matplotlib 绘图
│   └── map_overlay.py             # 地图叠加
│
├── environment/                    # 🌍 环境模块
│   ├── __init__.py
│   ├── grid_map.py                # 网格地图
│   └── heightmap_loader.py        # 高程图加载
│
├── analysis/                       # 📉 分析工具
│   ├── __init__.py
│   └── metrics.py                 # 指标计算
│
├── utils/                          # 🛠️ 工具函数
│   ├── __init__.py
│   └── csv_logger.py              # CSV 日志
│
├── config/                         # ⚙️ 配置
│   ├── __init__.py
│   └── settings.py                # 设置
│
├── scripts/                        # 📜 脚本工具
│   ├── final_teacher_demo.py      # 教师演示脚本
│   ├── verify_*.py                # 验证脚本
│   └── replan_*.py                # 重规划脚本
│
├── archive/                        # 📦 归档
│   ├── deprecated_20260401/       # 2026-04-01 前的旧代码
│   │   ├── core/                  # 旧核心模块
│   │   └── demo/                  # 旧演示脚本
│   ├── debug_scripts/             # 调试脚本
│   ├── legacy_demo/               # 遗留演示
│   └── demo_*.py                  # 其他归档演示
│
├── data/                           # 📊 数据
│   └── test.png                   # 测试图像 (916x960)
│
├── result/                         # 📝 结果输出
│   └── latest/                    # 最新结果
│       ├── mission_output.json    # 任务 JSON
│       ├── main_view_interactive.html # 交互式页面
│       └── *.png                  # 可视化图像
│
├── figures/                        # 📈 结果图
│   └── powerline_v3_*.html        # 历史可视化
│
└── docs/                           # 📚 文档
    └── MISSION_JSON_AND_UI_FEATURES.md # JSON 格式说明
```

---

## 🔧 核心模块详解

### 1. 拓扑建模层

#### 1.1 core/topo.py - 拓扑图建模

**核心类：**
```python
@dataclass
class TopoNode:
    """拓扑节点"""
    id: str                          # 唯一标识
    kind: str                        # endpoint / split
    pos2d: Tuple[float, float]       # (x, y)
    pos3d: Tuple[float, float, float] # (x, y, z)
    deg: int                         # 度数（连接的边数）
    angle_change: float              # 角度变化量（split点）
    split_reason: str                # 切分原因：'length'/'angle'

@dataclass
class TopoEdge:
    """拓扑边"""
    id: str                          # 唯一标识
    u: str                           # 起始节点ID
    v: str                           # 结束节点ID
    line_id: str                     # 所属线路ID
    polyline: List[Tuple[float, float]] # 2D几何
    len2d: float                     # 长度
    is_straight: bool                # 是否近似直线

@dataclass
class TopoGraph:
    """拓扑图"""
    nodes: Dict[str, TopoNode]       # 节点字典
    edges: Dict[str, TopoEdge]       # 边字典
    adj: Dict[str, List[str]]        # 邻接表
```

**核心函数：**
```python
# 节点检测
detect_endpoint_nodes(lines) -> List[TopoNode]
detect_split_nodes(line, max_len=600, angle_thresh=60) -> List[TopoNode]

# 边切分
split_line_to_edges(line, nodes_on_line) -> List[TopoEdge]

# 图构建
build_topo_graph(nodes, edges) -> TopoGraph

# 节点合并
merge_duplicate_nodes(nodes, thresh=25.0) -> Tuple[List[TopoNode], Dict]
```

**切分规则：**
- 长度阈值：累积路径长度超过 `EDGE_MAX_LEN = 600px`
- 角度阈值：相邻段角度变化超过 `ANGLE_THRESH = 60°`
- 最小边长：切分产生的边长度必须 >= `EDGE_MIN_LEN = 150px`
- 切分优先级：端点 → 强角度切分点 → 长度切分点

#### 1.2 core/topo_task.py - 边任务建模

**核心类：**
```python
@dataclass
class EdgeTask:
    """边任务：基于拓扑边的巡检任务单元"""
    edge_id: str                      # 拓扑边ID
    u: str                            # 起始节点ID
    v: str                            # 结束节点ID
    line_id: str                      # 所属线路ID
    polyline: List[Tuple[float, float]] # 2D几何
    len2d: float                      # 长度
    inspection_points: List[Dict]     # 巡检点列表
    num_points: int                   # 巡检点数量
    is_straight: bool                 # 是否近似直线
```

**核心函数：**
```python
# 点边映射
map_points_to_edges(topo_graph, line_inspection_points_by_line) -> Dict

# 边任务构建
build_edge_tasks(topo_graph, line_inspection_points_by_line) -> List[EdgeTask]
```

### 2. 路径规划层

#### 2.1 core/topo_plan.py - 拓扑规划逻辑

**核心数据结构：**
```python
@dataclass
class MissionSegment:
    """任务段"""
    type: str                         # 'inspect' 或 'connect'
    from_edge_id: str                 # 起始边ID（connect段用）
    to_edge_id: str                   # 目标边ID
    geometry: List[Tuple[float, float]] # 路径几何
    length: float                     # 长度
    edge_id: str = None               # 边ID（inspect段用）

@dataclass
class EdgeGroup:
    """边分组（空间聚类）"""
    group_id: str                     # 分组ID
    edge_ids: List[str]               # 包含的边ID列表
    centroid: Tuple[float, float]     # 分组中心
    bbox: Tuple[float, float, float, float] # 边界框
    total_inspect_length: float       # 总巡检长度

@dataclass
class GroupedContinuousMission:
    """分组连续任务"""
    segments: List[MissionSegment]    # 任务段列表
    groups: Dict[str, EdgeGroup]      # 分组信息
    visit_order: List[str]            # 边访问顺序
    group_visit_order: List[str]      # 分组访问顺序
    total_length: float               # 总长度
    inspect_length: float             # 巡检长度
    connect_length: float             # 连接长度
```

**核心函数：**
```python
# 边邻接表构建
build_edge_adjacency_simple(topo_graph) -> dict

# 空间分组
group_edges_spatially(edge_tasks, centroids, eps=150.0) -> Dict

# 组间贪心排序
order_groups_greedy(groups, edge_task_map) -> List[str]

# 连接段生成（沿拓扑图）
generate_connection_segment_along_topo(
    from_point, to_point, topo_graph, edge_task_map
) -> Tuple[List[Tuple], float]

# 贪心规划
plan_topo_mission_greedy(topo_graph, edge_tasks, start_edge_id) -> TopoMission

# 分组连续任务构建
build_grouped_continuous_mission(
    topo_graph, edge_tasks, groups, group_visit_order, adjacency
) -> GroupedContinuousMission

# JSON导出
export_grouped_mission_to_json(
    mission, edge_tasks, line_inspection_points_by_line,
    output_path, terrain_3d, weather_info
) -> str
```

#### 2.2 core/topo_global_optimizer.py - 全局优化器

**核心数据结构：**
```python
@dataclass
class ConnectionCost:
    """连接代价模型"""
    geometric_distance: float         # 几何直线距离
    topo_path_length: float           # 拓扑图上的最短路径长度
    direction_change: float           # 方向变化角度（0-1）
    group_switch_penalty: float       # group切换惩罚
    total_cost: float                 # 综合总代价
```

**核心函数：**
```python
# 综合连接代价计算
compute_connection_cost_enhanced(
    from_edge_id, to_edge_id, from_direction, to_direction,
    from_point, to_point, topo_graph, edge_task_map, groups, weights
) -> ConnectionCost

# BFS找拓扑路径
find_topo_path(topo_graph, start_node, end_node) -> List[str]

# 计算拓扑路径长度
compute_topo_path_length(topo_graph, path, edge_task_map) -> float

# 方向变化惩罚
compute_direction_change_penalty(
    from_point, to_point, from_direction, to_direction,
    from_edge, to_edge
) -> float

# 起始边评估
evaluate_start_edge_candidate(
    start_edge_id, edge_tasks, topo_graph, groups, edge_task_map, adjacency
) -> float

# 生成候选起始边
generate_start_edge_candidates(
    edge_tasks, topo_graph, groups, edge_task_map, adjacency, max_candidates=10
) -> List[Tuple[str, float]]

# 模拟退火优化边顺序
optimize_edge_order_simulated_annealing(
    edge_tasks, topo_graph, edge_task_map, groups, adjacency,
    start_edge_id, initial_temp=1000.0, cooling_rate=0.95,
    iterations_per_temp=100, min_temp=1.0
) -> Tuple[List[str], Dict[str, str], float]

# 构建优化后的任务
build_optimized_mission(
    edge_order, edge_directions, edge_tasks, topo_graph,
    edge_task_map, groups, adjacency
) -> GroupedContinuousMission

# 主入口：全局拓扑优化
plan_global_topology_optimized_mission(
    topo_graph, edge_tasks, start_edge_id, enable_sa, eps
) -> GroupedContinuousMission
```

**代价权重：**
```python
weights = {
    'geometric': 1.0,      # 几何距离权重
    'topo': 0.1,           # 拓扑路径权重
    'direction': 0.1,      # 方向变化权重
    'group_switch': 10.0   # group切换惩罚
}
```

### 3. 主规划器

#### 3.1 planner/powerline_planner_v3_final.py - 主规划器

**核心类：**
```python
class PowerlinePlannerV3:
    """电网巡检路径规划器 V3.0"""

    def __init__(self, image_path, flight_height=30,
                 weather_scene="calm", weather_profile=None):
        # 天气相关
        self.weather_scene = weather_scene
        self.weather_profile = get_weather_profile(weather_scene)
        self.mission_wind = {...}  # 风况信息

        # 拓扑层数据
        self.topo_nodes = []        # List[TopoNode]
        self.topo_edges = []        # List[TopoEdge]
        self.topo_graph = None      # TopoGraph
        self.edge_tasks = []        # List[EdgeTask]
```

**核心步骤：**
```python
# Step 1: HSV 红色提取
step1_extract_redline_hsv() -> ndarray

# Step 2: 形态学修复
step2_fix_breaks() -> ndarray

# Step 3: 骨架化
step3_skeletonize() -> ndarray

# Step 4: 独立线路识别
step4_extract_independent_lines() -> List[IndependentLine]

# Step 5: 巡检点生成
step5_generate_line_inspection_points() -> List[LineInspectionPoint]

# Step 7.5: 拓扑图建模
step7_5_build_topo() -> TopoGraph

# Step 8.5: EdgeTask 建模
step8_5_build_edge_tasks() -> List[EdgeTask]

# Step 9.0: 拓扑贪心规划
step9_0_plan_topo_mission_greedy(start_edge_id) -> TopoMission

# Step 9.1: 连续任务规划
step9_1_plan_continuous_mission(start_edge_id) -> ContinuousMission

# Step 9.2: 分组连续任务规划
step9_2_plan_grouped_continuous_mission(eps=150.0) -> GroupedContinuousMission

# Step 9.3: 分组优化任务规划
step9_3_plan_grouped_mission_optimized(eps=150.0) -> GroupedContinuousMission

# Step 9.4: 全局拓扑优化（新旧算法对比+自动择优）
step9_4_plan_global_topology_optimized(
    start_edge_id, enable_sa=False, eps=150.0
) -> GroupedContinuousMission

# Step 9.5: 拓扑路径规划
step9_5_plan_topo_mission(start_pos, wind) -> Tuple[List, Dict, List]
```

### 4. 天气模块

#### 4.1 weather/wind_model.py - 风况模型

**核心函数：**
```python
# 获取天气配置
get_weather_profile(scene) -> Dict

# 支持的天气场景
scenes = {
    "calm": "微风",
    "crosswind": "侧风",
    "headwind_strong": "强逆风",
    "tailwind_efficient": "顺风高效",
    "gusty_high_risk": "阵风高风险"
}

# 天气配置包含
{
    "scene": "calm",
    "label": "微风",
    "wind_speed": 2.0,
    "wind_direction": 0.0,
    "gust_factor": 1.1,
    "risk_level": "low",
    "energy_factor": 1.0,
    "description": "..."
}
```

---

## 🚀 使用方式

### 方式1：使用主入口（推荐）

```bash
python demo/demo_visualization_main.py
```

**输出：**
- `result/latest/mission_output.json` - 标准任务 JSON
- `result/latest/main_view_interactive.html` - 交互式主展示页
- `result/latest/map_overlay_test.png` - 2D 地图叠加

### 方式2：使用规划器 API

```python
from planner.powerline_planner_v3_final import PowerlinePlannerV3

# 创建规划器
planner = PowerlinePlannerV3(
    image_path='data/test.png',
    flight_height=30,
    weather_scene='calm'  # calm, crosswind, gusty_high_risk, ...
)

# 执行完整流程
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()
planner.step4_extract_independent_lines()
planner.step5_generate_line_inspection_points()

# 设置地形
terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
planner.step6_map_line_points_to_3d(terrain)
planner.step6_smooth_terrain(terrain)

# 拓扑建模
topo_graph = planner.step7_5_build_topo()
edge_tasks = planner.step8_5_build_edge_tasks()

# 全局优化路径规划
mission = planner.step9_4_plan_global_topology_optimized(
    enable_sa=False,  # 使用贪心策略
    eps=150.0
)

# 查看结果
print(f"总长度: {mission.total_length:.1f}px")
print(f"巡检: {mission.inspect_length:.1f}px")
print(f"连接: {mission.connect_length:.1f}px")
```

### 方式3：直接使用拓扑规划模块

```python
from core.topo import build_topo_graph
from core.topo_task import build_edge_tasks
from core.topo_global_optimizer import plan_global_topology_optimized_mission

# 假设已有 topo_graph 和 edge_tasks
mission = plan_global_topology_optimized_mission(
    topo_graph=topo_graph,
    edge_tasks=edge_tasks,
    start_edge_id=None,  # 自动选择
    enable_sa=False,     # 使用贪心策略
    eps=150.0           # 分组距离阈值
)
```

---

## 📊 系统功能清单

| 功能模块 | 状态 | 说明 | 代码位置 |
|----------|------|------|---------|
| **电网提取** | ✅ | HSV色彩空间识别 | planner/powerline_planner_v3_final.py:175 |
| **骨架化** | ✅ | 骨架提取中心线 | planner/powerline_planner_v3_final.py:249 |
| **独立线路识别** | ✅ | 多连通域分离 | planner/powerline_planner_v3_final.py:Step4 |
| **巡检点生成** | ✅ | 端点/拐点/采样点 | planner/powerline_planner_v3_final.py:Step5 |
| **拓扑建模** | ✅ | 节点检测、边切分、图构建 | core/topo.py |
| **EdgeTask 建模** | ✅ | 点边映射、任务构建 | core/topo_task.py |
| **空间分组** | ✅ | DBSCAN 聚类 | core/topo_plan.py:1967 |
| **边顺序优化** | ✅ | 模拟退火/贪心 | core/topo_global_optimizer.py:408 |
| **连接段生成** | ✅ | BFS 沿拓扑图路径 | core/topo_plan.py:1098 |
| **自动择优** | ✅ | 新旧算法对比 | planner/powerline_planner_v3_final.py:3083 |
| **JSON 导出** | ✅ | 标准格式输出 | core/topo_plan.py:3769 |
| **3D 可视化** | ✅ | Plotly 交互式图表 | visualization/plotly_viewer.py |
| **天气影响** | ✅ | 多场景风况模型 | weather/wind_model.py |
| **地图叠加** | ✅ | 真实地图底图 | visualization/map_overlay.py |

---

## 📝 JSON 输出格式

### mission_output.json 结构

```json
{
  "mission_info": {
    "total_length": 12345.6,
    "inspect_length": 10000.0,
    "connect_length": 2345.6,
    "num_segments": 50,
    "num_inspect_segments": 25,
    "num_connect_segments": 24,
    "start_point": [100, 200],
    "end_point": [800, 600]
  },
  "groups": {
    "Group_0": {
      "group_id": "Group_0",
      "edge_ids": ["line_0_edge_0", "line_0_edge_1"],
      "centroid": [400, 300],
      "bbox": [100, 200, 600, 400],
      "total_inspect_length": 1000.0
    }
  },
  "group_visit_order": ["Group_0", "Group_1"],
  "edge_visit_order": ["line_0_edge_0", "line_0_edge_1", ...],
  "segments": [
    {
      "type": "inspect",
      "edge_id": "line_0_edge_0",
      "from_edge_id": null,
      "to_edge_id": "line_0_edge_0",
      "geometry": [[100, 200], [110, 210], ...],
      "length": 500.0
    },
    {
      "type": "connect",
      "from_edge_id": "line_0_edge_0",
      "to_edge_id": "line_1_edge_0",
      "geometry": [[200, 300], [250, 350], ...],
      "length": 150.0
    }
  ],
  "inspection_points": [
    {
      "point_id": "line_0_pt_0",
      "line_id": "line_0",
      "edge_id": "line_0_edge_0",
      "position_2d": [100, 200],
      "position_3d": [100, 200, 30],
      "point_type": "endpoint"
    }
  ],
  "full_path_2d": [[100, 200], [110, 210], ...],
  "full_path_3d": [[100, 200, 30], [110, 210, 30], ...],
  "weather": {
    "scene": "calm",
    "label": "微风",
    "wind_speed": 2.0,
    "wind_direction": 0.0,
    "risk_level": "low",
    "energy_factor": 1.0
  }
}
```

---

## 🔄 调用链关系

### 完整调用链

```
[demo_visualization_main.py]
    ↓
[PowerlinePlannerV3.step1~step6]  // 图像处理、骨架提取、3D映射
    ↓
[PowerlinePlannerV3.step7_5_build_topo]
    → detect_topo_nodes()  // core/topo.py:414
    → split_lines_to_edges()  // core/topo.py:835
    → merge_duplicate_nodes()  // core/topo.py:448
    → build_topo_graph()  // core/topo.py:880
    ↓
[PowerlinePlannerV3.step8_5_build_edge_tasks]
    → build_edge_tasks()  // core/topo_task.py:206
    ↓
[PowerlinePlannerV3.step9_4_plan_global_topology_optimized]
    │
    ├─→ [方案1] build_grouped_continuous_mission()  // core/topo_plan.py:2236
    │       → group_edges_spatially()  // DBSCAN
    │       → order_groups_greedy()
    │       → [组内] build_continuous_mission_greedy()
    │       → [组间] generate_connection_segment_along_topo()
    │
    └─→ [方案2] plan_global_topology_optimized_mission()  // core/topo_global_optimizer.py:708
            → build_edge_adjacency_simple()  // core/topo_plan.py:783
            → group_edges_spatially()
            → generate_start_edge_candidates()  // core/topo_global_optimizer.py:310
            → optimize_edge_order_simulated_annealing()  // core/topo_global_optimizer.py:408
            → build_optimized_mission()  // core/topo_global_optimizer.py:552
    ↓
[自动择优]  // 比较 connect_length 和 total_length
    ↓
[export_grouped_mission_to_json]  // core/topo_plan.py:3769
```

### 路径段生成流程

```
对于每一对相邻的边 (edge_i → edge_j):
    ├─ if edge_j 与 edge_i 相邻 (共享节点):
    │   → 直接添加 inspect 段 (连接代价=0)
    │
    └─ else (不相邻):
        → find_topo_path()  // BFS 找拓扑图最短路径
        → generate_connection_segment_along_topo()  // 沿图生成 connect 段
        → 添加 connect 段
        → 添加 inspect 段
```

---

## 🎯 核心算法说明

### 算法本质

| 组件 | 算法 | 说明 |
|-----|-----|-----|
| **边顺序优化** | 模拟退火（已禁用）→ 贪心最近邻 | 代码中 `enable_sa=False` |
| **节点间路径** | BFS（广度优先搜索） | `find_topo_path` |
| **空间分组** | DBSCAN 聚类 | `group_edges_spatially` |
| **组内访问** | 贪心最近邻 | `build_continuous_mission_greedy` |
| **组间连接** | 基于拓扑图的最短路径 | `generate_connection_segment_along_topo` |

### 优化目标（三级择优）

1. **一级目标**：最小化 `connect_length`（连接段总长度）
2. **二级目标**：最小化 `connect_segment` 数量
3. **三级目标**：最小化 `total_length`

### 是否全局最优？

**答案：否**

- 理论保证：❌ 无（模拟退火是启发式，贪心是局部最优）
- 实际运行：⚠️ 贪心策略（`enable_sa=False`）
- 搜索空间：⚠️ 受限（只在"边访问顺序"空间搜索）
- 约束处理：✅ 部分（考虑了拓扑约束，但未考虑 UAV 动力学）

---

## 🔧 开发指南

### 添加新的规划算法

1. 在 `core/` 目录创建新文件（如 `topo_my_algorithm.py`）
2. 实现规划函数，返回 `GroupedContinuousMission`
3. 在 `planner/powerline_planner_v3_final.py` 中添加新的 `step9_x` 方法
4. 在 `demo/demo_visualization_main.py` 中调用

### 添加新的天气场景

1. 在 `weather/wind_model.py` 的 `WEATHER_SCENES` 中添加配置
2. 使用时指定 `weather_scene="your_scene"`

### 自定义代价权重

1. 修改 `core/topo_global_optimizer.py:78-84` 的默认权重
2. 或在调用时传入自定义 `weights` 字典

---

## 📝 版本历史

| 版本 | 日期 | 状态 | 说明 |
|------|------|------|------|
| **Step 7.5** | 2026-03-XX | ✅ 主线 | 拓扑图建模（节点检测、边切分、图构建） |
| **Step 8.5** | 2026-03-XX | ✅ 主线 | EdgeTask 建模（点边映射、任务构建） |
| **Step 9.2** | 2026-04-01 | ✅ 稳定 | 分组感知连续航迹规划 |
| **Step 9.4** | 2026-04-06 | ✅ **当前** | 全局拓扑优化（新旧算法对比+自动择优） |

### 当前版本特性（Step 9.4）

**核心改进：**
1. 全局拓扑优化算法（基于论文思想）
2. 综合代价模型（几何+拓扑+方向+组切换）
3. 自动择优机制（新旧算法对比）
4. 模拟退火优化（可配置，默认关闭）

**已知限制：**
- 模拟退火在当前数据集上未表现优势（已禁用）
- 未考虑 UAV 动力学约束
- 代价权重为人工设定
- 风况未融入路径规划（仅在能耗评估）

---

## 🤝 贡献指南

1. 新功能添加到 `core/` 模块
2. 演示代码添加到 `demo/`
3. 保持 `planner/powerline_planner_v3_final.py` 向后兼容
4. 旧代码归档到 `archive/`
5. 更新相关文档（README_CURRENT_FLOW.md 和 PROJECT_STRUCTURE.md）

---

**文档维护：** 本文档随代码演进持续更新
**最后更新：** 2026-04-06
**维护者：** UAV 巡检项目组
