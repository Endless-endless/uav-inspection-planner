# UAV 电网巡检项目 - 当前主流程说明

**更新时间：** 2026-04-06
**项目状态：** 主线稳定，基于拓扑结构的全局优化路径规划
**核心特性：** 拓扑图建模 + 分组连续巡检 + 自动择优算法

---

## 一、当前项目主线流程

### 1.1 核心处理流程

```
输入图像 (data/test.png)
    ↓
Step 1: HSV 红色提取
    ↓
Step 2: 形态学修复（膨胀+腐蚀）
    ↓
Step 3: 骨架化提取
    ↓
Step 4: 独立线路识别 (多连通域分离)
    ↓
Step 5: 巡检点生成 (端点/拐点/采样点)
    ↓
Step 6: 3D 地形映射
    ↓
Step 7.5: 拓扑图建模
    ├─ 节点检测（端点 + 角度/长度切分点）
    ├─ 边切分（按节点分割线路）
    ├─ 节点合并（空间聚类去重）
    └─ 图构建（邻接表结构）
    ↓
Step 8.5: EdgeTask 建模
    └─ 巡检点映射到拓扑边
    ↓
Step 9.4: 全局拓扑优化路径规划
    ├─ [方案1] 贪心分组规划（旧算法）
    ├─ [方案2] 全局拓扑优化（新算法）
    │   ├─ 空间分组（DBSCAN 聚类）
    │   ├─ 起始边优化（枚举候选）
    │   ├─ 边顺序优化（模拟退火/贪心）
    │   ├─ 方向选择（端点优先）
    │   └─ 路径生成（inspect + connect 段）
    └─ [自动择优] 比较选择更优方案
    ↓
输出: mission_output.json + 交互式可视化页面
```

### 1.2 核心数据结构

```
独立线路 (IndependentLine)
    ↓ [节点检测]
拓扑节点 (TopoNode)
    ├─ endpoint: 线路端点
    └─ split: 角度/长度切分点
    ↓ [边切分]
拓扑边 (TopoEdge)
    └─ polyline: 骨架点序列
    ↓ [图构建]
拓扑图 (TopoGraph)
    └─ adj: 邻接表 {node_id: [neighbor_ids]}
    ↓ [点边映射]
边任务 (EdgeTask)
    └─ inspection_points: 该边的巡检点
    ↓ [分组优化]
分组连续任务 (GroupedContinuousMission)
    ├─ segments: [inspect段, connect段, ...]
    ├─ groups: 空间分组信息
    └─ visit_order: 边访问顺序
    ↓ [JSON导出]
标准 JSON 输出 (mission_output.json)
```

### 1.3 路径段类型说明

| 段类型 | type 字段 | 说明 | 生成方式 |
|-------|----------|------|---------|
| **巡检段** | `inspect` | 沿拓扑边进行巡检的路径 | 直接使用边的 polyline |
| **连接段** | `connect` | 不同拓扑边之间的转移路径 | BFS 沿拓扑图找最短路径 |

---

## 二、拓扑路径查找算法详解

### 2.1 图模型

**数据结构：** 无向图 + 邻接表表示
- **节点（TopoNode）**：拓扑节点，包含位置、度数、类型（endpoint/split）
- **边（TopoEdge）**：拓扑边，连接两个节点，包含 polyline 和长度
- **图（TopoGraph）**：邻接表 `adj: Dict[str, List[str]]`

### 2.2 核心算法

| 组件 | 算法 | 代码位置 | 说明 |
|-----|-----|---------|-----|
| **边顺序优化** | 模拟退火（已禁用）→ 贪心最近邻 | topo_global_optimizer.py:408 | 实际使用贪心策略 |
| **节点间路径** | BFS（广度优先搜索） | topo_global_optimizer.py:139 | 找拓扑图最短路径 |
| **空间分组** | DBSCAN 聚类 | topo_plan.py:1967 | eps=150.0 |
| **连接代价** | 综合代价模型 | topo_global_optimizer.py:48 | 几何+拓扑+方向+组切换 |

### 2.3 代价函数

```python
# 综合连接代价（compute_connection_cost_enhanced）
total_cost = (
    1.0 * geometric_distance +      # 几欧氏距离
    0.1 * topo_path_length +         # 拓扑图路径长度
    0.1 * direction_change +         # 方向变化惩罚 (0-1)
    group_switch_penalty             # 组切换惩罚 (10.0)
)
```

### 2.4 优化目标（三级择优）

1. **一级目标**：最小化 `connect_length`（连接段总长度）
2. **二级目标**：最小化 `connect_segment` 数量
3. **三级目标**：最小化 `total_length`

### 2.5 算法性质

| 维度 | 评估 | 说明 |
|-----|-----|-----|
| **理论保证** | ❌ 无全局最优保证 | 模拟退火是启发式，贪心是局部最优 |
| **实际运行** | ⚠️ 贪心策略 | `enable_sa=False`，使用改进的贪心算法 |
| **约束处理** | ✅ 拓扑约束 | 相邻边连接代价=0，体现图结构 |
| **UAV 动力学** | ❌ 未考虑 | 无转弯半径、速度变化等约束 |

---

## 三、当前主线 Demo

### 3.1 推荐入口脚本

| 文件 | 说明 |
|------|------|
| **demo/demo_visualization_main.py** | 🌟 **完整流程入口**，生成 JSON + 可视化 |
| **demo/generate_interactive_main_view.py** | 生成交互式主展示页 |

### 3.2 推荐使用方式

**标准用法（推荐）：**
```bash
python demo/demo_visualization_main.py
```

**输出文件：**
- `result/latest/mission_output.json` - 标准任务 JSON
- `result/latest/main_view_interactive.html` - 交互式主展示页
- `result/latest/map_overlay_test.png` - 2D 地图叠加

**天气场景配置（可选）：**
```python
# 在 demo_visualization_main.py 中修改
weather_scene = "calm"  # calm, crosswind, headwind_strong, tailwind_efficient, gusty_high_risk
```

---

## 四、核心模块说明

### 4.1 规划器

| 文件 | 说明 | 核心功能 |
|------|------|---------|
| **planner/powerline_planner_v3_final.py** | 🌟 主规划器 | 整合所有步骤，提供完整规划流程 |
| **planner/astar3d.py** | A* 路径规划 | 3D 空间路径规划（备用） |
| **planner/powerline_inspector.py** | 巡检器 | 巡检点生成和管理 |

### 4.2 核心拓扑模块

| 文件 | 说明 | 核心类/函数 |
|------|------|-----------|
| **core/topo.py** | 🔧 拓扑图建模 | `TopoNode`, `TopoEdge`, `TopoGraph`, `build_topo_graph` |
| **core/topo_task.py** | 边任务建模 | `EdgeTask`, `build_edge_tasks` |
| **core/topo_plan.py** | 拓扑规划逻辑 | `MissionSegment`, `GroupedContinuousMission`, `build_edge_adjacency_simple` |
| **core/topo_global_optimizer.py** | 🔧 全局优化器 | `optimize_edge_order_simulated_annealing`, `compute_connection_cost_enhanced` |

### 4.3 可视化模块

| 文件 | 说明 |
|------|------|
| **core/visualization_enhanced.py** | 可视化增强模块 |
| **visualization/plotly_viewer.py** | Plotly 3D 可视化 |
| **visualization/map_overlay.py** | 地图叠加可视化 |

---

## 五、功能特性

### 5.1 拓扑建模 ✅
- ✅ 自动检测端点节点（线路端点）
- ✅ 智能切分节点（角度 > 60° 或长度 > 600px）
- ✅ 节点空间聚类合并（距离 < 25px）
- ✅ 邻接表图结构构建
- ✅ 连通分量统计

### 5.2 路径规划 ✅
- ✅ 基于拓扑图的路径查找（BFS 最短路径）
- ✅ 空间分组优化（DBSCAN 聚类）
- ✅ 边访问顺序优化（模拟退火/贪心）
- ✅ 自动择优机制（新旧算法对比）
- ✅ 起始边智能选择

### 5.3 连接段生成 ✅
- ✅ 使用拓扑图路径（非退化直线）
- ✅ 支持长距离连接（动态阈值）
- ✅ Geometry 为真实多点 polyline
- ✅ 插值密集路径点（step_size=10px）
- ✅ 支持不连通图的邻近性虚拟边

### 5.4 JSON 输出 ✅
- ✅ 完整的 groups 信息
- ✅ edge_visit_order（边访问顺序）
- ✅ segments（inspect + connect 段列表）
- ✅ inspection_points（巡检点详情）
- ✅ full_path_2d / full_path_3d（完整路径）
- ✅ 统计信息（长度、段数、分组信息）
- ✅ 天气信息（风况、风险等级）

### 5.5 天气影响 ✅
- ✅ 多种天气场景（calm, crosswind, gusty 等）
- ✅ 风速/风向配置
- ✅ 风险等级评估
- ✅ 能耗因子计算
- ✅ 天气统计导出

---

## 六、数据源

**当前统一使用：** `data/test.png` (916 x 960 pixels)

**支持格式：**
- PNG/JPG 电网图像
- 红色线路自动识别
- 支持多条独立电路

---

## 七、项目清理说明

### 7.1 已归档
- 冗余 demo 脚本 → `archive/deprecated_20260401/demo/`
- 未使用的 core 模块 → `archive/deprecated_20260401/core/`
- 调试脚本 → `archive/debug_scripts/`

### 7.2 当前最小必要文件集

**核心规划：**
- `planner/powerline_planner_v3_final.py` - 主规划器
- `planner/astar3d.py` - A* 路径规划
- `planner/node.py` - 节点定义

**拓扑模块：**
- `core/topo.py` - 拓扑图定义
- `core/topo_task.py` - EdgeTask 定义
- `core/topo_plan.py` - 拓扑规划逻辑
- `core/topo_global_optimizer.py` - 全局优化器

**演示入口：**
- `demo/demo_visualization_main.py` - 完整流程入口
- `demo/generate_interactive_main_view.py` - 交互式页面生成
- `demo/demo_ui_animation.py` - 辅助导航页

---

## 八、版本说明

| 版本 | 状态 | 说明 | 核心特性 |
|------|------|------|---------|
| **Step 7.5** | ✅ 主线 | 拓扑图建模 | 节点检测、边切分、图构建 |
| **Step 8.5** | ✅ 主线 | EdgeTask 建模 | 点边映射、任务构建 |
| **Step 9.4** | ✅ **当前版本** | 全局拓扑优化 | 新旧算法对比、自动择优 |

### 当前版本特性（Step 9.4）

**核心改进：**
1. 全局拓扑优化算法（基于论文思想）
2. 综合代价模型（几何+拓扑+方向+组切换）
3. 自动择优机制（新旧算法对比）
4. 模拟退火优化（可配置，默认关闭）

**算法本质：**
- 启发式局部最优（非全局最优保证）
- 基于拓扑图的贪心策略
- BFS 最短路径查找
- DBSCAN 空间分组

**已知限制：**
- 模拟退火在当前数据集上未表现优势（已禁用）
- 未考虑 UAV 动力学约束
- 代价权重为人工设定
- 风况未融入路径规划（仅在能耗评估）

---

## 九、快速开始

### 9.1 环境依赖

```bash
# 核心依赖
pip install numpy scipy matplotlib pillow

# 可选依赖（用于高级功能）
pip install scikit-learn  # DBSCAN 分组
pip install scikit-image  # 图像处理
pip install plotly        # 交互式可视化
```

### 9.2 运行示例

```bash
# 完整流程（推荐）
python demo/demo_visualization_main.py

# 查看结果
# 输出文件在 result/latest/ 目录
```

### 9.3 代码示例

```python
from planner.powerline_planner_v3_final import PowerlinePlannerV3

# 创建规划器
planner = PowerlinePlannerV3(
    image_path='data/test.png',
    flight_height=30,
    weather_scene='calm'
)

# 执行规划
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()
planner.step4_extract_independent_lines()
planner.step5_generate_line_inspection_points()
planner.step7_5_build_topo()
planner.step8_5_build_edge_tasks()

# 全局优化路径规划
mission = planner.step9_4_plan_global_topology_optimized(
    enable_sa=False,  # 使用贪心策略
    eps=150.0
)

print(f"总长度: {mission.total_length:.1f}px")
print(f"巡检: {mission.inspect_length:.1f}px")
print(f"连接: {mission.connect_length:.1f}px")
```

---

## 十、后续改进方向

### 方向1：启用并优化模拟退火 🔥
- 调整参数：initial_temp=5000, cooling_rate=0.99, iterations_per_temp=200
- 位置：`topo_global_optimizer.py:408-545`

### 方向2：考虑UAV动力学的路径平滑 🔥
- 添加转弯半径约束
- 使用 B 样条或 Dubins 曲线
- 位置：`topo_plan.py:1098-1200`

### 方向3：自适应代价权重学习 🔥
- 使用强化学习自动学习权重
- 根据历史表现动态调整
- 位置：`topo_global_optimizer.py:48-136`

### 方向4：多起点/多无人机协同
- 任务划分：K-means 聚类
- 协同优化：最小化总体完成时间

### 方向5：实时重规划能力
- 增量更新机制
- 缓存优化
- 优先级队列

---

**文档维护：** 本文档随代码演进持续更新
**最后更新：** 2026-04-06
**维护者：** UAV 巡检项目组
