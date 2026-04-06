# 阶段2：任务优化

## 概述

阶段2在阶段1的基础上，新增**任务优化层**，实现多独立线路的全局巡检路径规划。

---

## 核心设计

### 1. TaskLayer 架构

```
┌─────────────────────────────────────────────┐
│           阶段2：任务优化层                  │
├─────────────────────────────────────────────┤
│  tasks  →  cost_mat  →  ord  →  dir        │
│   ↓           ↓          ↓       ↓          │
│ 任务单元   代价矩阵    顺序    方向          │
│   ↓           ↓          ↓       ↓          │
│  g_path_2d, g_path_3d, g_stats              │
└─────────────────────────────────────────────┘
           ↕ (复用阶段1结果)
┌─────────────────────────────────────────────┐
│           阶段1：独立线路层                  │
├─────────────────────────────────────────────┤
│  independent_lines  →  line_inspection_pts  │
└─────────────────────────────────────────────┘
```

### 2. 命名规范

| 旧命名（阶段1） | 新命名（阶段2） |
|----------------|----------------|
| `LineTask` | `TaskLine` |
| `global_mission_path_2d` | `g_path_2d` |
| `global_mission_path_3d` | `g_path_3d` |
| `optimized_line_order` | `line_ord` |
| `optimized_line_directions` | `line_dir` |
| `build_line_tasks` | `build_tasks` |
| `solve_line_order_nearest_neighbor` | `init_ord_nn` |
| `improve_line_order_2opt` | `opt_ord_2opt` |
| `optimize_line_directions` | `opt_line_dir` |
| `build_global_mission_path` | `build_g_path` |

---

## 新增模块

### 1. core/costs.py

成本计算模块，提供通用的代价函数。

```python
dist_cost(p1, p2)              # 欧氏距离
turn_cost(p0, p1, p2)          # 转向角
wind_cost(p1, p2, wind)        # 风向影响
trans_cost(task_a, dir_a, task_b, dir_b, ...)  # 任务间切换
path_cost(path, wind)          # 路径总成本
```

**成本公式**：
```
cost = distance + 0.3 × turn + 0.2 × wind
```

---

### 2. core/mission_opt.py

任务层优化模块。

#### TaskLine 数据结构

```python
@dataclass
class TaskLine:
    id: str                  # 任务ID
    kind: str                # 类型："line" | 未来 "branch"/"seg"/"edge"
    line_id: str             # 原始线路ID
    pts: List                # 巡检点序列
    p_start: Tuple           # 起点
    p_end: Tuple             # 终点
    len2d: float             # 长度
    n_pts: int               # 点数
    meta: Dict               # 预留拓扑扩展
```

#### meta 预留字段

```python
meta = {
    'comp_id': None,         # 所属连通域
    'topo_id': None,         # 拓扑ID
    'parent': None,          # 父节点
    'children': [],          # 子节点
    'branch_flag': False     # 是否分支
}
```

#### 核心函数

```python
build_tasks(lines, pts_by_line)        # 从阶段1构建task
init_ord_nn(tasks, cost_mat, start_pos)  # 最近邻初始顺序
opt_ord_2opt(ord0, cost_mat)            # 2-opt改进
opt_line_dir(ord1, tasks, ...)          # 方向优化
```

---

### 3. core/mission_build.py

路径拼接模块。

```python
expand_task(task, dir_flag, pts_by_line)    # 展开任务为点序列
conn_seg(p_end, p_start)                    # 连接两段
build_g_path(ord1, dir_map, tasks, ...)     # 构建全局路径
```

---

## Planner 增量改造

### 新增属性

```python
self.tasks          # List[TaskLine]
self.cost_mat       # np.ndarray
self.line_ord       # List[int]
self.line_dir       # Dict[str, int]
self.g_path_2d      # List[Tuple]
self.g_path_3d      # List[Tuple]
self.g_stats        # Dict
```

### 新增方法

```python
def step7_build_tasks()         # 构建任务层
def step8_opt_mission(...)      # 优化顺序+方向
def step9_build_g_path(...)     # 生成全局路径
```

### 新增入口函数

```python
def plan_powerline_mission_v2(
    image_path, terrain_raw,
    flight_height=30, ...
) -> PowerlinePlannerV3
```

---

## API 使用

### 快速开始

```python
from planner.powerline_planner_v3_final import plan_powerline_mission_v2

planner = plan_powerline_mission_v2(
    image_path="data/test.png",
    terrain_raw=terrain,
    flight_height=25
)

# 访问结果
print(f"任务顺序: {[planner.tasks[i].id for i in planner.line_ord]}")
print(f"任务方向: {planner.line_dir}")
print(f"全局路径长度: {planner.g_stats['total_len']:.1f}px")
```

### 分步调用

```python
planner = PowerlinePlannerV3("data/test.png")

# 阶段1
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()
planner.step4_extract_independent_lines()
planner.step5_generate_line_inspection_points()
planner.step6_map_line_points_to_3d(terrain)

# 阶段2
planner.step7_build_tasks()
planner.step8_opt_mission()
planner.step9_build_g_path()
```

---

## 算法说明

### 任务顺序优化

1. **初始顺序**：最近邻贪心算法
2. **顺序改进**：2-opt 局部搜索

### 任务方向优化

对每个任务选择使切换成本最小的方向：
- 正向（dir=1）：从起点到终点
- 反向（dir=-1）：从终点到起点

### 路径拼接

按优化后的顺序和方向拼接所有任务，生成全局路径。

---

## 未来升级路径

### 从独立线路到拓扑图

| 当前（阶段2） | 未来（拓扑优化） |
|--------------|----------------|
| `TaskLine.kind = "line"` | `"branch"` / `"seg"` / `"edge"` |
| `build_tasks(lines, ...)` | `build_tasks(topo_graph, ...)` |
| `cost_mat[i,j]` (line→line) | `cost_mat[i,j]` (task→task) |

**无需推翻**：
- `dist_cost()`, `turn_cost()`, `wind_cost()` 保持不变
- `init_ord_nn()`, `opt_ord_2opt()` 保持不变
- `build_g_path()` 结构保持不变

---

## Demo 运行

```bash
python demo/demo_mission_stage2.py
```

**输出**：
- 任务数量
- 任务顺序
- 任务方向
- 全局路径长度
- 总成本
- result/step9_g_path.png
