# Mission JSON 导出与 Inspection Point 交互功能说明

**更新时间：** 2026-03-30

---

## 一、功能概述

本次更新为 Step9_2 基线版本添加了两个新功能：

1. **标准 Mission JSON 导出** - 将分组连续任务导出为标准 JSON 格式
2. **Inspection Point 交互** - 在 3D 可视化中支持点击 inspection point 查看详情

**重要约束：**
- ✅ 未修改 Step9_2 算法逻辑
- ✅ 未修改 planner 行为
- ✅ 仅添加导出和可视化功能

---

## 二、新增文件

| 文件 | 说明 |
|------|------|
| `demo/demo_ui_animation.py` | 更新：添加 JSON 导出和 inspection point 交互 |

---

## 三、修改文件

### 3.1 `core/topo_plan.py`

**新增函数：**

#### `export_grouped_mission_to_json()`
```python
def export_grouped_mission_to_json(
    mission: GroupedContinuousMission,
    edge_tasks: List[EdgeTask],
    line_inspection_points_by_line: Dict[str, List],
    output_path: str = "result/latest/mission_output.json",
    terrain_3d: Optional[np.ndarray] = None
) -> str
```

功能：将分组连续任务导出为标准 JSON 格式

参数：
- `mission`: 分组连续任务对象
- `edge_tasks`: 边任务列表
- `line_inspection_points_by_line`: {line_id: [巡检点列表]}
- `output_path`: 输出文件路径
- `terrain_3d`: 3D地形数组（可选，用于生成3D坐标）

返回：输出文件的完整路径

#### `load_mission_json_for_ui()`
```python
def load_mission_json_for_ui(json_path: str) -> Dict[str, Any]
```

功能：从 JSON 文件加载任务数据用于 UI 显示

**修改点：**
- 在文件末尾添加了约 450 行新代码
- 无任何对现有代码的修改

### 3.2 `core/visualization_enhanced.py`

**新增函数：**

#### `create_animation_view_with_inspection_points()`
```python
def create_animation_view_with_inspection_points(
    path_3d,
    mission_json_path,
    edge_tasks=None,
    output_file="output/animation_with_inspection.html",
    downsample_step=3
)
```

功能：创建带 inspection point 交互的 UAV 巡检动画

特性：
- 3D 路径动画
- 显示 inspection points（可点击）
- 点击点后显示信息弹窗
- 支持导航输入

**修改点：**
- 在文件末尾添加了约 330 行新代码
- 无任何对现有代码的修改

### 3.3 `planner/powerline_planner_v3_final.py`

**修改函数：**

#### `step6_smooth_terrain()`

在函数末尾添加了 `self.terrain_3d` 的创建代码，用于 UI 可视化：

```python
# 创建 terrain_3d 数组 (height, width, 3) 用于 UI 可视化
h, w = self.height_map_smooth.shape
self.terrain_3d = np.zeros((h, w, 3), dtype=np.float32)
y_coords, x_coords = np.mgrid[0:h, 0:w]
self.terrain_3d[:, :, 0] = x_coords.astype(np.float32)  # x 坐标
self.terrain_3d[:, :, 1] = y_coords.astype(np.float32)  # y 坐标
self.terrain_3d[:, :, 2] = self.height_map_smooth       # z 坐标（高度）
```

---

## 四、JSON 输出结构

### 4.1 文件位置

```
result/latest/mission_output.json
```

### 4.2 顶层结构

```json
{
  "metadata": { ... },
  "statistics": { ... },
  "groups": [ ... ],
  "visit_order": { ... },
  "segments": [ ... ],
  "inspection_points": [ ... ],
  "full_path": { ... }
}
```

### 4.3 元信息 (metadata)

```json
{
  "version": "1.0.0",
  "planner_name": "Step9_2_GroupedContinuousPlanner",
  "timestamp": "2026-03-30T09:26:08.056025",
  "export_date": "2026-03-30",
  "export_time": "09:26:08"
}
```

### 4.4 统计信息 (statistics)

```json
{
  "total_length": 3731.1,
  "inspect_length": 1273.5,
  "connect_length": 2457.6,
  "inspect_ratio": 34.1,
  "num_groups": 4,
  "num_edges": 12,
  "num_segments": 23,
  "num_inspection_points": 139,
  "intra_group_connect_length": 1234.5,
  "inter_group_connect_length": 1223.1
}
```

### 4.5 分组信息 (groups)

每个 group 包含：

```json
{
  "group_id": "Group_0",
  "edge_ids": ["edge_001", "edge_002"],
  "total_inspect_length": 350.5,
  "centroid": { "x": 1200.5, "y": 800.3 },
  "bbox": {
    "min_x": 1050.2,
    "min_y": 650.1,
    "max_x": 1350.8,
    "max_y": 950.6
  }
}
```

### 4.6 访问顺序 (visit_order)

```json
{
  "edge_visit_order": ["edge_001", "edge_002", ...],
  "edge_direction": {
    "edge_001": "forward",
    "edge_002": "reverse"
  },
  "group_visit_order": ["Group_0", "Group_1", ...]
}
```

### 4.7 路径段 (segments)

每个 segment 包含：

```json
{
  "segment_id": "seg_0000",
  "type": "inspect",
  "edge_id": "edge_001",
  "group_id": "Group_0",
  "length": 150.5,
  "direction": "forward",
  "geometry_2d": [[100.0, 200.0], [150.0, 250.0], ...],
  "geometry_3d": [[100.0, 200.0, 25.3], [150.0, 250.0, 25.5], ...]
}
```

### 4.8 Inspection Points

每个 point 包含：

```json
{
  "point_id": "IP_00001",
  "edge_id": "edge_001",
  "group_id": "Group_0",
  "line_id": "line_001",
  "point_type": "endpoint",
  "pixel_position": [120.5, 230.8],
  "position_3d": [120.5, 230.8, 25.4],
  "visit_order": 1,
  "priority": "high",
  "image_ref": null,
  "detection_result": null,
  "status": "uninspected",
  "source_reason": "线路端点"
}
```

**字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `point_id` | string | 唯一标识 (格式: IP_XXXXX) |
| `edge_id` | string | 所属边 ID |
| `group_id` | string | 所属分组 ID |
| `line_id` | string | 所属线路 ID |
| `point_type` | string | 点类型 (endpoint/turning/sample) |
| `pixel_position` | [x, y] | 2D 像素坐标 |
| `position_3d` | [x, y, z] | 3D 坐标（如果可用） |
| `visit_order` | int | 访问顺序 |
| `priority` | string | 优先级 (high/normal) |
| `image_ref` | string | 图片引用（可为 null） |
| `detection_result` | object | 检测结果（可为 null） |
| `status` | string | 状态 |

### 4.9 完整路径 (full_path)

```json
{
  "full_path_2d": [[100.0, 200.0], [150.0, 250.0], ...],
  "full_path_3d": [[100.0, 200.0, 25.3], [150.0, 250.0, 25.5], ...],
  "num_waypoints": 820
}
```

---

## 五、UI 交互功能

### 5.1 Inspection Point 显示

在 3D 可视化中，inspection points 按类型显示不同颜色和形状：

| 类型 | 颜色 | 形状 |
|------|------|------|
| endpoint | 红色 | 菱形 |
| turning | 橙色 | 圆形 |
| sample | 青色 | 圆形 |

### 5.2 点击交互

点击任意 inspection point 后，右侧显示详情面板，包含：

**基本信息：**
- 点 ID
- Edge ID
- Group ID
- Line ID
- 类型
- 访问顺序
- 优先级
- 像素位置
- 3D 位置

**图片区域：**
- 如果 `image_ref` 不为空：显示图片
- 如果为空：显示 "暂无图片"

**检测结果：**
- 如果 `detection_result` 不为空：显示 JSON 格式结果
- 如果为空：显示 "暂无检测结果"

### 5.3 面板样式

详情面板特点：
- 固定在右上角
- 半透明白色背景
- 蓝色边框
- 可通过"关闭"按钮关闭

---

## 六、如何运行

### 6.1 生成 JSON 和 UI

```bash
python demo/demo_ui_animation.py
```

### 6.2 输出文件

运行后会生成以下文件：

```
result/latest/
├── mission_output.json         # 标准任务 JSON
├── animation.html               # 交互式动画（带 inspection point）
├── 01_extraction.png           # 各阶段结果图
├── 02_morphology.png
...
```

### 6.3 查看结果

1. **查看 JSON：**
   ```bash
   cat result/latest/mission_output.json
   ```

2. **查看 UI：**
   - 在浏览器中打开 `result/latest/animation.html`
   - 点击 inspection point 查看详情
   - 使用播放按钮观看 UAV 飞行动画

---

## 七、验证 Step9_2 基线不变

### 7.1 未修改的内容

- ✅ `build_grouped_continuous_mission()` 算法逻辑
- ✅ `group_edges_spatially()` 分组逻辑
- ✅ `order_groups_greedy()` 排序逻辑
- ✅ `build_mission_segments()` 路径构建逻辑
- ✅ 任何影响规划结果的参数

### 7.2 仅新增的内容

- JSON 导出函数（不影响规划）
- UI 交互可视化函数（不影响规划）
- `terrain_3d` 属性（仅用于可视化）

### 7.3 验证方式

运行基线 demo，结果应与之前完全一致：

```bash
python demo/demo_topo_plan_grouped.py
```

输出结果应该相同：
- 分组数量
- 边访问顺序
- 路径长度
- 巡检比例

---

## 八、API 使用示例

### 8.1 导出 JSON

```python
from core.topo_plan import export_grouped_mission_to_json

json_path = export_grouped_mission_to_json(
    mission=mission,
    edge_tasks=edge_tasks,
    line_inspection_points_by_line=line_inspection_points_by_line,
    output_path="result/latest/mission_output.json",
    terrain_3d=planner.terrain_3d
)
```

### 8.2 加载 JSON

```python
from core.topo_plan import load_mission_json_for_ui

mission_data = load_mission_json_for_ui("result/latest/mission_output.json")

# 访问数据
stats = mission_data['statistics']
groups = mission_data['groups']
inspection_points = mission_data['inspection_points']
```

### 8.3 创建交互式 UI

```python
from core.visualization_enhanced import create_animation_view_with_inspection_points

create_animation_view_with_inspection_points(
    path_3d=path_3d,
    mission_json_path="result/latest/mission_output.json",
    edge_tasks=edge_tasks,
    output_file="result/latest/animation.html",
    downsample_step=3
)
```

---

## 九、故障排除

### 9.1 JSON 导出失败

**问题：** `AttributeError: 'GroupedContinuousMission' object has no attribute 'edge_to_group'`

**解决：** JSON 导出函数会自动构建 `edge_to_group` 属性，无需手动处理。

### 9.2 Inspection Point 不显示

**问题：** UI 中没有显示 inspection points

**检查：**
1. JSON 文件是否正确生成
2. `inspection_points` 数组是否为空
3. 检查 `edge_tasks` 是否包含 `inspection_points`

### 9.3 点击无反应

**问题：** 点击 inspection point 后没有弹出详情

**检查：**
1. 浏览器控制台是否有 JavaScript 错误
2. Plotly 版本是否正确
3. 检查 `customdata` 是否正确传递

---

**文档维护者：** Claude
**最后更新：** 2026-03-30
