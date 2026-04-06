# 阶段1：多独立线路识别与巡检点生成

## 概述

本阶段在现有 UAV 电网巡检仿真项目基础上，新增"多条独立红色电网线路"识别能力，每条线路保持独立，不自动连接。

## 新增模块

### 1. core/independent_lines.py

**功能**：从 skeleton 中提取多条独立红色电网线路

**主要数据结构**：
```python
@dataclass
class IndependentLine:
    id: str                              # 线路唯一标识（如 L_001）
    raw_pixels: List[Tuple[int, int]]    # 原始骨架像素
    ordered_pixels: List[Tuple[int, int]] # 排序后的有序折线
    length_2d: float                      # 2D 长度（像素）
    bbox: Tuple[int, int, int, int]      # 边界框
    component_size: int                   # 连通域大小
    endpoints: List[Tuple[int, int]]      # 端点列表
    is_valid: bool = True                 # 是否有效
```

**主要函数**：
- `extract_independent_line_components(skeleton, min_pixels)` - 提取所有独立连通域
- `order_component_as_polyline(line)` - 将单条连通域排序成有序 polyline
- `extract_independent_lines_from_skeleton(skeleton, min_pixels, sort_polylines)` - 完整流程

### 2. core/inspection_point_generator.py

**功能**：为多条独立线路生成巡检点

**主要数据结构**：
```python
@dataclass
class LineInspectionPoint:
    id: str                                    # 巡检点唯一标识
    line_id: str                               # 所属线路ID
    point_type: str                            # 点类型（endpoint/turning/sample）
    pixel_position: Tuple[float, float]        # 2D 像素坐标
    position_3d: Optional[Tuple[float, float, float]] # 3D 坐标
    line_index: int                            # 在线路中的索引
    priority: str                              # 优先级（high/normal）
    image_path: Optional[str] = None           # 图像路径（预留）
    detection_result: Optional[dict] = None    # 检测结果（预留）
    status: str = "uninspected"                # 状态
    source_reason: str                         # 生成原因
```

**巡检点生成规则**：
- **端点（endpoint）**：起点和终点，high priority
- **拐点（turning）**：角度变化大于阈值的点，high priority
- **采样点（sample）**：长线段等距采样，normal priority

**主要函数**：
- `detect_turning_points(polyline, angle_threshold_deg)` - 检测拐点
- `sample_points_along_polyline(polyline, spacing)` - 等距采样
- `generate_line_inspection_points(line, terrain, flight_height, spacing, angle_threshold_deg)` - 单条线路巡检点生成
- `generate_all_inspection_points(lines, terrain, flight_height, spacing, angle_threshold_deg)` - 批量生成
- `convert_line_points_to_legacy_format(points_by_line)` - 转换为旧格式

## 修改的模块

### 1. planner/powerline_planner_v3_final.py

**新增属性**（增量兼容，不删除旧属性）：
```python
self.independent_lines = []               # List[IndependentLine]
self.line_inspection_points = []         # List[LineInspectionPoint]
self.line_inspection_points_by_line = {} # Dict[line_id, List[LineInspectionPoint]]
self.primary_line_id = None              # 主线路ID（最长线路）
```

**新增方法**：
- `step4_extract_independent_lines(min_pixels=20)` - 提取多条独立线路
- `step5_generate_line_inspection_points(spacing, angle_threshold_deg)` - 生成巡检点
- `step6_map_line_points_to_3d(terrain_raw, gaussian_sigma, enhance_resolution)` - 映射到3D

**新增便捷函数**：
- `plan_powerline_independent_lines_v1(...)` - 多独立线路规划完整流程

### 2. core/inspection_tasks.py

**新增适配函数**：
- `convert_line_points_to_legacy_inspection_points(line_points_by_line)` - 转换为旧 InspectionPoint 格式
- `create_inspection_points_from_independent_lines(planner)` - 从规划器创建兼容点
- `merge_multi_line_to_single_path(line_points_by_line, sort_by_length)` - 合并为单路径

## 使用方式

### 方式1：使用新便捷函数

```python
from planner.powerline_planner_v3_final import plan_powerline_independent_lines_v1
from core.terrain import TerrainGenerator
from PIL import Image

# 加载图像
img = Image.open("data/map_power1.png")
w, h = img.size

# 生成地形
terrain = TerrainGenerator.generate_realistic_terrain(w, h)

# 执行多独立线路规划
planner = plan_powerline_independent_lines_v1(
    image_path="data/map_power1.png",
    terrain_raw=terrain,
    flight_height=25,
    min_pixels=20,              # 过滤噪声
    inspection_spacing=80.0,    # 采样间距
    angle_threshold_deg=25.0    # 拐点阈值
)

# 访问结果
print(f"识别到 {len(planner.independent_lines)} 条独立线路")
print(f"生成 {len(planner.line_inspection_points)} 个巡检点")

# 按线路访问
for line_id, points in planner.line_inspection_points_by_line.items():
    print(f"{line_id}: {len(points)} 个巡检点")
```

### 方式2：逐步调用

```python
from planner.powerline_planner_v3_final import PowerlinePlannerV3

planner = PowerlinePlannerV3("data/map_power1.png", flight_height=25)

# 步骤1-3：提取骨架（复用现有逻辑）
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()

# 步骤4：提取多条独立线路（新功能）
planner.step4_extract_independent_lines(min_pixels=20)

# 步骤5：生成巡检点（新功能）
planner.step5_generate_line_inspection_points(
    spacing=80.0,
    angle_threshold_deg=25.0
)

# 步骤6：映射到3D（新功能）
planner.step6_map_line_points_to_3d(terrain)
```

### 方式3：运行 Demo

```bash
python demo/demo_independent_lines_stage1.py
```

## 兼容性说明

### 旧功能保持不变

- 所有现有方法（`step1_extract_redline_hsv` 等）保持原样
- 旧便捷函数 `plan_powerline_inspection_v3()` 继续工作
- 旧 demo 不会受影响
- 现有可视化结构不被破坏

### 新旧数据转换

如果需要使用旧可视化模块处理多线路数据：

```python
from core.inspection_tasks import create_inspection_points_from_independent_lines

# 转换为旧 InspectionPoint 格式
legacy_points = create_inspection_points_from_independent_lines(planner)

# 用于现有可视化
from core.visualization_enhanced import create_animation_view
create_animation_view(path_3d, legacy_points, "output.html")
```

## 输出文件

- `result/step1_hsv_mask.png` - HSV 提取的红色区域
- `result/step2_fixed_mask.png` - 修复后的掩码
- `result/step3_skeleton.png` - 骨架化结果
- `result/step4_independent_lines.png` - 多条独立线路（不同颜色标注）
- `result/step5_line_inspection_points.png` - 巡检点可视化

## 数据结构关系

```
skeleton (二值图像)
    ↓
extract_independent_line_components()
    ↓
List[IndependentLine] (未排序)
    ↓
order_component_as_polyline()
    ↓
List[IndependentLine] (已排序)
    ↓
generate_all_inspection_points()
    ↓
List[LineInspectionPoint] + Dict[line_id, List[LineInspectionPoint]]
```

## 巡检点优先级

- **高优先级（high）**：
  - 端点（endpoint）
  - 拐点（turning）

- **普通优先级（normal）**：
  - 采样点（sample）

## 预留字段

`LineInspectionPoint` 预留了以下字段供后续阶段使用：

- `image_path` - 采集的图像路径
- `detection_result` - 检测结果（如绝缘子识别结果）

## 后续扩展方向

1. **阶段2**：图像采集与检测
   - 使用 `image_path` 字段存储采集图像
   - 使用 `detection_result` 存储检测结果

2. **阶段3**：多机协同
   - 基于线路分配任务
   - 协同路径规划

3. **可视化增强**：
   - 多线路同时显示
   - 分层展示不同线路的巡检点

## 注意事项

1. **独立性**：线路之间保持独立，不自动连接
2. **噪声过滤**：通过 `min_pixels` 参数过滤短线段
3. **参数调优**：
   - `min_pixels`：一般设为 20-50
   - `inspection_spacing`：根据实际需求，一般 80-150 像素
   - `angle_threshold_deg`：一般 20-30 度
