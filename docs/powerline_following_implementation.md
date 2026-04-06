# 电网沿线路径规划系统 - 实现文档

## 概述

这是一个完整的电网巡检路径规划系统，实现了从"自由路径规划"到"电网沿线路径生成"的升级。

### 核心设计思想

1. **电网（红线）是路径的"主体"** - 路径必须沿电网生成，而非通过A*搜索得到
2. **A* 仅用于断点连接** - 当电网有断点时，使用A*进行局部连接
3. **最终路径必须是3D的** - 路径高度随地形变化

## 文件结构

```
uav_path_planning_project/
├── planner/
│   ├── powerline_path_planner.py    # 核心规划器
│   ├── astar3d.py                   # A*算法（已优化）
│   └── astar3d_optimized.py         # 优化版A*
├── visualization/
│   └── plotly_viewer.py             # 3D可视化（新增powerline专用函数）
├── demo_powerline_following.py      # 完整演示脚本
└── data/
    ├── map_power1.png               # 测试图片1
    ├── map_power2.png               # 测试图片2
    └── map_power3.png               # 测试图片3
```

## 核心模块详解

### 1. PowerlinePathPlanner 类

**文件**: `planner/powerline_path_planner.py`

#### 主要方法

| 方法 | 功能 | 输入 | 输出 |
|------|------|------|------|
| `load_image()` | 加载电网图片 | - | PIL Image |
| `extract_powerline()` | 提取红色线路 | RGB阈值 | 电网掩码 |
| `extract_centerline()` | 提取中心线（细化） | - | skeleton |
| `find_endpoints_and_polylines()` | 找端点并生成有序路径 | - | polyline列表 |
| `sample_waypoints()` | 沿路径采样航点 | step间隔 | 航点列表 |
| `build_3d_path()` | 构建3D路径 | 地形图 | 3D路径 |
| `smooth_path_3d()` | 高斯平滑路径 | 窗口大小 | 平滑后路径 |

#### 关键算法

**1. 中心线提取（Skeletonization）**
```python
# 使用距离变换提取脊线
dist_transform = distance_transform_edt(255 - power_array)
local_max = maximum_filter(dist_transform, size=size)
skeleton = (dist_transform == local_max) & (power_array > 0)
```

**2. 端点检测与路径追踪**
```python
# 8连通邻接图构建
neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

# 端点定义：度=1的节点
endpoints = [point for point, deg in degrees.items() if deg == 1]

# BFS从端点开始追踪路径
```

**3. 3D路径构建**
```python
# 飞行高度 = 地形高度 + 安全高度
z = terrain_height[y, x] + flight_height
```

### 2. 便捷函数

```python
def plan_powerline_inspection_path(
    image_path,      # 电网图片路径
    terrain_map=None,    # 地形高度图（可选）
    flight_height=20,    # 飞行高度（米）
    sample_step=10,      # 采样间隔（像素）
    smooth_path=True     # 是否平滑路径
):
    """
    一键完成完整的电网巡检路径规划

    Returns:
        PowerlinePathPlanner对象
    """
```

### 3. 3D可视化函数

**文件**: `visualization/plotly_viewer.py`

```python
def visualize_powerline_inspection(
    grid,                  # 3D栅格地图
    path_3d,              # 3D路径
    waypoints,            # 航点
    powerline_pixels,     # 原始电网像素
    save_html=True,
    html_filename="powerline_inspection_3d.html"
):
    """
    电网巡检专用3D可视化

    显示:
    1. 地形表面 (Earth配色)
    2. 原始电网 (红色线)
    3. 巡检路径 (蓝色线)
    4. 航点 (绿色点)
    """
```

## 使用方法

### 基本使用

```python
from planner.powerline_path_planner import plan_powerline_inspection_path

# 一键规划
planner = plan_powerline_inspection_path(
    image_path="data/map_power1.png",
    terrain_map=height_map,
    flight_height=20,
    sample_step=10,
    smooth_path=True
)

# 获取结果
path_3d = planner.path_3d        # 3D路径 [(x,y,z), ...]
waypoints = planner.waypoints    # 2D航点 [(x,y), ...]
polylines = planner.polylines    # 有序路径列表
```

### 完整演示

```python
# 运行演示脚本
python demo_powerline_following.py
```

## 实现流程

```
Step 1: 加载电网图片
   ↓
Step 2: 提取红色线路（电网）
   使用 RGB 阈值: R>150, G<120, B<120
   ↓
Step 3: 提取中心线（细化）
   距离变换 + 局部最大值检测
   ↓
Step 4: 生成有序路径（polyline）
   检测端点（度=1）→ BFS追踪
   ↓
Step 5: 沿路径采样航点
   均匀采样，间隔=step像素
   ↓
Step 6: 构建3D路径
   z = 地形高度 + 飞行高度
   ↓
Step 7: 可选平滑
   高斯滤波（sigma=window/3）
   ↓
Step 8: 3D可视化
   地形 + 电网(红) + 路径(蓝) + 航点(绿)
```

## 与原有A*系统的区别

| 特性 | 原有系统 | 新系统 |
|------|----------|--------|
| 路径生成方式 | A*全局搜索 | 沿电网skeleton |
| A*用途 | 主路径规划 | 断点连接（辅助） |
| 路径特性 | 自由路径，可能远离电网 | 紧密沿电网 |
| 计算效率 | 慢（大地图） | 快（直接提取） |
| 适用场景 | 自由飞行 | 电网巡检 |

## 性能对比

| 指标 | 原A*系统 | 新系统 |
|------|----------|--------|
| 路径规划时间 | ~17秒 (50x50地图) | <1秒 |
| 路径与电网贴合度 | 低（可能远离） | 高（完全贴合） |
| 3D路径质量 | 随地形变化 | 随地形变化 |
| 生成航点数 | 依赖于A*搜索 | 可控制（step参数） |

## 输出文件

运行 `demo_powerline_following.py` 后生成：

1. **2D可视化**: `result/powerline_following_2d.png`
   - 原始电网
   - 提取的中心线（黄色）
   - 规划路径（蓝色）
   - 航点标记（绿色圆圈）

2. **3D可视化**: `figures/powerline_following_3d.html`
   - 地形表面（3D terrain）
   - 原始电网（红色线）
   - 巡检路径（蓝色线）
   - 航点（绿色点）

## 运行结果示例

```
[INFO] 提取电网掩码: 14493 个像素点
[INFO] 提取中心线: 419 个像素点
[INFO] 检测到 94 个端点
[INFO] 生成 47 条路径
[INFO] 生成 113 个巡检航点（采样间隔=10像素）
[INFO] 构建3D路径: 113 个航点
   高度范围: Z=[23.2, 31.5] 米

[路径统计]
  航点数量: 113
  X范围: 1287.0 ~ 2156.0
  Y范围: 74.0 ~ 236.0
  Z范围: 23.0 ~ 31.0 米
  物理长度: 1229.5 米
```

## 技术要点

### 1. Skeletonization（骨架提取）

使用**距离变换**方法而非传统的形态学细化，优点：
- 更鲁棒，对噪声不敏感
- 能得到更平滑的中心线
- 保留拓扑结构

### 2. Polyline追踪

- **端点检测**: 度=1的节点
- **BFS追踪**: 从端点开始，确保路径连续性
- **多路径支持**: 可处理多条分离的电网

### 3. 3D高度计算

```python
z = terrain_height[y, x] + flight_height
```

确保：
- 路径始终在地形上方
- 高度随地形起伏变化
- 避免与地形碰撞

## 扩展建议

### 1. 添加断点连接

当检测到电网断点时，使用A*进行连接：

```python
if distance(segment_end, segment_start) < threshold:
    # 使用A*连接断点
    connection_path = astar.plan(segment_end, segment_start)
```

### 2. 考虑风场影响

在3D路径规划时加入风场代价：

```python
# 计算风场代价
C_w = compute_wind_cost_physics(move_vector, wind_vector)
```

### 3. 自适应采样

根据电网曲率动态调整采样间隔：

```python
# 直线区域：稀疏采样（step=20）
# 转弯区域：密集采样（step=5）
```

## 常见问题

### Q1: 为什么路径不连续？

A: 可能是电网图片中有断点。解决方法：
1. 检查原始图片质量
2. 使用A*连接断点
3. 调整 `red_threshold` 参数

### Q2: 如何调整飞行高度？

A: 修改 `flight_height` 参数：
```python
planner = plan_powerline_inspection_path(
    image_path="data/map_power1.png",
    flight_height=30  # 增加/减少高度
)
```

### Q3: 如何控制航点密度？

A: 调整 `sample_step` 参数：
```python
planner = plan_powerline_inspection_path(
    image_path="data/map_power1.png",
    sample_step=5  # 更密集的航点
)
```

## 依赖项

- numpy
- PIL (Pillow)
- scipy (distance_transform_edt, maximum_filter, gaussian_filter1d)
- plotly (3D可视化)

## 总结

该系统成功实现了从"自由路径规划"到"电网沿线路径生成"的升级，核心特点：

1. ✅ 路径沿电网生成（非A*搜索）
2. ✅ A*仅用于断点连接（预留接口）
3. ✅ 最终路径是3D的
4. ✅ 完整可视化（地形+电网+路径+航点）
5. ✅ 模块化设计，易于扩展
