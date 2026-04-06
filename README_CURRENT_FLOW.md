# UAV 电网巡检项目 - 当前主流程说明

**更新时间：** 2026-04-01
**项目状态：** 主线稳定，基于拓扑结构的多电路一次性连续巡检功能完善

---

## 一、当前项目主线流程

### 1.1 核心处理流程

```
输入图像 (data/test.png)
    ↓
Step 1: HSV 红色提取
    ↓
Step 2: 形态学修复
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
    ↓
Step 8.5: EdgeTask 建模
    ↓
Step 9.2: 分组感知连续航迹规划
    ↓
输出: mission_output.json + 交互式可视化页面
```

### 1.2 核心数据结构

```
独立线路 (IndependentLine)
    ↓
拓扑节点 (TopoNode)
    ↓
拓扑边 (TopoEdge)
    ↓
拓扑图 (TopoGraph)
    ↓
边任务 (EdgeTask)
    ↓
分组连续任务 (GroupedContinuousMission)
    ↓
标准 JSON 输出 (mission_output.json)
```

---

## 二、当前主线 Demo

### 2.1 推荐入口脚本

| 文件 | 说明 |
|------|------|
| **demo/demo_visualization_main.py** | 完整流程入口，生成 JSON + 可视化 |
| **demo/demo_ui_animation.py** | 辅助导航页（可选） |

### 2.2 推荐使用方式

**标准用法（推荐）：**
```bash
python demo/demo_visualization_main.py
```

**输出文件：**
- `result/latest/mission_output.json` - 标准任务 JSON
- `result/latest/main_view_interactive.html` - 交互式主展示页
- `result/latest/map_overlay_test.png` - 2D 地图叠加

---

## 三、核心模块

### 3.1 规划器
- **planner/powerline_planner_v3_final.py** - 主规划器
- **planner/astar3d.py** - A* 路径规划
- **planner/node.py** - 节点定义

### 3.2 核心模块
- **core/topo.py** - 拓扑图定义
- **core/topo_task.py** - EdgeTask 定义
- **core/topo_plan.py** - 拓扑规划逻辑（含 GroupedContinuousMission）
- **core/visualization_enhanced.py** - 可视化增强模块

---

## 四、功能特性

### 4.1 多电路一次性连续巡检
- ✅ 多条电路统一纳入一次任务
- ✅ 巡检段与连接段组织清晰
- ✅ 连接段沿拓扑图路径生成（多点 polyline）
- ✅ 访问顺序合理优化

### 4.2 连接段生成
- ✅ 使用拓扑图路径（非退化直线）
- ✅ 支持长距离连接（动态阈值）
- ✅ Geometry 为真实多点 polyline

### 4.3 JSON 输出
- ✅ 完整的 groups 信息
- ✅ edge_visit_order
- ✅ segments（inspect + connect）
- ✅ inspection_points
- ✅ full_path_2d / full_path_3d
- ✅ 统计信息

---

## 五、数据源

**当前统一使用：** `data/test.png` (916 x 960 pixels)

---

## 六、项目清理说明

### 6.1 已归档
- 冗余 demo 脚本 → `archive/deprecated_20260401/demo/`
- 未使用的 core 模块 → `archive/deprecated_20260401/core/`

### 6.2 当前最小必要文件集
- **planner/**: powerline_planner_v3_final.py, astar3d.py, node.py
- **core/**: topo.py, topo_task.py, topo_plan.py, visualization_enhanced.py
- **demo/**: demo_visualization_main.py, demo_ui_animation.py, generate_interactive_main_view.py, generate_main_view_map_based.py

---

## 七、版本说明

| 版本 | 状态 | 说明 |
|------|------|------|
| **Step 7.5** | ✅ 主线 | 拓扑图建模 |
| **Step 8.5** | ✅ 主线 | EdgeTask 建模 |
| **Step 9.2** | ✅ **当前版本** | 分组感知连续航迹规划 |
