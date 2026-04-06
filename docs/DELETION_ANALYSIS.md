# 项目清理分析 - 删除建议清单

**分析日期：** 2026-04-01
**当前状态：** 主流程稳定，使用 data/test.png

---

## 一、保留文件（主流程必需）

### 1.1 主入口脚本
- ✅ **demo/demo_visualization_main.py** - 当前推荐主入口
- ✅ **demo/demo_ui_animation.py** - 辅助导航页（被 demo_visualization_main 引用）

### 1.2 可视化模块
- ✅ **demo/generate_interactive_main_view.py** - 交互式主展示页生成
- ⚠️ **demo/generate_main_view_map_based.py** - 2D 地图叠加生成（被主流程调用）

### 1.3 核心模块（主流程依赖）
- ✅ **core/topo.py** - 拓扑图定义
- ✅ **core/topo_task.py** - EdgeTask 定义
- ✅ **core/topo_plan.py** - 拓扑规划逻辑（含 GroupedContinuousMission）
- ✅ **core/visualization_enhanced.py** - 被 demo_ui_animation.py 使用

### 1.4 规划器
- ✅ **planner/powerline_planner_v3_final.py** - 主规划器
- ✅ **planner/astar3d.py** - A* 路径规划
- ✅ **planner/node.py** - 节点定义

### 1.5 输出目录
- ✅ **result/latest/** - 当前输出目录

---

## 二、建议删除文件

### 2.1 冗余 Demo 脚本（可归档）

| 文件 | 删除理由 | 影响 |
|------|----------|------|
| demo/demo_topo_plan_continuous.py | 被 demo_visualization_main.py 取代 | 无 |
| demo/demo_topo_plan_grouped.py | 被 demo_visualization_main.py 取代 | 无 |
| demo/demo_topo_plan_grouped_optimized.py | 实验版本，功能已整合到主流程 | 无 |

### 2.2 未使用的 Core 模块（可归档）

| 文件 | 删除理由 | 影响 |
|------|----------|------|
| core/costs.py | 主流程未使用 | 无 |
| core/energy.py | 主流程未使用 | 无 |
| core/execution.py | 主流程未使用 | 无 |
| core/independent_lines.py | 主流程未使用（功能在 planner 内） | 无 |
| core/inspection_point_generator.py | 主流程未使用 | 无 |
| core/inspection_tasks.py | 主流程未使用 | 无 |
| core/mission.py | 主流程未使用 | 无 |
| core/mission_build.py | 主流程未使用 | 无 |
| core/mission_opt.py | 主流程未使用 | 无 |
| core/navigator.py | 主流程未使用 | 无 |
| core/optimizer.py | 主流程未使用 | 无 |
| core/path_optimizer.py | 主流程未使用 | 无 |
| core/planner.py | 主流程未使用（功能在 planner/ 目录） | 无 |
| core/terrain.py | 主流程未使用 | 无 |
| core/vis_adapter.py | 主流程未使用 | 无 |
| core/vision.py | 主流程未使用 | 无 |

### 2.3 过时文档（可更新或删除）

| 文件 | 处理建议 | 理由 |
|------|----------|------|
| README_CURRENT_FLOW.md | 需更新 | 引用 data/1.png |
| docs/fix_comparison.md | 可删除 | 历史对比文档 |
| docs/powerline_fixed_summary.md | 可删除 | 历史修复记录 |
| docs/V2_vs_V3对比.md | 可删除 | 版本对比历史 |
| docs/V2重构总结.md | 可删除 | 版本历史 |
| docs/V3优化总结.md | 可删除 | 版本历史 |
| docs/V4最终交付总结.md | 可删除 | 版本历史 |
| docs/修复前后对比.md | 可删除 | 历史对比 |

### 2.4 Scripts

| 文件 | 处理建议 | 理由 |
|------|----------|------|
| scripts/final_teacher_demo.py | 更新后保留 | 演示脚本，需更新为 test.png |

---

## 三、暂缓删除（需进一步确认）

### 3.1 可视化相关
- **demo/generate_main_view_map_based.py** - 2D 地图叠加，目前被主流程调用，但功能可能与交互式页面重复

### 3.2 已归档文件
- **archive/** 目录下所有文件 - 已归档，建议保留作为历史记录

---

## 四、删除执行计划

### Phase 1: 安全归档（不删除）
移动到 `archive/deprecated_20260401/`：
- 冗余 demo 脚本（3个）
- 未使用的 core 模块（16个）

### Phase 2: 更新后保留
- 更新 README_CURRENT_FLOW.md
- 更新 scripts/final_teacher_demo.py

### Phase 3: 删除过时文档
- 删除 6 个版本历史文档

---

## 五、验证清单

删除后必须验证：
- [ ] demo_visualization_main.py 可正常运行
- [ ] mission_output.json 正确生成
- [ ] main_view_interactive.html 可正常打开
- [ ] 使用 data/test.png 作为数据源
- [ ] 多电路连续巡检逻辑正常
