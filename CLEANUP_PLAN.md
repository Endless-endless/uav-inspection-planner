# UAV 电网巡检项目 — 仓库整理计划

**扫描日期：** 2026-05-19  
**当前分支：** `stable/unified-mission-v1`  
**仓库规模：** 约 283 个文件，约 692 MB（其中根目录演示视频约 593 MB）  
**执行状态：** 仅规划，**未删除、未移动任何文件**

---

## 一、扫描摘要

### 1.1 当前顶层目录

| 目录/文件 | 大小（约） | 角色 |
|-----------|-----------|------|
| `core/` | 1.1 MB | 拓扑、任务、规划核心 |
| `planner/` | 0.45 MB | 主规划器、Dijkstra、任务优化 |
| `input/` | 0.11 MB | UnifiedInput 主线 |
| `demo/` | 0.34 MB | 演示入口（图像 + Unified + Dijkstra） |
| `visualization/` | 0.18 MB | Plotly / 地图叠加 |
| `data/` | 5.36 MB | 输入图像与 JSON 样例 |
| `result/` | 55.05 MB | 运行输出（已在 .gitignore） |
| `figures/` | 35.63 MB | 历史 Plotly HTML |
| `archive/` | 0.30 MB | 已归档旧代码 |
| `scripts/` | 0.08 MB | 验证 / 重规划工具脚本 |
| `docs/` | 0.05 MB | 设计与阶段文档 |
| 根目录 `*.mp4` | **~593 MB** | 演示录屏（3 个） |
| `requirements.txt` 等 | 极小 | 依赖与说明 |

### 1.2 必须保护的稳定功能（整理时不可破坏）

| 功能线 | 关键路径 |
|--------|----------|
| 图像主线 demo | `demo/demo_visualization_main.py` → `planner/powerline_planner_v3_final.py` |
| UnifiedInput 主线 | `input/unified_*.py` + `demo/demo_unified_*.py` |
| TopoGraph | `core/topo.py` |
| EdgeTask | `core/topo_task.py` + `input/unified_task_adapter.py` |
| Mission | `core/topo_plan.py` + `input/unified_mission_adapter.py` |
| topology-aware optimizer | `planner/mission_optimizer.py` + `core/topo_global_optimizer.py` |
| Dijkstra planner | `planner/topo_dijkstra.py` + `demo/demo_dijkstra_*.py` |

---

## 二、必须保留

### 2.1 核心代码（生产逻辑）

```
core/
├── topo.py                      # TopoGraph 建模
├── topo_task.py                 # EdgeTask
├── topo_plan.py                 # 分组规划、Mission 导出、BFS 连接
├── topo_global_optimizer.py     # 全局拓扑优化 / build_optimized_mission
├── independent_lines.py         # 独立线路（Unified 适配依赖）
├── inspection_point_generator.py
├── visualization_enhanced.py    # UI 动画 demo 依赖
└── __init__.py

planner/
├── powerline_planner_v3_final.py   # 图像主线主规划器
├── topo_dijkstra.py                # Dijkstra + BFS 对比工具
├── mission_optimizer.py            # topology-aware 边序优化
├── mission_analysis.py             # 任务统计 / 对比
├── astar3d.py                      # 3D A*（规划器子模块）
├── node.py
└── __init__.py

input/
├── unified_input.py
├── unified_adapter.py
├── unified_task_adapter.py
├── unified_mission_adapter.py
└── __init__.py

config/settings.py
weather/wind_model.py              # 主规划器天气代价
visualization/
├── plotly_viewer.py
├── map_overlay.py                 # 主 demo Phase 5 使用
├── map_3d_overlay.py
└── matplotlib_plots.py

utils/csv_logger.py
requirements.txt
.gitignore                         # 需按第四节修订，但文件本身保留
```

### 2.2 稳定 Demo 入口（`demo/`）

| 文件 | 用途 | 输出目录 |
|------|------|----------|
| `demo_visualization_main.py` | **图像主线主入口** | `result/latest/` |
| `demo_ui_animation.py` | 辅助导航 / 动画 | 依赖主 JSON |
| `generate_interactive_main_view.py` | 交互式 HTML | `result/latest/` |
| `demo_unified_input.py` | UnifiedInput 加载校验 | stdout |
| `demo_unified_adapter.py` | Unified → IndependentLine | stdout |
| `demo_unified_topo_bridge.py` | Unified → TopoGraph | stdout |
| `demo_unified_edge_tasks.py` | EdgeTask 调试 | `result/unified_edge_tasks/` |
| `demo_unified_mission.py` | Unified Mission | `result/unified_mission/` |
| `demo_unified_mission_visualization.py` | Mission HTML | `result/unified_mission/` |
| `demo_mission_compare.py` | baseline vs optimized | `result/unified_mission/` |
| `demo_dijkstra_topo_compare.py` | 拓扑路径 BFS/Dijkstra | `result/dijkstra_test/` |
| `demo_dijkstra_contrast.py` | **对比专用输入** | `result/dijkstra_test/` |
| `demo_dijkstra_visualization.py` | Dijkstra 可视化 HTML | `result/dijkstra_test/` |
| `demo_mission_bfs_vs_dijkstra.py` | Mission 级 BFS/Dijkstra | `result/dijkstra_test/` |

### 2.3 输入数据（`data/`）

```
data/test.png                           # 图像主线唯一主图（必须）
data/sample_unified_input.json
data/sample_unified_topo_input.json
data/sample_dijkstra_contrast_input.json
data/section1/                        # 多地图 section 样例（map demo 可选）
```

### 2.4 文档（保留并建议后续合并更新）

```
README_CURRENT_FLOW.md                 # 主流程说明（需与现状对齐）
PROJECT_STRUCTURE.md                   # 结构说明（偏旧，保留作参考）
result/README.md                       # 输出目录约定
docs/MISSION_JSON_AND_UI_FEATURES.md
docs/STAGE1_INDEPENDENT_LINES.md
docs/STAGE2_MISSION.md
docs/PATH_OPTIMIZER.md
docs/EXECUTION_MODULE.md
docs/UAV_ANIMATION.md
docs/DIAGNOSIS_MULTI_CIRCUIT.md
docs/powerline_following_implementation.md
docs/DELETION_ANALYSIS.md              # 历史清理分析，整理完成后可归档
```

### 2.5 输出目录结构（保留目录本身，内容可清理）

```
result/latest/          # 图像主线（主 demo 写入）
result/unified_mission/
result/unified_edge_tasks/
result/dijkstra_test/
result/steps/           # 中间步骤图（调试用）
result/README.md
```

---

## 三、建议归档

> **操作方式：** 确认后移动到 `archive/` 子目录（如 `archive/deprecated_20260519/`），**不要直接删除**。  
> 下列文件当前仍在根目录，但已不属于两条稳定主线。

### 3.1 旧 Demo / 备用 Demo（`demo/`）

| 文件 | 标记 | 理由 |
|------|------|------|
| `demo/demo_start_point_driven.py` | 旧 demo | 起点驱动旁路，写入 `result/latest/start_driven_*`，与主流程并行 |
| `demo/generate_main_view_map_based.py` | 临时/冗余 | 主 demo 已改用 `visualization/map_overlay.py`，无引用 |

### 3.2 已归档但仍可整理的 `archive/`（保持不动或合并）

```
archive/deprecated_20260401/     # 旧 core + demo（已归档，保留）
archive/legacy_demo/             # Stage1/2 旧 demo
archive/debug_scripts/           # 调试脚本
archive/demo_*.py                # 性能/导航/执行旧 demo
archive/*.html                   # 旧动画 HTML
archive/test_*.py                # 性能测试
```

建议：**不二次移动** `archive/` 内部，仅在根目录新增归档时按日期分子目录。

### 3.3 工具脚本（`scripts/`）— 建议整体归档

| 文件 | 标记 | 理由 |
|------|------|------|
| `scripts/verify_extraction.py` | 临时验证 | 图像提取验收 |
| `scripts/verify_extraction_enhanced.py` | 临时验证 | 同上 |
| `scripts/verify_powerline_extraction.py` | 临时验证 | 同上 |
| `scripts/verify_coverage_difference.py` | 临时验证 | 覆盖差集验收 |
| `scripts/debug_topology_split.py` | 调试 | 拓扑切分调试 |
| `scripts/clean_mission_output.py` | 工具 | JSON 清理 |
| `scripts/replan_from_start.py` | 旁路 | 起点重规划，依赖 `start_driven_planner_v2` |
| `scripts/replan_service.py` | 旁路 | 重规划服务封装 |
| `scripts/final_teacher_demo.py` | 演示 | 教师演示，非 CI 入口 |

### 3.4 低活跃模块（可归档代码，非 demo）

| 路径 | 标记 | 理由 |
|------|------|------|
| `core/start_driven_planner.py` | 旧规划 | 仅 start-driven / scripts 使用 |
| `core/start_driven_planner_v2.py` | 旧规划 | 同上 |
| `mission/` | 旧模块 | TSP / multi_goal，主流程未引用 |
| `planner/powerline_inspector.py` | 旧规划 | 未被当前 demo import |
| `environment/` | 实验 | HeightMap3D，主规划器内置 terrain，未 import |
| `analysis/metrics.py` | 实验 | 无 demo 引用 |
| `maps/*.world.json` | 样例 | 3D world 描述，当前主线未用 |

### 3.5 历史可视化与输出

| 路径 | 标记 | 大小 | 理由 |
|------|------|------|------|
| `figures/powerline_v3_map_power*.html` | 历史输出 | ~35 MB | V3 早期 Plotly 页面，可重建 |
| `result/archive/` | 历史输出 | 小 | 修复前 mission/HTML，已由 README 说明 |
| `result/*.png`（根下 11 张） | 重复输出 | 中 | 与 `result/steps/` 内容重复，主 demo 也会重写 |
| `result/latest/*.old` | 临时备份 | 小 | 主 demo Phase -1 自动备份 |

---

## 四、可以删除

> **前提：** 用户书面确认后执行。下列项按**收益/风险**排序。  
> **当前禁止执行删除。**

### 4.1 高收益、低风险（建议优先）

| 路径 | 约大小 | 类型 | 删除理由 |
|------|--------|------|----------|
| `效果展示.mp4` | 155 MB | 演示视频 | 非代码、非输入，已在 .gitignore |
| `效果展示2.mp4` | 230 MB | 演示视频 | 同上 |
| `效果展示3.mp4` | 208 MB | 演示视频 | 同上 |
| `result/latest/*.old` | 小 | 缓存/备份 | 主 demo 自动生成，可再生成 |
| `result/latest/mission_output_before_replan.json.old` | 小 | 备份 | 同上 |

**合计可释放：约 593+ MB（主要为 mp4）**

### 4.2 中收益（删除前建议 diff 或备份）

| 路径 | 类型 | 删除理由 |
|------|------|----------|
| `result/archive/*` | 历史输出 | 修复前错误路径产物，见 `result/README.md` |
| `result/step*.png`（根目录 11 张） | 重复输出 | 与 `result/steps/` 重复 |
| `figures/*.html` | 历史 HTML | 可由 demo 重新生成（若仍需保留样例，改为归档） |
| `result/dijkstra_test/dijkstra_compare_view.html` | 生成物 | 可随 demo 重建 |

### 4.3 低收益 / 需代码确认后再删

| 路径 | 删除理由 | 风险 |
|------|----------|------|
| `docs/DELETION_ANALYSIS.md` | 2026-04-01 清理草案，部分路径已过时 | 低，建议归档而非删 |
| `PROJECT_STRUCTURE.md` 中过时章节 | 与现状不符 | 建议更新而非删整文件 |

### 4.4 明确不可删除（即使看起来冗余）

- `archive/deprecated_20260401/` — 唯一完整旧版快照  
- `data/test.png` — 图像主线硬依赖  
- 所有 `demo/demo_unified_*`、`demo/demo_dijkstra_*`、`demo/demo_visualization_main.py`  
- `input/`、`planner/topo_dijkstra.py`、`core/topo_global_optimizer.py`

---

## 五、应加入 / 修订 `.gitignore`

### 5.1 当前 `.gitignore` 问题

现有规则：

```gitignore
*.png
*.jpg
*.mp4
result/
```

**副作用：** `data/test.png`、`data/section1/*.png` 也被忽略，新 clone 无法直接跑主 demo。  
**建议：** 改为「忽略生成物，保留输入数据」的精细规则。

### 5.2 推荐 `.gitignore` 增补（确认后应用）

```gitignore
# --- Python / 环境（保留现有）---
__pycache__/
*.pyc
*.pyo
*.pyd
.venv/
venv/
env/

# --- IDE / OS ---
.vscode/
.idea/
.DS_Store
*.log

# --- 运行输出（整目录）---
result/

# --- 生成型可视化（可重建）---
figures/
*.html
!docs/**/*.html

# --- 演示媒体（根目录录屏）---
/*.mp4
/*.avi
/*.mov

# --- 临时 / 备份 ---
*.old
*.bak
*.json.bak
*.html.bak

# --- 生成图片：默认忽略，但保留 data/ 输入 ---
*.png
*.jpg
!data/**/*.png
!data/**/*.jpg

# --- 可选：Dijkstra / Unified 调试 JSON（若不想提交）---
# result/dijkstra_test/
# result/unified_mission/
# result/unified_edge_tasks/
```

### 5.3 建议纳入版本控制的文件（取消忽略）

| 文件 | 理由 |
|------|------|
| `data/test.png` | 主 demo 必需 |
| `data/sample_*.json` | Unified / Dijkstra 测试输入 |
| `requirements.txt` | 依赖锁定 |
| `CLEANUP_PLAN.md` | 本整理计划 |

### 5.4 建议始终不纳入 Git 的目录

```
result/                    # 全部运行产物
figures/                 # 历史 HTML
__pycache__/             # 字节码
*.old                    # demo 自动备份
根目录 *.mp4             # 演示录屏
```

---

## 六、建议最终目录结构

```
uav-inspection-topo/                    # 项目根
├── README.md                           # 合并 README_CURRENT_FLOW（待建）
├── README_CURRENT_FLOW.md              # 保留至合并完成
├── PROJECT_STRUCTURE.md                # 精简后保留
├── CLEANUP_PLAN.md                     # 本文件
├── requirements.txt
├── .gitignore                          # 按第五节修订
│
├── core/                               # 拓扑 + 任务 + 规划（稳定）
├── planner/                            # 规划器 + Dijkstra + optimizer
├── input/                              # UnifiedInput 适配层
├── visualization/                      # Plotly / 地图叠加
├── weather/                            # 风况模型
├── config/
├── utils/
│
├── demo/                               # 仅保留稳定 + 实验 demo
│   ├── demo_visualization_main.py      # 图像主线 ★
│   ├── demo_ui_animation.py
│   ├── generate_interactive_main_view.py
│   ├── demo_unified_*.py               # Unified 主线 ★
│   ├── demo_mission_compare.py
│   ├── demo_dijkstra_*.py              # Dijkstra ★
│   └── demo_mission_bfs_vs_dijkstra.py
│
├── data/                               # 仅输入（png/json）
│   ├── test.png
│   ├── sample_*.json
│   └── section1/                       # 可选多图
│
├── result/                             # .gitignore，本地生成
│   ├── latest/                         # 图像主线
│   ├── unified_mission/
│   ├── unified_edge_tasks/
│   ├── dijkstra_test/
│   └── steps/                          # 中间步骤（可选保留目录占位 README）
│
├── docs/                               # 设计文档
├── scripts/                            # → 建议迁至 archive/scripts_20260519/
├── archive/                            # 历史代码 + 旧 demo + 旧输出说明
│   ├── deprecated_20260401/
│   ├── legacy_demo/
│   ├── debug_scripts/
│   └── deprecated_20260519/            # 新一轮归档目标（规划）
│
├── maps/                               # → 可并入 archive/maps/
├── mission/                            # → 可并入 archive/mission/
├── environment/                        # → 可并入 archive/environment/
├── analysis/                           # → 可并入 archive/analysis/
└── media/                              # 【新建】演示视频移入，且 gitignore
    └── demos/
        ├── 效果展示.mp4
        ├── 效果展示2.mp4
        └── 效果展示3.mp4
```

### 6.1 Demo 分层（推荐标注）

在 `demo/README.md`（待建）中标注：

| 层级 | Demo | 说明 |
|------|------|------|
| **P0 生产** | `demo_visualization_main.py` | 对外展示入口 |
| **P1 Unified** | `demo_unified_*`, `demo_mission_compare` | JSON 输入主线 |
| **P2 Dijkstra** | `demo_dijkstra_*`, `demo_mission_bfs_vs_dijkstra` | 算法对比验证 |
| **P3 辅助** | `demo_ui_animation.py` | 依赖 P0 输出 |
| **归档候选** | `demo_start_point_driven.py` | 旁路实验 |

---

## 七、删除前需要再次确认的文件

以下文件**禁止在未确认前删除**：

### 7.1 可能被低估依赖

| 文件 | 需确认 |
|------|--------|
| `demo/generate_main_view_map_based.py` | 是否有外部文档/脚本硬编码调用 |
| `core/start_driven_planner_v2.py` | `scripts/replan_from_start.py` 是否仍在使用 |
| `planner/powerline_inspector.py` | 是否被未扫描到的 notebook / 外部项目引用 |
| `mission/tsp_solver.py` | 未来 TSP 优化是否计划复用 |
| `environment/heightmap_loader.py` | 3D 地形扩展是否保留 |
| `data/section1/*.png` | `generate_main_view_map_based` / section demo 是否还需要 |

### 7.2 输出与数据

| 文件 | 需确认 |
|------|--------|
| `result/latest/mission_output.json` | 是否为对外展示固定快照 |
| `result/latest/inspection_images/IP_*.jpg` | UI 动画是否依赖本地图片 |
| `result/archive/*` | 是否需要用于论文/报告前后对比 |
| `figures/powerline_v3_*.html` | 是否为答辩固定演示页 |

### 7.3 大文件

| 文件 | 需确认 |
|------|--------|
| `效果展示.mp4` / `效果展示2.mp4` / `效果展示3.mp4` | 是否需保留本地副本；若仅展示用途，删后是否可从网盘恢复 |
| `data/test.png` | **不可删**；若 Git 未跟踪需改为 `!data/**/*.png` |

### 7.4 文档

| 文件 | 需确认 |
|------|--------|
| `docs/DELETION_ANALYSIS.md` | 归档还是删除 |
| `PROJECT_STRUCTURE.md` | 重写还是归档 |

---

## 八、推荐执行顺序（待用户确认后）

```
Phase 0  ✅ 生成 CLEANUP_PLAN.md（当前步骤，已完成）
Phase 1  修订 .gitignore（保留 data/ 输入图）
Phase 2  归档：scripts/、start_driven、mission/、environment/、analysis/、旧 demo
Phase 3  移动根目录 mp4 → media/demos/（或删除）
Phase 4  清理 result/ 重复 png、*.old、result/archive/
Phase 5  清理 figures/ 或迁至 archive/figures/
Phase 6  更新 README / PROJECT_STRUCTURE / demo/README
Phase 7  验证：
         python demo/demo_visualization_main.py
         python demo/demo_unified_mission.py
         python demo/demo_dijkstra_contrast.py
```

---

## 九、文件标记速查表

### 9.1 `demo/` 全量标记

| 文件 | 标记 |
|------|------|
| `demo_visualization_main.py` | ✅ 必须保留（P0） |
| `demo_ui_animation.py` | ✅ 必须保留（P3） |
| `generate_interactive_main_view.py` | ✅ 必须保留 |
| `generate_main_view_map_based.py` | 📦 建议归档 |
| `demo_start_point_driven.py` | 📦 建议归档 |
| `demo_unified_input.py` | ✅ 必须保留 |
| `demo_unified_adapter.py` | ✅ 必须保留 |
| `demo_unified_topo_bridge.py` | ✅ 必须保留 |
| `demo_unified_edge_tasks.py` | ✅ 必须保留 |
| `demo_unified_mission.py` | ✅ 必须保留 |
| `demo_unified_mission_visualization.py` | ✅ 必须保留 |
| `demo_mission_compare.py` | ✅ 必须保留 |
| `demo_dijkstra_topo_compare.py` | ✅ 必须保留 |
| `demo_dijkstra_contrast.py` | ✅ 必须保留 |
| `demo_dijkstra_visualization.py` | ✅ 必须保留 |
| `demo_mission_bfs_vs_dijkstra.py` | ✅ 必须保留 |

### 9.2 输出 / 缓存标记

| 路径 | 标记 |
|------|------|
| `result/latest/` | 运行输出（gitignore） |
| `result/steps/` | 中间步骤图（gitignore） |
| `result/dijkstra_test/` | 测试输出（gitignore） |
| `result/unified_*` | 测试输出（gitignore） |
| `result/archive/` | 历史输出，可删（需确认） |
| `result/*.png`（根） | 重复，可删 |
| `*.old` | 临时备份，可删 |
| `__pycache__/` | 缓存，应 ignore |

---

## 十、结论

1. **稳定功能代码完整**，两条主线（图像 + Unified）+ Dijkstra + optimizer 均已就位，整理重点在**输出物、录屏、历史 demo**。  
2. **最大空间占用**为根目录 3 个 MP4（约 593 MB），其次 `result/`（55 MB）与 `figures/`（36 MB）。  
3. **`.gitignore` 需修订**，避免忽略 `data/test.png` 导致仓库不可用。  
4. **本计划未执行任何删除或移动**；请审阅第七节「再次确认」列表后，指定 Phase 1–7 的执行范围。

---

*生成方式：全仓库文件扫描 + 主 demo import 链分析 + 目录体积统计。*
