# 项目结构说明

**更新日期：** 2026-05-25  
**阶段：** Project Cleanup & Standardization

---

## 1. 当前主链

```
浏览器 Dashboard (web/index.html)
    ↓
FastAPI app.py
    ├─ POST /api/plan              → 规划 / 加载 mission JSON
    ├─ POST /api/replan            → 起终点重规划（Server Replan）
    ├─ POST /api/image-mission/*   → 图像主线生成与状态
    └─ GET  /api/map/*             → 底图与配置
    ↓
planner/powerline_planner_v3_final.py   主规划器
planner/replan_start_end.py             服务端重规划
planner/mission_result_builder.py       Dashboard JSON 构建
    ↓
result/latest/mission_output.json       权威任务快照
```

**前端状态权威：**

| 层级 | 模块 |
|------|------|
| 任务数据 | `web/static/mission_governance.js` → `MissionStore` |
| UI 阶段 | `AppPhaseManager` |
| 地图图层 | `web/static/layer_manager.js` → `LayerManager` |
| 核心 UI | `web/static/app.js` |
| 天气 / 自适应 | `web/static/weather_ui.js` |
| 重规划面板 | `web/static/replan_ui.js` |
| A/B 实验 | `web/static/experiment_ui.js` |
| 巡检播放 | `web/static/playback.js` |

**Legacy 入口（非 Dashboard 主链）：**

- `python demo/demo_visualization_main.py` — CLI 完整流程
- `GET /legacy/html` — 旧 Plotly 交互页（`demo/generate_interactive_main_view.py`）

---

## 2. 图像巡检点模式流程（`inspection_point_source = image`）

1. 用户在 Dashboard 选择「图像管线」+「图像巡检点」
2. `POST /api/plan` 携带 `inspection_point_source: "image"`、`image_path: data/test_point.png`
3. `PowerlinePlannerV3` 调用 `core/inspection_point_detector.py` 检测空心黑环
4. `core/inspection_point_generator.build_image_detected_inspection_points()` 吸附到拓扑边
5. 全局拓扑优化生成 inspect / connect 段
6. `mission_result_builder` 输出 JSON，`metadata.inspection_point_source` 归一化为 `"image"`
7. 前端 `MissionStore.loadFromDashboard()` 加载；间距控件隐藏

---

## 3. 自动采样模式流程（`inspection_point_source = spacing`）

1. 用户选择「自动采样」或统一 JSON 管线
2. 规划器按间距沿拓扑边采样巡检点（`step5_generate_line_inspection_points` 等）
3. `metadata.inspection_point_source = "spacing"`
4. 重规划面板可调整「规划点间距」

**归一化规则（前后端一致）：**

- 合法值仅：`spacing` | `image`
- 旧别名（`image_points`、`image_detected`、`auto`、`sampled` 等）经 `normalizeInspectionPointSource()` / `normalize_inspection_point_source()` 转换

---

## 4. Server Replan 规则

**入口：** `POST /api/replan` → `planner/replan_start_end.py`

| 规则 | 说明 |
|------|------|
| 巡检点来源 | 若原任务为 `image`，必须保留图像检测点，不得回退 spacing |
| 上下文 | 前端 `MissionStore.buildReplanPayload()` 发送 `inspection_points` + `inspection_point_source` |
| 变更范围 | 重排 visit order、connect 段、起终点连接；**不重新检测图像点** |
| 前端应用 | `MissionStore.applyServerReplan()`；若点数变化会 `console.warn` |

---

## 5. Client Adaptive 规则

**模块：** `web/static/weather_ui.js` + `MissionStore.applyAdaptiveConnectReroute()`

| 规则 | 说明 |
|------|------|
| 触发时机 | 播放中（`AppPhase.PLAYING`）+ 动态天气模拟开启 |
| 变更范围 | **仅修改一条 connect 段的 geometry_2d** |
| 不变项 | `inspection_points` 数组不被改写 |
| 可视化 | T1 临时图层闪烁新路径，结束后写入 MissionStore |
| 预测式 | 可选「预测式自动重规划」，基于未来航迹与移动天气区 |

---

## 6. Weather 模块说明

**后端：** `weather/weather_simulator.py`、`weather/weather_cost.py`  
**数据：** `data/weather_sample.json`（可选场景）

- **静态天气区：** mission JSON 内 `weather_zones`，L3 图层渲染
- **天气感知规划：** `weather_aware` + `weather_weight` 影响 connect 代价
- **动态模拟：** 前端定时更新 zone 中心/半径/强度，驱动 adaptive 逻辑
- **A/B 实验：** `experiment_ui.js` 对比 weather off vs on

---

## 7. Playback 模块说明

**文件：** `web/static/playback.js`

- 从 `MissionStore.getCurrentMission()` 读取任务（非直接改 `state.lastResult`）
- 构建时间轴：起飞 → 沿 inspect/connect 巡航 → 各巡检点停留 → 返航
- 与 `AppPhaseManager.setPlaybackStatus()` 同步 UI 阶段
- Server replan / adaptive 后通过 `reloadPlaybackTimeline()` 刷新

---

## 8. archive/deprecated_cleanup 说明

零引用或废弃 demo 已移入 `archive/deprecated_cleanup/`，**不删除**，详见该目录 `README.md`。

生产路径保留：`app.py`、`core/`、`planner/`（主链文件）、`weather/`、`web/`、`data/`、`result/latest/`。

---

## 9. 目录速查

```
app.py                          FastAPI 入口
core/                           拓扑、检测、采样
planner/                        规划器、重规划、JSON 构建
weather/                        天气代价与模拟
web/                            Dashboard 静态资源
demo/demo_visualization_main.py CLI 演示
visualization/dashboard_map.py  Dashboard 底图
archive/deprecated_cleanup/     本次清理归档
docs/PROJECT_STRUCTURE.md       本文档
```
