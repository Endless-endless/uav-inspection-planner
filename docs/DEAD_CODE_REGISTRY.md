# Dead Code Registry（治理阶段 — 仅标记，未删除）

**更新日期：** 2026-05-25  
**阶段：** Mission System Governance — 标记归档候选，生产主链不动。

---

## 确认 Dead（前端）

| 符号 | 位置 | 说明 |
|------|------|------|
| `buildImageInspectionOverlayTraces()` | `web/static/app.js` | 恒返回 `[]`，占位 stub |
| `#togglePoints` | `web/index.html` | HTML 存在，`app.js` 未绑定 |

---

## 可归档（Python）

| 路径 | 说明 |
|------|------|
| `archive/**` | 已隔离旧 replan / core / demo |
| `demo/generate_interactive_main_view.py` | Legacy Plotly HTML（仅 `/legacy/html`） |
| `core/visualization_enhanced.py` | 旧 3D/动画，Dashboard 未用 |
| `visualization/plotly_viewer.py` | 无 app 引用 |
| `visualization/matplotlib_plots.py` | 无 app 引用 |
| `visualization/map_3d_overlay.py` | 无 app 引用 |
| `planner/powerline_inspector.py` | 零外部 import |
| `utils/csv_logger.py` | 仅 utils 包导出 |

---

## 可归档（Demo 脚本 — 非 app 入口）

| 路径 |
|------|
| `demo/demo_dijkstra_*.py` |
| `demo/demo_unified_*.py`（除文档引用外） |
| `demo/demo_mission_*.py` |
| `demo/demo_ui_animation.py` |

---

## 重复逻辑（保留一份，其余待归档）

| 能力 | 权威实现 | 重复/legacy |
|------|----------|-------------|
| Server replan | `planner/replan_start_end.py` | `archive/.../replan_*.py` |
| Dashboard JSON | `planner/mission_result_builder.py` | `generate_interactive_main_view.py` 内联 |
| 地图底图 | `visualization/dashboard_map.py` | `visualization/map_overlay.py`（demo PNG） |
| Client adaptive | `web/static/app.js` + `MissionStore.applyAdaptiveConnectReroute` | 旧 `state.lastResult.segments` 直改 |

---

## 治理后权威状态源

| 层级 | 模块 |
|------|------|
| Mission 数据 | `web/static/mission_governance.js` → `MissionStore` |
| UI 阶段 | `AppPhaseManager` |
| 图层生命周期 | `web/static/layer_manager.js` → `LayerManager` |
| 服务端 mission JSON | `result/latest/mission_output.json` / `result/web_app/latest_replan_*.json` |
