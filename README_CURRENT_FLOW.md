# UAV 电网巡检 — 当前主流程

**更新时间：** 2026-05-25  
**状态：** Dashboard 主链 + 图像巡检点 + 治理层已落地

---

## 一、推荐入口

| 方式 | 命令 / URL | 说明 |
|------|------------|------|
| **Dashboard（主链）** | `python app.py` → http://127.0.0.1:8000 | 任务控制台、地图、播放、天气、重规划 |
| CLI 完整流程 | `python demo/demo_visualization_main.py` | 生成 `result/latest/mission_output.json` |
| 传统交互页 | http://127.0.0.1:8000/legacy/html | 旧 Plotly 页，非 Dashboard |

---

## 二、Dashboard 主链

```
web/index.html
  mission_governance.js   MissionStore / AppPhaseManager
  layer_manager.js        L0–L3 永久层 + T1–T4 瞬态层
  app.js                  核心：规划加载、地图渲染、元数据
  weather_ui.js           天气图层、动态模拟、自适应绕行
  replan_ui.js            起终点重规划
  experiment_ui.js        A/B 天气实验
  playback.js             巡检播放时间轴
       ↓
app.py  (/api/plan, /api/replan, /api/image-mission, …)
       ↓
planner/powerline_planner_v3_final.py
planner/replan_start_end.py
planner/mission_result_builder.py
       ↓
result/latest/mission_output.json
```

---

## 三、巡检点来源（二选一）

| 值 | UI 文案 | 行为 |
|----|---------|------|
| `spacing` | 自动采样 | 沿拓扑边按间距采样 |
| `image` | 图像巡检点 | 检测 `data/test_point.png` 黑环并吸附拓扑 |

字段统一为 `metadata.inspection_point_source`；旧别名由 `normalizeInspectionPointSource()` 转换。

---

## 四、重规划与自适应

| 类型 | 触发 | 改动范围 |
|------|------|----------|
| **Server Replan** | 面板「执行重规划」→ `/api/replan` | 全局重排；image 模式**保留**原巡检点 |
| **Client Adaptive** | 播放中 + 动态天气 | **仅**改一条 connect 段；巡检点不变 |

---

## 五、核心 Python 模块

| 模块 | 作用 |
|------|------|
| `planner/powerline_planner_v3_final.py` | 主规划器（HSV→骨架→拓扑→任务） |
| `planner/replan_start_end.py` | 起终点重规划服务 |
| `planner/mission_result_builder.py` | Dashboard JSON、来源归一化 |
| `core/inspection_point_detector.py` | 图像巡检点检测 |
| `core/inspection_point_generator.py` | 采样 / 图像点吸附 |
| `core/topo*.py` | 拓扑图与全局优化 |
| `weather/` | 天气区、代价、实验数据 |
| `visualization/dashboard_map.py` | Dashboard 底图配置 |

已归档（零 app 引用）：见 `archive/deprecated_cleanup/README.md`  
含 `plotly_viewer.py`、`powerline_inspector.py`、废弃 `demo_*` 等。

---

## 六、输出文件

| 路径 | 说明 |
|------|------|
| `result/latest/mission_output.json` | 标准任务 JSON |
| `result/latest/main_view_interactive.html` | Legacy 交互页 |
| `result/web_app/latest_replan_*.json` | Dashboard 重规划快照 |

---

## 七、快速开始

```bash
pip install -r requirements.txt
python app.py
# 浏览器打开控制台 → 选择图像管线 + 图像巡检点 → 生成任务
```

环境变量（可选）：

```bash
set UAV_INSPECTION_SOURCE=image   # spacing | image
python demo/demo_visualization_main.py
```

---

## 八、文档索引

| 文档 | 内容 |
|------|------|
| `docs/PROJECT_STRUCTURE.md` | 结构、流程、Replan/Adaptive/Weather/Playback 规则 |
| `docs/MISSION_GOVERNANCE.md` | MissionStore / AppPhase / LayerManager |
| `docs/DEAD_CODE_REGISTRY.md` | 死代码与归档登记 |
| `archive/deprecated_cleanup/README.md` | 本次清理归档清单 |

---

**维护：** 以 Dashboard + `app.py` 实际行为为准；勿再引用已移入 `archive/deprecated_cleanup/demo/` 的 unified/dijkstra 脚本作为当前流程。
