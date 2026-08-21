# GridVision 项目架构（答辩版）

## 系统流程

```
地图/图像输入
    → CV 检测 (real_map_cv / inspection_point_detector)
    → 拓扑图构建 (topo.py)
    → 巡检点生成 (inspection_point_generator + planner step5)
    → 边任务 / 访问序 (topo_task + topo_global_optimizer)
    → 连接路径 Dijkstra (topo_dijkstra)
    → 任务 JSON + dashboard.json (mission_result_builder)
    → 前端 Plotly 地图 (app.js buildMissionTraces)
    → 顺序播放 + 图传 (playback.js)
```

## 核心模块

| 模块 | 原路径 | 提交包 |
|------|--------|--------|
| 巡检点生成 | core/inspection_point_generator.py | backend/01_point_generation.py |
| 任务生成 | planner/powerline_planner_v3_final.py | backend/02_mission_generation.py |
| 路径规划 | planner/topo_dijkstra.py | backend/03_path_planning.py |
| 起终点重规划 | planner/replan_start_end.py | backend/04_replanning.py |
| Dashboard | planner/mission_result_builder.py | backend/05_dashboard_export.py |
| 地图渲染 | web/static/app.js | frontend/01_map_render.js |
| 巡检播放 | web/static/playback.js | frontend/02_playback.js |
| 图传 | web/static/playback.js | frontend/03_image_stream.js |

## 数据流

- 后端输出 `latest_dashboard.json`：`segments`（inspect/connect 交替）、`inspection_points`（raw/snapped 坐标）、`markers`
- 前端 `buildMissionRouteSequence` 按 segment 顺序展开播放 timeline，同坐标多点用 `routePointIds[]`
