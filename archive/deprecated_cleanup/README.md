# deprecated_cleanup 归档说明

**归档日期：** 2026-05-25  
**目的：** 低风险清理 — 零引用或已废弃 demo/可视化模块移入此处，不直接删除。

## 归档规则

- 移动前经全仓库 import / 字符串 / 路由 / script 引用检查
- 生产主链（`app.py`、`core/`、`planner/`、`weather/`、`web/`、`data/`、`result/latest/`）不移动
- 若需恢复，将文件移回原路径并更新 `visualization/__init__.py` 等导出

## 本目录内容

| 子目录 | 说明 |
|--------|------|
| `planner/` | `powerline_inspector.py` — 旧巡检器，零外部 import |
| `utils/` | `csv_logger.py`、`__init__.py` — 无 app 引用 |
| `config/` | ~~`settings.py`~~ — **保留在生产路径**（`planner/astar3d.py` 依赖） |
| `visualization/` | `plotly_viewer.py`、`matplotlib_plots.py`、`map_3d_overlay.py` |
| `demo/` | 废弃 unified/dijkstra/mission 对比 demo 脚本 |

## 仍保留在生产路径

- `demo/demo_visualization_main.py`、`demo/generate_interactive_main_view.py`（`/legacy/html`）
- `visualization/dashboard_map.py`、`visualization/map_overlay.py`
- `core/visualization_enhanced.py`（规划器 step11 懒加载）
