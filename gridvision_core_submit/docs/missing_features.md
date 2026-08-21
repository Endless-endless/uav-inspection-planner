# 核心提交包未包含的功能

以下内容仍在**原工程**中，本答辩包仅摘录核心函数，未包含完整可运行链路。

| 类别 | 原路径 | 说明 |
|------|--------|------|
| Web 服务入口 | `app.py` | FastAPI 路由、静态资源、任务 API |
| 页面壳层 | `web/index.html`, `style.css` | Dashboard UI 布局与样式 |
| 状态管理 | `web/static/mission_store.js` | MissionStore 全局状态 |
| 完整 CV 管线 | `core/real_map_cv.py`, `core/topo_plan.py` | 骨架提取、分组规划（topo_plan 4500+ 行） |
| 全局优化 | `core/topo_global_optimizer.py` | 访问序全局优化 |
| 线路提取 | `core/independent_lines.py`, `physical_line_chain.py` | Step4 独立线路 |
| 配置 | `config/settings.py` | 环境变量与数据集配置 |
| CLI 演示 | `demo/demo_visualization_main.py` | 命令行一键出任务 |
| 风场 | `weather/wind_model.py` | 非答辩展示主链 |
| 诊断脚本 | `scripts/diag_route_order.py` | 路线顺序调试 |
| 原始地图数据 | `data/*.png`, `data/*.json` | 除 sample_dashboard 外未打包 |
| 图传静态资源 | `figures/`, placeholder SVG | 运行期依赖 |

## 使用建议

- **答辩展示算法**：使用 `backend/` + `frontend/` 摘录配合 PPT 讲解。
- **现场完整演示**：运行原项目 `python app.py`，不要仅依赖本目录。
