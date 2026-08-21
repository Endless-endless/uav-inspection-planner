# GridVision 毕设答辩核心代码包

本目录从 GridVision 原工程**自动摘录**核心算法与播放逻辑，用于论文附录与答辩展示。  
**不替代**完整可运行工程；运行演示请使用原项目 `python app.py`。

## 项目简介

GridVision 是一套面向电力线路的无人机自主巡检系统：从卫星/测试地图中提取线路拓扑，生成巡检点与访问序，规划 connect/inspect 交替航线，并在 Web Dashboard 上可视化播放与模拟图传。

## 目录结构

```
gridvision_core_submit/
├── README.md
├── backend/          # Python 核心摘录
├── frontend/         # JavaScript 核心摘录
├── data/             # sample_dashboard.json
└── docs/             # 架构、审计、统计
```

## 运行方式（完整系统）

在原项目根目录：

```bash
pip install -r requirements.txt
python app.py
# 浏览器 http://127.0.0.1:8001
```

## 毕设展示功能

1. 图像/地图载入与线路识别
2. 13 巡检点生成（几何采样 + CV 检测融合）
3. inspect/connect 交替任务与 Dijkstra 连接
4. 起终点重规划
5. Dashboard 地图蓝/橙线路 + 绿色巡检点
6. 按 mission segment 顺序播放
7. 到达巡检点触发图传/侧栏

## 代码量

- 原工程约 29831 行 → 本包摘录约 2197 行（见 docs/code_statistics.md）

## 文件说明

详见 `docs/file_audit_report.md`、`docs/project_architecture.md`、`docs/missing_features.md`。
