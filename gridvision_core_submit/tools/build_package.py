#!/usr/bin/env python3
"""One-shot builder for gridvision_core_submit. Run from project root."""
from __future__ import annotations

import json
import os
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "gridvision_core_submit"


def read_lines(path: Path, start: int, end: int) -> str:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True)
    chunk = lines[start - 1 : end]
    return "".join(chunk)


def py_header(title: str, source: str, lines: str, note: str = "") -> str:
    return f'''"""
GridVision 毕设答辩核心代码摘录 — {title}
源文件: {source}
行号: {lines}
{note}
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""
'''


def js_header(title: str, source: str, lines: str) -> str:
    return f'''/**
 * GridVision 毕设答辩核心代码摘录 — {title}
 * 源文件: {source}
 * 行号: {lines}
 * 说明: 摘录供答辩展示，需与原工程 static 资源配合运行。
 */
'''


def extract_py(out_name: str, title: str, src: Path, ranges: list[tuple[int, int]], note: str = ""):
    parts = []
    line_desc = ", ".join(f"{a}-{b}" for a, b in ranges)
    parts.append(py_header(title, str(src.relative_to(ROOT)).replace("\\", "/"), line_desc, note))
    for a, b in ranges:
        parts.append(f"\n# ===== {src.name} L{a}-L{b} =====\n")
        parts.append(read_lines(src, a, b))
    (OUT / "backend" / out_name).write_text("".join(parts), encoding="utf-8")


def extract_js(out_name: str, title: str, src: Path, ranges: list[tuple[int, int]]):
    parts = [js_header(title, str(src.relative_to(ROOT)).replace("\\", "/"), ", ".join(f"{a}-{b}" for a, b in ranges))]
    for a, b in ranges:
        parts.append(f"\n/* ===== {src.name} L{a}-L{b} ===== */\n")
        parts.append(read_lines(src, a, b))
    (OUT / "frontend" / out_name).write_text("".join(parts), encoding="utf-8")


def count_project_lines():
    stats = {".py": 0, ".js": 0, ".css": 0, ".html": 0}
    per_file = []
    for p in ROOT.rglob("*"):
        if not p.is_file() or "gridvision_core_submit" in p.parts:
            continue
        if any(x in p.parts for x in (".git", "__pycache__")):
            continue
        ext = p.suffix.lower()
        if ext not in stats:
            continue
        n = sum(1 for _ in open(p, encoding="utf-8", errors="ignore"))
        stats[ext] += n
        per_file.append((n, p))
    return stats, per_file


CORE_KEEP = {
    "core/inspection_point_generator.py",
    "core/inspection_point_detector.py",
    "core/real_map_cv.py",
    "core/topo.py",
    "core/topo_task.py",
    "core/topo_plan.py",
    "core/topo_global_optimizer.py",
    "core/image_pixel_coords.py",
    "planner/powerline_planner_v3_final.py",
    "planner/mission_result_builder.py",
    "planner/replan_start_end.py",
    "planner/topo_dijkstra.py",
    "app.py",
    "visualization/dashboard_map.py",
    "web/static/app.js",
    "web/static/playback.js",
    "web/static/mission_store.js",
    "web/index.html",
    "demo/demo_visualization_main.py",
}

CORE_PARTIAL = {
    "config/settings.py": "配置项",
    "core/independent_lines.py": "线路提取辅助",
    "core/physical_line_chain.py": "物理链几何",
    "core/real_satellite_manual.py": "成都卫星图人工标注",
}

REMOVE_REASONS = {
    "weather/": "非答辩核心：风场模型，UI 已移除",
    "scripts/": "诊断脚本，非核心算法",
    "result/": "运行产物与调试输出",
    "docs/": "设计文档，已单独整理 project_architecture.md",
    "data/": "原始数据，提交包仅含 sample_dashboard.json",
    "maps/": "示例 world 配置",
    "web/static/style.css": "样式细节，答辩展示非核心",
    "web/static/mission_store.js": "状态层，逻辑已并入 app/playback 摘录",
    "demo/": "CLI 入口包装，核心在 planner",
    "input/": "空 stub",
    "archive/": "历史归档",
    "experiment/": "不存在/已删除",
}


def audit_file(rel: str) -> tuple[str, str]:
    rel = rel.replace("\\", "/")
    if rel in CORE_KEEP:
        return "KEEP", "答辩核心模块（完整保留或已摘录）"
    if rel in CORE_PARTIAL:
        return "KEEP", f"辅助模块：{CORE_PARTIAL[rel]}"
    for prefix, reason in REMOVE_REASONS.items():
        if rel.startswith(prefix) or rel == prefix.rstrip("/"):
            return "REMOVE", reason
    if rel.endswith(".png") or rel.endswith(".jpg") or rel.endswith(".svg"):
        return "REMOVE", "媒体/占位资源，非核心代码"
    if rel.endswith(".json") and not rel.endswith("sample_dashboard.json"):
        return "REMOVE", "数据/产物 JSON，提交包仅保留 sample"
    if rel.endswith(".md") and not rel.startswith("gridvision_core_submit/"):
        return "REMOVE", "文档，已汇总至 submit/docs"
    if rel.endswith(".html") and "web/index" not in rel:
        return "REMOVE", "生成物 HTML"
    return "REMOVE", "非答辩核心或未引用"


def build_audit(per_file):
    lines = ["# GridVision 文件审计报告\n", "自动生成，供毕设答辩代码筛选参考。\n\n"]
    keep, remove = [], []
    for n, p in sorted(per_file, key=lambda x: str(x[1])):
        rel = str(p.relative_to(ROOT)).replace("\\", "/")
        action, reason = audit_file(rel)
        entry = (rel, n, reason)
        (keep if action == "KEEP" else remove).append(entry)
    lines.append("## KEEP\n\n")
    for rel, n, reason in keep:
        lines.append(f"### KEEP\n\n`{rel}` ({n} 行)\n\n原因：{reason}\n\n")
    lines.append("## REMOVE\n\n")
    for rel, n, reason in remove:
        lines.append(f"### REMOVE\n\n`{rel}` ({n} 行)\n\n原因：{reason}\n\n")
    (OUT / "docs" / "file_audit_report.md").write_text("".join(lines), encoding="utf-8")
    return keep, remove


def main():
    for sub in ("backend", "frontend", "data", "docs"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)

    # Backend extracts
    extract_py(
        "01_point_generation.py",
        "巡检点生成",
        ROOT / "core/inspection_point_generator.py",
        [(166, 313), (530, 621)],
        "含沿折线采样、图像检测点吸附到拓扑。",
    )
    parts_mission = [
        py_header(
            "巡检任务生成",
            "planner/powerline_planner_v3_final.py + core/topo_task.py",
            "2364-2510, 644-722",
            "PowerlinePlannerV3 巡检点步骤 + build_edge_tasks。",
        )
    ]
    parts_mission.append("\n# ===== powerline_planner_v3_final.py L2364-L2510 =====\n")
    parts_mission.append(read_lines(ROOT / "planner/powerline_planner_v3_final.py", 2364, 2510))
    parts_mission.append("\n# ===== topo_task.py L644-L722 =====\n")
    parts_mission.append(read_lines(ROOT / "core/topo_task.py", 644, 722))
    (OUT / "backend" / "02_mission_generation.py").write_text("".join(parts_mission), encoding="utf-8")

    extract_py(
        "03_path_planning.py",
        "拓扑路径规划",
        ROOT / "planner/topo_dijkstra.py",
        [(98, 165), (303, 400)],
        "Dijkstra 最短路径与连接段生成。",
    )
    extract_py(
        "04_replanning.py",
        "起终点重规划",
        ROOT / "planner/replan_start_end.py",
        [(1401, 1705)],
        "build_start_end_replan_mission 完整摘录。",
    )
    extract_py(
        "05_dashboard_export.py",
        "Dashboard 导出",
        ROOT / "planner/mission_result_builder.py",
        [(292, 360), (839, 950)],
        "CV 坐标抽取 + build_dashboard_from_mission_json。",
    )

    # Frontend extracts
    extract_js(
        "01_map_render.js",
        "地图路径可视化",
        ROOT / "web/static/app.js",
        [(1505, 1710), (1898, 2250), (2307, 2531)],
    )
    extract_js(
        "02_playback.js",
        "巡检播放",
        ROOT / "web/static/playback.js",
        [(1345, 1434), (1615, 1680)],
    )
    extract_js(
        "03_image_stream.js",
        "图传切换",
        ROOT / "web/static/playback.js",
        [(496, 551)],
    )

    # Sample dashboard
    src_dash = ROOT / "result/web_app/latest_dashboard.json"
    if src_dash.exists():
        (OUT / "data/sample_dashboard.json").write_text(
            src_dash.read_text(encoding="utf-8"), encoding="utf-8"
        )

    stats, per_file = count_project_lines()
    keep, remove = build_audit(per_file)

    core_py = sum(1 for _ in (OUT / "backend").glob("*.py") for __ in [open(_, encoding="utf-8")])
    core_lines = {}
    for folder, ext in (("backend", ".py"), ("frontend", ".js")):
        core_lines[ext] = sum(
            sum(1 for _ in open(f, encoding="utf-8", errors="ignore"))
            for f in (OUT / folder).glob(f"*{ext}")
        )
    core_total = sum(core_lines.values())

    orig_total = sum(stats.values())
    ratio = 100.0 * core_total / orig_total if orig_total else 0

    stats_md = f"""# GridVision 代码统计

## 原项目

| 类型 | 行数 |
|------|------|
| Python | {stats['.py']} |
| JavaScript | {stats['.js']} |
| CSS | {stats['.css']} |
| HTML | {stats['.html']} |
| **总计** | **{orig_total}** |

## 核心提交版 (gridvision_core_submit)

| 类型 | 行数 |
|------|------|
| Python (backend/) | {core_lines.get('.py', 0)} |
| JavaScript (frontend/) | {core_lines.get('.js', 0)} |
| **总计** | **{core_total}** |

## 压缩比例

摘录核心代码约占原工程 **{ratio:.1f}%**（按行数计）。

> 说明：答辩包 intentionally 只保留算法与播放核心摘录，不含 CSS/HTML/配置/数据文件全量。
"""
    (OUT / "docs/code_statistics.md").write_text(stats_md, encoding="utf-8")

    arch = """# GridVision 项目架构（答辩版）

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
"""
    (OUT / "docs/project_architecture.md").write_text(arch, encoding="utf-8")

    readme = f"""# GridVision 毕设答辩核心代码包

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

- 原工程约 {orig_total} 行 → 本包摘录约 {core_total} 行（见 docs/code_statistics.md）

## 文件说明

详见 `docs/file_audit_report.md` 与 `docs/project_architecture.md`。
"""
    (OUT / "README.md").write_text(readme, encoding="utf-8")

    print("DONE", OUT)
    print("orig", orig_total, "core", core_total, f"{ratio:.1f}%")


if __name__ == "__main__":
    main()
