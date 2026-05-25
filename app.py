"""
UAV 电网巡检 — 统一 Web Dashboard

启动:
    python app.py

访问:
    http://127.0.0.1:8001

双管线:
  A. Image  — result/latest/mission_output.json + data/test.png
  B. Unified — data/sample_*.json 实时规划
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from input.unified_input import load_unified_input_from_json
from input.unified_mission_adapter import export_unified_mission, unified_input_to_mission
from input.unified_task_adapter import unified_input_to_edge_tasks
from planner.mission_analysis import analyze_mission, compare_mission_stats
from planner.mission_optimizer import build_optimized_unified_mission
from planner.mission_result_builder import (
    apply_custom_markers,
    build_dashboard_from_mission_json,
    build_dashboard_payload,
    build_image_pipeline_dashboard,
    build_mission_context,
    is_image_inspection_source,
    normalize_inspection_point_source,
    save_json,
)
from planner.replan_start_end import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    ReplanValidationError,
    build_start_end_replan_mission,
    validate_image_coords,
)
from visualization.dashboard_map import get_background_map_config, resolve_map_path
from weather.weather_cost import (
    compute_mission_weather_stats,
    load_weather_zones,
    make_weather_context,
)

WEB_DIR = ROOT / "web"
STATIC_DIR = WEB_DIR / "static"
DATA_DIR = ROOT / "data"
WEATHER_SAMPLE_PATH = DATA_DIR / "weather_sample.json"
OUTPUT_DIR = ROOT / "result" / "web_app"
LATEST_DIR = ROOT / "result" / "latest"
LATEST_MISSION_PATH = LATEST_DIR / "mission_output.json"
LATEST_HTML_PATH = LATEST_DIR / "main_view_interactive.html"
DEFAULT_MAP_PATH = "data/test.png"
DEFAULT_POINT_MAP_PATH = "data/test_point.png"
InspectionPointSource = Literal["spacing", "image"]
DEMO_VISUALIZATION_SCRIPT = ROOT / "demo" / "demo_visualization_main.py"
_image_gen_lock = threading.Lock()
_image_generating = False
_current_mission_context: Dict[str, Any] = {}

PipelineName = Literal["image", "unified"]
PlannerName = Literal["baseline", "optimized", "dijkstra", "legacy"]
MapModeName = Literal["topology_only", "image_overlay"]

app = FastAPI(
    title="无人机电网巡检任务控制中心",
    description="双管线：图像管线（传统 PNG）+ 统一 JSON 管线",
    version="1.1.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


class PlanRequest(BaseModel):
    pipeline: PipelineName = "unified"
    input_file: str = ""
    planner: str = "baseline"
    spacing: float = Field(50.0, ge=5.0, le=500.0)
    map_mode: MapModeName = "topology_only"
    weather_aware: bool = False
    weather_weight: float = Field(1.0, ge=0.0, le=5.0)
    inspection_point_source: InspectionPointSource = "spacing"
    image_path: str = DEFAULT_MAP_PATH


class ImageGenerateRequest(BaseModel):
    force: bool = False
    inspection_point_source: InspectionPointSource = "spacing"
    image_path: str = DEFAULT_MAP_PATH


class ReplanRequest(BaseModel):
    start: List[float] = Field(..., min_length=2, max_length=2)
    end: List[float] = Field(..., min_length=2, max_length=2)
    pipeline: PipelineName = "image"
    planning_spacing: float = Field(70.0, ge=20.0, le=300.0)
    weather_aware: bool = False
    weather_weight: float = Field(1.0, ge=0.0, le=5.0)
    inspection_point_source: Optional[str] = None
    image_path: Optional[str] = None
    dataset: Optional[str] = None
    spacing: Optional[float] = None
    snap_threshold: Optional[float] = None
    inspection_points: Optional[List[Dict[str, Any]]] = None


def _resolve_input_path(input_file: str) -> Path:
    p = Path(input_file)
    if not p.is_absolute():
        p = ROOT / p
    p = p.resolve()
    if not p.exists():
        raise HTTPException(status_code=404, detail=f"未找到输入文件: {input_file}")
    data_root = DATA_DIR.resolve()
    try:
        p.relative_to(data_root)
    except ValueError:
        raise HTTPException(status_code=400, detail="输入文件必须位于 data/ 目录下") from None
    return p


def _build_weather_context(enabled: bool, weather_weight: float) -> Dict[str, Any]:
    zones = load_weather_zones(WEATHER_SAMPLE_PATH)
    return make_weather_context(
        enabled=enabled,
        weather_weight=weather_weight,
        weather_zones=zones,
    )


def _attach_weather_dashboard_fields(
    dashboard: Dict[str, Any],
    weather_ctx: Dict[str, Any],
) -> None:
    zones = weather_ctx.get("weather_zones") or []
    stats = compute_mission_weather_stats(
        dashboard.get("segments") or [],
        zones,
        type_weights=weather_ctx.get("type_weights"),
        weather_weight=float(weather_ctx.get("weather_weight", 1.0)),
    ) if weather_ctx.get("enabled") else {
        "weather_penalty_total": 0.0,
        "weather_affected_edges": 0,
    }
    dashboard.setdefault("statistics", {}).update(stats)
    total_length = float(dashboard.get("statistics", {}).get("total_length", 0.0) or 0.0)
    weather_penalty_total = float(dashboard.get("statistics", {}).get("weather_penalty_total", 0.0) or 0.0)
    dashboard["statistics"]["total_cost"] = round(total_length + weather_penalty_total, 3)
    dashboard["statistics"]["weather_mode"] = "on" if weather_ctx.get("enabled") else "off"
    dashboard.setdefault("metadata", {}).update(
        {
            "weather_mode": "on" if weather_ctx.get("enabled") else "off",
            "weather_aware": bool(weather_ctx.get("enabled")),
            "weather_weight": float(weather_ctx.get("weather_weight", 1.0)),
            "weather_summary": {
                "top_zones": stats.get("top_weather_zones", []),
                "top_segments": stats.get("top_weather_segments", []),
            },
        }
    )
    dashboard["weather_zones"] = zones


def _resolve_image_path(image_path: str) -> str:
    rel = str(image_path or DEFAULT_MAP_PATH).replace("\\", "/")
    path = resolve_map_path(ROOT, rel)
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Map image not found: {rel}")
    try:
        path.relative_to(DATA_DIR.resolve())
    except ValueError:
        raise HTTPException(status_code=400, detail="图像文件必须位于 data/ 目录下") from None
    return rel


def _mission_generation_config(mission_data: Optional[Dict[str, Any]] = None) -> Dict[str, str]:
    meta = (mission_data or {}).get("metadata") or {}
    source = normalize_inspection_point_source(meta.get("inspection_point_source"))
    image_path = str(meta.get("map_image") or DEFAULT_MAP_PATH)
    return {
        "inspection_point_source": source,
        "image_path": image_path,
    }


def _update_current_mission_context(context: Dict[str, Any]) -> None:
    global _current_mission_context
    _current_mission_context = dict(context or {})


def _build_replan_mission_context(
    baseline: Dict[str, Any],
    req: ReplanRequest,
    *,
    dashboard: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    request_payload = req.model_dump(exclude_none=True)
    cached_dashboard = dashboard or {}
    if not cached_dashboard and _current_mission_context:
        cached_dashboard = {
            "metadata": {
                k: _current_mission_context.get(k)
                for k in (
                    "inspection_point_source",
                    "map_image",
                    "clean_map_image",
                    "image_detection_stats",
                    "image_inspection_overlay",
                    "input_file",
                    "pipeline",
                    "planner",
                    "spacing",
                )
            },
            "inspection_points": _current_mission_context.get("inspection_points") or [],
        }
    context = build_mission_context(
        baseline,
        request=request_payload,
        dashboard=cached_dashboard,
    )
    if is_image_inspection_source(context.get("inspection_point_source")):
        if not context.get("inspection_points"):
            context["inspection_points"] = cached_dashboard.get("inspection_points") or []
        if not context.get("image_inspection_overlay"):
            context["image_inspection_overlay"] = (
                baseline.get("image_inspection_overlay")
                or (baseline.get("metadata") or {}).get("image_inspection_overlay")
                or []
            )
    _update_current_mission_context(context)
    return context


def _run_image_demo_main(
    *,
    inspection_point_source: str = "spacing",
    image_path: str = DEFAULT_MAP_PATH,
) -> None:
    """调用图像主线 demo（优先 import main，失败则 subprocess）。"""
    import os

    os.environ["UAV_INSPECTION_SOURCE"] = inspection_point_source
    os.environ["UAV_IMAGE_PATH"] = image_path
    try:
        from demo.demo_visualization_main import main as demo_main

        demo_main(
            image_path=image_path,
            inspection_point_source=inspection_point_source,
        )
        return
    except Exception as import_err:
        if not DEMO_VISUALIZATION_SCRIPT.exists():
            raise RuntimeError(
                f"未找到 demo 脚本: {DEMO_VISUALIZATION_SCRIPT}"
            ) from import_err

        env = os.environ.copy()
        env["UAV_INSPECTION_SOURCE"] = inspection_point_source
        env["UAV_IMAGE_PATH"] = image_path
        proc = subprocess.run(
            [sys.executable, str(DEMO_VISUALIZATION_SCRIPT)],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
        )
        if proc.returncode != 0:
            err_tail = (proc.stderr or proc.stdout or "")[-4000:]
            raise RuntimeError(
                f"demo_visualization_main.py 执行失败（返回码 {proc.returncode}）:\n{err_tail}"
            ) from import_err


def ensure_image_pipeline_outputs(
    force: bool = False,
    *,
    inspection_point_source: str = "spacing",
    image_path: str = DEFAULT_MAP_PATH,
) -> Dict[str, Any]:
    """
    确保 result/latest/mission_output.json 存在。
    不存在或 force=True 时自动运行图像主线 demo。
    """
    global _image_generating

    mission_rel = "result/latest/mission_output.json"
    html_rel = "result/latest/main_view_interactive.html"
    image_path = _resolve_image_path(image_path)

    existing_config: Dict[str, str] = {}
    if LATEST_MISSION_PATH.exists():
        try:
            with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
                existing_config = _mission_generation_config(json.load(f))
        except Exception:
            existing_config = {}

    config_changed = (
        existing_config.get("inspection_point_source") != inspection_point_source
        or existing_config.get("image_path") != image_path
    )
    if config_changed:
        force = True

    if LATEST_MISSION_PATH.exists() and not force:
        return {
            "ok": True,
            "generated": False,
            "mission_file": mission_rel,
            "html_file": html_rel if LATEST_HTML_PATH.exists() else None,
            "message": "已使用现有图像主线 mission_output.json",
        }

    with _image_gen_lock:
        if _image_generating:
            return {
                "ok": False,
                "generated": False,
                "mission_file": mission_rel,
                "html_file": None,
                "message": "图像主线正在生成中，请稍后重试",
            }

        if LATEST_MISSION_PATH.exists() and not force:
            return {
                "ok": True,
                "generated": False,
                "mission_file": mission_rel,
                "html_file": html_rel if LATEST_HTML_PATH.exists() else None,
                "message": "已使用现有图像主线 mission_output.json",
            }

        _image_generating = True
        try:
            LATEST_DIR.mkdir(parents=True, exist_ok=True)
            _run_image_demo_main(
                inspection_point_source=inspection_point_source,
                image_path=image_path,
            )
        finally:
            _image_generating = False

    if not LATEST_MISSION_PATH.exists():
        return {
            "ok": False,
            "generated": False,
            "mission_file": mission_rel,
            "html_file": None,
            "message": "图像主线执行完成，但未找到 mission_output.json",
        }

    return {
        "ok": True,
        "generated": True,
        "mission_file": mission_rel,
        "html_file": html_rel if LATEST_HTML_PATH.exists() else None,
        "message": "已自动生成图像主线 mission（demo_visualization_main）",
    }


async def _run_image_replan(
    start: List[float],
    end: List[float],
    planning_spacing: float = 70.0,
    weather_context: Optional[Dict[str, Any]] = None,
    mission_context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    图像主线起终点重规划：planner/replan_start_end.py。
    仅写入 result/web_app/，不覆盖 result/latest/。
    """
    try:
        validate_image_coords(start, end, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    except ReplanValidationError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    gen = ensure_image_pipeline_outputs(force=False)
    if not gen.get("ok"):
        raise HTTPException(
            status_code=500,
            detail=gen.get("message", "Image pipeline not ready for replan"),
        )

    with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
        baseline = json.load(f)

    ctx = mission_context or build_mission_context(baseline)
    _update_current_mission_context(ctx)

    try:
        new_mission = build_start_end_replan_mission(
            baseline,
            start,
            end,
            planning_spacing=planning_spacing,
            weather_context=weather_context,
            mission_context=ctx,
        )
    except ReplanValidationError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Replan failed: {e}") from e

    replan_meta = new_mission.get("replan_metadata") or {}
    base_meta = baseline.get("metadata") or {}
    map_rel = str(
        ctx.get("clean_map_image")
        or base_meta.get("clean_map_image")
        or ctx.get("image_path")
        or base_meta.get("map_image")
        or DEFAULT_MAP_PATH
    )
    map_bg = get_background_map_config(ROOT, map_rel)
    resolved_source = normalize_inspection_point_source(
        replan_meta.get("inspection_point_source")
        or ctx.get("inspection_point_source")
        or base_meta.get("inspection_point_source")
    )
    dashboard = build_dashboard_from_mission_json(
        new_mission,
        pipeline="image",
        source="replan:start_end",
        planner="start_end_replan",
        input_file=map_rel,
        spacing=planning_spacing,
        map_background=map_bg,
        coordinate_mode="image_fixed",
        root=ROOT,
        extra_metadata={
            "planner": "start_end_replan",
            "start": replan_meta.get("start", start),
            "end": replan_meta.get("end", end),
            "end_connected": replan_meta.get("end_connected", False),
            "planning_spacing": replan_meta.get("planning_spacing", planning_spacing),
            "planning_point_count": replan_meta.get("planning_point_count"),
            "inspection_point_source": resolved_source,
            "map_image": ctx.get("image_path") or base_meta.get("map_image") or DEFAULT_MAP_PATH,
            "clean_map_image": ctx.get("clean_map_image") or base_meta.get("clean_map_image"),
            "image_detection_stats": ctx.get("image_detection_stats") or base_meta.get("image_detection_stats") or {},
            "image_inspection_overlay": ctx.get("image_inspection_overlay") or base_meta.get("image_inspection_overlay") or [],
            "replan": replan_meta,
            "image_generation": gen,
            "weather_mode": "on" if (weather_context or {}).get("enabled") else "off",
            "weather_weight": float((weather_context or {}).get("weather_weight", 1.0)),
        },
    )
    apply_custom_markers(
        dashboard,
        start=(float(start[0]), float(start[1])),
        end=(float(end[0]), float(end[1])),
    )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    save_json(OUTPUT_DIR / "latest_replan_mission.json", new_mission)
    _attach_weather_dashboard_fields(dashboard, weather_context or make_weather_context())
    save_json(OUTPUT_DIR / "latest_replan_dashboard.json", dashboard)
    dashboard["output_files"] = {
        "mission_snapshot": "latest_replan_mission.json",
        "dashboard": "latest_replan_dashboard.json",
        "source": "replan:start_end",
        "legacy_html": "/legacy/html",
    }
    return dashboard


def _list_datasets() -> List[Dict[str, str]]:
    datasets = []
    for f in sorted(DATA_DIR.glob("sample_*.json")):
        rel = f.relative_to(ROOT).as_posix()
        datasets.append({"id": f.name, "path": rel, "label": f.name})
    return datasets


def _run_baseline(unified, spacing: float) -> Dict[str, Any]:
    return unified_input_to_mission(
        unified,
        spacing=spacing,
        merge_thresh=25.0,
        enable_sa=False,
        eps=150.0,
    )


def _run_optimized(
    unified,
    spacing: float,
    connect_planner: str,
    weather_cost_config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    topo_graph, edge_tasks, line_pts = unified_input_to_edge_tasks(
        unified, spacing=spacing, merge_thresh=25.0
    )
    mission, edge_order, _edge_dirs = build_optimized_unified_mission(
        topo_graph,
        edge_tasks,
        eps=150.0,
        connect_planner=connect_planner,
        cost_config=weather_cost_config,
    )
    return {
        "topo_graph": topo_graph,
        "edge_tasks": edge_tasks,
        "line_inspection_points_by_line": line_pts,
        "mission": mission,
        "connect_planner": connect_planner,
    }


async def _plan_image_pipeline(
    *,
    inspection_point_source: str = "spacing",
    image_path: str = DEFAULT_MAP_PATH,
) -> Dict[str, Any]:
    """图像主线：自动生成（如需）并加载 result/latest/mission_output.json。"""
    image_path = _resolve_image_path(image_path)
    gen = ensure_image_pipeline_outputs(
        force=False,
        inspection_point_source=inspection_point_source,
        image_path=image_path,
    )
    if not gen.get("ok"):
        raise HTTPException(status_code=500, detail=gen.get("message", "Image pipeline generation failed"))

    try:
        dashboard = build_image_pipeline_dashboard(LATEST_MISSION_PATH, ROOT, map_rel=image_path)
    except FileNotFoundError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    snapshot = OUTPUT_DIR / "latest_image_mission_snapshot.json"
    with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
        mission_data = json.load(f)
    save_json(snapshot, mission_data)

    dashboard["metadata"]["image_generation"] = gen
    dashboard["metadata"]["inspection_point_source"] = normalize_inspection_point_source(
        inspection_point_source
    )
    dashboard["metadata"]["map_image"] = image_path
    dashboard["image_inspection_overlay"] = dashboard.get("metadata", {}).get("image_inspection_overlay") or []
    dashboard["output_files"] = {
        "mission_snapshot": "latest_image_mission_snapshot.json",
        "source": "result/latest/mission_output.json",
        "legacy_html": "/legacy/html",
    }
    if LATEST_HTML_PATH.exists():
        dashboard["output_files"]["legacy_html_path"] = "result/latest/main_view_interactive.html"

    _attach_weather_dashboard_fields(
        dashboard,
        _build_weather_context(enabled=False, weather_weight=1.0),
    )
    save_json(OUTPUT_DIR / "latest_dashboard.json", dashboard)
    dashboard["output_files"]["dashboard"] = "latest_dashboard.json"

    _update_current_mission_context(
        build_mission_context(
            mission_data,
            request={
                "inspection_point_source": inspection_point_source,
                "image_path": image_path,
            },
            dashboard=dashboard,
        )
    )
    return dashboard


async def _plan_unified_pipeline(req: PlanRequest) -> Dict[str, Any]:
    if not req.input_file:
        raise HTTPException(status_code=400, detail="统一管线模式必须提供 input_file")

    input_path = _resolve_input_path(req.input_file)
    planner = req.planner.lower()

    if planner not in ("baseline", "optimized", "dijkstra"):
        raise HTTPException(
            status_code=400,
            detail="统一管线规划器必须是 baseline | optimized | dijkstra",
        )

    try:
        unified = load_unified_input_from_json(str(input_path))
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"输入文件加载失败: {e}") from e

    connect_planner = "bfs"
    connect_note: Optional[str] = None
    weather_ctx = _build_weather_context(
        enabled=bool(req.weather_aware),
        weather_weight=float(req.weather_weight),
    )
    weather_cost_cfg = {"weather": weather_ctx}

    if planner == "baseline":
        mission_result = _run_baseline(unified, req.spacing)
        planner_label = "baseline"
        meta_planner_name = "baseline_global_topology"
    elif planner == "optimized":
        optimized_connect = "dijkstra" if weather_ctx.get("enabled") else "bfs"
        mission_result = _run_optimized(
            unified,
            req.spacing,
            connect_planner=optimized_connect,
            weather_cost_config=weather_cost_cfg,
        )
        planner_label = "optimized"
        connect_planner = optimized_connect
        if weather_ctx.get("enabled"):
            connect_note = "天气感知已开启：optimized 自动使用 Dijkstra 连接代价。"
        meta_planner_name = "topology_aware_optimized"
    else:
        mission_result = _run_optimized(
            unified,
            req.spacing,
            connect_planner="dijkstra",
            weather_cost_config=weather_cost_cfg,
        )
        planner_label = "dijkstra"
        connect_planner = "dijkstra"
        connect_note = (
            "请求 Dijkstra 连接规划；单段不可达时已 fallback 至 BFS（planner/topo_dijkstra）。"
        )
        if weather_ctx.get("enabled"):
            connect_note += " 已叠加天气惩罚代价。"
        meta_planner_name = "topology_aware_optimized+dijkstra_connect"

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    mission_json_path = OUTPUT_DIR / "latest_mission.json"

    export_unified_mission(
        mission_result,
        unified,
        output_path=mission_json_path,
        input_file=str(input_path),
    )

    with mission_json_path.open("r", encoding="utf-8") as f:
        exported = json.load(f)
    exported.setdefault("metadata", {})["planner_name"] = meta_planner_name
    exported["metadata"]["dashboard_planner"] = planner_label
    exported["metadata"]["pipeline"] = "unified"
    exported["metadata"]["weather_mode"] = "on" if weather_ctx.get("enabled") else "off"
    exported["metadata"]["weather_weight"] = float(weather_ctx.get("weather_weight", 1.0))
    with mission_json_path.open("w", encoding="utf-8") as f:
        json.dump(exported, f, indent=2, ensure_ascii=False)
        f.write("\n")

    analysis_stats = analyze_mission(mission_result["mission"])
    save_json(
        OUTPUT_DIR / "latest_analysis.json",
        {
            "pipeline": "unified",
            "planner": planner_label,
            "input_file": str(input_path.relative_to(ROOT)),
            "statistics": analysis_stats,
            "weather_mode": "on" if weather_ctx.get("enabled") else "off",
        },
    )

    compare_data: Optional[Dict[str, Any]] = None
    if planner in ("optimized", "dijkstra"):
        try:
            baseline_result = _run_baseline(unified, req.spacing)
            baseline_stats = analyze_mission(baseline_result["mission"])
            compare_data = {
                "pipeline": "unified",
                "planner": planner_label,
                "input_file": str(input_path.relative_to(ROOT)),
                "baseline": baseline_stats,
                "current": analysis_stats,
                "improvement": compare_mission_stats(baseline_stats, analysis_stats),
            }
            save_json(OUTPUT_DIR / "latest_compare.json", compare_data)
        except Exception as e:
            compare_data = {"error": str(e)}

    output_files: Dict[str, str] = {
        "mission": "latest_mission.json",
        "analysis": "latest_analysis.json",
    }
    if compare_data and "error" not in compare_data:
        output_files["compare"] = "latest_compare.json"

    unified_map_mode = "topology_only"
    if req.map_mode == "image_overlay":
        unified_map_mode = "topology_only"

    dashboard = build_dashboard_payload(
        mission_result,
        planner=planner_label,
        input_file=str(input_path.relative_to(ROOT)),
        spacing=req.spacing,
        connect_planner=connect_planner,
        connect_planner_note=connect_note,
        output_files=output_files,
        extra_metadata={
            "planner_name": meta_planner_name,
            "weather_mode": "on" if weather_ctx.get("enabled") else "off",
            "weather_aware": bool(weather_ctx.get("enabled")),
            "weather_weight": float(weather_ctx.get("weather_weight", 1.0)),
        },
        map_mode=unified_map_mode,
        root=ROOT,
    )
    dashboard["metadata"]["requested_map_mode"] = req.map_mode
    if req.map_mode == "image_overlay":
        dashboard["metadata"]["map_mode_note"] = (
            "Unified 管线不使用 data/test.png；已自动切换为 topology 自适应视口。"
        )
    _attach_weather_dashboard_fields(dashboard, weather_ctx)

    save_json(OUTPUT_DIR / "latest_dashboard.json", dashboard)
    output_files["dashboard"] = "latest_dashboard.json"
    dashboard["output_files"] = output_files

    return dashboard


@app.get("/")
async def index():
    index_path = WEB_DIR / "index.html"
    if not index_path.exists():
        raise HTTPException(status_code=500, detail="web/index.html missing")
    return FileResponse(index_path)


@app.get("/api/datasets")
async def api_datasets():
    return {"datasets": _list_datasets()}


@app.get("/api/map/config")
async def api_map_config(path: str = DEFAULT_MAP_PATH):
    """Image Pipeline 底图配置。"""
    map_rel = _resolve_image_path(path)
    return get_background_map_config(ROOT, map_rel)


@app.get("/api/map/background")
async def api_map_background(path: str = DEFAULT_MAP_PATH):
    map_rel = _resolve_image_path(path)
    map_path = resolve_map_path(ROOT, map_rel)
    return FileResponse(map_path, media_type="image/png")


INSPECTION_SAMPLE_IMAGES = [
    ROOT / "data" / "section1" / "1.png",
    ROOT / "data" / "section1" / "2.png",
    ROOT / "data" / "section1" / "3.png",
]
INSPECTION_IMAGE_DIR = ROOT / "figures" / "inspection_images"
INSPECTION_PLACEHOLDER = STATIC_DIR / "inspection_placeholder.svg"


@app.get("/api/inspection-sample/{idx}")
async def api_inspection_sample(idx: int):
    """Dashboard 巡检点示例图（无 image_ref 时轮换展示）。"""
    paths = [p for p in INSPECTION_SAMPLE_IMAGES if p.exists()]
    if not paths:
        raise HTTPException(status_code=404, detail="No inspection sample images")
    path = paths[idx % len(paths)]
    return FileResponse(path, media_type="image/png")


@app.get("/api/inspection-file")
async def api_inspection_file(path: str):
    """安全提供项目内巡检图片（相对 ROOT）。"""
    safe = Path(path).as_posix().lstrip("/")
    if ".." in safe.split("/"):
        raise HTTPException(status_code=400, detail="Invalid path")
    target = (ROOT / safe).resolve()
    try:
        target.relative_to(ROOT.resolve())
    except ValueError:
        raise HTTPException(status_code=400, detail="Path outside project") from None
    if not target.is_file():
        raise HTTPException(status_code=404, detail="Image not found")
    return FileResponse(target)


@app.get("/api/inspection-image/{filename}")
async def api_inspection_image(filename: str):
    """
    返回 figures/inspection_images 中的巡检图。
    若文件不存在，返回 placeholder。
    """
    safe = Path(filename).name
    if safe != filename:
        raise HTTPException(status_code=400, detail="Invalid filename")
    ext = Path(safe).suffix.lower()
    if ext not in (".jpg", ".jpeg", ".png"):
        raise HTTPException(status_code=400, detail="Unsupported image type")

    path = (INSPECTION_IMAGE_DIR / safe).resolve()
    try:
        path.relative_to(INSPECTION_IMAGE_DIR.resolve())
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid image path") from None

    if path.is_file():
        media = "image/png" if ext == ".png" else "image/jpeg"
        return FileResponse(path, media_type=media)

    if INSPECTION_PLACEHOLDER.exists():
        return FileResponse(INSPECTION_PLACEHOLDER, media_type="image/svg+xml")
    raise HTTPException(status_code=404, detail=f"Inspection image not found: {safe}")


@app.get("/api/image-mission/status")
async def api_image_mission_status():
    exists = LATEST_MISSION_PATH.exists()
    return {
        "available": exists,
        "path": "result/latest/mission_output.json",
        "html_available": LATEST_HTML_PATH.exists(),
        "html_path": "result/latest/main_view_interactive.html",
        "generating": _image_generating,
        "message": None if exists else "mission 不存在，点击加载将自动生成",
    }


@app.post("/api/image-mission/generate")
async def api_image_mission_generate(req: ImageGenerateRequest):
    image_path = _resolve_image_path(req.image_path)
    result = ensure_image_pipeline_outputs(
        force=req.force,
        inspection_point_source=req.inspection_point_source,
        image_path=image_path,
    )
    status = 200 if result.get("ok") else 500
    return JSONResponse(result, status_code=status)


@app.get("/legacy/html")
async def legacy_html():
    """旧版交互式主展示页（UAV 动画 / 重规划）。"""
    if not LATEST_HTML_PATH.exists():
        raise HTTPException(
            status_code=404,
            detail="main_view_interactive.html not found. Run image pipeline generate first.",
        )
    return FileResponse(LATEST_HTML_PATH, media_type="text/html")


@app.post("/api/replan")
async def api_replan(req: ReplanRequest):
    """起点驱动重规划（Image Pipeline）。结果仅保存至 result/web_app/。"""
    if req.pipeline != "image":
        return JSONResponse(
            {
                "ok": False,
                "segments": [],
                "statistics": {},
                "message": "当前仅支持 pipeline=image 的重规划",
            },
            status_code=400,
        )

    try:
        weather_ctx = _build_weather_context(
            enabled=bool(req.weather_aware),
            weather_weight=float(req.weather_weight),
        )
        with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
            baseline = json.load(f)
        mission_context = _build_replan_mission_context(baseline, req)
        dashboard = await _run_image_replan(
            req.start,
            req.end,
            planning_spacing=req.planning_spacing,
            weather_context=weather_ctx,
            mission_context=mission_context,
        )
    except HTTPException:
        raise
    except Exception as e:
        return JSONResponse(
            {
                "ok": False,
                "segments": [],
                "statistics": {},
                "message": str(e),
            },
            status_code=500,
        )

    return JSONResponse(
        {
            "ok": True,
            "segments": dashboard.get("segments", []),
            "statistics": dashboard.get("statistics", {}),
            "visit_order": dashboard.get("visit_order", []),
            "inspection_points": dashboard.get("inspection_points", []),
            "markers": dashboard.get("markers", {}),
            "map_background": dashboard.get("map_background"),
            "bounds": dashboard.get("bounds"),
            "metadata": dashboard.get("metadata", {}),
            "output_files": dashboard.get("output_files", {}),
            "message": dashboard.get("metadata", {})
            .get("replan", {})
            .get("end_note", "Replan completed"),
            "dashboard": dashboard,
        }
    )


@app.post("/api/plan")
async def api_plan(req: PlanRequest):
    pipeline = req.pipeline.lower()

    if pipeline == "image":
        image_path = _resolve_image_path(req.image_path)
        dashboard = await _plan_image_pipeline(
            inspection_point_source=req.inspection_point_source,
            image_path=image_path,
        )
    elif pipeline == "unified":
        dashboard = await _plan_unified_pipeline(req)
    else:
        raise HTTPException(status_code=400, detail="pipeline must be image | unified")

    return JSONResponse(dashboard)


@app.get("/api/output/{filename}")
async def api_output(filename: str):
    safe = Path(filename).name
    if safe != filename:
        raise HTTPException(status_code=400, detail="Invalid filename")
    path = OUTPUT_DIR / safe
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {safe}")
    return FileResponse(path, media_type="application/json", filename=safe)


def main():
    import socket
    import uvicorn

    def find_free_port(host: str = "127.0.0.1", start: int = 8001, max_tries: int = 50) -> int:
        for port in range(start, start + max_tries):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.bind((host, port))
                    return port
                except OSError:
                    continue
        raise RuntimeError(f"未找到可用端口，尝试范围: {start}–{start + max_tries - 1}")

    host = "127.0.0.1"
    port = find_free_port(host)

    print("=" * 60)
    print("无人机巡检任务控制中心（双管线）")
    print("=" * 60)
    print(f"URL: http://{host}:{port}")
    print(f"  图像管线: result/latest/mission_output.json + data/test.png")
    print(f"  统一管线: data/sample_*.json")
    print(f"传统页面: http://{host}:{port}/legacy/html")
    print("=" * 60)
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
