"""
UAV 电网巡检 — Image Pipeline Web Dashboard

启动:
    python app.py

访问:
    http://127.0.0.1:8001

图像管线: result/latest/mission_output.json + data/test.png
"""

from __future__ import annotations

import json
import os
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

from planner.mission_result_builder import (
    apply_custom_markers,
    build_dashboard_from_mission_json,
    build_image_pipeline_dashboard,
    build_mission_context,
    normalize_inspection_point_source,
    save_json,
)
from planner.replan_start_end import (
    ReplanValidationError,
    build_start_end_replan_mission,
    replan_image_bounds_from_baseline,
    validate_image_coords,
)
from visualization.dashboard_map import get_background_map_config, resolve_map_path

WEB_DIR = ROOT / "web"
STATIC_DIR = WEB_DIR / "static"
DATA_DIR = ROOT / "data"
OUTPUT_DIR = ROOT / "result" / "web_app"
LATEST_DIR = ROOT / "result" / "latest"
LATEST_MISSION_PATH = LATEST_DIR / "mission_output.json"
DEFAULT_MAP_PATH = "data/test.png"
DEFAULT_POINT_MAP_PATH = "data/test_point.png"
IMAGE_DATASETS_PATH = DATA_DIR / "image_datasets.json"
REAL_SATELLITE_POINT_MAP = "data/chengdu_real_point.png"
REAL_SATELLITE_DISPLAY_MAP = REAL_SATELLITE_POINT_MAP
REAL_SATELLITE_LINE_MAP = "data/chengdu_real_line.png"
REAL_SATELLITE_POINT_MAP = "data/chengdu_real_point.png"
InspectionPointSource = Literal["spacing", "image"]
DEMO_VISUALIZATION_SCRIPT = ROOT / "demo" / "demo_visualization_main.py"
_image_gen_lock = threading.Lock()
_image_generating = False
_current_mission_context: Dict[str, Any] = {}

app = FastAPI(
    title="无人机电网巡检任务控制中心",
    description="图像管线 Dashboard",
    version="1.2.0",
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
if DATA_DIR.is_dir():
    app.mount("/data", StaticFiles(directory=str(DATA_DIR)), name="data_files")


@app.on_event("startup")
async def _debug_log_app_identity():
    """启动时打印本文件路径；若由 python app.py 启动，环境变量含实际监听 URL。"""
    app_path = Path(__file__).resolve()
    listen = os.environ.get("_UAV_APP_LISTEN_URL", "")
    print("\n" + "=" * 60, flush=True)
    print("[app startup] FastAPI 已加载，当前 app 定义自:", flush=True)
    print(f"  __file__ = {app_path}", flush=True)
    if listen:
        print(f"  监听地址 = {listen}", flush=True)
        print(f"  健康检查: {listen}/debug/ping", flush=True)
    else:
        print(
            "  监听地址 = (未设置 _UAV_APP_LISTEN_URL；可能由 uvicorn CLI 启动，请看上方 'Uvicorn running on ...')",
            flush=True,
        )
    print("=" * 60 + "\n", flush=True)


class PlanRequest(BaseModel):
    inspection_point_source: InspectionPointSource = "spacing"
    image_path: str = DEFAULT_MAP_PATH
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    end_x: Optional[float] = None
    end_y: Optional[float] = None


class ImageGenerateRequest(BaseModel):
    force: bool = False
    inspection_point_source: InspectionPointSource = "image"
    image_path: str = DEFAULT_MAP_PATH


def _load_image_dataset_registry() -> Dict[str, Any]:
    if not IMAGE_DATASETS_PATH.exists():
        return {
            "default_id": "legacy_test",
            "datasets": [
                {
                    "id": "legacy_test",
                    "dataset": DEFAULT_POINT_MAP_PATH,
                    "clean_map_image": "auto",
                    "line_image": DEFAULT_MAP_PATH,
                    "point_image": DEFAULT_POINT_MAP_PATH,
                    "dataset_type": "legacy",
                }
            ],
        }
    with IMAGE_DATASETS_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)


def _image_dataset_profiles() -> List[Dict[str, Any]]:
    return list(_load_image_dataset_registry().get("datasets") or [])


def resolve_image_dataset_profile(image_path: str) -> Dict[str, Any]:
    """根据请求路径匹配图像数据集配置（真实卫星 / 旧 test 图）。"""
    rel = str(image_path or "").replace("\\", "/")
    profiles = _image_dataset_profiles()
    for profile in profiles:
        candidates = {
            str(profile.get("dataset") or ""),
            str(profile.get("point_image") or ""),
            str(profile.get("line_image") or ""),
            str(profile.get("clean_map_image") or ""),
        }
        if rel and rel in candidates:
            return dict(profile)
    default_id = _load_image_dataset_registry().get("default_id")
    for profile in profiles:
        if profile.get("id") == default_id:
            return dict(profile)
    if profiles:
        return dict(profiles[0])
    return {
        "id": "legacy_test",
        "dataset": DEFAULT_POINT_MAP_PATH,
        "clean_map_image": "auto",
        "line_image": DEFAULT_MAP_PATH,
        "point_image": DEFAULT_POINT_MAP_PATH,
        "dataset_type": "legacy",
    }


def resolve_detection_image_path(
    profile: Dict[str, Any],
    inspection_point_source: str,
) -> str:
    source = normalize_inspection_point_source(inspection_point_source)
    if source == "image":
        return str(profile.get("point_image") or profile.get("dataset") or DEFAULT_POINT_MAP_PATH)
    return str(profile.get("line_image") or profile.get("dataset") or DEFAULT_MAP_PATH)


def resolve_display_map_path(profile: Dict[str, Any]) -> Optional[str]:
    if str(profile.get("dataset_type") or "") == "real_satellite":
        return str(
            profile.get("point_image")
            or profile.get("clean_map_image")
            or profile.get("dataset")
            or REAL_SATELLITE_POINT_MAP
        )
    clean = str(profile.get("clean_map_image") or "")
    if clean and clean != "auto":
        return clean
    return None


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
    image_path = str(meta.get("map_image") or meta.get("point_image") or DEFAULT_MAP_PATH)
    display_map = str(meta.get("clean_map_image") or meta.get("display_map_image") or "")
    dataset_type = str(meta.get("dataset_type") or "")
    return {
        "inspection_point_source": source,
        "image_path": image_path,
        "display_map_image": display_map,
        "dataset_type": dataset_type,
    }


def _update_current_mission_context(context: Dict[str, Any]) -> None:
    global _current_mission_context
    _current_mission_context = dict(context or {})


def _run_image_demo_main(
    *,
    inspection_point_source: str = "spacing",
    image_path: str = DEFAULT_MAP_PATH,
    display_map_path: Optional[str] = None,
    dataset_profile: Optional[Dict[str, Any]] = None,
) -> None:
    """调用图像主线 demo（优先 import main，失败则 subprocess）。"""
    import os

    os.environ["UAV_INSPECTION_SOURCE"] = inspection_point_source
    os.environ["UAV_IMAGE_PATH"] = image_path
    if display_map_path:
        os.environ["UAV_DISPLAY_MAP"] = display_map_path
    else:
        os.environ.pop("UAV_DISPLAY_MAP", None)
    os.environ.pop("UAV_MANUAL_JSON_PATH", None)
    if dataset_profile:
        os.environ["UAV_DATASET_TYPE"] = str(dataset_profile.get("dataset_type") or "")
        if str(dataset_profile.get("dataset_type") or "") == "real_satellite":
            os.environ["UAV_IMAGE_PIXEL_COORDS"] = "1"
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
        if display_map_path:
            env["UAV_DISPLAY_MAP"] = display_map_path
        elif "UAV_DISPLAY_MAP" in env:
            del env["UAV_DISPLAY_MAP"]
        if "UAV_MANUAL_JSON_PATH" in env:
            del env["UAV_MANUAL_JSON_PATH"]
        if dataset_profile:
            env["UAV_DATASET_TYPE"] = str(dataset_profile.get("dataset_type") or "")
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
    image_path = _resolve_image_path(image_path)

    existing_config: Dict[str, str] = {}
    if LATEST_MISSION_PATH.exists():
        try:
            with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
                existing_config = _mission_generation_config(json.load(f))
        except Exception:
            existing_config = {}

    profile = resolve_image_dataset_profile(image_path)
    display_map = resolve_display_map_path(profile)
    config_changed = (
        existing_config.get("inspection_point_source") != inspection_point_source
        or existing_config.get("image_path") != image_path
        or (
            display_map
            and existing_config.get("display_map_image") != display_map
        )
    )
    if config_changed:
        force = True

    if LATEST_MISSION_PATH.exists() and not force:
        return {
            "ok": True,
            "generated": False,
            "mission_file": mission_rel,
            "message": "已使用现有图像主线 mission_output.json",
        }

    with _image_gen_lock:
        if _image_generating:
            return {
                "ok": False,
                "generated": False,
                "mission_file": mission_rel,
                "message": "图像主线正在生成中，请稍后重试",
            }

        if LATEST_MISSION_PATH.exists() and not force:
            return {
                "ok": True,
                "generated": False,
                "mission_file": mission_rel,
                "message": "已使用现有图像主线 mission_output.json",
            }

        _image_generating = True
        try:
            LATEST_DIR.mkdir(parents=True, exist_ok=True)
            _run_image_demo_main(
                inspection_point_source=inspection_point_source,
                image_path=image_path,
                display_map_path=display_map,
                dataset_profile=profile,
            )
        finally:
            _image_generating = False

    if not LATEST_MISSION_PATH.exists():
        return {
            "ok": False,
            "generated": False,
            "mission_file": mission_rel,
            "message": "图像主线执行完成，但未找到 mission_output.json",
        }

    return {
        "ok": True,
        "generated": True,
        "mission_file": mission_rel,
        "message": "已自动生成图像主线 mission（demo_visualization_main）",
    }


def _replan_dashboard_from_start_end(
    *,
    mission_data: Dict[str, Any],
    start_xy: List[float],
    end_xy: List[float],
    inspection_point_source: str,
    detection_path: str,
    display_map: Optional[str],
    map_rel_for_dashboard: str,
) -> Dict[str, Any]:
    """起终点重规划：保留巡检点，仅重建路径段。"""
    iw, ih = replan_image_bounds_from_baseline(mission_data)
    try:
        validate_image_coords(start_xy, end_xy, width=iw, height=ih)
    except ReplanValidationError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    ctx = build_mission_context(
        mission_data,
        request={
            "inspection_point_source": inspection_point_source,
            "image_path": detection_path,
            "display_map_image": display_map,
        },
    )
    try:
        new_mission = build_start_end_replan_mission(
            mission_data,
            start_xy,
            end_xy,
            mission_context=ctx,
        )
    except ReplanValidationError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Replan failed: {e}") from e

    meta = new_mission.get("metadata") or {}
    map_rel = str(
        meta.get("clean_map_image")
        or meta.get("display_map_image")
        or meta.get("map_image")
        or display_map
        or map_rel_for_dashboard
    )
    map_bg = get_background_map_config(ROOT, map_rel)
    dashboard = build_dashboard_from_mission_json(
        new_mission,
        pipeline="image",
        source="plan:start_end",
        planner="start_end_replan",
        input_file=map_rel,
        spacing=0,
        map_background=map_bg,
        coordinate_mode="image_fixed",
        extra_metadata={
            "planner": "start_end_replan",
            "inspection_point_source": normalize_inspection_point_source(inspection_point_source),
            "map_image": detection_path,
            "clean_map_image": display_map or meta.get("clean_map_image"),
            "display_map_image": display_map or meta.get("display_map_image"),
            "replan": {"start": start_xy, "end": end_xy},
        },
        root=ROOT,
    )
    apply_custom_markers(
        dashboard,
        start=(float(start_xy[0]), float(start_xy[1])),
        end=(float(end_xy[0]), float(end_xy[1])),
    )
    dashboard["image_inspection_overlay"] = (
        new_mission.get("image_inspection_overlay")
        or meta.get("image_inspection_overlay")
        or []
    )
    print(
        f"[plan-replan] start=({start_xy[0]:.1f},{start_xy[1]:.1f}) "
        f"end=({end_xy[0]:.1f},{end_xy[1]:.1f}) segments={len(dashboard.get('segments') or [])}",
        flush=True,
    )
    return dashboard


async def _plan_image_pipeline(
    *,
    inspection_point_source: str = "spacing",
    image_path: str = DEFAULT_MAP_PATH,
    start_xy: Optional[List[float]] = None,
    end_xy: Optional[List[float]] = None,
) -> Dict[str, Any]:
    """图像主线：自动生成（如需）并加载 result/latest/mission_output.json。"""
    profile = resolve_image_dataset_profile(image_path)
    detection_path = _resolve_image_path(
        resolve_detection_image_path(profile, inspection_point_source)
    )
    display_map = resolve_display_map_path(profile)
    if display_map:
        display_map = _resolve_image_path(display_map)
    gen = ensure_image_pipeline_outputs(
        force=False,
        inspection_point_source=inspection_point_source,
        image_path=detection_path,
    )
    if not gen.get("ok"):
        raise HTTPException(status_code=500, detail=gen.get("message", "Image pipeline generation failed"))

    map_rel_for_dashboard = display_map or detection_path
    try:
        dashboard = build_image_pipeline_dashboard(
            LATEST_MISSION_PATH, ROOT, map_rel=map_rel_for_dashboard
        )
    except FileNotFoundError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    snapshot = OUTPUT_DIR / "latest_image_mission_snapshot.json"
    with LATEST_MISSION_PATH.open("r", encoding="utf-8") as f:
        mission_data = json.load(f)
    save_json(snapshot, mission_data)

    if start_xy is not None and end_xy is not None:
        dashboard = _replan_dashboard_from_start_end(
            mission_data=mission_data,
            start_xy=start_xy,
            end_xy=end_xy,
            inspection_point_source=inspection_point_source,
            detection_path=detection_path,
            display_map=display_map,
            map_rel_for_dashboard=map_rel_for_dashboard,
        )

    dashboard["metadata"]["image_generation"] = gen
    dashboard["metadata"]["inspection_point_source"] = normalize_inspection_point_source(
        inspection_point_source
    )
    dashboard["metadata"]["map_image"] = detection_path
    dashboard["metadata"]["point_image"] = str(profile.get("point_image") or detection_path)
    dashboard["metadata"]["line_image"] = str(profile.get("line_image") or detection_path)
    if display_map:
        dashboard["metadata"]["clean_map_image"] = display_map
        dashboard["metadata"]["display_map_image"] = display_map
        if dashboard.get("map_background"):
            map_bg = get_background_map_config(ROOT, display_map)
            dashboard["map_background"] = map_bg
    dashboard["metadata"]["dataset"] = str(profile.get("dataset") or detection_path)
    dashboard["metadata"]["dataset_type"] = str(profile.get("dataset_type") or "legacy")
    dashboard["metadata"]["image_dataset_id"] = str(profile.get("id") or "")
    if dashboard["metadata"].get("dataset_type") == "real_satellite" or "chengdu_real_point" in str(
        map_rel_for_dashboard
    ):
        mb = dashboard.get("map_background") or {}
        iw, ih = mb.get("width"), mb.get("height")
        dashboard["metadata"]["coordinate_mode"] = "image_pixel_fixed"
        dashboard["metadata"]["pixel_coordinate_mode"] = True
        if iw and ih:
            dashboard["metadata"]["image_width"] = iw
            dashboard["metadata"]["image_height"] = ih
            dashboard["bounds"] = {
                "x_range": [0, float(iw)],
                "y_range": [float(ih), 0],
                "width": float(iw),
                "height": float(ih),
            }
    dashboard["image_inspection_overlay"] = dashboard.get("metadata", {}).get("image_inspection_overlay") or []
    dashboard["output_files"] = {
        "mission_snapshot": "latest_image_mission_snapshot.json",
        "source": "result/latest/mission_output.json",
    }

    save_json(OUTPUT_DIR / "latest_dashboard.json", dashboard)
    dashboard["output_files"]["dashboard"] = "latest_dashboard.json"

    _update_current_mission_context(
        build_mission_context(
            mission_data,
            request={
                "inspection_point_source": inspection_point_source,
                "image_path": detection_path,
                "display_map_image": display_map,
            },
            dashboard=dashboard,
        )
    )
    return dashboard


@app.get("/")
async def index():
    index_path = WEB_DIR / "index.html"
    if not index_path.exists():
        raise HTTPException(status_code=500, detail="web/index.html missing")
    return FileResponse(index_path)


@app.get("/api/image-datasets")
async def api_image_datasets():
    """图像管线数据集配置（检测输入 / 显示底图分离）。"""
    registry = _load_image_dataset_registry()
    return {
        "default_id": registry.get("default_id"),
        "datasets": _image_dataset_profiles(),
    }


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

    try:
        abs_path = str(path.resolve())
    except OSError:
        abs_path = str(path)
    print(f"[inspection-image-api] path={abs_path} exists=False", flush=True)

    if INSPECTION_PLACEHOLDER.exists():
        return FileResponse(INSPECTION_PLACEHOLDER, media_type="image/svg+xml")
    raise HTTPException(status_code=404, detail=f"Inspection image not found: {safe}")


@app.get("/api/image-mission/status")
async def api_image_mission_status():
    exists = LATEST_MISSION_PATH.exists()
    return {
        "available": exists,
        "path": "result/latest/mission_output.json",
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


@app.get("/debug/ping")
async def debug_ping():
    return {"ok": True, "app": "current app.py"}


@app.post("/api/plan")
async def api_plan(req: PlanRequest):
    image_path = _resolve_image_path(req.image_path)
    start_xy: Optional[List[float]] = None
    end_xy: Optional[List[float]] = None
    if all(v is not None for v in (req.start_x, req.start_y, req.end_x, req.end_y)):
        start_xy = [float(req.start_x), float(req.start_y)]
        end_xy = [float(req.end_x), float(req.end_y)]
    dashboard = await _plan_image_pipeline(
        inspection_point_source=req.inspection_point_source,
        image_path=image_path,
        start_xy=start_xy,
        end_xy=end_xy,
    )
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
    listen_url = f"http://{host}:{port}"
    os.environ["_UAV_APP_LISTEN_URL"] = listen_url

    print("=" * 60)
    print("无人机巡检任务控制中心（图像管线）")
    print("=" * 60)
    print(f"[DEBUG] 当前 app 文件: {Path(__file__).resolve()}")
    print(f"[DEBUG] 实际监听端口: {port}  (若 8001 被占用会自动递增)")
    print(f"[DEBUG] 健康检查: {listen_url}/debug/ping")
    print("=" * 60)
    print(f"URL: http://{host}:{port}")
    print(f"  图像管线: result/latest/mission_output.json")
    print("=" * 60)
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
