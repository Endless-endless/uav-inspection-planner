"""
将 Mission / EdgeTask 结果转换为 Web Dashboard 统一 JSON 结构。

前端仅依赖本模块输出的字典，不直接访问 Python 类。
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

INSPECTION_IMAGE_REL_DIR = Path("figures") / "inspection_images"


def _normalize_dashboard_inspection_point_id(pt: Dict[str, Any], index: int) -> str:
    """
    Dashboard 稳定图片 id：已有 IP_nnnn 则规范化零填充；否则按顺序分配 IP_0001…
    不把 L_/point_ 等用作 figures/inspection_images 下的文件名键。
    """
    raw = str(pt.get("point_id") or pt.get("id") or "").strip()
    m = re.match(r"^IP_(\d+)$", raw, re.IGNORECASE)
    if m:
        return f"IP_{int(m.group(1)):04d}"
    return f"IP_{index + 1:04d}"


def _inspection_ip_image_filename(point_id: Optional[str]) -> Optional[str]:
    """
    巡检拍图文件名：point_id=IP_0001 → IP_0001.jpg（位于 figures/inspection_images/）。
    """
    if not point_id or not isinstance(point_id, str):
        return None
    pid = point_id.strip()
    if not pid.upper().startswith("IP_"):
        return None
    base = Path(pid.replace("\\", "/")).name
    low = base.lower()
    if low.endswith((".jpg", ".jpeg", ".png")):
        return base
    return f"{base}.jpg"


def _inspection_image_stats_dict(
    inspection_points: List[Dict[str, Any]],
    root: Optional[Path],
) -> Dict[str, int]:
    """磁盘巡检图数量 + 与 IP_* 巡检点的文件对齐统计。"""
    img_dir = (root / INSPECTION_IMAGE_REL_DIR) if root else None
    total_images = 0
    if img_dir and img_dir.is_dir():
        exts = {".jpg", ".jpeg", ".png"}
        total_images = sum(1 for p in img_dir.iterdir() if p.is_file() and p.suffix.lower() in exts)
    mapped_points = 0
    missing_images = 0
    for pt in inspection_points or []:
        fn = _inspection_ip_image_filename(str(pt.get("point_id") or pt.get("id") or ""))
        if not fn:
            continue
        ok = bool(img_dir and (img_dir / fn).is_file())
        if ok:
            mapped_points += 1
        else:
            missing_images += 1
    return {
        "total_images": total_images,
        "mapped_points": mapped_points,
        "missing_images": missing_images,
    }


def _log_inspection_image_stats_console(stats: Dict[str, Any]) -> None:
    print(f"[inspection-image] total_images={int(stats.get('total_images', 0))}")
    print(f"[inspection-image] mapped_points={int(stats.get('mapped_points', 0))}")
    print(f"[inspection-image] missing_images={int(stats.get('missing_images', 0))}")


IMAGE_INSPECTION_SOURCES = frozenset(
    {"image", "image_points", "image-point", "image_detected", "image-point-source"}
)


def normalize_inspection_point_source(source: Optional[str]) -> str:
    raw = str(source or "spacing").strip().lower()
    if raw in IMAGE_INSPECTION_SOURCES:
        return "image"
    if raw == "spacing":
        return raw
    return "spacing"


def is_image_inspection_source(source: Optional[str]) -> bool:
    return normalize_inspection_point_source(source) == "image"


def is_image_inspection_point(point: Dict[str, Any]) -> bool:
    ptype = str(point.get("point_type") or point.get("type") or "").lower()
    reason = str(point.get("source_reason") or point.get("description") or "").lower()
    pid = str(point.get("point_id") or point.get("id") or "")
    if ptype == "image_detected":
        return True
    if "image" in reason or "black_dot" in reason:
        return True
    if pid.startswith("IP_"):
        return True
    if point.get("detection_result"):
        return True
    if point.get("raw_coord") or point.get("snapped_coord"):
        return True
    return False


def merge_mission_metadata_into_dashboard(
    metadata: Dict[str, Any],
    mission_data: Optional[Dict[str, Any]],
) -> None:
    mission_meta = (mission_data or {}).get("metadata") or {}
    if not mission_meta:
        return
    metadata["mission_metadata"] = mission_meta
    for key in (
        "inspection_point_source",
        "map_image",
        "clean_map_image",
        "display_map_image",
        "point_image",
        "line_image",
        "coordinate_mode",
        "pixel_coordinate_mode",
        "image_width",
        "image_height",
        "image_detection_stats",
        "image_inspection_overlay",
        "topo_edges_pixel",
    ):
        cur = metadata.get(key)
        missing = cur in (None, "", {}) or (
            key == "topo_edges_pixel" and isinstance(cur, list) and len(cur) == 0
        )
        if key in mission_meta and missing:
            metadata[key] = mission_meta[key]
    overlay = (mission_data or {}).get("image_inspection_overlay")
    if overlay and not metadata.get("image_inspection_overlay"):
        metadata["image_inspection_overlay"] = overlay


def build_mission_context(
    baseline: Dict[str, Any],
    *,
    request: Optional[Dict[str, Any]] = None,
    dashboard: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    base_meta = baseline.get("metadata") or {}
    nested_mm = base_meta.get("mission_metadata") if isinstance(base_meta.get("mission_metadata"), dict) else {}
    dash_meta = (dashboard or {}).get("metadata") or {}
    req = request or {}

    inspection_point_source = normalize_inspection_point_source(
        req.get("inspection_point_source")
        or dash_meta.get("inspection_point_source")
        or base_meta.get("inspection_point_source")
    )
    image_path = (
        req.get("image_path")
        or dash_meta.get("map_image")
        or base_meta.get("map_image")
        or nested_mm.get("map_image")
        or "data/test.png"
    )
    snap_threshold = req.get("snap_threshold")
    if snap_threshold is None:
        stats = base_meta.get("image_detection_stats") or dash_meta.get("image_detection_stats") or {}
        cfg = stats.get("detector_config") or {}
        snap_threshold = cfg.get("snap_threshold", 30.0)

    inspection_points = req.get("inspection_points")
    if not inspection_points:
        cached = (dashboard or {}).get("inspection_points")
        if cached and is_image_inspection_source(inspection_point_source):
            inspection_points = cached

    overlay = (
        baseline.get("image_inspection_overlay")
        or base_meta.get("image_inspection_overlay")
        or nested_mm.get("image_inspection_overlay")
        or dash_meta.get("image_inspection_overlay")
        or []
    )

    return {
        "dataset": req.get("dataset") or dash_meta.get("input_file") or base_meta.get("map_image"),
        "pipeline": req.get("pipeline") or dash_meta.get("pipeline") or "image",
        "planner": req.get("planner") or dash_meta.get("planner") or "legacy",
        "inspection_point_source": inspection_point_source,
        "image_path": str(image_path),
        "spacing": float(req.get("spacing") if req.get("spacing") is not None else dash_meta.get("spacing") or 50.0),
        "snap_threshold": float(snap_threshold or 30.0),
        "inspection_points": inspection_points or [],
        "image_inspection_overlay": overlay,
        "image_detection_stats": base_meta.get("image_detection_stats") or dash_meta.get("image_detection_stats") or {},
        "clean_map_image": base_meta.get("clean_map_image")
        or dash_meta.get("clean_map_image")
        or nested_mm.get("clean_map_image"),
        "weather_aware": bool(req.get("weather_aware")),
        "weather_weight": float(req.get("weather_weight") or 1.0),
    }


def _segment_to_dict(seg: Any, index: int) -> Dict[str, Any]:
    geometry = getattr(seg, "geometry", None) or []
    geom_2d = [[float(p[0]), float(p[1])] for p in geometry if len(p) >= 2]
    return {
        "segment_id": f"seg_{index:04d}",
        "type": getattr(seg, "type", "unknown"),
        "edge_id": getattr(seg, "edge_id", None),
        "from_edge_id": getattr(seg, "from_edge_id", None),
        "to_edge_id": getattr(seg, "to_edge_id", None),
        "length": round(float(getattr(seg, "length", 0.0) or 0.0), 2),
        "direction": getattr(seg, "direction", None),
        "geometry_2d": geom_2d,
    }


def _point_segment_distance(x: float, y: float, geometry: List[List[float]]) -> float:
    if len(geometry) < 2:
        return float("inf")
    best = float("inf")
    for a, b in zip(geometry, geometry[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        if denom <= 1e-9:
            d = ((x - ax) ** 2 + (y - ay) ** 2) ** 0.5
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / denom))
            px, py = ax + t * dx, ay + t * dy
            d = ((x - px) ** 2 + (y - py) ** 2) ** 0.5
        if d < best:
            best = d
    return best


def _nearest_inspect_segment_edge_id(x: float, y: float, mission: Any) -> Optional[str]:
    best: Optional[Tuple[float, str]] = None
    for seg in getattr(mission, "segments", None) or []:
        if getattr(seg, "type", None) != "inspect":
            continue
        edge_id = getattr(seg, "edge_id", None)
        geometry = getattr(seg, "geometry", None) or []
        if not edge_id or not str(edge_id).startswith("PL_"):
            continue
        d = _point_segment_distance(x, y, geometry)
        if best is None or d < best[0]:
            best = (d, str(edge_id))
    return best[1] if best is not None else None


def _finite_float(v: Any) -> Optional[float]:
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def _xy_from_sequence(seq: Any) -> Optional[Tuple[float, float]]:
    if seq is None:
        return None
    try:
        if len(seq) < 2:
            return None
    except TypeError:
        return None
    xf = _finite_float(seq[0])
    yf = _finite_float(seq[1])
    if xf is None or yf is None:
        return None
    return (xf, yf)


def _detection_dict(pt: Any) -> Dict[str, Any]:
    if isinstance(pt, dict):
        dr = pt.get("detection_result")
        return dr if isinstance(dr, dict) else {}
    dr = getattr(pt, "detection_result", None)
    return dr if isinstance(dr, dict) else {}


def extract_cv_dashboard_xy_and_snapped(
    pt: Any,
) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
    """
    Dashboard 主坐标 x/y：优先 CV / 原始像素，不得用 snapped_coord 作为主来源。
    顺序：original_pixel_x/y → raw_x/y → detection_result.raw_coord → pixel_position / position_2d / position。
    吸附/投影坐标单独返回供 snapped_x / snapped_y / metadata。
    """
    dr = _detection_dict(pt)
    display: Optional[Tuple[float, float]] = None

    if isinstance(pt, dict):
        ox = _finite_float(pt.get("original_pixel_x"))
        oy = _finite_float(pt.get("original_pixel_y"))
        if ox is not None and oy is not None:
            display = (ox, oy)
        if display is None:
            rx = _finite_float(pt.get("raw_x"))
            ry = _finite_float(pt.get("raw_y"))
            if rx is not None and ry is not None:
                display = (rx, ry)
        if display is None:
            display = _xy_from_sequence(dr.get("raw_coord"))
        if display is None and isinstance(pt, dict):
            display = _xy_from_sequence(pt.get("raw_coord"))
        if display is None:
            display = _xy_from_sequence(
                pt.get("pixel_position")
                or pt.get("position_2d")
                or pt.get("position")
                or pt.get("pos2d")
            )
    else:
        ox = _finite_float(getattr(pt, "original_pixel_x", None))
        oy = _finite_float(getattr(pt, "original_pixel_y", None))
        if ox is not None and oy is not None:
            display = (ox, oy)
        if display is None:
            rx = _finite_float(getattr(pt, "raw_x", None))
            ry = _finite_float(getattr(pt, "raw_y", None))
            if rx is not None and ry is not None:
                display = (rx, ry)
        if display is None:
            display = _xy_from_sequence(dr.get("raw_coord"))
        if display is None:
            display = _xy_from_sequence(getattr(pt, "raw_coord", None))
        if display is None:
            pos = getattr(pt, "pixel_position", None) or getattr(pt, "pos2d", None) or getattr(pt, "position", None)
            display = _xy_from_sequence(pos)

    snapped: Optional[Tuple[float, float]] = _xy_from_sequence(dr.get("snapped_coord"))
    if snapped is None and isinstance(pt, dict):
        snapped = _xy_from_sequence(pt.get("snapped_coord"))
    if snapped is None and isinstance(pt, dict):
        sx = _finite_float(pt.get("snapped_x"))
        sy = _finite_float(pt.get("snapped_y"))
        if sx is not None and sy is not None:
            snapped = (sx, sy)
    if snapped is None and not isinstance(pt, dict):
        snapped = _xy_from_sequence(getattr(pt, "snapped_coord", None))
    if snapped is None and not isinstance(pt, dict):
        sx = _finite_float(getattr(pt, "snapped_x", None))
        sy = _finite_float(getattr(pt, "snapped_y", None))
        if sx is not None and sy is not None:
            snapped = (sx, sy)

    return display, snapped


def _log_dashboard_inspection_point_rows(points: List[Dict[str, Any]]) -> None:
    """控制台：每条 Dashboard inspection_point 的坐标溯源。"""
    for row in points:
        print(
            "[dashboard-inspection-point]",
            {
                "point_id": row.get("point_id") or row.get("id"),
                "x": row.get("x"),
                "y": row.get("y"),
                "raw_x": row.get("raw_x"),
                "raw_y": row.get("raw_y"),
                "snapped_x": row.get("snapped_x"),
                "snapped_y": row.get("snapped_y"),
                "edge_id": row.get("edge_id"),
            },
        )


def _collect_inspection_points(mission_result: Dict[str, Any]) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    mission = mission_result.get("mission")
    task_by_id = getattr(mission, "task_by_id", None) or {}
    source_tasks: List[Tuple[str, Any]] = []
    seen: set = set()

    result_tasks = (
        mission_result.get("mission_tasks")
        or mission_result.get("physical_line_chains")
        or []
    )
    result_task_map = {
        str(getattr(task, "edge_id", None) or getattr(task, "id", "")): task
        for task in result_tasks
        if getattr(task, "edge_id", None) or getattr(task, "id", None)
    }

    for edge_id in getattr(mission, "visit_order", None) or []:
        eid = str(edge_id).rstrip("+-")
        task = task_by_id.get(eid) or result_task_map.get(eid)
        if task is not None and eid not in seen:
            source_tasks.append((eid, task))
            seen.add(eid)

    for task_id, task in result_task_map.items():
        eid = str(task_id)
        if eid not in seen and getattr(task, "inspection_points", None):
            source_tasks.append((eid, task))
            seen.add(eid)

    for task_id, task in task_by_id.items():
        eid = str(task_id)
        if eid not in seen and getattr(task, "inspection_points", None):
            source_tasks.append((eid, task))
            seen.add(eid)

    # Legacy fallback: old chain-based missions may not populate mission.task_by_id.
    if not source_tasks:
        for task in mission_result.get("edge_tasks") or []:
            edge_id = getattr(task, "edge_id", None)
            if edge_id is not None:
                source_tasks.append((str(edge_id), task))

    for edge_id, task in source_tasks:
        for pt in getattr(task, "inspection_points", None) or []:
            display, snapped = extract_cv_dashboard_xy_and_snapped(pt)
            if not display:
                continue
            dx, dy = display[0], display[1]
            if isinstance(pt, dict):
                pid = pt.get("point_id") or pt.get("id")
                ptype = pt.get("point_type", "sample")
                sfb = pt.get("snap_fallback")
                sdist = pt.get("snap_distance_px")
            else:
                pid = getattr(pt, "point_id", None) or getattr(pt, "id", None)
                ptype = getattr(pt, "point_type", "sample")
                sfb = getattr(pt, "snap_fallback", None)
                dr = getattr(pt, "detection_result", None) or {}
                if sfb is None and isinstance(dr, dict):
                    sfb = dr.get("snap_fallback")
                sdist = getattr(pt, "snap_distance_px", None)
                if sdist is None and isinstance(dr, dict):
                    sdist = dr.get("snap_distance_px", dr.get("snap_distance"))
            row: Dict[str, Any] = {
                "point_id": pid,
                "id": pid,
                "edge_id": (
                    edge_id
                    if str(edge_id).startswith("PL_")
                    else _nearest_inspect_segment_edge_id(float(dx), float(dy), mission) or edge_id
                ),
                "x": float(dx),
                "y": float(dy),
                "raw_x": float(dx),
                "raw_y": float(dy),
                "point_type": ptype,
            }
            if snapped is not None:
                row["snapped_x"] = float(snapped[0])
                row["snapped_y"] = float(snapped[1])
                row["snapped_coord"] = [float(snapped[0]), float(snapped[1])]
            if sfb is not None:
                row["snap_fallback"] = bool(sfb)
            if sdist is not None:
                try:
                    row["snap_distance_px"] = float(sdist)
                except (TypeError, ValueError):
                    pass
            points.append(row)
    return points


def _start_end_from_segments(segments: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not segments:
        return {"start": None, "end": None}
    first_geom = segments[0].get("geometry_2d") or []
    last_geom = segments[-1].get("geometry_2d") or []
    start = first_geom[0] if first_geom else None
    end = last_geom[-1] if last_geom else None
    return {
        "start": {"x": start[0], "y": start[1]} if start else None,
        "end": {"x": end[0], "y": end[1]} if end else None,
    }


def compute_geometry_bounds(
    segments: List[Dict[str, Any]],
    inspection_points: Optional[List[Dict[str, Any]]] = None,
    padding_ratio: float = 0.08,
    min_padding: float = 20.0,
) -> Dict[str, Any]:
    """根据 mission 几何自动计算 Unified 管线视口（不使用 test.png 尺寸）。"""
    xs: List[float] = []
    ys: List[float] = []

    for seg in segments:
        for p in seg.get("geometry_2d") or []:
            if len(p) >= 2:
                xs.append(float(p[0]))
                ys.append(float(p[1]))

    for pt in inspection_points or []:
        if pt.get("x") is not None and pt.get("y") is not None:
            xs.append(float(pt["x"]))
            ys.append(float(pt["y"]))

    if not xs or not ys:
        return {
            "x_range": [0, 1000],
            "y_range": [1000, 0],
            "width": 1000,
            "height": 1000,
        }

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span_x = max(xmax - xmin, 1.0)
    span_y = max(ymax - ymin, 1.0)
    pad_x = max(span_x * padding_ratio, min_padding)
    pad_y = max(span_y * padding_ratio, min_padding)

    return {
        "x_range": [xmin - pad_x, xmax + pad_x],
        "y_range": [ymax + pad_y, ymin - pad_y],
        "width": round(xmax - xmin + 2 * pad_x, 2),
        "height": round(ymax - ymin + 2 * pad_y, 2),
    }


def _segments_from_json(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    segments = []
    for i, seg in enumerate(data.get("segments", [])):
        segments.append({
            "segment_id": seg.get("segment_id", f"seg_{i:04d}"),
            "type": seg.get("type"),
            "edge_id": seg.get("edge_id"),
            "from_edge_id": seg.get("from_edge_id"),
            "to_edge_id": seg.get("to_edge_id"),
            "length": round(float(seg.get("length", 0)), 2),
            "direction": seg.get("direction"),
            "geometry_2d": seg.get("geometry_2d") or [],
        })
    return segments


def _edge_to_segment_map(segments: List[Dict[str, Any]]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for seg in segments:
        sid = seg.get("segment_id")
        eid = seg.get("edge_id")
        if seg.get("type") == "inspect" and sid:
            if eid:
                mapping[eid] = sid
            mapping[sid] = sid
    return mapping


def _scan_inspection_image_catalog(root: Optional[Path]) -> List[Dict[str, str]]:
    """扫描 figures/inspection_images 下可用图片，按文件名排序。"""
    if root is None:
        return []
    img_dir = root / INSPECTION_IMAGE_REL_DIR
    if not img_dir.exists() or not img_dir.is_dir():
        return []

    exts = {".jpg", ".jpeg", ".png"}
    files = [p for p in img_dir.iterdir() if p.is_file() and p.suffix.lower() in exts]
    files.sort(key=lambda p: p.stem.lower())

    catalog: List[Dict[str, str]] = []
    for p in files:
        rel = p.relative_to(root).as_posix()
        catalog.append({
            "filename": p.name,
            "image_path": rel,
            "image_url": f"/api/inspection-image/{p.name}",
        })
    return catalog


def enrich_inspection_points_for_dashboard(
    points: List[Dict[str, Any]],
    segments: List[Dict[str, Any]],
    *,
    root: Optional[Path] = None,
    image_catalog: Optional[List[Dict[str, str]]] = None,
) -> List[Dict[str, Any]]:
    """扩展 Dashboard 巡检点字段（含 segment_id、priority、image_url 等）。"""
    _ = image_catalog  # 保留签名兼容；图片键统一为 IP_nnnn + /api/inspection-image/
    img_dir = (root / INSPECTION_IMAGE_REL_DIR) if root else None
    edge_map = _edge_to_segment_map(segments)
    placeholders = [
        "/static/inspection_placeholder.svg",
        "/static/inspection_placeholder_1.svg",
        "/static/inspection_placeholder_2.svg",
    ]

    enriched: List[Dict[str, Any]] = []
    meta_exclude = {
        "x",
        "y",
        "point_id",
        "id",
        "edge_id",
        "segment_id",
        "point_type",
        "priority",
        "image_path",
        "image_url",
        "image_placeholder",
        "image_available",
        "description",
        "status",
        "metadata",
        "raw_x",
        "raw_y",
        "snapped_x",
        "snapped_y",
        "raw_coord",
        "snapped_coord",
    }
    for i, pt in enumerate(points):
        eid = pt.get("edge_id")
        sid = pt.get("segment_id") or (edge_map.get(eid) if eid else None)
        normalized_pid = _normalize_dashboard_inspection_point_id(pt, i)
        ip_file = f"{normalized_pid}.jpg"
        rel_path = (INSPECTION_IMAGE_REL_DIR / ip_file).as_posix()
        image_url = f"/api/inspection-image/{ip_file}"
        image_path = rel_path
        file_exists = bool(img_dir and (img_dir / ip_file).is_file())
        image_available = file_exists
        print(
            "[inspection-image-map] "
            f"point_id={normalized_pid} image_url={image_url} exists={file_exists}"
        )

        status = pt.get("status") or "pending"
        priority = pt.get("priority") or (
            "high" if (pt.get("point_type") or "") in ("endpoint", "start", "end") else "normal"
        )
        description = pt.get("description") or pt.get("source_reason") or ""

        x = float(pt["x"])
        y = float(pt["y"])
        rx = _finite_float(pt.get("raw_x"))
        ry = _finite_float(pt.get("raw_y"))
        if rx is None or ry is None:
            rx, ry = x, y
        sx = _finite_float(pt.get("snapped_x"))
        sy = _finite_float(pt.get("snapped_y"))
        sc = pt.get("snapped_coord")
        if sx is None and isinstance(sc, (list, tuple)) and len(sc) >= 2:
            sx = _finite_float(sc[0])
            sy = _finite_float(sc[1])

        meta_inner: Dict[str, Any] = {
            k: v
            for k, v in pt.items()
            if k not in meta_exclude
        }
        if sx is not None and sy is not None:
            meta_inner.setdefault("snapped_coord", [float(sx), float(sy)])

        enriched.append({
            "id": normalized_pid,
            "point_id": normalized_pid,
            "x": x,
            "y": y,
            "raw_x": float(rx),
            "raw_y": float(ry),
            "snapped_x": float(sx) if sx is not None else None,
            "snapped_y": float(sy) if sy is not None else None,
            "segment_id": sid,
            "edge_id": eid,
            "point_type": pt.get("point_type") or "sample",
            "priority": priority,
            "image_path": image_path,
            "image_url": image_url,
            "image_available": image_available,
            "image_placeholder": placeholders[i % len(placeholders)],
            "description": description,
            "status": status,
            "progress_index": i + 1,
            "metadata": meta_inner,
        })
    return enriched


def _inspection_points_from_json(
    data: Dict[str, Any],
    segments: Optional[List[Dict[str, Any]]] = None,
    *,
    root: Optional[Path] = None,
    image_catalog: Optional[List[Dict[str, str]]] = None,
) -> List[Dict[str, Any]]:
    segments = segments or _segments_from_json(data)
    raw_points: List[Dict[str, Any]] = []
    meta = data.get("metadata") or {}
    pixel_mode = bool(
        meta.get("pixel_coordinate_mode")
        or str(meta.get("coordinate_mode") or "").startswith("image_pixel")
        or meta.get("dataset_type") == "real_satellite"
    )
    for pt in data.get("inspection_points", []):
        if not isinstance(pt, dict):
            continue
        display, snapped = extract_cv_dashboard_xy_and_snapped(pt)
        if not display:
            continue
        dr = pt.get("detection_result") or {}
        entry: Dict[str, Any] = {
            "point_id": pt.get("point_id"),
            "id": pt.get("point_id"),
            "edge_id": pt.get("edge_id"),
            "segment_id": pt.get("segment_id"),
            "x": float(display[0]),
            "y": float(display[1]),
            "raw_x": float(display[0]),
            "raw_y": float(display[1]),
            "point_type": pt.get("point_type") or pt.get("type") or "sample",
            "priority": pt.get("priority"),
            "image_path": pt.get("image_path"),
            "image_url": pt.get("image_url"),
            "image_ref": pt.get("image_ref"),
            "description": pt.get("description") or pt.get("source_reason"),
            "status": pt.get("status"),
            "group_id": pt.get("group_id"),
            "line_id": pt.get("line_id"),
            "visit_order": pt.get("visit_order"),
            "source_reason": pt.get("source_reason"),
            "raw_coord": dr.get("raw_coord") or list(display),
            "detection_valid": True,
        }
        if snapped is not None:
            entry["snapped_x"] = float(snapped[0])
            entry["snapped_y"] = float(snapped[1])
            entry["snapped_coord"] = [float(snapped[0]), float(snapped[1])]
        else:
            sc = dr.get("snapped_coord")
            if isinstance(sc, (list, tuple)) and len(sc) >= 2:
                entry["snapped_x"] = float(sc[0])
                entry["snapped_y"] = float(sc[1])
                entry["snapped_coord"] = [float(sc[0]), float(sc[1])]
        raw_points.append(entry)
    # 图像任务：部分导出版本仅有 metadata.image_inspection_overlay，无顶层 inspection_points
    if pixel_mode and not raw_points:
        overlay = meta.get("image_inspection_overlay") or data.get("image_inspection_overlay") or []
        for ov in overlay:
            if not isinstance(ov, dict):
                continue
            display, snapped = extract_cv_dashboard_xy_and_snapped(ov)
            if not display:
                rc = ov.get("raw_coord")
                if not rc or len(rc) < 2:
                    continue
                display = (float(rc[0]), float(rc[1]))
                snapped = _xy_from_sequence(ov.get("snapped_coord"))
            raw_points.append({
                "point_id": ov.get("id"),
                "id": ov.get("id"),
                "edge_id": ov.get("edge_id"),
                "segment_id": ov.get("segment_id"),
                "x": float(display[0]),
                "y": float(display[1]),
                "raw_x": float(display[0]),
                "raw_y": float(display[1]),
                "point_type": "image_detected",
                "priority": "high",
                "description": ov.get("source"),
                "status": "uninspected",
                "line_id": ov.get("line_id"),
                "source_reason": "image_inspection_overlay",
                "raw_coord": list(ov.get("raw_coord") or display),
                "detection_valid": bool(ov.get("valid", True)),
                "snap_fallback": not bool(ov.get("valid", True)),
                "snap_distance_px": ov.get("snap_distance"),
            })
            if snapped is not None:
                raw_points[-1]["snapped_x"] = float(snapped[0])
                raw_points[-1]["snapped_y"] = float(snapped[1])
                raw_points[-1]["snapped_coord"] = [float(snapped[0]), float(snapped[1])]
            elif ov.get("snapped_coord"):
                sc = ov["snapped_coord"]
                if isinstance(sc, (list, tuple)) and len(sc) >= 2:
                    raw_points[-1]["snapped_x"] = float(sc[0])
                    raw_points[-1]["snapped_y"] = float(sc[1])
                    raw_points[-1]["snapped_coord"] = [float(sc[0]), float(sc[1])]
    return enrich_inspection_points_for_dashboard(
        raw_points, segments, root=root, image_catalog=image_catalog
    )


def _statistics_from_json(data: Dict[str, Any], inspection_points: List[Dict[str, Any]]) -> Dict[str, Any]:
    stats_raw = data.get("statistics", {})
    total = float(stats_raw.get("total_length", 0))
    connect_len = float(stats_raw.get("connect_length", 0))
    inspect_len = float(stats_raw.get("inspect_length", 0))
    connect_ratio = (connect_len / total) if total > 0 else 0.0
    inspect_ratio_raw = stats_raw.get("inspect_ratio", 0)
    inspect_ratio = (
        float(inspect_ratio_raw) / 100.0
        if float(inspect_ratio_raw) > 1
        else float(inspect_ratio_raw)
    )

    visit = data.get("visit_order", {})
    visit_order = visit.get("edge_visit_order", []) if isinstance(visit, dict) else (visit or [])

    nip = len(inspection_points)
    return {
        "total_length": round(total, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "connect_ratio": round(connect_ratio, 4),
        "inspect_ratio": round(inspect_ratio, 4),
        "num_segments": stats_raw.get("num_segments", 0),
        "num_inspection_points": nip,
        "inspection_points_count": nip,
        "dashboard_inspection_points": nip,
        "num_groups": stats_raw.get("num_groups", 0),
        "num_edges": stats_raw.get("num_edges", len(visit_order)),
    }


def _point_flow_log_dashboard(dashboard: Dict[str, Any], mission_data: Optional[Dict[str, Any]] = None) -> None:
    """控制台：CV 与 Dashboard 巡检点数量对齐检查。"""
    meta = dashboard.get("metadata") or {}
    if mission_data and isinstance(mission_data.get("metadata"), dict):
        meta = {**meta, **mission_data["metadata"]}
    idst = meta.get("image_detection_stats") or {}
    cv_am = idst.get("after_merge", idst.get("merged_points", idst.get("green_point_count")))
    n_ip = len(dashboard.get("inspection_points") or [])
    print(
        f"[point-flow] cv_after_merge={cv_am} dashboard_inspection_points={n_ip} "
        f"(detector={idst.get('detector', '—')})"
    )


def build_dashboard_from_mission_json(
    data: Dict[str, Any],
    *,
    pipeline: str,
    source: str,
    planner: str = "legacy",
    input_file: str = "",
    spacing: float = 50.0,
    map_background: Optional[Dict[str, Any]] = None,
    coordinate_mode: str = "auto_fit",
    output_files: Optional[Dict[str, str]] = None,
    extra_metadata: Optional[Dict[str, Any]] = None,
    root: Optional[Path] = None,
) -> Dict[str, Any]:
    """从已导出的 mission_output.json 构建 Dashboard 结构。"""
    segments = _segments_from_json(data)
    image_catalog = _scan_inspection_image_catalog(root) if root else []
    inspection_points = _inspection_points_from_json(
        data,
        segments,
        root=root,
        image_catalog=image_catalog,
    )
    statistics = _statistics_from_json(data, inspection_points)

    visit = data.get("visit_order", {})
    visit_order = visit.get("edge_visit_order", []) if isinstance(visit, dict) else (visit or [])

    metadata: Dict[str, Any] = {
        "pipeline": pipeline,
        "source": source,
        "planner": planner,
        "input_file": input_file,
        "spacing": spacing,
        "coordinate_mode": coordinate_mode,
    }
    if extra_metadata:
        metadata.update(extra_metadata)
    if pipeline == "image":
        bg_src = (
            metadata.get("clean_map_image")
            or metadata.get("display_map_image")
            or metadata.get("map_image")
            or input_file
            or "data/test.png"
        )
        metadata["background"] = str(bg_src).replace("\\", "/")
    merge_mission_metadata_into_dashboard(metadata, data)

    payload: Dict[str, Any] = {
        "segments": segments,
        "inspection_points": inspection_points,
        "statistics": statistics,
        "visit_order": visit_order,
        "group_visit_order": visit.get("group_visit_order", []) if isinstance(visit, dict) else [],
        "markers": _start_end_from_segments(segments),
        "metadata": metadata,
        "output_files": output_files or {},
        "map_background": map_background,
        "map_mode": "image_overlay" if pipeline == "image" else "topology_only",
        "image_inspection_overlay": (
            data.get("image_inspection_overlay")
            or metadata.get("image_inspection_overlay")
            or []
        ),
    }

    pixel_fixed = bool(
        metadata.get("pixel_coordinate_mode")
        or metadata.get("coordinate_mode") == "image_pixel_fixed"
        or str(metadata.get("coordinate_mode") or "").startswith("image_pixel")
    )
    if pixel_fixed:
        coordinate_mode = "image_fixed"
        metadata["coordinate_mode"] = "image_pixel_fixed"
        iw = metadata.get("image_width")
        ih = metadata.get("image_height")
        if (not iw or not ih) and map_background:
            iw = map_background.get("width")
            ih = map_background.get("height")
        if iw and ih:
            payload["bounds"] = {
                "x_range": [0, float(iw)],
                "y_range": [float(ih), 0],
                "width": float(iw),
                "height": float(ih),
            }
        elif map_background and map_background.get("axis"):
            payload["bounds"] = {
                "x_range": map_background["axis"]["x_range"],
                "y_range": map_background["axis"]["y_range"],
                "width": map_background.get("width"),
                "height": map_background.get("height"),
            }
    elif coordinate_mode == "auto_fit":
        payload["bounds"] = compute_geometry_bounds(segments, inspection_points)
    elif map_background and map_background.get("axis"):
        payload["bounds"] = {
            "x_range": map_background["axis"]["x_range"],
            "y_range": map_background["axis"]["y_range"],
            "width": map_background.get("width"),
            "height": map_background.get("height"),
        }

    _img_stats = _inspection_image_stats_dict(inspection_points, root)
    metadata["inspection_image_stats"] = _img_stats
    _log_inspection_image_stats_console(_img_stats)

    _point_flow_log_dashboard(payload, data)
    _log_dashboard_inspection_point_rows(inspection_points)
    return payload


def build_image_pipeline_dashboard(
    mission_json_path: Union[str, Path],
    root: Path,
    *,
    map_rel: str = "data/test.png",
    image_url: str = "/api/map/background",
) -> Dict[str, Any]:
    """
    图像主线：仅读取 result/latest/mission_output.json + data/test.png 坐标系。
    """
    from visualization.dashboard_map import get_background_map_config

    mission_json_path = Path(mission_json_path)
    if not mission_json_path.exists():
        raise FileNotFoundError(
            "Image Pipeline requires result/latest/mission_output.json. "
            "Please run python demo/demo_visualization_main.py first."
        )

    with mission_json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    meta = data.get("metadata") or {}
    map_rel = str(
        meta.get("display_map_image")
        or meta.get("clean_map_image")
        or meta.get("map_image")
        or map_rel
    )
    map_bg = get_background_map_config(root, map_rel)
    rel_source = mission_json_path.relative_to(root).as_posix() if mission_json_path.is_relative_to(root) else str(mission_json_path)

    dashboard = build_dashboard_from_mission_json(
        data,
        pipeline="image",
        source=rel_source,
        planner="legacy",
        input_file=map_rel,
        spacing=0,
        map_background=map_bg,
        coordinate_mode="image_fixed",
        extra_metadata={
            "background": map_rel,
            "map_image": meta.get("map_image") or map_rel,
            "clean_map_image": meta.get("clean_map_image"),
            "inspection_point_source": normalize_inspection_point_source(
                meta.get("inspection_point_source")
            ),
            "image_detection_stats": meta.get("image_detection_stats") or {},
            "image_inspection_overlay": data.get("image_inspection_overlay") or meta.get("image_inspection_overlay") or [],
        },
        root=root,
    )
    dashboard["image_inspection_overlay"] = dashboard.get("metadata", {}).get("image_inspection_overlay") or []
    return dashboard


def build_dashboard_payload(
    mission_result: Dict[str, Any],
    *,
    planner: str,
    input_file: str,
    spacing: float,
    connect_planner: Optional[str] = None,
    connect_planner_note: Optional[str] = None,
    output_files: Optional[Dict[str, str]] = None,
    extra_metadata: Optional[Dict[str, Any]] = None,
    map_mode: str = "topology_only",
    root: Optional[Path] = None,
) -> Dict[str, Any]:
    """Unified 管线：从实时规划结果构建 Dashboard（不绑定 test.png）。"""
    mission = mission_result["mission"]
    from planner.mission_analysis import analyze_mission

    stats = analyze_mission(mission)

    segments = [
        _segment_to_dict(seg, i)
        for i, seg in enumerate(getattr(mission, "segments", []) or [])
    ]
    inspection_points = enrich_inspection_points_for_dashboard(
        _collect_inspection_points(mission_result), segments, root=root
    )
    markers = _start_end_from_segments(segments)

    nip = len(inspection_points)
    statistics = {
        "total_length": stats["total_length"],
        "inspect_length": stats["inspect_length"],
        "connect_length": stats["connect_length"],
        "connect_ratio": stats["connect_ratio"],
        "inspect_ratio": stats["inspect_ratio"],
        "num_segments": stats["num_segments"],
        "num_inspection_points": nip,
        "num_groups": stats["num_groups"],
        "num_edges": stats["num_edges"],
        "num_inspect_segments": stats["num_inspect_segments"],
        "num_connect_segments": stats["num_connect_segments"],
    }
    _count_keys = ("num_inspection_points", "inspection_points_count", "dashboard_inspection_points")
    for _src in (
        mission_result.get("statistics") if isinstance(mission_result.get("statistics"), dict) else None,
        getattr(mission, "statistics", None),
    ):
        if not isinstance(_src, dict):
            continue
        for _k, _v in _src.items():
            if _k in _count_keys:
                continue
            statistics.setdefault(_k, _v)
    statistics["num_inspection_points"] = nip
    statistics["inspection_points_count"] = nip
    statistics["dashboard_inspection_points"] = nip

    metadata: Dict[str, Any] = {
        "pipeline": "unified",
        "source": input_file,
        "planner": planner,
        "input_file": input_file,
        "spacing": spacing,
        "connect_planner": connect_planner or ("bfs" if planner == "baseline" else planner),
        "coordinate_mode": "auto_fit",
    }
    if connect_planner_note:
        metadata["connect_planner_note"] = connect_planner_note
    if planner == "dijkstra":
        metadata.setdefault(
            "connect_planner_note",
            "连接段使用 Dijkstra；不可达时已按段 fallback 至 BFS。",
        )
    if extra_metadata:
        metadata.update(extra_metadata)

    _img_stats = _inspection_image_stats_dict(inspection_points, root)
    metadata["inspection_image_stats"] = _img_stats
    _log_inspection_image_stats_console(_img_stats)

    effective_map_mode = "topology_only" if map_mode != "image_overlay" else "topology_only"
    # Unified 永不使用 test.png 底图（避免 T 型 toy 叠加到真实地图）

    payload: Dict[str, Any] = {
        "segments": segments,
        "inspection_points": inspection_points,
        "statistics": statistics,
        "visit_order": stats.get("visit_order", []),
        "group_visit_order": stats.get("group_visit_order", []),
        "markers": markers,
        "metadata": metadata,
        "output_files": output_files or {},
        "map_background": None,
        "map_mode": effective_map_mode,
        "bounds": compute_geometry_bounds(segments, inspection_points),
    }
    payload["inspection_points"] = inspection_points
    _point_flow_log_dashboard(payload, mission_result if isinstance(mission_result, dict) else None)
    _log_dashboard_inspection_point_rows(inspection_points)
    return payload


def apply_custom_markers(
    payload: Dict[str, Any],
    start: Optional[Tuple[float, float]] = None,
    end: Optional[Tuple[float, float]] = None,
) -> Dict[str, Any]:
    """覆盖 Dashboard 起终点标记（用于重规划 UI）。"""
    markers = dict(payload.get("markers") or {})
    if start is not None:
        markers["start"] = {"x": float(start[0]), "y": float(start[1])}
    if end is not None:
        markers["end"] = {"x": float(end[0]), "y": float(end[1])}
    payload["markers"] = markers
    return payload


def save_json(path: Union[str, Path], payload: Dict[str, Any]) -> str:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    return str(path.resolve())
