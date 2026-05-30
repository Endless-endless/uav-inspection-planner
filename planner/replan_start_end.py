"""
Start–End driven mission replan for Image Pipeline Dashboard.

Given user start/end and a baseline mission JSON, rebuilds segments:
  start → connect → inspect all edges (full polylines) → connect_to_end → end
"""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from weather.weather_cost import compute_edge_weather_penalty

from core.topo import TopoGraph
from core.topo_task import EdgeTask
from planner.mission_result_builder import (
    is_image_inspection_point,
    is_image_inspection_source,
    normalize_inspection_point_source,
)
from planner.topo_dijkstra import generate_connection_segment_with_planner

IMAGE_WIDTH = 916
IMAGE_HEIGHT = 960

# Degenerate connect warning (two-point geometry = likely straight graph fallback)
STRAIGHT_FALLBACK_WARN_PX = 150.0

Point = Tuple[float, float]


def _merge_mission_metadata_nested(base_mission: Dict[str, Any]) -> None:
    """
    Dashboard 将完整 mission metadata 放在 metadata.mission_metadata。
    重规划需要 topo_edges_pixel、image_width 等与 mission_output.json 顶层一致。
    """
    meta = base_mission.get("metadata")
    if not isinstance(meta, dict):
        return
    nested = meta.get("mission_metadata")
    if not isinstance(nested, dict):
        return
    for k, v in nested.items():
        cur = meta.get(k)
        if cur in (None, "", [], {}) and v not in (None, "", [], {}):
            meta[k] = copy.deepcopy(v) if isinstance(v, (dict, list)) else v


def replan_image_bounds_from_baseline(base_mission: Dict[str, Any]) -> Tuple[int, int]:
    """像素坐标系宽高：metadata / mission_metadata → bounds → map_background → 默认 916×960。"""
    meta = base_mission.get("metadata") or {}
    nested = meta.get("mission_metadata") if isinstance(meta.get("mission_metadata"), dict) else {}
    bounds = base_mission.get("bounds") or {}
    mb = base_mission.get("map_background") or {}
    w = meta.get("image_width") or nested.get("image_width") or bounds.get("width") or mb.get("width")
    h = meta.get("image_height") or nested.get("image_height") or bounds.get("height") or mb.get("height")
    if w and h:
        return int(float(w)), int(float(h))
    return IMAGE_WIDTH, IMAGE_HEIGHT


def baseline_has_inspect_segments(base_mission: Optional[Dict[str, Any]]) -> bool:
    if not base_mission:
        return False
    for seg in base_mission.get("segments") or []:
        if seg.get("type") == "inspect" and seg.get("edge_id"):
            geom = seg.get("geometry_2d") or []
            if len(geom) >= 2:
                return True
    return False


class ReplanValidationError(ValueError):
    """Invalid start/end coordinates or mission data."""


def validate_image_coords(
    start_xy: List[float],
    end_xy: List[float],
    *,
    width: int = IMAGE_WIDTH,
    height: int = IMAGE_HEIGHT,
) -> Tuple[Point, Point]:
    """Validate start/end lie within image bounds."""
    try:
        sx, sy = float(start_xy[0]), float(start_xy[1])
        ex, ey = float(end_xy[0]), float(end_xy[1])
    except (TypeError, IndexError, ValueError) as e:
        raise ReplanValidationError("start/end must be [x, y] numbers") from e

    for name, x, y in (("Start", sx, sy), ("End", ex, ey)):
        if x < 0 or x > width or y < 0 or y > height:
            raise ReplanValidationError(
                f"{name}/End coordinate out of image bounds: "
                f"x must be [0,{width}], y must be [0,{height}]"
            )
    return (sx, sy), (ex, ey)


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_length(polyline: List[Point]) -> float:
    if len(polyline) < 2:
        return 0.0
    return sum(_dist(polyline[i], polyline[i + 1]) for i in range(len(polyline) - 1))


def _point_at_distance(polyline: List[Point], target: float) -> Point:
    if not polyline:
        return (0.0, 0.0)
    if len(polyline) == 1:
        return (float(polyline[0][0]), float(polyline[0][1]))
    remain = max(0.0, target)
    for i in range(len(polyline) - 1):
        p0 = polyline[i]
        p1 = polyline[i + 1]
        seg_len = _dist(p0, p1)
        if seg_len < 1e-9:
            continue
        if remain <= seg_len:
            t = remain / seg_len
            return (
                p0[0] + t * (p1[0] - p0[0]),
                p0[1] + t * (p1[1] - p0[1]),
            )
        remain -= seg_len
    return (float(polyline[-1][0]), float(polyline[-1][1]))


def _project_to_polyline_distance(point: Point, polyline: List[Point]) -> Optional[float]:
    if len(polyline) < 2:
        return None
    px, py = point
    best_d = float("inf")
    best_s = 0.0
    cum = 0.0
    for i in range(len(polyline) - 1):
        x1, y1 = polyline[i]
        x2, y2 = polyline[i + 1]
        vx, vy = (x2 - x1), (y2 - y1)
        seg_len = math.hypot(vx, vy)
        if seg_len < 1e-9:
            continue
        wx, wy = (px - x1), (py - y1)
        t = (wx * vx + wy * vy) / (seg_len * seg_len)
        t = max(0.0, min(1.0, t))
        qx, qy = (x1 + t * vx, y1 + t * vy)
        d = math.hypot(px - qx, py - qy)
        if d < best_d:
            best_d = d
            best_s = cum + t * seg_len
        cum += seg_len
    return best_s


def _slice_polyline(polyline: List[Point], s0: float, s1: float) -> List[Point]:
    """
    沿折线弧长 [s0,s1] 的子路径，保留中间顶点（避免仅 [起点,终点] 的直线弦）。
    """
    total = _path_length(polyline)
    if total < 1e-9:
        return [(float(polyline[0][0]), float(polyline[0][1]))] if polyline else []
    if len(polyline) < 2:
        return [(float(polyline[0][0]), float(polyline[0][1]))]

    a0 = max(0.0, min(total, min(s0, s1)))
    a1 = max(0.0, min(total, max(s0, s1)))
    if a1 - a0 < 1e-4:
        mid = 0.5 * (a0 + a1)
        delta = min(8.0, max(3.0, total * 0.02))
        a0 = max(0.0, mid - delta)
        a1 = min(total, mid + delta)

    out: List[Point] = []
    cum = 0.0
    for i in range(len(polyline) - 1):
        p_a, p_b = polyline[i], polyline[i + 1]
        seg_len = _dist(p_a, p_b)
        if seg_len < 1e-9:
            continue
        seg_lo, seg_hi = cum, cum + seg_len
        lo = max(a0, seg_lo)
        hi = min(a1, seg_hi)
        if hi < lo - 1e-9:
            cum += seg_len
            continue
        t_lo = max(0.0, min(1.0, (lo - seg_lo) / seg_len))
        t_hi = max(0.0, min(1.0, (hi - seg_lo) / seg_len))
        q_lo = (
            p_a[0] + t_lo * (p_b[0] - p_a[0]),
            p_a[1] + t_lo * (p_b[1] - p_a[1]),
        )
        q_hi = (
            p_a[0] + t_hi * (p_b[0] - p_a[0]),
            p_a[1] + t_hi * (p_b[1] - p_a[1]),
        )
        if not out:
            out.append((float(q_lo[0]), float(q_lo[1])))
        elif _dist(out[-1], q_lo) > 1e-4:
            out.append((float(q_lo[0]), float(q_lo[1])))
        if _dist(q_lo, q_hi) > 1e-4:
            out.append((float(q_hi[0]), float(q_hi[1])))
        cum += seg_len

    if len(out) < 2:
        p0 = _point_at_distance(polyline, a0)
        p1 = _point_at_distance(polyline, a1)
        if _dist(p0, p1) < 1e-6:
            return [p0, (p0[0] + 0.5, p0[1] + 0.5)]
        return [p0, p1]
    return out


def _interpolate_line(p0: Point, p1: Point, step: float = 5.0) -> List[Point]:
    """Dense Euclidean polyline (legacy fallback when TopoGraph is unavailable)."""
    total = _dist(p0, p1)
    if total < 1e-6:
        return [p0, p1]
    n = max(2, int(math.ceil(total / step)) + 1)
    out: List[Point] = []
    for i in range(n):
        t = i / (n - 1)
        out.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
    out[-1] = (float(p1[0]), float(p1[1]))
    return out


def _warn_straight_fallback(geometry: List[Point], role: str) -> None:
    if len(geometry) == 2:
        d = _dist(geometry[0], geometry[1])
        if d > STRAIGHT_FALLBACK_WARN_PX:
            print(
                f"[WARN] straight fallback (role={role}, points=2, dist={d:.1f}px)"
            )


def _weather_cost_config_for_dijkstra(
    weather_context: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if not weather_context or not weather_context.get("enabled"):
        return None
    return {
        "length_field": "len2d",
        "weather": {
            "enabled": True,
            "weather_zones": weather_context.get("weather_zones") or [],
            "type_weights": weather_context.get("type_weights"),
            "weather_weight": float(weather_context.get("weather_weight", 1.0)),
        },
    }


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _abs_map_path(project_root: Path, rel: str) -> str:
    p = Path(str(rel).replace("\\", "/"))
    if p.is_absolute():
        return str(p.resolve())
    return str((project_root / p).resolve())


def _try_load_topo_for_replan(
    project_root: Path,
    image_rel: str,
    inspection_point_source: str,
    inspection_spacing: float,
) -> Tuple[Optional[TopoGraph], Dict[str, EdgeTask]]:
    """
    Rebuild TopoGraph + EdgeTask map from the same PNG pipeline as the image dashboard.
    Used so connect segments follow graph geometry (Dijkstra) instead of Euclidean chords.
    """
    try:
        from planner.powerline_planner_v3_final import PowerlinePlannerV3
    except Exception as exc:  # pragma: no cover - import guard
        print(f"[WARN] replan topo: cannot import PowerlinePlannerV3: {exc}")
        return None, {}

    abs_img = _abs_map_path(project_root, image_rel)
    try:
        planner = PowerlinePlannerV3(image_path=abs_img, flight_height=30, weather_scene="calm")
        planner.inspection_point_source = inspection_point_source
        planner.step1_extract_redline_hsv()
        planner.step2_fix_breaks()
        planner.step3_skeletonize()
        planner.step4_extract_independent_lines()
        if inspection_point_source == "image":
            planner.step5_detect_image_inspection_points()
        else:
            planner.step5_generate_line_inspection_points(spacing=inspection_spacing)

        terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
        if inspection_point_source == "image":
            planner.step6_smooth_terrain(terrain)
        else:
            planner.step6_map_line_points_to_3d(terrain)

        planner.step7_5_build_topo()
        if inspection_point_source == "image":
            planner.step5_finalize_image_inspection_points()
            planner._map_existing_points_to_3d()

        edge_tasks = planner.step8_5_build_edge_tasks() or []
        edge_map = {et.edge_id: et for et in edge_tasks}
        tg = planner.topo_graph
        if tg is None or not edge_map:
            return None, {}
        return tg, edge_map
    except Exception as exc:
        print(f"[WARN] replan topo rebuild failed ({image_rel}): {exc}")
        return None, {}


def _connect_geometry_topo(
    point_a: Point,
    point_b: Point,
    *,
    role: str,
    from_edge_id: Optional[str],
    to_edge_id: Optional[str],
    topo_graph: Optional[TopoGraph],
    edge_task_map: Dict[str, EdgeTask],
    cost_config: Optional[Dict[str, Any]],
) -> List[Point]:
    """
    Connect along TopoGraph via Dijkstra (planner/topo_dijkstra.py).
    Falls back to dense Euclidean interpolation only if topo is missing or planner errors.
    同一 edge 上两点：沿该边 pixel_polyline 截取子折线，不拉直线、不走跨边 Dijkstra。
    """
    if (
        from_edge_id
        and to_edge_id
        and from_edge_id == to_edge_id
        and edge_task_map
        and from_edge_id in edge_task_map
    ):
        from core.image_pixel_coords import edge_pixel_polyline

        poly = edge_pixel_polyline(edge_task_map[from_edge_id])
        if len(poly) >= 2:
            sa = _project_to_polyline_distance(point_a, poly)
            sb = _project_to_polyline_distance(point_b, poly)
            if sa is not None and sb is not None:
                seg = _slice_polyline(poly, sa, sb)
                if len(seg) >= 2:
                    return seg

    if topo_graph is not None and edge_task_map:
        try:
            geom, _ = generate_connection_segment_with_planner(
                point_a,
                point_b,
                topo_graph,
                edge_task_map,
                connect_planner="dijkstra",
                cost_config=cost_config,
                use_proximity_bfs=False,
                from_edge_id=from_edge_id,
                to_edge_id=to_edge_id,
            )
            out = [(float(p[0]), float(p[1])) for p in geom]
            _warn_straight_fallback(out, role)
            return out
        except Exception as exc:
            print(f"[WARN] topo Dijkstra connect failed role={role}: {exc}")

    out = _interpolate_line(point_a, point_b)
    if len(out) == 2 and _dist(out[0], out[1]) > STRAIGHT_FALLBACK_WARN_PX:
        d = _dist(out[0], out[1])
        print(
            f"[WARN] straight fallback (role={role}, points=2, dist={d:.1f}px, "
            "euclidean_fallback)"
        )
    return out


def _weather_penalty_for_line(
    p0: Point,
    p1: Point,
    weather_context: Optional[Dict[str, Any]],
) -> float:
    if not weather_context or not weather_context.get("enabled"):
        return 0.0
    zones = weather_context.get("weather_zones") or []
    if not zones:
        return 0.0
    info = compute_edge_weather_penalty(
        [p0, p1],
        zones,
        type_weights=weather_context.get("type_weights"),
        weather_weight=float(weather_context.get("weather_weight", 1.0)),
    )
    return float(info.get("total_penalty", 0.0))


def _sample_polyline(
    polyline: List[Point],
    spacing: float,
) -> List[Point]:
    """Uniform sampling along polyline; always keeps first and last."""
    if len(polyline) < 2:
        return [tuple(p) for p in polyline]
    cum = [0.0]
    for i in range(1, len(polyline)):
        cum.append(cum[-1] + _dist(polyline[i - 1], polyline[i]))
    total = cum[-1]
    if total < 1e-6:
        return [polyline[0]]

    targets = [0.0]
    d = float(spacing)
    while d < total - 1e-6:
        targets.append(d)
        d += spacing
    if abs(targets[-1] - total) > 1e-6:
        targets.append(total)

    sampled: List[Point] = []
    j = 0
    for t in targets:
        while j < len(cum) - 2 and cum[j + 1] < t - 1e-9:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        if seg_len < 1e-9:
            alpha = 0.0
        else:
            alpha = (t - cum[j]) / seg_len
        px = polyline[j][0] + alpha * (polyline[j + 1][0] - polyline[j][0])
        py = polyline[j][1] + alpha * (polyline[j + 1][1] - polyline[j][1])
        pt = (round(px, 2), round(py, 2))
        if not sampled or _dist(sampled[-1], pt) > 1e-3:
            sampled.append(pt)
    if _dist(sampled[-1], polyline[-1]) > 1e-3:
        sampled.append((float(polyline[-1][0]), float(polyline[-1][1])))
    return sampled


def _topo_pixel_polyline_for_edge(base_mission: Dict[str, Any], eid: str) -> Optional[List[Point]]:
    """metadata / mission_metadata 中 topo_edges_pixel 的折线（与电网红线一致）。"""
    meta = base_mission.get("metadata") or {}
    nested = meta.get("mission_metadata") if isinstance(meta.get("mission_metadata"), dict) else {}
    edges = meta.get("topo_edges_pixel") or nested.get("topo_edges_pixel") or base_mission.get("topo_edges_pixel")
    if not isinstance(edges, list):
        return None
    for row in edges:
        if str(row.get("edge_id")) != str(eid):
            continue
        raw = row.get("pixel_polyline") or row.get("image_polyline") or row.get("original_polyline")
        if raw and len(raw) >= 2:
            return [(float(p[0]), float(p[1])) for p in raw if p is not None and len(p) >= 2]
    return None


def _baseline_inspect_polyline_for_edge(base_mission: Dict[str, Any], eid: str) -> Optional[List[Point]]:
    """baseline 中该 edge 最长的一条 inspect geometry。"""
    best: Optional[List[Point]] = None
    best_len = -1.0
    for seg in base_mission.get("segments", []) or []:
        if seg.get("type") != "inspect" or str(seg.get("edge_id")) != str(eid):
            continue
        geom = seg.get("geometry_2d") or []
        poly = [(float(p[0]), float(p[1])) for p in geom if len(p) >= 2]
        ln = _path_length(poly)
        if len(poly) >= 2 and ln > best_len:
            best_len = ln
            best = poly
    return best


def _reference_polyline_for_edge(
    base_mission: Dict[str, Any],
    eid: str,
    edge_task_map: Optional[Dict[str, EdgeTask]],
) -> Optional[List[Point]]:
    """EdgeTask 像素折线 → topo_edges_pixel → baseline inspect。"""
    if edge_task_map and eid in edge_task_map:
        from core.image_pixel_coords import edge_pixel_polyline

        pl = edge_pixel_polyline(edge_task_map[eid])
        if len(pl) >= 2:
            return pl
    pl_topo = _topo_pixel_polyline_for_edge(base_mission, eid)
    if pl_topo and len(pl_topo) >= 2:
        return pl_topo
    return _baseline_inspect_polyline_for_edge(base_mission, eid)


def _candidate_edge_ids_ordered(base_mission: Dict[str, Any]) -> List[str]:
    """visit_order JSON + inspect 段顺序，去重。"""
    visit = base_mission.get("visit_order", {})
    if isinstance(visit, dict):
        raw_order = list(visit.get("edge_visit_order") or [])
    elif isinstance(visit, list):
        raw_order = list(visit)
    else:
        raw_order = []
    seen = set()
    out: List[str] = []
    for tok in raw_order:
        s = str(tok or "").strip()
        if len(s) >= 2 and s[-1] in "+-":
            s = s[:-1]
        if not s or s in seen:
            continue
        seen.add(s)
        out.append(s)
    for seg in base_mission.get("segments", []) or []:
        if seg.get("type") != "inspect" or not seg.get("edge_id"):
            continue
        eid = str(seg["edge_id"])
        if eid not in seen:
            seen.add(eid)
            out.append(eid)
    return out


def _extract_inspect_tasks(
    base_mission: Dict[str, Any],
    *,
    edge_task_map: Optional[Dict[str, EdgeTask]] = None,
) -> Dict[str, Dict[str, Any]]:
    """
    每条 edge 一条沿像素折线的巡检几何：弧长上仅覆盖该边所有巡检点之间的区间，
    避免 inspect 段退化为 [首,末] 两点直线。
    """
    edge_points: Dict[str, List[Point]] = {}
    for pt in base_mission.get("inspection_points", []) or []:
        eid = pt.get("edge_id")
        pos = pt.get("pixel_position") or [pt.get("x"), pt.get("y")]
        if not eid or not pos or len(pos) < 2:
            continue
        edge_points.setdefault(str(eid), []).append((float(pos[0]), float(pos[1])))

    tasks: Dict[str, Dict[str, Any]] = {}

    if not edge_points:
        for seg in base_mission.get("segments", []) or []:
            if seg.get("type") != "inspect":
                continue
            eid = str(seg.get("edge_id") or "")
            if not eid:
                continue
            geom = seg.get("geometry_2d") or []
            polyline = [(float(p[0]), float(p[1])) for p in geom if len(p) >= 2]
            if len(polyline) < 2:
                continue
            ref = _reference_polyline_for_edge(base_mission, eid, edge_task_map) or polyline
            tasks[eid] = {
                "edge_id": eid,
                "polyline": ref,
                "length": round(_path_length(ref), 2),
            }
        return tasks

    for eid in _candidate_edge_ids_ordered(base_mission):
        if eid not in edge_points:
            continue
        ref = _reference_polyline_for_edge(base_mission, eid, edge_task_map)
        if not ref or len(ref) < 2:
            print(f"[WARN] replan: no reference polyline for edge={eid}, skip")
            continue
        pts = edge_points[eid]
        projected: List[float] = []
        for pt in pts:
            s = _project_to_polyline_distance(pt, ref)
            if s is not None:
                projected.append(s)
        if not projected:
            print(f"[WARN] replan: cannot project points onto edge={eid}, skip")
            continue
        s_min, s_max = min(projected), max(projected)
        trimmed = _slice_polyline(ref, s_min, s_max)
        if len(trimmed) < 2:
            total = _path_length(ref)
            trimmed = _slice_polyline(ref, max(0.0, s_min - 3.0), min(total, s_max + 3.0))
        if len(trimmed) < 2:
            continue
        tasks[eid] = {
            "edge_id": eid,
            "polyline": trimmed,
            "length": round(_path_length(trimmed), 2),
        }
        print(
            f"[replan-inspect-task] edge={eid} n_inspection_pts={len(pts)} "
            f"s_range=({min(projected):.2f},{max(projected):.2f}) "
            f"ref_len={_path_length(ref):.1f} out_pts={len(trimmed)}"
        )
    return tasks


def _strip_visit_direction_suffix(edge_token: str) -> str:
    """
    mission JSON 中 visit_order.edge_visit_order 常为 'L_001_edge_0+' / 'L_001_edge_0-'，
    与 segment.edge_id（无后缀）不一致；重规划须归一化后再与 tasks 对齐。
    """
    s = str(edge_token or "").strip()
    if len(s) >= 2 and s[-1] in "+-":
        return s[:-1]
    return s


def _edge_visit_order(base_mission: Dict[str, Any], tasks: Dict[str, Dict[str, Any]]) -> List[str]:
    visit = base_mission.get("visit_order", {})
    if isinstance(visit, dict):
        order = list(visit.get("edge_visit_order") or [])
    elif isinstance(visit, list):
        order = list(visit)
    else:
        order = []

    seen = set()
    ordered: List[str] = []
    for raw in order:
        eid = _strip_visit_direction_suffix(raw)
        if eid in tasks and eid not in seen:
            seen.add(eid)
            ordered.append(eid)
    for eid in tasks:
        if eid not in seen:
            seen.add(eid)
            ordered.append(eid)
    if not ordered:
        for seg in base_mission.get("segments", []):
            if seg.get("type") == "inspect" and seg.get("edge_id"):
                eid = str(seg["edge_id"])
                if eid in tasks and eid not in seen:
                    seen.add(eid)
                    ordered.append(eid)
    return ordered


def _rotate_order_nearest_start(
    ordered: List[str],
    tasks: Dict[str, Dict[str, Any]],
    start_xy: Point,
    weather_context: Optional[Dict[str, Any]] = None,
) -> Tuple[List[str], bool, Point]:
    """Pick first edge + direction minimizing distance from start to entry."""
    best_edge: Optional[str] = None
    best_forward = True
    best_dist = float("inf")
    best_entry: Point = start_xy

    for eid in ordered:
        poly = tasks[eid]["polyline"]
        for forward in (True, False):
            entry = poly[0] if forward else poly[-1]
            d = _dist(start_xy, entry) + _weather_penalty_for_line(start_xy, entry, weather_context)
            if d < best_dist:
                best_dist = d
                best_edge = eid
                best_forward = forward
                best_entry = entry

    if best_edge is None:
        return ordered, True, start_xy

    idx = ordered.index(best_edge)
    rotated = ordered[idx:] + ordered[:idx]
    return rotated, best_forward, best_entry


def _collect_baseline_inspection_points(
    base_mission: Dict[str, Any],
    visit_order: List[str],
) -> List[Dict[str, Any]]:
    """Reuse image-defined inspection points during replan."""
    points: List[Dict[str, Any]] = []
    edge_rank = {eid: idx for idx, eid in enumerate(visit_order)}
    for pt in base_mission.get("inspection_points", []) or []:
        pos = pt.get("pixel_position") or [pt.get("x"), pt.get("y")]
        if not pos or len(pos) < 2:
            continue
        copied = copy.deepcopy(pt)
        copied["x"] = float(pos[0])
        copied["y"] = float(pos[1])
        copied["pixel_position"] = [float(pos[0]), float(pos[1])]
        points.append(copied)
    points.sort(
        key=lambda p: (
            edge_rank.get(str(p.get("edge_id")), 10**6),
            float(p.get("visit_order") or 0),
        )
    )
    return points


def _filter_image_inspection_points(points: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [pt for pt in points or [] if is_image_inspection_point(pt)]


def _points_from_overlay(
    overlay: List[Dict[str, Any]],
    *,
    visit_order: List[str],
) -> List[Dict[str, Any]]:
    edge_rank = {eid: idx for idx, eid in enumerate(visit_order)}
    points: List[Dict[str, Any]] = []
    for item in overlay or []:
        if not item.get("valid"):
            continue
        snapped = item.get("snapped_coord")
        if not snapped or len(snapped) < 2:
            continue
        point_id = str(item.get("id") or f"IP_{len(points) + 1:04d}")
        points.append(
            {
                "point_id": point_id,
                "id": point_id,
                "edge_id": item.get("edge_id"),
                "x": float(snapped[0]),
                "y": float(snapped[1]),
                "pixel_position": [float(snapped[0]), float(snapped[1])],
                "point_type": "image_detected",
                "source_reason": "image_black_dot",
                "detection_result": {
                    "raw_coord": item.get("raw_coord"),
                    "snapped_coord": snapped,
                    "snap_distance": item.get("snap_distance"),
                    "confidence": item.get("confidence"),
                    "source": item.get("source"),
                },
            }
        )
    points.sort(
        key=lambda p: (
            edge_rank.get(str(p.get("edge_id")), 10**6),
            float((p.get("detection_result") or {}).get("distance_along_edge") or 0.0),
        )
    )
    return points


def _normalize_cached_inspection_points(
    points: List[Dict[str, Any]],
    *,
    visit_order: List[str],
) -> List[Dict[str, Any]]:
    edge_rank = {eid: idx for idx, eid in enumerate(visit_order)}
    normalized: List[Dict[str, Any]] = []
    for pt in points or []:
        pos = pt.get("pixel_position") or pt.get("snapped_coord")
        if (not pos or len(pos) < 2) and pt.get("x") is not None and pt.get("y") is not None:
            pos = [pt["x"], pt["y"]]
        if not pos or len(pos) < 2:
            continue
        copied = copy.deepcopy(pt)
        copied["x"] = float(pos[0])
        copied["y"] = float(pos[1])
        copied["pixel_position"] = [float(pos[0]), float(pos[1])]
        if not copied.get("point_id"):
            copied["point_id"] = copied.get("id") or f"IP_{len(normalized) + 1:04d}"
        normalized.append(copied)
    normalized.sort(
        key=lambda p: (
            edge_rank.get(str(p.get("edge_id")), 10**6),
            float(p.get("visit_order") or 0),
        )
    )
    return normalized


def _resolve_replan_inspection_points(
    base_mission: Dict[str, Any],
    *,
    visit_order: List[str],
    tasks: Dict[str, Dict[str, Any]],
    directions: Dict[str, bool],
    spacing: float,
    mission_context: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], str]:
    ctx = mission_context or {}
    source = str(
        ctx.get("inspection_point_source")
        or (base_mission.get("metadata") or {}).get("inspection_point_source")
        or "spacing"
    )

    if not is_image_inspection_source(source):
        return _build_planning_inspection_points(tasks, visit_order, directions, spacing), "spacing"

    cached = _normalize_cached_inspection_points(
        ctx.get("inspection_points") or [],
        visit_order=visit_order,
    )
    if cached:
        print(f"[Replan] reuse cached dashboard inspection points: {len(cached)}")
        return cached, "image"

    baseline_points = _filter_image_inspection_points(
        _collect_baseline_inspection_points(base_mission, visit_order)
    )
    if baseline_points:
        print(f"[Replan] reuse baseline image inspection points: {len(baseline_points)}")
        return baseline_points, "image"

    overlay = (
        ctx.get("image_inspection_overlay")
        or base_mission.get("image_inspection_overlay")
        or (base_mission.get("metadata") or {}).get("image_inspection_overlay")
        or []
    )
    overlay_points = _points_from_overlay(overlay, visit_order=visit_order)
    if overlay_points:
        print(f"[Replan] rebuild image inspection points from overlay: {len(overlay_points)}")
        return overlay_points, "image"

    raise ReplanValidationError(
        "Image inspection replan requires existing image-detected points, "
        "but none were found in baseline mission or request context."
    )


def _build_planning_inspection_points(
    tasks: Dict[str, Dict[str, Any]],
    visit_order: List[str],
    directions: Dict[str, bool],
    spacing: float,
) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    pid = 0
    for eid in visit_order:
        poly = tasks[eid]["polyline"]
        forward = directions.get(eid, True)
        line = poly if forward else list(reversed(poly))
        sampled = _sample_polyline(line, spacing)
        for i, (x, y) in enumerate(sampled):
            pid += 1
            ptype = "endpoint" if i == 0 or i == len(sampled) - 1 else "sample"
            points.append({
                "point_id": f"point_{pid:04d}",
                "edge_id": eid,
                "x": x,
                "y": y,
                "point_type": ptype,
                "pixel_position": [x, y],
            })
    return points


def _make_connect_segment(
    seg_idx: int,
    geometry: List[Point],
    *,
    role: str,
    from_edge_id: Optional[str],
    to_edge_id: Optional[str],
    segment_id: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "segment_id": segment_id or f"seg_{seg_idx:04d}",
        "type": "connect",
        "role": role,
        "edge_id": None,
        "from_edge_id": from_edge_id,
        "to_edge_id": to_edge_id,
        "length": round(_path_length(geometry), 2),
        "direction": "forward",
        "geometry_2d": [[p[0], p[1]] for p in geometry],
    }


def _make_inspect_segment(
    seg_idx: int,
    geometry: List[Point],
    edge_id: str,
    *,
    from_edge_id: Optional[str],
    direction: str,
) -> Dict[str, Any]:
    return {
        "segment_id": f"seg_{seg_idx:04d}",
        "type": "inspect",
        "edge_id": edge_id,
        "from_edge_id": from_edge_id,
        "to_edge_id": edge_id,
        "length": round(_path_length(geometry), 2),
        "direction": direction,
        "geometry_2d": [[p[0], p[1]] for p in geometry],
    }


def _log_replan_path_debug(
    segments: List[Dict[str, Any]],
    visit_order_edges: List[str],
    *,
    tag: str = "start_end_replan",
) -> None:
    """控制台调试：段数量、visit_order、各 connect 起终点、重复 connect 与相邻 connect 检查。"""
    inspect_segs = [s for s in segments if s.get("type") == "inspect"]
    connect_segs = [s for s in segments if s.get("type") == "connect"]
    n_insp = len(inspect_segs)
    n_conn = len(connect_segs)
    print(f"\n[{tag}] path-debug ===")
    print(f"  inspect_segment_count={n_insp}")
    print(f"  connect_segment_count={n_conn}")
    print(f"  visit_order (len={len(visit_order_edges)}): {visit_order_edges}")

    for i, s in enumerate(connect_segs):
        geom = s.get("geometry_2d") or []
        role = s.get("role") or "?"
        fe = s.get("from_edge_id")
        te = s.get("to_edge_id")
        if len(geom) >= 1:
            p0 = geom[0]
            p1 = geom[-1] if len(geom) >= 2 else p0
        else:
            p0 = ("?", "?")
            p1 = ("?", "?")
        print(
            f"  connect[{i}] role={role} from_edge={fe!r} to_edge={te!r} "
            f"start=({float(p0[0]):.1f},{float(p0[1]):.1f}) "
            f"end=({float(p1[0]):.1f},{float(p1[1]):.1f}) npts={len(geom)}"
        )

    for i, s in enumerate(inspect_segs):
        geom = s.get("geometry_2d") or []
        eid = s.get("edge_id")
        if len(geom) >= 1:
            q0 = geom[0]
            q1 = geom[-1] if len(geom) >= 2 else q0
        else:
            q0 = ("?", "?")
            q1 = ("?", "?")
        print(
            f"  inspect[{i}] edge_id={eid!r} start=({float(q0[0]):.1f},{float(q0[1]):.1f}) "
            f"end=({float(q1[0]):.1f},{float(q1[1]):.1f}) npts={len(geom)}"
        )

    dup_keys: Dict[tuple, int] = {}
    for s in connect_segs:
        key = (s.get("from_edge_id"), s.get("to_edge_id"), s.get("role"))
        dup_keys[key] = dup_keys.get(key, 0) + 1
    dup_report = {str(k): v for k, v in dup_keys.items() if v > 1}
    if dup_report:
        print(f"  [WARN] duplicate connect (from_edge,to_edge,role) -> counts: {dup_report}")
    else:
        print("  duplicate_connect_keys: none")

    bad_adj: List[int] = []
    for i in range(len(segments) - 1):
        if segments[i].get("type") == "connect" and segments[i + 1].get("type") == "connect":
            bad_adj.append(i)
    if bad_adj:
        print(f"  [ERROR] consecutive connect segment pairs at indices {bad_adj}")
    else:
        print("  consecutive_connect_check: ok")

    types = [s.get("type") for s in segments]
    gap_doubles: List[int] = []
    for i in range(len(types) - 3):
        if (
            types[i] == "inspect"
            and types[i + 1] == "connect"
            and types[i + 2] == "connect"
            and types[i + 3] == "inspect"
        ):
            gap_doubles.append(i)
    if gap_doubles:
        print(
            f"  [ERROR] two connect segments between one inspect pair "
            f"(pattern at seg indices): {gap_doubles}"
        )
    else:
        print("  at_most_one_connect_between_inspects: ok")

    if n_insp > 0 and n_conn > n_insp + 1:
        print(
            f"  [WARN] connect_segment_count ({n_conn}) > inspect_count+1 ({n_insp + 1}); "
            "possible redundant gap connects"
        )

    insp_edges = [s.get("edge_id") for s in inspect_segs]
    dup_insp = [e for e in set(insp_edges) if insp_edges.count(e) > 1]
    if dup_insp:
        print(f"  [WARN] same edge_id appears in multiple inspect segments: {dup_insp}")
    else:
        print("  duplicate_inspect_edge: none")

    print(f"[{tag}] path-debug === end\n")


def _log_replan_point_projections(
    base_mission: Dict[str, Any],
    tasks: Dict[str, Dict[str, Any]],
    visit_order: List[str],
) -> None:
    """每个巡检点在其 inspect 折线上的投影弧长 projected_s；按边、按 s 排序。"""
    print("\n[replan-projection] ===")
    edge_rows: Dict[str, List[Tuple[float, str, Point]]] = {}
    for pt in base_mission.get("inspection_points", []) or []:
        eid = str(pt.get("edge_id") or "")
        if eid not in tasks:
            continue
        pid = str(pt.get("point_id") or pt.get("id") or "?")
        pos = pt.get("pixel_position") or [pt.get("x"), pt.get("y")]
        if not pos or len(pos) < 2:
            continue
        xy = (float(pos[0]), float(pos[1]))
        s = _project_to_polyline_distance(xy, tasks[eid]["polyline"])
        if s is None:
            continue
        edge_rows.setdefault(eid, []).append((s, pid, xy))
    for eid in visit_order:
        rows = sorted(edge_rows.get(eid, []), key=lambda r: r[0])
        print(f"  edge={eid} inspection_points_along_polyline (n={len(rows)}):")
        for s, pid, xy in rows:
            print(f"    point_id={pid} projected_s={s:.2f} xy=({xy[0]:.1f},{xy[1]:.1f})")
    print("[replan-projection] === end\n")


def build_start_end_replan_mission(
    base_mission: Dict[str, Any],
    start_xy: List[float],
    end_xy: List[float],
    planning_spacing: float = 70.0,
    weather_context: Optional[Dict[str, Any]] = None,
    mission_context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Build full mission JSON: start → inspect all edges (dense polylines) → end.
    """
    base_mission = copy.deepcopy(base_mission)
    _merge_mission_metadata_nested(base_mission)
    iw, ih = replan_image_bounds_from_baseline(base_mission)
    start, end = validate_image_coords(start_xy, end_xy, width=iw, height=ih)
    spacing = max(20.0, float(planning_spacing))

    ctx_pre = mission_context or {}
    base_meta_early = base_mission.get("metadata") or {}
    image_rel = str(
        ctx_pre.get("image_path")
        or base_meta_early.get("map_image")
        or "data/test.png"
    ).replace("\\", "/")
    src_for_topo = normalize_inspection_point_source(
        ctx_pre.get("inspection_point_source")
        or base_meta_early.get("inspection_point_source")
        or "spacing"
    )
    topo_graph, edge_task_map = _try_load_topo_for_replan(
        _project_root(),
        image_rel,
        src_for_topo,
        spacing,
    )

    tasks = _extract_inspect_tasks(base_mission, edge_task_map=edge_task_map or {})
    if not tasks:
        raise ReplanValidationError("No inspect segments in baseline mission")

    visit_order = _edge_visit_order(base_mission, tasks)
    visit_order = [e for e in visit_order if e in tasks]
    if not visit_order:
        raise ReplanValidationError(
            "No edges with inspection tasks after pruning empty branches"
        )

    visit_order, first_forward, first_entry = _rotate_order_nearest_start(
        visit_order, tasks, start, weather_context=weather_context
    )

    _log_replan_point_projections(base_mission, tasks, visit_order)

    cost_cfg = _weather_cost_config_for_dijkstra(weather_context)
    topo_reload_ok = topo_graph is not None and bool(edge_task_map)
    missing_topo_edges = [eid for eid in visit_order if eid not in edge_task_map]
    if topo_reload_ok and missing_topo_edges:
        print(
            f"[WARN] replan: baseline visit_order has edges not in rebuilt topo "
            f"(count={len(missing_topo_edges)}): {missing_topo_edges[:8]}"
        )

    segments_out: List[Dict[str, Any]] = []
    directions: Dict[str, bool] = {}
    seg_idx = 0
    current: Point = start
    prev_edge: Optional[str] = None

    connect_start_geom = _connect_geometry_topo(
        start,
        first_entry,
        role="from_start",
        from_edge_id=None,
        to_edge_id=visit_order[0],
        topo_graph=topo_graph,
        edge_task_map=edge_task_map,
        cost_config=cost_cfg,
    )
    segments_out.append(
        _make_connect_segment(
            seg_idx,
            connect_start_geom,
            role="from_start",
            from_edge_id=None,
            to_edge_id=visit_order[0],
            segment_id="connect_from_start",
        )
    )
    seg_idx += 1
    current = first_entry

    for i, eid in enumerate(visit_order):
        poly = tasks[eid]["polyline"]
        if i == 0:
            forward = first_forward
        else:
            d_fwd = _dist(current, poly[0]) + _weather_penalty_for_line(current, poly[0], weather_context)
            d_rev = _dist(current, poly[-1]) + _weather_penalty_for_line(current, poly[-1], weather_context)
            forward = d_fwd <= d_rev
        directions[eid] = forward

        inspect_geom = poly if forward else list(reversed(poly))
        entry: Point = inspect_geom[0]
        exit_pt: Point = inspect_geom[-1]

        if i > 0 and _dist(current, entry) > 0.5:
            connect_geom = _connect_geometry_topo(
                current,
                entry,
                role="between_edges",
                from_edge_id=prev_edge,
                to_edge_id=eid,
                topo_graph=topo_graph,
                edge_task_map=edge_task_map,
                cost_config=cost_cfg,
            )
            segments_out.append(
                _make_connect_segment(
                    seg_idx,
                    connect_geom,
                    role="between_edges",
                    from_edge_id=prev_edge,
                    to_edge_id=eid,
                )
            )
            seg_idx += 1
            current = entry

        segments_out.append(
            _make_inspect_segment(
                seg_idx,
                inspect_geom,
                eid,
                from_edge_id=prev_edge,
                direction="forward" if forward else "reverse",
            )
        )
        seg_idx += 1
        current = exit_pt
        prev_edge = eid

    connect_end_geom = _connect_geometry_topo(
        current,
        end,
        role="to_end",
        from_edge_id=prev_edge,
        to_edge_id=None,
        topo_graph=topo_graph,
        edge_task_map=edge_task_map,
        cost_config=cost_cfg,
    )
    end_connected = _dist(connect_end_geom[-1], end) < 2.0
    segments_out.append(
        _make_connect_segment(
            seg_idx,
            connect_end_geom,
            role="to_end",
            from_edge_id=prev_edge,
            to_edge_id=None,
            segment_id="connect_to_end",
        )
    )
    seg_idx += 1

    if not end_connected:
        raise ReplanValidationError(
            "Failed to connect final path to end point"
        )

    inspect_len = sum(
        s["length"] for s in segments_out if s["type"] == "inspect"
    )
    connect_len = sum(
        s["length"] for s in segments_out if s["type"] == "connect"
    )
    total_len = inspect_len + connect_len

    base_meta = copy.deepcopy(base_mission.get("metadata") or {})
    ctx = mission_context or {}
    planning_points, resolved_source = _resolve_replan_inspection_points(
        base_mission,
        visit_order=visit_order,
        tasks=tasks,
        directions=directions,
        spacing=spacing,
        mission_context=ctx,
    )
    if is_image_inspection_source(resolved_source):
        resolved_source = "image"
        # 底图路径以 baseline 为准，禁止用 planning ctx 覆盖为 test.png 等默认图
        if ctx.get("image_detection_stats") and not base_meta.get("image_detection_stats"):
            base_meta["image_detection_stats"] = copy.deepcopy(ctx.get("image_detection_stats"))
        ovr = ctx.get("image_inspection_overlay")
        if ovr and not (base_meta.get("image_inspection_overlay") or []):
            base_meta["image_inspection_overlay"] = copy.deepcopy(ovr)
    else:
        resolved_source = "spacing"
    base_meta["inspection_point_source"] = resolved_source

    base_stats = copy.deepcopy(base_mission.get("statistics") or {})

    statistics = {
        **base_stats,
        "total_length": round(total_len, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "inspect_ratio": round(
            (inspect_len / total_len * 100.0) if total_len > 0 else 0.0, 2
        ),
        "num_segments": len(segments_out),
        "num_inspection_points": len(planning_points),
        "num_edges": len(visit_order),
    }

    mission = {
        "metadata": {
            **base_meta,
            "planner_name": "StartEndReplanPlanner",
            "replan_engine": "planner/replan_start_end.py",
        },
        "statistics": statistics,
        "groups": copy.deepcopy(base_mission.get("groups") or []),
        "visit_order": {
            "edge_visit_order": visit_order,
            "edge_direction": {
                eid: ("forward" if directions[eid] else "reverse")
                for eid in visit_order
            },
            "group_visit_order": base_mission.get("visit_order", {}).get(
                "group_visit_order", []
            )
            if isinstance(base_mission.get("visit_order"), dict)
            else [],
        },
        "segments": segments_out,
        "inspection_points": planning_points,
        "markers": {
            "start": {"x": start[0], "y": start[1]},
            "end": {"x": end[0], "y": end[1]},
        },
        "replan_metadata": {
            "planner": "start_end_replan",
            "start": [start[0], start[1]],
            "end": [end[0], end[1]],
            "end_connected": end_connected,
            "planning_spacing": spacing,
            "planning_point_count": len(planning_points),
            "inspection_point_source": resolved_source,
            "weather_aware": bool((weather_context or {}).get("enabled")),
            "weather_weight": float((weather_context or {}).get("weather_weight", 1.0)),
            "connect_planner": "dijkstra",
            "topo_context_reload": topo_reload_ok,
            "topo_reload_image": image_rel,
            "topo_reload_source": src_for_topo,
            "topo_missing_edges": missing_topo_edges[:32],
        },
    }
    if is_image_inspection_source(resolved_source):
        mission["image_inspection_overlay"] = copy.deepcopy(
            ctx.get("image_inspection_overlay")
            or base_meta.get("image_inspection_overlay")
            or base_mission.get("image_inspection_overlay")
            or []
        )
    _log_replan_path_debug(segments_out, visit_order, tag="start_end_replan")
    return mission
