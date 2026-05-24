"""
Start–End driven mission replan for Image Pipeline Dashboard.

Given user start/end and a baseline mission JSON, rebuilds segments:
  start → connect → inspect all edges (full polylines) → connect_to_end → end
"""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, List, Optional, Tuple
from weather.weather_cost import compute_edge_weather_penalty

IMAGE_WIDTH = 916
IMAGE_HEIGHT = 960

Point = Tuple[float, float]


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
    total = _path_length(polyline)
    if total < 1e-9:
        return list(polyline)
    start_s = max(0.0, min(total, s0))
    end_s = max(0.0, min(total, s1))
    if end_s < start_s:
        start_s, end_s = end_s, start_s
    start_pt = _point_at_distance(polyline, start_s)
    end_pt = _point_at_distance(polyline, end_s)
    out: List[Point] = [start_pt]
    cum = 0.0
    for i in range(len(polyline) - 1):
        seg_len = _dist(polyline[i], polyline[i + 1])
        if seg_len < 1e-9:
            continue
        if cum > start_s + 1e-9 and cum < end_s - 1e-9:
            out.append((float(polyline[i][0]), float(polyline[i][1])))
        cum += seg_len
    if _dist(out[-1], end_pt) > 1e-6:
        out.append(end_pt)
    if len(out) == 1:
        out.append(end_pt)
    return out


def _interpolate_line(p0: Point, p1: Point, step: float = 5.0) -> List[Point]:
    """Dense connect polyline for display (similar to legacy mission connects)."""
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


def _extract_inspect_tasks(base_mission: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    edge_points: Dict[str, List[Point]] = {}
    for pt in base_mission.get("inspection_points", []) or []:
        edge_id = pt.get("edge_id")
        pos = pt.get("pixel_position") or [pt.get("x"), pt.get("y")]
        if not edge_id or not pos or len(pos) < 2:
            continue
        edge_points.setdefault(edge_id, []).append((float(pos[0]), float(pos[1])))

    tasks: Dict[str, Dict[str, Any]] = {}
    for seg in base_mission.get("segments", []):
        if seg.get("type") != "inspect":
            continue
        eid = seg.get("edge_id")
        if not eid:
            continue
        geom = seg.get("geometry_2d") or []
        polyline: List[Point] = [
            (float(p[0]), float(p[1])) for p in geom if len(p) >= 2
        ]
        if len(polyline) < 2:
            continue
        required_points = edge_points.get(eid, [])
        if required_points:
            projected = [
                _project_to_polyline_distance(pt, polyline)
                for pt in required_points
            ]
            projected = [s for s in projected if s is not None]
            if projected:
                s_min = min(projected)
                s_max = max(projected)
                trimmed = _slice_polyline(polyline, s_min, s_max)
                if len(trimmed) >= 2:
                    print(
                        f"[DEBUG] dead-end branch skipped/truncated edge={eid} "
                        f"slice=({s_min:.2f},{s_max:.2f}) full={_path_length(polyline):.2f} "
                        f"trimmed={_path_length(trimmed):.2f}"
                    )
                    polyline = trimmed
        tasks[eid] = {
            "edge_id": eid,
            "polyline": polyline,
            "length": round(_path_length(polyline), 2),
        }
    return tasks


def _edge_visit_order(base_mission: Dict[str, Any], tasks: Dict[str, Dict[str, Any]]) -> List[str]:
    visit = base_mission.get("visit_order", {})
    if isinstance(visit, dict):
        order = list(visit.get("edge_visit_order") or [])
    elif isinstance(visit, list):
        order = list(visit)
    else:
        order = []

    ordered = [eid for eid in order if eid in tasks]
    for eid in tasks:
        if eid not in ordered:
            ordered.append(eid)
    if not ordered:
        for seg in base_mission.get("segments", []):
            if seg.get("type") == "inspect" and seg.get("edge_id"):
                eid = seg["edge_id"]
                if eid not in ordered:
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


def build_start_end_replan_mission(
    base_mission: Dict[str, Any],
    start_xy: List[float],
    end_xy: List[float],
    planning_spacing: float = 70.0,
    weather_context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Build full mission JSON: start → inspect all edges (dense polylines) → end.
    """
    start, end = validate_image_coords(start_xy, end_xy)
    spacing = max(20.0, float(planning_spacing))

    tasks = _extract_inspect_tasks(base_mission)
    if not tasks:
        raise ReplanValidationError("No inspect segments in baseline mission")

    visit_order = _edge_visit_order(base_mission, tasks)
    visit_order, first_forward, first_entry = _rotate_order_nearest_start(
        visit_order, tasks, start, weather_context=weather_context
    )

    segments_out: List[Dict[str, Any]] = []
    directions: Dict[str, bool] = {}
    seg_idx = 0
    current: Point = start
    prev_edge: Optional[str] = None

    connect_start_geom = _interpolate_line(start, first_entry)
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
            connect_geom = _interpolate_line(current, entry)
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

    connect_end_geom = _interpolate_line(current, end)
    end_connected = _dist(connect_end_geom[-1], end) < 1.0
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

    planning_points = _build_planning_inspection_points(
        tasks, visit_order, directions, spacing
    )

    base_meta = copy.deepcopy(base_mission.get("metadata") or {})
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
            "end_connected": True,
            "planning_spacing": spacing,
            "planning_point_count": len(planning_points),
            "weather_aware": bool((weather_context or {}).get("enabled")),
            "weather_weight": float((weather_context or {}).get("weather_weight", 1.0)),
        },
    }
    return mission
