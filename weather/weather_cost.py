from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

DEFAULT_WEIGHTS: Dict[str, float] = {
    "wind": 1.5,
    "rain": 1.2,
    "visibility": 1.0,
    "risk": 0.8,
}


def load_weather_zones(weather_file: Optional[Path]) -> List[Dict[str, Any]]:
    if weather_file is None:
        return []
    if not weather_file.exists():
        return []
    try:
        data = json.loads(weather_file.read_text(encoding="utf-8"))
    except Exception:
        return []
    zones = data.get("weather_zones")
    if not isinstance(zones, list):
        return []
    out: List[Dict[str, Any]] = []
    for z in zones:
        if not isinstance(z, dict):
            continue
        center = z.get("center")
        if not isinstance(center, (list, tuple)) or len(center) < 2:
            continue
        radius = float(z.get("radius", 0.0) or 0.0)
        if radius <= 0:
            continue
        out.append(
            {
                "id": z.get("id") or f"zone_{len(out)+1:03d}",
                "type": str(z.get("type") or "risk").lower(),
                "severity": float(z.get("severity", 0.0) or 0.0),
                "center": [float(center[0]), float(center[1])],
                "radius": radius,
                "description": z.get("description") or "",
                "wind_speed": z.get("wind_speed"),
                "rain_level": z.get("rain_level"),
                "visibility_level": z.get("visibility_level"),
                "velocity": z.get("velocity") or [0.0, 0.0],
                "expand_rate": float(z.get("expand_rate", 0.0) or 0.0),
                "severity_rate": float(z.get("severity_rate", 0.0) or 0.0),
                "lifetime": float(z.get("lifetime", 0.0) or 0.0),
                "dynamic": bool(z.get("dynamic", False)),
            }
        )
    return out


def make_weather_context(
    *,
    enabled: bool = False,
    weather_weight: float = 1.0,
    weather_zones: Optional[List[Dict[str, Any]]] = None,
    type_weights: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    tw = dict(DEFAULT_WEIGHTS)
    if type_weights:
        tw.update({str(k).lower(): float(v) for k, v in type_weights.items()})
    return {
        "enabled": bool(enabled),
        "weather_weight": float(weather_weight),
        "type_weights": tw,
        "weather_zones": weather_zones or [],
    }


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _segment_circle_length(a: Point, b: Point, center: Point, radius: float) -> float:
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = float(center[0]), float(center[1])
    dx, dy = bx - ax, by - ay
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-9:
        return 0.0

    fx, fy = ax - cx, ay - cy
    aa = dx * dx + dy * dy
    bb = 2.0 * (fx * dx + fy * dy)
    cc = fx * fx + fy * fy - radius * radius
    disc = bb * bb - 4.0 * aa * cc

    in_a = _dist((ax, ay), (cx, cy)) <= radius
    in_b = _dist((bx, by), (cx, cy)) <= radius
    if disc < 0:
        return seg_len if (in_a and in_b) else 0.0

    sqrt_disc = math.sqrt(max(0.0, disc))
    t1 = (-bb - sqrt_disc) / (2.0 * aa)
    t2 = (-bb + sqrt_disc) / (2.0 * aa)
    lo = max(0.0, min(t1, t2))
    hi = min(1.0, max(t1, t2))
    if hi <= 0.0 or lo >= 1.0:
        return seg_len if (in_a and in_b) else 0.0
    if in_a and in_b:
        return seg_len
    return max(0.0, (hi - lo) * seg_len)


def compute_weather_penalty(
    point: Sequence[float],
    weather_zones: Iterable[Dict[str, Any]],
    *,
    type_weights: Optional[Dict[str, float]] = None,
    weather_weight: float = 1.0,
) -> float:
    x = float(point[0])
    y = float(point[1])
    tw = dict(DEFAULT_WEIGHTS)
    if type_weights:
        tw.update({str(k).lower(): float(v) for k, v in type_weights.items()})

    total = 0.0
    for zone in weather_zones:
        center = zone.get("center") or [0.0, 0.0]
        radius = float(zone.get("radius", 0.0) or 0.0)
        if radius <= 0:
            continue
        if _dist((x, y), (float(center[0]), float(center[1]))) > radius:
            continue
        ztype = str(zone.get("type") or "risk").lower()
        severity = float(zone.get("severity", 0.0) or 0.0)
        total += severity * tw.get(ztype, tw["risk"])
    return total * float(weather_weight)


def compute_edge_weather_penalty(
    edge_polyline: Sequence[Sequence[float]],
    weather_zones: Iterable[Dict[str, Any]],
    *,
    type_weights: Optional[Dict[str, float]] = None,
    weather_weight: float = 1.0,
) -> Dict[str, Any]:
    if not edge_polyline or len(edge_polyline) < 2:
        return {
            "total_penalty": 0.0,
            "affected": False,
            "inside_length": 0.0,
            "components": {"wind": 0.0, "rain": 0.0, "visibility": 0.0, "risk": 0.0},
            "zone_hits": [],
        }

    tw = dict(DEFAULT_WEIGHTS)
    if type_weights:
        tw.update({str(k).lower(): float(v) for k, v in type_weights.items()})
    components = {"wind": 0.0, "rain": 0.0, "visibility": 0.0, "risk": 0.0}
    zone_hits: List[Dict[str, Any]] = []
    total_inside = 0.0

    points = [(float(p[0]), float(p[1])) for p in edge_polyline]
    for zone in weather_zones:
        center = zone.get("center") or [0.0, 0.0]
        radius = float(zone.get("radius", 0.0) or 0.0)
        if radius <= 0:
            continue
        ztype = str(zone.get("type") or "risk").lower()
        zkey = ztype if ztype in components else "risk"
        severity = float(zone.get("severity", 0.0) or 0.0)
        inside = 0.0
        cxy = (float(center[0]), float(center[1]))
        for i in range(len(points) - 1):
            inside += _segment_circle_length(points[i], points[i + 1], cxy, radius)
        if inside <= 1e-6:
            continue
        penalty = inside * severity * tw.get(zkey, tw["risk"])
        components[zkey] += penalty
        total_inside += inside
        zone_hits.append(
            {
                "zone_id": zone.get("id"),
                "type": zkey,
                "severity": severity,
                "inside_length": round(inside, 3),
                "penalty": round(penalty * float(weather_weight), 3),
            }
        )

    total_raw = sum(components.values())
    scaled_components = {k: round(v * float(weather_weight), 3) for k, v in components.items()}
    return {
        "total_penalty": round(total_raw * float(weather_weight), 3),
        "affected": total_raw > 1e-6,
        "inside_length": round(total_inside, 3),
        "components": scaled_components,
        "zone_hits": zone_hits,
    }


def compute_mission_weather_stats(
    segments: Sequence[Dict[str, Any]],
    weather_zones: Sequence[Dict[str, Any]],
    *,
    type_weights: Optional[Dict[str, float]] = None,
    weather_weight: float = 1.0,
    risk_severity_threshold: float = 0.7,
) -> Dict[str, Any]:
    total = 0.0
    affected_edges = 0
    risky_distance = 0.0
    zone_meta = {str(z.get("id")): z for z in weather_zones}
    zone_penalties: Dict[str, Dict[str, Any]] = {}
    segment_penalties: List[Dict[str, Any]] = []
    for seg in segments:
        geom = seg.get("geometry_2d") or []
        penalty = compute_edge_weather_penalty(
            geom,
            weather_zones,
            type_weights=type_weights,
            weather_weight=weather_weight,
        )
        seg_penalty = float(penalty.get("total_penalty", 0.0))
        if penalty["affected"]:
            affected_edges += 1
            total += seg_penalty
            segment_penalties.append(
                {
                    "segment_id": seg.get("segment_id"),
                    "type": seg.get("type"),
                    "penalty": round(seg_penalty, 3),
                }
            )
        for hit in penalty.get("zone_hits") or []:
            zid = str(hit.get("zone_id") or "unknown")
            z = zone_meta.get(zid, {})
            sev = float(hit.get("severity", z.get("severity", 0.0)) or 0.0)
            inside_len = float(hit.get("inside_length", 0.0) or 0.0)
            if sev >= float(risk_severity_threshold):
                risky_distance += inside_len
            cur = zone_penalties.setdefault(
                zid,
                {
                    "zone_id": zid,
                    "type": hit.get("type"),
                    "severity": sev,
                    "description": z.get("description", ""),
                    "penalty": 0.0,
                    "inside_length": 0.0,
                },
            )
            cur["penalty"] += float(hit.get("penalty", 0.0) or 0.0)
            cur["inside_length"] += inside_len

    top_zones = sorted(zone_penalties.values(), key=lambda x: x["penalty"], reverse=True)[:3]
    top_segments = sorted(segment_penalties, key=lambda x: x["penalty"], reverse=True)[:5]
    return {
        "weather_penalty_total": round(total, 3),
        "weather_affected_edges": int(affected_edges),
        "risky_distance": round(risky_distance, 3),
        "top_weather_zones": top_zones,
        "top_weather_segments": top_segments,
    }


def _point_zone_risk(point: Sequence[float], zone: Dict[str, Any]) -> float:
    center = zone.get("center") or [0.0, 0.0]
    if not center or len(center) < 2:
        return 0.0
    dx = float(point[0]) - float(center[0])
    dy = float(point[1]) - float(center[1])
    radius = float(zone.get("radius", 0.0) or 0.0)
    if radius <= 0 or math.hypot(dx, dy) > radius:
        return 0.0
    type_boost = 1.25 if str(zone.get("type", "")).lower() == "risk" else 1.0
    return float(zone.get("severity", 0.0) or 0.0) * type_boost


def predict_future_path_risk(
    path_points: Sequence[Sequence[float]],
    weather_zones: Sequence[Dict[str, Any]],
    *,
    prediction_window: float = 10.0,
    elapsed: float = 0.0,
    bounds: Optional[Dict[str, Sequence[float]]] = None,
    risk_threshold: float = 0.72,
    sample_step: float = 0.5,
    px_per_second: float = 500.0,
) -> Dict[str, Any]:
    """Predict whether moving weather zones will intersect a future path segment."""
    from weather.weather_simulator import forecast_weather_zones

    window = max(0.0, float(prediction_window))
    if window <= 0 or len(path_points) < 2 or not weather_zones:
        return {
            "predicted_risk": 0.0,
            "prediction_window": window,
            "predicted_affected_segments": 0,
            "time_to_risk": None,
        }

    step = max(0.25, float(sample_step))
    max_risk = 0.0
    time_to_risk: Optional[float] = None
    affected = 0

    def point_at_distance(distance: float) -> Optional[List[float]]:
        remain = max(0.0, distance)
        for idx in range(1, len(path_points)):
            ax, ay = float(path_points[idx - 1][0]), float(path_points[idx - 1][1])
            bx, by = float(path_points[idx][0]), float(path_points[idx][1])
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len <= 1e-6:
                continue
            if remain <= seg_len:
                t = remain / seg_len
                return [ax + t * (bx - ax), ay + t * (by - ay)]
            remain -= seg_len
        last = path_points[-1]
        return [float(last[0]), float(last[1])]

    t = 0.0
    while t <= window + 1e-6:
        point = point_at_distance(px_per_second * t)
        if not point:
            break
        forecast = forecast_weather_zones(
            weather_zones,
            t,
            elapsed=elapsed,
            bounds=bounds,
        )
        step_risk = 0.0
        for zone in forecast:
            step_risk = max(step_risk, _point_zone_risk(point, zone))
        if step_risk > max_risk:
            max_risk = step_risk
        if step_risk > risk_threshold:
            affected += 1
            if time_to_risk is None:
                time_to_risk = t
        t += step

    return {
        "predicted_risk": round(max_risk, 4),
        "prediction_window": window,
        "predicted_affected_segments": int(affected),
        "time_to_risk": round(time_to_risk, 2) if time_to_risk is not None else None,
    }
