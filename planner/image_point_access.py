"""Explicit Mission segments for off-route image inspection points."""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, Iterable, List, Sequence, Tuple

from core.topo_plan import MissionSegment


# Separate from the 120 px line-association threshold.  This threshold only
# decides whether a mapped evidence point needs an explicit physical visit.
POINT_ACCESS_TRIGGER_DISTANCE_PX = 5.0

Point = Tuple[float, float]


def geometry_length(geometry: Sequence[Sequence[float]]) -> float:
    return float(
        sum(
            math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            for a, b in zip(geometry, geometry[1:])
        )
    )


def _project(point: Point, geometry: Sequence[Sequence[float]]) -> Dict[str, Any] | None:
    if len(geometry) < 2:
        return None
    px, py = point
    best: Dict[str, Any] | None = None
    arc_base = 0.0
    for index, (a, b) in enumerate(zip(geometry, geometry[1:])):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dx, dy = bx - ax, by - ay
        segment_length = math.hypot(dx, dy)
        if segment_length <= 1e-12:
            continue
        t = max(
            0.0,
            min(1.0, ((px - ax) * dx + (py - ay) * dy) / (segment_length**2)),
        )
        anchor = (ax + t * dx, ay + t * dy)
        distance = math.hypot(px - anchor[0], py - anchor[1])
        candidate = {
            "edge_index": index,
            "t": t,
            "arc": arc_base + t * segment_length,
            "distance": distance,
            "anchor": anchor,
        }
        if best is None or distance < float(best["distance"]):
            best = candidate
        arc_base += segment_length
    return best


def _image_identity(point: Any) -> Tuple[str, Point, Dict[str, Any]] | None:
    if isinstance(point, dict):
        detection = point.get("detection_result") or {}
        point_id = point.get("point_id") or point.get("id")
        point_type = point.get("point_type") or point.get("type")
        source_reason = point.get("source_reason")
        pixel_position = point.get("pixel_position")
    else:
        detection = getattr(point, "detection_result", None) or {}
        point_id = getattr(point, "point_id", None) or getattr(point, "id", None)
        point_type = getattr(point, "point_type", None)
        source_reason = getattr(point, "source_reason", None)
        pixel_position = getattr(point, "pixel_position", None)

    image_point = (
        str(point_type or "").lower() == "image_detected"
        or str(source_reason or "").lower().startswith("image_")
        or bool(isinstance(detection, dict) and detection.get("raw_coord"))
    )
    if not image_point:
        return None
    if not point_id:
        raise ValueError("Image inspection point is missing stable point_id")
    raw = detection.get("raw_coord") if isinstance(detection, dict) else None
    raw = raw or pixel_position
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        raise ValueError(f"Image inspection point {point_id} is missing raw coordinate")
    return str(point_id), (float(raw[0]), float(raw[1])), dict(detection)


def _set_detection(point: Any, detection: Dict[str, Any]) -> None:
    if isinstance(point, dict):
        point["detection_result"] = detection
    else:
        point.detection_result = detection


def _access_candidates(
    geometry: Sequence[Sequence[float]],
    inspection_points: Iterable[Any],
    *,
    physical_id: str,
    trigger_distance_px: float,
    excluded_point_ids: Iterable[str] = (),
) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    seen_ids = {str(point_id) for point_id in excluded_point_ids}
    for point in inspection_points:
        identity = _image_identity(point)
        if identity is None:
            continue
        point_id, raw, detection = identity
        if point_id in seen_ids:
            continue
        seen_ids.add(point_id)
        projection = _project(raw, geometry)
        if projection is None or float(projection["distance"]) <= trigger_distance_px:
            continue
        anchor = projection["anchor"]
        access_geometry = [anchor, raw, anchor]
        access = {
            "inspection_point_id": point_id,
            "physical_id": str(physical_id),
            "anchor_coord": [float(anchor[0]), float(anchor[1])],
            "raw_coord": [float(raw[0]), float(raw[1])],
            "geometry_2d": [[float(p[0]), float(p[1])] for p in access_geometry],
            "length": geometry_length(access_geometry),
            "connect_mode": "free_flight",
            "planner": "euclidean",
            "reason": "point_access",
            "fallback_reason": None,
            "trigger_distance_px": float(trigger_distance_px),
        }
        detection.update(
            {
                "route_visit_coord": list(access["raw_coord"]),
                "point_access_anchor": list(access["anchor_coord"]),
                "point_access_mode": access["connect_mode"],
                "point_access_reason": access["reason"],
            }
        )
        _set_detection(point, detection)
        candidates.append(
            {
                **projection,
                "point_id": point_id,
                "raw": raw,
                "access": access,
            }
        )
    return sorted(candidates, key=lambda item: (float(item["arc"]), item["point_id"]))


def _split_geometry(
    geometry: Sequence[Sequence[float]],
    candidates: Sequence[Dict[str, Any]],
) -> List[Tuple[str, List[Point], Dict[str, Any] | None]]:
    polyline = [(float(p[0]), float(p[1])) for p in geometry if len(p) >= 2]
    by_edge: Dict[int, List[Dict[str, Any]]] = {}
    for item in candidates:
        by_edge.setdefault(int(item["edge_index"]), []).append(item)
    for rows in by_edge.values():
        rows.sort(key=lambda item: (float(item["t"]), item["point_id"]))

    parts: List[Tuple[str, List[Point], Dict[str, Any] | None]] = []
    current: List[Point] = [polyline[0]]

    def append_distinct(out: List[Point], value: Sequence[float]) -> None:
        candidate = (float(value[0]), float(value[1]))
        if not out or math.hypot(out[-1][0] - candidate[0], out[-1][1] - candidate[1]) > 1e-9:
            out.append(candidate)

    def flush_inspect() -> None:
        nonlocal current
        if len(current) >= 2 and geometry_length(current) > 1e-9:
            parts.append(("inspect", current, None))
        current = [current[-1]]

    for edge_index, (_start, end) in enumerate(zip(polyline, polyline[1:])):
        for item in by_edge.get(edge_index, []):
            append_distinct(current, item["anchor"])
            flush_inspect()
            access_geometry = [
                tuple(item["access"]["anchor_coord"]),
                tuple(item["access"]["raw_coord"]),
                tuple(item["access"]["anchor_coord"]),
            ]
            parts.append(("access", access_geometry, item["access"]))
        append_distinct(current, end)
    flush_inspect()
    return parts


def _object_inspect_part(source: MissionSegment, geometry: List[Point]) -> MissionSegment:
    segment = copy.copy(source)
    segment.geometry = geometry
    segment.length = geometry_length(geometry)
    return segment


def _object_access_segment(access: Dict[str, Any]) -> MissionSegment:
    segment = MissionSegment(
        type="connect",
        from_edge_id=access["physical_id"],
        to_edge_id=access["physical_id"],
        geometry=[tuple(p) for p in access["geometry_2d"]],
        length=float(access["length"]),
        connect_mode=access["connect_mode"],
        planner=access["planner"],
        reason=access["reason"],
        fallback_reason=None,
    )
    segment.inspection_point_id = access["inspection_point_id"]
    segment.physical_id = access["physical_id"]
    segment.anchor_coord = access["anchor_coord"]
    segment.raw_coord = access["raw_coord"]
    segment.trigger_distance_px = access["trigger_distance_px"]
    return segment


def apply_to_object_mission(
    mission: Any,
    *,
    trigger_distance_px: float = POINT_ACCESS_TRIGGER_DISTANCE_PX,
) -> int:
    """Insert explicit access segments in an already ordered object Mission."""
    task_by_id = getattr(mission, "task_by_id", None) or {}
    existing_access_ids = {
        str(getattr(segment, "inspection_point_id", ""))
        for segment in (getattr(mission, "segments", None) or [])
        if getattr(segment, "reason", None) == "point_access"
        and getattr(segment, "inspection_point_id", None)
    }
    output = []
    applied = 0
    for segment in getattr(mission, "segments", None) or []:
        if getattr(segment, "type", None) != "inspect" or not getattr(segment, "edge_id", None):
            output.append(segment)
            continue
        task = task_by_id.get(str(segment.edge_id))
        points = (
            (getattr(task, "inspection_points", None) or [])
            if task is not None
            else []
        )
        candidates = _access_candidates(
            segment.geometry,
            points,
            physical_id=str(segment.edge_id),
            trigger_distance_px=trigger_distance_px,
            excluded_point_ids=existing_access_ids,
        )
        if not candidates:
            output.append(segment)
            continue
        for kind, geometry, access in _split_geometry(segment.geometry, candidates):
            output.append(
                _object_access_segment(access)
                if kind == "access"
                else _object_inspect_part(segment, geometry)
            )
        applied += len(candidates)

    if applied:
        mission.segments = output
        mission.inspect_length = sum(s.length for s in output if s.type == "inspect")
        mission.connect_length = sum(s.length for s in output if s.type == "connect")
        mission.total_length = mission.inspect_length + mission.connect_length
        mission.full_path = [point for segment in output for point in segment.geometry]
    return applied


def _json_inspect_part(source: Dict[str, Any], geometry: List[Point], part_index: int) -> Dict[str, Any]:
    segment = copy.deepcopy(source)
    segment["segment_id"] = f"{source['segment_id']}_part_{part_index}"
    segment["geometry_2d"] = [[float(p[0]), float(p[1])] for p in geometry]
    segment["length"] = geometry_length(geometry)
    return segment


def _json_access_segment(source: Dict[str, Any], access: Dict[str, Any], access_index: int) -> Dict[str, Any]:
    return {
        "segment_id": f"{source['segment_id']}_access_{access_index}",
        "type": "connect",
        "edge_id": None,
        "physical_id": access["physical_id"],
        "from_edge_id": access["physical_id"],
        "to_edge_id": access["physical_id"],
        "length": geometry_length(access["geometry_2d"]),
        "direction": "forward",
        "geometry_2d": copy.deepcopy(access["geometry_2d"]),
        "connect_mode": access["connect_mode"],
        "planner": access["planner"],
        "reason": access["reason"],
        "fallback_reason": None,
        "topo_edge_ids": [],
        "from_component_ids": copy.deepcopy(source.get("from_component_ids") or []),
        "to_component_ids": copy.deepcopy(source.get("to_component_ids") or []),
        "inspection_point_id": access["inspection_point_id"],
        "anchor_coord": copy.deepcopy(access["anchor_coord"]),
        "raw_coord": copy.deepcopy(access["raw_coord"]),
        "trigger_distance_px": access["trigger_distance_px"],
    }


def apply_to_json_segments(
    segments: List[Dict[str, Any]],
    inspection_points: List[Dict[str, Any]],
    *,
    trigger_distance_px: float = POINT_ACCESS_TRIGGER_DISTANCE_PX,
) -> Tuple[List[Dict[str, Any]], int]:
    """Apply the same segment split to a start/end replan Mission."""
    points_by_edge: Dict[str, List[Dict[str, Any]]] = {}
    for point in inspection_points:
        edge_id = point.get("edge_id")
        if edge_id:
            points_by_edge.setdefault(str(edge_id), []).append(point)

    output: List[Dict[str, Any]] = []
    applied = 0
    existing_access_ids = {
        str(segment.get("inspection_point_id"))
        for segment in segments
        if segment.get("reason") == "point_access"
        and segment.get("inspection_point_id")
    }
    for segment in segments:
        if segment.get("type") != "inspect" or not segment.get("edge_id"):
            output.append(segment)
            continue
        candidates = _access_candidates(
            segment.get("geometry_2d") or [],
            points_by_edge.get(str(segment["edge_id"]), []),
            physical_id=str(segment["edge_id"]),
            trigger_distance_px=trigger_distance_px,
            excluded_point_ids=existing_access_ids,
        )
        if not candidates:
            output.append(segment)
            continue
        inspect_index = 0
        access_index = 0
        for kind, geometry, access in _split_geometry(segment["geometry_2d"], candidates):
            if kind == "access":
                access_segment = _json_access_segment(segment, access, access_index)
                output.append(access_segment)
                point = next(
                    row for row in inspection_points
                    if str(row.get("point_id") or row.get("id")) == access["inspection_point_id"]
                )
                point["segment_id"] = access_segment["segment_id"]
                access_index += 1
            else:
                output.append(_json_inspect_part(segment, geometry, inspect_index))
                inspect_index += 1
        applied += len(candidates)
    return output, applied
