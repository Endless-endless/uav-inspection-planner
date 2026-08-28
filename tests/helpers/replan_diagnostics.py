from __future__ import annotations

import math
from collections import Counter, defaultdict, deque

CONNECT_CLASSES = {"topology", "free_flight_between_components", "free_flight_endpoint_access", "unexpected_fallback", "unknown"}

def physical_ids(mission):
    return {str(s["edge_id"]) for s in mission.get("segments", []) if s.get("type") == "inspect" and s.get("edge_id")}

def effective_image_points(mission):
    points = [p for p in mission.get("inspection_points", []) if isinstance(p, dict)]
    if points:
        return points
    overlay = mission.get("image_inspection_overlay") or mission.get("metadata", {}).get("image_inspection_overlay") or []
    return [p for p in overlay if isinstance(p, dict) and p.get("valid", True)]

def point_identity(point):
    return (point.get("point_id") or point.get("id"), tuple(point.get("raw_coord") or point.get("pixel_position") or ()), tuple(point.get("snapped_coord") or point.get("detection_result", {}).get("snapped_coord") or ()))

def segment_gap(left, right):
    return math.dist(left["geometry_2d"][-1][:2], right["geometry_2d"][0][:2])

def polyline_length(geometry):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(geometry, geometry[1:]))

def build_topo_components(topo_edges):
    """Use exported node IDs only; never invent connectivity from visual proximity."""
    adjacency, usable = defaultdict(set), []
    for edge in topo_edges or []:
        if edge.get("u") is None or edge.get("v") is None:
            continue
        u, v = str(edge["u"]), str(edge["v"])
        adjacency[u].add(v); adjacency[v].add(u); usable.append(edge)
    node_component, component = {}, 0
    for node in adjacency:
        if node in node_component:
            continue
        queue, node_component[node] = deque([node]), component
        while queue:
            cur = queue.popleft()
            for nxt in adjacency[cur]:
                if nxt not in node_component:
                    node_component[nxt] = component; queue.append(nxt)
        component += 1
    line_components = defaultdict(set)
    for edge in usable:
        if edge.get("line_id"):
            line_components[str(edge["line_id"])].add(node_component[str(edge["u"])])
    return {line: next(iter(ids)) for line, ids in line_components.items() if len(ids) == 1}

def physical_identity_map(mission):
    """Extract explicit PL relations; numeric/string suffix matching is forbidden."""
    candidates = (
        mission.get("physical_line_chains")
        or mission.get("metadata", {}).get("physical_task_registry")
        or mission.get("metadata", {}).get("physical_line_chains")
        or []
    )
    if isinstance(candidates, dict):
        candidates = [dict(value, physical_id=key) for key, value in candidates.items()]
    line_components = build_topo_components(mission.get("metadata", {}).get("topo_edges_pixel") or [])
    result = {}
    for item in candidates:
        pid = item.get("physical_id") or item.get("id") or item.get("edge_id")
        if not pid:
            continue
        chains = list(item.get("member_chain_ids") or item.get("chain_ids") or [])
        topo = list(item.get("topo_edge_ids") or item.get("chain_topo_edge_ids") or [])
        lines = list(item.get("line_ids") or item.get("member_line_ids") or [])
        exported_components = {int(x) for x in (item.get("component_ids") or [])}
        components = exported_components or {line_components[line] for line in lines if line in line_components}
        result[str(pid)] = {"member_chain_ids": chains, "topo_edge_ids": topo, "line_ids": lines, "component_ids": sorted(components), "connected_component_id": next(iter(components)) if len(components) == 1 else None}
    return result

def classify_connect(segment, identity):
    """Classify by endpoint role and explicit topology provenance, not straightness."""
    if segment.get("role") in {"from_start", "to_end"}:
        return "free_flight_endpoint_access", "start/end network access"
    src, dst = segment.get("from_edge_id"), segment.get("to_edge_id")
    src_id, dst_id = identity.get(str(src)), identity.get(str(dst))
    if not src_id or not dst_id:
        return "unknown", f"missing physical identity: from={src!r}, to={dst!r}"
    sc, dc = src_id.get("connected_component_id"), dst_id.get("connected_component_id")
    if sc is not None and dc is not None and sc != dc:
        return "free_flight_between_components", f"component {sc} -> {dc}"
    mode = segment.get("connect_mode") or segment.get("planner_mode")
    reason = str(segment.get("fallback_reason") or segment.get("reason") or "")
    if mode == "free_flight" and segment.get("reason") == "between_components":
        return "free_flight_between_components", reason
    if mode == "fallback":
        return "unexpected_fallback", reason or "fallback without reason"
    if mode in {"along_topo", "dijkstra", "topology"} and not segment.get("topo_edge_ids"):
        return "unknown", "topology mode lacks adopted topo_edge_ids"
    if mode in {"along_topo", "dijkstra", "topology"} or segment.get("topo_edge_ids"):
        return "topology", f"recorded topology evidence: {mode or 'topo_edge_ids'}"
    if any(token in reason for token in ("missing", "exception", "too_few", "mapping")):
        return "unexpected_fallback", reason
    if sc is not None and sc == dc:
        return "unexpected_fallback", "same component but no successful topology evidence"
    return "unknown", "component or planner provenance is not exported"

def classify_mission_connects(mission):
    identity, evidence = physical_identity_map(mission), []
    for segment in mission.get("segments", []):
        if segment.get("type") != "connect":
            continue
        category, reason = classify_connect(segment, identity)
        evidence.append({"segment_id": segment.get("segment_id"), "from_edge_id": segment.get("from_edge_id"), "to_edge_id": segment.get("to_edge_id"), "category": category, "reason": reason})
    return Counter(row["category"] for row in evidence), evidence
