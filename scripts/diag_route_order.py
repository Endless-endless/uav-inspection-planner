import json
from pathlib import Path

CONT = 1e-3
ROOT = Path(__file__).resolve().parents[1]
p = ROOT / "result" / "web_app" / "latest_dashboard.json"
d = json.loads(p.read_text(encoding="utf-8"))
segs = d.get("segments", [])

print("=== 1. First 15 segments (dashboard.json order) ===")
for i, s in enumerate(segs[:15]):
    print(
        f"{i:2d} segment_id={s.get('segment_id')} "
        f"type={s.get('type')} edge_id={s.get('edge_id')}"
    )

# Build connect arrays WITH segment tags (app.js buildMissionTraces)
connect_x, connect_y, connect_seg = [], [], []
inspect_segs = []
for seg in segs:
    geom = seg.get("geometry_2d") or []
    if len(geom) < 2:
        continue
    sid = seg.get("segment_id")
    stype = seg.get("type")
    eid = seg.get("edge_id")
    if stype == "inspect":
        inspect_segs.append(
            {"segment_id": sid, "edge_id": eid, "geom": geom}
        )
    elif stype == "connect":
        for pt in geom:
            connect_x.append(pt[0])
            connect_y.append(pt[1])
            connect_seg.append(
                {"segment_id": sid, "type": "connect", "edge_id": eid}
            )
        connect_x.append(None)
        connect_y.append(None)
        connect_seg.append(None)
if connect_x and connect_x[-1] is None:
    connect_x.pop()
    connect_y.pop()
    connect_seg.pop()

# Inspect trace: flatten inspect segs in inspectSegs order (without point insert for segment tags)
inspect_x, inspect_y, inspect_seg = [], [], []
for seg in inspect_segs:
    sid, eid = seg["segment_id"], seg["edge_id"]
    for pt in seg["geom"]:
        inspect_x.append(pt[0])
        inspect_y.append(pt[1])
        inspect_seg.append(
            {"segment_id": sid, "type": "inspect", "edge_id": eid}
        )
    inspect_x.append(None)
    inspect_y.append(None)
    inspect_seg.append(None)
if inspect_x and inspect_x[-1] is None:
    inspect_x.pop()
    inspect_y.pop()
    inspect_seg.pop()

start = d.get("markers", {}).get("start", {})
sx, sy = float(start.get("x", 0)), float(start.get("y", 0))


def append_vertices(timeline, xs, ys, seg_meta):
    for i in range(len(xs)):
        x, y = xs[i], ys[i]
        meta = seg_meta[i] if seg_meta and i < len(seg_meta) else None
        if x is None or y is None:
            continue
        px, py = float(x), float(y)
        if timeline:
            prev = timeline[-1]
            if ((prev["x"] - px) ** 2 + (prev["y"] - py) ** 2) ** 0.5 <= CONT:
                continue
        entry = {"x": px, "y": py}
        if meta:
            entry.update(meta)
        timeline.append(entry)
    return timeline


timeline = [{"x": sx, "y": sy, "segment_id": "start", "type": "start", "edge_id": None}]
timeline = append_vertices(timeline, connect_x, connect_y, connect_seg)
timeline = append_vertices(timeline, inspect_x, inspect_y, inspect_seg)
for i, f in enumerate(timeline):
    f["routeIndex"] = i

print("\n=== 2. First 50 timeline frames (segment attribution) ===")
for i in range(min(50, len(timeline))):
    f = timeline[i]
    print(
        f"frame={i} segment_id={f.get('segment_id')} "
        f"segment_type={f.get('type')} edge_id={f.get('edge_id')}"
    )

print("\n=== Dashboard vs playback: first 20 segment types in mission ===")
for i, s in enumerate(segs[:20]):
    print(f"  mission[{i}] {s.get('segment_id')} {s.get('type')}")

print("\n=== Inspection points route_index (exact coord match on timeline) ===")
for pt in sorted(d.get("inspection_points", []), key=lambda p: p.get("progress_index") or 0):
    pid = pt.get("point_id") or pt.get("id")
    px, py = float(pt["x"]), float(pt["y"])
    ri = -1
    for j, f in enumerate(timeline):
        if abs(f["x"] - px) <= CONT and abs(f["y"] - py) <= CONT:
            ri = j
            break
    print(
        f"{pid} progress={pt.get('progress_index')} route_index={ri} "
        f"mission_seg={pt.get('segment_id')} "
        f"timeline_seg={timeline[ri].get('segment_id') if ri >= 0 else '?'} "
        f"timeline_type={timeline[ri].get('type') if ri >= 0 else '?'}"
    )

print("\n=== 3. Structure ===")
n_connect_frames = sum(1 for f in timeline if f.get("type") == "connect")
n_inspect_frames = sum(1 for f in timeline if f.get("type") == "inspect")
print(f"timeline_length={len(timeline)}")
print(f"connect_frames={n_connect_frames} inspect_frames={n_inspect_frames}")
print(f"first_inspect_frame_index={next((i for i,f in enumerate(timeline) if f.get('type')=='inspect'), None)}")
