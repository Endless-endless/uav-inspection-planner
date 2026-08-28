"""
PhysicalLineChain：跨 line_id 的链级任务合并层（在 Chain EdgeTask 之上）。

不修改 topo_graph / junction / replan；仅消费已有 Chain EdgeTask 与 TopoGraph。
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from core.image_pixel_coords import edge_pixel_polyline, freeze_pixel_polyline
from core.topo import TopoGraph
from core.topo_task import EdgeTask, _stitch_edge_task_polylines, project_point_to_polyline


def _merged_chain_node_set(task: Any, topo_graph: TopoGraph) -> Set[str]:
    ids = (getattr(task, "meta", None) or {}).get("chain_topo_edge_ids") or []
    nodes: Set[str] = set()
    for eid in ids:
        e = topo_graph.edges.get(eid)
        if e:
            nodes.add(e.u)
            nodes.add(e.v)
    if not nodes and getattr(task, "u", None) and getattr(task, "v", None):
        nodes.update([str(task.u), str(task.v)])
    return nodes


def _voltage_level(task: Any) -> Optional[str]:
    m = getattr(task, "meta", None) or {}
    v = m.get("voltage_level")
    if v is not None:
        return str(v)
    v = m.get("voltage")
    return str(v) if v is not None else None


def _acute_angle_deg(u: Tuple[float, float], v: Tuple[float, float]) -> float:
    nu = math.hypot(u[0], u[1])
    nv = math.hypot(v[0], v[1])
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    c = abs(u[0] * v[0] + u[1] * v[1]) / (nu * nv)
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def _closest_poly_index(poly: List[Tuple[float, float]], qx: float, qy: float) -> int:
    best_i = 0
    best_d = float("inf")
    for i, (x, y) in enumerate(poly):
        d = math.hypot(float(x) - qx, float(y) - qy)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def _tangent_from_index(poly: List[Tuple[float, float]], idx: int, toward: Tuple[float, float]) -> Tuple[float, float]:
    """在 idx 处选择沿折线指向 toward 一侧的单位切向。"""
    tx, ty = float(toward[0]), float(toward[1])
    px, py = float(poly[idx][0]), float(poly[idx][1])
    best: Optional[Tuple[float, float]] = None
    best_dot = -2.0
    for j in (idx - 1, idx + 1):
        if j < 0 or j >= len(poly):
            continue
        vx = px - float(poly[j][0])
        vy = py - float(poly[j][1])
        n = math.hypot(vx, vy)
        if n < 1e-9:
            continue
        vx, vy = vx / n, vy / n
        ux, uy = tx - px, ty - py
        un = math.hypot(ux, uy) or 1.0
        ux, uy = ux / un, uy / un
        dot = vx * ux + vy * uy
        if dot > best_dot:
            best_dot, best = dot, (vx, vy)
    if best is None:
        return (1.0, 0.0)
    return best


def _merge_pair_metrics(
    ta: EdgeTask,
    tb: EdgeTask,
    topo_graph: TopoGraph,
    *,
    max_dist: float = 200.0,
    max_angle_deg: float = 35.0,
) -> Tuple[bool, List[str], Optional[float], Optional[float]]:
    """
    返回 (是否可合并, 共享节点 id 排序列表, 最优距离, 对应夹角°)。
    距离/角度在「deg<=2 的共享节点」上取使距离最小的那一组。
    """
    sa = _merged_chain_node_set(ta, topo_graph)
    sb = _merged_chain_node_set(tb, topo_graph)
    shared_set = sa & sb
    if not shared_set:
        return False, [], None, None

    va, vb = _voltage_level(ta), _voltage_level(tb)
    voltage_ok = not (va is not None and vb is not None and va != vb)

    pa = edge_pixel_polyline(ta)
    pb = edge_pixel_polyline(tb)
    if len(pa) < 2 or len(pb) < 2:
        return False, sorted(shared_set), None, None

    best_d: Optional[float] = None
    best_ang: Optional[float] = None
    eligible = False

    for nid in shared_set:
        node = topo_graph.nodes.get(nid)
        if node is None:
            continue
        if int(getattr(node, "deg", 99) or 99) > 2:
            continue
        qx, qy = float(node.pos2d[0]), float(node.pos2d[1])
        ia = _closest_poly_index(pa, qx, qy)
        ib = _closest_poly_index(pb, qx, qy)
        d = math.hypot(pa[ia][0] - pb[ib][0], pa[ia][1] - pb[ib][1])
        ta_vec = _tangent_from_index(pa, ia, (qx, qy))
        tb_vec = _tangent_from_index(pb, ib, (qx, qy))
        ang = _acute_angle_deg(ta_vec, tb_vec)
        if best_d is None or d < best_d:
            best_d, best_ang = d, ang
        if voltage_ok and d < max_dist and ang < max_angle_deg:
            eligible = True

    if not voltage_ok:
        eligible = False
    return eligible, sorted(shared_set), best_d, best_ang


def _merge_pair_ok(
    ta: EdgeTask,
    tb: EdgeTask,
    topo_graph: TopoGraph,
    *,
    max_dist: float = 200.0,
    max_angle_deg: float = 35.0,
) -> bool:
    ok, _, _, _ = _merge_pair_metrics(
        ta, tb, topo_graph, max_dist=max_dist, max_angle_deg=max_angle_deg
    )
    return ok


class _UF:
    def __init__(self, items: List[str]):
        self.p = {x: x for x in items}

    def find(self, x: str) -> str:
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a: str, b: str) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


def _order_chain_ids_for_stitch(
    comp: Set[str],
    tasks_by_id: Dict[str, EdgeTask],
    topo_graph: TopoGraph,
) -> Optional[List[str]]:
    ids = list(comp)
    if len(ids) == 1:
        return ids
    if len(ids) > 8:
        return None

    for perm in itertools.permutations(ids):
        ts = [tasks_by_id[i] for i in perm]
        pl = _stitch_edge_task_polylines(ts)
        if len(pl) >= 2:
            return list(perm)
    return None


def _order_chain_ids_for_component(comp: Set[str], adj: Dict[str, Set[str]]) -> List[str]:
    """按合并图给 component 一个稳定遍历顺序，供 fallback 展示折线使用。"""
    ids = sorted(comp)
    if len(ids) <= 1:
        return ids

    local_degree = {
        cid: len([n for n in adj.get(cid, set()) if n in comp])
        for cid in ids
    }
    starts = [cid for cid in ids if local_degree[cid] <= 1]
    start = starts[0] if starts else ids[0]

    ordered: List[str] = []
    seen: Set[str] = set()

    def dfs(cid: str) -> None:
        seen.add(cid)
        ordered.append(cid)
        for nb in sorted(adj.get(cid, set())):
            if nb in comp and nb not in seen:
                dfs(nb)

    dfs(start)
    for cid in ids:
        if cid not in seen:
            dfs(cid)
    return ordered


def _shared_junction_point(
    ta: EdgeTask,
    tb: EdgeTask,
    topo_graph: TopoGraph,
) -> Optional[Tuple[float, float]]:
    shared = _merged_chain_node_set(ta, topo_graph) & _merged_chain_node_set(tb, topo_graph)
    if not shared:
        return None

    def rank(nid: str) -> Tuple[int, int, str]:
        node = topo_graph.nodes.get(nid)
        kind_rank = 0 if node and getattr(node, "kind", "") == "junction" else 1
        deg = int(getattr(node, "deg", 99) or 99) if node else 99
        return kind_rank, deg, nid

    for nid in sorted(shared, key=rank):
        node = topo_graph.nodes.get(nid)
        if node is not None:
            return (float(node.pos2d[0]), float(node.pos2d[1]))
    return None


def _append_polyline(
    out: List[Tuple[float, float]],
    poly: List[Tuple[float, float]],
) -> None:
    if not poly:
        return
    pts = [(float(x), float(y)) for x, y in poly]
    if out and pts and math.hypot(out[-1][0] - pts[0][0], out[-1][1] - pts[0][1]) < 1e-6:
        out.extend(pts[1:])
    else:
        out.extend(pts)


def _build_junction_fallback_polyline(
    ordered: List[str],
    tasks_by_id: Dict[str, EdgeTask],
    topo_graph: TopoGraph,
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    prev_task: Optional[EdgeTask] = None
    for cid in ordered:
        task = tasks_by_id[cid]
        if prev_task is not None:
            junction = _shared_junction_point(prev_task, task, topo_graph)
            if junction is not None:
                _append_polyline(out, [junction])
        _append_polyline(out, edge_pixel_polyline(task))
        prev_task = task
    return freeze_pixel_polyline(out)


def _polyline_length(poly: List[Tuple[float, float]]) -> float:
    return float(
        sum(
            math.hypot(poly[i + 1][0] - poly[i][0], poly[i + 1][1] - poly[i][1])
            for i in range(len(poly) - 1)
        )
    )


def _junction_nodes_in_set(node_ids: Set[str], topo_graph: TopoGraph) -> int:
    n = 0
    for nid in node_ids:
        node = topo_graph.nodes.get(nid)
        if node and getattr(node, "kind", "") == "junction":
            n += 1
    return n


def _collect_topo_edges(tasks: List[EdgeTask]) -> List[str]:
    out: List[str] = []
    for t in tasks:
        for teid in (getattr(t, "meta", None) or {}).get("chain_topo_edge_ids") or []:
            out.append(str(teid))
    return out


def _unique_line_ids(tasks: List[EdgeTask]) -> List[str]:
    lids: List[str] = []
    for t in tasks:
        lid = getattr(t, "line_id", "") or ""
        if lid and lid not in lids:
            lids.append(lid)
    return lids


def _merge_inspection_points(
    merged_poly: List[Tuple[float, float]],
    tasks_in_order: List[EdgeTask],
) -> Tuple[List[Any], int]:
    raw: List[Any] = []
    for t in tasks_in_order:
        raw.extend(list(getattr(t, "inspection_points", None) or []))
    if not merged_poly or len(merged_poly) < 2:
        return raw, len(raw)
    enriched: List[Any] = []
    for p in raw:
        xy = None
        if isinstance(p, dict):
            pos = p.get("pixel_position") or p.get("position_2d") or p.get("pos2d")
            if pos and len(pos) >= 2:
                xy = (float(pos[0]), float(pos[1]))
        else:
            pos = getattr(p, "pixel_position", None)
            if pos is not None and len(pos) >= 2:
                xy = (float(pos[0]), float(pos[1]))
        if xy is None:
            enriched.append(p)
            continue
        proj = project_point_to_polyline(xy, merged_poly)
        if isinstance(p, dict):
            q = dict(p)
            if proj:
                q["distance_along_edge"] = proj["distance_along_edge"]
            enriched.append(q)
        else:
            enriched.append(p)
    return enriched, len(enriched)


def _assign_points_to_physical_line(points: List[Any], physical_id: str) -> None:
    for p in points:
        if isinstance(p, dict):
            prev = p.get("edge_id")
            if prev and prev != physical_id:
                p.setdefault("member_chain_id", prev)
            p["edge_id"] = physical_id
        else:
            try:
                prev = getattr(p, "edge_id", None)
                if prev and prev != physical_id and not getattr(p, "member_chain_id", None):
                    setattr(p, "member_chain_id", prev)
                setattr(p, "edge_id", physical_id)
            except Exception:
                pass


def _terminal_uv(
    tasks_in_order: List[EdgeTask],
    merged_poly: List[Tuple[float, float]],
    topo_graph: TopoGraph,
) -> Tuple[str, str]:
    """用首尾像素在成员链的 topo 端点集合中就近取 u、v。"""
    if not merged_poly or len(merged_poly) < 2:
        t0 = tasks_in_order[0]
        t1 = tasks_in_order[-1]
        return str(t0.u), str(t1.v)

    def nearest_node_id(px: float, py: float) -> str:
        best = ""
        best_d = float("inf")
        pool: Set[str] = set()
        for t in tasks_in_order:
            pool |= _merged_chain_node_set(t, topo_graph)
        for nid in pool:
            node = topo_graph.nodes.get(nid)
            if not node:
                continue
            d = math.hypot(float(node.pos2d[0]) - px, float(node.pos2d[1]) - py)
            if d < best_d:
                best_d, best = d, nid
        return best

    x0, y0 = merged_poly[0]
    x1, y1 = merged_poly[-1]
    u = nearest_node_id(float(x0), float(y0))
    v = nearest_node_id(float(x1), float(y1))
    if not u:
        u = str(tasks_in_order[0].u)
    if not v:
        v = str(tasks_in_order[-1].v)
    return u, v


@dataclass
class PhysicalLineChain:
    """跨 Chain EdgeTask 的物理连续巡检单元（Mission 优化粒度）。"""

    id: str
    chain_ids: List[str]
    line_ids: List[str]
    topo_edge_ids: List[str]
    inspection_points: List[Any]
    polyline: List[Tuple[float, float]]
    pixel_polyline: List[Tuple[float, float]]
    original_polyline: List[Tuple[float, float]]
    image_polyline: List[Tuple[float, float]]
    u: str
    v: str
    length: float
    line_id: str = ""
    num_points: int = 0
    is_straight: bool = True
    split_reason: Optional[str] = None
    priority: int = 0
    estimated_time: float = 0.0
    meta: Dict[str, Any] = field(default_factory=dict)

    @property
    def edge_id(self) -> str:
        """与 EdgeTask 对齐，供 mission / adjacency 使用。"""
        return self.id

    @property
    def len2d(self) -> float:
        return float(self.length)


def build_physical_line_chains(
    chain_edge_tasks: List[EdgeTask],
    topo_graph: TopoGraph,
) -> List[PhysicalLineChain]:
    """
    从 Chain EdgeTask 集合构建 PhysicalLineChain（不使用 line_id 作为合并主键）。

    合并图：两链若满足共享 topo/junction 节点、deg<=2、夹角<35°、端距<200px、电压一致则连边；
    连通分量即为 PhysicalLineChain。折线拼接仅用于生成展示 polyline，失败时保留 component 并使用 junction fallback。
    """
    tasks = [t for t in chain_edge_tasks if (getattr(t, "num_points", 0) or 0) > 0]
    node_component: Dict[str, int] = {}
    component_id = 0
    for start_node in sorted(topo_graph.nodes):
        if start_node in node_component:
            continue
        stack = [start_node]
        node_component[start_node] = component_id
        while stack:
            node = stack.pop()
            for neighbor in sorted(topo_graph.adj.get(node, [])):
                if neighbor not in node_component:
                    node_component[neighbor] = component_id
                    stack.append(neighbor)
        component_id += 1
    tasks_by_id: Dict[str, EdgeTask] = {t.edge_id: t for t in tasks}
    ids = sorted(tasks_by_id.keys())

    print("[physical-chain-build] 开始构建 PhysicalLineChain ...")
    n_before = len(tasks)

    print(
        "[physical-chain-build] 候选 pair（chain_a / chain_b / distance / angle / shared_node / eligible）:"
    )
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            ta, tb = tasks_by_id[a], tasks_by_id[b]
            elig, shared_list, bd, bang = _merge_pair_metrics(ta, tb, topo_graph)
            sh = ",".join(shared_list) if shared_list else ""
            ds = f"{bd:.2f}" if bd is not None else "-"
            ag = f"{bang:.2f}" if bang is not None else "-"
            print(
                f"  chain_a={a} chain_b={b} distance={ds} angle={ag} "
                f"shared_node=[{sh}] eligible={elig}"
            )

    adj: Dict[str, Set[str]] = {i: set() for i in ids}
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            ta, tb = tasks_by_id[a], tasks_by_id[b]
            if _merge_pair_ok(ta, tb, topo_graph):
                adj[a].add(b)
                adj[b].add(a)

    uf = _UF(ids)
    for a in ids:
        for b in adj[a]:
            uf.union(a, b)

    comps: Dict[str, List[str]] = {}
    for x in ids:
        r = uf.find(x)
        comps.setdefault(r, []).append(x)

    physical: List[PhysicalLineChain] = []
    pid = 0

    for root in sorted(comps.keys(), key=lambda k: min(comps[k])):
        comp_set = set(comps[root])
        ordered = _order_chain_ids_for_stitch(comp_set, tasks_by_id, topo_graph)
        build_mode = "stitched"
        if not ordered:
            ordered = _order_chain_ids_for_component(comp_set, adj)
            build_mode = "junction_fallback"

        mem = [tasks_by_id[c] for c in ordered]
        merged_pl = _stitch_edge_task_polylines(mem)
        if len(merged_pl) >= 2:
            pl = [(float(x), float(y)) for x, y in merged_pl]
            build_mode = "stitched"
        else:
            pl = _build_junction_fallback_polyline(ordered, tasks_by_id, topo_graph)
            build_mode = "junction_fallback"
        if len(pl) < 2:
            pl = freeze_pixel_polyline(edge_pixel_polyline(mem[0]))

        pts, npt = _merge_inspection_points(pl, mem)
        u0, v0 = _terminal_uv(mem, pl, topo_graph)
        lids = _unique_line_ids(mem)
        teids = _collect_topo_edges(mem)
        straight = all(bool(getattr(x, "is_straight", True)) for x in mem)
        ln = _polyline_length(pl)
        meta = {
            "member_chain_ids": list(ordered),
            "member_line_ids": lids,
            "chain_topo_edge_ids": teids,
            "component_size": len(comp_set),
            "build_mode": build_mode,
            "component_ids": sorted({
                node_component[node]
                for task in mem
                for node in (getattr(task, "u", None), getattr(task, "v", None))
                if node in node_component
            }),
        }
        vl = _voltage_level(mem[0])
        if vl is not None:
            meta["voltage_level"] = vl
        physical_id = f"PL_{pid:03d}"
        _assign_points_to_physical_line(pts, physical_id)
        phy = PhysicalLineChain(
            id=physical_id,
            chain_ids=list(ordered),
            line_ids=lids,
            topo_edge_ids=teids,
            inspection_points=pts,
            polyline=list(pl),
            pixel_polyline=list(pl),
            original_polyline=list(pl),
            image_polyline=list(pl),
            u=u0,
            v=v0,
            length=ln,
            line_id=lids[0] if lids else "",
            num_points=npt,
            is_straight=bool(straight),
            split_reason=f"physical_line_{build_mode}",
            meta=meta,
        )
        jcnt = _junction_nodes_in_set(_merged_chain_node_set(phy, topo_graph), topo_graph)
        print(
            f"  [physical-chain-build] physical_id={phy.id} component_size={len(comp_set)} "
            f"member_chain_ids={phy.chain_ids} build_mode={build_mode} "
            f"member_line_ids={phy.line_ids} length={phy.length:.1f} inspection_count={phy.num_points} "
            f"junction_count={jcnt}"
        )
        physical.append(phy)
        pid += 1

    print(
        f"[physical-chain-build] 完成: ChainEdgeTask 数量={n_before} -> PhysicalLineChain 数量={len(physical)}"
    )
    print("[physical-chain-build] 最终 PhysicalLineChain -> member_chain_ids:")
    for pl in physical:
        print(f"  {pl.id}: {pl.chain_ids}")
    return physical
