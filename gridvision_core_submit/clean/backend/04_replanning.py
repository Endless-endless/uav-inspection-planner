"""
GridVision 毕设答辩核心代码摘录 — 起终点重规划
源文件: planner/replan_start_end.py
行号: 1401-1705
build_start_end_replan_mission 完整摘录。
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""

# ===== replan_start_end.py L1401-L1705 =====
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
    if edge_task_map:
        tasks = _remap_inspect_tasks_for_merged_chains(tasks, edge_task_map)

    if not tasks:
        raise ReplanValidationError("No inspect segments in baseline mission")

    cost_cfg = _weather_cost_config_for_dijkstra(weather_context)

    visit_order, order_source = _compute_replan_edge_visit_order(
        base_mission,
        tasks,
        topo_graph=topo_graph,
        edge_task_map=edge_task_map or {},
        start=start,
        weather_context=weather_context,
        cost_config=cost_cfg,
    )
    visit_order = [e for e in visit_order if e in tasks]
    if not visit_order:
        raise ReplanValidationError(
            "No edges with inspection tasks after pruning empty branches"
        )

    visit_order, first_forward, first_entry = _rotate_order_nearest_start(
        visit_order, tasks, start, weather_context=weather_context
    )


    topo_reload_ok = topo_graph is not None and bool(edge_task_map)
    missing_topo_edges = [eid for eid in visit_order if eid not in edge_task_map]
    if topo_reload_ok and missing_topo_edges:

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
    _cs_len = _path_length(connect_start_geom)
    _cs_n = len(connect_start_geom)
    _cs_f = _fmt_xy_pt(connect_start_geom[0]) if _cs_n else "(?,?)"
    _cs_l = _fmt_xy_pt(connect_start_geom[-1]) if _cs_n else "(?,?)"
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
            _cs_len = _path_length(connect_geom)
            _cs_n = len(connect_geom)
            _cs_f = _fmt_xy_pt(connect_geom[0]) if _cs_n else "(?,?)"
            _cs_l = _fmt_xy_pt(connect_geom[-1]) if _cs_n else "(?,?)"
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
    _ce_len = _path_length(connect_end_geom)
    _ce_n = len(connect_end_geom)
    _ce_f = _fmt_xy_pt(connect_end_geom[0]) if _ce_n else "(?,?)"
    _ce_l = _fmt_xy_pt(connect_end_geom[-1]) if _ce_n else "(?,?)"
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
    return mission
