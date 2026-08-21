"""
GridVision 毕设答辩核心代码摘录 — Dashboard 导出
源文件: planner/mission_result_builder.py
行号: 292-360, 839-950
CV 坐标抽取 + build_dashboard_from_mission_json。
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""

# ===== mission_result_builder.py L292-L360 =====
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



# ===== mission_result_builder.py L839-L950 =====
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

