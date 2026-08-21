"""
GridVision 毕设答辩核心代码摘录 — 巡检点生成
源文件: core/inspection_point_generator.py
行号: 166-313, 530-621
含沿折线采样、图像检测点吸附到拓扑。
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""

# ===== inspection_point_generator.py L166-L313 =====
def generate_line_inspection_points(
    line,  # IndependentLine 对象
    terrain: Optional[np.ndarray] = None,
    flight_height: float = 25.0,
    spacing: float = 100.0,
    angle_threshold_deg: float = 25.0,
    max_points_per_line: int = 50
) -> List[LineInspectionPoint]:
    """
    对单条独立线路生成巡检点

    规则：
    - 起点、终点必设（endpoint，high priority）
    - 明显拐点必设（turning，high priority）
    - 长线段等距补点（sample，normal priority）
    - 限制每条线最大点数，避免过密

    Args:
        line: IndependentLine 对象
        terrain: 地形高度图（可选）
        flight_height: 飞行高度（米）
        spacing: 采样间距（像素）
        angle_threshold_deg: 拐点角度阈值
        max_points_per_line: 每条线最大点数限制

    Returns:
        List[LineInspectionPoint]: 巡检点列表
    """
    if not line.ordered_pixels:
        return []

    polyline = line.ordered_pixels
    points = []
    point_count = 0
    used_indices = set()  # 记录已使用的索引，避免重复点

    # 1. 端点（起点和终点）- 最高优先级
    if len(polyline) >= 2:
        # 起点
        start_pos = polyline[0]
        start_pos_3d = _map_to_3d(start_pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="endpoint",
            pixel_position=(float(start_pos[0]), float(start_pos[1])),
            position_3d=start_pos_3d,
            line_index=0,
            priority="high",
            status="uninspected",
            source_reason="start_endpoint"
        ))
        used_indices.add(0)
        point_count += 1

        # 终点
        end_pos = polyline[-1]
        end_pos_3d = _map_to_3d(end_pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="endpoint",
            pixel_position=(float(end_pos[0]), float(end_pos[1])),
            position_3d=end_pos_3d,
            line_index=len(polyline) - 1,
            priority="high",
            status="uninspected",
            source_reason="end_endpoint"
        ))
        used_indices.add(len(polyline) - 1)
        point_count += 1

    # 2. 拐点检测 - 高优先级，确保保留
    turning_indices = detect_turning_points(polyline, angle_threshold_deg)
    added_turning = 0
    for idx in turning_indices:
        # 跳过起点和终点附近的拐点
        if idx < 5 or idx > len(polyline) - 5:
            continue

        # 避免重复添加
        if idx in used_indices:
            continue

        pos = polyline[idx]
        pos_3d = _map_to_3d(pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="turning",
            pixel_position=(float(pos[0]), float(pos[1])),
            position_3d=pos_3d,
            line_index=idx,
            priority="high",
            status="uninspected",
            source_reason=f"turning_point_angle_{angle_threshold_deg}"
        ))
        used_indices.add(idx)
        added_turning += 1
        point_count += 1

    # 3. 等距采样点 - 普通优先级，受最大点数限制
    remaining_quota = max_points_per_line - len(points)
    samples = sample_points_along_polyline(polyline, spacing)
    added_samples = 0
    for idx, dist, pos in samples:
        # 达到最大点数限制
        if added_samples >= remaining_quota:
            break

        # 避免与端点重复
        if idx < 5 or idx > len(polyline) - 5:
            continue

        # 避免与拐点重复
        if idx in used_indices:
            continue

        # 避免与已有采样点位置重复
        is_duplicate = any(abs(pos[0] - p.pixel_position[0]) < 3 and
                          abs(pos[1] - p.pixel_position[1]) < 3
                          for p in points if p.point_type == "sample")
        if is_duplicate:
            continue

        pos_3d = _map_to_3d(pos, terrain, flight_height)
        points.append(LineInspectionPoint(
            id=f"{line.id}_P_{point_count:03d}",
            line_id=line.id,
            point_type="sample",
            pixel_position=pos,
            position_3d=pos_3d,
            line_index=idx,
            priority="normal",
            status="uninspected",
            source_reason=f"sample_spacing_{spacing}px"
        ))
        used_indices.add(idx)
        added_samples += 1
        point_count += 1

    # 按线路索引排序
    points.sort(key=lambda p: p.line_index)


    return points


# ===== inspection_point_generator.py L530-L621 =====
def build_image_detected_inspection_points(
    detections: List[Dict[str, Any]],
    topo_graph,
    *,
    terrain: Optional[np.ndarray] = None,
    flight_height: float = 25.0,
    max_snap_distance: float = 30.0,
) -> Tuple[List[LineInspectionPoint], Dict[str, List[LineInspectionPoint]], List[Dict[str, Any]]]:
    """Convert detected image points into snapped LineInspectionPoint objects."""
    from core.topo_task import snap_point_to_topo_graph

    points_by_line: Dict[str, List[LineInspectionPoint]] = {}
    overlay: List[Dict[str, Any]] = []
    all_points: List[LineInspectionPoint] = []
    invalid_count = 0

    for det in detections or []:
        raw = det.get("coord") or det.get("pixel_position")
        if not raw or len(raw) < 2:
            continue
        raw_xy = (float(raw[0]), float(raw[1]))
        snap_strict = snap_point_to_topo_graph(raw_xy, topo_graph, max_distance=max_snap_distance)
        snap_any = snap_point_to_topo_graph(raw_xy, topo_graph, max_distance=None)
        snap = snap_strict or snap_any
        entry = {
            "id": det.get("id"),
            "raw_coord": [round(raw_xy[0], 2), round(raw_xy[1], 2)],
            "valid": snap is not None,
            "confidence": det.get("confidence"),
            "source": det.get("source"),
        }
        if snap is None:
            invalid_count += 1
            entry["snap_distance"] = None
            overlay.append(entry)
            continue

        snap_fallback = snap_strict is None
        dist_px = float(snap["distance"])
        snapped = snap["snapped_coord"]
        pos_3d = _map_to_3d(raw_xy, terrain, flight_height)
        point = LineInspectionPoint(
            id=str(det.get("id") or f"IP_{len(all_points) + 1:04d}"),
            line_id=str(snap["line_id"]),
            point_type="image_detected",
            pixel_position=(float(raw_xy[0]), float(raw_xy[1])),
            position_3d=pos_3d,
            line_index=int(snap.get("segment_index", 0)),
            priority="high",
            status="uninspected",
            source_reason="image_snap_fallback" if snap_fallback else "image_black_dot",
            detection_result={
                "raw_coord": [round(raw_xy[0], 2), round(raw_xy[1], 2)],
                "edge_id": snap["edge_id"],
                "distance_along_edge": round(float(snap["distance_along_edge"]), 2),
                "snap_distance": round(dist_px, 2),
                "snap_distance_px": round(dist_px, 2),
                "snapped_coord": [round(float(snapped[0]), 2), round(float(snapped[1]), 2)],
                "confidence": det.get("confidence"),
                "source": det.get("source"),
                "snap_fallback": snap_fallback,
            },
        )
        points_by_line.setdefault(point.line_id, []).append(point)
        all_points.append(point)
        entry.update(
            {
                "edge_id": snap["edge_id"],
                "snapped_coord": [round(float(snapped[0]), 2), round(float(snapped[1]), 2)],
                "distance_along_edge": round(float(snap["distance_along_edge"]), 2),
                "snap_distance": round(dist_px, 2),
                "snap_fallback": snap_fallback,
            }
        )
        overlay.append(entry)

    for line_id in points_by_line:
        points_by_line[line_id].sort(
            key=lambda p: float((p.detection_result or {}).get("distance_along_edge", p.line_index))
        )

    return all_points, points_by_line, overlay

