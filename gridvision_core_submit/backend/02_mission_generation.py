"""
GridVision 毕设答辩核心代码摘录 — 巡检任务生成
源文件: planner/powerline_planner_v3_final.py + core/topo_task.py
行号: 2364-2510, 644-722
PowerlinePlannerV3 巡检点步骤 + build_edge_tasks。
本文件为摘录，非可独立运行模块；完整依赖见原工程。
"""

# ===== powerline_planner_v3_final.py L2364-L2510 =====
    def step5_generate_line_inspection_points(
        self,
        spacing=100.0,
        angle_threshold_deg=25.0,
        max_points_per_line=50
    ):
        """
        STEP 5 (新): 为多条独立线路生成巡检点

        规则：
        - 起点、终点必设（endpoint，high priority）
        - 明显拐点必设（turning，high priority）
        - 长线段等距补点（sample，normal priority）
        - 限制每条线最大点数，避免过密

        Args:
            spacing: 采样间距（像素）
            angle_threshold_deg: 拐点角度阈值
            max_points_per_line: 每条线最大点数限制

        Returns:
            List[LineInspectionPoint]: 所有巡检点
        """
        print("[STEP 5] 生成线路巡检点...")

        if not self.independent_lines:
            print("[WARN] 尚未提取独立线路，请先调用 step4_extract_independent_lines()")
            return []

        from core.inspection_point_generator import (
            generate_all_inspection_points,
            save_inspection_points_visualization,
            get_inspection_points_statistics
        )

        # 使用当前地形（如果可用）
        terrain = self.height_map_smooth if hasattr(self, 'height_map_smooth') else None

        # 生成巡检点
        self.line_inspection_points, self.line_inspection_points_by_line = (
            generate_all_inspection_points(
                self.independent_lines,
                terrain=terrain,
                flight_height=self.flight_height,
                spacing=spacing,
                angle_threshold_deg=angle_threshold_deg,
                max_points_per_line=max_points_per_line
            )
        )

        # 保存可视化
        save_inspection_points_visualization(
            self.independent_lines,
            self.line_inspection_points_by_line,
            self.image_path,
            "result/step5_line_inspection_points.png"
        )

        # 打印统计信息
        stats = get_inspection_points_statistics(
            self.line_inspection_points,
            self.line_inspection_points_by_line
        )
        print(f"[巡检点] 统计：")
        print(f"  总点数: {stats['total_points']}")
        print(f"  线路数: {stats['total_lines']}")
        print(f"  - 端点: {stats['endpoint_count']}")
        print(f"  - 拐点: {stats['turning_count']}")
        print(f"  - 采样点: {stats['sample_count']}")
        print(f"  - 高优先级: {stats['high_priority_count']}")

        return self.line_inspection_points

    def load_real_satellite_manual_annotations(self, manual_json_path: str):
        """从手工标注 JSON 加载线路与巡检点（跳过 CV 检测）。"""
        from core.real_satellite_manual import apply_manual_dataset_to_planner

        return apply_manual_dataset_to_planner(self, manual_json_path)

    def step5_detect_image_inspection_points(self, detector_config=None):
        """Detect inspection points from the input image (green circles on real map)."""
        print("[STEP 5] 检测图像巡检点...")
        from core.inspection_point_detector import (
            detect_black_inspection_points_with_stats,
            generate_clean_map,
            resolve_detector_config,
        )

        cfg = resolve_detector_config(self.image_path, detector_config)
        self.inspection_detector_config = cfg
        red_mask = self.mask if getattr(self, "mask", None) is not None else None
        self.image_inspection_detections, self.image_detection_stats = (
            detect_black_inspection_points_with_stats(
                self.image_path, config=cfg, red_mask=red_mask
            )
        )
        try:
            from core.real_map_cv import (
                extract_real_map_green_mask,
                is_real_map_detection_image,
                save_green_mask,
            )

            if is_real_map_detection_image(self.image_path):
                rgb_u8 = np.array(self.image.convert("RGB"))
                green_mask = extract_real_map_green_mask(rgb_u8, red_mask=red_mask)
                save_green_mask(green_mask)
        except Exception as exc:
            print(f"[WARN] 真实地图绿色点 mask 调试输出失败: {exc}")
        print("[巡检点检测] 统计:")
        print(f"  - detector: {self.image_detection_stats.get('detector', 'legacy')}")
        print(f"  - raw contour candidates: {self.image_detection_stats.get('contour_candidates', 0)}")
        print(f"  - merged inspection points: {self.image_detection_stats.get('merged_points', 0)}")
        try:
            from core.real_map_cv import is_real_map_detection_image

            real_map_same_image = is_real_map_detection_image(self.image_path)
        except ImportError:
            real_map_same_image = False

        if real_map_same_image:
            self.clean_map_path = str(self.image_path).replace("\\", "/")
            print(f"[图像底图] 真实地图同源标注图: {self.clean_map_path}")
        elif os.environ.get("UAV_DISPLAY_MAP", "").strip() and os.path.isfile(
            os.environ.get("UAV_DISPLAY_MAP", "").strip()
        ):
            display_map = os.environ.get("UAV_DISPLAY_MAP", "").strip()
            self.clean_map_path = display_map.replace("\\", "/")
            print(f"[图像底图] 使用固定显示底图: {self.clean_map_path}")
        else:
            try:
                self.clean_map_path = generate_clean_map(
                    self.image_path,
                    detections=self.image_inspection_detections,
                    config=cfg,
                )
                print(f"[图像底图] 已生成无黑圈底图: {self.clean_map_path}")
            except Exception as exc:
                self.clean_map_path = None
                print(f"[WARN] clean map 生成失败: {exc}")
        self.inspection_point_source = "image"
        self.line_inspection_points = []
        self.line_inspection_points_by_line = {}
        label = "绿色巡检圆" if self.image_detection_stats.get("detector") == "green_hsv" else "图像特征点"
        print(f"[图像巡检点] 检测到 {len(self.image_inspection_detections)} 个{label}")
        return self.image_inspection_detections


# ===== topo_task.py L644-L722 =====
def build_edge_tasks(topo_graph, line_inspection_points_by_line: Dict[str, List]) -> List[EdgeTask]:
    """
    按 line_id 连通链构建合并 EdgeTask（每链一个任务，仅包含有巡检点的链）。
    """
    print("[边任务构建] 开始构建合并链边任务...")

    line_ids = sorted({e.line_id for e in topo_graph.edges.values()})
    chain_specs_by_line = {
        lid: _build_line_chain_specs(topo_graph, lid, log_merge=True) for lid in line_ids
    }
    edge_points = map_points_to_edges(
        topo_graph, line_inspection_points_by_line, chain_specs_by_line=chain_specs_by_line
    )

    edge_tasks: List[EdgeTask] = []
    merged_topo_count = 0
    lines_with_tasks: Set[str] = set()

    for line_id in line_ids:
        for cid, ordered, merged, uu, vv in chain_specs_by_line.get(line_id, []):
            pts = edge_points.get(cid, [])
            if not pts:
                print(
                    f"[edge-task-build] chain_id={cid} line_id={line_id} "
                    f"topo_edges={list(ordered)} mapped_points=0 fallback_points=0 generated=False"
                )
                continue
            pts_sorted = sorted(
                pts,
                key=lambda p: float(
                    p.get("distance_along_edge", 0.0)
                    if isinstance(p, dict)
                    else getattr(p, "distance_along_edge", 0.0)
                ),
            )
            merged_len = _polyline_len2d(merged)
            straight = all(
                (topo_graph.edges[eid].is_straight for eid in ordered if eid in topo_graph.edges)
            ) if ordered else True
            task = EdgeTask(
                edge_id=cid,
                u=uu or (topo_graph.edges[ordered[0]].u if ordered else ""),
                v=vv or (topo_graph.edges[ordered[-1]].v if ordered else ""),
                line_id=line_id,
                polyline=list(merged),
                pixel_polyline=list(merged),
                original_polyline=list(merged),
                image_polyline=list(merged),
                len2d=float(merged_len),
                inspection_points=pts_sorted,
                num_points=len(pts_sorted),
                is_straight=bool(straight),
                split_reason="merged_chain",
                meta={"chain_topo_edge_ids": list(ordered)},
            )
            edge_tasks.append(task)
            merged_topo_count += len(ordered)
            lines_with_tasks.add(line_id)
            fallback_count = sum(
                1 for p in pts_sorted if _point_mapping_mode(p).endswith("_fallback")
            )
            print(
                f"[edge-task-build] chain_id={cid} line_id={line_id} "
                f"topo_edges={list(ordered)} mapped_points={len(pts_sorted)} "
                f"fallback_points={fallback_count} generated=True"
            )

    total_topo = len(topo_graph.edges)
    print(
        f"[edge-task] total_edge_tasks={len(edge_tasks)} total_lines={len(lines_with_tasks)} "
        f"merged_from_topo_edges={merged_topo_count} topo_edges_total={total_topo}"
    )
    print(f"  [边任务构建] 完成: {len(edge_tasks)} 个合并链任务")
    edge_task_pts = sum(len(t.inspection_points or []) for t in edge_tasks)
    print(f"[point-flow] edge_task_points={edge_task_pts}")

    return edge_tasks


