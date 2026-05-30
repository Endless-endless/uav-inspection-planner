"""真实地图图像管线：强制使用原图像素坐标，禁止拓扑节点重布局坐标。"""

from __future__ import annotations

import os
from typing import Any, List, Sequence, Tuple

Point = Tuple[float, float]


def use_image_pixel_coords() -> bool:
    raw = str(os.environ.get("UAV_IMAGE_PIXEL_COORDS", "")).strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True
    if str(os.environ.get("UAV_DATASET_TYPE", "")).strip() == "real_satellite":
        return True
    return False


def _as_point_list(poly: Sequence[Sequence[float]]) -> List[Point]:
    return [(float(p[0]), float(p[1])) for p in (poly or []) if p is not None and len(p) >= 2]


def freeze_pixel_polyline(poly: Sequence[Sequence[float]]) -> List[Point]:
    """复制折线为独立像素坐标列表（写入 TopoEdge / EdgeTask）。"""
    return _as_point_list(poly)


def edge_pixel_polyline(edge_or_task: Any) -> List[Point]:
    """优先 pixel_polyline / original_polyline / image_polyline，回退 polyline。"""
    for name in ("pixel_polyline", "original_polyline", "image_polyline", "polyline"):
        raw = getattr(edge_or_task, name, None)
        if raw:
            return _as_point_list(raw)
    return []
