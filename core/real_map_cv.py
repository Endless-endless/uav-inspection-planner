"""真实卫星标注图 CV：红线（电网）+ 绿色实心圆（巡检点）及调试输出。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
from PIL import Image, ImageDraw

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from skimage.measure import label, regionprops
except ImportError:  # pragma: no cover
    label = None
    regionprops = None

REAL_MAP_CANONICAL_IMAGE = "data/chengdu_real_point.png"
REAL_MAP_MARKERS = ("chengdu_real_point", "chengdu_real_line")

DEBUG_DIR = Path("result/debug")

# OpenCV HSV: H 0–180, S/V 0–255
RED_HSV_LOWER1 = (0, 120, 120)
RED_HSV_UPPER1 = (10, 255, 255)
RED_HSV_LOWER2 = (170, 120, 120)
RED_HSV_UPPER2 = (180, 255, 255)

GREEN_HSV_LOWER = (45, 120, 80)
GREEN_HSV_UPPER = (85, 255, 255)

RED_MIN_COMPONENT_AREA = 300
GREEN_MIN_AREA = 80
GREEN_MAX_AREA = 900
GREEN_MIN_CIRCULARITY = 0.6
GREEN_MERGE_DISTANCE = 25.0
GREEN_MAX_DIST_TO_RED_PX = 20.0
REAL_MAP_MIN_POLYLINE_LENGTH_PX = 80.0


def canonical_real_map_image_path(path: Optional[str] = None) -> str:
    """识别与显示统一使用标注点图。"""
    if path and is_real_map_detection_image(path):
        p = str(path).replace("\\", "/")
        if "chengdu_real_point" in p.lower():
            return REAL_MAP_CANONICAL_IMAGE
        return p
    return REAL_MAP_CANONICAL_IMAGE


def is_real_map_detection_image(path: str) -> bool:
    p = str(path or "").replace("\\", "/").lower()
    return any(m in p for m in REAL_MAP_MARKERS)


def _ensure_debug_dir() -> Path:
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    return DEBUG_DIR


def _filter_small_components_cv2(mask: np.ndarray, min_area: int) -> np.ndarray:
    if cv2 is None:
        return mask
    n, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def extract_real_map_red_mask(rgb: np.ndarray) -> np.ndarray:
    """高饱和红色双区间 + close + dilate；剔除小连通域（<300px）。"""
    if rgb.dtype != np.uint8:
        rgb_u8 = np.clip(rgb, 0, 255).astype(np.uint8)
    else:
        rgb_u8 = rgb

    if cv2 is not None:
        bgr = cv2.cvtColor(rgb_u8, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array(RED_HSV_LOWER1), np.array(RED_HSV_UPPER1))
        m2 = cv2.inRange(hsv, np.array(RED_HSV_LOWER2), np.array(RED_HSV_UPPER2))
        mask = cv2.bitwise_or(m1, m2)
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        k_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)
        mask = cv2.dilate(mask, k_dilate, iterations=1)
        return _filter_small_components_cv2(mask, RED_MIN_COMPONENT_AREA)

    from skimage.color import rgb2hsv

    hsv = rgb2hsv(rgb_u8.astype(np.float32) / 255.0)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    m1 = (h <= 10 / 180.0) & (s >= 120 / 255.0) & (v >= 120 / 255.0)
    m2 = (h >= 170 / 180.0) & (s >= 120 / 255.0) & (v >= 120 / 255.0)
    mask = ((m1 | m2).astype(np.uint8)) * 255
    return morph_close_dilate_red_mask(mask)


def morph_close_dilate_red_mask(mask: np.ndarray) -> np.ndarray:
    m = (mask > 0).astype(np.uint8) * 255
    if cv2 is not None:
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        k_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_close, iterations=2)
        m = cv2.dilate(m, k_dilate, iterations=1)
        return _filter_small_components_cv2(m, RED_MIN_COMPONENT_AREA)
    try:
        from skimage.morphology import binary_closing, binary_dilation, disk

        bool_m = m > 0
        bool_m = binary_closing(bool_m, disk(2))
        bool_m = binary_dilation(bool_m, disk(1))
        m = bool_m.astype(np.uint8) * 255
    except ImportError:
        pass
    return _filter_small_components_cv2(m, RED_MIN_COMPONENT_AREA)


def extract_real_map_green_mask(
    rgb: np.ndarray,
    red_mask: Optional[np.ndarray] = None,
) -> np.ndarray:
    """绿色实心圆 mask（排除红线像素）。"""
    if rgb.dtype != np.uint8:
        rgb_u8 = np.clip(rgb, 0, 255).astype(np.uint8)
    else:
        rgb_u8 = rgb

    if cv2 is not None:
        bgr = cv2.cvtColor(rgb_u8, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(GREEN_HSV_LOWER), np.array(GREEN_HSV_UPPER))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    else:
        from skimage.color import rgb2hsv
        from skimage.morphology import binary_closing, binary_opening, disk

        hsv = rgb2hsv(rgb_u8.astype(np.float32) / 255.0)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        lo_h, hi_h = 45 / 180.0, 85 / 180.0
        green = (h >= lo_h) & (h <= hi_h) & (s >= 120 / 255.0) & (v >= 80 / 255.0)
        mask = (green.astype(np.uint8)) * 255
        bool_g = mask > 0
        bool_g = binary_opening(bool_g, disk(1))
        bool_g = binary_closing(bool_g, disk(2))
        mask = bool_g.astype(np.uint8) * 255

    if red_mask is not None:
        mask = mask.copy()
        mask[np.asarray(red_mask) > 0] = 0

    return _filter_green_components(mask)


def _filter_green_components(mask: np.ndarray) -> np.ndarray:
    if cv2 is None:
        return mask
    n, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if GREEN_MIN_AREA <= area <= GREEN_MAX_AREA:
            out[labels == i] = 255
    return out


def _red_distance_field(red_mask: np.ndarray) -> Optional[np.ndarray]:
    """每个像素到最近红线像素的欧氏距离。"""
    if cv2 is None:
        return None
    inv = (np.asarray(red_mask) == 0).astype(np.uint8)
    return cv2.distanceTransform(inv, cv2.DIST_L2, 3)


def _distance_to_red(cx: float, cy: float, dist_field: Optional[np.ndarray]) -> float:
    if dist_field is None:
        return 0.0
    h, w = dist_field.shape[:2]
    ix = int(round(cx))
    iy = int(round(cy))
    if ix < 0 or iy < 0 or ix >= w or iy >= h:
        return 1e9
    return float(dist_field[iy, ix])


def count_mask_components(mask: np.ndarray, min_area: int = 1) -> int:
    if cv2 is None:
        if label is None:
            return 0
        return len(regionprops(label(mask > 0)))
    n, _, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
    return sum(1 for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_area)


def detect_green_inspection_points(
    rgb: np.ndarray,
    *,
    red_mask: Optional[np.ndarray] = None,
    merge_distance: float = GREEN_MERGE_DISTANCE,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any], np.ndarray]:
    """绿色实心圆 → 巡检点（原图像素坐标，须靠近红线）。"""
    if red_mask is None:
        red_mask = extract_real_map_red_mask(rgb)
    green_mask = extract_real_map_green_mask(rgb, red_mask=red_mask)
    dist_field = _red_distance_field(red_mask)
    candidates: List[Dict[str, Any]] = []

    if label is None or regionprops is None:
        return [], {"contour_candidates": 0, "green_point_count": 0}, green_mask

    for region in regionprops(label(green_mask > 0)):
        area = float(region.area)
        if area < GREEN_MIN_AREA or area > GREEN_MAX_AREA:
            continue
        min_row, min_col, max_row, max_col = region.bbox
        box_h = max_row - min_row
        box_w = max_col - min_col
        if box_h < 4 or box_w < 4:
            continue
        perimeter = float(region.perimeter or 0.0)
        circularity = 4.0 * np.pi * area / (perimeter ** 2 + 1e-6)
        if circularity < GREEN_MIN_CIRCULARITY:
            continue
        cy, cx = region.centroid
        if _distance_to_red(float(cx), float(cy), dist_field) > GREEN_MAX_DIST_TO_RED_PX:
            continue
        radius = 0.25 * (box_h + box_w)
        candidates.append(
            {
                "coord": [round(float(cx), 2), round(float(cy), 2)],
                "radius": round(float(radius), 2),
                "area": area,
                "circularity": round(float(circularity), 4),
                "confidence": round(min(1.0, circularity) * 0.85 + min(1.0, area / 400.0) * 0.15, 4),
                "source": "green_blob",
            }
        )

    merged = _merge_point_candidates(candidates, merge_distance)
    detections = [
        {
            "id": f"IP_{idx:04d}",
            "coord": item["coord"],
            "radius": item.get("radius"),
            "area": item.get("area"),
            "circularity": item.get("circularity"),
            "confidence": item.get("confidence"),
            "source": item.get("source", "green_blob"),
            "point_type": "image_detected",
        }
        for idx, item in enumerate(merged, start=1)
    ]

    stats = {
        "contour_candidates": len(candidates),
        "hough_candidates": 0,
        "merged_points": len(detections),
        "green_point_count": len(detections),
        "green_mask_pixels": int(np.sum(green_mask > 0)),
        "detector": "green_hsv",
    }
    return detections, stats, green_mask


def detect_real_map_inspection_points_with_stats(
    image_path: str,
    *,
    red_mask: Optional[np.ndarray] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    image_path = canonical_real_map_image_path(image_path)
    rgb = np.array(Image.open(image_path).convert("RGB"))
    h, w = rgb.shape[:2]
    if red_mask is None:
        red_mask = extract_real_map_red_mask(rgb)
    detections, stats, green_mask = detect_green_inspection_points(rgb, red_mask=red_mask)
    stats["image_width"] = w
    stats["image_height"] = h
    stats["image_path"] = image_path
    stats["red_mask_pixels"] = int(np.sum(red_mask > 0))
    stats["red_components"] = count_mask_components(red_mask, RED_MIN_COMPONENT_AREA)
    stats["green_components"] = count_mask_components(green_mask, GREEN_MIN_AREA)
    return detections, stats


def filter_independent_lines_for_real_map(lines: List[Any]) -> List[Any]:
    """剔除骨架折线长度 < 80px 的噪声线路（仅真实地图）。"""
    kept: List[Any] = []
    dropped = 0
    for line in lines or []:
        length = float(getattr(line, "length_2d", 0.0) or 0.0)
        if length <= 0 and getattr(line, "ordered_pixels", None):
            pts = line.ordered_pixels
            for i in range(1, len(pts)):
                x1, y1 = pts[i - 1]
                x2, y2 = pts[i]
                length += float(np.hypot(x2 - x1, y2 - y1))
            line.length_2d = length
        if length >= REAL_MAP_MIN_POLYLINE_LENGTH_PX:
            kept.append(line)
        else:
            dropped += 1
    if dropped:
        print(f"  [真实地图] 过滤短线段: {dropped} 条 (<{REAL_MAP_MIN_POLYLINE_LENGTH_PX}px)")
    return kept


def _merge_point_candidates(
    candidates: List[Dict[str, Any]],
    merge_distance: float,
) -> List[Dict[str, Any]]:
    ranked = sorted(candidates, key=lambda c: float(c.get("confidence", 0.0)), reverse=True)
    merged: List[Dict[str, Any]] = []
    for cand in ranked:
        cx, cy = cand["coord"]
        keep = True
        for existing in merged:
            ex, ey = existing["coord"]
            if (cx - ex) ** 2 + (cy - ey) ** 2 <= merge_distance ** 2:
                keep = False
                break
        if keep:
            merged.append(cand)
    merged.sort(key=lambda c: (c["coord"][1], c["coord"][0]))
    return merged


def save_red_mask(red_mask: np.ndarray) -> str:
    _ensure_debug_dir()
    out = DEBUG_DIR / "red_mask.png"
    Image.fromarray(red_mask).save(out)
    return str(out)


def save_green_mask(green_mask: np.ndarray) -> str:
    _ensure_debug_dir()
    out = DEBUG_DIR / "green_mask.png"
    Image.fromarray(green_mask).save(out)
    return str(out)


def save_detected_overlay(
    image_path: str,
    red_mask: np.ndarray,
    green_mask: np.ndarray,
    detections: List[Dict[str, Any]],
    *,
    polylines: Optional[List[List[Tuple[float, float]]]] = None,
) -> str:
    _ensure_debug_dir()
    image_path = canonical_real_map_image_path(image_path)
    rgb = np.array(Image.open(image_path).convert("RGB"))
    overlay = Image.fromarray(rgb).convert("RGBA")

    line_layer = Image.new("RGBA", overlay.size, (0, 120, 255, 0))
    line_alpha = Image.fromarray(red_mask).point(lambda p: 140 if p > 0 else 0)
    overlay = Image.composite(line_layer, overlay, line_alpha)

    draw = ImageDraw.Draw(overlay)
    if polylines:
        for poly in polylines:
            if len(poly) >= 2:
                draw.line(
                    [(float(x), float(y)) for x, y in poly],
                    fill=(0, 200, 255, 230),
                    width=2,
                )
    for det in detections:
        coord = det.get("coord") or det.get("pixel_position")
        if not coord or len(coord) < 2:
            continue
        cx, cy = float(coord[0]), float(coord[1])
        r = max(float(det.get("radius") or 8.0), 7.0)
        draw.ellipse(
            (cx - r, cy - r, cx + r, cy + r),
            outline=(0, 255, 100, 255),
            width=3,
        )
        draw.ellipse((cx - 3, cy - 3, cx + 3, cy + 3), fill=(255, 220, 0, 255))
        label = det.get("id") or ""
        if label:
            draw.text((cx + r + 2, cy - 6), str(label), fill=(255, 255, 255, 255))

    out = DEBUG_DIR / "detected_overlay.png"
    overlay.convert("RGB").save(out)
    return str(out)


def save_real_map_detection_debug(
    image_path: str,
    red_mask: np.ndarray,
    green_mask: np.ndarray,
    detections: List[Dict[str, Any]],
    *,
    polylines: Optional[List[List[Tuple[float, float]]]] = None,
    line_count: int = 0,
    stats: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    paths = {
        "red_mask": save_red_mask(red_mask),
        "green_mask": save_green_mask(green_mask),
        "detected_overlay": save_detected_overlay(
            image_path, red_mask, green_mask, detections, polylines=polylines
        ),
    }
    st = stats or {}
    gcnt = st.get("green_point_count", st.get("merged_points", len(detections)))
    print("[真实地图 CV 调试]")
    print(f"  - image size: {st.get('image_width')} x {st.get('image_height')}")
    print(f"  - red components: {st.get('red_components', count_mask_components(red_mask, RED_MIN_COMPONENT_AREA))}")
    print(f"  - extracted lines: {line_count}")
    print(f"  - green_point_count: {gcnt}")
    print(f"  - green components (raw): {st.get('green_components', count_mask_components(green_mask, GREEN_MIN_AREA))}")
    print(f"  - debug files: {paths}")
    return paths


# 兼容旧文件名
def save_chengdu_red_mask(red_mask: np.ndarray) -> str:
    return save_red_mask(red_mask)


def save_chengdu_green_mask(green_mask: np.ndarray) -> str:
    return save_green_mask(green_mask)


def save_chengdu_detected_overlay(*args, **kwargs) -> str:
    return save_detected_overlay(*args, **kwargs)


def save_chengdu_detection_debug(*args, **kwargs):
    return save_real_map_detection_debug(*args, **kwargs)


def save_red_mask_debug(red_mask: np.ndarray) -> str:
    return save_red_mask(red_mask)


def build_real_map_black_mask(*_args, **_kwargs) -> np.ndarray:
    return np.zeros((1, 1), dtype=np.uint8)
