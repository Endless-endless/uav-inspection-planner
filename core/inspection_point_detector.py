"""Detect hollow black-ring inspection points from power-line map images."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

try:
    from skimage.color import rgb2hsv
    from skimage.measure import label, regionprops
    from skimage.morphology import binary_closing, binary_dilation, binary_opening, disk
    from skimage.transform import hough_circle, hough_circle_peaks
except ImportError:  # pragma: no cover
    rgb2hsv = None
    label = None
    regionprops = None
    binary_opening = None
    binary_closing = None
    binary_dilation = None
    disk = None
    hough_circle = None
    hough_circle_peaks = None

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

Point = Tuple[float, float]


@dataclass
class InspectionDetectorConfig:
    black_threshold: int = 100
    black_v_threshold: float = 0.38
    min_radius: float = 4.0
    max_radius: float = 16.0
    min_circularity: float = 0.28
    min_ring_score: float = 5.5
    min_area: float = 8.0
    max_area: float = 350.0
    merge_distance: float = 10.0
    morph_close_radius: int = 4
    morph_open_radius: int = 1
    hough_min_dist: int = 12
    hough_threshold: float = 0.22
    ring_validate_ratio: float = 0.32
    snap_threshold: float = 30.0
    real_map_snap_threshold: float = 120.0
    max_bbox_aspect: float = 1.75
    max_extent: float = 0.72
    clean_dilate_radius: int = 10
    real_map_mode: bool = False


DEFAULT_DETECTOR_CONFIG = InspectionDetectorConfig()

# 真实地图：绿色圆点检测在 core.real_map_cv；此配置供 snap 等步骤使用
REAL_MAP_DETECTOR_CONFIG = InspectionDetectorConfig(
    min_radius=5.0,
    max_radius=18.0,
    min_circularity=0.6,
    min_ring_score=0.0,
    min_area=80.0,
    max_area=900.0,
    merge_distance=25.0,
    ring_validate_ratio=0.0,
    real_map_mode=True,
)


def _build_red_mask(hsv: np.ndarray) -> np.ndarray:
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    return ((hue < 0.05) | (hue > 0.94)) & (sat > 0.25) & (val > 0.2)


def _prepare_masks(
    rgb: np.ndarray,
    config: InspectionDetectorConfig,
    red_mask: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gray = np.mean(rgb.astype(np.float32), axis=2)
    if rgb2hsv is None:
        red = np.asarray(red_mask, dtype=bool) if red_mask is not None else np.zeros(rgb.shape[:2], dtype=bool)
        dark = gray < float(config.black_threshold)
        dark &= ~red
        ring = dark.copy()
    else:
        hsv = rgb2hsv(rgb.astype(np.float32) / 255.0)
        red = _build_red_mask(hsv)
        if red_mask is not None:
            red = red | np.asarray(red_mask, dtype=bool)
        dark = (gray < float(config.black_threshold)) | (hsv[:, :, 2] < float(config.black_v_threshold))
        dark &= ~red
        ring = dark.copy()

    if binary_opening is not None and disk is not None:
        ring = binary_opening(ring, disk(max(1, int(config.morph_open_radius))))
        ring = binary_closing(ring, disk(max(1, int(config.morph_close_radius))))
    return gray, red.astype(bool), dark.astype(bool), ring.astype(bool)


def _ring_validate_ratio(
    cx: float,
    cy: float,
    radius: float,
    *,
    gray: np.ndarray,
    red: np.ndarray,
    dark: np.ndarray,
    config: InspectionDetectorConfig,
) -> float:
    h, w = gray.shape[:2]
    hits = 0
    total = 0
    for angle in np.linspace(0.0, 2.0 * np.pi, 28, endpoint=False):
        x = int(round(cx + radius * np.cos(angle)))
        y = int(round(cy + radius * np.sin(angle)))
        if x < 0 or x >= w or y < 0 or y >= h:
            continue
        total += 1
        if dark[y, x] or (gray[y, x] < float(config.black_threshold) + 12 and not red[y, x]):
            hits += 1
    if total <= 0:
        return 0.0
    return hits / total


def _contour_candidates(
    ring_mask: np.ndarray,
    *,
    gray: np.ndarray,
    red: np.ndarray,
    dark: np.ndarray,
    config: InspectionDetectorConfig,
) -> List[Dict[str, Any]]:
    if label is None or regionprops is None:
        return []

    out: List[Dict[str, Any]] = []
    source_tag = "blob" if config.real_map_mode else "contour"
    for region in regionprops(label(ring_mask)):
        area = float(region.area)
        if area < config.min_area or area > config.max_area:
            continue

        min_row, min_col, max_row, max_col = region.bbox
        box_h = max_row - min_row
        box_w = max_col - min_col
        if box_h < 4 or box_w < 4:
            continue

        aspect = max(box_h, box_w) / max(min(box_h, box_w), 1)
        if aspect > config.max_bbox_aspect:
            continue

        extent = area / max(box_h * box_w, 1)
        if extent > config.max_extent:
            continue

        perimeter = float(region.perimeter or 0.0)
        circularity = 4.0 * np.pi * area / (perimeter ** 2 + 1e-6)
        ring_score = perimeter / (np.sqrt(area) + 1e-6)
        if config.real_map_mode:
            if circularity < config.min_circularity:
                continue
        elif ring_score < config.min_ring_score and circularity < config.min_circularity:
            continue

        cy, cx = region.centroid
        radius = 0.25 * (box_h + box_w)
        if radius < config.min_radius or radius > config.max_radius:
            continue

        if config.real_map_mode:
            iy, ix = int(round(cy)), int(round(cx))
            if 0 <= iy < gray.shape[0] and 0 <= ix < gray.shape[1]:
                if float(gray[iy, ix]) >= float(config.black_threshold):
                    continue
                if red[iy, ix]:
                    continue
            validate_ratio = 1.0
        else:
            validate_ratio = _ring_validate_ratio(
                cx, cy, radius, gray=gray, red=red, dark=dark, config=config
            )
            if validate_ratio < config.ring_validate_ratio:
                continue

        if config.real_map_mode:
            confidence = min(1.0, circularity) * 0.7 + min(1.0, extent) * 0.3
        else:
            confidence = min(1.0, ring_score / 12.0) * 0.55 + min(1.0, circularity) * 0.25 + validate_ratio * 0.2
        out.append(
            {
                "coord": [round(float(cx), 2), round(float(cy), 2)],
                "radius": round(float(radius), 2),
                "area": area,
                "circularity": round(float(circularity), 4),
                "ring_score": round(float(ring_score), 3),
                "confidence": round(float(confidence), 4),
                "source": source_tag,
            }
        )
    return out


def _hough_candidates_skimage(
    ring_mask: np.ndarray,
    *,
    gray: np.ndarray,
    red: np.ndarray,
    dark: np.ndarray,
    config: InspectionDetectorConfig,
) -> List[Dict[str, Any]]:
    if hough_circle is None or hough_circle_peaks is None:
        return []

    min_r = max(1, int(np.floor(config.min_radius)))
    max_r = max(min_r + 1, int(np.ceil(config.max_radius)))
    radii = np.arange(min_r, max_r + 1)
    hspaces = hough_circle(ring_mask, radii)
    _, cx, cy, rad = hough_circle_peaks(
        hspaces,
        radii,
        total_num_peaks=60,
        min_xdistance=int(config.hough_min_dist),
        min_ydistance=int(config.hough_min_dist),
        normalize=True,
        threshold=float(config.hough_threshold),
    )

    out: List[Dict[str, Any]] = []
    for x, y, r in zip(cx, cy, rad):
        radius = float(r)
        if radius < config.min_radius or radius > config.max_radius:
            continue
        validate_ratio = _ring_validate_ratio(
            float(x), float(y), radius, gray=gray, red=red, dark=dark, config=config
        )
        if validate_ratio < config.ring_validate_ratio:
            continue
        out.append(
            {
                "coord": [round(float(x), 2), round(float(y), 2)],
                "radius": round(radius, 2),
                "area": round(np.pi * radius * radius * 0.35, 2),
                "circularity": 0.75,
                "ring_score": 0.0,
                "confidence": round(0.65 + validate_ratio * 0.35, 4),
                "source": "hough_skimage",
            }
        )
    return out


def _hough_candidates_cv2(
    gray_u8: np.ndarray,
    *,
    gray: np.ndarray,
    red: np.ndarray,
    dark: np.ndarray,
    config: InspectionDetectorConfig,
) -> List[Dict[str, Any]]:
    if cv2 is None:
        return []

    blur = cv2.GaussianBlur(gray_u8, (5, 5), 1.2)
    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=float(config.hough_min_dist),
        param1=80,
        param2=max(8, int(12 * config.hough_threshold / 0.22)),
        minRadius=max(1, int(np.floor(config.min_radius))),
        maxRadius=max(2, int(np.ceil(config.max_radius))),
    )
    if circles is None:
        return []

    out: List[Dict[str, Any]] = []
    for x, y, r in circles[0]:
        radius = float(r)
        validate_ratio = _ring_validate_ratio(
            float(x), float(y), radius, gray=gray, red=red, dark=dark, config=config
        )
        if validate_ratio < config.ring_validate_ratio:
            continue
        out.append(
            {
                "coord": [round(float(x), 2), round(float(y), 2)],
                "radius": round(radius, 2),
                "area": round(np.pi * radius * radius * 0.35, 2),
                "circularity": 0.75,
                "ring_score": 0.0,
                "confidence": round(0.62 + validate_ratio * 0.38, 4),
                "source": "hough_cv2",
            }
        )
    return out


def _merge_candidates(
    candidates: List[Dict[str, Any]],
    *,
    merge_distance: float,
) -> List[Dict[str, Any]]:
    ranked = sorted(candidates, key=lambda c: float(c.get("confidence", 0.0)), reverse=True)
    merged: List[Dict[str, Any]] = []
    min_dist_sq = float(merge_distance) ** 2
    for cand in ranked:
        cx, cy = cand["coord"]
        if any(
            (cx - other["coord"][0]) ** 2 + (cy - other["coord"][1]) ** 2 < min_dist_sq
            for other in merged
        ):
            continue
        merged.append(cand)
    merged.sort(key=lambda c: (c["coord"][1], c["coord"][0]))
    return merged


def _log_detection_stats(stats: Dict[str, Any]) -> None:
    print("[巡检点检测] 统计:")
    print(f"  - raw contour candidates: {stats.get('contour_candidates', 0)}")
    print(f"  - hough candidates: {stats.get('hough_candidates', 0)}")
    print(f"  - merged inspection points: {stats.get('merged_points', 0)}")
    print(f"  - valid snapped points: {stats.get('valid_snapped_points', 'pending')}")
    print(f"  - invalid points: {stats.get('invalid_points', 'pending')}")


def detect_black_inspection_points(
    image_path: str,
    *,
    config: Optional[InspectionDetectorConfig] = None,
    red_mask: Optional[np.ndarray] = None,
    log_stats: bool = True,
) -> List[Dict[str, Any]]:
    """Detect hollow black-ring inspection points."""
    detections, stats = detect_black_inspection_points_with_stats(
        image_path,
        config=config,
        red_mask=red_mask,
    )
    if log_stats:
        _log_detection_stats(stats)
    return detections


def resolve_detector_config(
    image_path: str,
    config: Optional[InspectionDetectorConfig] = None,
) -> InspectionDetectorConfig:
    if config is not None:
        return config
    try:
        from core.real_map_cv import is_real_map_detection_image

        if is_real_map_detection_image(image_path):
            return REAL_MAP_DETECTOR_CONFIG
    except ImportError:
        pass
    return DEFAULT_DETECTOR_CONFIG


def detect_black_inspection_points_with_stats(
    image_path: str,
    *,
    config: Optional[InspectionDetectorConfig] = None,
    red_mask: Optional[np.ndarray] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    try:
        from core.real_map_cv import is_real_map_detection_image, detect_real_map_inspection_points_with_stats

        if is_real_map_detection_image(image_path):
            detections, stats = detect_real_map_inspection_points_with_stats(
                image_path, red_mask=red_mask
            )
            stats["detector_config"] = REAL_MAP_DETECTOR_CONFIG.__dict__.copy()
            return detections, stats
    except ImportError:
        pass

    cfg = resolve_detector_config(image_path, config)
    rgb = np.array(Image.open(image_path).convert("RGB"))
    gray, red, dark, ring = _prepare_masks(rgb, cfg, red_mask=red_mask)

    contour_input = ring

    contour_raw = _contour_candidates(
        contour_input, gray=gray, red=red, dark=dark, config=cfg
    )
    hough_raw: List[Dict[str, Any]] = []
    if not cfg.real_map_mode:
        hough_raw = _hough_candidates_skimage(ring, gray=gray, red=red, dark=dark, config=cfg)
        hough_raw.extend(
            _hough_candidates_cv2(
                gray.astype(np.uint8),
                gray=gray,
                red=red,
                dark=dark,
                config=cfg,
            )
        )

    merged = _merge_candidates(contour_raw + hough_raw, merge_distance=cfg.merge_distance)
    detections = [
        {
            "id": f"IP_{idx:04d}",
            "coord": item["coord"],
            "radius": item.get("radius"),
            "area": item.get("area"),
            "circularity": item.get("circularity"),
            "confidence": item.get("confidence"),
            "source": item.get("source"),
        }
        for idx, item in enumerate(merged, start=1)
    ]

    stats = {
        "contour_candidates": len(contour_raw),
        "hough_candidates": len(hough_raw),
        "merged_points": len(detections),
        "valid_snapped_points": None,
        "invalid_points": None,
        "detector_config": cfg.__dict__.copy(),
    }
    return detections, stats


def _inpaint_remove_regions(rgb: np.ndarray, remove_mask: np.ndarray) -> np.ndarray:
    """Fill removed ring regions using inpainting or nearest-neighbor fallback."""
    if not remove_mask.any():
        return rgb

    if cv2 is not None:
        inpaint_mask = remove_mask.astype(np.uint8) * 255
        return cv2.inpaint(rgb, inpaint_mask, 5, cv2.INPAINT_TELEA)

    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:
        out = rgb.copy()
        bg = np.median(rgb[~remove_mask].reshape(-1, 3), axis=0).astype(np.uint8)
        out[remove_mask] = bg
        return out

    _, indices = distance_transform_edt(remove_mask, return_distances=True, return_indices=True)
    out = rgb.copy()
    for channel in range(3):
        out[:, :, channel][remove_mask] = rgb[indices[0][remove_mask], indices[1][remove_mask], channel]
    return out


def build_clean_remove_mask(
    rgb: np.ndarray,
    detections: Optional[List[Dict[str, Any]]],
    config: Optional[InspectionDetectorConfig] = None,
) -> np.ndarray:
    cfg = config or DEFAULT_DETECTOR_CONFIG
    h, w = rgb.shape[:2]
    _, _, dark, ring = _prepare_masks(rgb, cfg)
    remove_mask = ring.copy()

    if detections:
        yy, xx = np.ogrid[:h, :w]
        for det in detections:
            coord = det.get("coord") or []
            if len(coord) < 2:
                continue
            cx, cy = float(coord[0]), float(coord[1])
            radius = float(det.get("radius") or 8.0)
            wipe_r = max(radius + 3.0, 8.0)
            remove_mask |= ((xx - cx) ** 2 + (yy - cy) ** 2) <= wipe_r ** 2

    if binary_dilation is not None and disk is not None and remove_mask.any():
        remove_mask = binary_dilation(remove_mask, disk(max(1, int(cfg.clean_dilate_radius))))
    return remove_mask


def generate_clean_map(
    image_path: str,
    *,
    output_path: Optional[str] = None,
    detections: Optional[List[Dict[str, Any]]] = None,
    config: Optional[InspectionDetectorConfig] = None,
    fill_color: Tuple[int, int, int] = (255, 255, 255),
) -> str:
    """Create overlay map with red lines preserved and hollow black rings removed."""
    from pathlib import Path

    cfg = config or DEFAULT_DETECTOR_CONFIG
    rgb = np.array(Image.open(image_path).convert("RGB"))
    remove_mask = build_clean_remove_mask(rgb, detections, cfg)

    clean = _inpaint_remove_regions(rgb, remove_mask)

    src = Path(image_path)
    out = Path(output_path) if output_path else src.parent / f"clean_{src.name}"
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(clean).save(out)
    return str(out.as_posix())


def detect_black_inspection_points_from_array(
    rgb: np.ndarray,
    *,
    red_mask: Optional[np.ndarray] = None,
    config: Optional[InspectionDetectorConfig] = None,
    **kwargs: Any,
) -> List[Dict[str, Any]]:
    tmp_path = None
    try:
        from tempfile import NamedTemporaryFile
        import os

        with NamedTemporaryFile(suffix=".png", delete=False) as tmp:
            tmp_path = tmp.name
            Image.fromarray(rgb.astype(np.uint8)).save(tmp_path)
        return detect_black_inspection_points(
            tmp_path,
            red_mask=red_mask,
            config=config,
            **kwargs,
        )
    finally:
        if tmp_path is not None:
            try:
                import os

                os.unlink(tmp_path)
            except OSError:
                pass
