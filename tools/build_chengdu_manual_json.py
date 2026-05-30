"""
一次性工具：从人工绘制的 line/point 叠加图导出 chengdu_real_manual.json。

仅用于生成权威 JSON 文件；Dashboard 运行时不再执行此脚本。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PIL import Image
import numpy as np

from core.independent_lines import extract_independent_lines_from_skeleton
from core.inspection_point_detector import detect_black_inspection_points_with_stats


def _red_mask(arr: np.ndarray) -> np.ndarray:
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    return ((r > 150) & (g < 100) & (b < 100)).astype(np.uint8) * 255


def main() -> None:
    line_path = ROOT / "data" / "chengdu_real_line.png"
    point_path = ROOT / "data" / "chengdu_real_point.png"
    sat_path = ROOT / "data" / "chengdu_real_satellite.png"
    out_path = ROOT / "data" / "chengdu_real_manual.json"

    with Image.open(sat_path) as im:
        width, height = im.size

    line_img = np.array(Image.open(line_path).convert("RGB"))
    mask = _red_mask(line_img)
    lines = extract_independent_lines_from_skeleton(mask, min_pixels=30)

    point_img = str(point_path)
    detections, stats = detect_black_inspection_points_with_stats(point_img)

    line_polys = {
        ln.id: ln.ordered_pixels
        for ln in lines
    }

    def assign_line_id(xy):
        from core.topo_task import project_point_to_polyline

        best_id = None
        best_d = 1e9
        for lid, poly in line_polys.items():
            proj = project_point_to_polyline(xy, poly)
            if proj and float(proj["distance"]) < best_d:
                best_d = float(proj["distance"])
                best_id = lid
        return best_id if best_d <= 80 else None

    inspection_points = []
    for i, det in enumerate(detections):
        coord = det.get("coord") or det.get("pixel_position")
        if not coord:
            continue
        xy = [round(float(coord[0]), 1), round(float(coord[1]), 1)]
        lid = assign_line_id(xy)
        inspection_points.append(
            {
                "id": det.get("id") or f"IP_{i + 1:04d}",
                "position": xy,
                "line_id": lid,
            }
        )

    payload = {
        "dataset_type": "real_satellite_manual",
        "clean_map_image": "data/chengdu_real_satellite.png",
        "image_width": width,
        "image_height": height,
        "lines": [
            {
                "id": ln.id,
                "polyline": [[int(x), int(y)] for x, y in ln.ordered_pixels],
            }
            for ln in lines
        ],
        "inspection_points": inspection_points,
        "export_note": "Generated from chengdu_real_line.png + chengdu_real_point.png overlay (authoring only)",
        "detection_stats": stats,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    print(f"Wrote {out_path}")
    print(f"  lines: {len(payload['lines'])}")
    print(f"  inspection_points: {len(payload['inspection_points'])}")


if __name__ == "__main__":
    main()
