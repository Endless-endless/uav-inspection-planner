"""
Web Dashboard 真实地图底图配置。

底图通过 Plotly layout.images 叠加，y 轴向下（range [height, 0]）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

try:
    from PIL import Image
except ImportError:
    Image = None

DEFAULT_MAP_REL = "data/test.png"


def resolve_map_path(root: Path, map_rel: str = DEFAULT_MAP_REL) -> Path:
    p = Path(map_rel)
    if not p.is_absolute():
        p = root / p
    return p.resolve()


def get_background_map_config(
    root: Path,
    map_rel: str = DEFAULT_MAP_REL,
    image_url: Optional[str] = None,
) -> Dict[str, Any]:
    """
    返回 Dashboard 可用的底图元数据（不含 base64，由前端通过 URL 加载）。

    layout.images 参数与 Dashboard Plotly 底图一致。
    """
    map_path = resolve_map_path(root, map_rel)
    rel_norm = str(map_rel).replace("\\", "/")
    if image_url is None:
        image_url = f"/{rel_norm}" if rel_norm.startswith("data/") else f"/api/map/background?path={map_rel}"
    if not map_path.exists():
        return {
            "available": False,
            "path": str(map_rel),
            "url": image_url,
            "width": 0,
            "height": 0,
            "layout_image": None,
            "axis": {"x_range": [0, 1], "y_range": [1, 0]},
        }

    width, height = 916, 960
    if Image is not None:
        with Image.open(map_path) as im:
            width, height = im.size

    layout_image = {
        "source": image_url,
        "xref": "x",
        "yref": "y",
        "x": 0,
        "y": height,
        "sizex": width,
        "sizey": height,
        "sizing": "stretch",
        "layer": "below",
        "opacity": 1.0,
    }

    return {
        "available": True,
        "path": str(map_path.relative_to(root)) if map_path.is_relative_to(root) else str(map_rel),
        "url": image_url,
        "width": width,
        "height": height,
        "layout_image": layout_image,
        "axis": {
            "x_range": [0, width],
            "y_range": [height, 0],
        },
        "coordinate_system": "image_upper_left_y_down",
    }


def attach_map_to_dashboard_payload(
    payload: Dict[str, Any],
    root: Path,
    map_rel: str = DEFAULT_MAP_REL,
) -> Dict[str, Any]:
    """在 Dashboard JSON 中附加 map_background 字段。"""
    payload["map_background"] = get_background_map_config(root, map_rel)
    meta = payload.setdefault("metadata", {})
    meta["default_map_mode"] = "image_overlay"
    meta["map_image"] = DEFAULT_MAP_REL
    return payload
