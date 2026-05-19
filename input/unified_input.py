"""
统一输入接口（Step 1）

为后续接入 GIS / 点云 / pampc 等数据源提供高层任务规划的标准输入结构。
当前不与 TopoGraph、EdgeTask 或主 demo 耦合。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union


# =====================================================
# 数据结构
# =====================================================


@dataclass
class UnifiedLine:
    """单条输电线路（有序折线点列）"""

    id: str
    points: List[Tuple[float, float]]
    voltage_level: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class UnifiedTower:
    """杆塔 / 塔位"""

    id: str
    position: Tuple[float, float]
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class UnifiedWeather:
    """风况与能见度（高层描述，非 MPC 风场）"""

    wind_speed: float = 0.0
    wind_direction: float = 0.0
    visibility: str = "good"


@dataclass
class UnifiedUAV:
    """无人机能力与约束（高层参数）"""

    max_speed: float = 10.0
    battery_capacity: float = 100.0
    safe_distance: float = 5.0


@dataclass
class UnifiedInput:
    """统一输入根对象"""

    lines: List[UnifiedLine] = field(default_factory=list)
    towers: List[UnifiedTower] = field(default_factory=list)
    weather: UnifiedWeather = field(default_factory=UnifiedWeather)
    uav: UnifiedUAV = field(default_factory=UnifiedUAV)
    metadata: Dict[str, Any] = field(default_factory=dict)


# =====================================================
# 序列化辅助
# =====================================================


def _to_point_pair(value: Any, context: str) -> Tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{context}: 坐标必须为 [x, y]")
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context}: 坐标必须为数值") from exc


def _line_from_dict(raw: Dict[str, Any]) -> UnifiedLine:
    if "id" not in raw:
        raise ValueError("lines[] 缺少 id")
    if "points" not in raw:
        raise ValueError(f"lines[{raw.get('id', '?')}] 缺少 points")

    points_raw = raw["points"]
    if not isinstance(points_raw, list):
        raise ValueError(f"lines[{raw['id']}] points 必须为数组")

    points = [
        _to_point_pair(p, f"lines[{raw['id']}].points[{i}]")
        for i, p in enumerate(points_raw)
    ]

    metadata = raw.get("metadata", {})
    if metadata is not None and not isinstance(metadata, dict):
        raise ValueError(f"lines[{raw['id']}] metadata 必须为对象")

    return UnifiedLine(
        id=str(raw["id"]),
        points=points,
        voltage_level=raw.get("voltage_level"),
        metadata=dict(metadata or {}),
    )


def _tower_from_dict(raw: Dict[str, Any]) -> UnifiedTower:
    if "id" not in raw:
        raise ValueError("towers[] 缺少 id")
    if "position" not in raw:
        raise ValueError(f"towers[{raw.get('id', '?')}] 缺少 position")

    metadata = raw.get("metadata", {})
    if metadata is not None and not isinstance(metadata, dict):
        raise ValueError(f"towers[{raw['id']}] metadata 必须为对象")

    return UnifiedTower(
        id=str(raw["id"]),
        position=_to_point_pair(raw["position"], f"towers[{raw['id']}].position"),
        metadata=dict(metadata or {}),
    )


def _weather_from_dict(raw: Dict[str, Any]) -> UnifiedWeather:
    if not isinstance(raw, dict):
        raise ValueError("weather 必须为对象")
    return UnifiedWeather(
        wind_speed=float(raw.get("wind_speed", 0.0)),
        wind_direction=float(raw.get("wind_direction", 0.0)),
        visibility=str(raw.get("visibility", "good")),
    )


def _uav_from_dict(raw: Dict[str, Any]) -> UnifiedUAV:
    if not isinstance(raw, dict):
        raise ValueError("uav 必须为对象")
    return UnifiedUAV(
        max_speed=float(raw.get("max_speed", 10.0)),
        battery_capacity=float(raw.get("battery_capacity", 100.0)),
        safe_distance=float(raw.get("safe_distance", 5.0)),
    )


def unified_input_from_dict(data: Dict[str, Any]) -> UnifiedInput:
    """从字典构建 UnifiedInput（供 JSON 加载与测试使用）"""
    if not isinstance(data, dict):
        raise ValueError("根对象必须为 JSON 对象")

    lines_raw = data.get("lines", [])
    if not isinstance(lines_raw, list):
        raise ValueError("lines 必须为数组")

    towers_raw = data.get("towers", [])
    if not isinstance(towers_raw, list):
        raise ValueError("towers 必须为数组")

    root_metadata = data.get("metadata", {})
    if root_metadata is not None and not isinstance(root_metadata, dict):
        raise ValueError("metadata 必须为对象")

    return UnifiedInput(
        lines=[_line_from_dict(item) for item in lines_raw],
        towers=[_tower_from_dict(item) for item in towers_raw],
        weather=_weather_from_dict(data.get("weather", {})),
        uav=_uav_from_dict(data.get("uav", {})),
        metadata=dict(root_metadata or {}),
    )


def unified_input_to_dict(data: UnifiedInput) -> Dict[str, Any]:
    """将 UnifiedInput 转为可 JSON 序列化的字典"""

    def _line_to_dict(line: UnifiedLine) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "id": line.id,
            "points": [[x, y] for x, y in line.points],
            "metadata": line.metadata,
        }
        if line.voltage_level is not None:
            out["voltage_level"] = line.voltage_level
        return out

    def _tower_to_dict(tower: UnifiedTower) -> Dict[str, Any]:
        x, y = tower.position
        return {
            "id": tower.id,
            "position": [x, y],
            "metadata": tower.metadata,
        }

    result: Dict[str, Any] = {
        "lines": [_line_to_dict(line) for line in data.lines],
        "towers": [_tower_to_dict(tower) for tower in data.towers],
        "weather": asdict(data.weather),
        "uav": asdict(data.uav),
    }
    if data.metadata:
        result["metadata"] = data.metadata
    return result


# =====================================================
# 公开 API
# =====================================================


def load_unified_input_from_json(path: Union[str, Path]) -> UnifiedInput:
    """从 JSON 文件加载统一输入"""
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        raw = json.load(f)
    data = unified_input_from_dict(raw)
    errors = validate_unified_input(data)
    if errors:
        raise ValueError(
            f"统一输入校验失败 ({path}):\n" + "\n".join(f"  - {e}" for e in errors)
        )
    return data


def save_unified_input_to_json(data: UnifiedInput, path: Union[str, Path]) -> None:
    """将统一输入保存为 JSON 文件"""
    errors = validate_unified_input(data)
    if errors:
        raise ValueError(
            "统一输入校验失败，无法保存:\n" + "\n".join(f"  - {e}" for e in errors)
        )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(unified_input_to_dict(data), f, ensure_ascii=False, indent=2)
        f.write("\n")


def validate_unified_input(data: UnifiedInput) -> List[str]:
    """
    校验统一输入结构与张量约束。

    Returns:
        错误信息列表；空列表表示通过。
    """
    errors: List[str] = []

    if not isinstance(data, UnifiedInput):
        errors.append("data 必须为 UnifiedInput 实例")
        return errors

    if not data.lines:
        errors.append("lines 不能为空")

    seen_line_ids: set = set()
    for line in data.lines:
        if not line.id or not str(line.id).strip():
            errors.append("lines 中存在空 id")
            continue
        if line.id in seen_line_ids:
            errors.append(f"lines 中 id 重复: {line.id}")
        seen_line_ids.add(line.id)

        if len(line.points) < 2:
            errors.append(f"lines[{line.id}] 至少需要 2 个点")
        for i, (x, y) in enumerate(line.points):
            if not _is_finite(x) or not _is_finite(y):
                errors.append(f"lines[{line.id}].points[{i}] 坐标必须为有限数值")

    seen_tower_ids: set = set()
    for tower in data.towers:
        if not tower.id or not str(tower.id).strip():
            errors.append("towers 中存在空 id")
            continue
        if tower.id in seen_tower_ids:
            errors.append(f"towers 中 id 重复: {tower.id}")
        seen_tower_ids.add(tower.id)

        x, y = tower.position
        if not _is_finite(x) or not _is_finite(y):
            errors.append(f"towers[{tower.id}].position 坐标必须为有限数值")

    w = data.weather
    if w.wind_speed < 0:
        errors.append("weather.wind_speed 不能为负")
    if not (0.0 <= w.wind_direction <= 360.0):
        errors.append("weather.wind_direction 应在 [0, 360] 范围内")
    if not w.visibility or not str(w.visibility).strip():
        errors.append("weather.visibility 不能为空")

    u = data.uav
    if u.max_speed <= 0:
        errors.append("uav.max_speed 必须大于 0")
    if u.battery_capacity <= 0:
        errors.append("uav.battery_capacity 必须大于 0")
    if u.safe_distance < 0:
        errors.append("uav.safe_distance 不能为负")

    return errors


def _is_finite(value: float) -> bool:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False
    return v == v and abs(v) != float("inf")
