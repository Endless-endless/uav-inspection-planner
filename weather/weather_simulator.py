from __future__ import annotations

import copy
import math
from typing import Any, Dict, List, Optional, Sequence


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class DynamicWeatherSimulator:
    """Small deterministic simulator for moving and evolving weather zones."""

    def __init__(
        self,
        weather_zones: Sequence[Dict[str, Any]],
        *,
        bounds: Optional[Dict[str, Sequence[float]]] = None,
    ) -> None:
        self._initial = [copy.deepcopy(z) for z in weather_zones]
        self.zones = [copy.deepcopy(z) for z in weather_zones]
        self.elapsed = 0.0
        self.bounds = bounds or {"x_range": [0.0, 1000.0], "y_range": [1000.0, 0.0]}

    def reset(self) -> List[Dict[str, Any]]:
        self.elapsed = 0.0
        self.zones = [copy.deepcopy(z) for z in self._initial]
        return copy.deepcopy(self.zones)

    def update_weather_zones(self, dt: float) -> List[Dict[str, Any]]:
        self.elapsed += max(0.0, float(dt))
        self.zones = update_weather_zones(
            self.zones,
            dt,
            elapsed=self.elapsed,
            bounds=self.bounds,
        )
        return copy.deepcopy(self.zones)


def update_weather_zones(
    weather_zones: Sequence[Dict[str, Any]],
    dt: float,
    *,
    elapsed: float = 0.0,
    bounds: Optional[Dict[str, Sequence[float]]] = None,
) -> List[Dict[str, Any]]:
    """Advance dynamic weather zones by dt seconds.

    Updates center, radius, and severity. Static zones pass through unchanged.
    """
    dt = max(0.0, float(dt))
    out: List[Dict[str, Any]] = []
    for zone in weather_zones:
        z = copy.deepcopy(zone)
        if not z.get("dynamic"):
            out.append(z)
            continue

        lifetime = float(z.get("lifetime", 0.0) or 0.0)
        if lifetime > 0 and elapsed > lifetime:
            phase = (elapsed - lifetime) / max(lifetime, 1.0)
            z["severity"] = _clamp(float(z.get("severity", 0.0) or 0.0) * math.exp(-phase), 0.0, 1.0)
            if z["severity"] <= 0.03:
                continue

        center = z.get("center") or [0.0, 0.0]
        velocity = z.get("velocity") or [0.0, 0.0]
        vx = float(velocity[0]) if len(velocity) >= 1 else 0.0
        vy = float(velocity[1]) if len(velocity) >= 2 else 0.0
        z["center"] = [float(center[0]) + vx * dt, float(center[1]) + vy * dt]
        z["radius"] = max(10.0, float(z.get("radius", 0.0) or 0.0) + float(z.get("expand_rate", 0.0) or 0.0) * dt)
        z["severity"] = _clamp(
            float(z.get("severity", 0.0) or 0.0) + float(z.get("severity_rate", 0.0) or 0.0) * dt,
            0.0,
            1.0,
        )

        if bounds:
            _bounce_zone(z, bounds)
        out.append(z)
    return out


def _bounce_zone(zone: Dict[str, Any], bounds: Dict[str, Sequence[float]]) -> None:
    x_range = bounds.get("x_range") or [0.0, 1000.0]
    y_range = bounds.get("y_range") or [1000.0, 0.0]
    xmin, xmax = min(float(x_range[0]), float(x_range[1])), max(float(x_range[0]), float(x_range[1]))
    ymin, ymax = min(float(y_range[0]), float(y_range[1])), max(float(y_range[0]), float(y_range[1]))
    center = zone.get("center") or [0.0, 0.0]
    velocity = list(zone.get("velocity") or [0.0, 0.0])
    if len(velocity) < 2:
        velocity = [0.0, 0.0]
    x, y = float(center[0]), float(center[1])
    if x < xmin or x > xmax:
        velocity[0] = -float(velocity[0])
        x = _clamp(x, xmin, xmax)
    if y < ymin or y > ymax:
        velocity[1] = -float(velocity[1])
        y = _clamp(y, ymin, ymax)
    zone["center"] = [x, y]
    zone["velocity"] = velocity
