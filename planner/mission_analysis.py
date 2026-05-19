"""
Mission 统计与分析（第六阶段）

用于 baseline / optimized 对比，核心指标为 connect_ratio。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional


def analyze_mission(mission) -> Dict[str, Any]:
    """
    分析 GroupedContinuousMission（或兼容对象）。

    Returns:
        统计字典，含 connect_ratio、visit_order 等。
    """
    total = float(getattr(mission, "total_length", 0.0) or 0.0)
    inspect_len = float(getattr(mission, "inspect_length", 0.0) or 0.0)
    connect_len = float(getattr(mission, "connect_length", 0.0) or 0.0)

    connect_ratio = (connect_len / total) if total > 0 else 0.0
    inspect_ratio = (inspect_len / total) if total > 0 else 0.0

    groups = getattr(mission, "groups", {}) or {}
    visit_order: List[str] = list(getattr(mission, "visit_order", []) or [])
    group_visit_order: List[str] = list(getattr(mission, "group_visit_order", []) or [])
    segments = getattr(mission, "segments", []) or []

    num_inspect_segments = sum(1 for s in segments if getattr(s, "type", None) == "inspect")
    num_connect_segments = sum(1 for s in segments if getattr(s, "type", None) == "connect")

    group_sizes = [len(g.edge_ids) for g in groups.values()] if groups else []
    avg_group_size = (
        sum(group_sizes) / len(group_sizes) if group_sizes else 0.0
    )

    return {
        "num_groups": len(groups),
        "num_segments": len(segments),
        "num_inspect_segments": num_inspect_segments,
        "num_connect_segments": num_connect_segments,
        "total_length": round(total, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "connect_ratio": round(connect_ratio, 4),
        "inspect_ratio": round(inspect_ratio, 4),
        "avg_group_size": round(avg_group_size, 2),
        "visit_order": visit_order,
        "group_visit_order": group_visit_order,
        "num_edges": len(visit_order),
    }


def analyze_mission_json(data: Dict[str, Any]) -> Dict[str, Any]:
    """从已导出的 mission_output.json 提取分析指标。"""
    stats = data.get("statistics", {})
    visit = data.get("visit_order", {})
    total = float(stats.get("total_length", 0.0))
    connect_len = float(stats.get("connect_length", 0.0))
    inspect_len = float(stats.get("inspect_length", 0.0))
    connect_ratio = (connect_len / total) if total > 0 else 0.0

    return {
        "num_groups": stats.get("num_groups", 0),
        "num_segments": stats.get("num_segments", 0),
        "num_inspect_segments": None,
        "num_connect_segments": None,
        "total_length": round(total, 2),
        "inspect_length": round(inspect_len, 2),
        "connect_length": round(connect_len, 2),
        "connect_ratio": round(connect_ratio, 4),
        "inspect_ratio": round(
            (inspect_len / total) if total > 0 else 0.0, 4
        ),
        "avg_group_size": None,
        "visit_order": visit.get("edge_visit_order", []),
        "group_visit_order": visit.get("group_visit_order", []),
        "num_edges": stats.get("num_edges", len(visit.get("edge_visit_order", []))),
    }


def compare_mission_stats(
    baseline: Dict[str, Any], optimized: Dict[str, Any]
) -> Dict[str, Any]:
    """计算 optimized 相对 baseline 的改进量。"""
    def _delta(key: str, lower_is_better: bool = True) -> Dict[str, Any]:
        b = float(baseline.get(key, 0))
        o = float(optimized.get(key, 0))
        diff = o - b
        pct = (diff / b * 100.0) if abs(b) > 1e-9 else 0.0
        improved = (diff < 0) if lower_is_better else (diff > 0)
        return {
            "baseline": b,
            "optimized": o,
            "delta": round(diff, 4),
            "delta_percent": round(pct, 2),
            "improved": improved,
        }

    return {
        "total_length": _delta("total_length", lower_is_better=True),
        "connect_length": _delta("connect_length", lower_is_better=True),
        "connect_ratio": _delta("connect_ratio", lower_is_better=True),
        "inspect_length": _delta("inspect_length", lower_is_better=False),
        "num_segments": _delta("num_segments", lower_is_better=True),
        "visit_order_changed": baseline.get("visit_order") != optimized.get("visit_order"),
    }
