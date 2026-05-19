"""
统一输入 → Mission 桥接层（第五阶段）

复用 plan_global_topology_optimized_mission 与 export_grouped_mission_to_json，
不修改 GroupedContinuousMission / 规划算法实现。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from core.topo_plan import export_grouped_mission_to_json
from core.topo_global_optimizer import plan_global_topology_optimized_mission
from core.topo_task import EdgeTask
from input.unified_input import UnifiedInput
from input.unified_task_adapter import unified_input_to_edge_tasks

DEFAULT_MISSION_OUTPUT_DIR = "result/unified_mission"
DEFAULT_MISSION_JSON = "result/unified_mission/mission_output.json"


def unified_input_to_weather_info(unified_input: UnifiedInput) -> Dict[str, Any]:
    """将 UnifiedInput.weather 转为 export_grouped_mission_to_json 可用的 weather_info。"""
    w = unified_input.weather
    return {
        "scene": "unified_input",
        "label": "统一输入",
        "wind_speed": float(w.wind_speed),
        "wind_direction": float(w.wind_direction),
        "gust_factor": 1.0,
        "risk_level": "low",
        "energy_factor": 1.0,
        "description": f"visibility={w.visibility}",
        "weather_penalty_total": 0.0,
        "estimated_energy_score": 0.0,
    }


def unified_input_to_mission(
    unified_input: UnifiedInput,
    spacing: float = 50.0,
    merge_thresh: float = 25.0,
    enable_sa: bool = False,
    eps: float = 150.0,
    start_edge_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    UnifiedInput → TopoGraph → EdgeTask → GroupedContinuousMission。

    Returns:
        {
            "topo_graph": TopoGraph,
            "edge_tasks": List[EdgeTask],
            "line_inspection_points_by_line": dict,
            "mission": GroupedContinuousMission,
        }
    """
    topo_graph, edge_tasks, line_inspection_points_by_line = unified_input_to_edge_tasks(
        unified_input,
        spacing=spacing,
        merge_thresh=merge_thresh,
    )

    mission = plan_global_topology_optimized_mission(
        topo_graph=topo_graph,
        edge_tasks=edge_tasks,
        start_edge_id=start_edge_id,
        enable_sa=enable_sa,
        eps=eps,
    )

    return {
        "topo_graph": topo_graph,
        "edge_tasks": edge_tasks,
        "line_inspection_points_by_line": line_inspection_points_by_line,
        "mission": mission,
    }


def export_unified_mission(
    mission_result: Dict[str, Any],
    unified_input: UnifiedInput,
    output_path: Union[str, Path] = DEFAULT_MISSION_JSON,
    input_file: Optional[Union[str, Path]] = None,
) -> str:
    """
    导出与主线格式一致的 mission_output.json，并补充 unified 管线 metadata。
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    weather_info = unified_input_to_weather_info(unified_input)

    export_grouped_mission_to_json(
        mission=mission_result["mission"],
        edge_tasks=mission_result["edge_tasks"],
        line_inspection_points_by_line=mission_result["line_inspection_points_by_line"],
        output_path=str(output_path),
        terrain_3d=None,
        weather_info=weather_info,
    )

    with output_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    meta = data.setdefault("metadata", {})
    meta["source"] = "unified_input"
    meta["input_file"] = str(Path(input_file).resolve()) if input_file else None
    meta["pipeline_version"] = "v1"
    meta["planner_name"] = "UnifiedInputBridge+GlobalTopologyOptimized"

    if unified_input.metadata:
        meta["unified_input_metadata"] = dict(unified_input.metadata)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")

    return str(output_path.resolve())


def print_mission_summary(mission_result: Dict[str, Any]) -> None:
    """打印 Mission 规划摘要。"""
    topo_graph = mission_result["topo_graph"]
    edge_tasks = mission_result["edge_tasks"]
    mission = mission_result["mission"]
    line_pts = mission_result["line_inspection_points_by_line"]

    total_line_pts = sum(len(v) for v in line_pts.values())
    total_task_pts = sum(t.num_points for t in edge_tasks)

    print(f"TopoGraph nodes: {len(topo_graph.nodes)}")
    print(f"TopoGraph edges: {len(topo_graph.edges)}")
    print(f"EdgeTask count: {len(edge_tasks)}")
    print(f"Mission groups: {len(mission.groups)}")
    print(f"Line-level inspection points: {total_line_pts}")
    print(f"EdgeTask inspection points: {total_task_pts}")
    print(f"Mission total_length: {mission.total_length:.2f} px")
    print(f"Mission inspect_length: {mission.inspect_length:.2f} px")
    print(f"Mission connect_length: {mission.connect_length:.2f} px")
    print(f"Mission segments: {len(mission.segments)}")
    print("-" * 50)
    print("edge_visit_order:")
    for i, eid in enumerate(mission.visit_order, 1):
        print(f"  {i}. {eid}")
    print("group_visit_order:", mission.group_visit_order)
    print("-" * 50)
