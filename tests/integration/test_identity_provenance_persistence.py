import hashlib
import json

from core.physical_line_chain import build_physical_line_chains
from core.topo import TopoEdge, TopoGraph, TopoNode
from core.topo_plan import EdgeGroup, GroupedContinuousMission, MissionSegment, export_grouped_mission_to_json
from core.topo_task import EdgeTask
from planner.mission_result_builder import build_dashboard_from_mission_json
from planner.topo_dijkstra import generate_connection_segment_with_planner
from tests.helpers.replan_diagnostics import classify_mission_connects, physical_identity_map

def _node(node_id, x):
    return TopoNode(id=node_id, kind="endpoint", pos2d=(x, 0.0), pos3d=(x, 0.0, 0.0), deg=1, line_ids=["L_A"], pt_indices={})

def _single_component_fixture():
    graph = TopoGraph()
    graph.add_node(_node("n0", 0.0)); graph.add_node(_node("n1", 10.0))
    graph.add_edge(TopoEdge(id="L_A_edge_0", u="n0", v="n1", line_id="L_A", polyline=[(0.0, 0.0), (10.0, 0.0)], len2d=10.0, pixel_polyline=[(0.0, 0.0), (10.0, 0.0)]))
    task = EdgeTask(edge_id="L_A_chain_0", u="n0", v="n1", line_id="L_A", polyline=[(0.0, 0.0), (10.0, 0.0)], len2d=10.0, pixel_polyline=[(0.0, 0.0), (10.0, 0.0)], inspection_points=[{"point_id": "IP_1", "pixel_position": [5.0, 0.0]}], num_points=1, meta={"chain_topo_edge_ids": ["L_A_edge_0"]})
    return graph, task

def _business_hash(segments):
    payload = [{"type": s["type"], "edge_id": s.get("edge_id"), "from_edge_id": s.get("from_edge_id"), "to_edge_id": s.get("to_edge_id"), "length": s["length"], "geometry_2d": s["geometry_2d"]} for s in segments]
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

def test_real_physical_objects_export_stable_identity_and_provenance(tmp_path):
    graph, chain = _single_component_fixture()
    physical = build_physical_line_chains([chain], graph)
    assert len(physical) == 1
    task = physical[0]
    assert task.id == "PL_000"
    assert task.chain_ids == ["L_A_chain_0"]
    assert task.topo_edge_ids == ["L_A_edge_0"]
    assert task.line_ids == ["L_A"]
    assert task.meta["component_ids"] == [0]

    provenance = {}
    geometry, length = generate_connection_segment_with_planner((2.0, 0.0), (8.0, 0.0), graph, {task.id: task}, connect_planner="dijkstra", from_edge_id=task.id, to_edge_id=task.id, provenance_out=provenance)
    mission = GroupedContinuousMission()
    mission.task_by_id = {task.id: task}
    mission.segments = [
        MissionSegment(type="inspect", edge_id=task.id, to_edge_id=task.id, geometry=list(task.polyline), length=task.length),
        MissionSegment(type="connect", from_edge_id=task.id, to_edge_id=task.id, geometry=geometry, length=length, connect_mode=provenance["connect_mode"], planner=provenance["planner"], reason=provenance["reason"], fallback_reason=provenance["fallback_reason"], topo_edge_ids=provenance["topo_edge_ids"], from_component_ids=[0], to_component_ids=[0]),
    ]
    mission.total_length = sum(s.length for s in mission.segments); mission.inspect_length = task.length; mission.connect_length = length
    mission.visit_order = [task.id]; mission.groups = {"G": EdgeGroup("G", [task.id], (5.0, 0.0), (0.0, 0.0, 10.0, 0.0), task.length)}; mission.group_visit_order = ["G"]; mission.edge_to_group = {task.id: "G"}
    output = tmp_path / "mission.json"
    export_grouped_mission_to_json(mission, [chain], {"L_A": []}, str(output))
    data = json.loads(output.read_text(encoding="utf-8"))
    identity = physical_identity_map(data)[task.id]
    assert identity["member_chain_ids"] == ["L_A_chain_0"]
    assert identity["topo_edge_ids"] == ["L_A_edge_0"]
    assert identity["line_ids"] == ["L_A"]
    assert identity["component_ids"] == [0]
    assert data["segments"][0]["physical_id"] == task.id
    connect = data["segments"][1]
    assert connect["connect_mode"] == "topology"
    assert connect["reason"] == "same_line"
    assert connect["topo_edge_ids"] == ["L_A_edge_0"]
    assert connect["fallback_reason"] is None
    counts, evidence = classify_mission_connects(data)
    assert counts["unknown"] == 0, evidence
    assert counts["topology"] == 1

    dashboard = build_dashboard_from_mission_json(data, pipeline="image", source="test")
    assert dashboard["segments"][1]["connect_mode"] == "topology"
    assert dashboard["segments"][1]["topo_edge_ids"] == ["L_A_edge_0"]
    assert dashboard["metadata"]["physical_task_registry"][0]["physical_id"] == task.id
    assert _business_hash(data["segments"]) == _business_hash(dashboard["segments"])

def test_disconnected_dijkstra_records_free_flight_not_fallback():
    graph, first = _single_component_fixture()
    graph.add_node(_node("n2", 30.0)); graph.add_node(_node("n3", 40.0))
    graph.add_edge(TopoEdge(id="L_B_edge_0", u="n2", v="n3", line_id="L_B", polyline=[(30.0, 0.0), (40.0, 0.0)], len2d=10.0, pixel_polyline=[(30.0, 0.0), (40.0, 0.0)]))
    second = EdgeTask(edge_id="L_B_chain_0", u="n2", v="n3", line_id="L_B", polyline=[(30.0, 0.0), (40.0, 0.0)], len2d=10.0, pixel_polyline=[(30.0, 0.0), (40.0, 0.0)], meta={"chain_topo_edge_ids": ["L_B_edge_0"]})
    provenance = {}
    generate_connection_segment_with_planner((10.0, 0.0), (30.0, 0.0), graph, {first.edge_id: first, second.edge_id: second}, connect_planner="dijkstra", use_proximity_bfs=False, from_edge_id=first.edge_id, to_edge_id=second.edge_id, provenance_out=provenance)
    assert provenance["connect_mode"] == "free_flight"
    assert provenance["reason"] == "between_components"
    assert provenance["fallback_reason"] is None
