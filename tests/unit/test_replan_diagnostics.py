import copy

import pytest

from tests.helpers.replan_diagnostics import CONNECT_CLASSES, classify_connect, physical_identity_map

def _identity(a=0, b=0):
    return {"PL_000": {"connected_component_id": a}, "PL_001": {"connected_component_id": b}}

@pytest.mark.parametrize(("segment", "identity", "expected"), [
    ({"role": "from_start"}, {}, "free_flight_endpoint_access"),
    ({"role": "to_end"}, {}, "free_flight_endpoint_access"),
    ({"from_edge_id": "PL_000", "to_edge_id": "PL_001"}, _identity(0, 1), "free_flight_between_components"),
    ({"from_edge_id": "PL_000", "to_edge_id": "PL_001", "connect_mode": "dijkstra", "topo_edge_ids": ["L_A_edge_0"]}, _identity(), "topology"),
    ({"from_edge_id": "PL_000", "to_edge_id": "PL_001", "fallback_reason": "dijkstra_exception"}, _identity(), "unexpected_fallback"),
    ({"from_edge_id": "PL_missing", "to_edge_id": "PL_001"}, _identity(), "unknown"),
])
def test_connect_classifier_business_rules(segment, identity, expected):
    category, _ = classify_connect(segment, identity)
    assert category in CONNECT_CLASSES
    assert category == expected

def test_physical_identity_map_uses_explicit_relationships_only():
    mission = {"metadata": {"physical_line_chains": [{"physical_id": "PL_007", "member_chain_ids": ["L_016_chain_0"], "topo_edge_ids": ["L_016_edge_0"], "line_ids": ["L_016"]}], "topo_edges_pixel": [{"edge_id": "L_016_edge_0", "line_id": "L_016", "u": "n0", "v": "n1"}]}}
    assert physical_identity_map(mission)["PL_007"] == {"member_chain_ids": ["L_016_chain_0"], "topo_edge_ids": ["L_016_edge_0"], "line_ids": ["L_016"], "component_ids": [0], "connected_component_id": 0}

def test_replan_failure_does_not_mutate_baseline(monkeypatch, baseline_copy):
    import app
    before = copy.deepcopy(baseline_copy)
    monkeypatch.setattr(app, "build_start_end_replan_mission", lambda *_a, **_k: (_ for _ in ()).throw(RuntimeError("controlled")))
    with pytest.raises(Exception):
        app._replan_dashboard_from_start_end(mission_data=baseline_copy, start_xy=[801, 410], end_xy=[751, 892], inspection_point_source="image", detection_path="data/chengdu_real_point.png", display_map="data/chengdu_real_point.png", map_rel_for_dashboard="data/chengdu_real_point.png")
    assert baseline_copy == before
