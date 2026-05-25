"""
BFS vs Dijkstra 连接路径可视化

输出: result/dijkstra_test/dijkstra_compare_view.html
"""

import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ImportError:
    print("[FAIL] pip install plotly")
    sys.exit(1)

from input.unified_input import load_unified_input_from_json
from input.unified_adapter import build_topo_graph_from_independent_lines, unified_input_to_independent_lines
from planner.topo_dijkstra import (
    dijkstra_shortest_path,
    expand_node_path_to_geometry,
    get_node_path_bfs,
)


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample = os.path.join(root, "data", "sample_unified_topo_input.json")
    out_html = os.path.join(root, "result", "dijkstra_test", "dijkstra_compare_view.html")

    unified = load_unified_input_from_json(sample)
    lines = unified_input_to_independent_lines(unified)
    topo_graph, _, _ = build_topo_graph_from_independent_lines(lines)

    # merged_0 与 merged_1 分属不同连通分量；选用图内可达的一对节点
    start_id, goal_id = "merged_0", "line_trunk_end"
    sa = topo_graph.get_node(start_id)
    sb = topo_graph.get_node(goal_id)
    if sa is None or sb is None:
        print("[FAIL] nodes not found")
        sys.exit(1)
    pa, pb = sa.pos2d, sb.pos2d

    bfs_nodes = get_node_path_bfs(topo_graph, start_id, goal_id)
    if not bfs_nodes:
        print(f"[WARN] BFS: no path {start_id}->{goal_id}, try merged_0->line_west_end")
        start_id, goal_id = "merged_0", "line_west_end"
        sa, sb = topo_graph.get_node(start_id), topo_graph.get_node(goal_id)
        pa, pb = sa.pos2d, sb.pos2d
        bfs_nodes = get_node_path_bfs(topo_graph, start_id, goal_id)

    bfs_geom, bfs_len = expand_node_path_to_geometry(pa, pb, bfs_nodes, topo_graph)

    dj = dijkstra_shortest_path(topo_graph, start_id, goal_id)
    if dj is None:
        print("[FAIL] Dijkstra unreachable")
        sys.exit(1)
    dj_nodes = dj["node_path"]
    dj_geom, dj_len = expand_node_path_to_geometry(pa, pb, dj_nodes, topo_graph)
    dj_cost = dj["total_cost"]

    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=(
            f"BFS | hops={len(bfs_nodes)-1} geom_len={bfs_len:.1f}px",
            f"Dijkstra | cost={dj_cost:.1f} geom_len={dj_len:.1f}px",
        ),
    )

    for col, geom, color, name in [
        (1, bfs_geom, "#95a5a6", "BFS connect"),
        (2, dj_geom, "#2980b9", "Dijkstra connect"),
    ]:
        xs = [p[0] for p in geom]
        ys = [p[1] for p in geom]
        fig.add_trace(
            go.Scatter(
                x=xs, y=ys, mode="lines",
                line=dict(color=color, width=3, dash="dash"),
                name=name,
            ),
            row=1, col=col,
        )

    for col in (1, 2):
        fig.add_trace(
            go.Scatter(
                x=[pa[0], pb[0]], y=[pa[1], pb[1]],
                mode="markers+text",
                marker=dict(size=12, color="#e74c3c"),
                text=[start_id, goal_id],
                textposition="top center",
                name="endpoints",
                showlegend=(col == 1),
            ),
            row=1, col=col,
        )

    for eid, edge in topo_graph.edges.items():
        xs = [p[0] for p in edge.polyline]
        ys = [p[1] for p in edge.polyline]
        fig.add_trace(
            go.Scatter(
                x=xs, y=ys, mode="lines",
                line=dict(color="#2ecc71", width=2),
                opacity=0.5,
                name="topo edge",
                showlegend=False,
            ),
            row=1, col=1,
        )

    same = bfs_nodes == dj_nodes
    fig.update_layout(
        title=f"BFS vs Dijkstra @ {start_id} → {goal_id} | paths_equal={same}",
        height=500,
        yaxis=dict(scaleanchor="x", scaleratio=1),
        yaxis2=dict(scaleanchor="x2", scaleratio=1),
    )

    os.makedirs(os.path.dirname(out_html), exist_ok=True)
    fig.write_html(out_html, include_plotlyjs="cdn")
    print(f"[OK] {out_html}")


if __name__ == "__main__":
    main()
