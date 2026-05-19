"""
Unified Mission 可视化：baseline + optimized 对比

用法:
    python demo/demo_unified_mission_visualization.py

依赖:
    python demo/demo_unified_mission.py
    python demo/demo_mission_compare.py

输出:
    result/unified_mission/unified_mission_view.html
"""

import json
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ImportError:
    print("[FAIL] 需要 plotly: pip install plotly")
    sys.exit(1)

from planner.mission_analysis import analyze_mission_json


def _load_mission(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _add_mission_traces(
    fig,
    data: dict,
    name_prefix: str,
    inspect_color: str,
    connect_color: str = "#95a5a6",
    row: int = None,
    col: int = None,
) -> None:
    subplot_kw = {}
    if row is not None and col is not None:
        subplot_kw = {"row": row, "col": col}

    for seg in data.get("segments", []):
        geom = seg.get("geometry_2d") or []
        if len(geom) < 2:
            continue
        xs = [p[0] for p in geom]
        ys = [p[1] for p in geom]
        seg_type = seg.get("type", "unknown")
        color = inspect_color if seg_type == "inspect" else connect_color
        width = 3 if seg_type == "inspect" else 2
        dash = None if seg_type == "inspect" else "dash"
        fig.add_trace(
            go.Scatter(
                x=xs,
                y=ys,
                mode="lines",
                name=f"{name_prefix} {seg_type}",
                line=dict(color=color, width=width, dash=dash),
                legendgroup=name_prefix,
                showlegend=True,
                hovertemplate=(
                    f"{name_prefix}<br>type={seg_type}<br>"
                    f"edge={seg.get('edge_id')}<br>len={seg.get('length')}<extra></extra>"
                ),
            ),
            **subplot_kw,
        )

    points = data.get("inspection_points", [])
    if points:
        px = [p["pixel_position"][0] for p in points]
        py = [p["pixel_position"][1] for p in points]
        fig.add_trace(
            go.Scatter(
                x=px,
                y=py,
                mode="markers",
                name=f"{name_prefix} points",
                marker=dict(size=5, color="#e74c3c", symbol="circle-open"),
                legendgroup=name_prefix,
                showlegend=True,
            ),
            **subplot_kw,
        )


def build_comparison_figure(baseline_data: dict, optimized_data: dict) -> go.Figure:
    b_stats = analyze_mission_json(baseline_data)
    o_stats = analyze_mission_json(optimized_data)

    fig = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=(
            f"Baseline | len={b_stats['total_length']}px "
            f"connect_ratio={b_stats['connect_ratio']:.1%}",
            f"Optimized | len={o_stats['total_length']}px "
            f"connect_ratio={o_stats['connect_ratio']:.1%}",
        ),
    )

    _add_mission_traces(fig, baseline_data, "baseline", "#27ae60", row=1, col=1)
    _add_mission_traces(fig, optimized_data, "optimized", "#2980b9", row=1, col=2)

    b_order = baseline_data.get("visit_order", {}).get("edge_visit_order", [])
    o_order = optimized_data.get("visit_order", {}).get("edge_visit_order", [])

    fig.update_layout(
        title=(
            f"Mission Compare | baseline connect={b_stats['connect_ratio']:.1%} "
            f"vs optimized={o_stats['connect_ratio']:.1%}"
        ),
        height=600,
        legend=dict(orientation="h", yanchor="bottom", y=1.08),
        margin=dict(l=40, r=40, t=100, b=60),
    )
    fig.update_xaxes(title_text="x (px)", row=1, col=1)
    fig.update_xaxes(title_text="x (px)", row=1, col=2)
    fig.update_yaxes(title_text="y (px)", scaleanchor="x", scaleratio=1, row=1, col=1)
    fig.update_yaxes(title_text="y (px)", scaleanchor="x2", scaleratio=1, row=1, col=2)

    fig.add_annotation(
        text="visit: " + " → ".join(b_order),
        xref="paper", yref="paper", x=0.25, y=-0.12,
        showarrow=False, font=dict(size=9), xanchor="center",
    )
    fig.add_annotation(
        text="visit: " + " → ".join(o_order),
        xref="paper", yref="paper", x=0.75, y=-0.12,
        showarrow=False, font=dict(size=9), xanchor="center",
    )

    return fig


def build_single_figure(data: dict, label: str) -> go.Figure:
    stats = analyze_mission_json(data)
    fig = go.Figure()
    _add_mission_traces(fig, data, label, "#27ae60")
    fig.update_layout(
        title=(
            f"{label} | total={stats['total_length']}px "
            f"connect_ratio={stats['connect_ratio']:.1%}"
        ),
        xaxis_title="x (px)",
        yaxis_title="y (px)",
        yaxis=dict(scaleanchor="x", scaleratio=1),
    )
    return fig


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(root, "result", "unified_mission")
    baseline_path = os.path.join(out_dir, "mission_output.json")
    optimized_path = os.path.join(out_dir, "optimized_mission_output.json")
    html_path = os.path.join(out_dir, "unified_mission_view.html")

    print("=" * 60)
    print("Unified Mission 可视化 (baseline + optimized)")
    print("=" * 60)

    if not os.path.isfile(baseline_path):
        print("[FAIL] 缺少 baseline: 请先运行 demo_unified_mission.py")
        sys.exit(1)

    baseline_data = _load_mission(baseline_path)

    if os.path.isfile(optimized_path):
        optimized_data = _load_mission(optimized_path)
        fig = build_comparison_figure(baseline_data, optimized_data)
        mode = "comparison"
    else:
        print("[WARN] 无 optimized_mission_output.json，仅显示 baseline")
        fig = build_single_figure(baseline_data, "baseline")
        mode = "baseline_only"

    os.makedirs(out_dir, exist_ok=True)
    fig.write_html(html_path, include_plotlyjs="cdn")
    print(f"[OK] HTML saved ({mode}): {html_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
