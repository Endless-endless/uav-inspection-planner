from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image
import plotly.graph_objects as go


IMAGE_PATH = Path("data/test.png")
MISSION_JSON = Path("result/latest/mission_output.json")
OUTPUT_HTML = Path("result/latest/final_teacher_demo.html")


def load_mission_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    # =====================================================
    # 加载地图
    # =====================================================
    img = Image.open(IMAGE_PATH)
    map_width, map_height = img.size

    # 转为 base64 用于嵌入
    from io import BytesIO
    import base64

    buffered = BytesIO()
    img.save(buffered, format="PNG")
    img_base64 = base64.b64encode(buffered.getvalue()).decode()
    img_data_uri = f"data:image/png;base64,{img_base64}"

    # =====================================================
    # 加载任务数据
    # =====================================================
    mission = load_mission_json(MISSION_JSON)

    # 提取路径
    full_path_dict = mission.get("full_path", {})
    path_2d = full_path_dict.get("full_path_2d", [])

    # 提取 inspection points
    inspection_points = mission.get("inspection_points", [])

    # =====================================================
    # 创建图形
    # =====================================================
    fig = go.Figure()

    # =====================================================
    # 添加地图作为背景（layout.images）
    # =====================================================
    fig.add_layout_image(
        dict(
            source=img_data_uri,
            xref="x",
            yref="y",
            x=0,
            y=map_height,
            sizex=map_width,
            sizey=map_height,
            sizing="stretch",
            layer="below",
            opacity=1.0
        )
    )

    # =====================================================
    # 添加路径
    # =====================================================
    if path_2d:
        path_x = [p[0] for p in path_2d]
        path_y = [p[1] for p in path_2d]

        # 创建 hover 信息
        hover_text = [f"路径点 {i+1}<br>坐标: ({x:.1f}, {y:.1f})"
                      for i, (x, y) in enumerate(path_2d)]

        fig.add_trace(go.Scatter(
            x=path_x,
            y=path_y,
            mode="lines",
            line=dict(
                color="blue",
                width=5
            ),
            name="UAV巡检路径",
            hovertext=hover_text,
            hoverinfo="text"
        ))

        # 标记起点
        if len(path_2d) > 0:
            fig.add_trace(go.Scatter(
                x=[path_2d[0][0]],
                y=[path_2d[0][1]],
                mode="markers+text",
                marker=dict(
                    color="#00FF00",
                    size=12,
                    symbol="square",
                    line=dict(color="white", width=2)
                ),
                name="起点",
                text=["起点"],
                textposition="top center",
                textfont=dict(size=11, color="green"),
                hovertext=[f"起点<br>坐标: ({path_2d[0][0]:.1f}, {path_2d[0][1]:.1f})"],
                hoverinfo="text"
            ))

        # 标记终点
        if len(path_2d) > 1:
            fig.add_trace(go.Scatter(
                x=[path_2d[-1][0]],
                y=[path_2d[-1][1]],
                mode="markers+text",
                marker=dict(
                    color="red",
                    size=12,
                    symbol="square",
                    line=dict(color="white", width=2)
                ),
                name="终点",
                text=["终点"],
                textposition="top center",
                textfont=dict(size=11, color="red"),
                hovertext=[f"终点<br>坐标: ({path_2d[-1][0]:.1f}, {path_2d[-1][1]:.1f})"],
                hoverinfo="text"
            ))

    # =====================================================
    # 添加 inspection points
    # =====================================================
    if inspection_points:
        # 按类型分类
        endpoint_points = [p for p in inspection_points if p.get("point_type") == "endpoint"]
        turning_points = [p for p in inspection_points if p.get("point_type") == "turning"]
        sample_points = [p for p in inspection_points if p.get("point_type") == "sample"]

        # 终点（红色菱形，小尺寸）
        if endpoint_points:
            end_x = [p["pixel_position"][0] for p in endpoint_points]
            end_y = [p["pixel_position"][1] for p in endpoint_points]

            end_hover = []
            for p in endpoint_points:
                hover = (f"point_id: {p.get('point_id')}<br>"
                        f"edge_id: {p.get('edge_id')}<br>"
                        f"group_id: {p.get('group_id')}<br>"
                        f"point_type: {p.get('point_type')}<br>"
                        f"visit_order: {p.get('visit_order')}")
                end_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=end_x,
                y=end_y,
                mode="markers",
                marker=dict(
                    color="red",
                    size=5,
                    symbol="diamond",
                    line=dict(color="white", width=1)
                ),
                name=f"巡检终点 ({len(endpoint_points)}个)",
                hovertext=end_hover,
                hoverinfo="text",
                visible=True
            ))

        # 转向点（橙色圆形，小尺寸）
        if turning_points:
            turn_x = [p["pixel_position"][0] for p in turning_points]
            turn_y = [p["pixel_position"][1] for p in turning_points]

            turn_hover = []
            for p in turning_points:
                hover = (f"point_id: {p.get('point_id')}<br>"
                        f"edge_id: {p.get('edge_id')}<br>"
                        f"group_id: {p.get('group_id')}<br>"
                        f"point_type: {p.get('point_type')}<br>"
                        f"visit_order: {p.get('visit_order')}")
                turn_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=turn_x,
                y=turn_y,
                mode="markers",
                marker=dict(
                    color="orange",
                    size=4,
                    symbol="circle",
                    line=dict(color="white", width=0.5)
                ),
                name=f"转向点 ({len(turning_points)}个)",
                hovertext=turn_hover,
                hoverinfo="text",
                visible=True
            ))

        # 采样点（绿色小点，默认隐藏）
        if sample_points:
            sample_x = [p["pixel_position"][0] for p in sample_points]
            sample_y = [p["pixel_position"][1] for p in sample_points]

            sample_hover = []
            for p in sample_points:
                hover = (f"point_id: {p.get('point_id')}<br>"
                        f"edge_id: {p.get('edge_id')}<br>"
                        f"group_id: {p.get('group_id')}<br>"
                        f"point_type: {p.get('point_type')}<br>"
                        f"visit_order: {p.get('visit_order')}")
                sample_hover.append(hover)

            fig.add_trace(go.Scatter(
                x=sample_x,
                y=sample_y,
                mode="markers",
                marker=dict(
                    color="green",
                    size=3,
                    symbol="circle",
                    opacity=0.6
                ),
                name=f"采样点 ({len(sample_points)}个) - 点击图例显示",
                hovertext=sample_hover,
                hoverinfo="text",
                visible=False
            ))

    # =====================================================
    # 设置布局
    # =====================================================
    fig.update_xaxes(
        range=[0, map_width],
        showgrid=False,
        showticklabels=False,
        showline=False,
        zeroline=False
    )

    fig.update_yaxes(
        range=[map_height, 0],  # 反转 y 轴
        showgrid=False,
        showticklabels=False,
        showline=False,
        zeroline=False
    )

    fig.update_layout(
        title=dict(
            text="UAV 电网巡检路径规划 - 最终演示<br><sub>基于真实地图 data/test.png | Step9_2 分组连续规划算法</sub>",
            x=0.5,
            xanchor='center',
            font=dict(size=18)
        ),
        xaxis_title="",
        yaxis_title="",
        hovermode="closest",
        showlegend=True,
        legend=dict(
            x=1.02,
            y=1,
            xanchor="left",
            yanchor="top",
            bgcolor="rgba(255, 255, 255, 0.85)",
            bordercolor="gray",
            borderwidth=1,
            font=dict(size=11)
        ),
        # 地图占满主要区域
        width=1800,
        height=1000,
        # 最小边距
        margin=dict(l=10, r=150, t=60, b=10),
        plot_bgcolor="white",
        paper_bgcolor="white"
    )

    # =====================================================
    # 保存 HTML
    # =====================================================
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(OUTPUT_HTML), include_plotlyjs=True)
    print(f"[OK] 最终演示页面已生成: {OUTPUT_HTML}")


if __name__ == "__main__":
    main()
