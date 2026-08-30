"""
=====================================================
UAV 电网巡检 - 主展示页入口（交互式版本）
=====================================================

功能：
1. 清理旧数据，确保生成全新的初始任务版本
2. 执行拓扑规划并生成 3D 巡检路径
3. 导出标准 mission JSON
4. 创建交互式主展示页（基于 Plotly layout.images）
5. 显示：真实地图底图 + 巡检路径 + inspection points（可交互）

使用方式：
    python demo/demo_visualization_main.py

输出：
    result/latest/mission_output.json - 标准任务 JSON（初始版本）
    result/latest/main_view_interactive.html - 交互式主展示页

特点：
- 每次运行都会清理旧数据并生成全新的初始任务版本
- 不会重用之前重规划的结果
- 使用 Plotly layout.images 显示真实地图（data/test.png）
- 路径和 inspection points 支持 hover 交互
- 图层控制（sample points 可通过图例开关）
- 系统界面风格（非报告页）

=====================================================
"""

import sys
import os
import json
from typing import Dict, Optional

import numpy as np
from PIL import Image

# 添加项目路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3
from core.topo_plan import export_grouped_mission_to_json


def main(
    image_path: str = "data/test.png",
    inspection_point_source: str = "spacing",
    inspection_spacing: float = 100.0,
    line_image_path: Optional[str] = None,
    point_image_path: Optional[str] = None,
    image_alignment: Optional[Dict] = None,
):
    """
    主函数：生成主展示页
    """
    print("="*70)
    print("UAV 电网巡检 - 主展示页生成")
    print("="*70)
    print()

    # =====================================================
    # Phase -1: 清理旧数据，确保生成全新的初始任务版本
    # =====================================================
    print("Phase -1: 清理旧数据（确保全新初始任务版本）...")
    print("-"*70)

    import shutil
    from pathlib import Path

    # 定义需要清理的文件
    files_to_clean = [
        "result/latest/mission_output.json",
        "result/latest/mission_output_before_replan.json",
    ]

    cleaned_count = 0
    for file_path in files_to_clean:
        path = Path(file_path)
        if path.exists():
            # 备份到 .old 文件（可选，便于调试）
            backup_path = Path(str(path) + ".old")
            try:
                shutil.copy(path, backup_path)
                path.unlink()
                print(f"  [清理] 已移除并备份: {file_path} -> {backup_path.name}")
                cleaned_count += 1
            except Exception as e:
                print(f"  [WARN] 无法清理 {file_path}: {e}")

    if cleaned_count == 0:
        print("  [信息] 无旧数据需要清理")
    else:
        print(f"  [完成] 已清理 {cleaned_count} 个旧文件")
    print()

    # =====================================================
    # Phase 0: 天气场景配置
    # =====================================================
    print("Phase 0: 天气场景配置...")
    print("-"*70)

    # 设置天气场景（可选：calm, crosswind, headwind_strong, tailwind_efficient, gusty_high_risk）
    weather_scene = "calm"  # 默认：微风

    line_image_path = line_image_path or image_path
    point_image_path = point_image_path or image_path
    line_size = Image.open(line_image_path).size
    point_size = Image.open(point_image_path).size
    if line_size != point_size and not image_alignment:
        raise ValueError(
            "线路图与巡检点图尺寸不一致: "
            f"line={line_size[0]}x{line_size[1]}, point={point_size[0]}x{point_size[1]}"
        )

    display_map = os.environ.get("UAV_DISPLAY_MAP", "").strip()
    dataset_type = os.environ.get("UAV_DATASET_TYPE", "").strip()
    if dataset_type == "real_satellite" or "chengdu_real" in point_image_path.replace("\\", "/"):
        os.environ["UAV_IMAGE_PIXEL_COORDS"] = "1"

    planner = PowerlinePlannerV3(
        image_path=line_image_path,
        point_image_path=point_image_path,
        image_alignment=image_alignment,
        flight_height=30,
        weather_scene=weather_scene
    )
    planner.inspection_point_source = inspection_point_source

    # 获取并显示天气信息
    weather_info = planner.get_weather_info()
    print(f"[当前天气场景] {weather_info.get('label', '未知')} ({weather_info.get('scene', 'unknown')})")
    print(f"  - 风速: {weather_info.get('wind_speed', 0):.1f} m/s")
    print(f"  - 风向: {weather_info.get('wind_direction', 0):.1f}°")
    print(f"  - 阵风因子: {weather_info.get('gust_factor', 1.0):.2f}")
    print(f"  - 风险等级: {weather_info.get('risk_level', 'unknown')}")
    print(f"  - 能耗因子: {weather_info.get('energy_factor', 1.0):.2f}")
    print()

    # =====================================================
    # Phase 1: 执行完整规划流程
    # =====================================================
    print("Phase 1: 执行拓扑规划...")
    print("-"*70)

    # 执行图像处理和线路识别
    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()
    if inspection_point_source == "image":
        planner.step5_detect_image_inspection_points()
    else:
        planner.step5_generate_line_inspection_points(spacing=inspection_spacing)

    # 设置地形
    terrain = np.zeros((planner.height, planner.width), dtype=np.float32)
    if inspection_point_source == "image":
        planner.step6_smooth_terrain(terrain)
    else:
        planner.step6_map_line_points_to_3d(terrain)

    # 拓扑建模
    topo_graph = planner.step7_5_build_topo()

    if inspection_point_source == "image":
        planner.step5_finalize_image_inspection_points()
        planner._map_existing_points_to_3d()

    # EdgeTask 建模
    edge_tasks = planner.step8_5_build_edge_tasks()

    print(f"[完成] 独立线路: {len(planner.independent_lines)} 条")
    print(f"[完成] 巡检点: {len(planner.line_inspection_points)} 个")
    print(f"[完成] 拓扑节点: {len(topo_graph.nodes)} 个")
    print(f"[完成] 拓扑边: {len(topo_graph.edges)} 条")
    print(f"[完成] 边任务: {len(edge_tasks)} 个")
    print()

    # =====================================================
    # Phase 2: 生成分组规划路径（全局拓扑优化）
    # =====================================================
    print()
    print("Phase 2: 生成分组规划路径（全局拓扑优化）...")
    print("-"*70)

    try:
        # 使用 step9_4 全局拓扑优化（基于论文思想）
        mission = planner.step9_4_plan_global_topology_optimized(
            enable_sa=False,  # 使用纯贪心策略（模拟退火在此数据集上未表现优势）
            eps=150.0
        )
    except ImportError as e:
        print(f"[WARN] 全局优化需要 scikit-learn，切换到基础连续规划")
        print(f"  错误: {e}")
        mission = planner.step9_1_plan_continuous_mission_greedy()

    if mission is None:
        print("[FAIL] 规划失败")
        return None

    print(f"[完成] 总长度: {mission.total_length:.1f}px")
    print(f"[完成] 巡检: {mission.inspect_length:.1f}px")
    print(f"[完成] 连接: {mission.connect_length:.1f}px")
    print()

    # =====================================================
    # Phase 2.5: 导出 JSON
    # =====================================================
    print()
    print("Phase 2.5: 导出任务 JSON...")
    print("-"*70)

    # 合并天气统计信息到 weather_info
    if hasattr(planner, 'stats') and planner.stats:
        weather_info_for_export = dict(weather_info)  # 创建副本
        # 添加天气统计信息
        if 'weather_penalty_total' in planner.stats:
            weather_info_for_export['weather_penalty_total'] = planner.stats['weather_penalty_total']
        if 'estimated_energy_score' in planner.stats:
            weather_info_for_export['estimated_energy_score'] = planner.stats['estimated_energy_score']
    else:
        weather_info_for_export = weather_info

    dataset_type = os.environ.get("UAV_DATASET_TYPE", "").strip() or None
    is_real_map = dataset_type == "real_satellite" or "chengdu_real" in point_image_path.replace("\\", "/")
    if is_real_map:
        map_image = point_image_path
        clean_map = point_image_path
        display_map = point_image_path
    else:
        display_map = os.environ.get("UAV_DISPLAY_MAP", "").strip() or None
        clean_map = display_map or getattr(planner, "clean_map_path", None) or point_image_path
        display_map = display_map or clean_map
        map_image = point_image_path
    extra_meta = {
        "inspection_point_source": planner.inspection_point_source,
        "map_image": map_image,
        "line_image": line_image_path,
        "point_image": point_image_path,
        "clean_map_image": clean_map,
        "display_map_image": display_map,
        "coordinate_mode": "image_pixel_fixed",
        "pixel_coordinate_mode": True,
        "image_width": getattr(planner, "width", None),
        "image_height": getattr(planner, "height", None),
        "image_inspection_overlay": planner.image_inspection_overlay,
        "image_detection_stats": getattr(planner, "image_detection_stats", {}),
        "image_alignment": dict(planner.image_alignment_metadata or {}),
    }
    if dataset_type:
        extra_meta["dataset_type"] = dataset_type
    json_path = export_grouped_mission_to_json(
        mission=mission,
        edge_tasks=edge_tasks,
        line_inspection_points_by_line=planner.line_inspection_points_by_line,
        output_path="result/latest/mission_output.json",
        terrain_3d=planner.terrain_3d,
        weather_info=weather_info_for_export,
        extra_metadata=extra_meta,
    )

    print(f"[完成] JSON 已导出: {json_path}")
    print()
    print("=" * 70)
    print("图像主线任务生成完成（Dashboard 使用 result/latest/mission_output.json）")
    print("=" * 70)

    return planner


if __name__ == "__main__":
    image_path = os.environ.get("UAV_IMAGE_PATH", "data/test.png")
    line_image_path = os.environ.get("UAV_LINE_IMAGE_PATH", "").strip() or image_path
    point_image_path = os.environ.get("UAV_POINT_IMAGE_PATH", "").strip() or image_path
    image_alignment_raw = os.environ.get("UAV_IMAGE_ALIGNMENT", "").strip()
    image_alignment = json.loads(image_alignment_raw) if image_alignment_raw else None
    inspection_point_source = os.environ.get("UAV_INSPECTION_SOURCE", "spacing")
    inspection_spacing = float(os.environ.get("UAV_INSPECTION_SPACING", "100"))
    planner = main(
        image_path=image_path,
        line_image_path=line_image_path,
        point_image_path=point_image_path,
        image_alignment=image_alignment,
        inspection_point_source=inspection_point_source,
        inspection_spacing=inspection_spacing,
    )
