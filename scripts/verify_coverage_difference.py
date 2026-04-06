"""
电网覆盖差集验证脚本

目的：找出哪些红色电网部分没有进入 inspect 段

验证流程：
1. 获取原始提取的电网（independent_lines）
2. 获取 mission 中的 inspect 段
3. 计算差集：original - inspect
4. 生成两张验收图：
   - Map 1: 覆盖差集图（灰色=原始提取，蓝色=inspect覆盖，红色=未覆盖）
   - Map 2: 段来源图（每个inspect段标注来源）
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy.spatial import cKDTree

from planner.powerline_planner_v3_final import PowerlinePlannerV3


def compute_point_coverage(original_polylines, inspect_polylines, tolerance=3.0):
    """
    计算原始点被 inspect 覆盖的情况

    Args:
        original_polylines: 原始提取的电网polylines列表
        inspect_polylines: inspect段的polylines列表
        tolerance: 覆盖容差（像素）

    Returns:
        covered_points: 被覆盖的原始点列表 [(x,y), ...]
        uncovered_points: 未被覆盖的原始点列表 [(x,y), ...]
    """
    # 收集所有 inspect 点用于快速查询
    all_inspect_points = []
    for poly in inspect_polylines:
        all_inspect_points.extend(poly)

    if not all_inspect_points:
        return [], [pt for poly in original_polylines for pt in poly]

    inspect_tree = cKDTree(all_inspect_points)

    covered_points = []
    uncovered_points = []

    for poly in original_polylines:
        for pt in poly:
            # 查找最近的 inspect 点
            dist, _ = inspect_tree.query(pt)
            if dist <= tolerance:
                covered_points.append(pt)
            else:
                uncovered_points.append(pt)

    return covered_points, uncovered_points


def generate_coverage_difference_map():
    """生成覆盖差集图（Map 1）"""
    print("="*70)
    print("Map 1: 覆盖差集图生成")
    print("="*70)
    print()

    # =====================================================
    # Step 1: 获取原始提取的电网
    # =====================================================
    print("Step 1: 获取原始提取的电网...")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/test.png',
        flight_height=30
    )

    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()

    original_polylines = []
    original_info = {}  # line_id -> (polyline, length, bbox)

    for line in planner.independent_lines:
        if line.ordered_pixels:
            poly = np.array(line.ordered_pixels)
            original_polylines.append(poly)
            original_info[line.id] = {
                'polyline': poly,
                'length': line.length_2d,
                'point_count': len(poly)
            }

    print(f"[完成] 原始电网数: {len(original_polylines)}")
    for line_id, info in sorted(original_info.items()):
        print(f"  - {line_id}: {info['length']:.1f}px, {info['point_count']} 点")
    print()

    # =====================================================
    # Step 2: 获取 inspect 段
    # =====================================================
    print("Step 2: 获取 inspect 段...")
    print("-"*70)

    json_path = 'result/latest/mission_output.json'
    if not os.path.exists(json_path):
        print(f"[错误] mission JSON 不存在: {json_path}")
        return None

    with open(json_path, 'r', encoding='utf-8') as f:
        mission = json.load(f)

    inspect_polylines = []
    inspect_info = []  # list of (segment_id, edge_id, geometry, length)

    for seg in mission.get('segments', []):
        if seg['type'] == 'inspect':
            geo = seg.get('geometry_2d', [])
            if geo:
                poly = np.array(geo)
                inspect_polylines.append(poly)
                inspect_info.append({
                    'segment_id': seg['segment_id'],
                    'edge_id': seg.get('edge_id', 'N/A'),
                    'geometry': poly,
                    'length': seg['length']
                })

    print(f"[完成] Inspect 段数: {len(inspect_polylines)}")
    total_inspect_length = sum(s['length'] for s in inspect_info)
    print(f"[完成] Inspect 总长度: {total_inspect_length:.1f}px")
    print()

    # =====================================================
    # Step 3: 计算覆盖差集
    # =====================================================
    print("Step 3: 计算覆盖差集...")
    print("-"*70)

    covered_pts, uncovered_pts = compute_point_coverage(
        original_polylines, inspect_polylines, tolerance=3.0
    )

    total_original_pts = len(covered_pts) + len(uncovered_pts)
    coverage_ratio = len(covered_pts) / total_original_pts * 100 if total_original_pts > 0 else 0

    print(f"[完成] 原始总点数: {total_original_pts}")
    print(f"[完成] 覆盖点数: {len(covered_pts)} ({coverage_ratio:.1f}%)")
    print(f"[完成] 未覆盖点数: {len(uncovered_pts)} ({100-coverage_ratio:.1f}%)")
    print()

    # 按线路统计未覆盖情况
    print("[按线路统计未覆盖情况]")
    for line_id, info in original_info.items():
        poly = info['polyline']
        line_uncovered = []
        for pt in poly:
            pt_tuple = (pt[0], pt[1])
            # 检查是否在未覆盖列表中
            for up in uncovered_pts:
                if up[0] == pt[0] and up[1] == pt[1]:
                    line_uncovered.append(pt)
                    break

        uncovered_count = len(line_uncovered)
        uncovered_ratio = uncovered_count / len(poly) * 100 if len(poly) > 0 else 0

        print(f"  {line_id}: {uncovered_count}/{len(poly)} 点未覆盖 ({uncovered_ratio:.1f}%)")
    print()

    # =====================================================
    # Step 4: 生成差集图
    # =====================================================
    print("Step 4: 生成差集图...")
    print("-"*70)

    img = Image.open('data/test.png')
    img_array = np.array(img)

    fig, ax = plt.subplots(figsize=(16, 16))
    ax.imshow(img_array, origin='upper')

    # 绘制原始提取的电网（灰色，半透明）
    for line_id, info in original_info.items():
        poly = info['polyline']
        ax.plot(poly[:, 0], poly[:, 1],
               color='gray', linewidth=2, alpha=0.3,
               linestyle='-')

    # 绘制被覆盖的部分（蓝色，粗线）
    if covered_pts:
        covered_array = np.array(covered_pts)
        # 按原始线路顺序绘制，保证连线正确
        for line_id, info in original_info.items():
            poly = info['polyline']
            covered_in_line = []
            for pt in poly:
                pt_tuple = (pt[0], pt[1])
                if any(up[0] == pt[0] and up[1] == pt[1] for up in covered_pts):
                    covered_in_line.append(pt)
            if len(covered_in_line) > 1:
                covered_in_line = np.array(covered_in_line)
                ax.plot(covered_in_line[:, 0], covered_in_line[:, 1],
                       color='#1f77b4', linewidth=4, alpha=0.8,
                       label='已覆盖' if line_id == list(original_info.keys())[0] else "")

    # 绘制未覆盖的部分（红色，粗线+散点）
    if uncovered_pts:
        uncovered_array = np.array(uncovered_pts)
        ax.scatter(uncovered_array[:, 0], uncovered_array[:, 1],
                  c='red', s=10, alpha=0.8, zorder=10,
                  label='未覆盖' if len(uncovered_pts) > 0 else "")

        # 尝试连接未覆盖点（按线路）
        for line_id, info in original_info.items():
            poly = info['polyline']
            uncovered_in_line = []
            for pt in poly:
                pt_tuple = (pt[0], pt[1])
                if any(up[0] == pt[0] and up[1] == pt[1] for up in uncovered_pts):
                    uncovered_in_line.append(pt)
            if len(uncovered_in_line) > 1:
                uncovered_in_line = np.array(uncovered_in_line)
                ax.plot(uncovered_in_line[:, 0], uncovered_in_line[:, 1],
                       color='red', linewidth=3, alpha=0.7)

    ax.set_title(f'电网覆盖差集图（Map 1）\\n'
                 f'原始提取: {total_original_pts}点 | '
                 f'已覆盖: {len(covered_pts)}点 ({coverage_ratio:.1f}%) | '
                 f'未覆盖: {len(uncovered_pts)}点 ({100-coverage_ratio:.1f}%)',
                 fontsize=14, fontweight='bold')

    # 添加图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='gray', alpha=0.3, label='原始提取的电网'),
        Patch(facecolor='#1f77b4', label='Inspect 已覆盖'),
        Patch(facecolor='red', label='Inspect 未覆盖')
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=12)

    output_path = 'result/latest/coverage_difference_map.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[完成] 差集图已保存: {output_path}")
    print()

    return {
        'total_original_points': total_original_pts,
        'covered_points': len(covered_pts),
        'uncovered_points': len(uncovered_pts),
        'coverage_ratio': coverage_ratio
    }


def generate_segment_source_map():
    """生成段来源图（Map 2）"""
    print("="*70)
    print("Map 2: 段来源图生成")
    print("="*70)
    print()

    # =====================================================
    # Step 1: 获取原始电网和 inspect 段
    # =====================================================
    print("Step 1: 加载数据...")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/test.png',
        flight_height=30
    )

    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()

    # 建立线路映射
    original_lines = {}  # line_id -> polyline
    for line in planner.independent_lines:
        if line.ordered_pixels:
            original_lines[line.id] = np.array(line.ordered_pixels)

    # 读取 mission JSON
    json_path = 'result/latest/mission_output.json'
    with open(json_path, 'r', encoding='utf-8') as f:
        mission = json.load(f)

    inspect_segments = []
    for seg in mission.get('segments', []):
        if seg['type'] == 'inspect':
            inspect_segments.append({
                'segment_id': seg['segment_id'],
                'edge_id': seg.get('edge_id', 'N/A'),
                'geometry': np.array(seg.get('geometry_2d', [])),
                'length': seg['length']
            })

    print(f"[完成] 原始线路: {len(original_lines)} 条")
    print(f"[完成] Inspect 段: {len(inspect_segments)} 段")
    print()

    # =====================================================
    # Step 2: 分析段到线路的映射关系
    # =====================================================
    print("Step 2: 分析段到线路的映射...")
    print("-"*70)

    # 统计每个 edge_id 对应的段
    edge_to_segments = {}  # edge_id -> list of segments
    for seg in inspect_segments:
        edge_id = seg['edge_id']
        if edge_id not in edge_to_segments:
            edge_to_segments[edge_id] = []
        edge_to_segments[edge_id].append(seg)

    print(f"[统计] 共 {len(edge_to_segments)} 个不同的 edge_id")
    print()

    print("[Edge -> Segment 映射详情]")
    for edge_id, segs in sorted(edge_to_segments.items()):
        total_length = sum(s['length'] for s in segs)
        print(f"  {edge_id}: {len(segs)} 段, 总长度 {total_length:.1f}px")
        for s in segs:
            print(f"    - {s['segment_id']}: {s['length']:.1f}px, {len(s['geometry'])} 点")
    print()

    # =====================================================
    # Step 3: 分析覆盖情况
    # =====================================================
    print("Step 3: 分析原始线路覆盖情况...")
    print("-"*70)

    coverage_analysis = {}  # line_id -> coverage info

    for line_id, polyline in original_lines.items():
        # 查找对应的 edge_id
        matching_edge_id = None
        for edge_id in edge_to_segments.keys():
            if edge_id.startswith(line_id.replace('Line_', '')):
                matching_edge_id = edge_id
                break

        if matching_edge_id is None:
            print(f"[警告] {line_id} 没有对应的 EdgeTask")
            coverage_analysis[line_id] = {
                'original_length': np.sum(np.sqrt(np.sum(np.diff(polyline, axis=0)**2, axis=1))),
                'inspect_length': 0,
                'coverage_ratio': 0,
                'segments': []
            }
            continue

        segments = edge_to_segments[matching_edge_id]
        total_inspect_length = sum(s['length'] for s in segments)
        original_length = np.sum(np.sqrt(np.sum(np.diff(polyline, axis=0)**2, axis=1)))

        coverage_analysis[line_id] = {
            'original_length': original_length,
            'inspect_length': total_inspect_length,
            'coverage_ratio': total_inspect_length / original_length * 100 if original_length > 0 else 0,
            'segments': segments,
            'edge_id': matching_edge_id
        }

    print("[原始线路覆盖分析]")
    for line_id, analysis in sorted(coverage_analysis.items()):
        print(f"  {line_id}:")
        print(f"    原始长度: {analysis['original_length']:.1f}px")
        print(f"    Inspect 长度: {analysis['inspect_length']:.1f}px")
        print(f"    覆盖率: {analysis['coverage_ratio']:.1f}%")
        if 'edge_id' in analysis:
            print(f"    对应 Edge: {analysis['edge_id']}")
    print()

    # =====================================================
    # Step 4: 生成段来源图
    # =====================================================
    print("Step 4: 生成段来源图...")
    print("-"*70)

    img = Image.open('data/test.png')
    img_array = np.array(img)

    fig, ax = plt.subplots(figsize=(16, 16))
    ax.imshow(img_array, origin='upper')

    # 为每个 edge_id 分配不同颜色
    edge_ids = list(edge_to_segments.keys())
    colors = plt.cm.tab10(np.linspace(0, 1, max(10, len(edge_ids))))
    edge_colors = {edge_id: colors[i % len(colors)] for i, edge_id in enumerate(edge_ids)}

    # 绘制原始线路（灰色背景）
    for line_id, polyline in original_lines.items():
        ax.plot(polyline[:, 0], polyline[:, 1],
               color='gray', linewidth=3, alpha=0.3,
               linestyle='--', label='原始电网' if line_id == list(original_lines.keys())[0] else "")

    # 绘制 inspect 段，按 edge_id 着色
    for edge_id, segments in edge_to_segments.items():
        color = edge_colors[edge_id]
        for seg in segments:
            geo = seg['geometry']
            if len(geo) > 0:
                ax.plot(geo[:, 0], geo[:, 1],
                       color=color, linewidth=4, alpha=0.9,
                       label=edge_id if seg == segments[0] else "")

                # 标注 segment_id
                mid_idx = len(geo) // 2
                mid_x, mid_y = geo[mid_idx]
                ax.text(mid_x, mid_y, seg['segment_id'].split('_')[-1],
                       fontsize=8, ha='center', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                edgecolor=color, linewidth=1.5, alpha=0.9),
                       zorder=10)

    # 创建图例
    legend_elements = [plt.Line2D([0], [0], color='gray', linewidth=3, alpha=0.3, linestyle='--', label='原始电网')]
    for edge_id, color in edge_colors.items():
        legend_elements.append(plt.Line2D([0], [0], color=color, linewidth=4, label=edge_id))

    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)

    total_original = sum(a['original_length'] for a in coverage_analysis.values())
    total_inspect = sum(a['inspect_length'] for a in coverage_analysis.values())

    ax.set_title(f'Inspect 段来源图（Map 2）\\n'
                 f'原始电网: {len(original_lines)}条 ({total_original:.1f}px) | '
                 f'Inspect 段: {len(inspect_segments)}段 ({total_inspect:.1f}px) | '
                 f'总体覆盖率: {total_inspect/total_original*100:.1f}%',
                 fontsize=14, fontweight='bold')

    output_path = 'result/latest/segment_source_map.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[完成] 段来源图已保存: {output_path}")
    print()

    return coverage_analysis


if __name__ == "__main__":
    print()
    print("="*70)
    print("电网覆盖差集验证")
    print("="*70)
    print()

    # Map 1: 覆盖差集图
    diff_result = generate_coverage_difference_map()

    # Map 2: 段来源图
    coverage_analysis = generate_segment_source_map()

    # 输出总结
    print()
    print("="*70)
    print("验证总结")
    print("="*70)
    print()

    if diff_result:
        print(f"[覆盖差集统计]")
        print(f"  原始总点数: {diff_result['total_original_points']}")
        print(f"  已覆盖点数: {diff_result['covered_points']} ({diff_result['coverage_ratio']:.1f}%)")
        print(f"  未覆盖点数: {diff_result['uncovered_points']} ({100-diff_result['coverage_ratio']:.1f}%)")
        print()

    print("[线路覆盖分析]")
    total_original = 0
    total_inspect = 0
    for line_id, analysis in sorted(coverage_analysis.items()):
        original = analysis['original_length']
        inspect = analysis['inspect_length']
        ratio = analysis['coverage_ratio']
        total_original += original
        total_inspect += inspect

        status = "OK" if ratio >= 90 else ("?" if ratio >= 50 else "NG")
        print(f"  {line_id}: {original:.1f}px -> {inspect:.1f}px ({ratio:.1f}%) {status}")

    overall_ratio = total_inspect / total_original * 100 if total_original > 0 else 0
    print(f"  总计: {total_original:.1f}px → {total_inspect:.1f}px ({overall_ratio:.1f}%)")
    print()

    print("[输出文件]")
    print("  - result/latest/coverage_difference_map.png (Map 1: 覆盖差集)")
    print("  - result/latest/segment_source_map.png (Map 2: 段来源)")
    print()
