"""
电网提取验收脚本

功能：
1. 验证 data/test.png 中的红色电网提取是否正确
2. 输出每条独立电网的统计信息
3. 生成验收图（不同颜色标注不同电网）
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

from planner.powerline_planner_v3_final import PowerlinePlannerV3


def verify_powerline_extraction():
    """电网提取验收"""
    print("="*70)
    print("电网提取验收")
    print("="*70)
    print()

    # =====================================================
    # Step 1: 执行电网提取
    # =====================================================
    print("Step 1: 执行电网提取...")
    print("-"*70)

    planner = PowerlinePlannerV3(
        image_path='data/test.png',
        flight_height=30
    )

    planner.step1_extract_redline_hsv()
    planner.step2_fix_breaks()
    planner.step3_skeletonize()
    planner.step4_extract_independent_lines()

    print(f"[完成] 识别出 {len(planner.independent_lines)} 条独立电网")
    print()

    # =====================================================
    # Step 2: 统计每条电网信息
    # =====================================================
    print("Step 2: 统计每条电网信息...")
    print("-"*70)

    stats = []
    for i, line in enumerate(planner.independent_lines):
        # 计算 bbox
        if line.polyline:
            x_coords = [p[0] for p in line.polyline]
            y_coords = [p[1] for p in line.polyline]
            bbox = {
                'min_x': min(x_coords),
                'max_x': max(x_coords),
                'min_y': min(y_coords),
                'max_y': max(y_coords)
            }
        else:
            bbox = {'min_x': 0, 'max_x': 0, 'min_y': 0, 'max_y': 0}

        # 计算端点数量
        endpoint_count = len([p for p in line.inspection_points
                            if p.point_type == 'endpoint'])

        stat = {
            'line_id': line.line_id,
            'length': line.length_2d,
            'num_points': len(line.polyline) if line.polyline else 0,
            'bbox': bbox,
            'endpoint_count': endpoint_count
        }
        stats.append(stat)

        print(f"  {line.line_id}:")
        print(f"    长度: {stat['length']:.1f} px")
        print(f"    点数: {stat['num_points']}")
        print(f"    端点: {stat['endpoint_count']}")
        print(f"    BBOX: x[{stat['bbox']['min_x']:.0f}, {stat['bbox']['max_x']:.0f}] "
              f"y[{stat['bbox']['min_y']:.0f}, {stat['bbox']['max_y']:.0f}]")
    print()

    # =====================================================
    # Step 3: 生成验收图
    # =====================================================
    print("Step 3: 生成验收图...")
    print("-"*70)

    # 读取原图
    img = Image.open('data/test.png')
    img_array = np.array(img)

    fig, ax = plt.subplots(figsize=(16, 16))

    # 显示底图
    ax.imshow(img_array, origin='upper')

    # 为每条电网分配不同颜色
    colors = plt.cm.tab10(np.linspace(0, 1, len(planner.independent_lines)))
    line_colors = {line.line_id: colors[i] for i, line in enumerate(planner.independent_lines)}

    # 绘制每条电网
    for line in planner.independent_lines:
        if line.polyline:
            polyline = np.array(line.polyline)
            ax.plot(polyline[:, 0], polyline[:, 1],
                   color=line_colors[line.line_id],
                   linewidth=4, alpha=0.9,
                   label=line.line_id)

            # 标注线路编号
            mid_idx = len(polyline) // 2
            mid_x, mid_y = polyline[mid_idx]
            ax.text(mid_x, mid_y, line.line_id.replace('Line_', 'L'),
                   fontsize=10, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                            edgecolor=line_colors[line.line_id], linewidth=2,
                            alpha=0.9),
                   zorder=10)

    # 标注端点
    for line in planner.independent_lines:
        for pt in line.inspection_points:
            if pt.point_type == 'endpoint':
                ax.scatter(pt.pixel_position[0], pt.pixel_position[1],
                          c='red', marker='o', s=30, zorder=5,
                          edgecolors='white', linewidths=1)

    ax.set_title(f'电网提取验收图 - 识别出 {len(planner.independent_lines)} 条独立电网',
                 fontsize=16, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)

    output_path = 'result/latest/powerline_extraction_verification.png'
    os.makedirs('result/latest', exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[完成] 验收图已保存: {output_path}")
    print()

    # =====================================================
    # Step 4: 输出结构化统计
    # =====================================================
    print("Step 4: 结构化统计...")
    print("-"*70)

    summary = {
        'total_lines': len(planner.independent_lines),
        'line_stats': stats,
        'total_length': sum(s['length'] for s in stats),
        'total_points': sum(s['num_points'] for s in stats),
        'total_endpoints': sum(s['endpoint_count'] for s in stats)
    }

    print(f"总电网数: {summary['total_lines']}")
    print(f"总长度: {summary['total_length']:.1f} px")
    print(f"总点数: {summary['total_points']}")
    print(f"总端点: {summary['total_endpoints']}")
    print()

    # 输出 JSON
    import json
    json_path = 'result/latest/powerline_extraction_stats.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        # 转换为可序列化的格式
        serializable_stats = []
        for s in stats:
            s_copy = s.copy()
            s_copy['bbox'] = {k: float(v) if isinstance(v, (np.integer, np.floating)) else v
                            for k, v in s['bbox'].items()}
            serializable_stats.append(s_copy)

        json.dump({
            'summary': {
                'total_lines': summary['total_lines'],
                'total_length': float(summary['total_length']),
                'total_points': summary['total_points'],
                'total_endpoints': summary['total_endpoints']
            },
            'lines': serializable_stats
        }, f, indent=2, ensure_ascii=False)

    print(f"[完成] 统计 JSON 已保存: {json_path}")
    print()

    return planner, stats


def verify_inspect_vs_connect_segments():
    """验收 inspect 和 connect 段"""
    print("="*70)
    print("Inspect vs Connect 段验收")
    print("="*70)
    print()

    import json
    json_path = 'result/latest/mission_output.json'
    if not os.path.exists(json_path):
        print(f"[跳过] mission JSON 不存在: {json_path}")
        return

    with open(json_path, 'r', encoding='utf-8') as f:
        mission = json.load(f)

    segments = mission.get('segments', [])
    print(f"总段数: {len(segments)}")

    inspect_count = 0
    connect_count = 0
    inspect_length = 0
    connect_length = 0

    # 绘制验收图
    from PIL import Image
    img = Image.open('data/test.png')
    img_array = np.array(img)

    fig, ax = plt.subplots(figsize=(16, 16))
    ax.imshow(img_array, origin='upper')

    for seg in segments:
        if seg['type'] == 'inspect':
            inspect_count += 1
            inspect_length += seg['length']
            geo = seg.get('geometry_2d', [])
            if geo:
                geo_np = np.array(geo)
                ax.plot(geo_np[:, 0], geo_np[:, 1],
                       color='#1f77b4', linewidth=5, alpha=0.8,
                       label='inspect' if inspect_count == 1 else "")
        else:
            connect_count += 1
            connect_length += seg['length']
            geo = seg.get('geometry_2d', [])
            if geo:
                geo_np = np.array(geo)
                ax.plot(geo_np[:, 0], geo_np[:, 1],
                       color='#ff7f0e', linewidth=3, alpha=0.6,
                       linestyle='--', label='connect' if connect_count == 1 else "")

    ax.set_title(f'Inspect vs Connect 段验收图\\nInspect: {inspect_count}段 ({inspect_length:.1f}px) | Connect: {connect_count}段 ({connect_length:.1f}px)',
                 fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')

    output_path = 'result/latest/inspect_connect_verification.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Inspect 段: {inspect_count} 段, {inspect_length:.1f} px")
    print(f"Connect 段: {connect_count} 段, {connect_length:.1f} px")
    print(f"[完成] 验收图已保存: {output_path}")
    print()


if __name__ == "__main__":
    # 电网提取验收
    planner, stats = verify_powerline_extraction()

    print()
    print("="*70)
    print("验收结论")
    print("="*70)
    print()

    # 分析结果
    print(f"1. 识别出 {len(stats)} 条独立电网:")
    for s in stats:
        print(f"   - {s['line_id']}: {s['length']:.1f}px, {s['endpoint_count']} 个端点")

    print()
    print("2. 电网之间的跳跃分析:")
    print("   - 如果源图本身是断开的独立电网，则需要 connect segment")
    print("   - 如果源图是连续的但被错误分割，则需要修复提取逻辑")

    print()
    print("3. 漏检/误合并检查:")
    print("   - 请对比验收图与原始 data/test.png")
    print("   - 检查是否有红线被遗漏")
    print("   - 检查是否有红线被错误合并")

    print()
    print("[输出文件]")
    print("  - result/latest/powerline_extraction_verification.png")
    print("  - result/latest/powerline_extraction_stats.json")

    # Inspect vs Connect 验收
    print()
    verify_inspect_vs_connect_segments()

    print()
    print("[输出文件]")
    print("  - result/latest/inspect_connect_verification.png")
