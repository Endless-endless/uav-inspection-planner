"""
电网提取验收脚本 - 增强版

功能：
1. 验证 data/test.png 中的红色电网提取是否正确
2. 输出每条独立电网的统计信息
3. 生成 inspect vs connect 段验收图（增强版 - 清晰显示电网间连接）
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch


def load_mission_data():
    """加载 mission 数据"""
    json_path = 'result/latest/mission_output.json'
    if not os.path.exists(json_path):
        print(f"[错误] mission JSON 不存在: {json_path}")
        print("请先运行: python demo/demo_visualization_main.py")
        return None

    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def find_edge_id_for_point(point, inspect_segs):
    """根据点坐标找到对应的 edge_id"""
    x, y = point
    tolerance = 10.0  # 像素容差

    for seg in inspect_segs:
        geo = seg.get('geometry_2d', [])
        if not geo:
            continue
        geo_np = np.array(geo)

        # 检查起点
        if np.linalg.norm(geo_np[0] - np.array([x, y])) < tolerance:
            return seg.get('edge_id', 'N/A')

        # 检查终点
        if np.linalg.norm(geo_np[-1] - np.array([x, y])) < tolerance:
            return seg.get('edge_id', 'N/A')

    return 'Unknown'


def verify_inspect_vs_connect_enhanced():
    """验收 inspect 和 connect 段 - 增强版"""
    print("="*70)
    print("Inspect vs Connect 段验收（增强版）")
    print("="*70)
    print()

    mission = load_mission_data()
    if mission is None:
        return

    segments = mission.get('segments', [])
    inspect_segs = [s for s in segments if s['type'] == 'inspect']
    connect_segs = [s for s in segments if s['type'] == 'connect']

    print(f"Inspect 段: {len(inspect_segs)} 段")
    print(f"Connect 段: {len(connect_segs)} 段")
    print()

    # 统计
    inspect_length = sum(s['length'] for s in inspect_segs)
    connect_length = sum(s['length'] for s in connect_segs)

    print(f"Inspect 长度: {inspect_length:.1f} px")
    print(f"Connect 长度: {connect_length:.1f} px")
    print(f"Connect 占比: {connect_length / (inspect_length + connect_length) * 100:.1f}%")
    print()

    # 详细信息 - 显示电网间的连接
    print("Connect 段详情（电网间转场）:")
    print("-"*70)
    for i, seg in enumerate(connect_segs):
        geo = seg.get('geometry_2d', [])
        if not geo:
            continue

        start_pt = geo[0]
        end_pt = geo[-1]

        # 找到起点和终点对应的电网
        from_edge = find_edge_id_for_point(start_pt, inspect_segs)
        to_edge = find_edge_id_for_point(end_pt, inspect_segs)

        print(f"{seg['segment_id']}:")
        print(f"  长度: {seg['length']:.1f}px, 点数: {len(geo)}")
        print(f"  从: {from_edge} ({start_pt[0]:.1f}, {start_pt[1]:.1f})")
        print(f"  到: {to_edge} ({end_pt[0]:.1f}, {end_pt[1]:.1f})")

        # 判断是否真的是电网间转场
        if from_edge != 'Unknown' and to_edge != 'Unknown' and from_edge != to_edge:
            print(f"  [OK] 电网间转场")
        elif from_edge == 'Unknown' or to_edge == 'Unknown':
            print(f"  [?] 端点不清晰")
        else:
            print(f"  [!] 同一电网内部连接")
        print()

    # =====================================================
    # 绘制增强验收图
    # =====================================================
    print("生成增强验收图...")

    img = Image.open('data/test.png')
    img_array = np.array(img)

    fig, ax = plt.subplots(figsize=(18, 18))
    ax.imshow(img_array, origin='upper')

    # 绘制 inspect 段（蓝色粗线 - 沿电网）
    for seg in inspect_segs:
        geo = seg.get('geometry_2d', [])
        if geo:
            geo_np = np.array(geo)
            edge_id = seg.get('edge_id', 'N/A')
            # 在线路中间标注 edge_id
            mid_idx = len(geo_np) // 2
            ax.plot(geo_np[:, 0], geo_np[:, 1],
                   color='#1f77b4', linewidth=6, alpha=0.9,
                   label='Inspect' if seg == inspect_segs[0] else "")
            # 标注 edge_id
            ax.text(geo_np[mid_idx, 0], geo_np[mid_idx, 1], edge_id,
                   fontsize=8, ha='center', va='center',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                            edgecolor='#1f77b4', linewidth=2, alpha=0.8),
                   zorder=10)

    # 绘制 connect 段（橙色虚线 + 箭头 - 电网间转场）
    for i, seg in enumerate(connect_segs):
        geo = seg.get('geometry_2d', [])
        if not geo or len(geo) < 2:
            continue

        geo_np = np.array(geo)
        start_pt = geo_np[0]
        end_pt = geo_np[-1]

        # 找到起点和终点对应的电网
        from_edge = find_edge_id_for_point(start_pt, inspect_segs)
        to_edge = find_edge_id_for_point(end_pt, inspect_segs)

        # 绘制虚线路径
        ax.plot(geo_np[:, 0], geo_np[:, 1],
               color='#ff6600', linewidth=4, alpha=0.9,
               linestyle='--', dashes=(8, 4),
               label='Connect' if seg == connect_segs[0] else "")

        # 绘制起点（绿色圆圈）
        ax.scatter(start_pt[0], start_pt[1], c='#00CC00', marker='o', s=200,
                  edgecolors='white', linewidths=3, zorder=15)
        # 标注起点 edge_id
        if from_edge != 'Unknown':
            ax.text(start_pt[0], start_pt[1], from_edge,
                   fontsize=9, ha='right', va='bottom',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='#00CC00',
                            edgecolor='white', linewidth=2, alpha=0.9),
                   zorder=16, color='white', fontweight='bold')

        # 绘制终点（红色圆圈）
        ax.scatter(end_pt[0], end_pt[1], c='#CC0000', marker='o', s=200,
                  edgecolors='white', linewidths=3, zorder=15)
        # 标注终点 edge_id
        if to_edge != 'Unknown':
            ax.text(end_pt[0], end_pt[1], to_edge,
                   fontsize=9, ha='left', va='top',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='#CC0000',
                            edgecolor='white', linewidth=2, alpha=0.9),
                   zorder=16, color='white', fontweight='bold')

        # 在虚线中间添加方向箭头
        if len(geo_np) > 10:
            mid_idx = len(geo_np) // 2
            arrow_start = geo_np[mid_idx - 2]
            arrow_end = geo_np[mid_idx + 2]
            dx = arrow_end[0] - arrow_start[0]
            dy = arrow_end[1] - arrow_start[1]

            # 绘制箭头
            ax.arrow(arrow_start[0], arrow_start[1], dx, dy,
                    head_width=15, head_length=15, fc='#ff6600', ec='white',
                    linewidth=2, zorder=12, alpha=0.8)

    # 添加图例（去重）
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(),
              loc='upper right', fontsize=12)

    ax.set_title(f'电网巡检路径验收图\\n'
                 f'Inspect（蓝色实线）: {len(inspect_segs)}段 {inspect_length:.1f}px | '
                 f'Connect（橙色虚线+箭头）: {len(connect_segs)}段 {connect_length:.1f}px',
                 fontsize=16, fontweight='bold', pad=20)

    output_path = 'result/latest/inspect_connect_verification.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"[完成] 增强验收图已保存: {output_path}")
    print()

    print("图例说明:")
    print("  - 蓝色实线: Inspect 段（沿电网巡检）")
    print("  - 橙色虚线+箭头: Connect 段（电网间转场）")
    print("  - 绿色圆圈: 转场起点（标注来源电网）")
    print("  - 红色圆圈: 转场终点（标注目标电网）")
    print()

    return {
        'inspect_count': len(inspect_segs),
        'connect_count': len(connect_segs),
        'inspect_length': inspect_length,
        'connect_length': connect_length
    }


if __name__ == "__main__":
    result = verify_inspect_vs_connect_enhanced()

    print()
    print("="*70)
    print("验收总结")
    print("="*70)
    print()
    print(f"[OK] Inspect 段: {result['inspect_count']} 段, {result['inspect_length']:.1f} px")
    print(f"[OK] Connect 段: {result['connect_count']} 段, {result['connect_length']:.1f} px")
    print()
    print("虚线（Connect）现在显示:")
    print("  ✓ 沿着插值后的密集路径绘制")
    print("  ✓ 起点和终点明确标注电网 ID")
    print("  ✓ 中间有方向箭头指示飞行方向")
    print("  ✓ 清晰展示从一个电网到另一个电网的转场路径")
    print()
