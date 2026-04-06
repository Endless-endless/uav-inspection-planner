"""
调试 L_003 和 L_004 的拓扑切分
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planner.powerline_planner_v3_final import PowerlinePlannerV3

planner = PowerlinePlannerV3(
    image_path='data/test.png',
    flight_height=30
)

# 执行到 step7_5
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()
planner.step4_extract_independent_lines()

print("\n" + "="*70)
print("L_003 和 L_004 原始信息")
print("="*70)
for line in planner.independent_lines:
    if line.id in ['L_003', 'L_004']:
        print(f"{line.id}:")
        print(f"  总点数: {len(line.ordered_pixels)}")
        print(f"  总长度: {line.length_2d:.1f}px")
        print(f"  端点: {line.endpoints}")
        print()

# 执行 step7_5 构建拓扑
planner.step7_5_build_topo()

print("\n" + "="*70)
print("L_003 和 L_004 的拓扑边信息")
print("="*70)

# 查找 L_003 和 L_004 的拓扑边
for edge_id, edge in planner.topo_graph.edges.items():
    if 'L_003' in edge_id or 'L_004' in edge_id:
        print(f"{edge_id}:")
        print(f"  u: {edge.u}, v: {edge.v}")
        print(f"  polyline 长度: {len(edge.polyline)} 点")
        print(f"  len2d: {edge.len2d:.1f}px")
        print(f"  is_straight: {edge.is_straight}")
        print(f"  split_reason: {edge.split_reason}")

        # 计算在原始线路中的索引范围
        line = None
        for l in planner.independent_lines:
            if l.id == edge.line_id:
                line = l
                break

        if line and edge.polyline:
            first_pt = edge.polyline[0]
            last_pt = edge.polyline[-1]

            # 查找这些点在原始 ordered_pixels 中的索引
            try:
                first_idx = line.ordered_pixels.index(first_pt)
                last_idx = line.ordered_pixels.index(last_pt)
                print(f"  原始线路索引范围: [{first_idx}, {last_idx}] (共 {last_idx - first_idx + 1} 点)")
                print(f"  原始线路总点数: {len(line.ordered_pixels)}")
                print(f"  覆盖比例: {(last_idx - first_idx + 1) / len(line.ordered_pixels) * 100:.1f}%")
            except ValueError:
                print(f"  [警告] 无法在原始线路中找到端点")
        print()

# 检查拓扑节点
print("\n" + "="*70)
print("L_003 和 L_004 上的拓扑节点")
print("="*70)

for node in planner.topo_nodes:
    if 'L_003' in node.line_ids or 'L_004' in node.line_ids:
        line_ids_str = ', '.join(node.line_ids)
        print(f"  {node.id}:")
        print(f"    kind: {node.kind}")
        print(f"    line_ids: {line_ids_str}")
        print(f"    pt_indices: {node.pt_indices}")
        print(f"    pos2d: {node.pos2d}")
        print()
