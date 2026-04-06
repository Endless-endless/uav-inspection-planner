"""
UAV导航功能演示 - 完整执行路径版

演示从任意输入点导航到最近巡检点并执行剩余任务的完整流程
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.navigator import (
    find_nearest_inspection_point,
    navigate_from_input,
    create_navigation_visualization,
    create_navigation_animation
)
from core.inspection_tasks import generate_inspection_points

print("=" * 70)
print("      UAV导航功能演示 - 完整执行路径")
print("=" * 70)

# =====================================================
# 第1步：创建测试路径
# =====================================================
print("\n[第1步] 创建测试路径...")

# 创建一个螺旋上升的路径
path_3d = []
for i in range(50):
    angle = i * 0.2
    x = 100 + 50 * np.cos(angle)
    y = 100 + 50 * np.sin(angle)
    z = 50 + i * 2  # 逐渐升高
    path_3d.append((x, y, z))

print(f"  路径点数: {len(path_3d)}")

# =====================================================
# 第2步：生成巡检点
# =====================================================
print("\n[第2步] 生成巡检点...")

wind_vector = np.array([5.0, 0.0])
inspection_points = generate_inspection_points(
    path_3d,
    wind_vector,
    photo_interval=8,
    height_change_threshold=5.0,
    high_cost_factor=0.5
)

photo_count = sum(1 for p in inspection_points if p.is_photo)
print(f"  拍照点数: {photo_count}")

# =====================================================
# 第3步：测试导航功能
# =====================================================
print("\n[第3步] 测试导航功能...")

test_inputs = [
    # (name, position)
    ("路径起点附近", (80, 100, 50)),
    ("路径中部附近", (150, 100, 100)),
    ("路径远处", (200, 200, 150)),
    ("路径终点附近", (100, 80, 140)),
]

for name, input_pos in test_inputs:
    print(f"\n  测试: {name}")
    print(f"    输入点: {input_pos}")

    nav_info = navigate_from_input(input_pos, path_3d, inspection_points, connection_points=8)

    print(f"    最近巡检点: 索引 {nav_info['nearest_index']}")
    print(f"    接入距离: {nav_info['distance']:.1f}m")
    print(f"    接入段点数: {len(nav_info['connection_segment'])}")
    print(f"    剩余拍照点: {nav_info['remaining_photo_count']}")
    print(f"    完整执行路径点: {len(nav_info['full_execution_path'])}")

# =====================================================
# 第4步：创建静态可视化
# =====================================================
print("\n[第4步] 创建导航可视化...")

# 选择一个有代表性的输入点
input_pos = (150, 100, 100)  # 路径中部附近

fig, nav_info = create_navigation_visualization(
    input_pos=input_pos,
    path_3d=path_3d,
    inspection_points=inspection_points,
    output_file="output/navigation_view.html",
    connection_points=8
)

# =====================================================
# 第5步：创建执行动画
# =====================================================
print("\n[第5步] 创建执行动画...")

fig_anim, nav_info_anim = create_navigation_animation(
    input_pos=input_pos,
    path_3d=path_3d,
    inspection_points=inspection_points,
    output_file="output/navigation_animation.html",
    connection_points=8,
    downsample_step=2
)

# =====================================================
# 完成
# =====================================================
print("\n" + "=" * 70)
print("                    演示完成！")
print("=" * 70)

print(f"\n[输出文件]")
print(f"  - output/navigation_view.html (静态可视化)")
print(f"  - output/navigation_animation.html (执行动画)")

print(f"\n[功能说明]")
print(f"  1. 输入任意位置坐标")
print(f"  2. 自动找到最近的拍照巡检点")
print(f"  3. 生成平滑接入段（线性插值8个点）")
print(f"  4. 构建完整执行路径：输入点 → 接入段 → 巡检段")
print(f"  5. 动画演示UAV完整执行流程")

print(f"\n[可视化元素]")
print(f"  - 紫色圆点: 输入点（起飞位置）")
print(f"  - 红色钻石: 最近巡检点（接入点）")
print(f"  - 绿色加粗线: 完整执行路径")
print(f"  - 灰色虚线: 原始完整路径（参考）")
print(f"  - 黄色圆点: 所有巡检点")

print(f"\n[动画流程]")
print(f"  1. UAV从输入点起飞（橙色）")
print(f"  2. 沿接入段飞行，显示 'Connecting...'")
print(f"  3. 到达最近巡检点，显示 'Arrived'")
print(f"  4. 进入巡检段，在拍照点触发红色+停顿")
print(f"  5. 执行剩余所有巡检任务")

# =====================================================
# API使用示例
# =====================================================
print(f"\n[API使用示例]")
print(f"```python")
print(f"from core.navigator import navigate_from_input")
print(f"")
print(f"# 输入点坐标")
print(f"input_pos = (150, 100, 100)")
print(f"")
print(f"# 获取导航信息（包含完整执行路径）")
print(f"nav_info = navigate_from_input(input_pos, path_3d, inspection_points)")
print(f"")
print(f"# 返回结果:")
print(f"# - nearest_point: 最近巡检点对象")
print(f"# - nearest_index: 在路径中的索引")
print(f"# - distance: 接入距离（米）")
print(f"# - connection_segment: 接入段路径点")
print(f"# - remaining_path: 剩余巡检路径")
print(f"# - full_execution_path: 完整执行路径（接入段+巡检段）")
print(f"# - remaining_points: 剩余巡检点列表")
print(f"# - remaining_photo_count: 剩余拍照点数量")
print(f"```")

print(f"\n[核心改进]")
print(f"  - 动画从输入点开始（不是从巡检点开始）")
print(f"  - 接入段使用线性插值平滑过渡")
print(f"  - 完整执行路径统一展示")
print(f"  - 巡检点正确触发视觉反馈")
