"""
事件驱动UAV动画测试

验证：
1. UAV到达巡检点时变红+放大+显示文本
2. 巡检点停顿一帧（模拟拍照）
3. 动画流畅快速
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.visualization_enhanced import create_animation_view, downsample_path
from core.inspection_tasks import generate_inspection_points, InspectionPoint

def create_test_path():
    """创建测试路径"""
    path = []
    for i in range(0, 100, 5):
        x = i
        y = i * 0.3
        z = 50 + np.sin(i / 10) * 8
        path.append((x, y, z))
    return path

def create_test_inspection_points(path_3d):
    """创建测试任务点"""
    inspection_points = []

    # 每5个点设置一个拍照点
    for i in range(0, len(path_3d), 5):
        point = InspectionPoint(
            position=path_3d[i],
            index=i,
            is_photo=True,
            priority="normal",
            gimbal_angle=-30,
            action="photo",
            cost=0,
            reason="test_photo"
        )
        inspection_points.append(point)

    return inspection_points

print("=" * 70)
print("      事件驱动UAV动画测试")
print("=" * 70)

# 创建测试路径
path_3d = create_test_path()
print(f"\n[测试] 路径点数: {len(path_3d)}")

# 创建测试任务点
inspection_points = create_test_inspection_points(path_3d)
print(f"[测试] 任务点数: {len(inspection_points)}")

# 显示任务点信息
print(f"\n[任务点列表]")
for p in inspection_points:
    print(f"  索引 {p.index}: {p.action} | {p.priority}")

# 创建事件驱动动画
print(f"\n[创建动画]")
fig = create_animation_view(
    path_3d=path_3d,
    inspection_points=inspection_points,
    output_file="output/event_driven_animation.html",
    downsample_step=3
)

print(f"\n" + "=" * 70)
print(f"                    完成！")
print(f"=" * 70)

print(f"\n[输出文件]")
print(f"  - output/event_driven_animation.html")

print(f"\n[动画特性]")
print(f"  1. UAV正常飞行: 橙色，大小5")
print(f"  2. 到达拍照点: 变红，大小8，显示 'WP X - Photo'")
print(f"  3. 拍照点停顿: 重复一帧（模拟拍照动作）")
print(f"  4. 播放速度: duration=50ms，快速流畅")

print(f"\n[提示]")
print(f"  打开 HTML 文件，点击 'Play' 按钮观看动画")
print(f"  注意观察 UAV 在拍照点时的变化")
