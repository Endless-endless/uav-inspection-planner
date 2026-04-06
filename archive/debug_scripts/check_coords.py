"""检查坐标系统"""
import sys
sys.path.append('.')
from planner.powerline_planner_v3_final import PowerlinePlannerV3
import numpy as np

# 生成简单的地形
terrain = np.zeros((960, 916), dtype=np.float32)

# 运行规划
planner = PowerlinePlannerV3(
    image_path='data/1.png',
    flight_height=30
)

# 执行所有步骤
planner.step1_extract_redline_hsv()
planner.step2_fix_breaks()
planner.step3_skeletonize()
planner.step4_extract_independent_lines()
planner.step5_generate_line_inspection_points()
planner.step6_map_to_3d()  # 这会设置3D坐标
planner.step6_smooth_terrain(terrain)  # 设置地形
planner.step7_build_task_graph()
planner.step8_optimize_task_order()
planner.step9_build_g_path()
planner.step10_prepare_vis()

if planner and hasattr(planner, 'g_path_3d') and planner.g_path_3d:
    path_3d = planner.g_path_3d
    print(f"路径点数: {len(path_3d)}")

    # 检查坐标范围
    x_vals = [p[0] for p in path_3d]
    y_vals = [p[1] for p in path_3d]
    z_vals = [p[2] for p in path_3d]

    print(f"X范围: {min(x_vals):.1f} ~ {max(x_vals):.1f}")
    print(f"Y范围: {min(y_vals):.1f} ~ {max(y_vals):.1f}")
    print(f"Z范围: {min(z_vals):.1f} ~ {max(z_vals):.1f}")

    # 检查图像尺寸
    if hasattr(planner, 'image'):
        w, h = planner.image.size
        print(f"图像尺寸: {w} x {h}")

    # 诊断
    print(f"\n诊断:")
    print(f"  路径X范围 ({min(x_vals):.0f}-{max(x_vals):.0f}) vs 图像宽度 ({w})")
    print(f"  路径Y范围 ({min(y_vals):.0f}-{max(y_vals):.0f}) vs 图像高度 ({h})")
    print(f"  路径Z范围 ({min(z_vals):.1f}-{max(z_vals):.1f}) vs 地图Z (0)")

    if max(z_vals) > 100:
        print(f"\n[警告] 路径Z值过大！地图(Z=0)可能被缩得太小而看不见")
        print(f"建议：调整路径Z坐标或调整scene的aspectratio")
