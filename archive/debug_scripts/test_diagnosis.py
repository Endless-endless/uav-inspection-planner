"""诊断Stage2地图底图问题"""
import numpy as np
from PIL import Image
import plotly.graph_objects as go

# 加载图像
img_path = 'data/1.png'
img = Image.open(img_path)
img_array = np.array(img)
h, w = img_array.shape[:2]
print(f"图像尺寸: {w} x {h}")

# 创建测试图形
fig = go.Figure()

# 1. 地图底图
X, Y = np.meshgrid(np.arange(w), np.arange(h))
Z = np.zeros_like(X, dtype=float)

print(f"地图底图:")
print(f"  X范围: {X.min()} ~ {X.max()}")
print(f"  Y范围: {Y.min()} ~ {Y.max()}")
print(f"  Z范围: {Z.min()} ~ {Z.max()}")
print(f"  图像数组形状: {img_array.shape}")

fig.add_trace(go.Surface(
    x=X, y=Y, z=Z,
    surfacecolor=np.flipud(img_array),
    colorscale='Viridis',
    showscale=False,
    opacity=1.0,
    name='Map'
))

# 2. 添加测试路径点（使用像素坐标）
test_x = [100, 200, 300]
test_y = [100, 200, 300]
test_z = [30, 30, 30]  # 飞行高度

print(f"\n测试路径:")
print(f"  X范围: {min(test_x)} ~ {max(test_x)}")
print(f"  Y范围: {min(test_y)} ~ {max(test_y)}")
print(f"  Z范围: {min(test_z)} ~ {max(test_z)}")

fig.add_trace(go.Scatter3d(
    x=test_x, y=test_y, z=test_z,
    mode='markers',
    marker=dict(size=10, color='red'),
    name='Test Path'
))

fig.update_layout(
    scene=dict(
        aspectmode='data',
        camera=dict(eye=dict(x=1.5, y=1.5, z=1.2))
    ),
    width=800,
    height=600
)

fig.write_html('result/test_diagnosis.html')
print("\n已生成 result/test_diagnosis.html")
print("请在浏览器中打开，查看地图底图是否显示正常")
