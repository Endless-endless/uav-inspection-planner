"""
可视化增强模块

功能：
1. create_animation_view - UAV沿优化路径飞行的动画
2. create_animation_view_with_tasks - 带任务点的动画
3. create_analysis_view - 原始vs优化路径对比分析
4. create_analysis_view_with_tasks - 带任务点的对比分析
"""

import numpy as np
import plotly.graph_objects as go
import os


def downsample_path(path_3d, step=3):
    """
    路径降采样：每隔step个点取一个点

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        step: 采样步长（默认3）

    Returns:
        降采样后的路径
    """
    return path_3d[::step]


def create_animation_view(path_3d, inspection_points=None, output_file="output/animation.html", downsample_step=3):
    """
    创建UAV沿优化路径飞行的动画视图（增强版：带导航输入面板）

    特性：
    - 页面中嵌入坐标输入面板
    - 用户可输入任意起点，自动接入最近巡检点
    - 点击按钮后生成并播放执行动画
    - 不显示黄色小圆点（巡检点标记）

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        inspection_points: 巡检任务点列表 (List[InspectionPoint])
        output_file: 输出HTML文件路径
        downsample_step: 路径降采样步长（默认3）
    """
    print("[动画视图] 创建UAV巡检动画（增强版：带导航输入面板）...")

    import json

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    original_count = len(path_3d)

    # 【1️⃣】提取巡检点索引（并验证）
    photo_indices = set()
    inspection_points_data = []  # 用于JavaScript的数据结构

    if inspection_points:
        for p in inspection_points:
            point_data = {
                'index': p.index,
                'position': list(p.position),
                'isPhoto': p.is_photo
            }
            inspection_points_data.append(point_data)

            if p.is_photo:
                if 0 <= p.index < len(path_3d):
                    photo_indices.add(p.index)
                else:
                    print(f"  [警告] 巡检点索引 {p.index} 超出路径范围 (0-{len(path_3d)-1})")

        print(f"  [任务] 检测到 {len(photo_indices)} 个拍照点")
        if photo_indices:
            print(f"  [验证] 拍照点索引: {sorted(photo_indices)}")
            print(f"  [验证] 路径范围: 0-{len(path_3d)-1}")

    # 【2️⃣】构建"保留巡检点"的降采样路径
    path_vis = []
    for i, p in enumerate(path_3d):
        if i % downsample_step == 0 or i in photo_indices:
            path_vis.append((i, p))

    path_indices = [item[0] for item in path_vis]
    path_coords = [item[1] for item in path_vis]
    path_x = [p[0] for p in path_coords]
    path_y = [p[1] for p in path_coords]
    path_z = [p[2] for p in path_coords]

    print(f"  [降采样] 路径点: {original_count} → {len(path_vis)}")
    print(f"  [保留] 强制保留 {len(photo_indices)} 个巡检点")

    # 【3️⃣】创建初始图形（不带动画帧，不自动播放）
    fig = go.Figure()

    # Trace 0: 原始路径（灰色虚线，参考用）
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='lightgray', width=2, dash='dot'),
        name='Original Path',
        hovertemplate='Original<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 1: 执行路径（绿色加粗，初始=完整路径）
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='green', width=4),
        name='Execution Path',
        hovertemplate='Execution<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 2: 输入点（初始隐藏，动态添加）
    fig.add_trace(go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode='markers',
        marker=dict(size=12, color='purple', symbol='circle',
                   line=dict(color='white', width=2)),
        name='Input Point',
        hovertemplate='Input Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 3: 最近巡检点（初始隐藏，动态添加）
    fig.add_trace(go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode='markers',
        marker=dict(size=15, color='red', symbol='diamond',
                   line=dict(color='yellow', width=2)),
        name='Nearest Inspection Point',
        hovertemplate='Nearest Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 4: UAV（初始在路径起点）
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]],
        y=[path_y[0]],
        z=[path_z[0]],
        mode='markers',
        marker=dict(size=5, color='orange'),
        name='UAV',
        hovertemplate='UAV<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 【4️⃣】配置布局（不带播放按钮，初始不播放）
    fig.update_layout(
        title=dict(
            text="UAV Inspection Flight - Enter Coordinates to Start",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            dragmode='orbit',
            uirevision='lock',
            aspectmode='data',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1200,
        height=800,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)")
    )

    # 【5️⃣】生成Plotly HTML div（不包含完整HTML结构）
    plot_div = fig.to_html(
        full_html=False,
        include_plotlyjs='cdn',
        div_id='uav-plot'
    )

    # 【6️⃣】准备JavaScript数据
    path_data = [[float(p[0]), float(p[1]), float(p[2])] for p in path_3d]
    photo_indices_list = sorted(list(photo_indices))

    js_data = f"""
    <script>
    // 嵌入的数据
    window.UAV_DATA = {{
        path: {json.dumps(path_data)},
        inspectionPoints: {json.dumps(inspection_points_data)},
        photoIndices: {json.dumps(photo_indices_list)},
        connectionPoints: 8,
        downsampleStep: {downsample_step}
    }};

    // 全局状态
    window.UAV_STATE = {{
        hasStarted: false,
        animationFrames: []
    }};
    </script>
    """

    # 【7️⃣】构建导航面板HTML
    nav_panel_html = """
    <div id="nav-panel">
        <h3>&#23548;&#36890;&#36755;&#20837;</h3>
        <p style="font-size:11px;color:#666;margin-bottom:10px;">
            &#36755;&#20837;&#36215;&#28857;&#22352;&#26631;,&#31995;&#32479;&#23558;&#33258;&#21160;&#25509;&#20837;&#26368;&#36817;&#24037;&#28857;&#28982;&#21518;&#25191;&#34892;&#24037;&#26816;&#20219;&#21153;
        </p>
        <div class="input-row">
            <label>X:</label>
            <input type="number" id="input-x" value="150" step="1" placeholder="示例值">
        </div>
        <div class="input-row">
            <label>Y:</label>
            <input type="number" id="input-y" value="100" step="1" placeholder="示例值">
        </div>
        <div class="input-row">
            <label>Z:</label>
            <input type="number" id="input-z" value="100" step="1" placeholder="示例值">
        </div>
        <button onclick="startNavigation()">&#24320;&#22987;&#24037;&#26816;</button>
        <div id="nav-info">
            &#31561;&#24453;&#36755;&#20837;...
        </div>
    </div>
    """

    # 【8️⃣】构建CSS样式
    css_styles = """
    <style>
    #nav-panel {
        position: absolute;
        top: 10px;
        right: 10px;
        background: rgba(255,255,255,0.95);
        border: 2px solid #4CAF50;
        border-radius: 8px;
        padding: 15px;
        z-index: 1000;
        box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        min-width: 200px;
        font-family: Arial, sans-serif;
    }
    #nav-panel h3 {
        margin: 0 0 5px 0;
        color: #333;
        font-size: 16px;
        text-align: center;
    }
    #nav-panel .input-row {
        margin: 8px 0;
    }
    #nav-panel label {
        display: inline-block;
        width: 25px;
        font-weight: bold;
        color: #555;
    }
    #nav-panel input {
        width: 80px;
        padding: 5px;
        border: 1px solid #ddd;
        border-radius: 4px;
        font-size: 12px;
    }
    #nav-panel button {
        width: 100%;
        background: #4CAF50;
        color: white;
        border: none;
        padding: 10px;
        margin-top: 10px;
        border-radius: 4px;
        cursor: pointer;
        font-size: 14px;
        font-weight: bold;
    }
    #nav-panel button:hover {
        background: #45a049;
    }
    #nav-info {
        margin-top: 12px;
        padding-top: 10px;
        border-top: 1px solid #ddd;
        font-size: 12px;
        color: #666;
        line-height: 1.5;
    }
    </style>
    """

    # 【9️⃣】构建JavaScript导航函数
    js_functions = """
    <script>
    // 1. 找最近巡检点
    function findNearestInspectionPoint(inputPos) {
        var photoPoints = UAV_DATA.inspectionPoints.filter(p => p.isPhoto);
        var minDist = Infinity;
        var nearest = null;

        photoPoints.forEach(function(p) {
            var dist = Math.sqrt(
                Math.pow(inputPos[0] - p.position[0], 2) +
                Math.pow(inputPos[1] - p.position[1], 2) +
                Math.pow(inputPos[2] - p.position[2], 2)
            );
            if (dist < minDist) {
                minDist = dist;
                nearest = p;
            }
        });
        return {point: nearest, distance: minDist};
    }

    // 2. 生成连接段（线性插值）
    function generateConnectionSegment(inputPos, targetPos, numPoints) {
        var segment = [];
        for (var i = 1; i <= numPoints; i++) {
            var t = i / (numPoints + 1);
            segment.push([
                inputPos[0] + t * (targetPos[0] - inputPos[0]),
                inputPos[1] + t * (targetPos[1] - inputPos[1]),
                inputPos[2] + t * (targetPos[2] - inputPos[2])
            ]);
        }
        segment.push([targetPos[0], targetPos[1], targetPos[2]]);
        return segment;
    }

    // 3. 构建完整执行路径
    function buildExecutionPath(inputPos, nearestPoint) {
        var connection = generateConnectionSegment(
            inputPos,
            nearestPoint.position,
            UAV_DATA.connectionPoints
        );
        var remaining = UAV_DATA.path.slice(nearestPoint.index);
        return connection.concat(remaining.slice(1));
    }

    // 4. 重新生成动画帧（分段判断拍照点）
    function regenerateAnimationFrames(execPath, startIndex) {
        var frames = [];
        var connectionLength = UAV_DATA.connectionPoints + 1;  // +1 包含目标点

        // 遍历执行路径，分段判断
        execPath.forEach(function(point, i) {
            var isPhoto;
            var originalIdx;
            var color;
            var size;
            var text;

            if (i < connectionLength) {
                // 接入段：不是拍照点
                isPhoto = false;
                originalIdx = -1;
                color = (i === connectionLength - 1) ? 'red' : 'orange';  // 到达点用红色
                size = 5;
                text = (i === connectionLength - 1) ? 'Arrived' : 'Connecting...';
            } else {
                // 巡检段：计算原始路径索引
                originalIdx = startIndex + (i - connectionLength);
                isPhoto = UAV_DATA.photoIndices.includes(originalIdx);

                if (isPhoto) {
                    color = 'red';
                    size = 8;
                    text = 'WP ' + originalIdx + ' - Photo';
                } else {
                    color = 'orange';
                    size = 5;
                    text = 'WP ' + originalIdx;
                }
            }

            // 创建帧
            frames.push({
                data: [{
                    x: [point[0]],
                    y: [point[1]],
                    z: [point[2]],
                    mode: 'markers' + (isPhoto || text === 'Arrived' ? '+text' : ''),
                    marker: {size: size, color: color},
                    text: [text],
                    textposition: 'top center',
                    textfont: {size: 10, color: 'black'}
                }],
                traces: [4],  // UAV trace index
                name: 'Frame ' + (i + 1)
            });

            // 拍照点停顿效果
            if (isPhoto) {
                frames.push(frames[frames.length - 1]);
            }
        });

        return frames;
    }

    // 5. 更新可视化
    function updateVisualization(inputPos, nearestPoint, execPath) {
        // 准备数据
        var inputX = [inputPos[0]], inputY = [inputPos[1]], inputZ = [inputPos[2]];
        var nearestX = [nearestPoint.position[0]];
        var nearestY = [nearestPoint.position[1]];
        var nearestZ = [nearestPoint.position[2]];
        var execX = execPath.map(p => p[0]);
        var execY = execPath.map(p => p[1]);
        var execZ = execPath.map(p => p[2]);

        // 更新 traces
        Plotly.react('uav-plot',
            [
                // Trace 0: 原始路径（不变）
                {x: UAV_DATA.path.map(p => p[0]), y: UAV_DATA.path.map(p => p[1]), z: UAV_DATA.path.map(p => p[2]),
                 mode: 'lines', line: {color: 'lightgray', width: 2, dash: 'dot'}, name: 'Original Path',
                 hovertemplate: 'Original<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>', type: 'scatter3d'},

                // Trace 1: 执行路径（绿色加粗）
                {x: execX, y: execY, z: execZ,
                 mode: 'lines', line: {color: 'green', width: 4}, name: 'Execution Path',
                 hovertemplate: 'Execution<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>', type: 'scatter3d'},

                // Trace 2: 输入点（紫色）
                {x: inputX, y: inputY, z: inputZ,
                 mode: 'markers', marker: {size: 12, color: 'purple', symbol: 'circle', line: {color: 'white', width: 2}},
                 name: 'Input Point', type: 'scatter3d'},

                // Trace 3: 最近巡检点（红色）
                {x: nearestX, y: nearestY, z: nearestZ,
                 mode: 'markers', marker: {size: 15, color: 'red', symbol: 'diamond', line: {color: 'yellow', width: 2}},
                 name: 'Nearest Inspection Point', type: 'scatter3d'},

                // Trace 4: UAV（初始在输入点）
                {x: [inputPos[0]], y: [inputPos[1]], z: [inputPos[2]],
                 mode: 'markers', marker: {size: 5, color: 'orange'}, name: 'UAV', type: 'scatter3d'}
            ],
            {
                title: {text: 'UAV Inspection Flight - Executing...'},
                scene: {
                    dragmode: 'orbit', uirevision: 'lock', aspectmode: 'data',
                    camera: {up: {x: 0, y: 0, z: 1}, eye: {x: 1.5, y: 1.5, z: 1.2}, center: {x: 0, y: 0, z: 0}},
                    xaxis: {title: 'X (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'},
                    yaxis: {title: 'Y (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'},
                    zaxis: {title: 'Z (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'}
                },
                width: 1200, height: 800,
                paper_bgcolor: 'white', plot_bgcolor: 'white',
                font: {family: 'Arial', size: 12},
                legend: {x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)'}
            },
            {displayModeBar: true, displaylogo: false}
        );
    }

    // 6. 主函数：开始导航
    function startNavigation() {
        // 读取输入
        var inputX = parseFloat(document.getElementById('input-x').value);
        var inputY = parseFloat(document.getElementById('input-y').value);
        var inputZ = parseFloat(document.getElementById('input-z').value);

        if (isNaN(inputX) || isNaN(inputY) || isNaN(inputZ)) {
            document.getElementById('nav-info').innerHTML = '<span style="color:red;">&#35831;&#36755;&#20837;&#26377;&#25928;&#30340;&#22352;&#26631;</span>';
            return;
        }

        var inputPos = [inputX, inputY, inputZ];

        // 找最近巡检点
        var result = findNearestInspectionPoint(inputPos);
        if (!result.point) {
            document.getElementById('nav-info').innerHTML = '<span style="color:red;">&#26410;&#25214;&#21040;&#24037;&#28857;</span>';
            return;
        }

        // 构建执行路径
        var execPath = buildExecutionPath(inputPos, result.point);

        // 控制台打印：调试信息
        console.log('=== Animation Start ===');
        console.log('execPath.length:', execPath.length);
        console.log('inputPos:', inputPos);
        console.log('nearestPoint.index:', result.point.index);
        console.log('distance:', result.distance.toFixed(1) + 'm');

        // 生成动画帧
        var frames = regenerateAnimationFrames(execPath, result.point.index);

        console.log('frames.length:', frames.length);

        // 提取帧名称数组
        var frameNames = frames.map(f => f.name);

        console.log('frameNames:', frameNames);
        console.log('animation started');

        // 更新图表（UAV 初始位置在输入点，同时绑定帧）
        var inputX_arr = [inputPos[0]], inputY_arr = [inputPos[1]], inputZ_arr = [inputPos[2]];
        var nearestX = [result.point.position[0]];
        var nearestY = [result.point.position[1]];
        var nearestZ = [result.point.position[2]];
        var execX = execPath.map(p => p[0]);
        var execY = execPath.map(p => p[1]);
        var execZ = execPath.map(p => p[2]);

        // 使用 Plotly.react 更新图表
        Plotly.react('uav-plot',
            [
                // Trace 0: 原始路径（不变）
                {x: UAV_DATA.path.map(p => p[0]), y: UAV_DATA.path.map(p => p[1]), z: UAV_DATA.path.map(p => p[2]),
                 mode: 'lines', line: {color: 'lightgray', width: 2, dash: 'dot'}, name: 'Original Path',
                 hovertemplate: 'Original<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>', type: 'scatter3d'},

                // Trace 1: 执行路径（绿色加粗）
                {x: execX, y: execY, z: execZ,
                 mode: 'lines', line: {color: 'green', width: 4}, name: 'Execution Path',
                 hovertemplate: 'Execution<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>', type: 'scatter3d'},

                // Trace 2: 输入点（紫色）
                {x: inputX_arr, y: inputY_arr, z: inputZ_arr,
                 mode: 'markers', marker: {size: 12, color: 'purple', symbol: 'circle', line: {color: 'white', width: 2}},
                 name: 'Input Point', type: 'scatter3d'},

                // Trace 3: 最近巡检点（红色）
                {x: nearestX, y: nearestY, z: nearestZ,
                 mode: 'markers', marker: {size: 15, color: 'red', symbol: 'diamond', line: {color: 'yellow', width: 2}},
                 name: 'Nearest Inspection Point', type: 'scatter3d'},

                // Trace 4: UAV（初始在输入点）
                {x: [inputPos[0]], y: [inputPos[1]], z: [inputPos[2]],
                 mode: 'markers', marker: {size: 5, color: 'orange'}, name: 'UAV', type: 'scatter3d'}
            ],
            {
                title: {text: 'UAV Inspection Flight - Executing...'},
                scene: {
                    dragmode: 'orbit', uirevision: 'lock', aspectmode: 'data',
                    camera: {up: {x: 0, y: 0, z: 1}, eye: {x: 1.5, y: 1.5, z: 1.2}, center: {x: 0, y: 0, z: 0}},
                    xaxis: {title: 'X (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'},
                    yaxis: {title: 'Y (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'},
                    zaxis: {title: 'Z (m)', showgrid: false, showbackground: true, backgroundcolor: 'rgb(240, 240, 240)'}
                },
                width: 1200, height: 800,
                paper_bgcolor: 'white', plot_bgcolor: 'white',
                font: {family: 'Arial', size: 12},
                legend: {x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)'},
                updatemenus: [
                    {
                        type: 'buttons',
                        showactive: false,
                        x: 0.1, y: 1.02,
                        xanchor: 'right',
                        yanchor: 'top',
                        pad: {t: 10, r: 10},
                        buttons: [
                            {
                                label: '&#25773;&#25918;&#21160;&#30011;',
                                method: 'animate',
                                args: [
                                    frameNames,
                                    {
                                        frame: {duration: 400, redraw: false},
                                        transition: {duration: 0},
                                        fromcurrent: false
                                    }
                                ]
                            }
                        ]
                    }
                ]
            },
            {displayModeBar: true, displaylogo: false}
        );

        // 添加帧到图表
        Plotly.addFrames('uav-plot', frames);

        // 立即播放动画
        setTimeout(function() {
            Plotly.animate('uav-plot', frameNames, {
                frame: {duration: 400, redraw: false},
                transition: {duration: 0},
                fromcurrent: false
            });
        }, 100);

        // 更新信息面板
        var remainingPhotos = UAV_DATA.photoIndices.filter(i => i >= result.point.index).length;
        document.getElementById('nav-info').innerHTML =
            '<strong>&#25191;&#34892;&#20013;</strong><br><br>' +
            '&#25509;&#20837;&#28857;&#32034;&#24341;: ' + result.point.index + '<br>' +
            '&#25509;&#20837;&#36317;&#31163;: ' + result.distance.toFixed(1) + 'm<br>' +
            '&#21171;&#20313;&#25293;&#28857;: ' + remainingPhotos + '<br>' +
            '&#25191;&#34892;&#33258;&#28857;: ' + execPath.length;

        // 标记已开始
        UAV_STATE.hasStarted = true;
    }
    </script>
    """

    # 【10️拼】拼接完整HTML并一次性写入
    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>UAV Inspection Flight - Navigation</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    {css_styles}
</head>
<body>
    {nav_panel_html}
    {plot_div}
    {js_data}
    {js_functions}
</body>
</html>
"""

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(full_html)

    print(f"  [保存] {output_file}")
    print(f"  [统计] 原始点: {original_count}, 降采样: {len(path_vis)}")
    print(f"  [任务] 拍照点: {len(photo_indices)}")
    print(f"  [导航] 页面包含坐标输入面板，用户可输入起点开始巡检")

    return fig


def create_analysis_view(original_path, optimized_path, wind_vector,
                        output_file="output/comparison.html",
                        show_terrain=False, terrain_map=None,
                        inspection_points=None):
    """
    创建原始vs优化路径对比分析视图

    Args:
        original_path: 原始路径 [(x, y, z), ...]
        optimized_path: 优化后路径 [(x, y, z), ...]
        wind_vector: 风向量 [wx, wy]
        output_file: 输出HTML文件路径
        show_terrain: 是否显示地形
        terrain_map: 地形高度图
        inspection_points: 巡检任务点列表 (可选)
    """
    print("[分析视图] 创建路径对比分析...")

    from core.path_optimizer import compute_total_cost

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 计算成本
    original_cost = compute_total_cost(original_path, wind_vector)
    optimized_cost = compute_total_cost(optimized_path, wind_vector)

    improvement = original_cost['total_cost'] - optimized_cost['total_cost']
    improvement_pct = (improvement / original_cost['total_cost']) * 100

    print(f"  原始成本: {original_cost['total_cost']:.2f}")
    print(f"  优化成本: {optimized_cost['total_cost']:.2f}")
    print(f"  改善: {improvement:.2f} ({improvement_pct:.1f}%)")

    # 创建图形
    fig = go.Figure()

    # 提取坐标
    orig_x = [p[0] for p in original_path]
    orig_y = [p[1] for p in original_path]
    orig_z = [p[2] for p in original_path]

    opt_x = [p[0] for p in optimized_path]
    opt_y = [p[1] for p in optimized_path]
    opt_z = [p[2] for p in optimized_path]

    # 1. 添加原始路径（蓝色）
    fig.add_trace(go.Scatter3d(
        x=orig_x,
        y=orig_y,
        z=orig_z,
        mode='lines',
        line=dict(color='blue', width=3),
        name='Original Path',
        hovertemplate='Original<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 2. 添加优化路径（绿色）
    fig.add_trace(go.Scatter3d(
        x=opt_x,
        y=opt_y,
        z=opt_z,
        mode='lines',
        line=dict(color='green', width=3),
        name='Optimized Path',
        hovertemplate='Optimized<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 3. 标记高成本段（红色）
    segment_costs = original_cost['segment_costs']
    avg_cost = original_cost['avg_cost']
    std_cost = np.std(segment_costs)
    high_cost_threshold = avg_cost + 0.5 * std_cost

    for i, cost in enumerate(segment_costs):
        if cost > high_cost_threshold:
            # 绘制高成本段
            fig.add_trace(go.Scatter3d(
                x=[orig_x[i], orig_x[i+1]],
                y=[orig_y[i], orig_y[i+1]],
                z=[orig_z[i], orig_z[i+1]],
                mode='lines',
                line=dict(color='red', width=6),
                showlegend=False,
                hovertemplate=f'High Cost Segment<br>Cost: {cost:.1f}<extra></extra>'
            ))

    # 【新增】4.5. 添加巡检点可视化（黄色点）
    if inspection_points:
        photo_x, photo_y, photo_z = [], [], []
        for p in inspection_points:
            if p.is_photo:
                x, y, z = p.position
                photo_x.append(x)
                photo_y.append(y)
                photo_z.append(z)

        if photo_x:
            fig.add_trace(go.Scatter3d(
                x=photo_x,
                y=photo_y,
                z=photo_z,
                mode='markers',
                marker=dict(size=5, color='yellow'),
                name='Inspection Points',
                hovertemplate='Inspection Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
            ))
            print(f"  [巡检点] 显示 {len(photo_x)} 个拍照点")

    # 4. 添加起点和终点标记
    fig.add_trace(go.Scatter3d(
        x=[orig_x[0]],
        y=[orig_y[0]],
        z=[orig_z[0]],
        mode='markers',
        marker=dict(size=8, color='green', symbol='diamond'),
        name='Start',
        showlegend=False
    ))

    fig.add_trace(go.Scatter3d(
        x=[orig_x[-1]],
        y=[orig_y[-1]],
        z=[orig_z[-1]],
        mode='markers',
        marker=dict(size=8, color='red', symbol='diamond'),
        name='End',
        showlegend=False
    ))

    # 5. 成本对比标注（关键）
    cost_text = (
        f"<b>Cost Comparison</b><br>"
        f"Before: {original_cost['total_cost']:.2f}<br>"
        f"After: {optimized_cost['total_cost']:.2f}<br>"
        f"<b>Improvement: {improvement:.2f} ({improvement_pct:.1f}%)</b>"
    )

    fig.add_annotation(
        text=cost_text,
        xref="paper", yref="paper",
        x=0.02, y=0.98,
        showarrow=False,
        font=dict(size=14, color="black"),
        bgcolor="rgba(255,255,255,0.8)",
        bordercolor="gray",
        borderwidth=1,
        align="left"
    )

    # 6. 风向标注
    wind_speed = np.linalg.norm(wind_vector)
    wind_dir = np.degrees(np.arctan2(wind_vector[1], wind_vector[0]))
    wind_text = f"Wind: {wind_speed:.1f} m/s, {wind_dir:.0f}°"

    fig.add_annotation(
        text=wind_text,
        xref="paper", yref="paper",
        x=0.02, y=0.85,
        showarrow=False,
        font=dict(size=12, color="blue"),
        bgcolor="rgba(255,255,255,0.7)",
        bordercolor="blue",
        borderwidth=1
    )

    # 设置布局
    fig.update_layout(
        title=dict(
            text=f"Path Optimization Analysis - Improvement: {improvement_pct:.1f}%",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            aspectmode='data',
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1400,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.75, bgcolor="rgba(255,255,255,0.8)")
    )

    # 保存文件
    fig.write_html(output_file, include_plotlyjs='cdn',
                   config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [保存] {output_file}")
    print(f"  [改善] {improvement_pct:.1f}%")

    return {
        'original_cost': original_cost['total_cost'],
        'optimized_cost': optimized_cost['total_cost'],
        'improvement': improvement,
        'improvement_pct': improvement_pct
    }


def create_simple_comparison(original_path, optimized_path, wind_vector,
                           output_file="output/comparison.html"):
    """
    简化版对比视图（不含地形）
    """
    return create_analysis_view(original_path, optimized_path, wind_vector, output_file)


def create_animation_view_with_tasks(path_3d, inspection_points,
                                     output_file="output/animation_with_tasks.html",
                                     downsample_step=3, frame_step=4):
    """
    创建带任务点的UAV巡检动画（性能优化版）

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        inspection_points: 巡检任务点列表
        output_file: 输出HTML文件路径
        downsample_step: 路径降采样步长（默认3）
        frame_step: 动画帧降采样步长（默认4）
    """
    print("[动画视图] 创建带任务点的UAV巡检动画（性能优化）...")

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    original_count = len(path_3d)

    # 【优化1】路径降采样
    if downsample_step > 1:
        path_3d = downsample_path(path_3d, downsample_step)
        print(f"  [降采样] 路径点: {original_count} → {len(path_3d)}")

        # 同步过滤inspection_points
        inspection_points = [p for p in inspection_points if p.index % downsample_step == 0]

    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    # 创建图形
    fig = go.Figure()

    # 【优化2】只添加优化路径
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='blue', width=3),
        name='Path',
        hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 【优化3】只显示高优先级拍照点
    high_priority_x = [p.position[0] for p in inspection_points if p.is_photo and p.priority == 'high']
    high_priority_y = [p.position[1] for p in inspection_points if p.is_photo and p.priority == 'high']
    high_priority_z = [p.position[2] for p in inspection_points if p.is_photo and p.priority == 'high']

    if high_priority_x:
        fig.add_trace(go.Scatter3d(
            x=high_priority_x,
            y=high_priority_y,
            z=high_priority_z,
            mode='markers',
            marker=dict(size=6, color='red', line=dict(color='black', width=1)),
            name='Key Points',
            showlegend=False,
            hovertemplate='Key Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # UAV trace索引
    uav_trace_index = len(fig.data)

    # 添加UAV初始位置
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]],
        y=[path_y[0]],
        z=[path_z[0]],
        mode='markers',
        marker=dict(size=4, color='orange', opacity=1.0),
        name='UAV',
        showlegend=False
    ))

    # 【优化4】动画帧降采样
    frames = []
    frame_indices = range(0, len(path_x), frame_step)

    for frame_idx, i in enumerate(frame_indices):
        x, y, z = path_x[i], path_y[i], path_z[i]

        # 查找当前点是否是拍照点
        current_point = None
        for p in inspection_points:
            if p.index == i * downsample_step:  # 调整索引匹配
                current_point = p
                break

        # 根据是否是拍照点设置颜色
        if current_point and current_point.is_photo:
            uav_color = 'red' if current_point.priority == 'high' else 'yellow'
            uav_size = 6
        else:
            uav_color = 'orange'
            uav_size = 4

        uav_update = go.Scatter3d(
            x=[x],
            y=[y],
            z=[z],
            mode='markers',
            marker=dict(size=uav_size, color=uav_color, opacity=1.0)
        )

        frames.append(go.Frame(
            data=[uav_update],
            traces=[uav_trace_index],
            name=f'Frame {frame_idx+1}'
        ))

    fig.frames = frames

    # 设置布局
    fig.update_layout(
        title=dict(
            text="UAV Inspection Flight - with Photo Points",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            aspectmode='data',
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1200,
        height=800,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
        updatemenus=[
            dict(
                type='buttons',
                showactive=False,
                x=1.0,
                y=1.15,
                xanchor='right',
                yanchor='top',
                direction='left',
                buttons=[
                    dict(
                        label='▶ Play',
                        method='animate',
                        args=[None, {
                            'frame': {'duration': 100, 'redraw': False, 'fromcurrent': True},
                            'transition': {'duration': 50}
                        }]
                    ),
                    dict(
                        label='⏸ Pause',
                        method='animate',
                        args=[[None], {
                            'frame': {'duration': 0, 'redraw': False, 'fromcurrent': True},
                            'mode': 'immediate',
                            'transition': {'duration': 0}
                        }]
                    )
                ]
            )
        ]
    )

    fig.write_html(output_file, include_plotlyjs='cdn',
                   config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [保存] {output_file}")
    print(f"  [优化] 原始点: {original_count}, 降采样后: {len(path_3d)}")
    print(f"  [动画] {len(frames)} 帧（每{frame_step}个点一帧）")
    print(f"  [性能] 帧数减少: {int((1 - len(frames)/original_count) * 100)}%")
    print(f"  [任务点] {len(high_priority_x)} 个关键点")

    return fig


def create_analysis_view_with_tasks(original_path, optimized_path, wind_vector,
                                   inspection_points,
                                   output_file="output/comparison_with_tasks.html"):
    """
    创建带任务点的路径对比分析视图

    Args:
        original_path: 原始路径
        optimized_path: 优化后路径
        wind_vector: 风向量
        inspection_points: 巡检任务点列表
        output_file: 输出文件路径
    """
    print("[分析视图] 创建带任务点的路径对比分析...")

    from core.path_optimizer import compute_total_cost

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 计算成本
    original_cost = compute_total_cost(original_path, wind_vector)
    optimized_cost = compute_total_cost(optimized_path, wind_vector)

    improvement = original_cost['total_cost'] - optimized_cost['total_cost']
    improvement_pct = (improvement / original_cost['total_cost']) * 100

    print(f"  原始成本: {original_cost['total_cost']:.2f}")
    print(f"  优化成本: {optimized_cost['total_cost']:.2f}")
    print(f"  改善: {improvement:.2f} ({improvement_pct:.1f}%)")

    fig = go.Figure()

    # 原始路径（蓝色）
    orig_x = [p[0] for p in original_path]
    orig_y = [p[1] for p in original_path]
    orig_z = [p[2] for p in original_path]

    fig.add_trace(go.Scatter3d(
        x=orig_x,
        y=orig_y,
        z=orig_z,
        mode='lines',
        line=dict(color='blue', width=3),
        name='Original Path'
    ))

    # 优化路径（绿色）
    opt_x = [p[0] for p in optimized_path]
    opt_y = [p[1] for p in optimized_path]
    opt_z = [p[2] for p in optimized_path]

    fig.add_trace(go.Scatter3d(
        x=opt_x,
        y=opt_y,
        z=opt_z,
        mode='lines',
        line=dict(color='green', width=3),
        name='Optimized Path'
    ))

    # 高成本段标红
    segment_costs = original_cost['segment_costs']
    avg_cost = original_cost['avg_cost']
    std_cost = np.std(segment_costs)
    high_cost_threshold = avg_cost + 0.5 * std_cost

    for i, cost in enumerate(segment_costs):
        if cost > high_cost_threshold:
            fig.add_trace(go.Scatter3d(
                x=[orig_x[i], orig_x[i+1]],
                y=[orig_y[i], orig_y[i+1]],
                z=[orig_z[i], orig_z[i+1]],
                mode='lines',
                line=dict(color='red', width=6),
                showlegend=False,
                hovertemplate=f'High Cost: {cost:.1f}<extra></extra>'
            ))

    # 任务点：普通拍照点（黄色）
    normal_photo_x = [p.position[0] for p in inspection_points if p.is_photo and p.priority == 'normal']
    normal_photo_y = [p.position[1] for p in inspection_points if p.is_photo and p.priority == 'normal']
    normal_photo_z = [p.position[2] for p in inspection_points if p.is_photo and p.priority == 'normal']

    if normal_photo_x:
        fig.add_trace(go.Scatter3d(
            x=normal_photo_x,
            y=normal_photo_y,
            z=normal_photo_z,
            mode='markers',
            marker=dict(size=6, color='yellow', line=dict(color='black', width=1)),
            name='Photo Points',
            hovertemplate='Photo Point<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # 任务点：高优先级（红色）
    high_priority_x = [p.position[0] for p in inspection_points if p.priority == 'high']
    high_priority_y = [p.position[1] for p in inspection_points if p.priority == 'high']
    high_priority_z = [p.position[2] for p in inspection_points if p.priority == 'high']

    if high_priority_x:
        fig.add_trace(go.Scatter3d(
            x=high_priority_x,
            y=high_priority_y,
            z=high_priority_z,
            mode='markers',
            marker=dict(size=8, color='red', symbol='diamond', line=dict(color='black', width=1)),
            name='High Priority',
            hovertemplate='High Priority<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # 起终点标记
    fig.add_trace(go.Scatter3d(
        x=[orig_x[0]],
        y=[orig_y[0]],
        z=[orig_z[0]],
        mode='markers',
        marker=dict(size=8, color='green', symbol='diamond'),
        name='Start',
        showlegend=False
    ))

    fig.add_trace(go.Scatter3d(
        x=[orig_x[-1]],
        y=[orig_y[-1]],
        z=[orig_z[-1]],
        mode='markers',
        marker=dict(size=8, color='red', symbol='diamond'),
        name='End',
        showlegend=False
    ))

    # 成本对比标注
    cost_text = (
        f"<b>Cost Comparison</b><br>"
        f"Before: {original_cost['total_cost']:.2f}<br>"
        f"After: {optimized_cost['total_cost']:.2f}<br>"
        f"<b>Improvement: {improvement:.2f} ({improvement_pct:.1f}%)</b><br><br>"
        f"<b>Tasks</b><br>"
        f"Photo Points: {sum(1 for p in inspection_points if p.is_photo)}<br>"
        f"High Priority: {sum(1 for p in inspection_points if p.priority == 'high')}"
    )

    fig.add_annotation(
        text=cost_text,
        xref="paper", yref="paper",
        x=0.02, y=0.98,
        showarrow=False,
        font=dict(size=12, color="black"),
        bgcolor="rgba(255,255,255,0.85)",
        bordercolor="gray",
        borderwidth=1,
        align="left"
    )

    # 风向标注
    wind_speed = np.linalg.norm(wind_vector)
    wind_dir = np.degrees(np.arctan2(wind_vector[1], wind_vector[0]))
    wind_text = f"Wind: {wind_speed:.1f} m/s, {wind_dir:.0f}°"

    fig.add_annotation(
        text=wind_text,
        xref="paper", yref="paper",
        x=0.02, y=0.82,
        showarrow=False,
        font=dict(size=11, color="blue"),
        bgcolor="rgba(255,255,255,0.7)",
        bordercolor="blue",
        borderwidth=1
    )

    # 设置布局
    fig.update_layout(
        title=dict(
            text=f"Path Optimization with Tasks - Improvement: {improvement_pct:.1f}%",
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        scene=dict(
            aspectmode='data',
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.5, y=1.5, z=1.2),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='X (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            yaxis=dict(title='Y (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
            zaxis=dict(title='Z (m)', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(240, 240, 240)"),
        ),
        width=1400,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=12),
        legend=dict(x=0.02, y=0.70, bgcolor="rgba(255,255,255,0.8)")
    )

    fig.write_html(output_file, include_plotlyjs='cdn',
                   config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [保存] {output_file}")
    print(f"  [任务点] {len(inspection_points)} 个 ({sum(1 for p in inspection_points if p.is_photo)} 拍照)")

    return {
        'original_cost': original_cost['total_cost'],
        'optimized_cost': optimized_cost['total_cost'],
        'improvement': improvement,
        'improvement_pct': improvement_pct,
        'task_count': len(inspection_points),
        'photo_count': sum(1 for p in inspection_points if p.is_photo)
    }


def create_stage2_mission_view(
    anim_path_3d,
    vis_pts,
    vis_tasks,
    vis_stats,
    terrain=None,
    image_obj=None,
    image_path=None,
    output_file=None,
    downsample_step=3
):
    """
    创建阶段2任务优化可视化HTML（增强版 + 地图底图）

    特性：
    - 地图底图 + 轻量地形 + 路径/点/UAV
    - 弱化地形背景，突出任务路径
    - 线路层级：背景层、主路径层、高亮层
    - 优化巡检点大小设计
    - 压缩图例，增强演示感
    - 分区信息面板，UAV尾迹效果

    Args:
        anim_path_3d: 动画3D路径 [(x, y, z), ...]
        vis_pts: 可视化巡检点列表 (List[VisPoint])
        vis_tasks: 可视化任务段列表 (List[VisTask])
        vis_stats: 可视化统计信息 (VisStats)
        terrain: 地形高度图（可选）
        image_obj: PIL图像对象（用于底图显示）
        image_path: 原始图片路径（可选）
        output_file: 输出HTML文件路径（可选，仅用于返回路径）
        downsample_step: 路径降采样步长（默认3）

    Returns:
        str: 完整HTML字符串
    """
    print("[Stage2视图] 创建任务优化可视化（增强版 + 地图底图）...")

    import json
    import os
    from PIL import Image

    # 路径降采样
    original_count = len(anim_path_3d)
    path_3d = anim_path_3d[::downsample_step] if downsample_step > 1 else anim_path_3d

    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    print(f"  [降采样] 路径点: {original_count} → {len(path_3d)}")

    # 创建图形
    fig = go.Figure()

    # ===== 1. 地图底图（主背景） =====
    if image_obj is not None:
        img_array = np.array(image_obj)
        h, w = img_array.shape[:2]
        X, Y = np.meshgrid(np.arange(w), np.arange(h))

        # 计算路径Z坐标的平均值，将地图放置在路径下方
        if path_z:
            min_z = min(path_z)
            map_z = min_z * 0.1 if min_z > 0 else 0
        else:
            map_z = 0

        Z = np.full_like(X, map_z, dtype=float)

        fig.add_trace(go.Surface(
            x=X, y=Y, z=Z,
            surfacecolor=img_array,
            showscale=False,
            opacity=1.0,
            name='Map',
            showlegend=False,
            hovertemplate='Map<br>X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
        ))
    else:
        print("  [警告] image_obj为None，跳过地图底图创建")

    # ===== 2. 地形（已移除 - 使用地图作为主背景） =====
    # 不再添加纯色地形面，保持地图底图的清晰可见

    # ===== 3. 全局优化路径（加粗20%） =====
    main_path_color = '#00C853'  # 鲜绿色
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color=main_path_color, width=6),  # 原5 → 6, 加粗20%
        name='Optimal Path',
        hovertemplate='Optimal Path<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # ===== 4. 巡检点（最终优化） =====
    endpoint_x, endpoint_y, endpoint_z = [], [], []
    turning_x, turning_y, turning_z = [], [], []
    sample_x, sample_y, sample_z = [], [], []

    for pt in vis_pts:
        try:
            pt_suffix = pt.point_id.split('_')[-1]
            idx = int(pt_suffix)
            if idx % downsample_step != 0 and idx != 0 and idx != len(anim_path_3d) - 1:
                continue
        except:
            pass

        if pt.point_type == 'endpoint':
            endpoint_x.append(pt.position_3d[0])
            endpoint_y.append(pt.position_3d[1])
            endpoint_z.append(pt.position_3d[2])
        elif pt.point_type == 'turning':
            turning_x.append(pt.position_3d[0])
            turning_y.append(pt.position_3d[1])
            turning_z.append(pt.position_3d[2])
        else:
            sample_x.append(pt.position_3d[0])
            sample_y.append(pt.position_3d[1])
            sample_z.append(pt.position_3d[2])

    # 采样点（0.5 → 0.2, 缩小到40%）
    if sample_x:
        fig.add_trace(go.Scatter3d(
            x=sample_x, y=sample_y, z=sample_z,
            mode='markers',
            marker=dict(size=0.2, color='#90CAF9', opacity=0.3),
            name='Sample Points',
            showlegend=False,
            hovertemplate='Sample<br>%{text}<extra></extra>',
            text=[f"{pt.line_id}" for pt in vis_pts if pt.point_type == 'sample']
        ))

    # 拐点（2 → 1, 缩小到50%）
    if turning_x:
        fig.add_trace(go.Scatter3d(
            x=turning_x, y=turning_y, z=turning_z,
            mode='markers',
            marker=dict(size=1, color='#FF5252', symbol='circle', line=dict(color='white', width=0.5)),
            name='Turning Points',
            hovertemplate='Turning<br>%{text}<extra></extra>',
            text=[f"{pt.line_id}<br>{pt.point_type}" for pt in vis_pts if pt.point_type == 'turning']
        ))

    # 端点（3 → 1.7, 缩小到55%）
    if endpoint_x:
        fig.add_trace(go.Scatter3d(
            x=endpoint_x, y=endpoint_y, z=endpoint_z,
            mode='markers',
            marker=dict(size=1.7, color='#4CAF50', symbol='diamond', line=dict(color='white', width=0.5)),
            name='Endpoints',
            hovertemplate='Endpoint<br>%{text}<extra></extra>',
            text=[f"{pt.line_id}<br>{pt.point_type}" for pt in vis_pts if pt.point_type == 'endpoint']
        ))

    # ===== 5. 起点（8 → 5, 缩小到62.5%） =====
    if path_x:
        fig.add_trace(go.Scatter3d(
            x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
            mode='markers',
            marker=dict(size=5, color='#00E676', symbol='diamond',
                       line=dict(color='white', width=1.5)),
            name='Start',
            hovertemplate='Start<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # ===== 6. 终点（8 → 5, 缩小到62.5%） =====
    if path_x:
        fig.add_trace(go.Scatter3d(
            x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
            mode='markers',
            marker=dict(size=5, color='#AA00FF', symbol='diamond',
                       line=dict(color='white', width=1.5)),
            name='End',
            hovertemplate='End<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

    # ===== 7. UAV尾迹（淡色轨迹） =====
    uav_trail_index = len(fig.data)
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
        mode='lines',
        line=dict(color='#FFB74D', width=1.5),
        name='UAV Trail',
        showlegend=False
    ))

    # ===== 8. UAV当前位置（5 → 2.5, 缩小到50%，改为小圆点） =====
    uav_trace_index = len(fig.data)
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]] if path_x else [],
        y=[path_y[0]] if path_y else [],
        z=[path_z[0]] if path_z else [],
        mode='markers',
        marker=dict(size=2.5, color='#FF9800', symbol='circle',
                   line=dict(color='white', width=0.5)),
        name='UAV',
        showlegend=False
    ))

    # ===== 9. 构建统计信息标题 =====
    title_text = (
        f"UAV Powerline Inspection - Mission Execution<br>"
        f"<sup style='font-size: 11px; color: #666;'>"
        f"Task: 01/{vis_stats.total_tasks:02d} | "
        f"Progress: 0/{vis_stats.total_points} points | "
        f"Distance: {vis_stats.total_length:.0f}px"
        f"</sup>"
    )

    # ===== 10. 设置布局 =====
    fig.update_layout(
        title=dict(
            text=title_text,
            font=dict(size=20, color='rgb(40, 40, 40)'),
            x=0.05, xanchor='left'
        ),
        scene=dict(
            aspectmode='manual',  # 使用手动模式，控制XYZ轴比例
            aspectratio=dict(x=1, y=1, z=0.3),  # Z轴压缩，确保地图平面显示正常
            dragmode='orbit',
            uirevision='lock',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                eye=dict(x=1.6, y=1.6, z=1.3),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(title='', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(245, 245, 245)"),
            yaxis=dict(title='', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(245, 245, 245)"),
            zaxis=dict(title='', showgrid=False, showbackground=True,
                     backgroundcolor="rgb(245, 245, 245)"),
        ),
        width=1500,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Segoe UI, Arial, sans-serif", size=12),
        legend=dict(
            x=0.02, y=0.95,
            bgcolor="rgba(255,255,255,0.85)",
            bordercolor="rgb(200, 200, 200)",
            borderwidth=1,
            traceorder='normal'
        )
    )

    # ===== 11. 生成动画帧 =====
    frames = []
    trail_length = 15  # 尾迹长度

    for i, (x, y, z) in enumerate(zip(path_x, path_y, path_z)):
        # 计算尾迹
        trail_start = max(0, i - trail_length)
        trail_x = path_x[trail_start:i+1]
        trail_y = path_y[trail_start:i+1]
        trail_z = path_z[trail_start:i+1]

        # 检查当前点是否是关键点
        is_keypoint = False
        for pt in vis_pts:
            if abs(pt.position_3d[0] - x) < 1 and abs(pt.position_3d[1] - y) < 1:
                if pt.point_type in ['endpoint', 'turning']:
                    is_keypoint = True
                    break

        # UAV颜色和大小（关键点时稍大）
        if is_keypoint:
            color = '#FF1744'  # 亮红色
            size = 3  # 原6 → 3, 缩小到50%
        else:
            color = '#FF9800'  # 橙色
            size = 2.5  # 原5 → 2.5, 缩小到50%

        frames.append(go.Frame(
            data=[
                # UAV尾迹
                go.Scatter3d(
                    x=trail_x, y=trail_y, z=trail_z,
                    mode='lines',
                    line=dict(color='#FFB74D', width=2)
                ),
                # UAV位置
                go.Scatter3d(
                    x=[x], y=[y], z=[z],
                    mode='markers',
                    marker=dict(size=size, color=color, symbol='circle',
                               line=dict(color='white', width=0.5))
                )
            ],
            traces=[uav_trail_index, uav_trace_index],
            name=f'Frame_{i+1}'
        ))

    fig.frames = frames

    # ===== 12. 生成Plotly HTML div =====
    plot_div = fig.to_html(
        full_html=False,
        include_plotlyjs='cdn',
        div_id='stage2-plot'
    )

    # ===== 13. 准备JavaScript数据 =====
    # 预先生成任务列表
    tasks_list = [
        {
            'taskId': t.task_id,
            'lineId': t.line_id,
            'direction': t.direction,
            'startIdx': t.point_start_idx,
            'endIdx': t.point_end_idx
        } for t in vis_tasks
    ]

    # 预先生成路径列表
    path_list = [[float(p[0]), float(p[1]), float(p[2])] for p in anim_path_3d]

    js_data = f"""
    <script>
    window.STAGE2_DATA = {{
        path: {json.dumps(path_list)},
        tasks: {json.dumps(tasks_list)},
        stats: {json.dumps({
            'totalTasks': vis_stats.total_tasks,
            'totalPoints': vis_stats.total_points,
            'totalLength': vis_stats.total_length,
            'totalCost': vis_stats.total_cost,
            'taskOrder': vis_stats.task_order,
            'taskDirections': vis_stats.task_directions
        })},
        downsampleStep: {downsample_step}
    }};
    </script>
    """

    # ===== 14. 优化后的控制面板 =====
    control_panel = """
    <div id="control-panel">
        <div class="panel-section">
            <h3>总览信息</h3>
            <div class="info-grid">
                <div class="info-item">
                    <span class="info-label">任务数</span>
                    <span class="info-value" id="total-tasks">-</span>
                </div>
                <div class="info-item">
                    <span class="info-label">巡检点</span>
                    <span class="info-value" id="total-points">-</span>
                </div>
                <div class="info-item">
                    <span class="info-label">总长度</span>
                    <span class="info-value" id="total-length">-</span>
                </div>
                <div class="info-item">
                    <span class="info-label">总成本</span>
                    <span class="info-value" id="total-cost">-</span>
                </div>
            </div>
        </div>

        <div class="panel-section current-task-section">
            <h3>当前任务</h3>
            <div class="current-task-content">
                <div class="task-id" id="curr-task-id">等待开始...</div>
                <div class="task-detail">
                    <span class="task-label">线路:</span>
                    <span class="task-value" id="curr-line">-</span>
                </div>
                <div class="task-detail">
                    <span class="task-label">方向:</span>
                    <span class="task-value" id="curr-dir">-</span>
                </div>
                <div class="task-detail">
                    <span class="task-label">进度:</span>
                    <span class="task-value" id="curr-progress">0/0</span>
                </div>
            </div>
        </div>

        <div class="panel-section">
            <div class="button-row">
                <button class="btn-primary" onclick="playAnimation()">
                    <span>&#9658;</span> 播放
                </button>
                <button class="btn-secondary" onclick="pauseAnimation()">
                    <span>&#10074;&#10074;</span> 暂停
                </button>
            </div>
            <button class="btn-reset" onclick="resetAnimation()">
                <span>&#8634;</span> 重置
            </button>
        </div>
    </div>
    """

    # ===== 15. 优化后的CSS样式 =====
    css_styles = """
    <style>
    #control-panel {
        position: absolute;
        top: 10px;
        right: 10px;
        background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
        border: 1px solid #e0e0e0;
        border-radius: 12px;
        padding: 0;
        z-index: 1000;
        box-shadow: 0 6px 20px rgba(0,0,0,0.12);
        min-width: 260px;
        font-family: 'Segoe UI', Arial, sans-serif;
        overflow: hidden;
    }

    .panel-section {
        padding: 16px;
        border-bottom: 1px solid #eee;
    }

    .panel-section:last-child {
        border-bottom: none;
    }

    .current-task-section {
        background: linear-gradient(135deg, #FFF3E0 0%, #FFE0B2 100%);
    }

    #control-panel h3 {
        margin: 0 0 12px 0;
        color: #333;
        font-size: 14px;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }

    .current-task-section h3 {
        color: #E65100;
    }

    .info-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 8px;
    }

    .info-item {
        display: flex;
        flex-direction: column;
        align-items: center;
        padding: 6px;
        background: #f5f5f5;
        border-radius: 6px;
    }

    .info-label {
        font-size: 10px;
        color: #757575;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }

    .info-value {
        font-size: 16px;
        font-weight: 600;
        color: #1976D2;
    }

    .current-task-content {
        display: flex;
        flex-direction: column;
        gap: 8px;
    }

    .task-id {
        font-size: 16px;
        font-weight: 700;
        color: #E65100;
        text-align: center;
        padding: 8px;
        background: white;
        border-radius: 6px;
    }

    .task-detail {
        display: flex;
        justify-content: space-between;
        font-size: 12px;
    }

    .task-label {
        color: #8D6E63;
        font-weight: 500;
    }

    .task-value {
        color: #5D4037;
        font-weight: 600;
    }

    .button-row {
        display: flex;
        gap: 8px;
        margin-bottom: 8px;
    }

    button {
        flex: 1;
        background: linear-gradient(135deg, #4CAF50 0%, #45a049 100%);
        color: white;
        border: none;
        padding: 10px 16px;
        border-radius: 8px;
        cursor: pointer;
        font-size: 13px;
        font-weight: 600;
        transition: all 0.2s;
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 6px;
    }

    button:hover {
        transform: translateY(-1px);
        box-shadow: 0 4px 8px rgba(0,0,0,0.15);
    }

    button:active {
        transform: translateY(0);
    }

    .btn-secondary {
        background: linear-gradient(135deg, #FF9800 0%, #F57C00 100%);
    }

    .btn-reset {
        width: 100%;
        background: linear-gradient(135deg, #757575 0%, #616161 100%);
    }

    button span {
        font-size: 14px;
    }
    </style>
    """

    # ===== 16. 增强的JavaScript函数 =====
    js_functions = """
    <script>
    let currentFrameIndex = 0;
    let animationTimer = null;

    // 初始化统计信息
    function initializeStats() {
        if (typeof STAGE2_DATA !== 'undefined' && STAGE2_DATA.stats) {
            document.getElementById('total-tasks').textContent = STAGE2_DATA.stats.totalTasks || '-';
            document.getElementById('total-points').textContent = STAGE2_DATA.stats.totalPoints || '-';
            document.getElementById('total-length').textContent = (STAGE2_DATA.stats.totalLength || 0).toFixed(0) + 'px';
            document.getElementById('total-cost').textContent = (STAGE2_DATA.stats.totalCost || 0).toFixed(0);
        }

        // 删除默认动画
        setTimeout(function() {
            try {
                Plotly.deleteFrames('stage2-plot');
            } catch(e) {}
        }, 500);
    }

    // 使用DOMContentLoaded确保DOM加载完成
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initializeStats);
    } else {
        // DOM已经加载完成
        initializeStats();
    }

    // 播放动画
    function playAnimation() {
        const frameNames = STAGE2_DATA.path.map((_, i) => 'Frame_' + (i + 1));

        // 从当前位置继续
        const remainingFrames = frameNames.slice(currentFrameIndex);

        Plotly.animate('stage2-plot', remainingFrames, {
            frame: {duration: 120, redraw: false},
            transition: {duration: 0},
            fromcurrent: true
        });

        // 更新任务显示
        updateTaskDisplay(currentFrameIndex);

        // 自动更新
        startAutoUpdate();
    }

    // 暂停动画
    function pauseAnimation() {
        Plotly.animate('stage2-plot', null, {
            mode: 'immediate',
            transition: {duration: 0}
        });

        stopAutoUpdate();
    }

    // 重置动画
    function resetAnimation() {
        stopAutoUpdate();
        currentFrameIndex = 0;

        // 重置UAV位置
        const path = STAGE2_DATA.path;
        if (path.length > 0) {
            Plotly.restyle('stage2-plot', {
                x: [[path[0][0]], [path[0][0]]],
                y: [[path[0][1]], [path[0][1]]],
                z: [[path[0][2]], [path[0][2]]]
            }, [4, 5]);  // trail, uav
        }

        updateTaskDisplay(0);
    }

    // 自动更新任务显示
    function startAutoUpdate() {
        stopAutoUpdate();

        animationTimer = setInterval(function() {
            currentFrameIndex++;
            if (currentFrameIndex >= STAGE2_DATA.path.length) {
                stopAutoUpdate();
                return;
            }
            updateTaskDisplay(currentFrameIndex);
        }, 120);
    }

    function stopAutoUpdate() {
        if (animationTimer) {
            clearInterval(animationTimer);
            animationTimer = null;
        }
    }

    // 更新任务显示
    function updateTaskDisplay(pathIdx) {
        const tasks = STAGE2_DATA.tasks;
        const stats = STAGE2_DATA.stats;
        const downsample = STAGE2_DATA.downsampleStep || 1;

        // 查找当前任务
        for (let i = 0; i < tasks.length; i++) {
            const task = tasks[i];
            if (pathIdx >= task.startIdx && pathIdx <= task.endIdx) {
                const dirText = task.direction > 0 ? '正向' : '反向';
                document.getElementById('curr-task-id').textContent = `Task ${task.taskId}`;
                document.getElementById('curr-line').textContent = task.lineId;
                document.getElementById('curr-dir').textContent = dirText;

                // 计算进度
                const taskProgress = Math.min(pathIdx - task.startIdx + 1, task.endIdx - task.startIdx + 1);
                const totalInTask = task.endIdx - task.startIdx + 1;
                document.getElementById('curr-progress').textContent = `${taskProgress}/${totalInTask}`;

                // 更新标题
                const titleEl = document.querySelector('g.title text');
                if (titleEl) {
                    const progress = Math.min(pathIdx * downsample + 1, stats.totalPoints);
                    titleEl.textContent = `UAV Powerline Inspection - Mission Execution\\nTask: ${(i+1).toString().padStart(2, '0')}/${stats.totalTasks.toString().padStart(2, '0')} | Progress: ${progress}/${stats.totalPoints} points | Distance: ${stats.totalLength.toFixed(0)}px`;
                }

                return;
            }
        }

        // 如果没有找到任务（完成状态）
        document.getElementById('curr-task-id').textContent = '任务完成';
        document.getElementById('curr-line').textContent = '-';
        document.getElementById('curr-dir').textContent = '-';
        document.getElementById('curr-progress').textContent = `${stats.totalPoints}/${stats.totalPoints}`;
    }
    </script>
    """

    # ===== 17. 拼接完整HTML =====
    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>UAV Powerline Inspection - Mission Execution</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    {css_styles}
</head>
<body>
    {control_panel}
    {plot_div}
    {js_data}
    {js_functions}
</body>
</html>
"""

    print(f"  [生成] Stage2 HTML完成（增强版 + 地图底图）")
    print(f"  [任务] {vis_stats.total_tasks} 个任务段")
    print(f"  [点数] {vis_stats.total_points} 个巡检点")

    return full_html


# =====================================================
# 新功能：带 Inspection Point 交互的动画视图
# =====================================================

def create_animation_view_with_inspection_points(
    path_3d,
    mission_json_path,
    edge_tasks=None,
    output_file="output/animation_with_inspection.html",
    downsample_step=3
):
    """
    创建清晰可读的工程展示界面

    特性：
    - Inspection Points 分级显示（endpoint/turning 默认显示，sample 默认隐藏）
    - 右侧导航面板（坐标范围提示、推荐起点、非法值提示）
    - 自动对准 mission 区域的初始视角
    - 清晰可读的工程展示

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        mission_json_path: 任务 JSON 文件路径
        edge_tasks: 边任务列表（可选）
        output_file: 输出HTML文件路径
        downsample_step: 路径降采样步长
    """
    print("[动画视图] 创建清晰可读的工程展示界面...")

    from core.topo_plan import load_mission_json_for_ui
    import json

    # 加载任务 JSON
    print(f"  [加载] 任务 JSON: {mission_json_path}")
    mission_data = load_mission_json_for_ui(mission_json_path)

    inspection_points = mission_data.get('inspection_points', [])
    print(f"  [加载] 巡检点: {len(inspection_points)} 个")

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 计算坐标范围和推荐起点
    all_x = [p[0] for p in path_3d]
    all_y = [p[1] for p in path_3d]
    all_z = [p[2] for p in path_3d]

    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    z_min, z_max = min(all_z), max(all_z)

    # 推荐起点：路径第一个点
    start_x, start_y, start_z = path_3d[0]

    # 计算相机中心点（mission 区域中心）
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    center_z = (z_min + z_max) / 2

    # 计算合适的相机距离
    range_x = x_max - x_min
    range_y = y_max - y_min
    max_range = max(range_x, range_y, z_max - z_min)
    camera_distance = max_range * 1.5

    # 路径降采样
    path_coords = path_3d[::downsample_step]
    path_x = [p[0] for p in path_coords]
    path_y = [p[1] for p in path_coords]
    path_z = [p[2] for p in path_coords]

    print(f"  [坐标范围] X:[{x_min:.0f}, {x_max:.0f}] Y:[{y_min:.0f}, {y_max:.0f}] Z:[{z_min:.1f}, {z_max:.1f}]")
    print(f"  [推荐起点] X:{start_x:.0f} Y:{start_y:.0f} Z:{start_z:.1f}")

    # 创建初始图形
    fig = go.Figure()

    # Trace 0: 原始完整路径（浅灰色虚线，参考用）
    fig.add_trace(go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        line=dict(color='#BDBDBD', width=3, dash='dot'),
        name='Full Path (Reference)',
        hovertemplate='Full Path<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 0.5: 实际执行路径（蓝色加粗，初始为空，导航后更新）
    fig.add_trace(go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode='lines',
        line=dict(color='#2196F3', width=6),
        name='Execution Path',
        hovertemplate='Execution Path<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # Trace 1: 输入点（用于导航反馈）
    fig.add_trace(go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode='markers',
        marker=dict(size=12, color='purple', symbol='circle',
                   line=dict(color='white', width=2)),
        name='Input Point',
        hovertemplate='Input Point<extra></extra>'
    ))

    # Trace 2: 最近巡检点（用于导航反馈）
    fig.add_trace(go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode='markers',
        marker=dict(size=15, color='red', symbol='diamond',
                   line=dict(color='yellow', width=2)),
        name='Nearest Point',
        hovertemplate='Nearest Point<extra></extra>'
    ))

    # Trace 3: Endpoints（红色，较大，diamond）- 默认显示
    endpoints = [p for p in inspection_points if p.get('point_type') == 'endpoint']
    if endpoints:
        fig.add_trace(go.Scatter3d(
            x=[p['pixel_position'][0] for p in endpoints],
            y=[p['pixel_position'][1] for p in endpoints],
            z=[p.get('position_3d', [p['pixel_position'][0], p['pixel_position'][1], 30])[2] if p.get('position_3d') else 30 for p in endpoints],
            mode='markers',
            marker=dict(size=12, color='red', symbol='diamond',
                       line=dict(color='white', width=2)),
            name='Endpoints',
            customdata=endpoints,
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Type: %{customdata.point_type}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))

    # Trace 4: Turning points（橙色，中等，circle）- 默认显示
    turning_points = [p for p in inspection_points if p.get('point_type') == 'turning']
    if turning_points:
        fig.add_trace(go.Scatter3d(
            x=[p['pixel_position'][0] for p in turning_points],
            y=[p['pixel_position'][1] for p in turning_points],
            z=[p.get('position_3d', [p['pixel_position'][0], p['pixel_position'][1], 30])[2] if p.get('position_3d') else 30 for p in turning_points],
            mode='markers',
            marker=dict(size=8, color='orange', symbol='circle',
                       line=dict(color='white', width=1)),
            name='Turning Points',
            customdata=turning_points,
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Type: %{customdata.point_type}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))

    # Trace 5: Sample points（青色，较小，低透明度）- 初始隐藏
    sample_points = [p for p in inspection_points if p.get('point_type') == 'sample']
    if sample_points:
        fig.add_trace(go.Scatter3d(
            x=[p['pixel_position'][0] for p in sample_points],
            y=[p['pixel_position'][1] for p in sample_points],
            z=[p.get('position_3d', [p['pixel_position'][0], p['pixel_position'][1], 30])[2] if p.get('position_3d') else 30 for p in sample_points],
            mode='markers',
            marker=dict(size=5, color='cyan', symbol='circle',
                       opacity=0.4, line=dict(color='white', width=1)),
            name='Sample Points',
            customdata=sample_points,
            visible=False,  # 初始隐藏
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Type: %{customdata.point_type}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))
    else:
        # 如果没有 sample points，添加一个空的 trace 保持索引一致
        fig.add_trace(go.Scatter3d(
            x=[], y=[], z=[],
            mode='markers',
            marker=dict(size=1, color='cyan', opacity=0.4),
            name='Sample Points',
            visible=False,
            hovertemplate='Sample Points<extra></extra>'
        ))

    # Trace 6: UAV 起点
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]],
        y=[path_y[0]],
        z=[path_z[0]],
        mode='markers',
        marker=dict(size=6, color='green', symbol='diamond'),
        name='UAV Start',
        hovertemplate='UAV Start<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 配置布局（修复可见性问题）
    inspect_ratio = mission_data['statistics']['inspect_ratio']
    total_length = mission_data['statistics']['total_length']

    # Debug 输出
    print(f"  [DEBUG] === 图形配置 ===")
    print(f"  [DEBUG] X range: [{x_min:.1f}, {x_max:.1f}] (span: {x_max - x_min:.1f})")
    print(f"  [DEBUG] Y range: [{y_min:.1f}, {y_max:.1f}] (span: {y_max - y_min:.1f})")
    print(f"  [DEBUG] Z range: [{z_min:.1f}, {z_max:.1f}] (span: {z_max - z_min:.1f})")
    print(f"  [DEBUG] Path points: {len(path_x)}")
    print(f"  [DEBUG] Endpoints: {len(endpoints)}")
    print(f"  [DEBUG] Turning points: {len(turning_points)}")
    print(f"  [DEBUG] Sample points: {len(sample_points)}")

    # 判断是否为平面数据（Z范围接近0）
    z_span = z_max - z_min
    is_planar = z_span < 1.0  # Z范围小于1米视为平面数据

    if is_planar:
        # 平面数据：使用俯视角度 + data模式
        camera_eye = dict(x=1.5, y=1.5, z=1.8)  # 俯视角度
        camera_center = dict(x=0, y=0, z=0)
        aspectmode = 'data'  # 使用data模式，让Plotly自动计算范围
        print(f"  [DEBUG] 检测到平面数据，使用俯视角度 + data模式")
    else:
        # 3D数据：使用默认视角
        camera_eye = dict(x=1.5, y=1.5, z=1.2)
        camera_center = dict(x=0, y=0, z=0)
        aspectmode = 'data'
        print(f"  [DEBUG] 使用标准3D视角")

    # 基本场景配置
    scene_config = dict(
        aspectmode=aspectmode,
        camera=dict(
            eye=camera_eye,
            center=camera_center,
            up=dict(x=0, y=0, z=1)
        ),
        xaxis=dict(
            title='X (px)',
            showgrid=False,
            showbackground=True,
            backgroundcolor="rgb(245, 245, 245)"
        ),
        yaxis=dict(
            title='Y (px)',
            showgrid=False,
            showbackground=True,
            backgroundcolor="rgb(245, 245, 245)"
        ),
        zaxis=dict(
            title='Z (m)',
            showgrid=False,
            showbackground=True,
            backgroundcolor="rgb(245, 245, 245)"
        )
    )

    fig.update_layout(
        title=dict(
            text=f"UAV Inspection Mission - {len(inspection_points)} Points (Inspect: {inspect_ratio:.1f}%) | Sample Points: {len(sample_points)}",
            font=dict(size=16, color='rgb(50, 50, 50)')
        ),
        scene=scene_config,
        width=1400,
        height=900,
        paper_bgcolor='white',
        plot_bgcolor='white',
        font=dict(family="Arial", size=11),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)", font=dict(size=10)),
        margin=dict(l=0, r=320, t=0, b=0)  # 右侧留出空间给导航面板
    )

    # 对于平面数据，额外设置Z轴范围以避免过度压缩
    if is_planar:
        z_center = (z_max + z_min) / 2
        xy_max_span = max(x_max - x_min, y_max - y_min)
        fig.update_scenes(
            zaxis=dict(
                range=[z_center - xy_max_span/2, z_center + xy_max_span/2],
                title='Z (m)',
                showgrid=False,
                showbackground=True,
                backgroundcolor="rgb(245, 245, 245)"
            )
        )
        print(f"  [DEBUG] 设置Z轴范围: [{z_center - xy_max_span/2:.1f}, {z_center + xy_max_span/2:.1f}]")

    # 生成 Plotly HTML div
    plot_div = fig.to_html(
        full_html=False,
        include_plotlyjs='cdn',
        div_id='uav-plot'
    )

    # 准备 JavaScript 数据
    path_data = [[float(p[0]), float(p[1]), float(p[2])] for p in path_3d]
    endpoints_data = [p for p in inspection_points if p.get('point_type') == 'endpoint']
    turning_points_data = [p for p in inspection_points if p.get('point_type') == 'turning']
    sample_points_data = [p for p in inspection_points if p.get('point_type') == 'sample']

    js_data = f"""
    <script>
    // 嵌入的数据
    window.UAV_DATA = {{
        path: {json.dumps(path_data)},
        inspectionPoints: {json.dumps(inspection_points)},
        endpoints: {json.dumps(endpoints_data)},
        turningPoints: {json.dumps(turning_points_data)},
        samplePoints: {json.dumps(sample_points_data)},
        totalLength: {total_length},
        downsampleStep: {downsample_step},
        bounds: {{
            x_min: {x_min:.1f}, x_max: {x_max:.1f},
            y_min: {y_min:.1f}, y_max: {y_max:.1f},
            z_min: {z_min:.1f}, z_max: {z_max:.1f}
        }},
        startPoint: {{
            x: {start_x:.1f}, y: {start_y:.1f}, z: {start_z:.1f}
        }}
    }};

    // 当前显示状态
    window.SHOW_SAMPLE = false;
    </script>
    """

    # 构建导航面板 HTML（增强版）
    nav_panel_html = f"""
    <div id="nav-panel">
        <h3>&#23548;&#36890;&#36755;&#20837;</h3>

        <!-- 坐标范围提示 -->
        <div style="background:#f5f5f5; padding:8px; border-radius:4px; margin-bottom:10px; font-size:10px; color:#666;">
            <div style="font-weight:bold; margin-bottom:3px;">坐标范围:</div>
            <div>X: {x_min:.0f} ~ {x_max:.0f}</div>
            <div>Y: {y_min:.0f} ~ {y_max:.0f}</div>
            <div>Z: {z_min:.1f} ~ {z_max:.1f}</div>
        </div>

        <!-- 推荐起点 -->
        <div style="background:#e8f5e8; padding:8px; border-radius:4px; margin-bottom:10px; font-size:10px;">
            <div style="font-weight:bold; color:#2e7d32;">推荐起点:</div>
            <div style="color:#1b5e20;">
                X: <strong>{start_x:.0f}</strong> |
                Y: <strong>{start_y:.0f}</strong> |
                Z: <strong>{start_z:.1f}</strong>
            </div>
        </div>

        <p style="font-size:11px;color:#666;margin-bottom:10px;">
            &#36755;&#20837;&#36215;&#28857;&#22352;&#26631;,&#31995;&#32479;&#23558;&#33258;&#21160;&#25509;&#20837;&#26368;&#36817;&#24037;&#28857;&#28982;&#21518;&#25191;&#34892;&#24037;&#26816;&#20219;&#21153;
        </p>
        <div class="input-row">
            <label>X:</label>
            <input type="number" id="input-x" value="{start_x:.0f}" step="1">
        </div>
        <div class="input-row">
            <label>Y:</label>
            <input type="number" id="input-y" value="{start_y:.0f}" step="1">
        </div>
        <div class="input-row">
            <label>Z:</label>
            <input type="number" id="input-z" value="{start_z:.1f}" step="1">
        </div>
        <button onclick="startNavigation()">&#24320;&#22987;&#24037;&#26816;</button>
        <div id="nav-info">
            &#31561;&#24453;&#36755;&#20837;...
        </div>

        <!-- Sample Points 开关 -->
        <div style="margin-top:15px; padding-top:15px; border-top:2px solid #ddd;">
            <div style="display:flex; align-items:center; justify-content:space-between; margin-bottom:8px;">
                <span style="font-size:12px; font-weight:bold;">Sample Points</span>
                <label style="font-size:11px; display:flex; align-items:center; cursor:pointer;">
                    <input type="checkbox" id="toggle-sample" onchange="toggleSamplePoints()" style="margin-right:4px;">
                    显示
                </label>
            </div>
        </div>

        <!-- Inspection Point 信息区域（初始隐藏） -->
        <div id="inspection-info" style="display:none; margin-top:10px; padding-top:10px; border-top:1px solid #ddd;">
            <h4 style="margin:0 0 8px 0; color:#2196F3; font-size:13px;">&#24037;&#26816;&#28857;</h4>
            <table style="width:100%; font-size:11px; border-collapse:collapse;">
                <tr><td style="padding:2px; font-weight:bold; color:#555;">ID:</td>
                    <td style="padding:2px;" id="info-point-id">-</td></tr>
                <tr><td style="padding:2px; font-weight:bold; color:#555;">Edge:</td>
                    <td style="padding:2px;" id="info-edge-id">-</td></tr>
                <tr><td style="padding:2px; font-weight:bold; color:#555;">Group:</td>
                    <td style="padding:2px;" id="info-group-id">-</td></tr>
                <tr><td style="padding:2px; font-weight:bold; color:#555;">类型:</td>
                    <td style="padding:2px;" id="info-type">-</td></tr>
                <tr><td style="padding:2px; font-weight:bold; color:#555;">顺序:</td>
                    <td style="padding:2px;" id="info-order">-</td></tr>
            </table>
        </div>
    </div>
    """

    # 构建CSS样式
    css_styles = """
    <style>
    #nav-panel {
        position: absolute;
        top: 10px;
        right: 10px;
        background: rgba(255,255,255,0.98);
        border: 1px solid #ddd;
        border-radius:6px;
        padding: 12px;
        z-index: 1000;
        box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        min-width: 220px;
        max-width: 280px;
        font-family: Arial, sans-serif;
        max-height: 95vh;
        overflow-y: auto;
    }
    #nav-panel h3 {
        margin: 0 0 8px 0;
        color: #333;
        font-size: 15px;
        text-align: center;
    }
    #nav-panel h4 {
        margin: 0 0 8px 0;
        color: #2196F3;
        font-size:13px;
    }
    #nav-panel .input-row {
        margin: 6px 0;
        display: flex;
        align-items: center;
    }
    #nav-panel label {
        display: inline-block;
        width: 20px;
        font-weight: bold;
        color:#555;
        font-size: 11px;
    }
    #nav-panel input {
        flex: 1;
        padding: 4px 6px;
        border: 1px solid #ddd;
        border-radius:3px;
        font-size: 11px;
    }
    #nav-panel button {
        width: 100%;
        background: #4CAF50;
        color: white;
        border: none;
        padding: 8px;
        margin-top: 8px;
        border-radius:4px;
        cursor: pointer;
        font-size: 13px;
        font-weight:bold;
    }
    #nav-panel button:hover {
        background: #45a049;
    }
    #nav-info {
        margin-top: 10px;
        padding-top: 8px;
        border-top:1px solid #ddd;
        font-size:11px;
        color:#666;
        line-height:1.4;
    }
    </style>
    """

    # 构建JavaScript导航函数
    js_functions = """
    <script>
    // 切换 Sample Points 显示
    function toggleSamplePoints() {
        var checkbox = document.getElementById('toggle-sample');
        window.SHOW_SAMPLE = checkbox.checked;

        var plot = document.getElementById('uav-plot');
        var traceIndex = -1;

        // 找到 Sample Points trace 索引
        plot.data.forEach(function(trace, index) {{
            if (trace.name === 'Sample Points') {{
                traceIndex = index;
            }}
        }});

        if (traceIndex >= 0) {{
            Plotly.restyle(plot, {{visible: checkbox.checked}}, [traceIndex]);
        }}
    }}

    // 显示 inspection point 信息
    function showInspectionInfo(point) {
        const infoDiv = document.getElementById('inspection-info');
        if (!infoDiv) return;

        document.getElementById('info-point-id').textContent = point.point_id || 'N/A';
        document.getElementById('info-edge-id').textContent = point.edge_id || 'N/A';
        document.getElementById('info-group-id').textContent = point.group_id || 'N/A';
        document.getElementById('info-type').textContent = point.point_type || 'N/A';
        document.getElementById('info-order').textContent = point.visit_order || 'N/A';

        infoDiv.style.display = 'block';
    }

    // 隐藏 inspection point 信息
    function hideInspectionInfo() {
        const infoDiv = document.getElementById('inspection-info');
        if (infoDiv) {
            infoDiv.style.display = 'none';
        }
    }

    // 1. 找最近巡检点，并返回该点在路径中的索引
    function findNearestInspectionPoint(inputPos) {{
        if (!UAV_DATA.inspectionPoints || UAV_DATA.inspectionPoints.length === 0) {{
            return {{point: null, distance: 0, pathIndex: -1}};
        }}

        var minDist = Infinity;
        var nearest = null;
        var nearestPathIndex = -1;

        UAV_DATA.inspectionPoints.forEach(function(p) {{
            // 获取巡检点位置（优先使用position_3d，否则用pixel_position）
            var pos = p.position_3d || p.pixel_position || p.position;
            if (!pos || pos.length < 2) {{
                return;
            }}

            var dist = Math.sqrt(
                Math.pow(inputPos[0] - pos[0], 2) +
                Math.pow(inputPos[1] - pos[1], 2) +
                Math.pow(inputPos[2] - (pos[2] || 30), 2)
            );

            if (dist < minDist) {{
                minDist = dist;
                nearest = p;
                // 找到该巡检点在 full_path 中的最近索引
                nearestPathIndex = findPathIndexForPoint(pos);
            }}
        }});
        return {{point: nearest, distance: minDist, pathIndex: nearestPathIndex}};
    }}

    // 在路径中找到最接近给定点的索引
    function findPathIndexForPoint(pointPos) {{
        var minDist = Infinity;
        var bestIndex = 0;

        for (var i = 0; i < UAV_DATA.path.length; i++) {{
            var pathPos = UAV_DATA.path[i];
            var dist = Math.sqrt(
                Math.pow(pointPos[0] - pathPos[0], 2) +
                Math.pow(pointPos[1] - pathPos[1], 2) +
                Math.pow((pointPos[2] || 30) - (pathPos[2] || 30), 2)
            );
            if (dist < minDist) {{
                minDist = dist;
                bestIndex = i;
            }}
        }}
        return bestIndex;
    }}

    // 2. 从路径索引开始截取执行路径
    function getExecutionPathFromIndex(startIndex) {{
        if (startIndex < 0 || startIndex >= UAV_DATA.path.length) {{
            return [];
        }}
        var downsampleStep = UAV_DATA.downsampleStep || 3;
        var result = [];
        for (var i = startIndex; i < UAV_DATA.path.length; i += downsampleStep) {{
            result.push(UAV_DATA.path[i]);
        }}
        return result;
    }}

    // 3. 开始导航
    function startNavigation() {{
        // 验证输入是否为数字
        var xInput = document.getElementById('input-x').value;
        var yInput = document.getElementById('input-y').value;
        var zInput = document.getElementById('input-z').value;

        if (xInput === '' || yInput === '' || zInput === '' ||
            isNaN(xInput) || isNaN(yInput) || isNaN(zInput)) {{
            document.getElementById('nav-info').innerHTML =
                '<span style="color:#d32f2f;">输入错误</span><br>' +
                '请输入有效的数字坐标';
            hideInspectionInfo();
            return;
        }}

        var x = parseFloat(xInput);
        var y = parseFloat(yInput);
        var z = parseFloat(zInput);

        // 验证坐标范围
        var bounds = UAV_DATA.bounds;
        if (x < bounds.x_min || x > bounds.x_max ||
            y < bounds.y_min || y > bounds.y_max) {{
            document.getElementById('nav-info').innerHTML =
                '<span style="color:#d32f2f;">坐标超出范围</span><br>' +
                'X范围: ' + bounds.x_min.toFixed(0) + ' ~ ' + bounds.x_max.toFixed(0) + '<br>' +
                'Y范围: ' + bounds.y_min.toFixed(0) + ' ~ ' + bounds.y_max.toFixed(0) + '<br>' +
                '请输入范围内的坐标';
            hideInspectionInfo();
            return;
        }}

        // 找最近的巡检点
        var result = findNearestInspectionPoint([x, y, z]);

        if (result.point && result.pathIndex >= 0) {{
            // 找到最近点，更新执行路径
            var plot = document.getElementById('uav-plot');

            // 1. 计算从匹配点开始的执行路径
            var execPath = getExecutionPathFromIndex(result.pathIndex);
            if (execPath.length === 0) {{
                document.getElementById('nav-info').innerHTML =
                    '<span style="color:#d32f2f;">路径计算失败</span><br>' +
                    '起点索引: ' + result.pathIndex;
                hideInspectionInfo();
                return;
            }}

            // 2. 更新 Execution Path trace (Trace 1)
            var execX = execPath.map(function(p) {{ return p[0]; }});
            var execY = execPath.map(function(p) {{ return p[1]; }});
            var execZ = execPath.map(function(p) {{ return p[2]; }});

            Plotly.restyle(plot, {{
                x: [execX], y: [execY], z: [execZ]
            }}, [1]);  // Trace 1: Execution Path

            // 3. 更新 UAV Start marker 到匹配点 (Trace 7)
            var nearestPos = result.point.position_3d || result.point.pixel_position || result.point.position;
            if (!nearestPos || nearestPos.length < 3) {{
                nearestPos = [nearestPos[0], nearestPos[1], z];
            }}
            Plotly.restyle(plot, {{
                x: [[nearestPos[0]]], y: [[nearestPos[1]]], z: [[nearestPos[2] || z]]
            }}, [7]);  // Trace 7: UAV Start

            // 4. 更新输入点 marker (Trace 2)
            Plotly.restyle(plot, {{
                x: [[x]], y: [[y]], z: [[z]]
            }}, [2]);  // Trace 2: Input Point

            // 5. 更新最近点 marker (Trace 3)
            Plotly.restyle(plot, {{
                x: [[nearestPos[0]]], y: [[nearestPos[1]]], z: [[nearestPos[2] || z]]
            }}, [3]);  // Trace 3: Nearest Point

            // 6. 显示状态信息
            document.getElementById('nav-info').innerHTML =
                '<span style="color:#2e7d32;">已匹配最近工点</span><br>' +
                '<table style="width:100%; font-size:11px; margin-top:8px;">' +
                '<tr><td style="padding:2px;"><strong>ID:</strong></td><td style="padding:2px;">' + result.point.point_id + '</td></tr>' +
                '<tr><td style="padding:2px;"><strong>Edge:</strong></td><td style="padding:2px;">' + result.point.edge_id + '</td></tr>' +
                '<tr><td style="padding:2px;"><strong>Group:</strong></td><td style="padding:2px;">' + (result.point.group_id || 'N/A') + '</td></tr>' +
                '<tr><td style="padding:2px;"><strong>类型:</strong></td><td style="padding:2px;">' + result.point.point_type + '</td></tr>' +
                '<tr><td style="padding:2px;"><strong>顺序:</strong></td><td style="padding:2px;">' + result.point.visit_order + '</td></tr>' +
                '<tr><td style="padding:2px;"><strong>距离:</strong></td><td style="padding:2px;">' + result.distance.toFixed(1) + ' m</td></tr>' +
                '<tr><td style="padding:2px;"><strong>路径点:</strong></td><td style="padding:2px;">' + execPath.length + ' 个</td></tr>' +
                '</table>';

            // 7. 显示 inspection point 信息
            showInspectionInfo(result.point);

        }} else {{
            document.getElementById('nav-info').innerHTML =
                '<span style="color:#d32f2f;">未找到工点</span><br>' +
                '可能原因:<br>' +
                '&bull; 巡检点数量为 0<br>' +
                '&bull; 输入坐标不在任务范围内';
            hideInspectionInfo();
        }}
    }}
    </script>
    """

    # 构建完整 HTML
    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>UAV Inspection - Engineering View</title>
    <script src="https://cdn.plotly/plotly-latest.min.js"></script>
    {css_styles}
</head>
<body>
    {nav_panel_html}
    {plot_div}
    {js_data}
    {js_functions}
    <script>
    // Plotly 点击事件处理 - 点击 inspection point 时显示信息
    document.getElementById('uav-plot').on('plotly_click', function(data) {{
        var point = data.points[0];
        var traceName = point.data.name;

        // 只处理 Endpoints, Turning Points, Sample Points 的点击
        if (traceName === 'Endpoints' || traceName === 'Turning Points' || traceName === 'Sample Points') {{
            var customData = point.data.customdata;
            var pointIndex = point.pointIndex;

            if (customData && customData[pointIndex]) {{
                showInspectionInfo(customData[pointIndex]);
            }} else {{
                hideInspectionInfo();
            }}
        }} else {{
            // 点击其他区域时隐藏 inspection 信息
            hideInspectionInfo();
        }}
    }});
    </script>
</body>
</html>
"""

    # 写入文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(full_html)

    print(f"  [完成] 动画已保存: {output_file}")
    print(f"  [统计] Endpoints: {len(endpoints)}, Turning: {len(turning_points)}, Sample: {len(sample_points)}")

    return full_html


def create_main_view(
    path_3d,
    inspection_points=None,
    terrain_3d=None,
    powerline_pixels=None,
    independent_lines=None,
    output_file="result/latest/main_view.html",
    title="UAV 电网巡检路径规划"
):
    """
    创建主展示页 - 基于旧 plotly_viewer.py 的高质量3D可视化风格

    图层结构：
    1. 地形层 - terrain surface
    2. 原始电网层 - 红色线
    3. 巡检路径层 - 蓝色主线
    4. Inspection points 层 - 分级显示
       - endpoint: 红色 diamond (默认显示)
       - turning: 橙色 circle (默认显示)
       - sample: 青色 circle (默认隐藏，可切换)

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        inspection_points: 巡检点列表 (从mission JSON加载)
        terrain_3d: 地形数据 (height, width, 3) array
        powerline_pixels: 原始电网像素坐标列表
        independent_lines: 独立线路列表
        output_file: 输出HTML文件路径
        title: 标题
    """
    print("[主展示页] 创建高质量3D可视化...")

    import json
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # 数据准备
    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    # 计算统计信息
    total_length = 0
    for i in range(1, len(path_3d)):
        dx = path_3d[i][0] - path_3d[i-1][0]
        dy = path_3d[i][1] - path_3d[i-1][1]
        dz = path_3d[i][2] - path_3d[i-1][2]
        total_length += np.sqrt(dx**2 + dy**2 + dz**2)

    # 计算inspect_ratio（如果有inspection_points）
    inspect_length = 0
    if inspection_points:
        # 简化计算：巡检长度约为路径长度的30-40%
        inspect_length = total_length * 0.35
    inspect_ratio = (inspect_length / total_length * 100) if total_length > 0 else 0

    # 解析inspection points
    endpoints = []
    turning_points = []
    sample_points = []

    if inspection_points:
        for p in inspection_points:
            point_type = p.get('point_type', 'unknown')
            pos = p.get('position_3d') or p.get('pixel_position')
            if pos and len(pos) >= 2:
                point_data = {
                    'x': pos[0],
                    'y': pos[1],
                    'z': pos[2] if len(pos) > 2 else 30,
                    'point_id': p.get('point_id', 'N/A'),
                    'edge_id': p.get('edge_id', 'N/A'),
                    'group_id': p.get('group_id', 'N/A'),
                    'visit_order': p.get('visit_order', 'N/A')
                }
                if point_type == 'endpoint':
                    endpoints.append(point_data)
                elif point_type == 'turning':
                    turning_points.append(point_data)
                elif point_type == 'sample':
                    sample_points.append(point_data)

    # 创建figure
    fig = go.Figure()

    # ===== 1. 地形层 =====
    if terrain_3d is not None:
        h, w = terrain_3d.shape[:2]
        # X, Y坐标网格
        x_coords = np.arange(w)
        y_coords = np.arange(h)
        X, Y = np.meshgrid(x_coords, y_coords)
        Z = terrain_3d[:, :, 2] if terrain_3d.ndim == 3 else terrain_3d

        fig.add_trace(go.Surface(
            x=X, y=Y, z=Z,
            colorscale='Earth',
            colorbar=dict(title='地形高度 (m)', x=1.02),
            opacity=0.85,
            name='地形',
            showscale=True,
            hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))
        print(f"  [地形] 已添加: {h}x{w}")
    else:
        # 添加地面参考平面
        x_min, x_max = min(path_x), max(path_x)
        y_min, y_max = min(path_y), max(path_y)
        xx, yy = np.meshgrid([x_min, x_max], [y_min, y_max])
        zz = np.full_like(xx, 0)

        fig.add_trace(go.Surface(
            x=xx, y=yy, z=zz,
            colorscale=[[0, 'rgba(240,240,240,0.4)'], [1, 'rgba(240,240,240,0.4)']],
            showscale=False,
            showlegend=False,
            hovertemplate='地面<extra></extra>'
        ))
        print(f"  [地面] 已添加参考平面")

    # ===== 2. 原始电网层 =====
    if powerline_pixels:
        # 处理 IndependentLine 对象或像素列表
        line_count = 0
        for line_obj in powerline_pixels:
            # 获取像素坐标
            if hasattr(line_obj, 'pixels'):
                pixels = line_obj.pixels
            elif isinstance(line_obj, list) and len(line_obj) > 0:
                pixels = line_obj
            else:
                continue

            if pixels:
                pl_x = [p[0] if isinstance(p, (list, tuple)) else p for p in pixels]
                pl_y = [p[1] if isinstance(p, (list, tuple)) else 0 for p in pixels]
                pl_z = [p[2] if (isinstance(p, (list, tuple)) and len(p) > 2) else 30 for p in pixels]

                fig.add_trace(go.Scatter3d(
                    x=pl_x, y=pl_y, z=pl_z,
                    mode='lines',
                    line=dict(width=2, color='rgb(255, 80, 80)'),
                    name='原始电网',
                    hovertemplate='电网<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>',
                    showlegend=(line_count == 0)  # 只显示一次图例
                ))
                line_count += 1
        print(f"  [电网] 已添加: {line_count} 条")

    # ===== 3. 巡检路径层 =====
    fig.add_trace(go.Scatter3d(
        x=path_x, y=path_y, z=path_z,
        mode='lines',
        line=dict(width=5, color='rgb(50, 100, 255)'),
        name=f'巡检路径 ({len(path_3d)}点)',
        hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 起点
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
        mode='markers',
        marker=dict(size=10, color='rgb(0, 200, 100)', symbol='diamond',
                   line=dict(width=2, color='white')),
        name='起点',
        hovertemplate='起点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 终点
    fig.add_trace(go.Scatter3d(
        x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
        mode='markers',
        marker=dict(size=10, color='rgb(255, 100, 0)', symbol='diamond',
                   line=dict(width=2, color='white')),
        name='终点',
        hovertemplate='终点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # ===== 4. Inspection Points 层 =====
    # Endpoints (红色 diamond)
    if endpoints:
        fig.add_trace(go.Scatter3d(
            x=[p['x'] for p in endpoints],
            y=[p['y'] for p in endpoints],
            z=[p['z'] for p in endpoints],
            mode='markers',
            marker=dict(size=8, color='rgb(220, 50, 50)', symbol='diamond',
                       line=dict(width=1, color='white')),
            name='Endpoints',
            customdata=endpoints,
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Group: %{customdata.group_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))

    # Turning points (橙色 circle)
    if turning_points:
        fig.add_trace(go.Scatter3d(
            x=[p['x'] for p in turning_points],
            y=[p['y'] for p in turning_points],
            z=[p['z'] for p in turning_points],
            mode='markers',
            marker=dict(size=5, color='rgb(255, 150, 50)', symbol='circle',
                       line=dict(width=1, color='white')),
            name='Turning Points',
            customdata=turning_points,
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Group: %{customdata.group_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))

    # Sample points (青色 circle, 初始隐藏)
    if sample_points:
        fig.add_trace(go.Scatter3d(
            x=[p['x'] for p in sample_points],
            y=[p['y'] for p in sample_points],
            z=[p['z'] for p in sample_points],
            mode='markers',
            marker=dict(size=3, color='rgb(0, 200, 200)', symbol='circle',
                       opacity=0.4, line=dict(width=1, color='white')),
            name='Sample Points',
            visible=False,
            customdata=sample_points,
            hovertemplate=(
                "%{customdata.point_id}<br>"
                "Edge: %{customdata.edge_id}<br>"
                "Group: %{customdata.group_id}<br>"
                "Order: %{customdata.visit_order}<extra></extra>"
            )
        ))

    # ===== 5. 布局与视角 =====
    # 计算场景范围
    x_min, x_max = min(path_x), max(path_x)
    y_min, y_max = min(path_y), max(path_y)
    z_min, z_max = min(path_z), max(path_z)
    z_span = z_max - z_min

    # 判断是否为平面数据
    is_planar = z_span < 1.0

    if is_planar:
        # 平面数据：使用俯视角度
        camera_eye = dict(x=1.5, y=1.5, z=1.8)
        # 设置Z轴范围
        xy_max_span = max(x_max - x_min, y_max - y_min)
        zaxis_range = [z_min - xy_max_span/2, z_max + xy_max_span/2]
        print(f"  [视角] 平面数据，俯视角度")
    else:
        # 3D数据：标准视角
        camera_eye = dict(x=1.3, y=1.3, z=0.9)
        zaxis_range = None
        print(f"  [视角] 标准3D视角")

    # 标题文本
    title_text = f"{title}<br><sup>"
    title_text += f"总长度: {total_length:.0f}m | "
    title_text += f"巡检比: {inspect_ratio:.1f}% | "
    title_text += f"巡检点: {len(inspection_points) if inspection_points else 0}</sup>"

    # 场景配置
    scene_config = dict(
        aspectmode='data',
        camera=dict(
            eye=camera_eye,
            center=dict(x=0, y=0, z=0),
            up=dict(x=0, y=0, z=1)
        ),
        xaxis=dict(
            backgroundcolor="rgb(245, 245, 245)",
            gridcolor="white",
            title=dict(text='X (px)', font=dict(size=14, color='rgb(50, 50, 50)'))
        ),
        yaxis=dict(
            backgroundcolor="rgb(245, 245, 245)",
            gridcolor="white",
            title=dict(text='Y (px)', font=dict(size=14, color='rgb(50, 50, 50)'))
        ),
        zaxis=dict(
            backgroundcolor="rgb(245, 245, 245)",
            gridcolor="white",
            title=dict(text='Z (m)', font=dict(size=14, color='rgb(50, 50, 50)'))
        )
    )

    # 如果是平面数据，设置Z轴范围
    if is_planar and zaxis_range:
        scene_config['zaxis']['range'] = zaxis_range

    fig.update_layout(
        scene=scene_config,
        width=1400,
        height=900,
        margin=dict(l=0, r=0, t=60, b=0),
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.8)",
            font=dict(size=11)
        ),
        title=dict(
            text=title_text,
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        font=dict(family="Arial, sans-serif"),
        paper_bgcolor='white'
    )

    # ===== 6. 生成HTML =====
    plot_div = fig.to_html(
        full_html=False,
        include_plotlyjs='cdn',
        div_id='main-plot'
    )

    # 构建完整HTML
    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>UAV Inspection - Main View</title>
    <script src="https://cdn.plotly/plotly-latest.min.js"></script>
    <style>
        body {{
            margin: 0;
            padding: 0;
            font-family: Arial, sans-serif;
            background: white;
        }}
        #control-panel {{
            position: absolute;
            top: 10px;
            right: 10px;
            background: rgba(255,255,255,0.95);
            border: 1px solid #ddd;
            border-radius: 6px;
            padding: 12px;
            z-index: 1000;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            min-width: 180px;
        }}
        #control-panel h4 {{
            margin: 0 0 10px 0;
            color: #333;
            font-size: 14px;
            text-align: center;
        }}
        .control-row {{
            margin: 8px 0;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }}
        .control-row label {{
            font-size: 12px;
            color: #555;
            cursor: pointer;
        }}
    </style>
</head>
<body>
    <div id="control-panel">
        <h4>显示控制</h4>
        <div class="control-row">
            <label for="toggle-sample">
                <input type="checkbox" id="toggle-sample" onchange="toggleSamplePoints()">
                Sample Points
            </label>
        </div>
    </div>
    {plot_div}
    <script>
    function toggleSamplePoints() {{
        var checkbox = document.getElementById('toggle-sample');
        var plot = document.getElementById('main-plot');
        var traceIndex = -1;

        plot.data.forEach(function(trace, index) {{
            if (trace.name === 'Sample Points') {{
                traceIndex = index;
            }}
        }});

        if (traceIndex >= 0) {{
            Plotly.restyle(plot, {{visible: checkbox.checked}}, [traceIndex]);
        }}
    }}
    </script>
</body>
</html>
"""

    # 写入文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(full_html)

    print(f"  [完成] 主展示页已保存: {output_file}")
    print(f"  [统计] Endpoints: {len(endpoints)}, Turning: {len(turning_points)}, Sample: {len(sample_points)}")
    print(f"  [统计] 总长度: {total_length:.0f}px, 巡检比: {inspect_ratio:.1f}%")

    return full_html


def create_map_3d_overlay(
    path_3d,
    map_image_path,
    output_file="result/latest/map_3d_overlay.html"
):
    """
    创建3D地图叠加页 - 将路径叠加到真实地图上

    基于 visualization/map_3d_overlay.py 的风格

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        map_image_path: 地图图片路径
        output_file: 输出HTML文件路径
    """
    print("[3D地图叠加] 创建3D路径叠加到真实地图...")

    from PIL import Image

    # 加载地图
    try:
        img = Image.open(map_image_path)
        img_array = np.array(img)
        height, width = img_array.shape[:2]
    except Exception as e:
        print(f"  [错误] 无法加载地图: {e}")
        return None

    # 创建figure
    fig = go.Figure()

    # 地面网格（使用地图纹理）
    x_coords = np.arange(width)
    y_coords = np.arange(height)
    X, Y = np.meshgrid(x_coords, y_coords)
    Z = np.zeros_like(X)

    # 添加地图表面
    fig.add_trace(go.Surface(
        x=X, y=Y, z=Z,
        surfacecolor=np.flipud(img_array),
        colorscale='Viridis',
        showscale=False,
        opacity=0.9,
        name='地图',
        hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
    ))

    # 提取路径坐标
    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    # 绘制路径
    fig.add_trace(go.Scatter3d(
        x=path_x, y=path_y, z=path_z,
        mode='lines',
        line=dict(width=5, color='rgb(50, 100, 255)'),
        name=f'巡检路径 ({len(path_3d)}点)',
        hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 起点
    fig.add_trace(go.Scatter3d(
        x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
        mode='markers',
        marker=dict(size=12, color='rgb(0, 200, 100)', symbol='diamond'),
        name='起点',
        hovertemplate='起点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 终点
    fig.add_trace(go.Scatter3d(
        x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
        mode='markers',
        marker=dict(size=12, color='rgb(220, 50, 50)', symbol='diamond'),
        name='终点',
        hovertemplate='终点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))

    # 布局
    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.2)),
            xaxis=dict(title='X (px)', backgroundcolor="rgb(245, 245, 245)", gridcolor="white"),
            yaxis=dict(title='Y (px)', backgroundcolor="rgb(245, 245, 245)", gridcolor="white"),
            zaxis=dict(title='Z (m)', backgroundcolor="rgb(245, 245, 245)", gridcolor="white")
        ),
        width=1400,
        height=900,
        title="UAV 巡检路径 - 3D地图叠加",
        paper_bgcolor='white'
    )

    # 保存HTML
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    fig.write_html(output_file, include_plotlyjs='cdn')

    print(f"  [完成] 3D地图叠加页已保存: {output_file}")

    return output_file


def create_map_2d_overlay(
    path_3d,
    map_image_path,
    output_file="result/latest/map_overlay.png"
):
    """
    创建2D地图叠加页 - 将路径叠加到真实地图上（PNG输出）

    基于 visualization/map_overlay.py 的风格

    Args:
        path_3d: 3D路径 [(x, y, z), ...]
        map_image_path: 地图图片路径
        output_file: 输出PNG文件路径
    """
    print("[2D地图叠加] 创建2D路径叠加到真实地图...")

    import matplotlib.pyplot as plt
    from PIL import Image

    # 加载地图
    try:
        map_img = Image.open(map_image_path)
        map_array = np.array(map_img)
        img_height, img_width = map_array.shape[:2]
    except Exception as e:
        print(f"  [错误] 无法加载地图: {e}")
        return None

    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 10), dpi=150)

    # 显示地图背景
    ax.imshow(map_array, origin='upper')

    # 提取路径坐标（只取x, y）
    path_x = [p[0] for p in path_3d]
    path_y = [p[1] for p in path_3d]
    path_z = [p[2] for p in path_3d]

    # 计算路径统计
    length = 0
    for i in range(1, len(path_3d)):
        dx = path_3d[i][0] - path_3d[i-1][0]
        dy = path_3d[i][1] - path_3d[i-1][1]
        dz = path_3d[i][2] - path_3d[i-1][2]
        length += np.sqrt(dx**2 + dy**2 + dz**2)

    avg_height = np.mean(path_z) if path_z else 0

    # 绘制路径
    ax.plot(path_x, path_y,
            color='red',
            linewidth=2.5,
            alpha=0.8,
            label='巡检路径')

    # 绘制方向箭头（每隔一段距离）
    arrow_interval = max(1, len(path_3d) // 20)
    for i in range(0, len(path_3d) - 1, arrow_interval):
        x1, y1 = path_3d[i][0], path_3d[i][1]
        x2, y2 = path_3d[i + 1][0], path_3d[i + 1][1]
        dx, dy = x2 - x1, y2 - y1
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            ax.arrow(x1, y1, dx * 0.3, dy * 0.3,
                    head_width=3, head_length=3,
                    fc='red', ec='red', alpha=0.5)

    # 绘制起点
    ax.plot(path_x[0], path_y[0], 'go',
           markersize=12,
           markeredgecolor='white',
           markeredgewidth=2,
           label='起点',
           zorder=5)

    # 绘制终点
    ax.plot(path_x[-1], path_y[-1], 'bo',
           markersize=12,
           markeredgecolor='white',
           markeredgewidth=2,
           label='终点',
           zorder=5)

    # 显示统计信息
    info_text = f'路径长度: {length:.0f}px\n'
    info_text += f'航点数: {len(path_3d)}\n'
    info_text += f'平均高度: {avg_height:.1f}m'

    props = dict(boxstyle='round', facecolor='white', alpha=0.8)
    ax.text(0.02, 0.98, info_text,
           transform=ax.transAxes,
           fontsize=10,
           verticalalignment='top',
           bbox=props)

    # 设置坐标轴
    ax.set_xlabel('X (px)', fontsize=12)
    ax.set_ylabel('Y (px)', fontsize=12)
    ax.set_title('UAV 巡检路径 - 2D地图叠加', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)

    plt.tight_layout()

    # 保存图片
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  [完成] 2D地图叠加图已保存: {output_file}")
    print(f"  [统计] 路径长度: {length:.0f}px, 航点数: {len(path_3d)}")

    return output_file


def create_main_view_from_old_viewer(
    terrain_3d,
    path_3d_pixels,
    powerline_pixels_list,
    inspection_points=None,
    output_file="result/latest/main_view.html",
    title="UAV电网巡检路径规划"
):
    """
    主展示页 - 直接基于旧 plotly_viewer.visualize_powerline_inspection() 重建

    完全按照旧函数的代码骨架实现，不做简化修改。

    图层结构：
    1. 地形表面 (terrain surface)
    2. 原始电网 (红色线)
    3. 巡检路径 (蓝色线)
    4. 起点/终点
    5. Inspection points (endpoint/turning 默认显示, sample 隐藏)

    Args:
        terrain_3d: 地形数据 (h, w, 3) array
        path_3d_pixels: 3D路径像素坐标 [(x, y, z), ...]
        powerline_pixels_list: 原始电网像素坐标列表 [[(x,y), ...], ...]
        inspection_points: 巡检点列表（从mission JSON加载）
        output_file: 输出HTML文件路径
        title: 标题
    """
    print("[主展示页] 基于旧 visualize_powerline_inspection 重建...")

    fig = go.Figure()

    # 获取地形尺寸
    h, w = terrain_3d.shape[:2]
    res = 1.0  # 像素分辨率
    b_min = [0, 0, 0]  # 边界最小值

    # ===== 1. 绘制地形表面 =====
    # 提取Z坐标
    Z = terrain_3d[:, :, 2]
    x_coords = np.arange(w) * res + b_min[0]
    y_coords = np.arange(h) * res + b_min[1]
    X, Y = np.meshgrid(x_coords, y_coords)

    fig.add_trace(go.Surface(
        x=X,
        y=Y,
        z=Z,
        colorscale='Earth',
        colorbar=dict(title='地形高度 (m)', x=1.02),
        opacity=0.85,
        name='地形',
        showscale=True,
        hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
    ))
    print(f"  [地形] 已添加: {h}x{w}")

    # ===== 2. 绘制原始电网 (红色线) =====
    if powerline_pixels_list:
        for line_pixels in powerline_pixels_list:
            if line_pixels:
                pl_x = [p[0] * res + b_min[0] for p in line_pixels]
                pl_y = [p[1] * res + b_min[1] for p in line_pixels]
                # 电网在地形上方，使用地形高度+小偏移
                pl_z = []
                for x, y in line_pixels:
                    h_idx, w_idx = int(y), int(x)
                    if 0 <= h_idx < h and 0 <= w_idx < w:
                        terrain_z = terrain_3d[h_idx, w_idx, 2]
                    else:
                        terrain_z = 30
                    pl_z.append(terrain_z + 2)  # 电网离地2米

                fig.add_trace(go.Scatter3d(
                    x=pl_x,
                    y=pl_y,
                    z=pl_z,
                    mode='lines',
                    line=dict(width=3, color='rgb(255, 50, 50)'),
                    name='原始电网',
                    hovertemplate='电网<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
                ))
        print(f"  [电网] 已添加: {len(powerline_pixels_list)} 条")

    # ===== 3. 绘制巡检路径 (蓝色线) =====
    if path_3d_pixels:
        path_x = [p[0] * res + b_min[0] for p in path_3d_pixels]
        path_y = [p[1] * res + b_min[1] for p in path_3d_pixels]
        path_z = [p[2] for p in path_3d_pixels]

        fig.add_trace(go.Scatter3d(
            x=path_x,
            y=path_y,
            z=path_z,
            mode='lines',
            line=dict(width=5, color='rgb(50, 100, 255)'),
            name=f'巡检路径 ({len(path_3d_pixels)}点)',
            hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 起点标记
        fig.add_trace(go.Scatter3d(
            x=[path_x[0]], y=[path_y[0]], z=[path_z[0]],
            mode='markers',
            marker=dict(size=8, color='rgb(0, 200, 100)', symbol='diamond'),
            name='起点',
            hovertemplate='起点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))

        # 终点标记
        fig.add_trace(go.Scatter3d(
            x=[path_x[-1]], y=[path_y[-1]], z=[path_z[-1]],
            mode='markers',
            marker=dict(size=8, color='rgb(255, 100, 0)', symbol='diamond'),
            name='终点',
            hovertemplate='终点<br>X: %{x:.0f}<br>Y: %{y:.0f}<br>Z: %{z:.1f}m<extra></extra>'
        ))
        print(f"  [路径] 已添加: {len(path_3d_pixels)} 点")

    # ===== 4. 绘制 Inspection Points =====
    if inspection_points:
        endpoints = []
        turning_points = []
        sample_points = []

        for p in inspection_points:
            point_type = p.get('point_type', 'unknown')
            pos = p.get('position_3d') or p.get('pixel_position')
            if pos and len(pos) >= 2:
                point_data = {
                    'x': pos[0],
                    'y': pos[1],
                    'z': pos[2] if len(pos) > 2 else 30,
                    'point_id': p.get('point_id', 'N/A'),
                    'edge_id': p.get('edge_id', 'N/A'),
                    'group_id': p.get('group_id', 'N/A'),
                    'visit_order': p.get('visit_order', 'N/A'),
                    'point_type': point_type
                }
                if point_type == 'endpoint':
                    endpoints.append(point_data)
                elif point_type == 'turning':
                    turning_points.append(point_data)
                elif point_type == 'sample':
                    sample_points.append(point_data)

        # Endpoints (红色 diamond)
        if endpoints:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in endpoints],
                y=[p['y'] for p in endpoints],
                z=[p['z'] for p in endpoints],
                mode='markers',
                marker=dict(size=7, color='rgb(220, 50, 50)', symbol='diamond',
                           line=dict(width=1, color='white')),
                name='Endpoints',
                customdata=endpoints,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Group: %{customdata.group_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        # Turning points (橙色 circle)
        if turning_points:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in turning_points],
                y=[p['y'] for p in turning_points],
                z=[p['z'] for p in turning_points],
                mode='markers',
                marker=dict(size=5, color='rgb(255, 150, 50)', symbol='circle',
                           line=dict(width=1, color='white')),
                name='Turning Points',
                customdata=turning_points,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Group: %{customdata.group_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        # Sample points (青色 circle, 初始隐藏)
        if sample_points:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in sample_points],
                y=[p['y'] for p in sample_points],
                z=[p['z'] for p in sample_points],
                mode='markers',
                marker=dict(size=3, color='rgb(0, 200, 200)', symbol='circle',
                           opacity=0.4, line=dict(width=1, color='white')),
                name='Sample Points',
                visible=False,
                customdata=sample_points,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Group: %{customdata.group_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        print(f"  [巡检点] Endpoints: {len(endpoints)}, Turning: {len(turning_points)}, Sample: {len(sample_points)}")

    # ===== 计算路径统计 =====
    path_length = 0
    if path_3d_pixels:
        for i in range(1, len(path_3d_pixels)):
            dx = (path_3d_pixels[i][0] - path_3d_pixels[i-1][0]) * res
            dy = (path_3d_pixels[i][1] - path_3d_pixels[i-1][1]) * res
            dz = (path_3d_pixels[i][2] - path_3d_pixels[i-1][2])
            path_length += np.sqrt(dx**2 + dy**2 + dz**2)

        min_z = min(p[2] for p in path_3d_pixels)
        max_z = max(p[2] for p in path_3d_pixels)

        title_text = (f"{title}<br>"
                     f"<sup>路径长度: {path_length:.0f}px | 航点: {len(path_3d_pixels)} | "
                     f"飞行高度: {min_z:.1f}-{max_z:.1f}m</sup>")
    else:
        title_text = title

    # ===== 设置场景布局（完全照搬旧函数） =====
    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.3, y=1.3, z=0.9),
                center=dict(x=0, y=0, z=0)
            ),
            xaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='X (px)', font=dict(size=14))
            ),
            yaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='Y (px)', font=dict(size=14))
            ),
            zaxis=dict(
                backgroundcolor="rgb(245, 245, 245)",
                gridcolor="white",
                title=dict(text='Z (m)', font=dict(size=14))
            )
        ),
        width=1200,
        height=900,
        margin=dict(l=0, r=0, t=50, b=0),
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.8)"
        ),
        title=dict(
            text=title_text,
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        font=dict(family="Arial, sans-serif"),
        paper_bgcolor='white'
    )

    # ===== 保存HTML =====
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    fig.write_html(output_file, include_plotlyjs='cdn',
                  config={'displayModeBar': True, 'displaylogo': False})

    print(f"  [完成] 主展示页已保存: {output_file}")

    return output_file


def create_main_view_with_map_background(
    path_3d_pixels,
    inspection_points,
    map_image_path="data/test.png",
    powerline_pixels_list=None,
    output_file="result/latest/main_view_map_based.html",
    title="UAV电网巡检路径规划"
):
    """
    主展示页 - 基于 data/test.png 地图底图的 2.5D 展示

    将路径、起点、终点、inspection points 叠加到地图图像上。
    将地图贴到平面上作为伪 3D 底图，路径和点略微抬高显示。

    Args:
        path_3d_pixels: 3D路径像素坐标 [(x, y, z), ...]
        inspection_points: 巡检点列表
        map_image_path: 地图图片路径（使用 data/test.png）
        powerline_pixels_list: 原始电网像素坐标列表
        output_file: 输出HTML文件路径
        title: 标题
    """
    print("[主展示页] 创建基于 data/test.png 地图底图的 2.5D 展示...")

    from PIL import Image
    import plotly.graph_objects as go

    # 加载地图图片
    try:
        map_img = Image.open(map_image_path)
        img_array = np.array(map_img)
        img_height, img_width = img_array.shape[:2]
        print(f"  [地图] 加载成功: {img_width}x{img_height}")
    except Exception as e:
        print(f"  [错误] 无法加载地图: {e}")
        return None

    fig = go.Figure()

    # ===== 1. 绘制地图底图（贴到平面 Z=0 上） =====
    x_coords = np.arange(img_width)
    y_coords = np.arange(img_height)
    X, Y = np.meshgrid(x_coords, y_coords)
    Z = np.zeros_like(X)  # 地图在 Z=0 平面上

    fig.add_trace(go.Surface(
        x=X, y=Y, z=Z,
        surfacecolor=np.flipud(img_array),  # 图片需要翻转
        colorscale='Viridis',
        showscale=False,
        opacity=1.0,
        name='地图底图',
        hovertemplate='X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
    ))
    print(f"  [底图] 已添加: data/test.png (平面 Z=0)")

    # ===== 2. 绘制原始电网（红色线，略微抬高） =====
    if powerline_pixels_list:
        powerline_z_offset = 2  # 电网略微抬高
        for line_pixels in powerline_pixels_list:
            if line_pixels:
                pl_x = [p[0] for p in line_pixels]
                pl_y = [p[1] for p in line_pixels]
                pl_z = [powerline_z_offset] * len(line_pixels)

                fig.add_trace(go.Scatter3d(
                    x=pl_x, y=pl_y, z=pl_z,
                    mode='lines',
                    line=dict(width=3, color='rgb(255, 50, 50)'),
                    name='原始电网',
                    showlegend=(powerline_pixels_list.index(line_pixels) == 0),
                    hovertemplate='电网<br>X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
                ))
        print(f"  [电网] 已添加: {len(powerline_pixels_list)} 条")

    # ===== 3. 绘制巡检路径（蓝色线，略微抬高） =====
    if path_3d_pixels:
        path_z_offset = 5  # 路径略微抬高
        path_x = [p[0] for p in path_3d_pixels]
        path_y = [p[1] for p in path_3d_pixels]
        path_z = [path_z_offset] * len(path_3d_pixels)

        fig.add_trace(go.Scatter3d(
            x=path_x, y=path_y, z=path_z,
            mode='lines',
            line=dict(width=5, color='rgb(50, 100, 255)'),
            name=f'巡检路径 ({len(path_3d_pixels)}点)',
            hovertemplate='路径<br>X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
        ))

        # 起点（略微抬高）
        fig.add_trace(go.Scatter3d(
            x=[path_x[0]], y=[path_y[0]], z=[path_z_offset + 2],
            mode='markers',
            marker=dict(size=10, color='rgb(0, 200, 100)', symbol='diamond',
                       line=dict(width=2, color='white')),
            name='起点',
            hovertemplate='起点<br>X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
        ))

        # 终点（略微抬高）
        fig.add_trace(go.Scatter3d(
            x=[path_x[-1]], y=[path_y[-1]], z=[path_z_offset + 2],
            mode='markers',
            marker=dict(size=10, color='rgb(255, 100, 0)', symbol='diamond',
                       line=dict(width=2, color='white')),
            name='终点',
            hovertemplate='终点<br>X: %{x:.0f}<br>Y: %{y:.0f}<extra></extra>'
        ))
        print(f"  [路径] 已添加: {len(path_3d_pixels)} 点")

    # ===== 4. 绘制 Inspection Points =====
    point_z_offset = 8  # inspection points 略微抬高
    if inspection_points:
        endpoints = []
        turning_points = []
        sample_points = []

        for p in inspection_points:
            point_type = p.get('point_type', 'unknown')
            pos = p.get('position_3d') or p.get('pixel_position')
            if pos and len(pos) >= 2:
                point_data = {
                    'x': pos[0],
                    'y': pos[1],
                    'z': point_z_offset,
                    'point_id': p.get('point_id', 'N/A'),
                    'edge_id': p.get('edge_id', 'N/A'),
                    'group_id': p.get('group_id', 'N/A'),
                    'visit_order': p.get('visit_order', 'N/A'),
                    'point_type': point_type
                }
                if point_type == 'endpoint':
                    endpoints.append(point_data)
                elif point_type == 'turning':
                    turning_points.append(point_data)
                elif point_type == 'sample':
                    sample_points.append(point_data)

        # Endpoints（红色 diamond）
        if endpoints:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in endpoints],
                y=[p['y'] for p in endpoints],
                z=[p['z'] for p in endpoints],
                mode='markers',
                marker=dict(size=8, color='rgb(220, 50, 50)', symbol='diamond',
                           line=dict(width=1.5, color='white')),
                name='Endpoints',
                customdata=endpoints,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        # Turning points（橙色 circle）
        if turning_points:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in turning_points],
                y=[p['y'] for p in turning_points],
                z=[p['z'] for p in turning_points],
                mode='markers',
                marker=dict(size=6, color='rgb(255, 150, 50)', symbol='circle',
                           line=dict(width=1, color='white')),
                name='Turning Points',
                customdata=turning_points,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        # Sample points（青色 circle，初始隐藏）
        if sample_points:
            fig.add_trace(go.Scatter3d(
                x=[p['x'] for p in sample_points],
                y=[p['y'] for p in sample_points],
                z=[p['z'] for p in sample_points],
                mode='markers',
                marker=dict(size=4, color='rgb(0, 200, 200)', symbol='circle',
                           opacity=0.5, line=dict(width=1, color='white')),
                name='Sample Points',
                visible=False,
                customdata=sample_points,
                hovertemplate=(
                    "%{customdata.point_id}<br>"
                    "Edge: %{customdata.edge_id}<br>"
                    "Order: %{customdata.visit_order}<extra></extra>"
                )
            ))

        print(f"  [巡检点] Endpoints: {len(endpoints)}, Turning: {len(turning_points)}, Sample: {len(sample_points)}")

    # ===== 计算路径统计 =====
    path_length = 0
    if path_3d_pixels:
        for i in range(1, len(path_3d_pixels)):
            dx = path_3d_pixels[i][0] - path_3d_pixels[i-1][0]
            dy = path_3d_pixels[i][1] - path_3d_pixels[i-1][1]
            dz = path_3d_pixels[i][2] - path_3d_pixels[i-1][2]
            path_length += np.sqrt(dx**2 + dy**2 + dz**2)

        min_z = min(p[2] for p in path_3d_pixels)
        max_z = max(p[2] for p in path_3d_pixels)

        title_text = (f"{title} - 基于 data/test.png 地图底图 (2.5D)<br>"
                     f"<sup>路径长度: {path_length:.0f}px | 航点: {len(path_3d_pixels)} | "
                     f"巡检点: {len(inspection_points) if inspection_points else 0}</sup>")
    else:
        title_text = f"{title} - 基于 data/test.png 地图底图 (2.5D)"

    # ===== 设置场景布局（俯视角度） =====
    fig.update_layout(
        scene=dict(
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.8),  # 俯视角度
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1)
            ),
            xaxis=dict(
                backgroundcolor="rgb(240, 240, 240)",
                gridcolor="white",
                title=dict(text='X (px)', font=dict(size=14, color='rgb(50, 50, 50)'))
            ),
            yaxis=dict(
                backgroundcolor="rgb(240, 240, 240)",
                gridcolor="white",
                title=dict(text='Y (px)', font=dict(size=14, color='rgb(50, 50, 50)'))
            ),
            zaxis=dict(
                backgroundcolor="rgb(240, 240, 240)",
                gridcolor="white",
                title=dict(text='Z (m)', font=dict(size=14, color='rgb(50, 50, 50)')),
                range=[0, 20]  # Z轴范围限制在 0-20 米
            )
        ),
        width=1400,
        height=900,
        margin=dict(l=0, r=0, t=60, b=0),
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.9)"
        ),
        title=dict(
            text=title_text,
            font=dict(size=18, color='rgb(50, 50, 50)')
        ),
        font=dict(family="Arial, sans-serif"),
        paper_bgcolor='white'
    )

    # 添加控制面板（用于切换 Sample Points）
    plot_div = fig.to_html(full_html=False, include_plotlyjs='cdn', div_id='main-plot')

    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>UAV Inspection - Main View (Map Based)</title>
    <script src="https://cdn.plotly/plotly-latest.min.js"></script>
    <style>
        body {{
            margin: 0;
            padding: 0;
            font-family: Arial, sans-serif;
            background: white;
        }}
        #control-panel {{
            position: absolute;
            top: 10px;
            right: 10px;
            background: rgba(255,255,255,0.95);
            border: 1px solid #ddd;
            border-radius: 6px;
            padding: 12px;
            z-index: 1000;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            min-width: 160px;
        }}
        #control-panel h4 {{
            margin: 0 0 10px 0;
            color: #333;
            font-size: 14px;
            text-align: center;
        }}
        .control-row {{
            margin: 8px 0;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }}
        .control-row label {{
            font-size: 12px;
            color: #555;
            cursor: pointer;
        }}
        .info-text {{
            font-size: 10px;
            color: #888;
            text-align: center;
            margin-top: 10px;
            font-style: italic;
        }}
    </style>
</head>
<body>
    <div id="control-panel">
        <h4>显示控制</h4>
        <div class="control-row">
            <label for="toggle-sample">
                <input type="checkbox" id="toggle-sample" onchange="toggleSamplePoints()">
                Sample Points
            </label>
        </div>
        <div class="info-text">
            基于 data/test.png<br>
            2.5D 展示
        </div>
    </div>
    {plot_div}
    <script>
    function toggleSamplePoints() {{
        var checkbox = document.getElementById('toggle-sample');
        var plot = document.getElementById('main-plot');
        var traceIndex = -1;

        plot.data.forEach(function(trace, index) {{
            if (trace.name === 'Sample Points') {{
                traceIndex = index;
            }}
        }});

        if (traceIndex >= 0) {{
            Plotly.restyle(plot, {{visible: checkbox.checked}}, [traceIndex]);
        }}
    }}
    </script>
</body>
</html>
"""

    # 保存HTML
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(full_html)

    print(f"  [完成] 主展示页已保存: {output_file}")
    print(f"  [说明] 基于 data/test.png 的 2.5D 展示，地图在 Z=0 平面")

    return output_file
