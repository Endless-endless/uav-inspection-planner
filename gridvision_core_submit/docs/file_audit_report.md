# GridVision 文件审计报告
自动生成，供毕设答辩代码筛选参考。

## KEEP

### KEEP

`app.py` (756 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`config/settings.py` (252 行)

原因：辅助模块：配置项

### KEEP

`core/image_pixel_coords.py` (35 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/independent_lines.py` (422 行)

原因：辅助模块：线路提取辅助

### KEEP

`core/inspection_point_detector.py` (539 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/inspection_point_generator.py` (655 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/physical_line_chain.py` (554 行)

原因：辅助模块：物理链几何

### KEEP

`core/real_map_cv.py` (660 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/real_satellite_manual.py` (168 行)

原因：辅助模块：成都卫星图人工标注

### KEEP

`core/topo.py` (1389 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/topo_global_optimizer.py` (2036 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/topo_plan.py` (4575 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`core/topo_task.py` (1075 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`demo/demo_visualization_main.py` (262 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`planner/mission_result_builder.py` (1030 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`planner/powerline_planner_v3_final.py` (3737 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`planner/replan_start_end.py` (1705 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`planner/topo_dijkstra.py` (468 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`visualization/dashboard_map.py` (95 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`web/index.html` (326 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`web/static/app.js` (3186 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`web/static/mission_store.js` (125 行)

原因：答辩核心模块（完整保留或已摘录）

### KEEP

`web/static/playback.js` (1722 行)

原因：答辩核心模块（完整保留或已摘录）

## REMOVE

### REMOVE

`config/__init__.py` (49 行)

原因：非答辩核心或未引用

### REMOVE

`core/__init__.py` (5 行)

原因：非答辩核心或未引用

### REMOVE

`demo/__init__.py` (8 行)

原因：CLI 入口包装，核心在 planner

### REMOVE

`input/__init__.py` (1 行)

原因：空 stub

### REMOVE

`planner/__init__.py` (3 行)

原因：非答辩核心或未引用

### REMOVE

`result/_audit_point_segment_distance.py` (100 行)

原因：运行产物与调试输出

### REMOVE

`result/dijkstra_test/dijkstra_compare_view.html` (7 行)

原因：运行产物与调试输出

### REMOVE

`result/latest/main_view_interactive.html` (1468 行)

原因：运行产物与调试输出

### REMOVE

`result/unified_mission/unified_mission_view.html` (7 行)

原因：运行产物与调试输出

### REMOVE

`scripts/diag_route_order.py` (125 行)

原因：诊断脚本，非核心算法

### REMOVE

`visualization/__init__.py` (5 行)

原因：非答辩核心或未引用

### REMOVE

`weather/__init__.py` (0 行)

原因：非答辩核心：风场模型，UI 已移除

### REMOVE

`weather/wind_model.py` (617 行)

原因：非答辩核心：风场模型，UI 已移除

### REMOVE

`web/static/style.css` (1664 行)

原因：样式细节，答辩展示非核心

