# Result Directory Structure

## latest/ - 主展示结果（基于修复后的 topology）
当前推荐查看的输出集合，基于 core/topo.py 修复后的完整结果。

**主要文件：**
- `main_view_interactive.html` - 交互式主展示页（推荐首先查看）
- `mission_output.json` - 标准任务输出 JSON
- `map_overlay_test.png` - 2D 地图叠加图
- `coverage_difference_map.png` - 覆盖差集验收图
- `segment_source_map.png` - 段来源验收图
- `inspect_connect_verification.png` - Inspect vs Connect 验收图

**验证数据：**
- L_003: 558.0px (100% 覆盖)
- L_004: 531.0px (100% 覆盖，分两条边)
- 总 Inspect 长度: 2572.55px
- 点级别覆盖率: 100%

## steps/ - Pipeline 中间步骤可视化
展示各个处理阶段的中间结果，用于理解算法流程。

- `step1_hsv_mask.png` - HSV 颜色提取
- `step2_fixed_mask.png` - 形态学修复
- `step3_skeleton.png` - 骨架提取
- `step4_independent_lines.png` - 独立线路识别
- `step5_line_inspection_points.png` - 巡检点生成
- `step7_5_topo_nodes.png` - 拓扑节点
- `step8_5_edge_task_summary.png` - 边任务汇总
- `step8_5_topo_edges.png` - 拓扑边
- `step9_2_edge_groups.png` - 边分组
- `step9_2_topo_plan_grouped.png` - 分组规划

## archive/ - 已归档的旧版本结果
包含修复前基于错误路径生成的旧文件，仅用于历史对比。

- `main_view_start_driven.html` - 旧版起点驱动页面（基于截断路径）
- `start_driven_mission.json` - 旧版起点驱动 JSON
- `mission_stage2_demo.html` - 旧版 demo 页面

## 修复说明 (2026-04-01)
修复了 core/topo.py 中合并节点排序逻辑的 bug，该 bug 导致 L_003 和 L_004 的 edge 被错误截断。

**修改位置：** core/topo.py, split_line_to_edges() 函数
**修改内容：**
1. get_sort_key() 对合并节点使用正确的端点索引
2. edge 创建时优先使用端点索引而非 min/max

**修复效果：**
- L_003: 253点 → 394点 (恢复 141点)
- L_004: 179点 → 345点 (恢复 166点)
- 总覆盖率: 75.3% → 100%
