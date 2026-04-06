# 多电路一次性巡检 - 诊断报告

**诊断时间**: 2026-04-01  
**诊断对象**: Step 9.2 GroupedContinuousMission  
**诊断目标**: 评估现有实现是否满足"多电路一次性巡检"需求

---

## A. 当前 Step 9.2 已实现了什么

### 1. 拓扑建模层（已完成）
- ✅ 从12条独立线路构建拓扑图
- ✅ 21个拓扑节点（endpoint + split）
- ✅ 12条拓扑边（TopoEdge）
- ✅ 边任务建模（EdgeTask，132个巡检点已映射）

### 2. 分组规划层（已实现）
- ✅ 基于 DBSCAN 的空间分组（eps=150px）
- ✅ 4个空间分组（Group_0~3）
- ✅ 分组内连续规划（adjacency-based）
- ✅ 分组间贪心连接（最近 centroid）

### 3. 路径输出层（已实现）
- ✅ 23个 segments（12个 inspect + 11个 connect）
- ✅ 边访问顺序优化（模拟退火）
- ✅ 标准化 JSON 导出（mission_output.json）
- ✅ 3D 全路径生成

### 4. 可视化层（已实现，不动）
- ✅ `demo/demo_ui_animation.py` - 动画入口
- ✅ `core/visualization_enhanced.py` - 可视化逻辑
- ✅ 读取 mission_output.json 展示结果

---

## B. 距离"多电路一次性巡检"最终目标还差什么

### 关键缺陷 1：连接路径不沿拓扑图

**当前实现**：
```python
def generate_connection_segment(point_a, point_b):
    geometry = [point_a, point_b]  # 直线！
    length = np.linalg.norm(np.array(point_b) - np.array(point_a))
    return geometry, length
```

**问题**：
- 所有11个连接段都是2点直线
- 没有使用拓扑图的已有路径
- 长连接（如1287px）直接跨越，可能中间有可走的拓扑边

**证据**：
```
seg_0007: (12, 919) -> (1177, 372), length=1287px
seg_0009: (1182, 361) -> (1392, 744), length=437px
seg_0021: (1373, 950) -> (1567, 1272), length=376px
```

这些连接段应该沿着拓扑图的真实路径走，而不是直线穿越。

### 关键缺陷 2：adjacency_cost 计算后未使用

**当前实现**：
- ✅ `build_adjacency_cost()` 计算了拓扑代价
- ✅ `compute_transition_cost_simple()` 考虑了图路径
- ❌ 但 `generate_connection_segment()` 没有使用图路径

**断层**：
```
compute_transition_cost_simple() → 返回 (cost, is_adjacent, graph_path)
                                        ↓
                                   graph_path 被忽略
                                        ↓
generate_connection_segment() → 只用两点画直线
```

### 关键缺陷 3：分组参数未优化

**当前参数**：
- DBSCAN eps=150px（硬编码）
- min_samples=1（默认）

**可能问题**：
- eps=150 可能导致错误分组或过度分割
- 缺乏参数验证和敏感性分析

---

## C. 哪些问题属于 topo graph 建模问题

### 基本良好，但可增强

**现状**：
- ✅ 12条拓扑边正确识别
- ✅ 节点去重避免过度切分
- ✅ EdgeTask 正确映射巡检点

**可选增强**（非必须）：
- T型连接识别（Phase 1.2）
- Junction 合并（如果存在多路交汇）

**判断**：
- topo graph 建模基本满足需求
- 不需要大改

---

## D. 哪些问题属于 task grouping / adjacency cost 问题

### 核心问题：连接段生成逻辑

**问题定位**：
1. **grouping** - DBSCAN 参数可能需要调优（次要）
2. **adjacency cost** - 计算正确，但未被使用（主要）
3. **连接段生成** - 完全未使用拓扑路径（主要）

**代码断层**：
```python
# topo_plan.py:1106
def generate_connection_segment(point_a, point_b):
    # ❌ 只返回直线
    return [point_a, point_b], length

# 应该改为：
def generate_connection_segment_along_topo(
    point_a, point_b, topo_graph, edge_task_map
):
    # 1. 找到 point_a, point_b 对应的拓扑节点
    # 2. 查询拓扑图最短路径
    # 3. 沿路径展开为 geometry
    # 4. 返回完整路径
```

---

## E. 哪些问题其实不用改代码，只需要重新解释现有结构

### 误判1："需要重做UI"

**实际情况**：
- 现有 UI 完全支持多电路展示
- `mission_output.json` 已包含完整数据
- 可视化只需要读取 JSON，不需要修改

### 误判2："需要新建final html"

**实际情况**：
- `demo/demo_ui_animation.py` 就是最终入口
- `demo/demo_visualization_main.py` 也是入口
- 只需要确保 JSON 格式不变

### 误判3："Step 9.2 不完整"

**实际情况**：
- Step 9.2 已实现完整流程
- 唯一问题是连接段生成逻辑
- 修改1个函数即可，不需要重做

---

## F. 最小修改方案：改哪 1-2 个模块，为什么

### 核心修改：`core/topo_plan.py`

**修改点 1**：`generate_connection_segment()` 函数

**改为**：`generate_connection_segment_along_topo()`

**逻辑**：
1. 接收 `point_a`, `point_b`, `topo_graph`, `edge_task_map`
2. 找到最近的拓扑节点
3. 使用 `get_shortest_path()` 获取图路径
4. 沿路径展开为 geometry
5. 返回完整的多点路径（而非2点直线）

**影响**：
- 连接段从2点变为多点
- 连接段真正沿着拓扑图走
- 总长度可能略增（但更真实）

---

**修改点 2**：`build_grouped_continuous_mission()` 函数

**改为**：传递 `topo_graph` 参数给连接段生成

**逻辑**：
```python
# 当前（第1969行）：
connect_geo, connect_len = generate_connection_segment(
    current_end_point,
    get_edge_geometry_with_direction(...)[0]
)

# 改为：
connect_geo, connect_len = generate_connection_segment_along_topo(
    current_end_point,
    get_edge_geometry_with_direction(...)[0],
    topo_graph,
    edge_task_map
)
```

---

### 可选修改：DBSCAN eps 参数

**位置**：`step9_2_plan_grouped_continuous_mission(eps=150.0)`

**建议**：
- 先保持 eps=150
- 运行诊断工具分析参数敏感性
- 如有需要再调整

---

## 总结：修改优先级

### 必须（P0）：
1. ✅ 修改 `generate_connection_segment()` 使用拓扑路径
2. ✅ 修改 `build_grouped_continuous_mission()` 传递拓扑图参数

### 可选（P1）：
3. ⚠️ DBSCAN eps 参数调优（需要诊断）

### 不需要（P-）：
4. ❌ 不修改 UI 层
5. ❌ 不新建 final html
6. ❌ 不改 topo graph 建模

---

## 预期效果

修改后的连接段将：
- 从2点直线 → 多点路径
- 从几何近邻 → 拓扑真实路径
- 保持 JSON 格式不变
- 现有可视化无需修改

**结论**：最小修改方案 = 修改1个核心函数 + 1个调用点
