# 巡检执行模块 - 快速参考

## 🎯 功能概述

巡检执行模块 (`core/execution.py`) 提供了从任意位置开始执行巡检任务的能力。

## 🔧 核心函数

### 1. find_closest_index
```python
from core.execution import find_closest_index

idx = find_closest_index(path_3d, current_pos)
```
**功能**: 找到路径中距离当前位置最近的点索引

**参数**:
- `path_3d`: 3D路径点列表 `[(x, y, z), ...]`
- `current_pos`: 当前位置 `(x, y)`

**返回**: 最近点的索引 `int`

---

### 2. get_sub_path
```python
from core.execution import get_sub_path

sub_path, sub_tasks, start_idx = get_sub_path(path_3d, tasks, current_pos)
```
**功能**: 截取从当前位置开始的剩余子路径和任务

**参数**:
- `path_3d`: 完整3D路径
- `tasks`: 完整任务列表
- `current_pos`: 当前位置

**返回**: `(sub_path, sub_tasks, start_index)`

---

### 3. simulate_execution
```python
from core.execution import simulate_execution

result = simulate_execution(sub_path, sub_tasks)
```
**功能**: 模拟巡检执行过程

**参数**:
- `sub_path`: 子路径
- `sub_tasks`: 子任务列表

**返回**: 执行统计字典
```python
{
    'total_steps': 总步骤数,
    'tasks_completed': 完成任务数,
    'total_distance': 总飞行距离
}
```

---

## 🚀 使用示例

### 示例1: 从路径中间开始
```python
from core.execution import get_sub_path, simulate_execution
from core.planner import plan_powerline_inspection

# 生成路径
planner = plan_powerline_inspection(...)
path_3d = planner.path_3d
tasks = planner.tasks

# 假设UAV在路径1/3处
current_pos = (path_3d[len(path_3d)//3][0], path_3d[len(path_3d)//3][1])

# 获取子路径并执行
sub_path, sub_tasks, idx = get_sub_path(path_3d, tasks, current_pos)
result = simulate_execution(sub_path, sub_tasks)

print(f"执行了 {result['total_steps']} 步")
print(f"完成 {result['tasks_completed']} 个任务")
print(f"飞行 {result['total_distance']:.1f} 米")
```

### 示例2: 完整演示
```bash
python demo/demo_execution.py
```

---

## 📊 输出示例

```
======================================================================
开始模拟巡检执行
======================================================================
[执行] 步骤 1: UAV位置 = (960.0, 716.0, 63.8)
  [任务] 执行拍照 - 相机角度: -45°
  [任务] 任务ID: 1
[执行] 步骤 2: UAV位置 = (967.0, 736.0, 63.9)
[执行] 步骤 3: UAV位置 = (973.0, 756.0, 66.0)
...
======================================================================
巡检执行完成
======================================================================
总步骤: 21
完成任务: 3
飞行距离: 679.0 米
```

---

## 🎓 答辩要点

1. **完整执行流程**: "我们实现了从任意位置开始执行巡检的完整逻辑"

2. **自动定位**: "系统可以自动定位UAV在路径中的位置"

3. **智能匹配**: "到达任务点时自动触发对应的拍照任务"

4. **逐步输出**: "每一步的执行状态都可以清晰看到"

---

## 🔗 相关模块

- `core/planner.py` - 路径规划
- `core/mission.py` - 任务生成
- `demo/demo_execution.py` - 演示程序

---

## 📅 版本信息

- **版本**: V3.1
- **状态**: 已验证，可演示
- **日期**: 2026-03-22
