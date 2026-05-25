# Mission System Governance

**阶段：** Mission System Governance Phase  
**目标：** 统一 Mission 来源、App 阶段、可视化图层生命周期（非功能扩张）

---

## 权威状态源

| 模块 | 文件 | 职责 |
|------|------|------|
| **MissionStore** | `web/static/mission_governance.js` | 当前 mission 唯一读写入口 |
| **AppPhaseManager** | 同上 | idle / planning / mission_ready / playing / paused / adaptive_warning / replanning / finished |
| **LayerManager** | `web/static/layer_manager.js` | L0–L3 长期层 + T1–T4 临时层生命周期 |

`state.lastResult` 现为 **MissionStore 只读代理**（getter/setter），旧代码逐步迁移中。

---

## inspection_point_source

**仅允许：** `spacing` | `image`

输入别名（`image_points`、`detect` 等）在 `MissionStore.normalizeSource()` 中归一化。

---

## 四类 Mission 修改规则

| 类型 | 允许 | 禁止 |
|------|------|------|
| 初始 plan | 生成 spacing/image 点 | — |
| Server replan | 改 segments / visit order | 切换 source；image 模式下重采样点 |
| Client adaptive | 改 **connect** segment 几何 | 改 inspection_points；写 JSON |
| Predictive | 触发 adaptive（同 client 规则） | 独立改 mission 结构 |

---

## Playback 同步

- 读取：`MissionStore.getMission()`（playback.js `getMissionForPlayback()`）
- Server replan / adaptive connect 完成后：`syncPlaybackAfterMissionChange()` → `reloadPlaybackTimeline()` / `resetPlayback()`

---

## UI 治理

`inspection_point_source === image` 时隐藏：

- `#spacingInput` 所在 field
- `#planningSpacingInput` 所在 field

---

## Dead Code

见 `docs/DEAD_CODE_REGISTRY.md`（本阶段只标记，不删除）。
