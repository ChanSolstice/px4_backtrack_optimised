# Bug Report: ReturnRejoin 期间轨迹进度未冻结导致回归失败

| 项目 | 内容 |
| --- | --- |
| 发现日期 | 2026-04-01 |
| 严重程度 | **Critical** — 进入 rejoin 后 100% 无法自主回归轨迹 |
| 影响范围 | `ReturnRejoin` 状态及后续 `ReturnActive` 恢复 |
| 根因文件 | `src/application/backtrack_usecase.cpp` |
| 关联 Bug | `BUG_REPORT_setpoint残留导致返航悬停.md`（Bug 1 修复后才能观察到本 Bug） |
| 修复状态 | 已修复（两行改动） |

---

## 1. 现象描述

Bug 1（`current_setpoint_` 残留）修复后，无人机进入 `ReturnActive` 可以开始沿轨迹飞行。但一旦因任何原因触发偏离检测进入 `ReturnRejoin`，表现为：

- **无人机在 rejoin 期间无法自主回归轨迹，持续悬停**
- 手动暂停（CH7）后手控将无人机拉向起飞点附近，恢复后可以完成降落——但这不是正常流程

实飞日志时间线：

```
[1775045060.005]  ReturnPreOffboard -> ReturnActive
[1775045062.905]  Deviation detected -> ReturnRejoin         ← 进入 rejoin
                  ... 约 197 秒无人机悬停在 ReturnRejoin ...
[1775045259.705]  CH7 暂停 -> Paused                         ← 手动介入
                  ... 手控飞向起飞点 ...
[1775045272.005]  CH7 恢复 -> ReturnPreOffboard
[1775045274.005]  ReturnPreOffboard -> ReturnActive
[1775045274.055]  Deviation detected -> ReturnRejoin         ← 1 tick 内立即再次触发
                  ... 46 秒后 ...
[1775045320.355]  Rejoin successful + Trajectory finished     ← 因为已经在起飞点附近
[1775045320.355]  ReturnActive -> LandingHover -> 正常降落
```

关键观察：

1. 第一次 rejoin 持续 197 秒无法退出
2. 暂停恢复后 1 tick 内立即再次触发 rejoin
3. 最终降落只因手动操控到了起飞点附近

---

## 2. 根因分析

### 2.1 对比 Pause 的正确实现

系统中已有一个正确的「暂停轨迹进度」实现——Pause 流程：

```cpp
// 暂停时（handleTriggers, line 195-196）
state_machine_.triggerPause();
trajectory_session_.freezeProgress();     // ✓ 冻结进度

// 恢复时（handleTriggers, line 209）
trajectory_session_.resumeProgress(now);  // ✓ 从冻结点恢复
```

### 2.2 Rejoin 流程的遗漏

Rejoin 场景在逻辑上同样需要暂停轨迹进度（无人机脱离轨迹期间，让进度停在偏离点等待回归），但代码中缺少 freeze/resume 调用：

```cpp
// handleRejoin() — 进入 rejoin（line 300-303）
if (rejoin_policy_.shouldTriggerRejoin(...)) {
    // ✗ 缺少 trajectory_session_.freezeProgress()
    state_machine_.enterRejoin();
}

// handleRejoin() — 退出 rejoin（line 316-319）
if (rejoin_policy_.shouldExitRejoin(...)) {
    // ✗ 缺少 trajectory_session_.resumeProgress(now)
    state_machine_.exitRejoin();
}
```

### 2.3 逐阶段故障链

**阶段 1 — 进入 ReturnRejoin，进度持续推进：**

```
时间轴 ──────────────────────────────────────────────►

轨迹进度（t_progress_）：
    t=0     t=2.9s                    t=36.34s (轨迹总时长)
    ├────────●                            │
    起点     偏离点                         终点(起飞点)
             ↑ 此时触发 rejoin
             │ 但 t_progress_ 没有冻结 → 继续以 wall time 推进

    t=2.9s   t=10s    t=36.34s   t=100s   t=199.7s
    ├────────┼────────┼──────────┼─────────┤
    偏离点    │        轨迹末端     ← t_progress_ 仍在增长（被 evaluate 钳位到 duration）
```

**阶段 2 — 退出条件无法满足：**

`shouldExitRejoin` 的判断：

```cpp
bool shouldExitRejoin(current_pos, traj_pos) {
    return (current_pos - traj_pos).norm() < success_dist_m;  // 0.1m
}
```

- `traj_pos` = `evaluate(now)` 返回的位置
- 当 `t_progress_` 超过 `duration` 后，`evaluate()` 钳位返回轨迹末端（起飞点）
- 退出条件变成：**无人机到起飞点的距离 < 0.1m**
- 无人机在半途偏离，距起飞点可能有数米 → 条件几乎不可能满足

```
无人机位置 (偏离点附近)              起飞点 (轨迹末端)
    ●─────────── 数米距离 ───────────●
    ▲                                ▲
    │                                │
  current_pos                    evaluate(now) 返回此处
                                 因为 t_progress_ >> duration
                                     │
                  dist = 数米 >> 0.1m → 永不退出
```

**阶段 3 — 暂停恢复后立即再次触发 rejoin：**

从日志看：

```
[1775045272.005]  resumeProgress(now)  ← frozen_progress_ ≈ 199.7s（在 pause 时冻结的）
                  t_start_walltime_ = now - 199.7s
[1775045274.005]  进入 ReturnActive
[1775045274.055]  evaluate(now) → t_progress_ ≈ 201.7s → 钳位到轨迹末端
                  无人机当前位置 vs 起飞点 → 偏离 → 1 tick 内立即触发 rejoin
```

`resumeProgress()` 原样恢复了那个已经跑飞到 199.7s 的进度，导致：

- `evaluate()` 立即返回轨迹末端（起飞点）
- 此时无人机虽然被手动拉近了起飞点，但仍有偏离
- 1 tick 内再次触发 rejoin

最终之所以能降落：手控已把无人机拉到距起飞点较近的位置，rejoin 持续 46 秒后，`shouldExitRejoin` 终于满足（无人机自然漂移到 < 0.1m），然后 `isFinished()` 立即为 true（因为 t_progress_ >> duration），直接进入 `LandingHover`。

### 2.4 Rejoin vs Pause 对称性对比

| 操作 | Pause 流程 | Rejoin 流程（修复前） |
|---|---|---|
| 进入时冻结进度 | `freezeProgress()` | **缺失** |
| 退出时恢复进度 | `resumeProgress(now)` | **缺失** |
| 进度行为 | 冻结在暂停时刻，恢复后无缝继续 | 持续推进，与无人机实际位置脱节 |

---

## 3. 是否 100% 必现

**只要触发过 rejoin，就 100% 必现。**

进入 `ReturnRejoin` 的前提是偏离超过 `rejoin_trigger_dist_m`（0.5m），这在以下场景下可能发生：

- 风扰导致短暂偏离
- 轨迹速度过快无人机跟不上
- OFFBOARD 切换瞬间的定位跳变
- Bug 1 未修复时（100% 触发）

一旦进入 rejoin，由于进度不冻结，退出条件会在数秒内从「回归偏离点」退化为「飞到起飞点」——这在飞行中途几乎不可能满足。

---

## 4. 修复方案

在 `handleRejoin()` 中为 rejoin 的进入和退出添加 freeze/resume 调用，与 Pause 流程对称。

### 文件：`src/application/backtrack_usecase.cpp`

**改动 1 — 进入 rejoin 时冻结进度（第 300 行附近）：**

```cpp
 if (rejoin_policy_.shouldTriggerRejoin(current_position_, traj_state.position)) {
     ROS_WARN("[BacktrackUseCase] Deviation detected, entering rejoin mode");
+    trajectory_session_.freezeProgress();  // 冻结轨迹进度，防止退出条件目标持续漂移
     state_machine_.enterRejoin();
 }
```

**改动 2 — 退出 rejoin 时恢复进度（第 316 行附近）：**

```cpp
 if (rejoin_policy_.shouldExitRejoin(current_position_, traj_state.position)) {
     ROS_INFO("[BacktrackUseCase] Rejoin successful, returning to active mode");
+    trajectory_session_.resumeProgress(ros::Time::now().toSec());  // 从冻结点继续
     state_machine_.exitRejoin();
 }
```

### 修复后行为

```
进入 rejoin:
  t_progress_ 冻结在偏离时刻（如 t=2.9s）
  → evaluate() 始终返回 t=2.9s 处的轨迹位置
  → 退出条件 = 回归到偏离点附近（< 0.1m）  ✓

退出 rejoin:
  resumeProgress(now) → t_start_walltime_ = now - 2.9s
  → evaluate() 从 t=2.9s 无缝继续推进
  → 无人机从偏离点恢复正常轨迹跟踪  ✓
```

---

## 5. 验证方法

### 5.1 单元测试验证

```cpp
// 进入 rejoin 后，进度应冻结
usecase.handleRejoin();  // 触发偏离
EXPECT_TRUE(trajectory_session.isProgressFrozen());

// 模拟时间推移
ros::Time::setNow(ros::Time::now() + ros::Duration(10.0));
auto state = trajectory_session.evaluate(ros::Time::now().toSec());
EXPECT_NEAR(state.t, frozen_time, 0.01);  // 进度不变

// 退出 rejoin 后，进度应恢复
// ... 将无人机移到轨迹附近 ...
usecase.handleRejoin();  // 触发退出
EXPECT_FALSE(trajectory_session.isProgressFrozen());
```

### 5.2 实飞 / SITL 验证

| 检查项 | 预期行为 |
|---|---|
| 进入 rejoin 后日志中 `t_progress_` 是否固定 | 应保持在偏离时刻的值不变 |
| rejoin 状态下 setpoint 是否指向偏离点 | 不应指向起飞点或轨迹末端 |
| 回归偏离点后是否退出 rejoin | 偏离距离 < 0.1m 时应退出 |
| 退出 rejoin 后是否继续沿轨迹飞行 | 应从偏离点处无缝继续 |
| 不再需要手动操控即可完成返航 | 全自主完成 |

---

## 6. 经验总结

此 Bug 的本质是**语义对称性遗漏**——系统中已有「暂停轨迹进度」的正确实现（Pause 流程的 freeze/resume），但在语义上完全相同的 Rejoin 场景中没有复用这一机制。

**设计原则：当无人机不沿轨迹飞行时（Pause、Rejoin、任何脱轨场景），必须冻结轨迹进度。wall time 推进的 `t_progress_` 只在无人机实际跟踪轨迹时才有物理意义。**

可考虑的防御措施：

1. 将「是否允许进度推进」绑定到状态机状态——只有 `ReturnActive` 状态下 `evaluate()` 才推进进度，其他状态自动冻结
2. 在 `evaluate()` 中增加断言或告警：如果 `t_progress_` 超过 `duration` 但调用方不是 `isFinished()` 检查，说明进度管理可能有遗漏
