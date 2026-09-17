# Bug Report: `current_setpoint_` 残留导致返航轨迹跟踪失效

| 项目 | 内容 |
| --- | --- |
| 发现日期 | 2026-04-01 |
| 严重程度 | **Critical** — 返航主流程 100% 必现，无人机无法沿轨迹飞行 |
| 影响范围 | `ReturnActive`、`ReturnRejoin` 两个状态 |
| 根因文件 | `src/domain/state_machine.cpp` |
| 修复状态 | 已修复（两行改动） |

---

## 1. 现象描述

正常飞行并录制足够点数后，通过 CH7 触发返航：

- 轨迹正常生成（342 点，路径 5.00m，时长 24.59s）
- 成功进入 OFFBOARD 并切换到 `ReturnActive`
- **无人机原地悬停不动，未沿轨迹飞行**
- 约 3 秒后触发偏离检测，进入 `ReturnRejoin`
- 进入 `ReturnRejoin` 后继续悬停，永远无法回归轨迹

日志时间线：

```
[1775042036.28] State transition: Recording -> ReturnPreOffboard
[1775042036.28] Generating trajectory from 342 points
[1775042036.28] Trajectory generated successfully! (Duration: 24.59s, Path: 5.00m)
[1775042037.28] Mode set to: OFFBOARD
[1775042037.88] OFFBOARD entered and trajectory ready, entering ReturnActive
[1775042037.88] State transition: ReturnPreOffboard -> ReturnActive
[1775042040.83] Deviation detected, entering rejoin mode          ← 仅 ~3s 后
[1775042040.83] State transition: ReturnActive -> ReturnRejoin    ← 此后永久悬停
```

观察到的 setpoint 与实际位置：

| | x | y | z |
|---|---|---|---|
| `/mavros/setpoint_raw/local` | 4.004 | -0.138 | 0.616 |
| 当前位置 | 4.015 | -0.155 | 0.614 |
| **差值** | **0.011** | **0.017** | **0.002** |

无人机在跟随 setpoint（误差仅 ~2cm），但 setpoint 本身未随轨迹推进——它是触发返航时锁定的悬停点。

---

## 2. 根因分析

### 2.1 核心机制

`computeSetpoint()`（[backtrack_usecase.cpp:323-329](../../src/application/backtrack_usecase.cpp#L323-L329)）有一个优先级判断：

```cpp
auto state_machine_setpoint = state_machine_.getSetpointTarget();
if (state_machine_setpoint.has_value()) {
    return state_machine_setpoint.value();   // ← 有值就直接返回
}

// ReturnActive：从轨迹评估          ← 被短路，永远走不到
// ReturnRejoin：从回归策略计算       ← 被短路，永远走不到
```

`getSetpointTarget()` 返回的是状态机内部的 `current_setpoint_` 成员变量。

### 2.2 `current_setpoint_` 的生命周期

| tick 函数 | 对 `current_setpoint_` 的处理 | 是否正确 |
|---|---|---|
| `tickRecording` | `= std::nullopt` | 正确 |
| `tickCh18HoverWait` | `= createHoldPositionSetpoint(...)` | 正确 |
| `tickReturnPreOffboard` | `= createHoldPositionSetpoint(...)` | 正确 |
| **`tickReturnActive`** | **不碰（未赋值）** | **Bug** |
| **`tickReturnRejoin`** | **不碰（未赋值）** | **Bug** |
| `tickPaused` | `= std::nullopt` | 正确 |
| `tickLandingHover` | `= createHoldPositionSetpoint(...)` | 正确 |
| `tickLandingTransition` | `= std::nullopt` | 正确 |

**`tickReturnActive` 和 `tickReturnRejoin` 是唯二没有设置 `current_setpoint_` 的 tick 函数。**

### 2.3 逐 tick 故障链

```
Tick N  — ReturnPreOffboard → ReturnActive 转换发生
├─ handleRejoin()          : 状态还是 ReturnPreOffboard → 跳过
├─ state_machine_.tick()
│  └─ tickReturnPreOffboard()
│     ├─ current_setpoint_ = createHoldPositionSetpoint(locked_pos)  ← 写入悬停点
│     └─ enterState(ReturnActive)                                     ← 状态切换
│        （current_setpoint_ 未被清除，仍保留悬停点）
└─ computeSetpoint()
   └─ getSetpointTarget() 有值 → 返回 locked_pos → 轨迹代码被跳过

Tick N+1 ~ N+59  — ReturnActive 持续 ~3 秒
├─ handleRejoin()
│  └─ evaluate(now) → t_progress 持续推进 → 轨迹「期望位置」不断前移
├─ state_machine_.tick()
│  └─ tickReturnActive()
│     └─ 未碰 current_setpoint_ → 悬停点残留
└─ computeSetpoint()
   └─ getSetpointTarget() 仍有值 → 返回 locked_pos → 无人机原地不动

Tick N+60  — 偏离触发
├─ handleRejoin()
│  └─ evaluate(now) → t_progress ≈ 3.0s
│     轨迹位置距起点 ≈ 0.6m（5m路径 / 24.59s × 3s）
│     无人机仍在起点 → 偏离 0.6m > 阈值 0.5m
│     → 触发 enterRejoin()
│
├─ state_machine_.tick()
│  └─ tickReturnRejoin()
│     └─ 同样未碰 current_setpoint_ → 悬停点继续残留
└─ computeSetpoint()
   └─ getSetpointTarget() 仍有值 → rejoin 目标计算被跳过 → 永久悬停
```

### 2.4 可视化

```
位置轴 ──────────────────────────────────►

                          0.5m 阈值
                         ◄────────►
locked_pos                        轨迹 t=3s 位置
(setpoint)                        (evaluate 返回)
    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●─ ─ ─ ─ ─ ─► 轨迹继续前进
    ▲                               ▲
    │                               │
    无人机（跟随 setpoint）          handleRejoin 认为无人机应该在这里
                                    │
                                 距离 ≈ 0.6m > 0.5m → 触发 rejoin

时间轴 ──────────────────────────────────►
    t=0      t=1      t=2      t=3
    ├────────┼────────┼────────┤
    │                          │
    轨迹起点                    0.6m 处
    无人机始终在此               轨迹期望位置在此
```

---

## 3. 是否 100% 必现

**是，每次触发返航都必现。** 这不是时序竞争或概率问题。

只要走过 `ReturnPreOffboard → ReturnActive` 这条路径（所有返航触发方式——CH7 长按、RC 丢失——都经过此路径），`tickReturnPreOffboard()` 必定写入 `current_setpoint_`，而 `tickReturnActive()` 必定不清除它。

唯一的变量是从触发 rejoin 到发现问题的时间，取决于轨迹速度：

- 阈值 0.5m，平均速度 v m/s → 大约 `0.5/v` 秒后触发
- 本例：v ≈ 0.2 m/s → 约 2.5~3s 后触发

CH18 故障确认路径同理：`Ch18HoverWait` 的 `tickCh18HoverWait()` 也写入 `current_setpoint_`，后续进入 `ReturnPreOffboard → ReturnActive` 后同样不清除。

---

## 4. 修复方案

在 `tickReturnActive()` 和 `tickReturnRejoin()` 中显式清除 `current_setpoint_`，让 `computeSetpoint()` 能走到轨迹评估 / 回归策略计算的分支。

### 文件：`src/domain/state_machine.cpp`

**改动 1 — `tickReturnActive()`（第 294 行附近）：**

```cpp
 void BacktrackStateMachine::tickReturnActive(const TickInput& input) {
+  current_setpoint_ = std::nullopt;  // 清除固定点，让UseCase层从轨迹计算setpoint
   should_publish_setpoint_ = true;
   current_mode_request_ = std::nullopt;
```

**改动 2 — `tickReturnRejoin()`（第 313 行附近）：**

```cpp
 void BacktrackStateMachine::tickReturnRejoin(const TickInput& input) {
+  current_setpoint_ = std::nullopt;  // 清除固定点，让UseCase层从回归策略计算setpoint
   should_publish_setpoint_ = true;
   current_mode_request_ = std::nullopt;
```

### 修复后执行链路

```
Tick N+1（ReturnActive）:
├─ tickReturnActive() → current_setpoint_ = std::nullopt
├─ computeSetpoint()
│  └─ getSetpointTarget() → nullopt → 进入轨迹评估分支
│     └─ evaluate(now) → 返回轨迹当前位置 → 发布给飞控
└─ 无人机跟随轨迹前进 ✓
```

---

## 5. 验证方法

### 5.1 单元测试验证

在 `tickReturnActive` 和 `tickReturnRejoin` 的测试中添加断言：

```cpp
// 从 ReturnPreOffboard 进入 ReturnActive 后
sm.tick(input_offboard_traj_ready);
EXPECT_EQ(sm.getCurrentState(), State::ReturnActive);
EXPECT_FALSE(sm.getSetpointTarget().has_value());  // ← 关键断言
```

### 5.2 实飞 / SITL 验证

| 检查项 | 预期行为 |
|---|---|
| 进入 ReturnActive 后 setpoint 是否变化 | `/mavros/setpoint_raw/local` 的 position 应随时间变化 |
| 无人机是否沿轨迹移动 | 肉眼 / rviz 可观察到无人机沿录制路径反向飞行 |
| 是否误触发 rejoin | 正常跟踪时不应触发（偏离 < 0.5m） |
| 轨迹完成后是否正常降落 | 应进入 LandingHover → LandingTransition → AUTO.LAND |

---

## 6. 经验总结

此 Bug 的本质是**状态机成员变量跨状态泄漏**——上一个状态写入的 `current_setpoint_` 被下一个状态无意间继承。

**设计原则：每个状态的 tick 函数应完整声明本状态所有输出变量的值，不依赖前驱状态的残留值。**

可考虑的防御措施：

1. 在 `enterState()` 中统一重置 `current_setpoint_ = std::nullopt`，而非依赖各 tick 函数自行清除
2. 添加静态分析 checklist：每个 tick 函数必须显式设置 `current_setpoint_`、`should_publish_setpoint_`、`current_mode_request_` 三个输出变量