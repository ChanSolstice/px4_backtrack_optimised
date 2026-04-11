# Bug Report: ReturnRejoin 最近点预瞄目标与冻结退出点不一致

| 项目 | 内容 |
| --- | --- |
| 发现日期 | 2026-04-11 |
| 严重程度 | **High** — 进入 `ReturnRejoin` 后可能原地抽动，无法稳定退出并继续返航 |
| 关联用例 | TC-18 直线分三段飞行返航验证 |
| 影响范围 | `ReturnRejoin` setpoint 计算、回归成功判据、返航中断恢复 |
| 根因文件 | `src/application/backtrack_usecase.cpp`、`src/domain/rejoin_policy.cpp` |
| 修复状态 | 已修复，待实飞复测 |

---

## 1. 现象描述

TC-18 中，修复低速段抽稀截断问题前，返航到第二段附近后出现：

- 状态进入 `ReturnRejoin`
- 机体在当前位置附近抽动
- 长时间没有 `Rejoin successful`
- 无法继续正常返航，只能人工暂停或接管

关键日志：

```text
[BacktrackUseCase] Deviation detected for 0.25 s, entering rejoin mode
[StateMachine] State transition: ReturnActive -> ReturnRejoin
```

最后一次进入 `ReturnRejoin` 后，直到人工暂停前没有再退出：

```text
1775895419.223888303 Deviation detected for 0.25 s, entering rejoin mode
1775895419.223956844 State transition: ReturnActive -> ReturnRejoin
1775895471.169879546 Ch7 press event detected, pausing
```

虽然这次实飞同时受到“抽稀截断轨迹”的影响，但 `ReturnRejoin` 本身也存在独立设计缺陷：**控制目标和退出判据使用的不是同一个轨迹点**。

---

## 2. 根因分析

### 2.1 进入 Rejoin 时冻结的是轨迹进度点

进入 `ReturnRejoin` 时，系统调用：

```cpp
trajectory_session_.freezeProgress();
```

这意味着 `TrajectorySession::evaluate(now)` 在 rejoin 期间会持续返回进入 rejoin 那一刻的冻结进度点。

退出判据也使用这个冻结点：

```cpp
auto traj_state = trajectory_session_.evaluate(now);

if (rejoin_policy_.shouldExitRejoin(current_position_, traj_state.position)) {
  ...
}
```

也就是说，`ReturnRejoin` 的成功条件是：

```text
当前位置距离冻结进度点 < rejoin_success_dist_m
```

### 2.2 旧 setpoint 却追“最近点 + lookahead”

旧的 `ReturnRejoin` setpoint 计算逻辑：

```cpp
double t_nearest = rejoin_policy_.findNearestTime(
    current_position_, trajectory_session_.getTrajectory());
auto rejoin_state = rejoin_policy_.computeRejoinTarget(
    current_position_, trajectory_session_.getTrajectory(), t_nearest);
```

`computeRejoinTarget()` 内部会做：

```cpp
double t_target = t_nearest + config_.lookahead_time_s;
auto target_state = traj->evaluate(t_target);
target_state.velocity = limitVelocity(target_state.velocity, config_.v_max_m_s);
```

因此控制目标是：

```text
当前位置最近轨迹点 + lookahead
```

但退出目标是：

```text
进入 rejoin 时冻结的轨迹进度点
```

这两个点在真实轨迹上经常不是同一个点。

### 2.3 目标不一致会造成“追不到退出点”

问题链路：

```text
进入 ReturnRejoin
  -> t_progress 冻结在偏离发生时刻
  -> 退出判据要求回到 frozen point

setpoint 计算
  -> 每帧重新搜索当前位置的最近轨迹点
  -> 再加 lookahead
  -> 控制目标可能落在 frozen point 前方、后方或另一段相近轨迹上

结果
  -> 飞机被控制去追 lookahead 目标
  -> 但状态机等待它进入 frozen point 成功圈
  -> 在直线分段、停顿、低速尾段或轨迹自接近处，容易卡住或抽动
```

特别是在 TC-18 这类直线分段路径中，轨迹局部速度很低、停顿点附近空间距离接近，最近点搜索容易让控制目标落到与冻结点不同的位置。此时即使飞机在“当前 setpoint”附近，也可能仍然不满足退出判据。

### 2.4 复用轨迹速度会放大慢速尾段问题

旧策略直接取 `Trajectory::evaluate(t_target)` 的速度，再做限幅：

```cpp
target_state.velocity = limitVelocity(target_state.velocity, config_.v_max_m_s);
```

如果 `t_target` 落在低速或末端收敛段，轨迹自身速度可能已经接近 0。此时即使当前位置距离退出点还有明显误差，发布速度也可能很小，表现为原地缓慢抽动。

---

## 3. 修复方案

### 3.1 ReturnRejoin 直接回拉到 Frozen Point

修复后，`ReturnRejoin` 不再搜索最近点，也不再加 `lookahead`。

主链路改为：

```cpp
auto frozen_state = trajectory_session_.evaluate(now);
auto rejoin_state = rejoin_policy_.computeFrozenPointTarget(
    current_position_, frozen_state);
```

其中 `trajectory_session_.evaluate(now)` 在 rejoin 期间返回的就是冻结进度点。

这样控制目标和退出判据统一为同一个点：

```text
ReturnRejoin setpoint target = frozen point
ReturnRejoin success target  = frozen point
```

### 3.2 使用比例收敛速度

新增 `RejoinPolicy::computeFrozenPointTarget()`：

```cpp
Vec3 delta = frozen_state.position - current_pos;
double distance = delta.norm();

double speed = std::min(config_.v_max_m_s, 1.0 * distance);
target_state.velocity = delta * (speed / distance);
```

速度语义：

- 距离较远时，以 `rejoin_v_max_m_s` 限速靠近
- 进入近距离后，速度随距离线性减小
- 到达冻结点附近时速度归零

这样可以避免到点附近仍给出过大速度导致穿越，也避免复用低速轨迹段速度导致拉不回来。

### 3.3 Yaw 复用 Frozen Point 姿态

修复后：

```cpp
auto target_state = frozen_state;
```

目标状态先完整复制冻结点轨迹状态，再只覆盖 velocity。

因此：

- position 来自冻结点
- yaw 来自冻结点
- yaw_rate 来自冻结点
- 不额外计算“朝向目标点”的 yaw

这样回归阶段不会因为位置拉回而引入额外原地转向扰动。

---

## 4. 修复后行为

### 4.1 正常进入 Rejoin

```text
ReturnActive
  -> 偏离冻结进度点超过 rejoin_trigger_dist_m 且持续达到 hold
  -> freezeProgress()
  -> ReturnRejoin
```

### 4.2 Rejoin 控制

```text
target_position = p_traj(t_progress_frozen)
target_velocity = normalize(target_position - current_position)
                  * min(rejoin_v_max_m_s, distance)
target_yaw      = yaw_traj(t_progress_frozen)
target_yaw_rate = yaw_rate_traj(t_progress_frozen)
```

### 4.3 Rejoin 退出

```text
if ||current_position - p_traj(t_progress_frozen)|| < rejoin_success_dist_m:
    resumeProgress(now)
    ReturnActive
```

控制目标与退出目标完全一致。

---

## 5. 验证方法

### 5.1 单元测试

新增测试：

- `tests/unit/test_rejoin_policy.cpp`
- `FrozenPointTargetPullsTowardFrozenPositionWithProportionalSpeed`
- `FrozenPointTargetStopsWhenAlreadyAtFrozenPosition`

覆盖点：

- 远距离时速度被 `rejoin_v_max_m_s` 限幅
- 近距离时速度按距离比例减小
- 已到冻结点时速度归零
- yaw 和 yaw_rate 复用冻结点姿态

### 5.2 实飞 / 联调验证

建议复测 TC-18 时关注：

1. 进入 `ReturnRejoin` 后，setpoint 位置是否固定在进入 rejoin 时的轨迹进度点。
2. `/mavros/setpoint_raw/local` 中速度方向是否指向该冻结点。
3. 靠近冻结点时速度是否逐步减小。
4. 进入 `rejoin_success_dist_m` 范围并持续满足 `rejoin_success_hold_s` 后，是否稳定退出到 `ReturnActive`。
5. 回归过程中 yaw 是否平稳，不出现额外原地转向。

---

## 6. 经验总结

`ReturnRejoin` 的控制目标和退出判据必须使用同一个参考点。

旧策略把控制目标定义为“当前位置最近点 + lookahead”，却把退出目标定义为“冻结进度点”，这会在真实飞行轨迹中制造不可解释的回归失败。

对于当前返航实现，冻结进度点是最闭环的 rejoin 目标：

- 它是进入 rejoin 时系统认为“本该到达”的点
- 它已经被进度冻结机制稳定保存
- 它和退出判据天然一致
- 它不依赖最近点搜索，避免自接近轨迹和停顿点附近的歧义

后续如果重新引入 lookahead，必须同时重定义退出判据，保证 setpoint 目标与 success target 仍然一致。
