# Bug Report: rejoin 单帧越阈值导致 `ReturnActive -> ReturnRejoin` 反复切换

| 项目 | 内容 |
| --- | --- |
| 发现日期 | 2026-04-09 |
| 严重程度 | **High** — 返航主流程可完成，但日志频繁刷屏，状态切换不稳定 |
| 影响范围 | `BacktrackUseCase::handleRejoin()`、`ReturnActive/ReturnRejoin` 状态切换 |
| 根因文件 | `src/application/backtrack_usecase.cpp` |
| 修复状态 | 已修复，待实飞验证 |

---

## 1. 现象描述

在一次“直线三段、段间停顿、按钮触发返航”的实飞中，日志反复出现：

```text
ReturnActive -> ReturnRejoin
ReturnRejoin -> ReturnActive
ReturnActive -> ReturnRejoin
ReturnRejoin -> ReturnActive
...
```

原始日志时间线（节选）：

```text
[1775723961.464] ReturnPreOffboard -> ReturnActive
[1775723965.064] Deviation detected, entering rejoin mode
[1775723965.064] ReturnActive -> ReturnRejoin
[1775723966.164] Rejoin successful, returning to active mode
[1775723966.164] ReturnRejoin -> ReturnActive
[1775723966.664] Deviation detected, entering rejoin mode
[1775723966.664] ReturnActive -> ReturnRejoin
[1775723967.764] Rejoin successful, returning to active mode
[1775723967.764] ReturnRejoin -> ReturnActive
```

关键观察：

1. 每次 rejoin 持续时间很短，通常约 `1s`
2. 退出后不久又再次进入
3. 整体返航仍能继续，说明不是“卡死在 rejoin”
4. 问题更像是偏离检测对瞬时误差过于敏感

---

## 2. 根因分析

### 2.1 原实现使用“单帧越阈值”直接触发

修复前的 `handleRejoin()` 逻辑本质上是：

```cpp
if (dist_to_traj > rejoin_trigger_dist_m) {
    enterRejoin();
}

if (dist_to_traj < rejoin_success_dist_m) {
    exitRejoin();
}
```

也就是说：

- 只要某一帧距离大于 `0.5m`
- 就立即从 `ReturnActive` 进入 `ReturnRejoin`

同样地：

- 只要某一帧距离小于 `0.2m`
- 就立即退出 `ReturnRejoin`

### 2.2 缺少持续判定，容易被停顿点附近的瞬时误差触发

在当前工程里，偏离距离来自：

- 实际位置 `current_position_`
- 轨迹当前进度点 `trajectory_session_.evaluate(now).position`

对于“停顿点附近局部轨迹不够平顺”的场景，误差可能在短时间内呈现：

```text
0.42m -> 0.48m -> 0.53m -> 0.47m -> 0.39m
```

这种情况下，虽然只是一次短暂尖峰：

- 第 3 帧超过 `0.5m`，立刻进入 rejoin
- rejoin 后很快又降回 `0.2m` 内，立刻退出
- 回到 `ReturnActive` 后，下一个尖峰又再次触发

于是形成反复切换。

### 2.3 状态机层没有迟滞，UseCase 层也没有候选窗口

当前系统的 rejoin 切换信号由 `BacktrackUseCase` 直接触发状态机：

- `state_machine_.enterRejoin()`
- `state_machine_.exitRejoin()`

状态机本身不维护任何“持续超限多久才切换”的迟滞逻辑。  
如果 UseCase 层也不加候选计时器，那么系统就会把瞬时误差当成真实状态变化。

---

## 3. 故障链

```text
停顿点附近局部轨迹/跟踪误差产生短时尖峰
  -> 某一帧 dist > 0.5m
  -> 立即进入 ReturnRejoin
  -> rejoin 后很快 dist < 0.2m
  -> 立即退出 ReturnRejoin
  -> ReturnActive 中再次出现短时尖峰
  -> 再次进入 ReturnRejoin
```

表现上就是日志不断刷：

```text
ReturnActive -> ReturnRejoin -> ReturnActive -> ReturnRejoin
```

---

## 4. 修复方案

### 4.1 为进入 rejoin 增加持续超阈值判定

文件：`src/application/backtrack_usecase.cpp`

新增候选起始时间：

- `rejoin_trigger_candidate_start_time_`

进入逻辑改为：

1. 首次检测到 `dist > rejoin_trigger_dist_m` 时，仅记录开始时间
2. 只有持续超阈值达到 `rejoin_trigger_hold_s` 才真正 `enterRejoin()`
3. 期间一旦回到阈值内，候选计时清零

默认配置：

```yaml
rejoin_trigger_hold_s: 0.25
```

### 4.2 为退出 rejoin 增加持续满足判定

同样新增：

- `rejoin_success_candidate_start_time_`

退出逻辑改为：

1. 首次检测到 `dist < rejoin_success_dist_m` 时，仅记录开始时间
2. 只有持续满足达到 `rejoin_success_hold_s` 才真正 `exitRejoin()`
3. 期间一旦再次超出成功阈值，候选计时清零

默认配置：

```yaml
rejoin_success_hold_s: 0.10
```

### 4.3 配置与日志同步

文件：

- `include/px4_backtrack/domain/rejoin_policy.hpp`
- `src/infrastructure/config_loader.cpp`
- `config/backtrack_params.yaml`

同步新增两个参数，并在日志里打印：

- `rejoin_trigger_hold_s`
- `rejoin_success_hold_s`

这样联调时可以直接从启动日志确认当前迟滞配置。

---

## 5. 修复后行为

修复后的 rejoin 语义变为：

```text
短时尖峰（< 0.25s）:
  不进入 rejoin

持续偏离（>= 0.25s）:
  进入 rejoin

回归短时命中阈值（< 0.10s）:
  不退出 rejoin

稳定回归（>= 0.10s）:
  退出 rejoin
```

这本质上为 rejoin 引入了时间迟滞，避免状态机对单帧噪声过度响应。

---

## 6. 验证方法

### 6.1 日志验证

修复后重点观察：

1. 是否仍然频繁出现 `ReturnActive -> ReturnRejoin`
2. 进入 rejoin 时日志应出现：

```text
Deviation detected for X.XX s, entering rejoin mode
```

3. 退出 rejoin 时日志应出现：

```text
Rejoin successful for X.XX s, returning to active mode
```

如果日志里 `X.XX s` 基本都接近 `0.25s / 0.10s`，说明持续判定生效。

### 6.2 TC-18 复测中的再次切换说明

2026-04-11 的 TC-18 复测日志中仍然出现多次：

```text
ReturnActive -> ReturnRejoin
ReturnRejoin -> ReturnActive
ReturnActive -> ReturnRejoin
```

但这次日志已经不是本报告最初描述的“单帧越阈值立即切换”。关键区别是新日志中进入 rejoin 前均带有持续时间：

```text
1775895395.420 Deviation detected for 0.30 s, entering rejoin mode
1775895398.520 Deviation detected for 0.25 s, entering rejoin mode
1775895401.420 Deviation detected for 0.30 s, entering rejoin mode
1775895406.924 Deviation detected for 0.30 s, entering rejoin mode
1775895419.224 Deviation detected for 0.25 s, entering rejoin mode
```

这说明：

- `rejoin_trigger_hold_s` 已经生效
- 系统不是对单帧噪声立即切状态
- 每次进入 `ReturnRejoin` 前，偏离都至少持续了 `0.25s ~ 0.30s`

因此，TC-18 复测中的“仍然多次进 rejoin”不应继续归因于本 bug 的原始根因，而应按新的故障链路分析：

```text
轨迹生成/跟踪目标异常
  -> 实际位置持续偏离当前轨迹进度点
  -> 偏离持续超过 rejoin_trigger_hold_s
  -> 合法进入 ReturnRejoin
  -> 退出后再次因持续偏离进入 ReturnRejoin
```

本次 TC-18 中已经确认有两个新的独立问题会造成这种持续偏离：

1. **低速连续飞行抽稀锚点覆盖导致返航路径截断**
   录制点 `455 -> 24`，路径长度从约 `11.56m` 截断为约 `4.42m`，当前轨迹进度点与实际可飞路径不再一致。

2. **ReturnRejoin 最近点预瞄目标与冻结退出点不一致**
   旧 rejoin setpoint 追“当前位置最近点 + lookahead”，但退出判据看 frozen point，导致回归目标和成功目标不一致。

所以本报告的修复结论仍成立：它解决的是**单帧瞬时尖峰触发**。
如果后续日志仍出现多次 `ReturnActive -> ReturnRejoin`，需要先看日志中的 `Deviation detected for X.XX s`：

- 若没有 `for X.XX s` 或 `X.XX` 明显小于 `rejoin_trigger_hold_s`，说明本 bug 回归。
- 若 `X.XX >= rejoin_trigger_hold_s`，说明是持续偏离，应继续排查轨迹生成、进度推进、控制跟踪能力或 rejoin 目标一致性。

### 6.3 实飞验证

关注以下现象：

1. 停顿点附近是否不再出现密集 rejoin 切换
2. 返航轨迹轻微偏差时是否继续稳定沿轨迹飞行，而不是立即切状态
3. 确实发生持续偏离时，是否仍能正常进入 rejoin 并恢复

---

## 7. 经验总结

这个问题的本质是**把瞬时误差尖峰误判成了稳定偏离状态**。

rejoin 这种“模式切换”不应采用单帧阈值触发，而应至少具备：

- 距离阈值
- 时间持续判定

否则在真实飞行中，只要局部轨迹稍有抖动、定位稍有波动，系统就会在 `ReturnActive` 和 `ReturnRejoin` 之间来回抖动，既污染日志，也会增加控制上的不必要扰动。
