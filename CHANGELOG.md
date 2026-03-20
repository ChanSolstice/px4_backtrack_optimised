# Changelog

## 2026-03-20 — 代码审核修复

基于同事审核反馈，修复 4 项编译/逻辑/并发问题并增强触发降级逻辑。

### 修复

- **fix(state_machine):** `tickLandingTransition` 中 `input.current_time` 改为 `input.now`，修复编译错误
- **fix(usecase):** 飞行周期复位只生效一次的问题 — DISARM 时清零 `arm_after_disarm_detected_`，使后续架次能正确清空旧航迹
- **fix(trigger):** RC 丢失检测拆分为 `checkRCLossEvent()`（边沿触发）和 `isRCLost()`（电平查询），修复 `/status_rc_loss` 在触发返航的同一 tick 就回到 `false` 的语义错误
- **fix(subscribers):** `StateSubscriber` 和 `RCSubscriber` 改为 copy-then-invoke 模式，释放锁后再调用外部回调，与 `OdomSubscriber` 保持一致

### 增强

- **feat(usecase):** 新增 `handleReturnTrigger()` 统一收口触发逻辑；触发时若 `record_min_points` 不满足，降级为原地降落而非忽略
- **feat(usecase):** Paused 状态下 RC loss 和 CH7 恢复也检查点数条件，不足时直接降落
- **feat(state_machine):** 新增 `triggerLanding()`，允许 usecase 直接发起降落流程
- **feat(state_machine):** `tickLandingTransition` 在非 OFFBOARD 模式下直接请求 `AUTO.LAND`，跳过无意义的 POSCTL 过渡

### 涉及文件

- `include/px4_backtrack/domain/state_machine.hpp`
- `src/domain/state_machine.cpp`
- `include/px4_backtrack/domain/trigger_evaluator.hpp`
- `src/domain/trigger_evaluator.cpp`
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `src/application/backtrack_usecase.cpp`
- `src/adapters/ros_subscribers.cpp`
