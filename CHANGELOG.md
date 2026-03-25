# Changelog

## 2026-03-25 — trajectory_core API 对齐与文档刷新

对齐新版 `trajectory_core` API 文档，并同步更新返航侧时间戳接入语义与仓库文档。

### 修复

- **fix(trajectory):** `TrajectorySession::generate()` 不再按 `record_max_hz` 生成人造等间隔时间轴，改为对逆序录制时间做镜像映射 `t_new = t_last - t_rev`，在保持单调递增的同时保留原始采样间隔用于 `trajectory_core` 的时间质量诊断
- **fix(trajectory):** 生成结果告警处理从旧的 `StatusCode::TIME_INVALID_FALLBACK` 迁移到 `Result` warning 接口，改用 `hasTimeFallback()` / `isCorridorExceeded()`

### 重构

- **refactor(usecase):** `TrajectorySession::generate()` 移除 `record_max_hz` 参数，调用链同步收敛为“录制缓存提供时间戳，轨迹会话负责逆序+镜像映射”
- **refactor(logging):** 轨迹生成日志补充 `TimeReport.time_mode`，便于联调时区分正常时间模式与 fallback 模式

### 文档

- **docs(api):** `doc/API接口文档.md` 按新版 `trajectory_core` 能力刷新，补充 `warning_flags`、`ResultWarning`、`curvatureFromArcLength()`、逐点 `dt` yaw limiter、YAML 覆盖范围等说明
- **docs(system):** 新增 `doc/SYSTEM_OVERVIEW.md`，提供基于当前 C++ 实现整理的系统级概览
- **docs(tech):** `doc/返航实现技术文档.md` 中的 waypoint 时间戳构造规则更新为镜像映射语义
- **docs(llm):** `LLM_MAP.md` 更新 `trajectory_core` 时间戳接入真相，并补充对高层概览文档使用边界的提醒

### 涉及文件

- `include/px4_backtrack/domain/trajectory_session.hpp`
- `src/domain/trajectory_session.cpp`
- `src/application/backtrack_usecase.cpp`
- `doc/API接口文档.md`
- `doc/SYSTEM_OVERVIEW.md`
- `doc/返航实现技术文档.md`
- `LLM_MAP.md`

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
