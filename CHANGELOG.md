# Changelog

## 2026-04-10 — 轨迹失败退化为 waypoint 返航

为 `trajectory_core` 的可恢复非几何退化失败补上退化返航链路，避免系统在 `ReturnPreOffboard` 卡住。

### 修复

- **feat(usecase):** `BacktrackUseCase::handleTrajectoryGeneration()` 在可恢复的非“点过近”失败时切换到 waypoint 退化返航，不再只记日志后停留在 `ReturnPreOffboard`
- **feat(domain):** 新增 `WaypointFallbackSession`，负责将录制点逆序重采样为 0.3m 稀疏路径，并按当前位置推进 waypoint 目标
- **fix(usecase):** 退化返航继续复用 `/mavros/setpoint_raw/local` 的 `PositionTarget` 发布链，不引入新的 ROS 控制接口
- **fix(usecase):** 退化返航暂停后恢复从头开始，正常轨迹返航仍保持原有断点续航语义
- **fix(trajectory):** 非法 `trajectory_core` 配置显式分类为 `InvalidConfig`，不会被 waypoint fallback 掩盖

### 测试与文档

- **test(fallback):** 新增 `test_waypoint_fallback_session.cpp`，覆盖稀疏路径构建、推进和 restart 行为
- **test(usecase):** `test_backtrack_usecase.cpp` 增加“非法配置不会误入 waypoint 退化返航”用例
- **docs(readme/llm):** README、LLM_MAP 和技术文档同步更新退化返航的当前实现语义

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `src/domain/waypoint_fallback_session.cpp`
- `include/px4_backtrack/domain/waypoint_fallback_session.hpp`
- `tests/unit/test_backtrack_usecase.cpp`
- `tests/unit/test_waypoint_fallback_session.cpp`
- `CMakeLists.txt`
- `README.md`
- `LLM_MAP.md`
- `doc/返航实现技术文档.md`

## 2026-04-10 — 点过近退化轨迹触发原地降落

修复“起飞后原地悬停，录制点数足够但点过近/起终点几何退化，触发返航后轨迹生成失败却未降级降落”的问题。

### 修复

- **fix(usecase):** `BacktrackUseCase::handleTrajectoryGeneration()` 仅在 `TrajectorySession::generate()` 因“点过近/起终点几何退化”失败时触发原地降落，不再把所有生成失败都一刀切降级
- **refactor(trajectory):** `TrajectorySession` 新增最近一次生成失败分类，供 usecase 精确区分“点过近退化”与其它生成失败

### 测试与文档

- **test(usecase):** 新增 `test_backtrack_usecase.cpp`，覆盖“原地悬停录到足够点数、返航生成失败后应进入 `LandingTransition` 并请求 `AUTO.LAND`”场景
- **docs(readme/llm):** README 和 LLM_MAP 同步补充“仅点过近/起终点几何退化失败时降级原地降落”的当前行为语义
- **docs(test):** `doc/返航功能测试文档.md` 将 `BUG-01` 标记为已修复，并保留 `TC-19` 作为对应回归用例

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `src/domain/trajectory_session.cpp`
- `include/px4_backtrack/domain/trajectory_session.hpp`
- `tests/unit/test_backtrack_usecase.cpp`
- `tests/unit/test_trajectory_session.cpp`
- `CMakeLists.txt`
- `README.md`
- `LLM_MAP.md`
- `doc/返航功能测试文档.md`

## 2026-04-10 — 抑制停顿点抽动与 rejoin 抖动

针对“直线分段飞行、段间短暂停顿”场景补强返航稳定性，降低停顿点附近的抽动和 `ReturnActive -> ReturnRejoin` 反复切换。

### 修复

- **fix(trajectory):** `TrajectorySession::generate()` 在真正调用 `trajectory_core` 前先对录制点做空间抽稀，使用 `max(min_point_dist_m, ds_m * 0.5)` 压缩停顿悬停形成的高密度点云团，减少停顿点附近的局部折返和抽动
- **fix(rejoin):** `BacktrackUseCase::handleRejoin()` 为进入/退出 rejoin 增加持续判定，新增 `rejoin_trigger_hold_s` / `rejoin_success_hold_s`，避免单帧越阈值导致 `ReturnActive` 与 `ReturnRejoin` 反复切换
- **fix(config):** `trajectory_config.yaml` 将 `min_point_dist_m` 从 `0.01` 提高到 `0.05`，并在 `backtrack_params.yaml` 中新增 rejoin 持续判定参数

### 测试与文档

- **test(trajectory):** `test_trajectory_session.cpp` 新增高密度停顿点云团场景，验证轨迹长度不会因停顿漂移而明显膨胀
- **docs(bug):** 新增两份 bug report，分别记录“停顿点高密度录制导致返航抽动”和“rejoin 单帧越阈值导致状态反复切换”
- **docs(readme/llm):** README 和 LLM_MAP 同步补充“生成前抽稀”和“rejoin 持续判定”的当前行为语义

### 涉及文件

- `src/domain/trajectory_session.cpp`
- `src/application/backtrack_usecase.cpp`
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `include/px4_backtrack/domain/rejoin_policy.hpp`
- `src/infrastructure/config_loader.cpp`
- `config/trajectory_config.yaml`
- `config/backtrack_params.yaml`
- `tests/unit/test_trajectory_session.cpp`
- `doc/BUG_REPORT_停顿点高密度录制导致返航抽动.md`
- `doc/BUG_REPORT_rejoin单帧越阈值导致状态反复切换.md`

## 2026-04-08 — 参数与测试文档同步调整

结合实飞联调结果回调返航与轨迹参数，并同步刷新返航功能测试文档中的基线参数表与测试项。

### 修改

- **update(config):** `record_min_points` 从 `100` 调整为 `30`，降低返航触发所需录制点数门槛
- **update(config):** `record_max_hz` 从 `10.0` 调整为 `15.0`，提高录制频率上限
- **update(config):** `rejoin_success_dist_m` 从 `0.1` 调整为 `0.2`，放宽回归成功阈值
- **update(config):** `trajectory_config.yaml` 中 `v_max` 从 `2.0` 调整为 `1.0`，`vz_max` 从 `0.8` 调整为 `0.6`

### 文档

- **docs(test):** `doc/返航功能测试文档.md` 参数基线改为与当前 YAML 保持一致，并补充 `trajectory_core` 参数表

### 涉及文件

- `config/backtrack_params.yaml`
- `config/trajectory_config.yaml`
- `doc/返航功能测试文档.md`

## 2026-04-08 — 修复 ReturnRejoin 期间进度未冻结

修复进入 `ReturnRejoin` 后轨迹进度仍按 wall time 推进，导致回归目标漂移到轨迹末端、无人机无法自主回归的问题。

### 修复

- **fix(rejoin):** 在 `BacktrackUseCase::handleRejoin()` 中，进入 rejoin 时调用 `trajectory_session_.freezeProgress()`，退出 rejoin 时调用 `trajectory_session_.resumeProgress(now)`，使 rejoin 与 pause 语义保持对称

### 测试与文档

- **test(state_machine):** 补充状态机相关测试覆盖，验证进入 rejoin 后不会再残留固定点 setpoint
- **docs(bug):** 新增两份 bug report，分别记录 `ReturnRejoin` 期间进度未冻结问题和 `current_setpoint_` 残留导致返航悬停问题
- **docs(test):** `doc/返航功能测试文档.md` 补充实飞问题记录与用例结果

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `tests/unit/test_state_machine.cpp`
- `doc/BUG_REPORT_rejoin进度未冻结导致轨迹跟踪失败.md`
- `doc/BUG_REPORT_setpoint残留导致返航悬停.md`
- `doc/返航功能测试文档.md`

## 2026-04-01 — 修复固定点 setpoint 残留导致返航悬停

修复 `ReturnPreOffboard` 写入的固定点 setpoint 在进入 `ReturnActive` / `ReturnRejoin` 后未清除，导致 `computeSetpoint()` 一直被短路到悬停点的问题。

### 修复

- **fix(state_machine):** `tickReturnActive()` 和 `tickReturnRejoin()` 显式将 `current_setpoint_` 置为 `std::nullopt`，让 usecase 可以正确从轨迹评估和 rejoin 策略计算 setpoint

### 涉及文件

- `src/domain/state_machine.cpp`

## 2026-03-30 — 补充测试文档

补齐仓库的单元测试说明文档和返航功能测试文档，便于后续联调、验收与缺陷回归追踪。

### 新增

- **docs(test):** 新增 `doc/单元测试文档.md`，说明当前 `gtest` 覆盖范围、运行方式、结果判读与扩展建议
- **docs(test):** 新增 `doc/返航功能测试文档.md`，定义返航主流程、触发、rejoin、降落和 `AUTO.LAND` 让步等测试项

### 文档

- **docs(readme):** `README.md` 新增测试文档入口与目录索引

### 涉及文件

- `doc/单元测试文档.md`
- `doc/返航功能测试文档.md`
- `README.md`

## 2026-03-26 — 触发参数配置来源收口到 YAML

清理 `TriggerEvaluator` 触发参数的双重默认来源，避免头文件默认构造值与 `backtrack_params.yaml` 同时存在导致行为漂移。

### 修改

- **refactor(config):** `TriggerConfig` 移除 `CH7` / `CH18` / `RC loss` 业务默认值，改为仅承载配置数据
- **refactor(config):** `ConfigLoader` 对触发参数改为必填读取；参数服务器缺失时直接报错，不再静默回退到代码默认值
- **test(trigger):** `TriggerEvaluator` 单测改为显式构造触发配置，避免测试继续依赖头文件隐式默认值

### 文档

- **docs(readme):** 明确 `backtrack_params.yaml` 是触发参数的唯一运行时来源
- **docs(llm):** 补充触发参数加载链路与“不要恢复头文件默认值”的约束

### 涉及文件

- `include/px4_backtrack/domain/trigger_evaluator.hpp`
- `src/infrastructure/config_loader.cpp`
- `tests/unit/test_trigger_evaluator.cpp`
- `README.md`
- `LLM_MAP.md`

## 2026-03-25 — 新增 Debug 可视化功能

新增可选的 debug 输出能力：轨迹生成成功后自动导出录制航点与返航轨迹数据，支持离线 CSV 分析和 rviz 实时可视化。

### 新增

- **feat(debug):** 新增 `DebugDumper` (infrastructure 层)，轨迹生成成功后将录制航点和采样轨迹写入带时间戳的 CSV 文件，输出目录可配置
- **feat(debug):** 新增 `DebugPublisher` (adapters 层)，发布录制航点 (`MarkerArray`) 和返航轨迹 (`nav_msgs/Path`) 到 latched debug 话题，供 rviz 实时查看
- **feat(debug):** 新增 Python 可视化脚本 `scripts/plot_debug_csv.py`，支持 3D 路径对比、俯视图、高度剖面和偏航剖面绘制
- **feat(config):** `backtrack_params.yaml` 新增 4 个 debug 参数：`debug_enable_csv_dump`、`debug_csv_output_dir`、`debug_trajectory_sample_dt`、`debug_enable_rviz_publish`

### 修改

- **refactor(usecase):** `BacktrackUseCase` 新增 `setDebugDumper()` / `setDebugPublisher()` setter 注入和 `performDebugDump()` 方法，在轨迹生成成功后自动调用
- **build:** `CMakeLists.txt` 和 `package.xml` 新增 `visualization_msgs` 依赖，新增 `debug_dumper.cpp` 和 `debug_publishers.cpp` 编译目标

### 涉及文件

- `include/px4_backtrack/infrastructure/debug_dumper.hpp` (新增)
- `src/infrastructure/debug_dumper.cpp` (新增)
- `include/px4_backtrack/adapters/debug_publishers.hpp` (新增)
- `src/adapters/debug_publishers.cpp` (新增)
- `scripts/plot_debug_csv.py` (新增)
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `src/application/backtrack_usecase.cpp`
- `src/backtrack_node.cpp`
- `include/px4_backtrack/infrastructure/config_loader.hpp`
- `src/infrastructure/config_loader.cpp`
- `config/backtrack_params.yaml`
- `CMakeLists.txt`
- `package.xml`

## 2026-03-25 — 新增核心领域层单元测试

为当前 C++ 实现补充第一批 `gtest` 单元测试，并把测试目标接入 `catkin` 构建链路。

### 新增

- **test(recording):** 新增 `RecordingBuffer` 单元测试，覆盖首点时间戳、录制节流、非单调时间戳拒收、`reset()` 复位
- **test(trigger):** 新增 `TriggerEvaluator` 单元测试，覆盖 `CH7` 按下/长按、`CH18` 故障确认、RC loss 的边沿/电平语义
- **test(state_machine):** 新增 `BacktrackStateMachine` 单元测试，覆盖 ARM 进入录制、`ReturnPreOffboard -> ReturnActive`、`AUTO.LAND` 全局优先级、直接降落路径
- **test(rejoin):** 新增 `RejoinPolicy` 单元测试，覆盖偏离/回归阈值、最近点搜索、预瞄目标与速度限幅
- **test(trajectory):** 新增 `TrajectorySession` 单元测试，覆盖点数不足、逆序轨迹生成、暂停恢复进度、完成态判断

### 构建

- **build(test):** `CMakeLists.txt` 新增 `catkin_add_gtest` 测试目标，并为当前工作空间显式提供共享 `test_main.cpp`，修复测试目标缺失 `main()` 的链接问题

### 文档

- **docs(system):** `doc/SYSTEM_OVERVIEW.md` 补充状态机主状态迁移图、全局覆盖规则与 `LandingTransition` 内部阶段图
- **docs(llm):** `LLM_MAP.md` 更新测试现状说明，明确仓库已具备基础领域层单元测试
- **docs(readme):** `README.md` 补充测试目录、测试命令与 `SYSTEM_OVERVIEW` 文档入口

### 涉及文件

- `CMakeLists.txt`
- `doc/SYSTEM_OVERVIEW.md`
- `tests/unit/test_main.cpp`
- `tests/unit/test_recording_buffer.cpp`
- `tests/unit/test_trigger_evaluator.cpp`
- `tests/unit/test_state_machine.cpp`
- `tests/unit/test_rejoin_policy.cpp`
- `tests/unit/test_trajectory_session.cpp`

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
