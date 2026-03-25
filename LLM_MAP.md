# LLM_MAP

这份文档不是给最终用户看的，而是给后续阅读、解释或修改本仓库的 LLM/自动化代理看的。目标是让模型先建立正确的代码地图，再动手改动，减少把旧版 Python 文档误当成现状、或者只改一层没改全链路的风险。

## 先记住这 10 条

1. 这是 ROS1 `catkin` 工程，不是 ROS2。
2. 当前主实现入口是 [`src/backtrack_node.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/backtrack_node.cpp)。
3. 控制主循环在 `BacktrackNode` 的 20Hz `timerCallback()`，不是在订阅回调里直接控飞。
4. 真正的编排核心在 [`src/application/backtrack_usecase.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/application/backtrack_usecase.cpp)。
5. 返航轨迹由 `trajectory_core` 生成，配置来自 [`config/trajectory_config.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/trajectory_config.yaml)。
6. 坐标语义按 ENU 处理，但 setpoint 通过 `mavros_msgs/PositionTarget` 发到 `/mavros/setpoint_raw/local`。
7. 录制点保留相对时间戳；生成轨迹时会在逆序后把时间戳镜像成新的单调递增序列，以保留原始 dt 间隔用于 `trajectory_core` 的时间质量诊断。
8. `doc/TECHNICAL_DOCUMENTATION.md` 和 [`doc/backtrack_node.py`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/backtrack_node.py) 是旧版 Python 基线，不要把它们当成当前代码的直接事实来源。
9. 需要判断“行为真相”时，优先级是：`src/*.cpp` + `include/*.hpp` > `config/*.yaml` > `launch/` > `doc/返航实现技术文档.md` > 旧版 Python 文档。
10. 当前代码有开发中痕迹，写说明或补丁前要先检查接口是否自洽。
11. 仓库现在已经有基础 `gtest` 单元测试，优先先跑 `RecordingBuffer` / `TriggerEvaluator` / `StateMachine` / `RejoinPolicy` / `TrajectorySession`，不要再默认“完全没有测试”。

## 真正的入口与控制流

控制流从 [`src/backtrack_node.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/backtrack_node.cpp) 开始：

1. `ConfigLoader::loadFromROS()` 读取 ROS 参数和 `trajectory_core` 配置。
2. 构造 ROS adapters：
   - `StateSubscriber`
   - `OdomSubscriber`
   - `RCSubscriber`
   - `SetpointPublisher`
   - `StatusPublisher`
   - `SetModeClient`
3. 构造 `application::BacktrackUseCase`。
4. 将 3 个订阅回调直接转发到 usecase：
   - `onOdomReceived()`
   - `onRCReceived()`
   - `onStateReceived()`
5. 20Hz 定时器调用 `BacktrackUseCase::tick(now)`。
6. `tick()` 返回 `TickOutput`，入口层再决定：
   - 是否发布 setpoint
   - 是否请求 `set_mode`
   - 发布当前状态和 RC 丢失相关输出

如果你要理解“系统为什么这样飞”，重点读 `BacktrackUseCase` 和 `BacktrackStateMachine`，而不是 ROS adapters。

## 文件地图

### 入口层

- [`src/backtrack_node.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/backtrack_node.cpp)
  组装对象、绑定回调、驱动 20Hz tick。改这里通常意味着你在改进程级 wiring 或主循环频率。

### Application 层

- [`include/px4_backtrack/application/backtrack_usecase.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/application/backtrack_usecase.hpp)
- [`src/application/backtrack_usecase.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/application/backtrack_usecase.cpp)

这里是最重要的编排层，职责包括：

- 持有所有核心 domain 对象
- 缓存当前 `armed/mode/position/yaw`
- 处理飞行周期复位和录制延时
- 检测触发条件
- 请求轨迹生成
- 检测偏离并进入/退出 rejoin
- 根据状态机输出决定 setpoint 与 mode request

如果需求是“改变返航流程”，先看这里。

### Domain 层

- [`include/px4_backtrack/domain/state_machine.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/domain/state_machine.hpp)
- [`src/domain/state_machine.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/state_machine.cpp)
  纯状态迁移与每态输出。它不直接读 ROS，也不直接调 `trajectory_core`。

- [`include/px4_backtrack/domain/recording_buffer.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/domain/recording_buffer.hpp)
- [`src/domain/recording_buffer.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/recording_buffer.cpp)
  用 `record_max_hz` 节流录制 `/Odometry_px4`，存相对时间戳。

- [`include/px4_backtrack/domain/trigger_evaluator.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/domain/trigger_evaluator.hpp)
- [`src/domain/trigger_evaluator.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/trigger_evaluator.cpp)
  处理 `CH7`、`CH18` 和 RC 消息丢失。

- [`include/px4_backtrack/domain/trajectory_session.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/domain/trajectory_session.hpp)
- [`src/domain/trajectory_session.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/trajectory_session.cpp)
  封装 `trajectory_core::TrajectoryGenerator`、轨迹对象、执行进度、暂停/恢复。

- [`include/px4_backtrack/domain/rejoin_policy.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/domain/rejoin_policy.hpp)
- [`src/domain/rejoin_policy.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/rejoin_policy.cpp)
  负责偏离判定、最近点搜索、预瞄目标与速度限幅。

### Adapters 层

- [`include/px4_backtrack/adapters/ros_subscribers.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/adapters/ros_subscribers.hpp)
- [`src/adapters/ros_subscribers.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/adapters/ros_subscribers.cpp)
  封装 3 个订阅器，默认话题分别是 `/mavros/state`、`/Odometry_px4`、`/mavros/rc/in`。

- [`include/px4_backtrack/adapters/ros_publishers.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/adapters/ros_publishers.hpp)
- [`src/adapters/ros_publishers.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/adapters/ros_publishers.cpp)
  封装 `/mavros/setpoint_raw/local`、`/px4_backtrack/state`、`/status_rc_loss`。

- [`include/px4_backtrack/adapters/ros_services.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/adapters/ros_services.hpp)
- [`src/adapters/ros_services.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/adapters/ros_services.cpp)
  封装 `/mavros/set_mode`，带 1s 节流。

### Infrastructure 层

- [`include/px4_backtrack/infrastructure/config_loader.hpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack/infrastructure/config_loader.hpp)
- [`src/infrastructure/config_loader.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/infrastructure/config_loader.cpp)

修改参数字段时，通常必须同时改：

- `UseCaseConfig` / 相关 domain config
- `ConfigLoader`
- [`config/backtrack_params.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/backtrack_params.yaml)
- `README.md`

## 行为语义上的关键点

### 1. 录制和轨迹生成不是一回事

- `RecordingBuffer` 保存录制点与相对时间戳。
- `TrajectorySession::generate()` 会把点序逆转，并把时间戳映射为 `t_new = t_last - t_rev`，从而在保持单调递增的同时保留原始采样间隔信息。
- `trajectory_core` 最终执行的轨迹时间轴仍由库内部速度剖面生成；录制时间戳主要用于时间质量检查与 warning，而不是直接决定返航执行时序。

### 2. `CH7` 的“按下事件”和“长按事件”共享一套内部状态

`TriggerEvaluator::checkCh7PressEvent()` 依赖 `ch7_state_` 已经被 `checkCh7LongPress()` 更新过。在当前实现里，`BacktrackUseCase::handleTriggers()` 先调用 `checkCh7LongPress()`，再决定是否调用 `checkCh7PressEvent()`。

如果你改触发顺序，可能把暂停逻辑改坏。

### 3. RC 丢失检测：边沿触发与电平查询分离

RC 丢失检测已拆分为两个方法：

- `checkRCLossEvent()` — 边沿语义，仅在 `rc_loss_triggered_` 从 `false` 变为 `true` 的那一刻返回一次 `true`，用于 `handleTriggers()` 触发返航
- `isRCLost()` — 电平语义，只要 `rc_loss_triggered_` 为 `true` 就持续返回 `true`，用于填充 `TickOutput.rc_loss` 并发布到 `/status_rc_loss`

两者共享底层状态：

- `onRCReceived()` 中调用 `updateLastRcMsgTime(ros::Time::now().toSec())`
- `tick(now)` 中调用 `updateNow(now)`
- 超时+防抖逻辑在 `checkRCLossEvent()` 中推进，`isRCLost()` 只读 `rc_loss_triggered_`

重要：`checkRCLossEvent()` 在 `handleTriggers()` 的最顶部、状态分支之前调用，确保 RC 超时的状态推进不被任何分支或 `checkMinPoints()` 挡住。

如果只改其中一条链路，RC loss 会失真。

### 4. pause/resume 的恢复不是直接回到 `ReturnActive`

暂停时：

- `state_machine_.triggerPause()`
- `trajectory_session_.freezeProgress()`

恢复时：

- `state_machine_.triggerReturn(Event::Ch7LongPress)`
- `trajectory_session_.resumeProgress(now)`

因此恢复路径会重新走 `ReturnPreOffboard`，而不是直接跳回 `ReturnActive`。

### 5. AUTO.LAND 是状态机里的全局高优先级分支

`BacktrackStateMachine::tick()` 一开始就检查 `mode == "AUTO.LAND"`。如果命中，会清理 setpoint 和 mode request，并把状态退回 `Recording` 或 `IdleDisarmed`。

如果你改降落逻辑，必须评估这条全局分支是否还成立。

### 6. 触发时点数不足会降级为原地降落

所有触发路径（CH7 长按、CH18 故障确认、RC 丢失）在 `Recording` 状态下会先检查 `checkMinPoints()`：

- 点数足够 → 正常进入返航流程
- 点数不足 → 调用 `state_machine_.triggerLanding()` 直接进入 `LandingTransition`

Paused 状态下 RC loss 或 CH7 恢复同理：点数不足时直接降落而非恢复返航。

`triggerLanding()` 进入的 `LandingTransition` 会判断当前模式：非 OFFBOARD 直接请求 `AUTO.LAND`，OFFBOARD 先走 `POSCTL` 过渡。

## 常见改动应该改哪里

| 目标 | 必看文件 |
| --- | --- |
| 改返航状态、加新状态 | `state_machine.hpp/.cpp`，然后回看 `backtrack_usecase.cpp` |
| 改触发阈值或触发语义 | `trigger_evaluator.hpp/.cpp`、`backtrack_params.yaml`、`config_loader.cpp` |
| 改录制条件或录制节流 | `recording_buffer.hpp/.cpp`、`backtrack_usecase.cpp` |
| 改轨迹生成方式 | `trajectory_session.hpp/.cpp`、`trajectory_config.yaml` |
| 改偏离检测或回归策略 | `rejoin_policy.hpp/.cpp`、`backtrack_usecase.cpp` |
| 改话题名或消息类型 | `ros_subscribers.hpp/.cpp`、`ros_publishers.hpp/.cpp`、`launch/backtrack.launch` |
| 改参数加载方式 | `config_loader.hpp/.cpp`、`backtrack_params.yaml` |
| 改启动频率 | `backtrack_params.yaml`、`src/backtrack_node.cpp` |

## 配置文件地图

- [`config/backtrack_params.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/backtrack_params.yaml)
  返航节点参数，按录制、触发、OFFBOARD、rejoin、降落分组
- [`config/trajectory_config.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/trajectory_config.yaml)
  直接传给 `trajectory_core::TrajectoryConfig::loadFromYaml()`
- [`launch/backtrack.launch`](/home/channing/Drone/Projects/px4_backtrack_optimised/launch/backtrack.launch)
  默认只启动 `backtrack_node` 并加载 `backtrack_params.yaml`

## 文档可信度排序

按“接近当前代码”的程度排序：

1. [`src/backtrack_node.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/backtrack_node.cpp) 和 `src/**`
2. [`include/px4_backtrack/**`](/home/channing/Drone/Projects/px4_backtrack_optimised/include/px4_backtrack)
3. [`config/backtrack_params.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/backtrack_params.yaml) 与 [`config/trajectory_config.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/trajectory_config.yaml)
4. [`doc/返航实现技术文档.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/返航实现技术文档.md)
5. [`doc/SYSTEM_OVERVIEW.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/SYSTEM_OVERVIEW.md)
6. [`doc/实现任务清单.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/实现任务清单.md)
7. [`doc/TECHNICAL_DOCUMENTATION.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/TECHNICAL_DOCUMENTATION.md) 与 [`doc/backtrack_node.py`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/backtrack_node.py)

## 当前容易踩坑的地方

- `checkRCLossEvent()` 必须在 `handleTriggers()` 的最顶部、状态分支之前调用。如果挪到状态分支内部，`isRCLost()` 在某些状态下会失真。
- `handleReturnTrigger()` 在点数不足时调用 `triggerLanding()`，这会跳过 `ReturnPreOffboard` 直接进 `LandingTransition`。如果你在 `LandingTransition` 之前加了新逻辑，要评估这条直达路径是否会绕过它。
- [`doc/SYSTEM_OVERVIEW.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/SYSTEM_OVERVIEW.md) 是高层概览，不是行为规范；尤其涉及 `trajectory_core` 输入时间轴时，必须以 [`src/domain/trajectory_session.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/domain/trajectory_session.cpp) 为准。
- 仓库已有 `tests/unit/`，但目前主要覆盖领域层和轻集成逻辑，还没有 ROS topic 级 mock/rostest。涉及 adapters、launch、参数装配时，仍然要做一次编译和联调。

## 测试地图

- [`tests/unit/test_recording_buffer.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/tests/unit/test_recording_buffer.cpp)
  覆盖录制节流、时间戳单调性和 `reset()`
- [`tests/unit/test_trigger_evaluator.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/tests/unit/test_trigger_evaluator.cpp)
  覆盖 `CH7` / `CH18` / RC loss 触发语义
- [`tests/unit/test_state_machine.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/tests/unit/test_state_machine.cpp)
  覆盖主状态迁移和 `AUTO.LAND` 高优先级
- [`tests/unit/test_rejoin_policy.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/tests/unit/test_rejoin_policy.cpp)
  覆盖最近点搜索、预瞄和速度限幅
- [`tests/unit/test_trajectory_session.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/tests/unit/test_trajectory_session.cpp)
  覆盖轨迹生成、逆序语义和暂停恢复进度

运行方式：

- 在 catkin 工作空间根目录先执行 `catkin_make --pkg trajectory_core px4_backtrack_optimised`
- 再执行 `catkin_make run_tests_px4_backtrack_optimised`
- 最后执行 `catkin_test_results`

## 给后续 LLM 的工作建议

- 先从 `src/backtrack_node.cpp -> src/application/backtrack_usecase.cpp -> src/domain/state_machine.cpp` 读一遍，再决定改动面。
- 需要改参数时，不要只改 YAML，`ConfigLoader` 和 config struct 往往也要同步改。
- 需要改状态名、话题名、接口语义时，记得同步更新 `README.md` 和本文件。
- 遇到“文档和代码不一致”时，优先相信代码，并把不一致显式写出来，不要擅自脑补。
