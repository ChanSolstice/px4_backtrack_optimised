# Changelog

## 2026-10-22 — 返航重构 R5：退出后沿截断的轨迹返航

飞手退出返航后，录制截断到已经返航到的位置；之后再失控，新任务先沿退出后的飞行路径回到退出位置，再沿原路回到录制起点，不再飞回第一次返航的触发点（方案第 7.4 节、R-77、R-78）。R6–R8 还没有实施，本分支尚未实飞。

### 代码

- **feat(domain):** 录制缓存支持截断、续录时间接续（第一个新点接在末点之后 1/`record_max_hz`）和游标搜索（从游标往前 2 m 窗口内找最近的录制点，只减不增）
- **refactor(domain):** 状态机的截断命令改为"是否截断"，任何阶段退出都截断到执行层的游标（R-78）：悬停、准备阶段退出时游标仍在末点，等于不截断；半路进入交接后退出时保留游标之前的原路，不再截断到起点
- **feat(application):** UseCase 维护游标（只在跟随进度推进时移动，轨迹用路径投影点、航点降级用飞机位置），退出时截断并记录事件 `mission/trail_cut`，录制恢复时接续时间；规划提交事件增加录制时长
- **config:** 新增必填参数 `cursor_window_m`（2.0 m）

### 测试

- 返航单测 195 个：录制缓存新增 5 个；UseCase 新增 S6（退出时截断到游标并继续录制）、S11（退出后失控沿截断后的轨迹 + 手动段返航，不回原触发点，续录时间接续）、悬停中退出不截断
- 错误注入 52 项全部发现（新增 6 项，新增组件 recording_buffer）
- 闭环危害场景 27 项全部通过

### 文档

- 方案补充审核记录 R-77、R-78，头部更新实施进度
- 系统总览、README、LLM_MAP、前端对接文档、单元测试文档、闭环仿真与 SITL 测试文档改为 R5 的实现

## 2026-10-22 — 返航重构 R4：后台规划

规划移到后台线程（方案第 7.5 节、R-76），控制周期不再等待求解。R5–R8 还没有实施，本分支尚未实飞。

### 代码

- **feat(application):** 新增 `PlanningWorker`（移植自 fix/return-engineering-contracts）：单工作线程，最多一个待处理请求，用代次号丢弃过期结果，取消时只丢弃结果，析构时等正在运行的求解结束；规划成功后在规划线程导出 CSV 和 RViz
- **feat(application):** UseCase 在任务开始时提交录制快照，之后每拍取结果；10 s 未完成告警 `PLANNING_SLOW`，30 s 未完成则取消规划、在控制线程构建航点降级（`PLANNING_TIMEOUT_FALLBACK`，持续到任务结束）；任务结束时取消未完成的规划；规划事件增加提交、取消、超时
- **feat(domain):** 规划状态去掉 `Timeout`；条件增加 `planning_slow`、`planning_timed_out`，告警由状态机计算
- **config:** 新增必填参数 `planning_warn_s`（10 s）、`planning_timeout_s`（30 s）

### 测试

- 返航单测 187 个：新增 `PlanningWorker` 6 个；UseCase 新增 S15（悬停期间规划：每拍照常发布，结果在悬停结束后处理）、S16（规划超时改为航点降级）和规划中退出，共 3 个；状态机补"悬停期间规划就绪不提前结束悬停"
- 闭环危害场景使用真实的规划线程，H07 增加"规划期间 setpoint 不中断"：27 项检查全部通过。H07 失控后进入 OFFBOARD 从 6.1 s 缩短到 1.4 s，最长 tick 从 4703 ms 降到 1 ms
- 错误注入 46 项全部发现（新增 4 项）；其中"规划就绪提前结束悬停"首轮未被发现，补了状态机用例
- amd64 和 arm64（QEMU）全量单测均通过

### 文档

- 方案补充审核记录 R-76，头部更新实施进度
- 系统总览、README、LLM_MAP、单元测试文档、闭环仿真与 SITL 测试文档改为 R4 的实现

## 2026-10-21 — 返航重构 R3：新状态机接入运行

节点改为运行新状态机（方案第 13 节 R3），dev 的暂停/恢复语义已删除。R4–R8 还没有实施，本分支尚未实飞。

### 代码

- **feat(application):** `BacktrackUseCase` 接入新状态机，每拍按"条件层 → `Conditions` 快照 → `ReturnStateMachine::step()` → 执行命令、推进路径、计算 setpoint"的顺序处理（R-71）；规划暂时仍同步执行
- **feat(application):** 新增 `ModeCommander`：模式意图改变时立即发送，未生效时每 1 s 重发，3 s 未生效告警 `MODE_REQUEST_FAILED`（R-70）；`set_mode` 改为在单独线程中异步调用
- **feat(infrastructure):** 新增调试日志 `DebugLog`：每次启动一个运行目录，事件写入 `events.jsonl`，第一行为生效参数和提交号；写失败时告警 `DEBUG_LOG_FAILED`（R-72）
- **feat(domain):** 链路正常时，预发结束后等到下一条飞控状态才申请 OFFBOARD，避免覆盖飞手刚做的切换（R-75，闭环场景 H14 发现）
- **feat(domain):** `TrajectorySession::probe()` 按当前位置查询横向偏差、不推进进度，重新接入时检查的是当前偏差（R-74）
- **feat(node):** 订阅 `/mavros/extended_state`；只采信 `connected` 为真的 `/mavros/state`；忽略含非有限值的里程计（R-73）；自身断流只在连续发布期间计算（R-69）
- **refactor(domain):** 删除 dev 的 `BacktrackStateMachine`、`TriggerEvaluator` 和航点降级的 `restart()`
- **config:** 参数全部必填（含 `publish_hz`）；新增链路恢复与退化、输入新鲜度、在空中判定、任务与控制权、模式请求、调试日志等参数
- **fix(scripts):** arm64 单测镜像改用单架构的 `arm64v8/ros`，此前构建出的 arm64 镜像实际是 amd64

### 测试

- 返航单测 178 个：新增 `ModeCommander` 9 个、`DebugLog` 5 个，状态机（R-75）和轨迹会话（R-74）各 1 个；UseCase 用例改为用简化假飞控覆盖方案第 11 节的场景（37 个，含迁移的 dev 用例）；删除旧状态机和触发器的 18 个
- 闭环危害场景改为驱动新实现：26 项检查全部通过（dev 为 17 项失败）
- 错误注入 42 项全部发现（新增 R-75 和 UseCase 接线 8 项）
- amd64 和 arm64（QEMU）全量单测均通过

### 文档

- 方案补充审核记录 R-69–R-75，头部注明实施进度
- 系统总览、README、LLM_MAP、前端对接文档改为 R3 的实现；返航实现技术文档并入系统总览后删除
- 单元测试文档、闭环仿真与 SITL 测试文档补充 R3 的用例说明和执行记录
- 修正指向已删除或已改写源码的链接

## 2026-10-21 — 返航重构 R1、R2，SITL 确认与文档整理

返航重构按方案第 13 节实施到 R2。新组件都有单测，但还没有接入流程，节点的运行行为仍是 dev。

### 代码

- **feat(domain):** R1 条件层：`LinkMonitor`（CH18 漏桶、RC 超时、本模块卡顿保护、恢复与退化窗口）、`PilotInput`（CH7 逐帧门控与按键指令）、`BatteryMonitor`（读数有效性、确认与锁存）、`AirborneMonitor`（在空中判定）
- **feat(domain):** R2 条件快照 `Conditions` 与 `ReturnStateMachine`：三个并行区域、策略矩阵、每拍输出与事件；`Vec3` 移到 `types.hpp`
- **refactor(domain):** 按避免过度防御精简条件层与状态机（方案 R-68）

### 测试

- 新增单测 94 个（条件层 46、状态机场景 46、安全约束穷举 2），返航共 156 个
- 新增错误注入检查 `scripts/analysis/run_mutation_check.sh`：34 项全部发现
- 新增 PX4 v1.15.4 SITL 飞控行为检查 `scripts/sitl/`：N1（断流回退后自动恢复）、N2（回中或原地保持不判定落地）已确认

### 文档

- 方案补充审核记录 R-55–R-68；系统性分析记录 N1、N2 的 SITL 结果
- 文档整理：删除过时和重复的文档（Python 基线、旧问题清单、旧任务清单、已被取代的低电量方案、trajectory_core 接口文档副本），修正失效链接；补全单元测试文档，新增闭环仿真与 SITL 测试文档；标注描述重构前实现的文档的适用范围；2026-09-07 审核补充处理状态

## 2026-10-20 — 返航触发与状态机重构：方案、系统性分析与测试工具

本次只新增规划文档和测试工具，没有修改返航模块代码；实施按方案第 13 节 R1–R8 分阶段进行。

- 新增[返航触发与状态机重构方案](doc/design/返航触发与状态机重构方案.md)：组合触发问题审核、改动固件的前提与推论、单一返航任务、暂停改退出、失控重新申请控制权、悬停优先的超时升级、后台规划、三个并行区域的状态机、调试日志；决策 D1–D29 已全部确认。
- 新增[返航程序系统性分析](doc/reviews/返航程序系统性分析_2026-10-20.md)：STPA 危害分析、PX4 v1.15.4 源码核对、dev 闭环仿真（26 项检查 17 项失败）、时序预算与证据计划。
- 新增测试工具：
  - `scripts/docker/`：ROS Noetic 容器编译与单测；
  - `scripts/bench/`：轨迹生成耗时基准；
  - `scripts/analysis/`：假飞控驱动真实 UseCase 的闭环危害场景 H01–H14。

## 2026-09-07 — 文档分类与双仓库架构审核

- 按参考、设计、测试、审核、历史资料增加一级文档目录，新增导航并更新链接。
- 记录 `b0222e6` / `trajectory_core@fb959e4` 的架构与运行边界审核，区分离线复现、代码确认和待验证风险，列出修复顺序。
- 本次仅调整文档与历史文件位置，没有修改算法或重跑 SITL。

## 2026-08-29 — 弧长驱动返航与统一 Rejoin 跟随

将返航执行基准从 wall time 改为实际位置的局部路径投影，消除沿路径速度落后被误判为横向偏离而反复 Rejoin 的根因。

### 改造

- **refactor(trajectory):** `TrajectorySession` 仅维护单调 `projected_s`、`target_s` 和横向误差，以局部投影和路径速度预瞄生成 setpoint
- **fix(completion):** 完成判定同时约束剩余弧长、终点距离和实际水平速度，不再依赖轨迹 CPU 秒表
- **refactor(rejoin):** `ReturnRejoin` 复用同一弧长跟随器，扩大局部搜索、缩短预瞄并平滑限速；删除全局最近时间搜索和冻结点回拉逻辑
- **fix(yaw):** `ReturnPreOffboard` 与暂停恢复先保持位置并对准路径切向 yaw，15° 内才允许开始水平返航
- **fix(yaw):** 用至少 `0.20 m` 的稳定弦长估计录制路径切向，避免端点厘米级悬停漂移导致返航 yaw 翻转
- **fix(recording):** 保留真实末点时回退删除过近的中间点，满足末端最小间距且不截断低速路径
- **fix(odom):** 按 `nav_msgs/Odometry` 语义将 child/body twist 旋转到父 ENU，再用于 Rejoin 与终点速度判定
- **fix(continuity):** Rejoin 退出同时检查横向误差、实际速度和已发布目标速度，限制单周期参考速度向量变化不超过 `0.3 m/s`
- **fix(rejoin):** 将 `0.4 m` Rejoin 安全走廊与固定 `0.2 m` 终点精度解耦，避免正常三维跟踪误差使 Rejoin 卡到终点
- **fix(state):** AUTO.LAND 接管后进入专用 `AutoLandYield`，不再对外误报 `Recording`

### 测试

- 覆盖慢速沿线跟随、单调进度、自交路径局部投影、空间终点判定和投影失败保持
- 覆盖起始/恢复 yaw 对准、Rejoin 路径 yaw、速度未收敛不退出和退出前后目标连续性
- 当前两个仓库源码在干净 catkin 工作空间完成实际 testcase `162 / 162`；`catkin_test_results` 原始 `324` 为 XML 重复累计，`0` 失败
- Docker SITL 完成 3 m/8 m 直线、L 型、S 型、普通自交、受控 Rejoin 和加速度 A/B；生产配置名义场景 Rejoin 为 `0`
- 直线起始掉头将 PX4 `MC_YAWRATE_MAX` 临时设为 `45 deg/s` 后，实测 yaw rate 峰值 `44.90 deg/s`；该试验没有修改仓库配置
- 闭合且起终点重合的 figure-8 暴露 `trajectory_core` 端点 yaw/速度方向冲突，当前列为禁止实飞的跨仓库残余风险

### 已知边界

- `ReturnPreOffboard` 会一步设置返航起始切向 yaw，并以 `15°` 门限决定何时开始平移；应用层没有起始 yaw 斜坡或速率限制器
- `trajectory_config.yaml` 的 `yaw_rate_max` 只约束运动轨迹 yaw，原地对准速率由 PX4 参数和飞控内环决定

## 2026-04-12 — AUTO.LAND 让步锁存

修复 TC-11 `AUTO.LAND` 优先级让步测试中，外部降落接管后节点仍可能重新触发返航并请求 `OFFBOARD` 的问题。

### 修复

- **fix(usecase):** 新增 `auto_land_yield_active_` 锁存，armed 状态下一旦检测到 `AUTO.LAND`，直到 DISARM 前跳过录制延时、触发检测、轨迹生成、fallback 推进和 rejoin 处理
- **fix(usecase):** `AUTO.LAND` 让步检查早于 `checkRCLossEvent()`，避免降落期间消费 RC loss 边沿
- **fix(state_machine):** `AUTO.LAND` 分支始终清理 setpoint 和 mode request；armed 时退回 `Recording`，disarmed 时进入 `IdleDisarmed`，避免仍 armed 时卡在 idle

### 测试与文档

- **test(usecase):** 新增 `AutoLandYieldSuppressesRetriggerUntilDisarm`，覆盖 `AUTO.LAND` 后模式短暂跳出且 Ch7 再次动作时不再请求 `OFFBOARD`
- **test(state_machine):** 补充 `AUTO.LAND` 后 disarm 进入 `IdleDisarmed` 的状态机用例
- **docs(llm/changelog):** 更新 LLM_MAP 和变更记录，明确 `AUTO.LAND` 让步锁存与 RC loss 触发顺序约束

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `include/px4_backtrack/application/backtrack_usecase.hpp`
- `src/domain/state_machine.cpp`
- `tests/unit/test_backtrack_usecase.cpp`
- `tests/unit/test_state_machine.cpp`
- `LLM_MAP.md`
- `CHANGELOG.md`

## 2026-04-11 — ReturnRejoin 回拉冻结点

修复 `ReturnRejoin` 中“退出判据看冻结进度点，但 setpoint 追最近点预瞄目标”的目标不一致问题，降低回归阶段原地抽动和无法退出的风险。

### 修复

- **fix(rejoin):** `ReturnRejoin` setpoint 改为直接回拉到冻结进度点，不再在全轨迹上搜索最近点并叠加 `lookahead`
- **fix(rejoin):** 回归速度改为当前位置指向冻结点的比例收敛速度，`speed = min(rejoin_v_max_m_s, 1.0 * distance)`，靠近目标时线性减速
- **fix(rejoin):** 回归期间 yaw 和 yaw_rate 直接复用冻结点轨迹姿态，避免回归时额外原地转向扰动

### 测试与文档

- **test(rejoin):** `test_rejoin_policy.cpp` 新增冻结点回拉测试，覆盖速度限幅、近距离比例减速、到点速度归零和 yaw 复用
- **docs(bug):** 新增 bug report，记录 `ReturnRejoin` 最近点预瞄目标与冻结退出点不一致导致卡住的问题
- **docs(readme/llm):** README 和 LLM_MAP 同步更新当前 `ReturnRejoin` 控制目标语义

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `src/domain/rejoin_policy.cpp`
- `include/px4_backtrack/domain/rejoin_policy.hpp`
- `tests/unit/test_rejoin_policy.cpp`
- `README.md`
- `LLM_MAP.md`
- `doc/archive/BUG_REPORT_ReturnRejoin最近点预瞄目标与冻结退出点不一致.md`

## 2026-04-11 — 修复低速飞行段抽稀被截断

修复 TC-18 直线分三段返航验证中，`min_point_dist_m=0.08` 时低速连续飞行段被生成前抽稀误删，导致返航轨迹被截断并在第二段附近卡入 `ReturnRejoin` 的问题。

### 修复

- **fix(trajectory):** `thinRecordedPointsForGeneration()` 在中间点距离小于阈值时直接忽略当前点，不再用当前点覆盖上一个有效点，避免低速连续飞行时“距离锚点”被不断前移
- **fix(trajectory):** 保持首点和末点保留语义不变，确保返航轨迹仍覆盖完整录制路径的起终点

### 测试与文档

- **test(trajectory):** `test_trajectory_session.cpp` 新增慢速直线用例，覆盖每帧位移小于 `min_point_dist_m` 但累计距离持续增长的场景
- **docs(bug):** 新增 TC-18 bug report，记录 `455 -> 24` 抽稀异常、路径长度 `11.56m -> 4.42m` 截断和修复后的 `455 -> 104` 重放结果
- **docs(readme/llm):** README 和 LLM_MAP 同步更新生成前抽稀语义：低于阈值的中间点被忽略，不移动上一个有效点

### 涉及文件

- `src/domain/trajectory_session.cpp`
- `tests/unit/test_trajectory_session.cpp`
- `README.md`
- `LLM_MAP.md`
- `doc/archive/BUG_REPORT_低速连续飞行抽稀锚点覆盖导致返航路径截断.md`

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
- `doc/reference/返航实现技术文档.md`

## 2026-04-10 — 点过近退化轨迹触发原地降落

修复“起飞后原地悬停，录制点数足够但点过近/起终点几何退化，触发返航后轨迹生成失败却未降级降落”的问题。

### 修复

- **fix(usecase):** `BacktrackUseCase::handleTrajectoryGeneration()` 仅在 `TrajectorySession::generate()` 因“点过近/起终点几何退化”失败时触发原地降落，不再把所有生成失败都一刀切降级
- **refactor(trajectory):** `TrajectorySession` 新增最近一次生成失败分类，供 usecase 精确区分“点过近退化”与其它生成失败

### 测试与文档

- **test(usecase):** 新增 `test_backtrack_usecase.cpp`，覆盖“原地悬停录到足够点数、返航生成失败后应进入 `LandingTransition` 并请求 `AUTO.LAND`”场景
- **docs(readme/llm):** README 和 LLM_MAP 同步补充“仅点过近/起终点几何退化失败时降级原地降落”的当前行为语义
- **docs(test):** `doc/testing/返航功能测试文档.md` 将 `BUG-01` 标记为已修复，并保留 `TC-19` 作为对应回归用例

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
- `doc/testing/返航功能测试文档.md`

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
- `doc/archive/BUG_REPORT_停顿点高密度录制导致返航抽动.md`
- `doc/archive/BUG_REPORT_rejoin单帧越阈值导致状态反复切换.md`

## 2026-04-08 — 参数与测试文档同步调整

结合实飞联调结果回调返航与轨迹参数，并同步刷新返航功能测试文档中的基线参数表与测试项。

### 修改

- **update(config):** `record_min_points` 从 `100` 调整为 `30`，降低返航触发所需录制点数门槛
- **update(config):** `record_max_hz` 从 `10.0` 调整为 `15.0`，提高录制频率上限
- **update(config):** `rejoin_success_dist_m` 从 `0.1` 调整为 `0.2`，放宽回归成功阈值
- **update(config):** `trajectory_config.yaml` 中 `v_max` 从 `2.0` 调整为 `1.0`，`vz_max` 从 `0.8` 调整为 `0.6`

### 文档

- **docs(test):** `doc/testing/返航功能测试文档.md` 参数基线改为与当前 YAML 保持一致，并补充 `trajectory_core` 参数表

### 涉及文件

- `config/backtrack_params.yaml`
- `config/trajectory_config.yaml`
- `doc/testing/返航功能测试文档.md`

## 2026-04-08 — 修复 ReturnRejoin 期间进度未冻结

修复进入 `ReturnRejoin` 后轨迹进度仍按 wall time 推进，导致回归目标漂移到轨迹末端、无人机无法自主回归的问题。

### 修复

- **fix(rejoin):** 在 `BacktrackUseCase::handleRejoin()` 中，进入 rejoin 时调用 `trajectory_session_.freezeProgress()`，退出 rejoin 时调用 `trajectory_session_.resumeProgress(now)`，使 rejoin 与 pause 语义保持对称

### 测试与文档

- **test(state_machine):** 补充状态机相关测试覆盖，验证进入 rejoin 后不会再残留固定点 setpoint
- **docs(bug):** 新增两份 bug report，分别记录 `ReturnRejoin` 期间进度未冻结问题和 `current_setpoint_` 残留导致返航悬停问题
- **docs(test):** `doc/testing/返航功能测试文档.md` 补充实飞问题记录与用例结果

### 涉及文件

- `src/application/backtrack_usecase.cpp`
- `tests/unit/test_state_machine.cpp`
- `doc/archive/BUG_REPORT_rejoin进度未冻结导致轨迹跟踪失败.md`
- `doc/archive/BUG_REPORT_setpoint残留导致返航悬停.md`
- `doc/testing/返航功能测试文档.md`

## 2026-04-01 — 修复固定点 setpoint 残留导致返航悬停

修复 `ReturnPreOffboard` 写入的固定点 setpoint 在进入 `ReturnActive` / `ReturnRejoin` 后未清除，导致 `computeSetpoint()` 一直被短路到悬停点的问题。

### 修复

- **fix(state_machine):** `tickReturnActive()` 和 `tickReturnRejoin()` 显式将 `current_setpoint_` 置为 `std::nullopt`，让 usecase 可以正确从轨迹评估和 rejoin 策略计算 setpoint

### 涉及文件

- `src/domain/state_machine.cpp`

## 2026-03-30 — 补充测试文档

补齐仓库的单元测试说明文档和返航功能测试文档，便于后续联调、验收与缺陷回归追踪。

### 新增

- **docs(test):** 新增 `doc/testing/单元测试文档.md`，说明当前 `gtest` 覆盖范围、运行方式、结果判读与扩展建议
- **docs(test):** 新增 `doc/testing/返航功能测试文档.md`，定义返航主流程、触发、rejoin、降落和 `AUTO.LAND` 让步等测试项

### 文档

- **docs(readme):** `README.md` 新增测试文档入口与目录索引

### 涉及文件

- `doc/testing/单元测试文档.md`
- `doc/testing/返航功能测试文档.md`
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

- **docs(system):** `doc/reference/SYSTEM_OVERVIEW.md` 补充状态机主状态迁移图、全局覆盖规则与 `LandingTransition` 内部阶段图
- **docs(llm):** `LLM_MAP.md` 更新测试现状说明，明确仓库已具备基础领域层单元测试
- **docs(readme):** `README.md` 补充测试目录、测试命令与 `SYSTEM_OVERVIEW` 文档入口

### 涉及文件

- `CMakeLists.txt`
- `doc/reference/SYSTEM_OVERVIEW.md`
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

- **docs(api):** `doc/reference/API接口文档.md` 按新版 `trajectory_core` 能力刷新，补充 `warning_flags`、`ResultWarning`、`curvatureFromArcLength()`、逐点 `dt` yaw limiter、YAML 覆盖范围等说明
- **docs(system):** 新增 `doc/reference/SYSTEM_OVERVIEW.md`，提供基于当前 C++ 实现整理的系统级概览
- **docs(tech):** `doc/reference/返航实现技术文档.md` 中的 waypoint 时间戳构造规则更新为镜像映射语义
- **docs(llm):** `LLM_MAP.md` 更新 `trajectory_core` 时间戳接入真相，并补充对高层概览文档使用边界的提醒

### 涉及文件

- `include/px4_backtrack/domain/trajectory_session.hpp`
- `src/domain/trajectory_session.cpp`
- `src/application/backtrack_usecase.cpp`
- `doc/reference/API接口文档.md`
- `doc/reference/SYSTEM_OVERVIEW.md`
- `doc/reference/返航实现技术文档.md`
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
