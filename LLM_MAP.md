# LLM_MAP

文档入口见 [doc/README.md](doc/README.md)。本文给后续阅读或修改本仓库的 LLM/自动化代理看：先建立正确的代码地图，再动手改动。当前实现的完整说明见 [系统总览](doc/reference/SYSTEM_OVERVIEW.md)；要达到的行为和全部规则以[返航触发与状态机重构方案](doc/design/返航触发与状态机重构方案.md)（下称"方案"）为准。

## 先记住这些

1. ROS1 `catkin` 工程（Noetic），不是 ROS2。本机没有 ROS，编译和测试都在 Docker 中。
2. 入口 [`src/backtrack_node.cpp`](src/backtrack_node.cpp)：单线程 `ros::spin`，20 Hz 定时器驱动控制周期。另有两个后台线程：异步 `set_mode`（[`ros_services.cpp`](src/adapters/ros_services.cpp)）和调试日志写盘（[`debug_log.cpp`](src/infrastructure/debug_log.cpp)）。
3. 编排在 [`src/application/backtrack_usecase.cpp`](src/application/backtrack_usecase.cpp)：组装 `Conditions` → `ReturnStateMachine::step()` → 执行命令、计算 setpoint。UseCase 不访问 ROS，所有输入的接收时间由节点传入，所以可以在单测里完全控制时间。
4. 所有状态转移都在 [`return_state_machine.cpp`](src/domain/return_state_machine.cpp) 的 `step()` 中完成，规则编号对应方案第 6.3 节。不要在 UseCase 里改状态或加分支。
5. 条件层（`link_monitor`、`pilot_input`、`battery_monitor`、`airborne_monitor`）只做检测，输出电平和带编号的按键指令，是否响应由状态机决定。
6. 设计变化先改方案文档，并在第 15 节记一条审核记录（R-xx），再改代码；用户已确认的决策是 D1–D33。
7. 本分支实施到 R7（D30 安全距离、D33 找点接上、第 8.1 节超时升级、JSON 状态话题和调试日志完善）。剩下 R8 文档与部署，见[系统总览](doc/reference/SYSTEM_OVERVIEW.md)第 9 节。
8. 参数全部在 [`config/backtrack_params.yaml`](config/backtrack_params.yaml) 中必填，代码中没有默认值；新增参数要同时改 config 结构体、[`config_loader.cpp`](src/infrastructure/config_loader.cpp) 和 YAML。`trajectory_core` 的配置在启动时校验，非法则节点退出，运行时不处理配置非法。
9. 用户要求整体简洁、避免过度防御：不加初值已保证的标志、不重复复位、不为小概率场景堆保护；不了解下方环境时，悬停优先于降落。

## 每个控制周期

见[系统总览](doc/reference/SYSTEM_OVERVIEW.md)第 3.1 节。要点：

- 先把异步模式请求的结果送回 `ModeCommander`，再让条件层推进到当前时间；
- 规划中时先取规划线程的结果，超时则改为航点降级；
- 用执行层"最近一次"的结果组装条件、调用 `step()`，之后才推进跟随进度、计算 setpoint（执行事实晚一拍，R-71）；
- 模式意图交给 `ModeCommander`，由它决定本拍是否发出请求；
- 事件（区域转换、条件边沿、任务、规划、模式请求、断流）随 `TickOutput` 返回，节点写到 rosout 和调试日志。

## 文件地图

### 入口与应用层

- [`src/backtrack_node.cpp`](src/backtrack_node.cpp)：加载参数、创建适配器和 UseCase、驱动控制周期、发布状态话题、写事件和逐拍记录。
- [`backtrack_usecase`](src/application/backtrack_usecase.cpp)：条件组装、命令执行、setpoint 计算、自身断流判定（R-69）、偏离与回归的持续判定。
- [`mode_commander`](src/application/mode_commander.cpp)：模式请求的发送、重发、生效观察和失败告警（方案第 7.7 节，R-70）。
- [`planning_worker`](src/application/planning_worker.cpp)：后台规划线程、失败分类与航点降级、规划 CSV（输入每次、轨迹在就绪时）和 RViz 导出（方案第 7.5 节，R-76）。单测用不启动线程的方式，由 `runPending()` 决定结果到达的时刻。
- [`tick_report`](src/application/tick_report.cpp)：状态话题 `/px4_backtrack/status` 的 JSON；逐拍记录 `trace.csv` 的表头、行和解析（`parseTraceRow` 用于回放到 `step()`）。条件快照的列只有一份字段表 `forEachConditionField`，`Conditions` 加字段时要加在这里（方案第 9 节，R-88）。

### 领域层

条件层（方案第 5 节）：

- [`link_monitor`](src/domain/link_monitor.cpp)：CH18 漏桶、RC 超时（含本模块卡顿保护）、恢复与退化。
- [`pilot_input`](src/domain/pilot_input.cpp)：CH7 逐帧门控、`press_id`、长按与短按指令。
- [`battery_monitor`](src/domain/battery_monitor.cpp)：读数有效性、低电量确认与锁存。
- [`airborne_monitor`](src/domain/airborne_monitor.cpp)：在空中判定（落地状态 + 相对解锁高度）。

决策：

- [`return_conditions.hpp`](include/px4_backtrack/domain/return_conditions.hpp)：每拍的条件快照。
- [`return_state_machine`](src/domain/return_state_machine.cpp)：三个并行区域（任务阶段、控制权、跟随方式），输出 setpoint 策略、模式意图、命令、告警和事件。

执行层：

- [`recording_buffer`](src/domain/recording_buffer.cpp)：录制节流与相对时间戳；退出时的截断、续录时间接续和游标搜索（方案第 7.4 节）；断点扫描和找点接上的查找（第 7.8 节）。
- [`trajectory_session`](src/domain/trajectory_session.cpp)：抽稀、逆序、调用 `trajectory_core`、局部投影、单调弧长进度、预瞄、完成判定；`probe()` 只查询当前偏差不推进进度（R-74）。
- [`waypoint_fallback_session`](src/domain/waypoint_fallback_session.cpp)：轨迹生成可恢复地失败时的航点降级。
- [`rejoin_policy`](src/domain/rejoin_policy.cpp)：偏离阈值、回归限速、退出时的速度收敛。
- [`path_yaw`](src/domain/path_yaw.cpp)：按稳定弦长估计返航切向 yaw。
- [`types.hpp`](include/px4_backtrack/domain/types.hpp)：`Vec3`、`SetpointTarget`。

### 适配层与基础设施

- [`ros_subscribers`](src/adapters/ros_subscribers.cpp)：`/mavros/state`（含 `connected`）、`/mavros/extended_state`、`/Odometry_px4`（速度旋转到父 ENU）、`/mavros/rc/in`、`/mavros/battery`。
- [`ros_publishers`](src/adapters/ros_publishers.cpp)：`/mavros/setpoint_raw/local`、`/px4_backtrack/state`、`/status_rc_loss`、`/px4_backtrack/low_battery`、`/px4_backtrack/status`。
- [`ros_services`](src/adapters/ros_services.cpp)：异步 `set_mode`。
- [`debug_publishers`](src/adapters/debug_publishers.cpp)、[`debug_dumper`](src/infrastructure/debug_dumper.cpp)：规划结果的可视化；规划 CSV 写到调试日志的运行目录（`plan_<n>_input.csv`、`plan_<n>_trajectory.csv`）。
- [`config_loader`](src/infrastructure/config_loader.cpp)：参数必填，启动时校验 `trajectory_core` 配置。
- [`debug_log`](src/infrastructure/debug_log.cpp)：调试日志（运行目录及保留个数、`events.jsonl`、`trace.csv`、后台写线程、有界队列）。

### Scripts

- [`scripts/docker/run_tests.sh`](scripts/docker/run_tests.sh)：在全新的 ROS Noetic 容器中编译并运行全部单测。
- [`scripts/analysis/`](scripts/analysis/)：闭环危害场景（假飞控驱动真实 UseCase，H01–H16）和错误注入检查，复用上面的容器。
- [`scripts/bench/`](scripts/bench/)：轨迹生成耗时基准。
- [`scripts/sitl/`](scripts/sitl/)：PX4 v1.15.4 SITL（SIH）中确认方案依赖的飞控行为（N1、N2），以及运行本节点回放方案第 11 节的场景（`run_sitl_checks.sh scenarios`）。
- [`scripts/analysis/show_debug_log.py`](scripts/analysis/show_debug_log.py)：按时间线查看 `events.jsonl`。
- [`scripts/plot_debug_csv.py`](scripts/plot_debug_csv.py)：规划 CSV 绘图（`--dir <运行目录>`）。

## 行为语义上的关键点

### 1. 录制与规划不是一回事

- 只在 `Recording` 和 `Exited` 中录制，任务期间不录制；时间戳为相对首点的里程计消息时间。
- 退出时录制截断到游标（UseCase 维护，只在跟随进度推进时向起点移动），所以悬停、准备阶段退出等于不截断；退出后续录的时间戳接在末点之后。不要改回按阶段决定截断方式：半路进入交接后截断到起点，会让下一次任务从退出位置直线飞向起点（R-78）。
- 规划时先空间抽稀（`max(min_point_dist_m, ds_m * 0.5)`），距离不足的中间点直接忽略，不覆盖上一个有效点。不要改回 `reduced.back() = point`，否则低速连续飞行段会被整段吞掉。
- 逆序后时间戳映射为 `t_new = t_last − t_rev`，只用于 `trajectory_core` 的时间质量诊断；执行由实际位置的弧长投影推进，不使用录制时间或 wall time。

### 2. CH7：长按返航、短按退出

- 长按（≥ 2 s）只在 `Recording` 中、链路未丢失时启动任务；任务中只记录；退出后只告警。
- 短按（0.3～2 s，松开时生效）只在任务中、按下全程链路正常、按下晚于任务开始时退出；链路异常时告警 `EXIT_REJECTED_LINK`。
- 失控帧或 RC 超时期间 CH7 不可信；指令按 `press_id` 消费，不会因落在两个 tick 之间而丢失。

### 3. 链路丢失与低电量是电平

条件都是电平，没有"被消费的边沿"：录制延时内已经成立的失控或低电量，进入 `Recording` 后照常启动任务（dev 的 A2、AR-07 问题由此解决）。同一拍多个条件成立时，只要有失控就先悬停。

### 4. 控制权：退出、回退与重新申请

- 链路正常时被切出 OFFBOARD，或短按，进入 `Exited`（不再有 `Paused` 和恢复）。
- 本模块自身断流引起的飞控回退不算接管：继续发布，等飞控自动回到 OFFBOARD（D22，N1 已在 SITL 确认）。
- 链路丢失或退化时被切出：重新申请控制权；跟随阶段回到 OFFBOARD 且对准、偏差 ≤ 0.7 m 才重新接入。
- OFFBOARD 确认前、链路正常时飞手切到其他模式：进入 `Exited`。为了先看到飞手的切换，链路正常时预发结束后要等下一条飞控状态才申请 OFFBOARD（R-75）。

### 5. AUTO.LAND 让步锁存到上锁

飞控进入 AUTO.LAND 后状态机进入 `AutoLandYield` 并锁存到上锁，不发 setpoint、不重启返航。唯一例外：链路丢失期间模式被切走时，按间隔重新请求 AUTO.LAND（方案第 7.6 节）。

### 6. 落地保护

不在空中（飞控未报告落地，且高出解锁高度 0.5 m）时不启动任务；任务的悬停、准备阶段变为不在空中，或其他阶段飞控报告落地，直接请求 AUTO.LAND。只用 `connected` 为真的 `/mavros/state` 判断解锁过程；节点在已解锁时启动视为空中重启，只看落地状态。

### 7. 规划失败的分流

点过近 → 原地降落；其他失败 → 航点降级；航点降级也失败 → 原地降落；点数不足 → 最多等 3 s 补点，仍不足则原地降落；规划 30 s 未完成 → 丢弃结果，在控制线程构建航点降级。规划在后台线程，控制周期不等它；悬停期间到达的结果在悬停结束后才处理（D3）。

### 8. Rejoin 是同一路径跟随器的低速模式

偏离用当前位置到局部路径投影点的距离判定，持续 0.25 s 才进入；回归时扩大前向窗口、缩短预瞄、平滑限速，位置和 yaw 仍取路径目标；退出要求横向偏差、实际速度和已发布目标速度同时满足。生产配置触发/退出距离为 0.5 m / 0.4 m，终点精度独立为 0.2 m。不要重新引入全轨迹最近点搜索、冻结时间点或固定冷却时间。

### 9. 起始对准

准备阶段保持触发位置，yaw 设为路径起始切向，误差 ≤ 15° 且已持有控制权才开始跟随（低电量且准备超时后不再等对准，R-85）；没有 yaw 斜坡，原地转速由 PX4（`MC_YAWRATE_MAX`）限制。航点降级没有切向，不要求对准。

### 10. 安全距离（D30）

只沿录制轨迹飞行，不在录制轨迹上的直线移动不超过 `safe_distance_m`（0.7 m），超过就原地保持并告警：保持点都取飞机所在的位置（重新申请期间跟着飞机，拿回 OFFBOARD 后固定）；开始和恢复跟随前检查偏差；跟随中投影失败或偏差超限时不回归；录制轨迹中相邻两点超过安全距离即为断点，规划不跨越，返航停在断点处悬停、不降落。用户明确要求杜绝直线飞向起点（2026-10-22），原地悬停时找点接上（D33）：在游标之前（停在断点处时为断点之前）找离飞机不超过安全距离、最近的录制点，截断到该点并重新规划，回到准备阶段。改这些逻辑前先读方案第 7.8 节。

### 11. 超时与升级（R6）

准备（30 s）、重新申请（10 s）、进度停滞（60 s，含停在断点处的到点悬停）三个等待由 `updateWaits` 每拍按上一拍的状态累计，只在里程计新鲜时计时（D31），重新申请期间准备和停滞冻结。升级（规则 6）放在各阶段的规则之后：先继续返航（找点接上、低电量时不再等起始对准），仍不能继续、等待超时且确认低电量，才原地降落（交接原因 `ESCALATED`，直接 AUTO.LAND）。电量正常或读数无效时只保持并告警（D11），不要加"超时就降落"；退出后低电量只告警（D1）。改这部分先读方案第 8.1 节和 R-85。

## 常见改动应该改哪里

| 目标 | 必看文件 |
| --- | --- |
| 改状态转移或输出 | 方案第 6、7 节 → `return_state_machine.hpp/.cpp` → 状态机单测与穷举检查 → 注入表 |
| 改条件判定（链路、CH7、电量、在空中） | 方案第 5 节 → 对应 `domain/*_monitor`、`pilot_input` → 单测 → 注入表 |
| 改条件组装或执行（规划、跟随、setpoint） | `backtrack_usecase.cpp`，以及 `trajectory_session`、`rejoin_policy`、`waypoint_fallback_session` |
| 改模式请求 | `mode_commander.cpp`、`ros_services.cpp` |
| 改话题名或消息类型 | `ros_subscribers`、`ros_publishers`、`backtrack_node.cpp`、`launch/backtrack.launch`，并同步前端对接文档 |
| 改参数 | 对应 config 结构体、`config_loader.cpp`、`backtrack_params.yaml`；闭环场景的 `productionConfig()` 和单测的 `makeConfig()` |
| 改调试日志 | `debug_log.cpp`、`backtrack_node.cpp`；逐拍记录和状态话题的格式在 `tick_report.cpp`，同步前端对接文档 |

## 文档可信度排序

1. `src/**`、`include/**`
2. `config/*.yaml`、`launch/`
3. [`doc/reference/SYSTEM_OVERVIEW.md`](doc/reference/SYSTEM_OVERVIEW.md)：当前实现的说明
4. [`doc/testing/`](doc/testing/)：测试方法和执行记录

重构要达到的行为（R8 还没有实现）以方案为准。`doc/testing/` 中注明"只适用于 dev"的文档不描述当前行为。

## 容易踩坑的地方

- 修改 `step()` 的判定顺序会改变规则优先级；先读方案第 6.3 节，再看穷举检查（`test_return_state_machine_invariants.cpp`）是否覆盖到。
- 条件比执行层的事实晚一拍：在单测里断言时间时要算上这一拍。
- 单测和闭环场景都要先送一条"未解锁"的飞控状态再解锁，并在解锁前就有里程计；否则 `AirborneMonitor` 按空中重启处理，高度条件不起作用。
- `nav_msgs/Odometry.twist` 位于 `child_frame_id`，订阅适配器已经旋转到父 ENU，不要再当作机体系速度处理。
- 加了新的关键判定，就在 [`scripts/analysis/mutation_check.py`](scripts/analysis/mutation_check.py) 的注入表中加一项，确认有单测能发现。

## 测试地图

用例逐条说明见 [`doc/testing/单元测试文档.md`](doc/testing/单元测试文档.md)，闭环场景、SITL 和基准见 [`doc/testing/闭环仿真与SITL测试.md`](doc/testing/闭环仿真与SITL测试.md)。

- [`test_link_monitor`](tests/unit/test_link_monitor.cpp)、[`test_pilot_input`](tests/unit/test_pilot_input.cpp)、[`test_battery_monitor`](tests/unit/test_battery_monitor.cpp)、[`test_airborne_monitor`](tests/unit/test_airborne_monitor.cpp)：条件层
- [`test_return_state_machine`](tests/unit/test_return_state_machine.cpp)：状态机场景
- [`test_return_state_machine_invariants`](tests/unit/test_return_state_machine_invariants.cpp)：随机游走和单步穷举，检查安全约束 SC1–SC7、SC9
- [`test_backtrack_usecase`](tests/unit/test_backtrack_usecase.cpp)：UseCase 闭环场景，用简化的飞控与链路模型覆盖方案第 11 节的场景；也包括状态话题的 JSON（S22）和逐拍记录回放（S31）
- [`test_mode_commander`](tests/unit/test_mode_commander.cpp)、[`test_planning_worker`](tests/unit/test_planning_worker.cpp)、[`test_debug_log`](tests/unit/test_debug_log.cpp)、[`test_config_loader`](tests/unit/test_config_loader.cpp)：模式请求、后台规划、调试日志、配置校验
- [`test_trajectory_session`](tests/unit/test_trajectory_session.cpp)、[`test_rejoin_policy`](tests/unit/test_rejoin_policy.cpp)、[`test_waypoint_fallback_session`](tests/unit/test_waypoint_fallback_session.cpp)、[`test_recording_buffer`](tests/unit/test_recording_buffer.cpp)、[`test_path_yaw`](tests/unit/test_path_yaw.cpp)、[`test_ros_subscribers`](tests/unit/test_ros_subscribers.cpp)：执行层与适配

运行：`DOCKER="sudo docker" scripts/docker/run_tests.sh amd64`；之后 `scripts/analysis/run_mutation_check.sh amd64`、`scripts/analysis/run_hazard_scenarios.sh amd64`。

## 给后续 LLM 的工作建议

- 先读方案第 6、7 节和[系统总览](doc/reference/SYSTEM_OVERVIEW.md)，再从 `backtrack_node.cpp → backtrack_usecase.cpp → return_state_machine.cpp` 读一遍。
- 先计划后改动；设计变化先写回方案并记审核记录，再写代码和单测；提交按依赖顺序分步，每个提交只做一件事。
- 改完跑全部单测、错误注入和闭环场景；涉及与飞控的交互时做 SITL。
- 遇到文档和代码不一致，以代码为准，并把不一致写出来修正文档。
