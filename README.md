# px4_backtrack_optimised

`px4_backtrack_optimised` 是一个基于 ROS1 `catkin` 的 PX4 返航节点，采用接近 DDD 的分层结构实现。节点在飞行中录制航迹，在遥控链路丢失、低电量或飞手长按 CH7 时启动返航任务：用 `trajectory_core` 从录制点逆序生成平滑轨迹，通过 MAVROS 下发 `PositionTarget` 与模式切换请求，沿原路飞回录制起点后交给 PX4 的 `AUTO.LAND` 降落。

这份 README 面向开发者和集成人员，重点说明仓库现状、模块职责、ROS 接口、配置与启动方式。全部文档见[文档导航](doc/README.md)。

## 核心能力

- 解锁 15 s 后开始录制航迹（不超过 15 Hz）。退出返航时录制截断到已返航到的位置，之后继续录制；再次失控时先沿退出后的飞行路径回到退出位置，再沿原路回到录制起点。
- 三类触发：遥控链路丢失（CH18 失控帧漏桶确认，或 RC 消息中断）、低电量（持续 3 s 低于 15%，锁存到下一次解锁）、CH7 长按 2 s。不在空中时不启动；录制点不足时最多等 3 s 补点，仍不足则原地降落。
- 失控时先原地悬停 15 s，期间完成规划；低电量和长按直接进入准备。
- 规划在后台线程进行，控制周期照常发布保持点；10 s 未完成告警，30 s 未完成则改为航点降级返航。
- 规划前对悬停形成的密集录制点做空间抽稀；生成失败时改为逆序航点降级返航，起终点过近时原地降落。
- 按实际位置在路径上的局部投影推进进度，不依赖 wall time；偏离超过 0.5 m 时沿路径低速回归。
- 控制权由状态机管理：预发 setpoint 后申请 OFFBOARD；本模块断流引起的飞控回退不算接管，等飞控自动回到 OFFBOARD；链路正常时飞手拨开关或短按 CH7 退出返航，链路丢失或不稳定时被切出则重新申请控制权。
- 只沿录制轨迹飞行（D30）：离路径超过安全距离（0.7 m）、重新拿到控制权时飞机已被移开、或录制轨迹中有超过安全距离的跳变时，原地悬停并告警，不直线飞过去；悬停时在录制轨迹上找离飞机不超过安全距离的点，找到就从那里继续返航（D33）。
- 返航卡住时按超时升级（方案第 8.1 节）：准备 30 s、重新申请 10 s、进度停滞 60 s。超时后电量正常时保持并告警；确认低电量时先尝试继续返航，仍不能继续才原地降落（D32）。计时只在里程计新鲜时累计。
- 到点悬停 5 s 后降落交接：链路正常时经 POSCTL 请求 AUTO.LAND，链路丢失时直接请求；飞控进入 AUTO.LAND 后让步到上锁。
- 模式请求在单独线程中异步调用，未生效时每 1 s 重发，3 s 未生效告警。
- 发布状态字符串、链路丢失、低电量，以及 JSON 状态话题（控制权、任务、模式请求、告警码）；事件写 rosout，可选写入调试日志（事件、逐拍记录、规划 CSV）。
- 可选在 rviz 中查看录制点和返航轨迹，可选用 rosbag 录原始话题。

## 仓库现状

当前仓库的主实现是 C++ 版本，入口为 [`src/backtrack_node.cpp`](src/backtrack_node.cpp)。

本分支（`feat/return-state-policy`）按[返航触发与状态机重构方案](doc/design/返航触发与状态机重构方案.md)分阶段实施（方案第 13 节）。R1 条件层、R2 状态机核心、R3 UseCase 接线、R4 后台规划、R5 退出后截断轨迹、R5b 安全距离（D30）、R5c 原地悬停时找点接上（D33）、R6 超时与升级、R7 上报与调试日志、R8 文档与部署已完成（2026-10-23），节点运行的是新状态机，dev 的暂停/恢复语义已删除。

本分支还没有实飞，实飞版本仍是 dev。完整 SITL 已完成（PX4 v1.15.4 SIH，14 个场景全部通过，含 F8，见[闭环仿真与 SITL 测试](doc/testing/闭环仿真与SITL测试.md)第 4 节）；上机按[部署说明](doc/reference/部署说明.md)部署，按[实飞测试清单](doc/testing/实飞测试清单.md)先做台架检查，再逐项实飞。

## 架构概览

仓库按“入口 + Application + Domain + Adapters + Infrastructure”组织：

- `src/backtrack_node.cpp`
  加载参数，创建订阅、发布、服务客户端和 `BacktrackUseCase`，按 `publish_hz` 驱动控制周期，发布状态话题，把事件和逐拍记录写到 rosout 和调试日志
- `include/px4_backtrack/application` / `src/application`
  `BacktrackUseCase` 组装条件、调用状态机、执行命令、计算 setpoint，不访问 ROS；`ModeCommander` 负责模式请求的发送、重发和失败告警；`PlanningWorker` 在后台线程规划；`tick_report` 生成状态话题的 JSON 和逐拍记录的行
- `include/px4_backtrack/domain` / `src/domain`
  条件层（链路、CH7、电量、在空中）、`ReturnStateMachine`（任务阶段、控制权、跟随方式三个区域），以及录制、轨迹会话、航点降级、回归限速、切向 yaw 等执行组件
- `include/px4_backtrack/adapters` / `src/adapters`
  ROS 消息转换、异步 `set_mode`、可视化发布
- `include/px4_backtrack/infrastructure` / `src/infrastructure`
  参数加载（全部必填，启动时校验 `trajectory_core` 配置）、调试日志、规划 CSV 导出

各模块的职责和每个控制周期的处理顺序见[系统总览](doc/reference/SYSTEM_OVERVIEW.md)第 3 节。

## 主要执行流程

1. 启动时 `ConfigLoader` 从参数服务器读取返航参数（缺少任何一项即启动失败），再加载并校验 `trajectory_config.yaml`（非法即启动失败）；打开调试日志时，先写入生效参数和版本。
2. 订阅回调把飞控状态、落地状态、里程计、RC 和电池数据连同接收时间送入 `BacktrackUseCase`。
3. 每个控制周期（20 Hz）：条件层推进到当前时间 → 组装 `Conditions` 快照 → `ReturnStateMachine::step()` 给出命令、setpoint 策略和模式意图 → UseCase 执行命令（新飞行周期复位、提交或取消后台规划）、推进路径进度、计算 setpoint；规划结果在之后的周期中取回 → `ModeCommander` 决定本拍要发出的模式请求。
4. 节点发布状态（含 JSON 状态话题）和 setpoint，在单独线程中调用 `/mavros/set_mode`，并把事件写到 rosout 和调试日志；打开逐拍记录时每拍再写一行。

## ROS 接口

### 输入

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/state` | `mavros_msgs/State` | 解锁状态和飞控模式；只用 `connected` 为真的消息 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | 飞控落地状态，用于在空中判定 |
| `/Odometry_px4` | `nav_msgs/Odometry` | 当前位置、线速度、航向和录制时间戳；含非有限值的消息被忽略 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | `CH7`（按钮）、`CH18`（失控标志）和 RC 消息时序 |
| `/mavros/battery` | `sensor_msgs/BatteryState` | 电池电量百分比（`percentage`，0.0~1.0） |

### 输出

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 任务中的保持点、路径目标和回归目标 |
| `/px4_backtrack/state` | `std_msgs/String` | 状态字符串，含义见[前端对接文档](doc/reference/前端对接开发文档.md)第 3 节 |
| `/status_rc_loss` | `std_msgs/Bool` | 链路丢失：CH18 失控帧累计 1 s 或 RC 消息中断 0.35 s 时为 `true`，持续正常 2 s 后回到 `false` |
| `/px4_backtrack/low_battery` | `std_msgs/Bool` | 低电量，确认后锁存到下一次解锁 |
| `/px4_backtrack/status` | `std_msgs/String` | JSON，每拍一条：状态、控制权、任务编号和原因、模式请求、告警码，字段见[前端对接文档](doc/reference/前端对接开发文档.md) |
| `/px4_backtrack/debug/waypoints` | `visualization_msgs/MarkerArray` | 录制航点可视化（latched，需启用 `debug_enable_rviz_publish`） |
| `/px4_backtrack/debug/trajectory` | `nav_msgs/Path` | 返航轨迹可视化（latched，需启用 `debug_enable_rviz_publish`） |

### 服务

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 请求切换 `OFFBOARD`、`POSCTL`、`AUTO.LAND`；在单独线程中调用，不阻塞控制周期 |

说明：

- 坐标约定使用 ENU；`SetpointPublisher` 将 ENU 值直接填入 `FRAME_LOCAL_NED` 的 `PositionTarget`，依赖 MAVROS 完成坐标转换。
- `/Odometry_px4` 的 pose 位于父 ENU，twist 按 `nav_msgs/Odometry` 语义位于 `child_frame_id`；订阅适配器会先将线速度旋转到父 ENU，再交给终点和回归判定。
- 告警码在 `/px4_backtrack/status` 中上报，也写 rosout 和调试日志。

## 配置

### 返航参数

文件：[`config/backtrack_params.yaml`](config/backtrack_params.yaml)

参数全部必填，代码中没有默认值；launch 没有加载其中任何一项时，节点启动时报错退出。各参数的含义和建议值见方案第 10 节。主要分组：

- 基础：`publish_hz`、`record_delay_s`、`record_min_points`、`record_max_hz`
- `CH7`：`enable_trigger_ch7`、`ch7_pressed_threshold`、`ch7_hold_s`、`ch7_exit_min_s`
- `CH18`：`enable_trigger_ch18`、`ch18_failsafe_threshold`、`ch18_confirm_s`、`link_loss_leak_rate`、`ch18_hover_wait_s`
- RC 消息丢失：`enable_trigger_rc_loss`、`rc_loss_timeout_s`、`rc_loss_debounce_s`
- 链路恢复与退化：`link_recover_s`、`link_degraded_window_s`
- 低电量：`enable_trigger_low_battery`、`low_battery_threshold`、`low_battery_confirm_s`
- 输入新鲜度与在空中判定：`odom_stale_s`、`fcu_state_stale_s`、`airborne_min_height_m`
- 任务与控制权：`return_pending_max_s`、`pre_offboard_setpoint_count`、`fallback_gap_s`、`fallback_window_s`、`fallback_timeout_s`、`safe_distance_m`、`handover_posctl_timeout_s`、`mode_request_interval_s`、`mode_request_timeout_s`
- 后台规划：`planning_warn_s`、`planning_timeout_s`
- 超时与升级：`prepare_warn_s`、`prepare_timeout_s`、`reclaim_timeout_s`、`stall_min_progress_m`、`stall_warn_s`、`stall_timeout_s`
- 退出后新轨迹：`cursor_window_m`
- 回归：`rejoin_trigger_dist_m`、`rejoin_success_dist_m`、`rejoin_v_max_m_s`、`rejoin_lookahead_time_s`、`rejoin_trigger_hold_s`、`rejoin_success_hold_s`
- 降落：`landing_hover_s`
- 轨迹配置路径：`trajectory_config_path`
- 调试：`debug_trajectory_sample_dt`、`debug_enable_rviz_publish`、`debug_log_enable`、`debug_log_dir`、`debug_log_keep_runs`、`debug_log_trace`

几点约定：

- 回归以 `0.5 m` 触发、以 `0.4 m` 回到安全走廊；终点精度独立固定为 `0.2 m`，二者互不影响。
- `rejoin_lookahead_time_s` 同时用于正常预瞄和回归时缩短预瞄。
- 准备阶段先对准返航起始切向 yaw，误差大于 15° 时不开始水平返航。目标 yaw 一次设置，原地转速由 PX4（如 `MC_YAWRATE_MAX`）限制；`trajectory_config.yaml` 的 `yaw_rate_max` 只约束运动轨迹。

### `trajectory_core` 参数

文件：[`config/trajectory_config.yaml`](config/trajectory_config.yaml)

该文件控制平滑轨迹生成的采样、走廊约束、拐角处理、yaw 平滑、末端收敛以及速度加速度限制。

当前返航侧在调用 `trajectory_core` 之前，还会基于 `max(min_point_dist_m, ds_m * 0.5)` 对录制点做一次额外空间抽稀，用于压缩停顿/悬停阶段形成的高密度点云团。抽稀时首点和末点始终保留；中间点只有在距离上一个有效点达到阈值时才加入，距离不足时直接忽略，不会覆盖上一个有效点。这样低速连续飞行可以靠累计距离保留主路径，真正悬停的密集点会被压缩。

## 构建与启动

### 依赖

工程显式依赖：

- ROS1 `catkin`
- `roscpp`
- `std_msgs`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `mavros_msgs`
- `visualization_msgs`
- `tf2`
- `trajectory_core`
- `Eigen3`

`trajectory_core` 必须已经存在于同一个 catkin 工作空间，或已经通过环境正确导出。机载固定用 `origin/dev` 的 `f27f3ad`（D13），部署步骤见[部署说明](doc/reference/部署说明.md)。

### 构建

在 catkin 工作空间根目录执行：

```bash
catkin_make --pkg px4_backtrack_optimised
source devel/setup.bash
```

### 启动

```bash
roslaunch px4_backtrack_optimised backtrack.launch
```

`launch/backtrack.launch` 会加载 [`config/backtrack_params.yaml`](config/backtrack_params.yaml)。参数没有代码默认值，调整时直接修改该 YAML；缺少参数或 `trajectory_config.yaml` 校验不通过时节点启动失败。

需要原始话题时加上 `record_bag:=true`，用 `rosbag record` 录输入、setpoint 和本模块的话题，每 1 GB 分一个文件。`bag_dir` 默认 `~/.ros`，须已存在：

```bash
roslaunch px4_backtrack_optimised backtrack.launch record_bag:=true bag_dir:=$HOME/Projects/backtrack_ws/debug
```

## 调试与可视化

调试日志和 rviz 发布由 `backtrack_params.yaml` 开关控制，录包见上面的启动参数。当前配置打开了 rviz 发布，关闭了调试日志。

### 调试日志

设置 `debug_log_enable: true` 后，节点每次启动在 `debug_log_dir`（留空为 `~/.ros/backtrack_logs`，非空时为绝对路径或以 `~/` 开头）下建一个运行目录 `run_YYYYMMDD_HHMMSS/`，只保留最近 `debug_log_keep_runs`（20）个。写盘在后台线程；写失败时停止写日志并告警 `DEBUG_LOG_FAILED`，返航不受影响。试飞期间建议打开。目录里有：

- `events.jsonl`：第一行是生效参数和本模块、`trajectory_core` 的提交号；之后是状态转换（带规则编号和原因）、飞控模式和解锁变化、条件边沿、CH7 指令、任务开始与结束、规划的提交、结果、超时和取消、模式请求、自身断流，以及告警的出现和消失；
- `trace.csv`（`debug_log_trace: true` 时，约 30 MB/小时）：每拍一行，条件快照和本拍输出，可以用 PlotJuggler 打开，也可以回放到状态机复现当时的决策；
- `plan_<n>_input.csv`、`plan_<n>_trajectory.csv`：第 n 次规划的输入（每次都写）和采样轨迹（规划成功时写，采样步长 `debug_trajectory_sample_dt`）。

```bash
# 按时间线查看事件；-t 按类型过滤，-m 只看某个任务
python3 scripts/analysis/show_debug_log.py ~/Projects/backtrack_ws/debug/run_20261023_140023 -t state,control,alert

# 绘制最近一次规划的输入和轨迹（依赖 matplotlib、numpy）
python3 scripts/plot_debug_csv.py --dir ~/Projects/backtrack_ws/debug/run_20261023_140023
```

### rviz 实时可视化

设置 `debug_enable_rviz_publish: true` 后，轨迹生成成功时发布：

- `/px4_backtrack/debug/waypoints`（`visualization_msgs/MarkerArray`）— 绿色球体标记 + 连线，起止点分别用红/蓝大球标注
- `/px4_backtrack/debug/trajectory`（`nav_msgs/Path`）— 返航轨迹路径

两个话题均为 latched，rviz 连接后即可看到最新数据。

## 目录速览

```text
.
├── CMakeLists.txt
├── package.xml
├── launch/backtrack.launch
├── config/
│   ├── backtrack_params.yaml
│   └── trajectory_config.yaml
├── include/px4_backtrack/       # 头文件，分层同 src/
├── src/
│   ├── adapters/                # ROS 订阅、发布、异步 set_mode、可视化
│   ├── application/             # BacktrackUseCase、ModeCommander、后台规划、上报格式
│   ├── domain/                  # 条件层、ReturnStateMachine、录制、轨迹会话、航点降级、回归
│   ├── infrastructure/          # 参数加载、调试日志、规划 CSV 导出
│   └── backtrack_node.cpp
├── tests/unit/                  # gtest 单测（17 个文件，见单元测试文档）
├── scripts/
│   ├── docker/                  # ROS Noetic 容器编译与单测
│   ├── analysis/                # 闭环危害场景、错误注入检查、调试日志查看
│   ├── bench/                   # 轨迹生成耗时基准
│   ├── sitl/                    # PX4 SITL 飞控行为检查和返航场景
│   └── plot_debug_csv.py        # 规划 CSV 绘图
└── doc/
    ├── README.md                # 文档导航
    ├── reference/               # 系统说明、前端对接、部署说明
    ├── design/                  # 设计方案
    ├── testing/                 # 测试方法、清单和执行记录
    ├── reviews/                 # 审核与分析
    └── archive/                 # 已修复缺陷的分析报告
```

## 测试

| 层次 | 命令 | 说明 |
| --- | --- | --- |
| 单元测试 | `scripts/docker/run_tests.sh amd64` | 在全新的 ROS Noetic 容器中编译并运行全部单测（233 个）和 trajectory_core 的单测；参数换成 `arm64` 时在 QEMU 中按机载架构复核（本机编译加单测约 30 分钟） |
| 错误注入 | `scripts/analysis/run_mutation_check.sh amd64` | 检验单测能否发现典型错误（90 种） |
| 闭环危害场景 | `scripts/analysis/run_hazard_scenarios.sh amd64` | 假飞控驱动真实 `BacktrackUseCase`，场景 H01–H16 |
| 规划耗时 | `scripts/bench/run_gen_bench.sh amd64 10 30` | 轨迹生成耗时随录制时长的增长 |
| SITL | `scripts/sitl/run_sitl_checks.sh [check\|scenarios]` | PX4 v1.15.4 SITL 中确认方案依赖的飞控行为（N1、N2），运行本节点回放 14 个返航场景（含 F8） |

本机需要 sudo 运行 Docker 时设置 `DOCKER="sudo docker"`。后三类复用单元测试留下的容器，先跑一次单元测试。方法与执行记录见[单元测试文档](doc/testing/单元测试文档.md)和[闭环仿真与 SITL 测试](doc/testing/闭环仿真与SITL测试.md)。

已有 ROS 工作空间时，也可以直接在工作空间根目录执行：

```bash
catkin_make --pkg trajectory_core px4_backtrack_optimised
catkin_make run_tests_px4_backtrack_optimised
catkin_test_results
```

## 相关文档

- [文档导航](doc/README.md)：按用途分类的完整索引。
- [返航触发与状态机重构方案](doc/design/返航触发与状态机重构方案.md)：当前开发的依据。
- [系统总览](doc/reference/SYSTEM_OVERVIEW.md)：当前实现的接口、模块划分、处理顺序、状态机、执行层、调试与观测。
- [前端对接开发文档](doc/reference/前端对接开发文档.md)：状态字符串、话题语义、状态话题的 JSON、告警码和 UI 建议。
- [部署说明](doc/reference/部署说明.md)：机载环境、固定 `trajectory_core` 版本、编译、配置、起飞前检查、日志位置、更新与回退。
- [返航程序系统性分析（2026-10-20）](doc/reviews/返航程序系统性分析_2026-10-20.md)：危害分析、PX4 源码核对、闭环仿真和 SITL 确认。
- [单元测试文档](doc/testing/单元测试文档.md)、[闭环仿真与 SITL 测试](doc/testing/闭环仿真与SITL测试.md)：测试方法与执行记录。
- [实飞测试清单](doc/testing/实飞测试清单.md)：上机前的 SITL、台架检查和实飞项目。
- `trajectory_core` 的接口说明见该仓库的 `doc/reference/API接口文档.md`。

## 已知注意点

- `README` 与 `LLM_MAP` 以当前 C++ 代码为准。
- 电量正常或读数无效时，返航卡住只保持并告警，不会降落（D11）；电量读数无效时电池耗尽前没有降落兜底，起飞前要确认没有 `BATTERY_INVALID`。
- 设计前提：无人机速度不超过 2 m/s；里程计中断属于本模块不处理的极端情况，延迟时只冻结进度、原地保持（D31）。
- 单元测试只覆盖少量 ROS adapters，没有 launch 级测试（SITL 场景用机载参数文件启动节点）；修改状态机、条件层、轨迹接口或参数装配后，除单测外还要跑闭环场景，涉及与飞控交互时再做 SITL。
- 航线模块和本模块都发 setpoint，两者之间还没有仲裁；测试返航时不要让航线模块处于回放状态（方案第 16 节）。
