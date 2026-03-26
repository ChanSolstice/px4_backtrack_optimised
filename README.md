# px4_backtrack_optimised

`px4_backtrack_optimised` 是一个基于 ROS1 `catkin` 的 PX4 返航节点，采用接近 DDD 的分层结构实现。节点在飞行中录制航迹，在满足触发条件后使用 `trajectory_core` 生成逆序返航轨迹，并通过 MAVROS 下发 `PositionTarget` 与模式切换请求。

这份 README 面向开发者和集成人员，重点说明仓库现状、模块职责、ROS 接口、配置与启动方式。更细的设计背景见 [`doc/返航实现技术文档.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/返航实现技术文档.md)。

## 核心能力

- 飞行周期复位后延时开始录制航迹
- 支持 3 类返航触发：`CH7` 长按、`CH18` 故障确认、RC 消息丢失
- 使用 `trajectory_core` 从录制点逆序生成平滑返航轨迹
- 返航中支持偏离检测与 rejoin
- 返航中支持暂停，再次触发后恢复
- 触发时若录制点数不足，自动降级为原地降落
- 返航结束后先固定点悬停，再发起 `POSCTL -> AUTO.LAND` 过渡（非 OFFBOARD 时直接请求 `AUTO.LAND`）
- 发布内部状态与 RC 丢失持续状态，便于联调
- 可选 debug 模式：轨迹生成后自动导出录制航点与返航轨迹 CSV，并可在 rviz 中实时查看

## 仓库现状

当前仓库的主实现是 C++ 版本，入口为 [`src/backtrack_node.cpp`](/home/channing/Drone/Projects/px4_backtrack_optimised/src/backtrack_node.cpp)。`doc/TECHNICAL_DOCUMENTATION.md` 与 [`doc/backtrack_node.py`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/backtrack_node.py) 描述的是旧版 Python 基线；阅读当前工程时，应优先以 C++ 源码和 [`doc/返航实现技术文档.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/返航实现技术文档.md) 为准。

## 架构概览

仓库按“入口 + Application + Domain + Adapters + Infrastructure”组织：

- `src/backtrack_node.cpp`
  负责组装配置、订阅器、发布器、服务客户端和 `BacktrackUseCase`
- `include/px4_backtrack/application` / `src/application`
  用例层，负责把订阅数据、状态机、轨迹会话和触发器串成完整控制流程
- `include/px4_backtrack/domain` / `src/domain`
  领域层，包含状态机、录制缓存、触发判定、轨迹会话、rejoin 策略
- `include/px4_backtrack/adapters` / `src/adapters`
  ROS 适配层，封装 MAVROS 订阅、发布与 `set_mode` 服务调用
- `include/px4_backtrack/infrastructure` / `src/infrastructure`
  配置加载，负责把 ROS 参数和 `trajectory_core` YAML 转成 `UseCaseConfig`

## 主要执行流程

1. 启动节点后，`ConfigLoader` 从参数服务器读取返航参数，再加载 `trajectory_config.yaml`。
2. `BacktrackNode` 建立 `/mavros/state`、`/Odometry_px4`、`/mavros/rc/in` 订阅，以及 setpoint、状态发布器和 `/mavros/set_mode` 客户端。
3. 订阅回调把状态、里程计和 RC 数据送入 `BacktrackUseCase`。
4. 20Hz 定时器调用 `BacktrackUseCase::tick()`，由它依次处理飞行周期复位、录制延时、触发检测、轨迹生成、rejoin 与状态机推进。
5. `BacktrackStateMachine` 决定当前是否应该发布 setpoint、是否请求切换 `OFFBOARD` / `POSCTL` / `AUTO.LAND`。
6. Adapters 层把领域层的 `SetpointTarget` 转成 `mavros_msgs/PositionTarget` 并发布。

## ROS 接口

### 输入

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/state` | `mavros_msgs/State` | 获取 `armed` 与当前飞控模式 |
| `/Odometry_px4` | `nav_msgs/Odometry` | 提供当前位置、航向和录制时间戳 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | 提供 `CH7`、`CH18` 和 RC 消息时序 |

### 输出

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 发布悬停、返航和 rejoin setpoint |
| `/px4_backtrack/state` | `std_msgs/String` | 发布内部状态机状态 |
| `/status_rc_loss` | `std_msgs/Bool` | RC 丢失持续状态（`isRCLost()` 电平语义，丢失期间持续为 `true`） |
| `/px4_backtrack/debug/waypoints` | `visualization_msgs/MarkerArray` | 录制航点可视化（latched，需启用 `debug_enable_rviz_publish`） |
| `/px4_backtrack/debug/trajectory` | `nav_msgs/Path` | 返航轨迹可视化（latched，需启用 `debug_enable_rviz_publish`） |

### 服务

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 请求切换 `OFFBOARD`、`POSCTL`、`AUTO.LAND` |

说明：

- 坐标约定使用 ENU；`SetpointPublisher` 将 ENU 值直接填入 `FRAME_LOCAL_NED` 的 `PositionTarget`，依赖 MAVROS 完成坐标转换。
- `/status_rc_loss` 是电平语义：RC 丢失期间持续为 `true`，RC 恢复后自动回到 `false`。

## 配置

### 返航参数

文件：[`config/backtrack_params.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/backtrack_params.yaml)

说明：

- `CH7` / `CH18` / `RC loss` 触发参数现在以 `backtrack_params.yaml` 为唯一运行时来源
- `ConfigLoader` 会从 ROS 参数服务器强制读取这些触发参数；如果 launch 没有把它们加载进来，节点会在启动时报错退出
- `TriggerConfig` 头文件不再提供业务默认值，避免“头文件默认值”和 YAML 配置并存导致行为不一致

主要分组：

- 基础录制参数：`publish_hz`、`record_delay_s`、`record_min_points`、`record_max_hz`
- `CH7` 参数：`enable_trigger_ch7`、`ch7_pressed_threshold`、`ch7_hold_s`
- `CH18` 参数：`enable_trigger_ch18`、`ch18_failsafe_threshold`、`ch18_confirm_s`、`ch18_hover_wait_s`
- RC 丢失参数：`enable_trigger_rc_loss`、`rc_loss_timeout_s`、`rc_loss_debounce_s`
- OFFBOARD 参数：`pre_offboard_setpoint_count`
- Rejoin 参数：`rejoin_trigger_dist_m`、`rejoin_success_dist_m`、`rejoin_v_max_m_s`、`rejoin_lookahead_time_s`
- 降落参数：`landing_hover_s`
- 轨迹配置路径：`trajectory_config_path`
- Debug 参数：`debug_enable_csv_dump`、`debug_csv_output_dir`、`debug_trajectory_sample_dt`、`debug_enable_rviz_publish`

### `trajectory_core` 参数

文件：[`config/trajectory_config.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/trajectory_config.yaml)

该文件控制平滑轨迹生成的采样、走廊约束、拐角处理、yaw 平滑、末端收敛以及速度加速度限制。

## 构建与启动

### 依赖

工程显式依赖：

- ROS1 `catkin`
- `roscpp`
- `std_msgs`
- `nav_msgs`
- `geometry_msgs`
- `mavros_msgs`
- `visualization_msgs`
- `trajectory_core`
- `Eigen3`

`trajectory_core` 必须已经存在于同一个 catkin 工作空间，或已经通过环境正确导出。

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

`launch/backtrack.launch` 会加载 [`config/backtrack_params.yaml`](/home/channing/Drone/Projects/px4_backtrack_optimised/config/backtrack_params.yaml)。
如果你调整触发阈值，应直接修改该 YAML，而不是依赖代码侧默认构造值。

## Debug 可视化

节点支持两种可选的 debug 输出，均通过 `backtrack_params.yaml` 开关控制，默认关闭。

### CSV 文件导出

设置 `debug_enable_csv_dump: true` 后，轨迹生成成功时自动将录制航点和采样轨迹写入 CSV：

- 输出目录默认为 `~/.ros/backtrack_debug/`，可通过 `debug_csv_output_dir` 指定
- 文件名带时间戳：`waypoints_YYYYMMDD_HHMMSS.csv`、`trajectory_YYYYMMDD_HHMMSS.csv`
- 采样步长由 `debug_trajectory_sample_dt` 控制（默认 0.05s）

使用 Python 脚本查看：

```bash
# 自动选取最新一组文件
python3 scripts/plot_debug_csv.py --latest

# 指定文件
python3 scripts/plot_debug_csv.py waypoints.csv trajectory.csv
```

脚本依赖 `matplotlib` 和 `numpy`，绘制 3D 路径对比、俯视图、高度剖面和偏航剖面。

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
├── include/px4_backtrack/
│   ├── adapters/
│   ├── application/
│   ├── domain/
│   └── infrastructure/
├── scripts/
│   └── plot_debug_csv.py
├── src/
│   ├── adapters/
│   ├── application/
│   ├── domain/
│   ├── infrastructure/
│   └── backtrack_node.cpp
├── tests/
│   └── unit/
│       ├── test_main.cpp
│       ├── test_recording_buffer.cpp
│       ├── test_trigger_evaluator.cpp
│       ├── test_state_machine.cpp
│       ├── test_rejoin_policy.cpp
│       └── test_trajectory_session.cpp
└── doc/
    ├── SYSTEM_OVERVIEW.md
    ├── 返航实现技术文档.md
    ├── 实现任务清单.md
    ├── TECHNICAL_DOCUMENTATION.md
    ├── API接口文档.md
    └── backtrack_node.py
```

## 测试

当前仓库已包含一组基础 `gtest` 单元测试，重点覆盖领域层与 `trajectory_core` 接入层：

- `RecordingBuffer`
- `TriggerEvaluator`
- `BacktrackStateMachine`
- `RejoinPolicy`
- `TrajectorySession`

在 catkin 工作空间根目录执行：

```bash
catkin_make --pkg trajectory_core px4_backtrack_optimised
catkin_make run_tests_px4_backtrack_optimised
catkin_test_results
```

如果只想手动运行单个测试二进制，可在 `devel/lib/px4_backtrack_optimised/` 下执行对应测试程序。

## 相关文档

- [`doc/SYSTEM_OVERVIEW.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/SYSTEM_OVERVIEW.md)
  基于当前 C++ 代码整理的系统级总览，含状态机迁移图与主要处理流程
- [`doc/返航实现技术文档.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/返航实现技术文档.md)
  当前 C++ 方案的设计说明，最接近本仓库目标结构
- [`doc/实现任务清单.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/实现任务清单.md)
  分阶段实现计划，适合回看模块拆分意图
- [`doc/API接口文档.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/API接口文档.md)
  `trajectory_core` 的接口说明
- [`doc/TECHNICAL_DOCUMENTATION.md`](/home/channing/Drone/Projects/px4_backtrack_optimised/doc/TECHNICAL_DOCUMENTATION.md)
  旧版 Python 节点行为文档，仅可作为历史对照

## 已知注意点

- `README` 与 `LLM_MAP` 以当前 C++ 代码为准，不以旧版 Python 文档为准。
- 当前仓库已有基础单元测试，但尚未覆盖 ROS adapters 和 launch 级联调；修改状态机、触发器、轨迹接口或参数装配后，仍建议至少做一次编译和联调验证。
