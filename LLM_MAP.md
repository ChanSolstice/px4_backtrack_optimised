# 代码入口

当前行为与验收唯一维护入口：[工程修复与验证](doc/工程修复与验证.md)。先读该文件，再按下表定位源码。历史报告不作为当前实现规范。

| 职责 | 文件入口 |
| --- | --- |
| 节点组装、steady 收时、WallTimer、输出意图 | src/backtrack_node.cpp |
| 输入保护、飞行周期、触发、规划、控制优先级 | src/application/backtrack_usecase.cpp |
| 单任务规划、降级、debug、结果代次 | src/application/planning_worker.cpp |
| 状态和保持/模式输出 | src/domain/state_machine.cpp |
| 连续参考时钟、局部投影、同源前馈、完成 | src/domain/trajectory_session.cpp |
| 原始三维折线和制动预算 | src/domain/recorded_route.cpp |
| 保留拐角、停稳转向、剩余航点恢复 | src/domain/waypoint_fallback_session.cpp |
| 录制、触发、路径 yaw、Rejoin 退出 | src/domain/recording_buffer.cpp、trigger_evaluator.cpp、path_yaw.cpp、rejoin_policy.cpp |
| ROS 坐标和消息校验、setpoint/状态、后台模式 RPC | src/adapters/ros_subscribers.cpp、ros_publishers.cpp、ros_services.cpp |
| 配置、CSV 和可视化 | src/infrastructure/、src/adapters/debug_publishers.cpp |
| 当前运行参数 | config/backtrack_params.yaml、trajectory_config.yaml、launch/backtrack.launch |
| 单元/业务回归与真实节点隔离联调 | tests/unit/、tests/integration/ros_control_scenarios.py |

必须保留的约定：

- UseCase 由串行 ROS 主线程持有；后台工作只持有独立快照/意图/结果，不直接操作业务状态。
- DISARM、AUTO.LAND、外部 OFFBOARD 退出优先于输入/完成和自动触发。定位复位不得发送旧坐标；外部 ControlYield 不自动抢回控制。
- 实际投影设置进度上界；同一个参考进度给出 position/velocity/acceleration/yaw/yaw_rate。时间到达名义终点不等于飞机到点。
- fallback 不依赖失败轨迹，不丢原始拐角，不从头恢复；临时接入只在局部剩余路径。
- 时钟使用 steady；消息 stamp 用于消息有效性和录制；odom twist 先从 child 旋转到父 ENU。
- 新飞行、暂停和接管取消旧代次；同步 RPC 结果不作为模式已切换依据。
- 原路偏移指标与目标跟踪误差分别记录。不得把软件门槛、mock 或编译通过表述为实机达标。

修改配置应同时检查 YAML、ConfigLoader、UseCaseConfig 与验证预算；修改状态或话题应同步唯一维护文档。每个问题独立提交，算法接口以 trajectory_core 公共头文件及该仓库的统一审核为准。
