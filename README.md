# px4_backtrack_optimised

ROS1 / MAVROS 的多旋翼原路返航节点。飞行中录制航迹，CH7、CH18 故障、RC 丢失或低电量触发后，使用独立 `trajectory_core` 生成逆序轨迹；规划失败尝试保持原路拐角的 position+yaw 航点模式。

实际进度约束连续参考时钟，支持慢飞、长时间停留、Rejoin 和剩余路径恢复。原路三维监测、输入有效性与控制权交接由返航模块负责；规划/调试和模式服务在专门后台线程执行。

当前架构、接口、参数、每项修复、验证边界及后续计划统一见 [工程修复与验证](doc/工程修复与验证.md)。代码入口见 [LLM_MAP](LLM_MAP.md)，历史资料在 `doc/archive/`。当前 ROS 回归通过，PX4 SIH 因健康检查拒绝解锁，飞控闭环及 RK3588/实飞待验收。

```bash
# Docker ROS_Noetic；src 中同时放 trajectory_core 和本仓库
source /opt/ros/noetic/setup.bash
cd /root/ROS_Noetic/backtrack_ws
catkin_make --pkg trajectory_core px4_backtrack_optimised -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
roslaunch px4_backtrack_optimised backtrack.launch
```

定位源和 MAVROS 由工程启动。参数入口为 `config/backtrack_params.yaml` 与 `config/trajectory_config.yaml`；更改后需按统一文档的预算复验。指定航线模块应直接消费 `trajectory_core`，不依赖本节点的状态机。
