# px4_backtrack_optimised

ROS1 / MAVROS 的多旋翼原路返航节点。飞行中录制航迹，CH7、CH18 故障、RC 丢失或低电量触发后，使用独立 `trajectory_core` 生成逆序轨迹；规划失败尝试保持原路拐角的 position+yaw 航点模式。

实际进度约束连续参考时钟，支持慢飞、长时间停留、Rejoin 和剩余路径恢复。原路三维监测、输入有效性与控制权交接由返航模块负责；规划/调试和模式服务在专门后台线程执行。

默认保留 CSV 导出，并额外录制原始 ROS 输入/输出、控制快照、完整规划输入/结果及文本日志。磁盘写入由独立 rosbag 进程承担；`config/backtrack_params.yaml` 的 `log_output_dir` 可设置绝对路径，留空使用运行用户的 `~/.ros/backtrack_logs`。Docker 中填写容器内路径，并挂载该目录以在主机保留日志。离线工具生成事件、统计和失败规划输入，使用方式见统一文档的“运行日志与离线排查”。

当前架构、接口、参数、每项修复、验证边界及后续计划统一见 [工程修复与验证](doc/工程修复与验证.md)。代码入口见 [LLM_MAP](LLM_MAP.md)，历史资料在 `doc/archive/`。当前 98 个 C++ 返航回归和 31 个 Python 日志测试通过；真实 ROS 日志管线联调通过。已有 Gazebo 理想外部定位的正常、航点降级和运动触发闭环记录；实际激光 SLAM 接入、完整场景和 RK3588/实飞仍待验收。

```bash
# Docker ROS_Noetic；trajectory_core 需使用匹配接口的修复版本，见统一文档
source /opt/ros/noetic/setup.bash
cd /root/ROS_Noetic/backtrack_ws
catkin_make --force-cmake --pkg trajectory_core px4_backtrack_optimised \
  -DCMAKE_BUILD_TYPE=Release \
  -DBACKTRACK_BUILD_REVISION="$(git -C src/px4_backtrack_optimised describe --always --dirty)" -j4
source devel/setup.bash
roslaunch px4_backtrack_optimised backtrack.launch
```

定位源和 MAVROS 由工程启动。参数入口为 `config/backtrack_params.yaml` 与 `config/trajectory_config.yaml`；更改后需按统一文档的预算复验。指定航线模块应直接消费 `trajectory_core`，不依赖本节点的状态机。
