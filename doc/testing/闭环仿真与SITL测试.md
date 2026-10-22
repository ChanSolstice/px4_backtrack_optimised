# 闭环仿真与 SITL 测试

本文档说明单元测试之外的三类测试：闭环危害场景、PX4 SITL 飞控行为检查和规划耗时基准，包括用途、运行方式和执行记录。单元测试与错误注入见[单元测试文档](单元测试文档.md)。

## 1. 测试层次

| 层次 | 测什么 | 工具 | 说明 |
| --- | --- | --- | --- |
| 单元测试 | 条件层、状态机、执行层和 UseCase 接线；状态机安全约束的穷举 | `tests/unit/`，`scripts/docker/run_tests.sh` | [单元测试文档](单元测试文档.md) |
| 错误注入 | 单测能否发现典型错误 | `scripts/analysis/run_mutation_check.sh` | 单元测试文档第 8 节 |
| 闭环危害场景 | 真实 `BacktrackUseCase` 在按 PX4 源码写的假飞控下的整体行为 | `scripts/analysis/run_hazard_scenarios.sh` | 第 2 节 |
| SITL 飞控行为检查 | 重构方案依赖的 PX4 行为（N1、N2） | `scripts/sitl/run_sitl_checks.sh` | 第 3 节 |
| 规划耗时基准 | 轨迹生成耗时随录制时长的增长 | `scripts/bench/run_gen_bench.sh` | 第 4 节 |
| 完整 SITL、台架、实飞 | 方案第 11 节的场景 | R6 完成后做完整 SITL；清单按方案第 14.3–14.5 节编写，R8 定稿 | dev 版本的测试文档见[返航功能测试文档](返航功能测试文档.md)、[当前版本返航飞行测试清单](当前版本返航飞行测试清单.md) |

所有脚本都在 Docker 中运行，本机需要 sudo 时设置 `DOCKER="sudo docker"`。闭环场景、注入检查和基准复用 `scripts/docker/run_tests.sh` 留下的容器 `backtrack_noetic_<arch>`，先跑一次单测即可。

## 2. 闭环危害场景

**做法：** [`scripts/analysis/hazard_scenarios.cpp`](../../scripts/analysis/hazard_scenarios.cpp) 把真实的 `BacktrackUseCase` 放进按 PX4 v1.15.4 源码写的假飞控和接收机模型里闭环运行：模式回退与恢复、开关通道、摇杆超控、落地判定、自动上锁、心跳频率等。控制循环的阻塞按真实耗时计入仿真时间；规划使用真实的后台线程，规划进行期间按墙钟推进仿真时间，结果到达的时刻与实际耗时一致（R4 起）。模型规则、出处和局限见[系统性分析](../reviews/返航程序系统性分析_2026-10-20.md)第 1 节。

**场景：** H01–H14（含 H06a、H06b）共 15 个，每个场景有 1–3 项检查，共 27 项（R4 给 H07 增加了"规划期间 setpoint 不中断"，此前为 26 项）。场景定义与 dev 的结果见系统性分析第 5 节，通过标准见第 10 节。

**运行：**

```bash
DOCKER="sudo docker" scripts/docker/run_tests.sh amd64      # 先编译
DOCKER="sudo docker" scripts/analysis/run_hazard_scenarios.sh amd64 [场景编号 ...]
```

**执行记录：**

| 日期 | 受测代码 | 结果 | 说明 |
| --- | --- | --- | --- |
| 2026-10-20 | dev（重构前） | 26 项中 17 项失败 | 作为反例对照；H13 通过，因为 dev 不处理被切出 |
| 2026-10-21 | R3（`7c17473`） | 26 项全部通过 | 要点见下 |
| 2026-10-22 | R4（`24620be`） | 27 项全部通过 | H07 的变化见下；其余场景与 R3 相同 |

R3 起场景驱动新实现：按新接口送入里程计、RC、飞控状态、落地状态和电量，模式请求由 `ModeCommander` 决定，服务结果按"已发出"回报。通过标准为全部通过。R3 结果的要点：

- H01：首个失控帧后 1.4 s 进入 OFFBOARD，悬停 15 s 后返航，在距录制起点 0.2 m 处上锁。
- 链路正常的任务（H02、H13、H14）等到预发结束后的下一条飞控状态才申请 OFFBOARD（R-75），在任务开始后约 1.1 s 进入 OFFBOARD。R-75 是 R3 首次运行这些场景时由 H14 发现的：飞手在任务开始 0.2 s 后拨开关，模块预发结束就申请 OFFBOARD，覆盖了飞手的选择。
- H07：规划在任务开始的那一拍同步执行，阻塞 4.7 s，从首个失控帧到进入 OFFBOARD 约 6.1 s（H01 为 1.4 s）。阻塞发生在进入 OFFBOARD 之前，没有引起回退，所以检查通过。R4 改为后台规划后，H07 还要满足 setpoint 最大间隔 < 0.2 s（方案第 13 节），届时补上这项检查。
- H13：卡顿引起的回退在 0.3 s 后自动回到 OFFBOARD，任务继续，在录制起点上锁。
- H14：飞手拨开关后进入 `Exited`，之后模块没有再请求 OFFBOARD 或 AUTO.LAND。

报告中模块状态的时刻是该拍结束的时刻：阻塞的那一拍，状态变化记在阻塞结束时（H07 的 `Ch18HoverWait` 记为 1815.8 s，任务实际在约 1811.1 s 开始）。

R4 改为后台规划后，H07 的任务在 1811.1 s 开始，1811.4 s 进入 OFFBOARD，从首个失控帧算 1.4 s，与 H01 相同；最长 tick 1 ms（R3 为 4703 ms），任务中 setpoint 最大间隔 0.05 s（新增检查，阈值 0.2 s），没有回退。规划在 15 s 悬停内完成，悬停照常持续到 1826.1 s。

## 3. SITL 飞控行为检查

**目的：** 在真实的 PX4 中确认重构方案依赖的两条飞控行为：

- N1：本模块断流超过 `COM_OF_LOSS_T` 时飞控回退 POSCTL，setpoint 恢复后自动回到 OFFBOARD（D22 的前提）；
- N2：触地后摇杆回中，或 OFFBOARD 原地保持时，飞控不判定落地、不自动上锁（D23 的前提）。对照：油门保持最低时判定落地并自动上锁。

**环境：** PX4 v1.15.4 SITL（SIH 四旋翼，无 Gazebo）+ ROS Noetic MAVROS，都在 Docker 中（[`scripts/sitl/Dockerfile`](../../scripts/sitl/Dockerfile)）。PX4 从同级的 PX4-Autopilot 仓库克隆，只拉 SITL 需要的子模块，编译结果放在 `~/.cache/backtrack_px4_sitl`（属主为 root，删除需要 sudo）。

**做法：** [`check_fcu_behavior.py`](../../scripts/sitl/check_fcu_behavior.py) 用 `MANUAL_CONTROL` 模拟摇杆（`COM_RC_IN_MODE=1`），摇杆回中对应机载失控帧的输出：

1. OFFBOARD 起飞，悬停中停发 setpoint 3.5 s，观察回退；只恢复发布、不请求模式，观察是否回到 OFFBOARD（N1）；
2. POSCTL 油门最低下降，下降速度消失（触地）时立即回中，观察 5 s（N2a）；
3. 切到 OFFBOARD 保持当前位置，观察 5 s（N2b）；
4. 对照：POSCTL 油门保持最低，应判定落地并自动上锁。

飞行结束后读 ULog：飞控侧 N1 的精确时间，以及 N2a、N2b 期间飞机按 SIH 真值是否确实在地面。

**运行：**

```bash
DOCKER="sudo docker" scripts/sitl/run_sitl_checks.sh          # 首次：克隆、编译 PX4，然后检查（编译耗时较长）
DOCKER="sudo docker" scripts/sitl/run_sitl_checks.sh check    # 之后只跑检查，约 3 分钟
```

退出码：0 全部符合；1 与预期不符；2 起飞等准备失败；3 本次 N2 无效，重跑即可。

**SIH 的局限：** 触地后测距不再参与高度融合，估计高度在地面以约 0.1–0.3 m/s 漂移；回中或原地保持时高度控制跟着估计走，有时把飞机带离地面。所以 N2a、N2b 各段要求按真值连续在地面不少于 3 s（落地检测约需 1 s），否则判为无效（退出码 3）。离地高度不能用估计值或 `/mavros/altitude` 的 `bottom_clearance` 判断，它们在地面也会漂移。

**执行记录（2026-10-20/21，`scripts/sitl` 提交 `433887c` 前后的 7 次有 ULog 的运行）：**

| 项目 | 结果 |
| --- | --- |
| N1 | 7/7 符合：最后一个 setpoint 后 1.02–1.11 s 回退 POSCTL，回退期间保持高度；恢复发布后 0.02 s 回到 OFFBOARD，期间没有请求模式 |
| N2a | 6 次有效（在地面 4.3–10.9 s），都没有判定落地、一直解锁；1 次无效 |
| N2b | 3 次有效（在地面 4.7–10.2 s），都没有判定落地、一直解锁；4 次无效 |
| 对照 | 每次都判定落地并在约 2 s 后自动上锁；从地面开始时约 3 s 判定落地 |

没有出现反例。结论与分析见系统性分析第 6 节 N1、N2；`/mavros/state` 约 1 Hz，模块看到模式变化最多晚约 1 s，在方案的回退窗口（3 s）之内。

## 4. 规划耗时基准

[`scripts/bench/gen_bench.cpp`](../../scripts/bench/gen_bench.cpp) 用固定随机种子合成出航路径，经 `RecordingBuffer` 录制后调用与 dev 相同的轨迹生成，测量耗时随录制时长的增长。输入构造和完整数据见方案附录 A。

```bash
DOCKER="sudo docker" scripts/bench/run_gen_bench.sh amd64 1 5 10 30    # 参数为录制分钟数
```

| 日期 | 平台 | 10 分钟录制 | 30 分钟录制 |
| --- | --- | --- | --- |
| 2026-10-19 | 本开发机 amd64（Docker ROS Noetic） | 0.61 s | 4.8 s |

机载 RK3588 的耗时待实测（方案第 14.2 节），用来确定规划超时参数。
