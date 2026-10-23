# 闭环仿真与 SITL 测试

本文档说明单元测试之外的三类测试：闭环危害场景、PX4 SITL 飞控行为检查和规划耗时基准，包括用途、运行方式和执行记录。单元测试与错误注入见[单元测试文档](单元测试文档.md)。

## 1. 测试层次

| 层次 | 测什么 | 工具 | 说明 |
| --- | --- | --- | --- |
| 单元测试 | 条件层、状态机、执行层和 UseCase 接线；状态机安全约束的穷举 | `tests/unit/`，`scripts/docker/run_tests.sh` | [单元测试文档](单元测试文档.md) |
| 错误注入 | 单测能否发现典型错误 | `scripts/analysis/run_mutation_check.sh` | 单元测试文档第 8 节 |
| 闭环危害场景 | 真实 `BacktrackUseCase` 在按 PX4 源码写的假飞控下的整体行为 | `scripts/analysis/run_hazard_scenarios.sh` | 第 2 节 |
| SITL 飞控行为检查 | 重构方案依赖的 PX4 行为（N1、N2） | `scripts/sitl/run_sitl_checks.sh` | 第 3 节 |
| SITL 返航场景 | 真实 PX4（SIH）和 MAVROS 下本节点的整体行为（方案第 11 节中涉及飞控交互的场景） | `scripts/sitl/run_sitl_checks.sh scenarios` | 第 4 节 |
| 规划耗时基准 | 轨迹生成耗时随录制时长的增长 | `scripts/bench/run_gen_bench.sh` | 第 5 节 |
| 台架、实飞 | 方案第 11 节的场景 | 清单按方案第 14.4、14.5 节编写，R8 定稿 | dev 版本的测试文档见[返航功能测试文档](返航功能测试文档.md)、[当前版本返航飞行测试清单](当前版本返航飞行测试清单.md) |

所有脚本都在 Docker 中运行，本机需要 sudo 时设置 `DOCKER="sudo docker"`。闭环场景、注入检查和基准复用 `scripts/docker/run_tests.sh` 留下的容器 `backtrack_noetic_<arch>`，先跑一次单测即可。

## 2. 闭环危害场景

**做法：** [`scripts/analysis/hazard_scenarios.cpp`](../../scripts/analysis/hazard_scenarios.cpp) 把真实的 `BacktrackUseCase` 放进按 PX4 v1.15.4 源码写的假飞控和接收机模型里闭环运行：模式回退与恢复、开关通道、摇杆超控、落地判定、自动上锁、心跳频率等。控制循环的阻塞按真实耗时计入仿真时间；规划使用真实的后台线程，规划进行期间按墙钟推进仿真时间，结果到达的时刻与实际耗时一致（R4 起）。模型规则、出处和局限见[系统性分析](../reviews/返航程序系统性分析_2026-10-20.md)第 1 节。

**场景：** H01–H16（含 H06a、H06b）共 17 个，每个场景有 1–3 项检查，共 33 项（R4 给 H07 增加了"规划期间 setpoint 不中断"；R5b 新增 H15 的 2 项；R5c 新增 H16 的 3 项；R6 给 H15 增加 1 项）。H15 是低电量返航中链路时断时续，飞手拨开关切出并向北推杆约 5 s，同时 MAVROS 的模式服务暂时不可用（71～78 s），模块拿不回 OFFBOARD；检查服务恢复、模块重新拿到 OFFBOARD 后原地保持（之后水平移动不超过 1 m），并告警 `PATH_OFFSET`（D30）；重新申请 10 s 超时、电量低，原地降落（R6）。H16 是往返航线（去程向东、回程在去程以北约 1.2 m），低电量返航沿回程向东时链路时断时续，飞手切出并把飞机向南推到去程附近；检查拿回 OFFBOARD 后找点接上（D33），之后在 OFFBOARD 中离录制轨迹不超过安全距离、没有沿回程继续向东，最后在录制起点上锁。场景定义与 dev 的结果见系统性分析第 5 节，通过标准见第 10 节。

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
| 2026-10-22 | R5（`1b63dc7`） | 27 项全部通过 | 与 R4 相同；现有场景都不包含"退出后再失控"，R5 的新任务路径由单测 S11 覆盖 |
| 2026-10-22 | R5b（`df95ab5`） | 29 项全部通过 | H15：81.5 s 拿回 OFFBOARD，之后移动 0.00 m，出现 `PATH_OFFSET`。用 D30 之前的保持逻辑运行 H15 时，先直线飞回切出点约 7 m，再继续返航，之后离该位置最远 28.29 m，检查失败 |
| 2026-10-23 | 参数调整与 D31（`a8b6738`） | 29 项全部通过 | 轨迹参数改为 v_max 0.6、vz_max 0.4 等后返航变慢，H01 的返航段由 37.3 s 变为 57.6 s，仍在场景时长内完成；场景中 RC 中断确认同步为 0.35 s |
| 2026-10-23 | 安全距离改为 0.7 m（`6dc2462`） | 29 项全部通过 | 只有 H15 出现 `PATH_OFFSET`，没有场景进入 `ReturnRejoin` |
| 2026-10-23 | R5c 找点接上（`9b524ef`） | 32 项全部通过 | H16：64.5 s 拿回 OFFBOARD 并找点接上，之后在 OFFBOARD 中离录制轨迹最远 0.24 m，在距录制起点 0.2 m 处上锁。把找点接上的两个条件置为假再运行，飞机原地悬停在距起点 7.6 m 处，H16 三项都失败。H07 最长 tick 2 ms：30 分钟录制约 2.7 万个点，每拍找点的查找约 30 µs，不是主要开销 |
| 2026-10-23 | R6 超时升级（`40981ef`） | 33 项全部通过 | 只有 H15 的行为变化：它是低电量任务，原来 71～81 s 的模式服务中断会在拿回 OFFBOARD 之前就重新申请超时降落。中断改为 71～78 s 后：78.5 s 拿回 OFFBOARD，离路径超过安全距离、找不到点，原地保持（水平移动 0.00 m）；81.5 s 重新申请超时，原地降落并上锁（新增检查）。其余场景的检查结果不变 |

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

## 4. SITL 返航场景

**目的：** 在真实的 PX4 v1.15.4（SIH）和 MAVROS 下运行返航节点，回放方案第 11 节中涉及飞控交互的场景（方案第 14.3 节，R-86）。与第 2 节的闭环危害场景相比，这里的飞控模式逻辑、MAVROS 的心跳频率和服务调用、节点的话题和参数加载都是真实的。

**做法：** 同一个容器（第 3 节的镜像，加装编译本节点的依赖）里运行 PX4 SITL、MAVROS 和本节点：

- 节点用机载的参数文件，只打开调试日志（[`backtrack_sitl.launch`](../../scripts/sitl/backtrack_sitl.launch)）；里程计用 `/mavros/local_position/odom`，遥控和电量话题重映射到场景脚本。
- [`sitl_scenarios.py`](../../scripts/sitl/sitl_scenarios.py) 扮演飞手、接收机和电池：
  - 飞手用 `MANUAL_CONTROL` 摇杆（`COM_RC_IN_MODE=1`）在 POSCTL 下起飞到约 3 m，录制开始后向前约 10 m、向右约 8 m；
  - 接收机以 50 Hz 发 18 个通道，失控时 CH18 = 800、摇杆回中、CH7 保持失控瞬间的值；
  - 飞手拨开关、地面站切模式用 `/mavros/set_mode` 模拟，本模块卡顿用 `SIGSTOP`、`SIGCONT`。
- 每个场景重新启动 PX4、MAVROS 和节点；判定看节点的状态话题、`/mavros/state` 和调试日志 `events.jsonl`。

**场景：**

| 场景 | 内容 | 检查 |
| --- | --- | --- |
| S1 | 低电量返航中失控 | 不退出；链路丢失后不请求 POSCTL；在返航终点附近降落并上锁 |
| S4 | 录制中失控 | 第一帧失控帧后 1.5 s 内申请 OFFBOARD；悬停 15 s 后返航；在返航终点附近降落 |
| S6 | 返航中短按 CH7 | 进入 `Exited`、截断录制、飞控进入 POSCTL；之后不再申请 OFFBOARD |
| S7 | 返航中链路正常时拨开关 | 进入 `Exited`；之后不再申请 OFFBOARD |
| S11 | 长按返航，短按退出，手动飞一段后失控 | 开始新任务；经过退出位置；在返航终点附近降落 |
| S12 | 返航中失控后地面站切出 | 重新申请并回到 OFFBOARD；不退出；在返航终点附近降落 |
| S13 | 在地面解锁、未起飞时失控、低电量 | 不启动任务、不请求模式、不起飞；告警 `TRIGGER_IGNORED_LANDED` |
| S15 | 长录制（默认 5 分钟）后失控 | 后台规划完成；OFFBOARD 不因断流回退 |
| S18 | 降落交接中飞手拨开关 | 模块看到飞手的模式后不再请求 AUTO.LAND，飞机留在飞手的模式 |
| S21 | 链路丢失时飞控降落中被切到 POSCTL | 重新请求 AUTO.LAND，告警 `AUTOLAND_OVERRIDDEN`；降落并上锁 |
| S26 | 返航中本节点卡顿 1.3 s | 不退出；在返航终点附近降落 |
| S27 | 返航开始、OFFBOARD 确认前拨开关到 ALTCTL | 进入 `Exited`；没有申请 OFFBOARD |

返航的场景还记录跟随中离返航轨迹的最大距离（只记录，见下文）。

**运行：**

```bash
DOCKER="sudo docker" scripts/sitl/run_sitl_checks.sh build              # 首次：编译 PX4 SITL
DOCKER="sudo docker" scripts/sitl/run_sitl_checks.sh scenarios          # 全部场景
DOCKER="sudo docker" scripts/sitl/run_sitl_checks.sh scenarios S4 S12   # 指定场景
```

每个场景约 1.5～2.5 分钟（含 PX4 启动、估计器对准、解锁后 15 s 的录制延时），全部约 27 分钟。S15 的录制时长用环境变量 `S15_MINUTES` 设置。日志在容器的 `/tmp/sitl/<场景>/`：PX4、ROS 的输出，节点的调试日志，以及场景脚本记录的位置、setpoint 和离返航轨迹的距离（`track.csv`、`path.csv`）。

**SIH 的位置保持误差：** OFFBOARD 中 SIH 四旋翼的位置保持误差平均约 0.3 m、最大约 0.7 m，真值与估计值都在晃动，周期约 10 s。这与本模块无关：只发位置 setpoint 时一样，把模拟 GPS 的噪声减小到十分之一也没有改善；同一次飞行中 POSCTL 悬停的误差在 0.1 m 以内。这个误差接近回归阈值（0.5 m）和安全距离（0.7 m），所以 SITL 中跟随时常有回归，偶尔原地保持（`PATH_OFFSET`），飞机晃回安全距离以内后继续。因此"离返航轨迹的最大距离"只作记录，降落位置按 1.5 m 判定。PX4 SIH 偶尔在起飞后传感器超时（`px4.log` 中出现 "No valid data from"），飞控进入失效保护、拒绝 OFFBOARD，这次场景判为无效，脚本自动重跑一次。机载用 SLAM 定位，悬停误差应该小得多；实飞时要记录跟随误差，确认这两个阈值有足够余量（方案第 14.5 节）。

**执行记录：**

| 日期 | 受测代码 | 结果 | 说明 |
| --- | --- | --- | --- |
| 2026-10-23 | `ecf3ed9`（节点代码同 `40981ef`） | 12 个场景全部通过 | 要点见下 |

- S1 首次运行时，SIH 在起飞后约 5 s 传感器超时（PX4 报 "No valid data from Baro 0/Compass 0"、"Failsafe: blind land"，自行切到 ALTCTL），之后拒绝 OFFBOARD；节点停在准备阶段，每 1 s 重发申请，符合设计。重跑通过：链路丢失后没有请求 POSCTL，交接直接 AUTO.LAND。
- S4：第一帧失控帧后 1.33 s 申请 OFFBOARD，悬停 15.00 s。
- S7：拨开关后 0.05 s 进入 `Exited`。飞控心跳固定 1 Hz，这次恰好紧接着发出；各场景中从请求模式到看到新模式为 0.05～1.05 s，与第 3 节一致。
- S11：退出后失控开始第二个任务，返航经过退出位置（最近 0.12 m）。
- S12：被地面站切出后 1.9 s 回到 OFFBOARD，没有退出。
- S15：5 分钟录制 3080 个点，后台规划 87 ms；悬停和返航开始阶段 OFFBOARD 没有回退、没有断流。
- S26：节点冻结 1.3 s，ULog 中 OFFBOARD 回退到 POSCTL，节点恢复发布后 0.26 s 自动回到 OFFBOARD。回退只有零点几秒，1 Hz 的飞控状态没有看到，节点也不需要处理（D22），没有当作外部切出重新申请。
- 降落位置距返航终点 0.54～0.79 m；跟随中离返航轨迹最远 0.46～1.32 m，有的场景出现回归和 `PATH_OFFSET` 原地保持，原因是上面说的 SIH 位置保持误差。

## 5. 规划耗时基准

[`scripts/bench/gen_bench.cpp`](../../scripts/bench/gen_bench.cpp) 用固定随机种子合成出航路径，经 `RecordingBuffer` 录制后调用与 dev 相同的轨迹生成，测量耗时随录制时长的增长。输入构造和完整数据见方案附录 A。

```bash
DOCKER="sudo docker" scripts/bench/run_gen_bench.sh amd64 1 5 10 30    # 参数为录制分钟数
```

| 日期 | 平台 | 10 分钟录制 | 30 分钟录制 |
| --- | --- | --- | --- |
| 2026-10-19 | 本开发机 amd64（Docker ROS Noetic） | 0.61 s | 4.8 s |

机载 RK3588 的耗时待实测（方案第 14.2 节），用来确定规划超时参数。
