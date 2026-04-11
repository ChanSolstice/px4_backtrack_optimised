# Bug Report: 低速连续飞行抽稀锚点覆盖导致返航路径截断

| 项目 | 内容 |
| --- | --- |
| 发现日期 | 2026-04-11 |
| 严重程度 | **High** — 返航轨迹被截断，实飞卡入 `ReturnRejoin` 后无法继续返航 |
| 关联用例 | TC-18 直线分三段飞行返航验证 |
| 影响范围 | `TrajectorySession` 生成前空间抽稀、`ReturnActive` 轨迹跟踪、`ReturnRejoin` 回归 |
| 根因文件 | `src/domain/trajectory_session.cpp` |
| 修复状态 | 已修复，待实飞复测 |

---

## 1. 现象描述

TC-18 使用直线分三段飞行并触发返航，参数中 `min_point_dist_m` 设置为 `0.08m`。

实飞表现：

- 返航到第二段附近后进入 `ReturnRejoin`
- 状态一直停在 `ReturnRejoin`
- 机体在原地附近抽动，无法继续完成返航
- 手动暂停或解除后仍容易再次触发 rejoin

关键日志：

```text
[TrajectorySession] Thinned recorded points: 455 -> 24 (min_spacing=0.080 m)
[TrajectorySession] Duration: 20.29 s
[TrajectorySession] Path length: 4.42 m
[BacktrackUseCase] Deviation detected for 0.25 s, entering rejoin mode
[StateMachine] State transition: ReturnActive -> ReturnRejoin
```

最后一次卡住前的状态变化：

```text
1775895419.223888303 Deviation detected for 0.25 s, entering rejoin mode
1775895419.223956844 State transition: ReturnActive -> ReturnRejoin
```

之后直到人工按键暂停前，日志中没有再次出现 `Rejoin successful`。

---

## 2. CSV 分析结论

本次 debug 文件：

- `waypoints_20260411_161632.csv`
- `trajectory_20260411_161632.csv`

原始录制数据：

```text
录制点数: 455
录制时长: 45.40 s
原始路径长度: 约 11.56 m
原始起点: x=0.051, y=-0.081, z=0.773
原始终点: x=8.636, y=0.456, z=0.776
```

异常抽稀后的数据：

```text
抽稀点数: 24
生成轨迹长度: 约 4.42 m
生成轨迹起点: x=8.636, y=0.456, z=0.776
生成轨迹终点: x=4.289, y=0.157, z=0.795
```

结论：

- 生成轨迹没有回到原始起点
- 原始路径前半段约 `0m -> 4.29m` 被抽稀逻辑删除
- 即使不触发 rejoin，这条轨迹也只能返到中途位置，不能完成完整返航

抽稀后还出现两个明显时间断层：

```text
7.035m -> 8.638m: dt = 11.3s, distance = 1.60m
5.051m -> 6.013m: dt = 9.8s,  distance = 1.01m
```

这说明中间连续移动段被误删，只剩下少数段尾点。

---

## 3. 根因分析

### 3.1 抽稀逻辑错误移动了距离锚点

修复前的核心逻辑：

```cpp
if (dist >= min_spacing_m) {
  reduced.push_back(point);
  continue;
}

reduced.back() = point;
```

这段代码的意图是压缩停顿点云团，但实际效果是：

1. 当前点距离上一个有效点不足阈值
2. 代码用当前点覆盖上一个有效点
3. 下一帧继续拿“刚覆盖过的新点”作为距离锚点
4. 低速连续飞行时，每帧位移都小于阈值，于是永远累计不到阈值
5. 整段低速路径最后只剩该段末尾一个点

对于 `record_max_hz` 约 `10Hz` 的数据，`min_point_dist_m=0.08m` 等价于要求单帧速度至少约 `0.8m/s` 才容易保留新点。

TC-18 中大量飞行段速度低于这个值，因此被误判为“过密点”，最终被覆盖吞掉。

### 3.2 `min_point_dist_m` 越大，截断风险越高

提高 `min_point_dist_m` 本来是为了压缩悬停漂移点云团，但在旧逻辑下它同时提高了“单帧必须跨过的距离”。

因此：

- `0.01m` 时问题不明显，但停顿点抽动更严重
- `0.08m` 时停顿点压缩增强，但低速段更容易被整段删除

这不是参数调优能彻底解决的问题，而是抽稀算法语义错误。

### 3.3 Rejoin 卡住是截断轨迹的后续表现

异常生成的轨迹在 `t=8.5s` 后速度已经低于 `0.1m/s`，末段几乎贴近停滞：

```text
t=16.0s 位置约 x=4.501, y=0.192, z=0.796，速度约 0.013m/s
t=20.3s 位置约 x=4.289, y=0.157, z=0.795，速度约 0.000m/s
```

最后一次进入 `ReturnRejoin` 时，轨迹进度约在 `t=15.96s`，正处于这个极慢尾段。由于轨迹本身已经被截断且目标变化极小，实际控制表现为原地抽动，并且很难重新满足回归成功条件。

---

## 4. 修复方案

### 4.1 距离不足时忽略当前中间点

修复后的抽稀语义：

```cpp
reduced.push_back(points.front());

for (size_t i = 1; i + 1 < points.size(); ++i) {
  const auto& point = points[i];
  double dist = (point.position - reduced.back().position).norm();

  if (dist >= min_spacing_m) {
    reduced.push_back(point);
  }
}
```

关键变化：

- 上一个有效点作为累计距离锚点保持不动
- 中间点距离不足时直接忽略
- 低速连续飞行时，距离会从上一个有效点持续累计
- 累计达到阈值后自然保留新的点
- 悬停点云团仍会被压缩，因为位置不会累计离开阈值范围

### 4.2 继续保留首点和末点

首点和末点仍保持强制保留：

```cpp
if ((points.back().position - reduced.back().position).norm() > 1e-9) {
  reduced.push_back(points.back());
} else {
  reduced.back() = points.back();
}
```

这个末点覆盖只发生在终点与最后有效点几乎重合时，用于保留最终时间戳、yaw 和精确终点，不会影响中间路径累计。

---

## 5. 验证结果

### 5.1 使用 TC-18 CSV 重放抽稀逻辑

修复后对同一份 `waypoints_20260411_161632.csv` 重放抽稀：

```text
raw_count: 455
fixed_reduced_count: 104
raw_path_length_m: 11.557
fixed_reduced_path_length_m: 10.837
first_reduced: x=0.051, y=-0.081, z=0.773
last_reduced:  x=8.636, y=0.456, z=0.776
```

结论：

- 抽稀后仍覆盖完整起终点
- 路径长度接近原始路径
- 不再出现 `455 -> 24` 这种大规模截断

### 5.2 单元测试

新增测试：

- `tests/unit/test_trajectory_session.cpp`
- `SlowStraightLineKeepsAccumulatedDistanceWhenThinning`

测试构造：

- 每帧位移 `0.02m`
- 总长度 `1.0m`
- `min_point_dist_m = 0.08m`

验证目标：

- 每帧位移虽然小于阈值，但累计距离应保留主路径
- 生成轨迹长度应保持在 `0.8m ~ 1.2m`

---

## 6. 经验总结

生成前抽稀必须区分两类场景：

- **悬停点云团**：位置长期停留在小范围内，应压缩
- **低速连续飞行**：单帧位移小，但累计位移持续增长，应保留

判断依据不能是“当前点和上一帧点是否足够远”，而应是“当前点和上一个有效锚点是否足够远”。

距离不足时覆盖锚点会破坏累计距离语义，是本次 TC-18 返航路径截断的直接原因。
