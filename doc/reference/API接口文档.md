# trajectory_core API 接口文档

本文档详细说明如何在您的程序中集成和使用 `trajectory_core` 轨迹生成库。

## 目录

- [快速开始](#快速开始)
- [核心数据结构](#核心数据结构)
- [主要接口类](#主要接口类)
- [完整使用流程](#完整使用流程)
- [错误处理](#错误处理)
- [高级用法](#高级用法)
- [完整示例代码](#完整示例代码)
- [常见问题](#常见问题)

---

## 快速开始

### 最小化示例

```cpp
#include <trajectory_core/trajectory_core.hpp>

int main() {
    // 1. 准备输入路径点
    std::vector<trajectory_core::Waypoint> waypoints;
    waypoints.emplace_back(0, 0, 5, 0, 0.0);           // x, y, z, yaw, t
    waypoints.emplace_back(10, 0, 5, 0, 5.0);
    waypoints.emplace_back(10, 10, 5, M_PI/2, 10.0);

    // 2. 创建生成器和配置
    trajectory_core::TrajectoryGenerator generator;
    trajectory_core::TrajectoryConfig config;  // 使用默认配置

    // 3. 生成轨迹
    auto result = generator.generate(waypoints, config);

    // 4. 检查结果并使用
    if (result.isSuccess()) {
        auto states = result.traj->sample(0.05);  // 20Hz 采样
        for (const auto& state : states) {
            // 使用 state.position, state.velocity, state.yaw, state.yaw_rate
        }
    } else {
        std::cerr << "Generation failed: " << result.message << std::endl;
    }

    return 0;
}
```

### 编译配置

#### CMakeLists.txt 示例

```cmake
find_package(catkin REQUIRED COMPONENTS
    trajectory_core
)

catkin_package(
    CATKIN_DEPENDS trajectory_core
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

#### package.xml 依赖

```xml
<build_depend>trajectory_core</build_depend>
<exec_depend>trajectory_core</exec_depend>
```

---

## 核心数据结构

### Waypoint - 输入路径点

```cpp
namespace trajectory_core {
    struct Waypoint {
        Vec3 position;      // 位置 (m), ENU 坐标系
        double yaw;         // 航向角 (rad), 绕 +Z 轴，逆时针为正
        double timestamp;   // 时间戳 (s), 可以是绝对或相对时间
    };
}
```

**构造方法**:
```cpp
// 方法 1: 使用 Vec3
Eigen::Vector3d pos(10.0, 20.0, 5.0);
Waypoint wp(pos, 0.5, 1.0);

// 方法 2: 直接指定坐标
Waypoint wp(10.0, 20.0, 5.0, 0.5, 1.0);

// 方法 3: 使用 emplace_back
std::vector<Waypoint> waypoints;
waypoints.emplace_back(10.0, 20.0, 5.0, 0.5, 1.0);
```

**坐标系说明**:
- **ENU (East-North-Up)**: X 轴指向东，Y 轴指向北，Z 轴指向上
- **Yaw**: 从 +X 轴开始测量，绕 +Z 轴逆时针旋转为正
- **单位**: 位置单位为米，yaw 单位为弧度，时间戳单位为秒

**时间戳要求**:
- 应为单调递增序列
- 可以是绝对时间或相对时间（内部会转为相对时间）
- 如果时间戳异常（非单调、大间隙），会触发回退模式

> 说明：当前版本输出轨迹的时间轴主要由内部“速度剖面 + 时间积分”生成，`timestamp` 主要用于时间质量检查与回退标志（`TimeReport/StatusCode`），不保证严格复现录制数据的原始时序。

### TrajectoryConfig - 配置参数

```cpp
struct TrajectoryConfig {
    // 采样与拟合
    double ds_m = 0.15;                     // 弧长重采样间距 (m)
    double max_deviation_m = 0.15;          // 走廊约束半径 (m)
    double dt_sample_s = 0.05;              // 内部时间采样步长 (s)，默认 20Hz（不等同于输出采样频率）
    double min_point_dist_m = 0.01;         // 去重点最小距离 (m)
    double max_dt_gap_s = 0.5;              // 时间间隙异常阈值 (s)

    // 拐角处理
    double corner_angle_threshold_deg = 135.0;  // 拐角检测角度阈值 (deg)：角度 >= 阈值判定为拐角
    double corner_blend_len_m = 0.3;            // 拐角过渡长度 (m)
    double corner_blend_ratio = 0.4;            // 拐角混合比例（安全系数，避免跨越相邻段）
    double corner_tangent_factor = 1.0;         // Hermite 拐角过渡切向量系数

    // 走廊检查
    double ds_check_m = 0.075;              // 走廊检查采样间距 (m)，通常为 ds_m / 2

    // Yaw 处理
    bool enable_yaw_smooth = true;              // 启用 yaw 平滑
    double max_yaw_deviation_rad = 0.05;        // 最大 yaw 偏差 (rad)
    double yaw_rate_max = 0.6;                  // 最大 yaw 角速度 (rad/s)
    double max_expected_error_rad = 0.3;        // 期望最大 yaw 误差 (rad)，用于计算控制增益
    double k_yaw_rate_limiter = 2.0;            // yaw rate limiter 增益（通常 yaw_rate_max / max_expected_error_rad）
    bool enable_yaw_rate_constraint_on_velocity = true;  // yaw 约束影响速度
    double v_yaw_min_warn = 0.3;                // yaw 约束导致的速度过低告警阈值 (m/s)

    // 末端收敛
    bool enable_end_converge = true;            // 启用末端收敛
    double s_stop_m = 3.0;                      // 收敛触发弧长 (m)
    double r_stop_m = 2.0;                      // 收敛触发距离 (m)
    double v_end_max = 0.8;                     // 收敛段最大速度 (m/s)
    double r_yaw_hold_m = 1.0;                  // Yaw 保持距离 (m)
    bool end_mode_latch = true;                 // 收敛锁定：进入收敛后不再允许加速退出

    // 速度/加速度约束
    double v_max = 2.0;                         // 最大水平速度 (m/s)
    double vz_max = 0.8;                        // 最大竖直速度 (m/s)
    double a_tan_max = 1.0;                     // 最大切向加速度 (m/s²)
    double a_lat_max = 1.0;                     // 最大横向加速度 (m/s²)
    double az_up_max = 1.0;                     // 最大上升加速度 (m/s²)
    double az_down_max = 1.0;                   // 最大下降加速度 (m/s²)

    // 起止速度
    double v_start_m_s = 0.0;                   // 起始速度 (m/s)
    double v_end_m_s = 0.0;                     // 终止速度 (m/s)

    // 输出选项
    bool enable_output_acc = false;             // 启用加速度输出
    double v_time_eps = 0.05;                   // 时间积分最小速度 (m/s)，防止除零

    // 方法
    bool validate() const;                      // 验证配置有效性
    double cornerAngleThresholdRad() const;     // 获取拐角阈值 (rad)
    static TrajectoryConfig loadFromYaml(const std::string& path);  // 从 YAML 加载
    void saveToYaml(const std::string& path) const;                 // 保存到 YAML
};
```

**重要说明**：
- `dt_sample_s` 是内部过程使用的采样步长（例如 yaw rate limiter 的离散步长兜底），与 `Trajectory::sample(dt)` 的输出采样频率无直接绑定关系。
- `loadFromYaml()/saveToYaml()` 当前实现为”简易 YAML”（一行一个 `key: value`，可带 `#` 注释），不支持嵌套结构/数组；未知 key 会被忽略；未出现的字段保持默认值。
- **YAML 读写仅覆盖以下字段的子集**，其余字段 round-trip 后会静默回退到 C++ 默认值：
  - 已覆盖（可被 YAML 读写）：`ds_m`、`max_deviation_m`、`dt_sample_s`、`min_point_dist_m`、`max_dt_gap_s`、`corner_angle_threshold_deg`、`corner_blend_len_m`、`corner_blend_ratio`、`enable_yaw_smooth`、`max_yaw_deviation_rad`、`yaw_rate_max`、`enable_yaw_rate_constraint_on_velocity`、`enable_end_converge`、`s_stop_m`、`r_stop_m`、`v_end_max`、`r_yaw_hold_m`、`v_max`、`vz_max`、`a_tan_max`、`a_lat_max`、`az_up_max`、`az_down_max`、`v_start_m_s`、`v_end_m_s`、`enable_output_acc`
  - **未覆盖（不会被 YAML 读写，始终使用 C++ 默认值）**：`corner_tangent_factor`、`ds_check_m`、`max_expected_error_rad`、`k_yaw_rate_limiter`、`v_yaw_min_warn`、`end_mode_latch`、`v_time_eps`
  - 如果需要持久化上述未覆盖字段，请通过代码直接赋值。

**使用示例**:
```cpp
// 使用默认配置
TrajectoryConfig config;

// 修改部分参数
config.v_max = 3.0;
config.yaw_rate_max = 0.8;
config.enable_end_converge = true;

// 验证配置
if (!config.validate()) {
    std::cerr << "Invalid configuration!" << std::endl;
    return;
}

// 从文件加载
auto config = TrajectoryConfig::loadFromYaml("config.yaml");

// 保存到文件
config.saveToYaml("output_config.yaml");
```

详细参数说明请参考 [参数配置指南](../../../trajectory_core/doc/reference/参数配置指南.md)。

### TrajectoryState - 轨迹状态

```cpp
struct TrajectoryState {
    double t;                           // 时间 (s)
    Vec3 position;                      // 位置 (m)
    Vec3 velocity;                      // 速度 (m/s)
    std::optional<Vec3> acceleration;   // 加速度 (m/s²), 可选
    double yaw;                         // Yaw 角 (rad), [-π, π]
    double yaw_rate;                    // Yaw 角速度 (rad/s)

    // 辅助方法
    double speed() const;               // 速度大小
    double speedHorizontal() const;     // 水平速度
    double speedVertical() const;       // 竖直速度
};
```

> 说明：`acceleration` 只有在生成时设置 `config.enable_output_acc = true` 才会输出，否则为 `std::nullopt`。

**使用示例**:
```cpp
TrajectoryState state = trajectory->evaluate(1.5);  // 在 t=1.5s 处采样

std::cout << "Position: " << state.position.transpose() << std::endl;
std::cout << "Velocity: " << state.velocity.transpose() << std::endl;
std::cout << "Speed: " << state.speed() << " m/s" << std::endl;
std::cout << "Yaw: " << state.yaw << " rad" << std::endl;
std::cout << "Yaw rate: " << state.yaw_rate << " rad/s" << std::endl;

if (state.acceleration.has_value()) {
    std::cout << "Acceleration: " << state.acceleration->transpose() << std::endl;
}
```

### Result - 生成结果

```cpp
struct Result {
    StatusCode code;                    // 主结果状态码
    std::unique_ptr<Trajectory> traj;   // 生成的轨迹（失败时为 nullptr）
    Report report;                      // 详细报告
    std::string message;                // 人类可读的消息
    uint32_t warning_flags;             // 非致命告警位域

    // 方法
    bool isSuccess() const;             // 主结果是否成功
    bool isUsable() const;              // 轨迹是否可用
    bool hasTimeFallback() const;       // 是否触发时间戳回退 warning
    bool isCorridorExceeded() const;    // 是否触发走廊超限 warning
};
```

**状态码 (StatusCode)**:
```cpp
enum class StatusCode {
    SUCCESS,                    // 成功生成轨迹
    INVALID_INPUT,              // 输入无效（点数不足、配置非法等）
    FIT_FAILED,                 // 几何插值/内部拟合步骤失败
    PROFILE_INFEASIBLE,         // 无法在速度/加速度约束下生成速度剖面
    INTERNAL_ERROR              // 内部错误
};

enum class ResultWarning {
    NONE = 0,
    TIME_FALLBACK = 1,          // 时间戳异常，回退到内部时间参数化
    CORRIDOR_EXCEEDED = 2       // 轨迹可用，但超过 max_deviation_m
};
```

> 注意：`StatusCode` 只表示主结果；时间戳回退、走廊超限等非致命信息通过 `warning_flags` 暴露。走廊约束是否满足，应以 `result.isCorridorExceeded()` 或 `result.report.fit.fit_converged` 为准。

> **关于 `traj` 的空值语义**：当前实现中，所有非 `SUCCESS` 的路径都直接返回 `traj = nullptr`；只有生成流程完整走到末尾才会构造 `Trajectory` 对象。因此，**不要依赖"失败但部分可用"的轨迹**——如果 `!result.isSuccess()`，则 `result.traj` 一定为空。使用前务必检查 `result.isSuccess()` 或 `result.isUsable()`。

**使用示例**:
```cpp
auto result = generator.generate(waypoints, config);

// 检查主结果是否成功
if (result.isSuccess()) {
    std::cout << "Trajectory generated successfully!" << std::endl;

    if (result.hasTimeFallback()) {
        std::cout << "Warning: timestamp fallback active" << std::endl;
    }
    if (result.isCorridorExceeded()) {
        std::cout << "Warning: corridor constraint exceeded" << std::endl;
    }

    // 使用轨迹
    auto states = result.traj->sample(0.05);

} else {
    std::cerr << "Failed: " << result.message << std::endl;
    std::cerr << "Status: " << statusCodeToString(result.code) << std::endl;
}

// 查看详细报告
std::cout << result.report.toString() << std::endl;
```

### Report - 详细报告

```cpp
struct Report {
    FitReport fit;          // 拟合报告
    TimeReport time;        // 时间质量报告
    ProfileReport profile;  // 速度剖面报告
    YawReport yaw;          // Yaw 处理报告

    std::string toString() const;  // 完整报告字符串
};
```

**FitReport - 拟合报告**:
```cpp
struct FitReport {
    // 点列统计
    int n_input;                    // 输入点数
    int n_preprocessed;             // 预处理后点数
    int n_resampled;                // 重采样后点数
    int n_control;                  // 预处理后路径点数（参考折线顶点数）
    int n_segments;                 // 轨迹段数
    int n_corners;                  // 检测到的拐角数

    // 偏差统计（相对输入折线）
    double max_deviation_m;         // 最大偏差
    double mean_deviation_m;        // 平均偏差
    double p95_deviation_m;         // 95% 分位偏差

    // 曲率统计
    CurvatureMethod kappa_method;   // 曲率计算方法
    double kappa_max;               // 最大曲率 (rad/m)
    double r_min;                   // 最小转弯半径 (m)
    double kappa_median;            // 曲率中位数
    double kappa_confidence;        // 曲率置信度 (0~1)
    int kappa_warning_flags;        // 曲率告警（位标志）

    // 拐角处理
    CornerContinuity corner_continuity_level;  // 拐角连续性级别
    double max_corner_pass_error_m;            // 最大拐角通过误差（预留）
    bool short_segment_warning;                // 是否存在短段告警

    // 高度统计
    double z_max;
    double z_min;
    double z_range;

    // 拟合过程
    int fit_iterations;             // 实际迭代次数（当前实现始终为 1）
    bool fit_converged;             // 走廊约束是否满足

    // Phase 2: 分段几何统计
    int n_linear;                   // LINEAR 直线段数量
    int n_hermite;                  // HERMITE_BLEND 过渡段数量
    double max_blend_corridor_deviation;  // blend 段最大走廊偏差
    int n_overlaps_resolved;        // blend 重叠规范化次数
};
```

**TimeReport - 时间质量报告**:
```cpp
struct TimeReport {
    TimeMode time_mode;             // 时间处理模式（是否触发回退）
    bool is_monotonic;              // 时间戳是否单调
    double max_dt_gap_s;            // 最大时间间隙
    bool has_time_fallback;         // 是否触发时间回退
    std::string fallback_reason;    // 回退原因
};
```

> **时间语义澄清**：`TimeReport` 和 `TimeMode` 是纯诊断信息，仅反映输入时间戳的质量检查结果。**无论时间戳是否正常（`USE_STAMP` 还是 `FALLBACK_*`），输出轨迹的时间轴都由内部"速度剖面 + `integrateTime()`"重新生成**，不会复现输入时间戳。`time_mode` 的实际影响仅限于 `TimeReport` 标记和 `warning_flags` 中的 `TIME_FALLBACK` 位。

**ProfileReport - 速度剖面报告**:
```cpp
struct ProfileReport {
    double total_duration_s;        // 总时长
    double max_speed_m_s;           // 最大速度
    double mean_speed_m_s;          // 平均速度
    double path_length_m;           // 路径长度

    double max_a_tan_m_s2;          // 最大切向加速度
    double max_a_lat_m_s2;          // 最大横向加速度
    double max_vz_m_s;              // 最大竖直速度（估计）
    double max_az_m_s2;             // 最大竖直加速度（预留/估计）

    bool end_converge_triggered;    // 末端收敛是否触发
    double end_converge_duration_s; // 收敛持续时间（估计）
    double end_max_speed_m_s;       // 收敛段最大速度

    // yaw 约束对速度的影响
    double yaw_limited_ratio;       // yaw 约束成为主限制因素的比例
    double min_v_due_to_yaw_m_s;    // yaw 约束导致的最小速度上界
    bool yaw_limit_warning;         // Yaw 限制警告
    bool curvature_guard_active;    // 曲率保护是否激活（基于分段解析曲率真实检测）
};
```

**YawReport - Yaw 处理报告**:
```cpp
struct YawReport {
    double max_yaw_smooth_diff_rad;     // Yaw 平滑最大偏差
    double mean_yaw_smooth_diff_rad;    // Yaw 平滑平均偏差

    double max_yaw_rate_rad_s;          // 最大 yaw 角速度
    bool yaw_rate_compliant;            // 是否满足 yaw 角速度约束

    double max_yaw_shaping_diff_rad;    // rate limiter 导致的最大 shaping 偏差
    double mean_yaw_shaping_diff_rad;   // rate limiter 导致的平均 shaping 偏差

    bool yaw_hold_triggered;            // Yaw 保持是否触发
    double yaw_hold_duration_s;         // Yaw 保持持续时间（估计）
};
```

**使用示例**:
```cpp
// 完整报告
std::cout << result.report.toString() << std::endl;

// 访问特定模块报告
std::cout << "Total duration: " << result.report.profile.total_duration_s << " s" << std::endl;
std::cout << "Path length: " << result.report.profile.path_length_m << " m" << std::endl;
std::cout << "Max speed: " << result.report.profile.max_speed_m_s << " m/s" << std::endl;
std::cout << "Max yaw rate: " << result.report.yaw.max_yaw_rate_rad_s << " rad/s" << std::endl;

// 检查约束满足情况
if (!result.report.yaw.yaw_rate_compliant) {
    std::cerr << "Warning: Yaw rate constraint violated!" << std::endl;
}

if (result.report.profile.yaw_limit_warning) {
    std::cerr << "Warning: Yaw constraint limited velocity!" << std::endl;
}
```

---

## 主要接口类

### TrajectoryGenerator - 轨迹生成器

这是库的主入口点，负责将路径点转换为连续轨迹。

```cpp
class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    /**
     * @brief 从路径点生成轨迹
     *
     * @param waypoints 输入路径点（ENU 坐标，位置 + yaw + 时间戳）
     * @param config 生成配置
     * @return Result 包含轨迹、状态和报告的结果对象
     *
     * 处理流程：
     * 1. 预处理：时间戳检查、yaw 展开、去重点
     * 2. 弧长重采样：生成等间距点
     * 3. 拐角检测 → 角点投影 → 共线 run 长度 → blend 生成与规范化
     * 4. 分段几何主链构建（LINEAR 直线段 + HERMITE_BLEND 角点过渡段）
     * 5. 时间参数化：生成速度剖面（受曲率/竖直/yaw-rate 约束影响）
     * 6. 末端收敛：靠近终点时对速度剖面做减速处理并重新积分时间
     * 7. Yaw 处理：参考折线投影 → 可选平滑 + 末端 yaw hold + 逐点 dt rate limiter
     * 8. 生成 Trajectory 对象
     */
    Result generate(const std::vector<Waypoint>& waypoints,
                   const TrajectoryConfig& config);
};
```

**使用示例**:
```cpp
// 创建生成器（可重复使用）
TrajectoryGenerator generator;

// 生成多条轨迹
for (const auto& path : paths) {
    auto result = generator.generate(path, config);
    if (result.isSuccess()) {
        // 处理轨迹
    }
}
```

### Trajectory - 轨迹对象

封装生成的轨迹，提供时间查询和采样功能。

```cpp
class Trajectory {
public:
    /**
     * @brief 在给定时间评估轨迹状态
     * @param t 相对于轨迹起点的时间 (s)
     * @return 时刻 t 的轨迹状态（实现会将 t clamp 到 [0, duration]）
     */
    TrajectoryState evaluate(double t) const;

    /**
     * @brief 按固定时间间隔采样轨迹
     * @param dt 时间步长（默认 0.05s = 20Hz）
     * @return 轨迹状态向量（dt<=0 或轨迹无效时返回空）
     */
    std::vector<TrajectoryState> sample(double dt = 0.05) const;

    /**
     * @brief 获取轨迹总时长
     * @return 时长 (s)
     */
    double duration() const;

    /**
     * @brief 获取路径总长度
     * @return 长度 (m)
     */
    double pathLength() const;

    /// 获取起始时间（始终为 0）
    double startTime() const;

    /// 获取结束时间（== duration()）
    double endTime() const;

    /**
     * @brief 根据弧长查询曲率
     *
     * 在有分段几何信息时使用解析曲率（LINEAR 段精确为 0，
     * HERMITE_BLEND 段使用 |r'×r''|/|r'|³），
     * 否则回退到重采样点的线性插值。
     *
     * @param s 从起点计的弧长 (m)
     * @return 曲率 (1/R)，单位 rad/m
     */
    double curvatureFromArcLength(double s) const;

    /**
     * @brief 检查轨迹是否有效
     */
    bool isValid() const;

    /**
     * @brief 获取目标位置
     */
    Vec3 goalPosition() const;

    /**
     * @brief 获取目标 yaw 角
     *
     * 返回生成阶段保存的终点参考 yaw（来自 unwrapped yaw），
     * 不经过 wrapToPi() 处理。当输入 yaw 跨越 ±π 时，
     * goalYaw() 与 evaluate(endTime()).yaw（已 wrap 到 [-π, π]）
     * 可能相差整圈。比较时建议用角度差归一化。
     */
    double goalYaw() const;
};
```

**使用示例**:

```cpp
// 假设已经生成了轨迹
auto& traj = result.traj;

// 1. 查询轨迹信息
double duration = traj->duration();       // 总时长
double length = traj->pathLength();       // 总路径长度
Vec3 goal_pos = traj->goalPosition();     // 终点位置
double goal_yaw = traj->goalYaw();        // 终点 yaw

std::cout << "Duration: " << duration << " s" << std::endl;
std::cout << "Length: " << length << " m" << std::endl;
std::cout << "Goal: " << goal_pos.transpose() << std::endl;

// 2. 在特定时间点评估轨迹
double t = 2.5;  // 2.5 秒时刻
auto state = traj->evaluate(t);
std::cout << "At t=" << t << "s:" << std::endl;
std::cout << "  Position: " << state.position.transpose() << std::endl;
std::cout << "  Velocity: " << state.velocity.transpose() << std::endl;
std::cout << "  Yaw: " << state.yaw << " rad" << std::endl;

// 3. 采样整条轨迹（用于离线分析或导出）
auto states = traj->sample(0.05);  // 20Hz 采样

std::cout << "Sampled " << states.size() << " states" << std::endl;

for (const auto& state : states) {
    // 保存到文件/数据库，或交给上层系统进一步处理
    saveState(state);
}

// 4. 自定义采样频率
auto states_10hz = traj->sample(0.1);   // 10Hz
auto states_50hz = traj->sample(0.02);  // 50Hz

// 5. 手动遍历轨迹
for (double t = 0.0; t <= traj->duration(); t += 0.05) {
    auto state = traj->evaluate(t);
    // 处理 state
}

// 6. 查询指定弧长处的曲率
double s = 2.0;  // 弧长 2m 处
double kappa = traj->curvatureFromArcLength(s);
std::cout << "Curvature at s=" << s << "m: " << kappa << " rad/m" << std::endl;
if (kappa > 1e-6) {
    std::cout << "Turning radius: " << 1.0 / kappa << " m" << std::endl;
}
```

---

## 完整使用流程

### 1. 准备输入数据

```cpp
#include <trajectory_core/trajectory_core.hpp>
#include <vector>

// 从录制数据或路径规划器获取路径点
std::vector<trajectory_core::Waypoint> waypoints;

// 方法 1: 手动创建路径点
waypoints.emplace_back(0.0, 0.0, 5.0, 0.0, 0.0);           // 起点
waypoints.emplace_back(5.0, 0.0, 5.0, 0.0, 2.5);
waypoints.emplace_back(10.0, 0.0, 5.0, 0.0, 5.0);
waypoints.emplace_back(10.0, 5.0, 5.0, M_PI/4, 7.5);
waypoints.emplace_back(10.0, 10.0, 5.0, M_PI/2, 10.0);     // 终点

// 方法 2: 从文件加载
std::ifstream file("recorded_path.csv");
std::string line;
while (std::getline(file, line)) {
    double x, y, z, yaw, t;
    // 解析 CSV 行
    std::istringstream ss(line);
    char comma;
    ss >> x >> comma >> y >> comma >> z >> comma >> yaw >> comma >> t;
    waypoints.emplace_back(x, y, z, yaw, t);
}

```

**输入数据要求**:
- 至少 2 个路径点
- 时间戳应单调递增（可选，异常时会自动回退）
- 相邻点不要太近（< 0.01m 会被自动去重）
- 坐标系应为 ENU

### 2. 配置参数

```cpp
// 创建配置对象
trajectory_core::TrajectoryConfig config;

// 场景 1: 室内狭窄环境 - 精确低速
config.v_max = 1.0;              // 最大速度 1 m/s
config.yaw_rate_max = 0.4;       // 慢速转向
config.ds_m = 0.1;               // 精细采样
config.max_deviation_m = 0.1;    // 紧走廊约束
config.corner_blend_len_m = 0.2; // 短拐角过渡

// 场景 2: 室外开阔环境 - 中速平稳
config.v_max = 3.0;              // 最大速度 3 m/s
config.vz_max = 1.5;             // 允许较快爬升/下降
config.yaw_rate_max = 0.8;       // 常规转向速度
config.ds_m = 0.2;               // 标准采样
config.max_deviation_m = 0.2;    // 标准走廊
config.corner_blend_len_m = 0.5; // 标准拐角

// 场景 3: 长距离快速飞行
config.v_max = 6.0;              // 高速
config.vz_max = 2.0;
config.yaw_rate_max = 1.2;       // 快速转向
config.ds_m = 0.3;               // 粗采样（减少计算量）
config.max_deviation_m = 0.5;    // 宽松走廊（允许更平滑）
config.corner_blend_len_m = 1.0; // 长拐角过渡

// 末端收敛配置（返航场景）
config.enable_end_converge = true;
config.s_stop_m = 3.0;           // 距终点 3m 开始减速
config.r_stop_m = 2.0;           // 距终点 2m 触发收敛
config.v_end_max = 0.5;          // 收敛段最大速度 0.5 m/s

// 验证配置
if (!config.validate()) {
    std::cerr << "Invalid trajectory configuration!" << std::endl;
    return;
}

// 或者从文件加载
try {
    auto config = trajectory_core::TrajectoryConfig::loadFromYaml(
        "config/trajectory.yaml"
    );
} catch (const std::exception& e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
}
```

### 3. 生成轨迹

```cpp
// 创建生成器
trajectory_core::TrajectoryGenerator generator;

// 生成轨迹
auto result = generator.generate(waypoints, config);

// 检查结果
if (!result.isSuccess()) {
    std::cerr << "Trajectory generation failed: " << result.message << std::endl;
    std::cerr << "Status code: "
              << trajectory_core::statusCodeToString(result.code) << std::endl;

    // 根据不同错误类型处理
    switch (result.code) {
        case trajectory_core::StatusCode::INVALID_INPUT:
            std::cerr << "Input waypoints are invalid" << std::endl;
            break;
        case trajectory_core::StatusCode::FIT_FAILED:
            std::cerr << "Geometry fitting failed, check input data and configuration" << std::endl;
            break;
        case trajectory_core::StatusCode::PROFILE_INFEASIBLE:
            std::cerr << "Cannot generate velocity profile, try reducing v_max or increasing a_tan_max" << std::endl;
            break;
        default:
            break;
    }

    return;
}

// 检查警告
if (result.hasTimeFallback()) {
    std::cerr << "Warning: timestamp anomaly detected, internal time fallback is active: "
              << result.message << std::endl;
}

// 走廊约束检查（即便主结果 SUCCESS，也建议检查）
if (result.isCorridorExceeded()) {
    std::cerr << "Warning: corridor constraint may be violated: max_deviation="
              << result.report.fit.max_deviation_m
              << " m > allowed=" << config.max_deviation_m << " m" << std::endl;
}

// 打印报告
std::cout << "Trajectory generated successfully!" << std::endl;
std::cout << "Duration: " << result.report.profile.total_duration_s << " s" << std::endl;
std::cout << "Path length: " << result.report.profile.path_length_m << " m" << std::endl;
std::cout << "Max speed: " << result.report.profile.max_speed_m_s << " m/s" << std::endl;
std::cout << "Max yaw rate: " << result.report.yaw.max_yaw_rate_rad_s << " rad/s" << std::endl;
std::cout << result.report.toString() << std::endl;
```

### 4. 使用轨迹

```cpp
auto& traj = result.traj;

// 方法 1: 一次性采样所有状态（适合离线分析）
auto all_states = traj->sample(0.05);  // 20Hz

// 保存到文件
std::ofstream csv("trajectory.csv");
csv << "t,x,y,z,vx,vy,vz,yaw,yaw_rate\n";
for (const auto& state : all_states) {
    csv << state.t << ","
        << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
        << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
        << state.yaw << "," << state.yaw_rate << "\n";
}
```

---

## 错误处理

### 状态码处理

```cpp
auto result = generator.generate(waypoints, config);

switch (result.code) {
    case trajectory_core::StatusCode::SUCCESS:
        // 成功生成（仍建议结合 warning/report 检查质量/约束是否满足）
        processTrajectory(result.traj);
        break;

    case trajectory_core::StatusCode::INVALID_INPUT:
        // 输入数据无效
        std::cerr << "Invalid input: " << result.message << std::endl;
        // 检查输入数据，可能需要：
        // - 增加路径点数量（至少 2 个）
        // - 检查点之间距离（不要过近）
        // - 检查时间戳
        break;

    case trajectory_core::StatusCode::FIT_FAILED:
        // 几何拟合失败（数值失败/内部失败）
        std::cerr << "Fitting failed: " << result.message << std::endl;
        // 可能的解决方案：
        // - 检查输入点是否退化（大量重合/总长度过小）
        // - 适当增大 ds_m（减少噪声）或放宽 max_deviation_m
        // - 如果只是走廊约束未满足：以 result.report.fit.fit_converged / max_deviation_m 为准（不一定会返回 FIT_FAILED）
        break;

    case trajectory_core::StatusCode::PROFILE_INFEASIBLE:
        // 无法生成速度剖面
        std::cerr << "Velocity profile infeasible: " << result.message << std::endl;
        // 可能的解决方案：
        // - 降低 config.v_max
        // - 增大 config.a_tan_max
        // - 增大 config.yaw_rate_max
        // - 检查路径曲率（是否有过急转弯）
        break;

    case trajectory_core::StatusCode::INTERNAL_ERROR:
        // 内部错误（不应该发生）
        std::cerr << "Internal error: " << result.message << std::endl;
        break;
}
```

### 配置验证

```cpp
trajectory_core::TrajectoryConfig config;

// 设置参数...

// 验证前检查
if (!config.validate()) {
    std::cerr << "Configuration validation failed!" << std::endl;

    // 常见错误检查
    if (config.corner_blend_len_m < 2.0 * config.ds_m) {
        std::cerr << "corner_blend_len_m (" << config.corner_blend_len_m
                  << ") < 2 * ds_m (" << 2.0 * config.ds_m << ")" << std::endl;
    }

    if (config.v_end_max > config.v_max) {
        std::cerr << "v_end_max (" << config.v_end_max
                  << ") > v_max (" << config.v_max << ")" << std::endl;
    }

    if (config.max_deviation_m < config.ds_m * 0.5) {
        std::cerr << "max_deviation_m (" << config.max_deviation_m
                  << ") too small relative to ds_m (" << config.ds_m << ")" << std::endl;
    }

    return;
}
```

### 异常处理

```cpp
try {
    // 从文件加载配置
    auto config = trajectory_core::TrajectoryConfig::loadFromYaml("config.yaml");

    // 生成轨迹
    auto result = generator.generate(waypoints, config);

    if (result.isSuccess()) {
        // 使用轨迹
        auto states = result.traj->sample(0.05);
    }

} catch (const std::runtime_error& e) {
    std::cerr << "Runtime error: " << e.what() << std::endl;
    // 文件读取错误、内存分配失败等

} catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;

} catch (...) {
    std::cerr << "Unknown exception" << std::endl;
}
```

---

## 高级用法

### 1. 批量处理多条路径

```cpp
void processMultiplePaths(
    const std::vector<std::vector<trajectory_core::Waypoint>>& paths,
    const trajectory_core::TrajectoryConfig& config) {

    trajectory_core::TrajectoryGenerator generator;

    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "Processing path " << (i + 1)
                  << "/" << paths.size() << std::endl;

        auto result = generator.generate(paths[i], config);

        if (result.isSuccess()) {
            // 保存轨迹
            saveTrajectory(result.traj, "trajectory_" + std::to_string(i) + ".csv");

            // 记录统计信息
            std::cout << "  Duration: " << result.report.profile.total_duration_s
                      << " s" << std::endl;
            std::cout << "  Length: " << result.report.profile.path_length_m
                      << " m" << std::endl;
        } else {
            std::cerr << "  Failed: " << result.message << std::endl;
        }
    }
}
```

### 2. 动态调整配置

```cpp
// 根据路径特征自动调整配置
trajectory_core::TrajectoryConfig adaptConfig(
    const std::vector<trajectory_core::Waypoint>& waypoints) {

    trajectory_core::TrajectoryConfig config;

    // 计算路径总长度
    double total_length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        total_length += (waypoints[i].position - waypoints[i-1].position).norm();
    }

    // 根据路径长度调整参数
    if (total_length < 20.0) {
        // 短路径：精确低速
        config.v_max = 1.5;
        config.ds_m = 0.1;
    } else if (total_length < 100.0) {
        // 中等路径：常规速度
        config.v_max = 3.0;
        config.ds_m = 0.2;
    } else {
        // 长路径：高速
        config.v_max = 5.0;
        config.ds_m = 0.3;
    }

    // 计算高度变化
    double z_min = waypoints[0].position.z();
    double z_max = z_min;
    for (const auto& wp : waypoints) {
        z_min = std::min(z_min, wp.position.z());
        z_max = std::max(z_max, wp.position.z());
    }
    double z_range = z_max - z_min;

    // 根据高度变化调整竖直速度
    if (z_range > 10.0) {
        config.vz_max = 1.5;
    } else {
        config.vz_max = 0.8;
    }

    return config;
}
```

### 3. 轨迹分析和可视化

```cpp
void analyzeTrajectory(const trajectory_core::Result& result) {
    if (!result.isSuccess()) return;

    auto& traj = result.traj;
    auto& report = result.report;

    // 1. 基本信息
    std::cout << "=== Trajectory Analysis ===" << std::endl;
    std::cout << "Duration: " << traj->duration() << " s" << std::endl;
    std::cout << "Length: " << traj->pathLength() << " m" << std::endl;
    std::cout << "Average speed: " << report.profile.mean_speed_m_s << " m/s" << std::endl;

    // 2. 约束满足情况
    std::cout << "\n=== Constraints ===" << std::endl;
    std::cout << "Max speed: " << report.profile.max_speed_m_s << " m/s" << std::endl;
    std::cout << "Max yaw rate: " << report.yaw.max_yaw_rate_rad_s << " rad/s" << std::endl;
    std::cout << "Yaw rate compliant: "
              << (report.yaw.yaw_rate_compliant ? "Yes" : "No") << std::endl;

    // 3. 质量指标
    std::cout << "\n=== Quality ===" << std::endl;
    std::cout << "Max deviation: " << report.fit.max_deviation_m << " m" << std::endl;
    std::cout << "Min turning radius: " << report.fit.r_min << " m" << std::endl;
    std::cout << "Corners detected: " << report.fit.n_corners << std::endl;

    // 4. 警告信息
    if (report.profile.yaw_limit_warning) {
        std::cout << "\n[WARNING] Yaw constraint limited velocity!" << std::endl;
    }
    if (!report.yaw.yaw_rate_compliant) {
        std::cout << "\n[WARNING] Yaw rate constraint violated!" << std::endl;
    }

    // 5. 采样并分析速度分布
    auto states = traj->sample(0.05);

    std::vector<double> speeds;
    for (const auto& state : states) {
        speeds.push_back(state.speed());
    }

    double speed_var = 0.0;
    for (double v : speeds) {
        double diff = v - report.profile.mean_speed_m_s;
        speed_var += diff * diff;
    }
    speed_var /= speeds.size();

    std::cout << "\n=== Speed Statistics ===" << std::endl;
    std::cout << "Speed variance: " << speed_var << std::endl;
    std::cout << "Speed std dev: " << std::sqrt(speed_var) << std::endl;
}
```

### 4. 保存和加载轨迹

```cpp
// 保存轨迹到 CSV
void saveTrajectory(const std::unique_ptr<trajectory_core::Trajectory>& traj,
                   const std::string& filename) {
    auto states = traj->sample(0.05);

    std::ofstream file(filename);
    file << "t,x,y,z,vx,vy,vz,ax,ay,az,yaw,yaw_rate,speed\n";

    for (const auto& state : states) {
        file << state.t << ","
             << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
             << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ",";

        if (state.acceleration.has_value()) {
            file << state.acceleration->x() << ","
                 << state.acceleration->y() << ","
                 << state.acceleration->z() << ",";
        } else {
            file << ",,," ;
        }

        file << state.yaw << "," << state.yaw_rate << "," << state.speed() << "\n";
    }
}
```

---

## 完整示例代码

### 示例 1: 基本轨迹生成

```cpp
#include <trajectory_core/trajectory_core.hpp>
#include <iostream>
#include <fstream>

int main() {
    // 创建路径点
    std::vector<trajectory_core::Waypoint> waypoints;
    waypoints.emplace_back(0, 0, 5, 0, 0.0);
    waypoints.emplace_back(5, 0, 5, 0, 2.5);
    waypoints.emplace_back(10, 0, 5, 0, 5.0);
    waypoints.emplace_back(10, 5, 5, M_PI/4, 7.5);
    waypoints.emplace_back(10, 10, 5, M_PI/2, 10.0);

    // 配置参数
    trajectory_core::TrajectoryConfig config;
    config.v_max = 2.0;
    config.yaw_rate_max = 0.6;
    config.enable_end_converge = true;

    if (!config.validate()) {
        std::cerr << "Invalid configuration!" << std::endl;
        return 1;
    }

    // 生成轨迹
    trajectory_core::TrajectoryGenerator generator;
    auto result = generator.generate(waypoints, config);

    if (!result.isSuccess()) {
        std::cerr << "Generation failed: " << result.message << std::endl;
        return 1;
    }

    // 打印信息
    std::cout << "Trajectory generated successfully!" << std::endl;
    std::cout << "Duration: " << result.traj->duration() << " s" << std::endl;
    std::cout << "Length: " << result.traj->pathLength() << " m" << std::endl;

    // 采样并保存
    auto states = result.traj->sample(0.05);
    std::ofstream file("trajectory.csv");
    file << "t,x,y,z,vx,vy,vz,yaw,yaw_rate\n";
    for (const auto& state : states) {
        file << state.t << ","
             << state.position.x() << "," << state.position.y() << "," << state.position.z() << ","
             << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
             << state.yaw << "," << state.yaw_rate << "\n";
    }

    std::cout << "Saved " << states.size() << " states to trajectory.csv" << std::endl;

    return 0;
}
```

## 常见问题

### Q1: 如何选择合适的参数？

**A**: 参考 [参数配置指南](../../../trajectory_core/doc/reference/参数配置指南.md) 中的典型场景配置，根据实际情况调整。关键参数：
- `v_max`: 根据飞行器性能和安全要求
- `ds_m` 和 `max_deviation_m`: 通常保持一致，0.1-0.3m
- `yaw_rate_max`: 0.6 rad/s 适合大多数情况

### Q2: 生成失败怎么办？

**A**: 根据 `StatusCode` 采取对应措施：
- `FIT_FAILED`: 几何拟合内部失败，优先检查输入点退化/参数合法性；若只是走廊约束未满足，请以 `report.fit.fit_converged` 与 `report.fit.max_deviation_m` 为准，再考虑增大 `max_deviation_m` 或调整 `ds_m`
- `PROFILE_INFEASIBLE`: 降低 `v_max` 或增大 `a_tan_max`
- `INVALID_INPUT`: 检查输入数据质量

### Q3: 如何处理 ENU/NED 坐标转换？

**A**:
- 输入给库：使用 ENU 坐标
- 库本身不处理外部飞控/中间件的坐标系转换；与 ROS、MAVROS、MAVLink 或其他系统的坐标映射，请以对应系统的官方文档和实际链路测试为准。

### Q4: 轨迹可以重复使用吗？

**A**: 可以。`Trajectory` 对象可以多次调用 `evaluate()` 和 `sample()`，是线程安全的（只读操作）。

### Q5: 如何调试轨迹生成问题？

**A**:
1. 打印 `Result::message` 查看错误信息
2. 查看 `Report` 中的详细统计信息（含分段几何统计 `n_linear`/`n_hermite`）
3. 使用 `trajectory_case_tool` 一键生成/导出/绘图（参见 README 可视化章节）
4. 将轨迹导出为 CSV，使用 `tools/plot_trajectory.py` 可视化（含曲率剖面、速度约束分解等）
5. 检查 `Report::yaw::yaw_rate_compliant` 等标志

### Q6: 如何提高实时性能？

**A**:
- 增大 `ds_m`（减少采样点数量）
- 使用更粗的 `dt_sample_s`
- 离线预先生成轨迹，不要在控制循环中生成

### Q7: 支持动态避障吗？

**A**:
库本身不支持动态避障。需要在上层实现：
- 实时重规划：检测到障碍物时重新生成轨迹
- 局部修正：使用其他算法（如 APF）局部调整轨迹

### Q8: 可以用于其他机器人吗？

**A**:
可以，只要坐标系是 ENU。但需要根据机器人特性调整：
- 地面机器人：保持输入 `z` 基本恒定，并将 `vz_max` 设为很小的正数（例如 `1e-3`），同时调整 `v_max` 和曲率约束
- 其他飞行器：调整速度、加速度、yaw 速率等参数

### Q9: 内存占用如何？

**A**:
- 取决于路径长度和采样密度
- 典型 100 个输入点，生成约 1000 个采样点，内存 < 1MB
- `Trajectory` 对象包含采样数据，长时间保存多个轨迹需注意内存

### Q10: 线程安全吗？

**A**:
- `TrajectoryGenerator::generate()`: 当前实现内部无可变状态，通常可并发调用；但未作为 API 保证，保守建议外部加锁或每线程一个实例
- `Trajectory::evaluate()` 和 `sample()`: 线程安全（只读操作）
- `TrajectoryConfig`: 值类型配置结构体；按 `const TrajectoryConfig&` 传入/只读使用时线程安全，但不要在多线程中共享同一实例并同时写入

---

## 附录

### A. 数据类型速查

| 类型 | 说明 | 备注 |
|------|------|------|
| `Vec3` | `Eigen::Vector3d` | 大小/对齐与编译器、Eigen 对齐选项相关 |
| `Waypoint` | 输入路径点 | 建议使用 `sizeof(trajectory_core::Waypoint)` 获取实际大小 |
| `TrajectoryState` | 轨迹状态 | 建议使用 `sizeof(trajectory_core::TrajectoryState)` 获取实际大小 |
| `TrajectoryConfig` | 配置参数 | 字段较多，建议使用 `sizeof(trajectory_core::TrajectoryConfig)` 获取实际大小 |

### B. 坐标系约定

- **ENU (East-North-Up)**
  - X 轴：东 (East)
  - Y 轴：北 (North)
  - Z 轴：上 (Up)
  - Yaw: 从 +X 轴开始，逆时针为正

### C. 单位约定

- 长度: 米 (m)
- 时间: 秒 (s)
- 角度: 弧度 (rad)
- 速度: 米/秒 (m/s)
- 加速度: 米/秒² (m/s²)
- 角速度: 弧度/秒 (rad/s)

### D. 枚举与辅助函数

以下枚举/函数位于 `include/trajectory_core/status_code.hpp`，用于报告、调试与状态判断：

```cpp
enum class TimeMode {
    USE_STAMP,
    FALLBACK_MVP,
    FALLBACK_ASSUMED_SPEED
};

enum class CurvatureMethod {
    DISCRETE_3PT,
    SEGMENT_ANALYTIC,
    BOTH
};

enum class CornerContinuity {
    C0_ONLY,
    WITH_BLEND_G1,
    WITH_BLEND_C2
};

enum class CurvatureWarning {
    NONE = 0,
    CURVATURE_SPIKE = 1,
    CURVATURE_NOISY = 2,
    CURVATURE_DISAGREE = 4
};

std::string statusCodeToString(StatusCode code);
std::string timeModeToString(TimeMode mode);
```

### E. 相关文档

- [参数配置指南](../../../trajectory_core/doc/reference/参数配置指南.md) - 库参数说明
- [返航改造方案](../design/弧长驱动返航路径跟随改造方案.md) - 返航侧的实现方案
- [算法流程](../../../trajectory_core/doc/reference/轨迹生成流程_流水线说明.md) - 库的当前生成流水线

---

**文档版本**: v2.0 (2026-03-24)
