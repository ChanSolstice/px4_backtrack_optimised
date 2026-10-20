#pragma once

#include <cstdint>
#include <limits>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// AirborneMonitor配置参数
struct AirborneMonitorConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  double airborne_min_height_m{};  ///< 在空中判定：比解锁高度至少高出的距离 (m)
  double fcu_state_stale_s{};      ///< 飞控状态新鲜度 (s)：landed_state 超过此时间未更新视为未定义
};

// ============================================================================
// 在空中判定状态
// ============================================================================

/// 飞控落地状态（取值与 mavros_msgs/ExtendedState::landed_state 一致）
enum class LandedState : uint8_t {
  Undefined = 0,
  OnGround = 1,
  InAir = 2,
  Takeoff = 3,
  Landing = 4
};

/// 解锁高度的来源
enum class ArmHeightRef {
  None,           ///< 还没有收到飞控状态，或未解锁
  Known,          ///< 已记录解锁高度
  NotApplicable   ///< 没有观察到解锁过程（空中重启）或解锁时还没有里程计：只按 landed_state 判定
};

/// 在空中判定快照
struct AirborneStatus {
  bool airborne{false};  ///< 在空中：飞控未报告落地，且比解锁高度高出 airborne_min_height_m
  LandedState landed_state{LandedState::Undefined};  ///< 生效的落地状态（过期或未收到时为 Undefined）
  ArmHeightRef arm_height{ArmHeightRef::None};       ///< 解锁高度的来源
  double height_above_arm_m{std::numeric_limits<double>::quiet_NaN()};  ///< 比解锁高度高出的距离
};

// ============================================================================
// AirborneMonitor类
// ============================================================================

/// 在空中判定（飞控落地状态 + 与解锁高度的差，正向判定）
///
/// 只看 landed_state 不够：触地瞬间摇杆回中，或 OFFBOARD 原地保持时，
/// setpoint 不带下降速度，PX4 不会报告落地。
class AirborneMonitor {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit AirborneMonitor(const AirborneMonitorConfig& config);

  /// 处理飞控解锁状态（/mavros/state）。观察到从未解锁到解锁时记录解锁高度。
  /// 只传入 connected 为真的消息：MAVROS 未连接飞控时也会发布 armed=false
  /// @param armed 是否解锁
  void onArmed(bool armed);

  /// 处理里程计高度（ENU，向上为正）
  /// @param z 高度 (m)
  void onHeight(double z);

  /// 处理飞控落地状态（/mavros/extended_state）
  /// @param t 接收时间 (s)
  /// @param state 落地状态
  void onLandedState(double t, LandedState state);

  /// 推进到当前时间（由20Hz tick调用）
  /// @param t 当前时间 (s)
  void update(double t);

  /// 获取当前状态
  const AirborneStatus& status() const { return status_; }

private:
  /// 配置参数
  AirborneMonitorConfig config_;

  /// 当前状态
  AirborneStatus status_;

  bool has_armed_state_{false};  ///< 是否收到过飞控解锁状态
  bool armed_{false};            ///< 最近一次解锁状态
  double z_arm_{0.0};            ///< 解锁高度 (m)
  bool has_height_{false};       ///< 是否收到过里程计高度
  double z_{0.0};                ///< 最近一次里程计高度 (m)
  bool has_landed_{false};       ///< 是否收到过落地状态
  LandedState landed_{LandedState::Undefined};  ///< 最近一次落地状态
  double landed_t_{0.0};         ///< 最近一次落地状态的接收时间
};

}  // namespace domain
}  // namespace px4_backtrack
