#pragma once

#include <limits>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// BatteryMonitor配置参数
struct BatteryMonitorConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  bool enable_low_battery{};       ///< 是否启用低电量检测
  double low_battery_threshold{};  ///< 低电量阈值 (0.0~1.0)，低于此值视为低电量
  double low_battery_confirm_s{};  ///< 低电量确认时间 (s)
};

// ============================================================================
// 电量状态
// ============================================================================

/// 电量状态快照（电平语义）
struct BatteryStatus {
  bool low{false};      ///< 低电量（锁存到新的飞行周期）
  bool invalid{false};  ///< 电量值无效：最近一次读数为 NaN、<0 或 >1，或还没有收到读数
  double percentage{std::numeric_limits<double>::quiet_NaN()};  ///< 最近一次有效读数
};

// ============================================================================
// BatteryMonitor类
// ============================================================================

/// 电量监测（有效性检查、持续确认、按飞行周期锁存）
///
/// 无效读数不参与判定：既不打断也不推进确认，低电量保持原值。
class BatteryMonitor {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit BatteryMonitor(const BatteryMonitorConfig& config);

  /// 处理一次电量读数
  /// @param t 接收时间 (s)
  /// @param percentage 电量百分比（0.0~1.0，来自 sensor_msgs/BatteryState）
  void onReading(double t, double percentage);

  /// 推进到当前时间（由20Hz tick调用）
  /// @param t 当前时间 (s)
  void update(double t);

  /// 新的飞行周期：清除低电量锁存和确认计时
  void reset();

  /// 获取当前状态
  const BatteryStatus& status() const { return status_; }

  /// 读数是否有效（有限值且在 [0, 1] 内）
  static bool isValidReading(double percentage);

private:
  /// 按最近一次有效读数推进确认
  void evaluate(double t);

  /// 配置参数
  BatteryMonitorConfig config_;

  /// 当前状态
  BatteryStatus status_;

  bool confirming_{false};   ///< 有效读数低于阈值，正在确认
  double low_since_t_{0.0};  ///< 本次确认的开始时间
};

}  // namespace domain
}  // namespace px4_backtrack
