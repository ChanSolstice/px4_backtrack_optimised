#include "px4_backtrack/domain/battery_monitor.hpp"

#include <cmath>

namespace px4_backtrack {
namespace domain {

namespace {

/// 时间比较容差，避免浮点误差让边界判定晚一帧
constexpr double kTimeEpsS = 1e-9;

}  // namespace

BatteryMonitor::BatteryMonitor(const BatteryMonitorConfig& config)
  : config_(config)
{
  // 还没有收到读数时视为无效，起飞前检查能发现电量话题缺失
  status_.invalid = config_.enable_low_battery;
}

void BatteryMonitor::onReading(double t, double percentage) {
  if (!config_.enable_low_battery) {
    return;
  }

  // 飞控上报电量未知时（battery_remaining=-1），MAVROS 换算出 -0.01
  if (!isValidReading(percentage)) {
    status_.invalid = true;
    return;
  }

  status_.invalid = false;
  status_.percentage = percentage;
  evaluate(t);
}

void BatteryMonitor::update(double t) {
  if (!config_.enable_low_battery) {
    return;
  }
  evaluate(t);
}

void BatteryMonitor::reset() {
  status_.low = false;
  confirming_ = false;
}

bool BatteryMonitor::isValidReading(double percentage) {
  return std::isfinite(percentage) && percentage >= 0.0 && percentage <= 1.0;
}

void BatteryMonitor::evaluate(double t) {
  // 已锁存，或最近一次读数无效（保持原值，不推进确认）
  if (status_.low || status_.invalid) {
    return;
  }

  if (status_.percentage >= config_.low_battery_threshold) {
    confirming_ = false;
    return;
  }

  if (!confirming_) {
    confirming_ = true;
    low_since_t_ = t;
  }
  if (t - low_since_t_ + kTimeEpsS >= config_.low_battery_confirm_s) {
    status_.low = true;
  }
}

}  // namespace domain
}  // namespace px4_backtrack
