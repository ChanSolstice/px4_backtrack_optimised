#include "px4_backtrack/domain/airborne_monitor.hpp"

namespace px4_backtrack {
namespace domain {

AirborneMonitor::AirborneMonitor(const AirborneMonitorConfig& config)
  : config_(config)
{
}

void AirborneMonitor::onArmed(bool armed) {
  if (!armed) {
    // 未解锁：等待下一次解锁
    has_armed_state_ = true;
    armed_ = false;
    status_.arm_height = ArmHeightRef::None;
    return;
  }

  if (!has_armed_state_) {
    // 第一次收到飞控状态就已解锁（例如空中重启）：当时的高度不是解锁高度
    has_armed_state_ = true;
    armed_ = true;
    status_.arm_height = ArmHeightRef::NotApplicable;
    return;
  }

  if (!armed_) {
    // 从未解锁到解锁：新的飞行周期，取当时的里程计高度
    armed_ = true;
    if (has_height_) {
      z_arm_ = z_;
      status_.arm_height = ArmHeightRef::Known;
    } else {
      status_.arm_height = ArmHeightRef::Pending;
    }
  }
}

void AirborneMonitor::onHeight(double z) {
  has_height_ = true;
  z_ = z;
  if (status_.arm_height == ArmHeightRef::Pending) {
    z_arm_ = z;
    status_.arm_height = ArmHeightRef::Known;
  }
}

void AirborneMonitor::onLandedState(double t, LandedState state) {
  has_landed_ = true;
  landed_ = state;
  landed_t_ = t;
}

void AirborneMonitor::update(double t) {
  // 没有收到、状态未定义或已过期时视为未定义，即"飞控未报告落地"
  const bool landed_fresh =
      has_landed_ && t - landed_t_ < config_.fcu_state_stale_s;
  status_.landed_state = landed_fresh ? landed_ : LandedState::Undefined;
  const bool fcu_not_on_ground = status_.landed_state != LandedState::OnGround;

  bool height_ok = false;
  status_.height_above_arm_m = std::numeric_limits<double>::quiet_NaN();
  switch (status_.arm_height) {
    case ArmHeightRef::Known:
      status_.height_above_arm_m = z_ - z_arm_;
      height_ok = status_.height_above_arm_m > config_.airborne_min_height_m;
      break;
    case ArmHeightRef::NotApplicable:
      height_ok = true;
      break;
    case ArmHeightRef::None:
    case ArmHeightRef::Pending:
      height_ok = false;
      break;
  }

  status_.airborne = fcu_not_on_ground && height_ok;
}

}  // namespace domain
}  // namespace px4_backtrack
