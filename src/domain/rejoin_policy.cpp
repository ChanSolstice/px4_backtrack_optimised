#include "px4_backtrack/domain/rejoin_policy.hpp"
#include <algorithm>
#include <cmath>

namespace px4_backtrack {
namespace domain {

RejoinPolicy::RejoinPolicy(const RejoinConfig& config)
  : config_(config)
{
}

bool RejoinPolicy::shouldTriggerRejoin(double cross_track_error) const {
  return cross_track_error > config_.trigger_dist_m;
}

bool RejoinPolicy::shouldExitRejoin(
    double cross_track_error,
    const Vec3& current_velocity,
    const Vec3& path_target_velocity,
    const Vec3& published_target_velocity) const {
  return cross_track_error < config_.success_dist_m &&
         (current_velocity - path_target_velocity).norm() <
             kExitVelocityErrorMps &&
         (published_target_velocity - path_target_velocity).norm() <=
             kMaxVelocityStepMps;
}

trajectory_core::TrajectoryState RejoinPolicy::applyRejoinLimits(
    const trajectory_core::TrajectoryState& path_target,
    double cross_track_error,
    const Vec3& previous_target_velocity) const {
  auto target = path_target;
  const double nominal_speed = target.velocity.norm();
  if (nominal_speed < 1e-9) {
    target.velocity = Vec3::Zero();
    return target;
  }

  const double recovery_span =
      std::max(1e-6, config_.trigger_dist_m - config_.success_dist_m);
  const double recovery_ratio = std::clamp(
      (config_.trigger_dist_m - cross_track_error) / recovery_span,
      0.0,
      1.0);
  const double limited_speed = std::min(
      nominal_speed,
      config_.v_max_m_s +
          recovery_ratio * std::max(0.0, nominal_speed - config_.v_max_m_s));
  Vec3 desired_velocity = target.velocity * (limited_speed / nominal_speed);

  // 进入 Rejoin 时也限制参考速度的单周期变化，避免从巡航速度瞬时降到限速。
  const Vec3 velocity_step = desired_velocity - previous_target_velocity;
  const double step_norm = velocity_step.norm();
  if (step_norm > kMaxVelocityStepMps) {
    desired_velocity = previous_target_velocity +
        velocity_step * (kMaxVelocityStepMps / step_norm);
  }
  target.velocity = desired_velocity;
  target.acceleration = std::nullopt;
  return target;
}

}  // namespace domain
}  // namespace px4_backtrack
