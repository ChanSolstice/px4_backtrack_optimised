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

}  // namespace domain
}  // namespace px4_backtrack
