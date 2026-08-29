#include "px4_backtrack/domain/rejoin_policy.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace px4_backtrack {
namespace domain {

RejoinPolicy::RejoinPolicy(const RejoinConfig& config)
  : config_(config)
{
}

bool RejoinPolicy::shouldTriggerRejoin(double cross_track_error) const {
  return cross_track_error > config_.trigger_dist_m;
}

bool RejoinPolicy::shouldExitRejoin(double cross_track_error) const {
  return cross_track_error < config_.success_dist_m;
}

double RejoinPolicy::findNearestTime(const Vec3& current_pos,
                                      const trajectory_core::Trajectory* traj) const {
  if (!traj || !traj->isValid()) {
    return 0.0;
  }

  // 采样轨迹并查找最近点
  // 使用较粗的采样间隔（20Hz）进行快速搜索
  const double dt_sample = 0.05;  // 20Hz
  auto samples = traj->sample(dt_sample);

  if (samples.empty()) {
    return 0.0;
  }

  // 查找距离当前位置最近的采样点
  double min_dist = std::numeric_limits<double>::max();
  double nearest_time = 0.0;

  for (const auto& state : samples) {
    double dist = (current_pos - state.position).norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_time = state.t;
    }
  }

  return nearest_time;
}

trajectory_core::TrajectoryState RejoinPolicy::computeRejoinTarget(
    const Vec3& current_pos,
    const trajectory_core::Trajectory* traj,
    double t_nearest) const {

  if (!traj || !traj->isValid()) {
    // 返回空状态
    return trajectory_core::TrajectoryState();
  }

  // 计算预瞄时间
  double t_target = t_nearest + config_.lookahead_time_s;

  // Clamp到轨迹时间范围
  t_target = std::clamp(t_target, 0.0, traj->duration());

  // 评估预瞄点的状态
  auto target_state = traj->evaluate(t_target);

  // 限幅速度
  target_state.velocity = limitVelocity(target_state.velocity, config_.v_max_m_s);

  return target_state;
}

trajectory_core::TrajectoryState RejoinPolicy::computeFrozenPointTarget(
    const Vec3& current_pos,
    const trajectory_core::TrajectoryState& frozen_state) const {
  auto target_state = frozen_state;

  Vec3 delta = frozen_state.position - current_pos;
  double distance = delta.norm();
  if (distance < 1e-6) {
    target_state.velocity = Vec3::Zero();
    return target_state;
  }

  constexpr double kRejoinVelocityGain = 1.0;
  double speed = std::min(config_.v_max_m_s, kRejoinVelocityGain * distance);
  target_state.velocity = delta * (speed / distance);

  return target_state;
}

Vec3 RejoinPolicy::limitVelocity(const Vec3& velocity, double v_max) const {
  double v_norm = velocity.norm();

  if (v_norm <= v_max || v_norm < 1e-6) {
    return velocity;
  }

  // 缩放速度向量
  return velocity * (v_max / v_norm);
}

}  // namespace domain
}  // namespace px4_backtrack
