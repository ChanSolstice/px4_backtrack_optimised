#include "px4_backtrack/domain/trajectory_session.hpp"
#include "px4_backtrack/domain/path_yaw.hpp"
#include <ros/console.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kLookaheadMinM = 0.15;
constexpr double kLookaheadMaxM = 0.80;
constexpr double kProjectionBackwardWindowM = 0.30;
constexpr double kProjectionCycleHorizonS = 0.50;
constexpr double kProjectionWindowMarginM = 0.20;
constexpr double kEndSpeedToleranceMps = 0.20;

}  // namespace

TrajectorySession::TrajectorySession()
  : traj_(nullptr),
    projected_s_(0.0),
    target_s_(0.0),
    cross_track_error_(std::numeric_limits<double>::infinity()),
    projection_valid_(false),
    trajectory_v_max_m_s_(0.0),
    last_generate_cost_ms_(0.0),
    last_failure_kind_(FailureKind::None)
{
}

void TrajectorySession::reset() {
  traj_.reset();
  projected_s_ = 0.0;
  target_s_ = 0.0;
  cross_track_error_ = std::numeric_limits<double>::infinity();
  projection_valid_ = false;
  target_state_ = trajectory_core::TrajectoryState();
  trajectory_v_max_m_s_ = 0.0;
  last_failure_kind_ = FailureKind::None;
  reference_time_ = clock_rate_ = 0.0;
}

bool TrajectorySession::generate(const std::vector<RecordPoint>& points,
                                  const trajectory_core::TrajectoryConfig& config) {
  last_failure_kind_ = FailureKind::None;

  if (!config.validate()) {
    ROS_ERROR("[TrajectorySession] Invalid trajectory configuration");
    last_failure_kind_ = FailureKind::InvalidConfig;
    return false;
  }

  // 检查输入点数
  if (points.size() < 2) {
    ROS_ERROR("[TrajectorySession] Not enough points to generate trajectory: %zu", points.size());
    last_failure_kind_ = FailureKind::Other;
    return false;
  }

  // 记录开始时间
  auto start_time = std::chrono::high_resolution_clock::now();

  // 构造逆序返航点，并由返航路径的水平切线生成机头航向。
  std::vector<RecordPoint> return_points(points.rbegin(), points.rend());
  std::vector<Vec3> return_positions;
  return_positions.reserve(return_points.size());
  for (const auto& point : return_points) {
    return_positions.push_back(point.position);
  }
  const auto return_yaws = computePathTangentYaws(
      return_positions, return_points.front().yaw);

  // 构造轨迹输入waypoints（逆序 + 保留原始采样间隔的相对关系）
  std::vector<trajectory_core::Waypoint> waypoints;
  waypoints.reserve(return_points.size());

  const size_t n = return_points.size();
  const double t_end = points.back().timestamp;

  for (size_t i = 0; i < n; ++i) {
    const auto& pt = return_points[i];

    // 逆序后重新映射到从 0 开始的单调递增时间轴。
    // 这样既满足 trajectory_core 对 timestamp 单调性的要求，
    // 又保留原始录制 dt 间隔用于时间质量诊断。
    double t_new = t_end - pt.timestamp;

    waypoints.emplace_back(pt.position, return_yaws[i], t_new);
  }

  ROS_INFO("[TrajectorySession] Generating trajectory from %zu waypoints (reversed, mirrored timestamps)",
           waypoints.size());

  // 调用trajectory_core生成轨迹
  // 给执行时钟的加减速留一半切向/竖直加速度预算。
  auto execution_config = config;
  execution_config.a_tan_max *= 0.5;
  execution_config.az_up_max *= 0.5;
  execution_config.az_down_max *= 0.5;
  execution_config.enable_output_acc = true;
  auto result = generator_.generate(waypoints, execution_config);

  // 记录结束时间并计算耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  last_generate_cost_ms_ = duration.count() / 1000.0;

  // 检查生成结果
  if (!result.isSuccess()) {
    ROS_ERROR("[TrajectorySession] Trajectory generation failed: %s", result.message.c_str());
    ROS_ERROR("[TrajectorySession]   Status: %s",
              trajectory_core::statusCodeToString(result.code).c_str());
    if (result.code == trajectory_core::StatusCode::INVALID_INPUT &&
        result.message.find("too close together") != std::string::npos) {
      last_failure_kind_ = FailureKind::TooCloseGeometry;
    } else if (result.code == trajectory_core::StatusCode::INVALID_INPUT &&
               result.message.find("configuration") != std::string::npos) {
      last_failure_kind_ = FailureKind::InvalidConfig;
    } else {
      last_failure_kind_ = FailureKind::Other;
    }
    return false;
  }

  // 检查警告
  if (result.hasTimeFallback()) {
    ROS_WARN("[TrajectorySession] Timestamp fallback: %s", result.message.c_str());
  }
  if (result.isCorridorExceeded()) {
    ROS_WARN("[TrajectorySession] Corridor constraint exceeded");
  }

  // 保存轨迹
  traj_ = std::move(result.traj);

  // 初始化空间进度和首个合法目标。首次 odometry 投影将在 ReturnActive 中更新。
  projected_s_ = 0.0;
  target_s_ = 0.0;
  cross_track_error_ = std::numeric_limits<double>::infinity();
  projection_valid_ = false;
  reference_time_ = clock_rate_ = 0.0;
  clock_acc_limit_ = 0.5 * std::min({config.a_tan_max, config.az_up_max, config.az_down_max}) / config.v_max;
  output_acceleration_ = config.enable_output_acc;
  target_state_ = traj_->evaluateProgress({0.0, 0.0, 0.0, 0.0});
  if (!output_acceleration_) target_state_.acceleration.reset();
  trajectory_v_max_m_s_ = config.v_max;

  // 输出INFO日志（用于验收和debug）
  ROS_INFO("[TrajectorySession] Trajectory generated successfully!");
  ROS_INFO("[TrajectorySession]   Generation time: %.2f ms", last_generate_cost_ms_);
  ROS_INFO("[TrajectorySession]   Duration: %.2f s", result.report.profile.total_duration_s);
  ROS_INFO("[TrajectorySession]   Path length: %.2f m", result.report.profile.path_length_m);
  ROS_INFO("[TrajectorySession]   Max speed: %.2f m/s", result.report.profile.max_speed_m_s);
  ROS_INFO("[TrajectorySession]   Max yaw rate: %.2f rad/s", result.report.yaw.max_yaw_rate_rad_s);
  ROS_INFO("[TrajectorySession]   Time mode: %s",
           trajectory_core::timeModeToString(result.report.time.time_mode).c_str());

  // 检查走廊约束
  if (!result.report.fit.fit_converged) {
    ROS_WARN("[TrajectorySession]   Corridor constraint may be violated: max_deviation=%.3f m",
             result.report.fit.max_deviation_m);
  }

  return true;
}

void TrajectorySession::suspend() {
  clock_rate_ = 0.0;
  target_state_.velocity.setZero();
  target_state_.yaw_rate = 0.0;
  if (target_state_.acceleration) target_state_.acceleration->setZero();
}

double TrajectorySession::arcAtTime(double time) const {
  if (time <= 0.0) return 0.0;
  if (time >= traj_->duration()) return traj_->pathLength();
  double lo = 0.0, hi = traj_->pathLength();
  for (int i = 0; i < 45; ++i) {
    const double mid = (lo + hi) * 0.5;
    if (traj_->evaluateByArcLength(mid).t < time) lo = mid;
    else hi = mid;
  }
  return (lo + hi) * 0.5;
}

bool TrajectorySession::updateProgress(const Vec3& current_position,
                                       double lookahead_time_s, double dt_s) {
  return updateProgressImpl(current_position, lookahead_time_s, 1.0, 0.0,
                            dt_s, trajectory_v_max_m_s_);
}

bool TrajectorySession::updateRejoinProgress(
    const Vec3& current_position, double lookahead_time_s,
    double trigger_distance_m, double dt_s, double speed_limit_m_s) {
  return updateProgressImpl(current_position, lookahead_time_s, 2.0,
                            trigger_distance_m, dt_s, speed_limit_m_s);
}

bool TrajectorySession::updateProgressImpl(
    const Vec3& current_position, double lookahead_time_s,
    double forward_window_scale, double lookahead_scale_distance_m,
    double dt_s, double speed_limit_m_s) {
  if (!isValid() || !current_position.allFinite() ||
      !std::isfinite(lookahead_time_s) || lookahead_time_s < 0.0 ||
      !std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.10 ||
      !std::isfinite(speed_limit_m_s) || speed_limit_m_s < 0.0) {
    projection_valid_ = false;
    suspend();
    return false;
  }
  const double path_length = traj_->pathLength();
  const double forward_window = std::max(kLookaheadMaxM,
      trajectory_v_max_m_s_ * kProjectionCycleHorizonS + kProjectionWindowMarginM);
  const auto projection = traj_->projectPosition(current_position,
      std::max(0.0, projected_s_ - kProjectionBackwardWindowM),
      std::min(path_length, projected_s_ + forward_window * forward_window_scale));
  if (!projection.valid) {
    projection_valid_ = false;
    suspend();
    return false;
  }
  projected_s_ = std::max(projected_s_, projection.arc_length);
  cross_track_error_ = projection.distance;
  projection_valid_ = true;

  // 空间预瞄只决定连续时钟的前进边界，绝不直接跳变位置目标。
  const double scale = lookahead_scale_distance_m > 0.0
      ? std::clamp(1.0-cross_track_error_/lookahead_scale_distance_m, 0.0, 1.0) : 1.0;
  const double preview = std::clamp(trajectory_v_max_m_s_ * lookahead_time_s * scale,
                                  kLookaheadMinM, 0.20);
  const double allowed_s = std::max(target_s_, std::min(path_length, projected_s_+preview));
  const double allowed_time = traj_->evaluateByArcLength(allowed_s).t;
  const double remaining_time = std::max(0.0, allowed_time-reference_time_);
  const double desired_rate = std::min({1.0,
      speed_limit_m_s / std::max(trajectory_v_max_m_s_, 1e-9),
      std::sqrt(2.0 * clock_acc_limit_ * remaining_time)});
  const double old_rate = clock_rate_;
  clock_rate_ += std::clamp(desired_rate-clock_rate_,
                           -clock_acc_limit_*dt_s, clock_acc_limit_*dt_s);
  const double clock_acc = (clock_rate_-old_rate)/dt_s;
  reference_time_ = std::min(allowed_time,
      reference_time_ + 0.5*(old_rate+clock_rate_)*dt_s);
  if (remaining_time <= 1e-10) clock_rate_ = 0.0;
  target_s_ = arcAtTime(reference_time_);
  const auto timed = traj_->evaluate(reference_time_);
  const double speed = timed.velocity.norm();
  const double nominal_tangent_acc = timed.acceleration && speed > 1e-9
      ? timed.velocity.dot(*timed.acceleration) / speed : 0.0;
  target_state_ = traj_->evaluateProgress({reference_time_, target_s_,
      speed*clock_rate_, nominal_tangent_acc*clock_rate_*clock_rate_ + speed*clock_acc});
  if (!output_acceleration_) target_state_.acceleration.reset();
  return true;
}

const trajectory_core::TrajectoryState& TrajectorySession::getTargetState() const {
  return target_state_;
}

bool TrajectorySession::isFinished(const Vec3& current_position,
                                   const Vec3& current_velocity,
                                   double position_tolerance_m) const {
  if (!traj_ || !traj_->isValid() || !projection_valid_ ||
      !std::isfinite(position_tolerance_m) || position_tolerance_m < 0.0 || !current_position.allFinite() ||
      !current_velocity.allFinite()) {
    return false;
  }

  const double remaining_s = std::max(0.0, traj_->pathLength() - projected_s_);
  if (remaining_s > position_tolerance_m ||
      traj_->pathLength()-target_s_ > position_tolerance_m) {
    return false;
  }

  const double speed = current_velocity.norm();
  return (current_position - traj_->goalPosition()).norm() <= position_tolerance_m &&
         speed <= kEndSpeedToleranceMps;
}

bool TrajectorySession::isValid() const {
  return traj_ && traj_->isValid();
}

const trajectory_core::Trajectory* TrajectorySession::getTrajectory() const {
  return traj_.get();
}

double TrajectorySession::getProjectedArcLength() const {
  return projected_s_;
}

double TrajectorySession::getTargetArcLength() const {
  return target_s_;
}

double TrajectorySession::getCrossTrackError() const {
  return cross_track_error_;
}

bool TrajectorySession::hasValidProjection() const {
  return projection_valid_;
}

double TrajectorySession::getLastGenerateCostMs() const {
  return last_generate_cost_ms_;
}

double TrajectorySession::getDuration() const {
  if (traj_ && traj_->isValid()) {
    return traj_->duration();
  }
  return 0.0;
}

TrajectorySession::FailureKind TrajectorySession::getLastFailureKind() const {
  return last_failure_kind_;
}

}  // namespace domain
}  // namespace px4_backtrack
