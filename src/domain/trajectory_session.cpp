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
constexpr double kEndHorizontalSpeedToleranceMps = 0.20;

std::vector<RecordPoint> thinRecordedPointsForGeneration(
    const std::vector<RecordPoint>& points,
    double min_spacing_m) {
  if (points.size() <= 2 || min_spacing_m <= 0.0) {
    return points;
  }

  std::vector<RecordPoint> reduced;
  reduced.reserve(points.size());
  reduced.push_back(points.front());

  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const auto& point = points[i];
    double dist = (point.position - reduced.back().position).norm();

    if (dist >= min_spacing_m) {
      reduced.push_back(point);
    }
  }

  const auto& endpoint = points.back();
  while (reduced.size() > 1 &&
         (endpoint.position - reduced.back().position).norm() < min_spacing_m) {
    reduced.pop_back();
  }

  if ((endpoint.position - reduced.back().position).norm() > 1e-9) {
    reduced.push_back(endpoint);
  } else {
    reduced.back() = endpoint;
  }

  return reduced;
}

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

  const double min_spacing_m = std::max(config.min_point_dist_m, config.ds_m * 0.5);
  auto reduced_points = thinRecordedPointsForGeneration(points, min_spacing_m);

  if (reduced_points.size() < 2) {
    ROS_ERROR("[TrajectorySession] Too few points after thinning: %zu", reduced_points.size());
    last_failure_kind_ = FailureKind::TooCloseGeometry;
    return false;
  }

  if (reduced_points.size() != points.size()) {
    ROS_INFO("[TrajectorySession] Thinned recorded points: %zu -> %zu (min_spacing=%.3f m)",
             points.size(),
             reduced_points.size(),
             min_spacing_m);
  }

  // 构造逆序返航点，并由返航路径的水平切线生成机头航向。
  std::vector<RecordPoint> return_points(reduced_points.rbegin(), reduced_points.rend());
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
  const double t_end = reduced_points.back().timestamp;

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
  auto result = generator_.generate(waypoints, config);

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
  target_state_ = traj_->evaluateByArcLength(0.0);
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

bool TrajectorySession::updateProgress(const Vec3& current_position,
                                       double lookahead_time_s) {
  return updateProgressImpl(current_position, lookahead_time_s, 1.0, 0.0);
}

bool TrajectorySession::updateRejoinProgress(
    const Vec3& current_position,
    double lookahead_time_s,
    double trigger_distance_m) {
  return updateProgressImpl(
      current_position, lookahead_time_s, 2.0, trigger_distance_m);
}

bool TrajectorySession::updateProgressImpl(
    const Vec3& current_position,
    double lookahead_time_s,
    double forward_window_scale,
    double lookahead_scale_distance_m) {
  if (!traj_ || !traj_->isValid() || !current_position.allFinite() ||
      !std::isfinite(lookahead_time_s) ||
      !std::isfinite(forward_window_scale) ||
      !std::isfinite(lookahead_scale_distance_m) ||
      forward_window_scale <= 0.0) {
    projection_valid_ = false;
    return false;
  }

  const double path_length = traj_->pathLength();
  const auto projection = projectLocal(current_position, forward_window_scale);
  if (!projection.valid) {
    projection_valid_ = false;
    return false;
  }

  // 正常路径跟随只允许空间进度前进，避免噪声和自交附近投影导致回退。
  projected_s_ = std::max(projected_s_, projection.arc_length);
  cross_track_error_ = projection.distance;
  projection_valid_ = true;

  const auto projected_state = traj_->evaluateByArcLength(projected_s_);
  double lookahead_scale = 1.0;
  if (lookahead_scale_distance_m > 0.0) {
    lookahead_scale = std::clamp(
        1.0 - cross_track_error_ / lookahead_scale_distance_m, 0.0, 1.0);
  }
  const double lookahead_m = std::clamp(
      projected_state.velocity.norm() * std::max(0.0, lookahead_time_s) *
          lookahead_scale,
      kLookaheadMinM,
      kLookaheadMaxM);
  const double candidate_target_s =
      std::min(projected_s_ + lookahead_m, path_length);
  // 目标弧长同样不回退，Rejoin 通过实际进度追上旧预瞄目标来平滑缩短预瞄。
  target_s_ = std::max(target_s_, candidate_target_s);
  target_state_ = traj_->evaluateByArcLength(target_s_);
  return true;
}

trajectory_core::ProjectionResult TrajectorySession::projectLocal(
    const Vec3& position, double forward_window_scale) const {
  const double forward_window = std::max(
      kLookaheadMaxM,
      trajectory_v_max_m_s_ * kProjectionCycleHorizonS +
          kProjectionWindowMarginM);
  const double s_min = std::max(0.0, projected_s_ - kProjectionBackwardWindowM);
  const double s_max = std::min(
      traj_->pathLength(), projected_s_ + forward_window * forward_window_scale);
  return traj_->projectPosition(position, s_min, s_max);
}

bool TrajectorySession::probe(const Vec3& position, double& cross_track_m) const {
  if (!traj_ || !traj_->isValid() || !position.allFinite()) {
    return false;
  }
  const auto projection = projectLocal(position, 1.0);
  if (projection.valid) {
    cross_track_m = projection.distance;
  }
  return projection.valid;
}

const trajectory_core::TrajectoryState& TrajectorySession::getTargetState() const {
  return target_state_;
}

bool TrajectorySession::isFinished(const Vec3& current_position,
                                   const Vec3& current_velocity,
                                   double position_tolerance_m) const {
  if (!traj_ || !traj_->isValid() || !projection_valid_ ||
      position_tolerance_m < 0.0 || !current_position.allFinite() ||
      !current_velocity.allFinite()) {
    return false;
  }

  const double remaining_s = std::max(0.0, traj_->pathLength() - projected_s_);
  if (remaining_s > position_tolerance_m) {
    return false;
  }

  const double horizontal_speed = current_velocity.head<2>().norm();
  return (current_position - traj_->goalPosition()).norm() <= position_tolerance_m &&
         horizontal_speed <= kEndHorizontalSpeedToleranceMps;
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
