#include "px4_backtrack/domain/trajectory_session.hpp"
#include <ros/console.h>
#include <chrono>
#include <algorithm>

namespace px4_backtrack {
namespace domain {

TrajectorySession::TrajectorySession()
  : traj_(nullptr),
    t_progress_(0.0),
    t_start_walltime_(0.0),
    progress_frozen_(false),
    frozen_progress_(0.0),
    last_generate_cost_ms_(0.0)
{
}

void TrajectorySession::reset() {
  traj_.reset();
  t_progress_ = 0.0;
  t_start_walltime_ = 0.0;
  progress_frozen_ = false;
  frozen_progress_ = 0.0;
}

bool TrajectorySession::generate(const std::vector<RecordPoint>& points,
                                  const trajectory_core::TrajectoryConfig& config) {
  // 检查输入点数
  if (points.size() < 2) {
    ROS_ERROR("[TrajectorySession] Not enough points to generate trajectory: %zu", points.size());
    return false;
  }

  // 记录开始时间
  auto start_time = std::chrono::high_resolution_clock::now();

  // 构造轨迹输入waypoints（逆序 + 保留原始采样间隔的相对关系）
  std::vector<trajectory_core::Waypoint> waypoints;
  waypoints.reserve(points.size());

  const size_t n = points.size();
  const double t_end = points.back().timestamp;

  for (size_t i = 0; i < n; ++i) {
    // 逆序索引
    size_t reverse_idx = n - 1 - i;
    const auto& pt = points[reverse_idx];

    // 逆序后重新映射到从 0 开始的单调递增时间轴。
    // 这样既满足 trajectory_core 对 timestamp 单调性的要求，
    // 又保留原始录制 dt 间隔用于时间质量诊断。
    double t_new = t_end - pt.timestamp;

    // 构造Waypoint
    waypoints.emplace_back(pt.position, pt.yaw, t_new);
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

  // 重置进度
  t_progress_ = 0.0;
  progress_frozen_ = false;
  frozen_progress_ = 0.0;

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

trajectory_core::TrajectoryState TrajectorySession::evaluate(double current_walltime) {
  if (!traj_ || !traj_->isValid()) {
    // 返回空状态
    return trajectory_core::TrajectoryState();
  }

  // 更新进度时间（如果未冻结）
  if (!progress_frozen_) {
    if (t_start_walltime_ == 0.0) {
      // 首次评估，记录开始时间
      t_start_walltime_ = current_walltime;
      t_progress_ = 0.0;
    } else {
      // 基于wall time推进进度
      t_progress_ = current_walltime - t_start_walltime_;
    }
  } else {
    // 冻结状态，使用冻结的进度
    t_progress_ = frozen_progress_;
  }

  // 评估轨迹状态
  return traj_->evaluate(t_progress_);
}

bool TrajectorySession::isFinished() const {
  if (!traj_ || !traj_->isValid()) {
    return false;
  }

  return t_progress_ >= traj_->duration();
}

bool TrajectorySession::isValid() const {
  return traj_ && traj_->isValid();
}

const trajectory_core::Trajectory* TrajectorySession::getTrajectory() const {
  return traj_.get();
}

double TrajectorySession::getProgress() const {
  return t_progress_;
}

void TrajectorySession::setProgress(double t_progress) {
  t_progress_ = t_progress;
}

void TrajectorySession::freezeProgress() {
  progress_frozen_ = true;
  frozen_progress_ = t_progress_;
}

void TrajectorySession::resumeProgress(double current_walltime) {
  progress_frozen_ = false;
  // 重新设置开始时间，使进度从frozen_progress_继续
  t_start_walltime_ = current_walltime - frozen_progress_;
  t_progress_ = frozen_progress_;
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

}  // namespace domain
}  // namespace px4_backtrack
