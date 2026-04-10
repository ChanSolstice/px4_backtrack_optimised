#pragma once

#include "recording_buffer.hpp"
#include <trajectory_core/trajectory_core.hpp>
#include <memory>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// TrajectorySession类
// ============================================================================

/// 轨迹会话管理（生成、进度跟踪、评估）
class TrajectorySession {
public:
  enum class FailureKind {
    None,
    TooCloseGeometry,
    Other,
  };

  /// 构造函数
  TrajectorySession();

  /// 重置会话（清空轨迹和进度）
  void reset();

  /// 生成轨迹（从录制点）
  /// @param points 录制点序列
  /// @param config trajectory_core配置
  /// @return 是否生成成功
  bool generate(const std::vector<RecordPoint>& points,
                const trajectory_core::TrajectoryConfig& config);

  /// 评估当前时刻的轨迹状态
  /// @param current_walltime 当前wall time (s)
  /// @return 轨迹状态
  trajectory_core::TrajectoryState evaluate(double current_walltime);

  /// 检查轨迹是否完成
  /// @return 是否完成
  bool isFinished() const;

  /// 检查轨迹是否有效
  /// @return 是否有效
  bool isValid() const;

  /// 获取轨迹对象（只读）
  /// @return 轨迹对象指针（可能为nullptr）
  const trajectory_core::Trajectory* getTrajectory() const;

  /// 获取当前进度时间
  /// @return 进度时间 (s)
  double getProgress() const;

  /// 设置进度时间（用于暂停/恢复）
  /// @param t_progress 进度时间 (s)
  void setProgress(double t_progress);

  /// 冻结进度（暂停时调用）
  void freezeProgress();

  /// 恢复进度（从暂停恢复时调用）
  /// @param current_walltime 当前wall time (s)
  void resumeProgress(double current_walltime);

  /// 获取最近一次生成轨迹的耗时
  /// @return 耗时 (ms)
  double getLastGenerateCostMs() const;

  /// 获取轨迹总时长
  /// @return 总时长 (s)，如果无效返回0
  double getDuration() const;

  /// 获取最近一次生成失败类型
  FailureKind getLastFailureKind() const;

private:
  /// 轨迹对象
  std::unique_ptr<trajectory_core::Trajectory> traj_;

  /// 当前进度时间（轨迹内部时间）
  double t_progress_;

  /// 开始执行时的wall time
  double t_start_walltime_;

  /// 进度是否冻结（暂停时为true）
  bool progress_frozen_;

  /// 冻结时的进度时间
  double frozen_progress_;

  /// 最近一次生成轨迹的耗时 (ms)
  double last_generate_cost_ms_;

  /// 最近一次生成失败类型
  FailureKind last_failure_kind_;

  /// 轨迹生成器（复用）
  trajectory_core::TrajectoryGenerator generator_;
};

}  // namespace domain
}  // namespace px4_backtrack
