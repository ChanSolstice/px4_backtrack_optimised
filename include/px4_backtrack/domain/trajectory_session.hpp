#pragma once

#include "recording_buffer.hpp"
#include <trajectory_core/trajectory_core.hpp>
#include <memory>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// TrajectorySession类
// ============================================================================

/// 轨迹会话管理（生成、空间进度跟踪、目标查询）
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

  /// 根据飞机实际位置更新路径投影进度和预瞄目标
  /// @param current_position 飞机当前位置
  /// @param lookahead_time_s 预瞄时间
  /// @return 本周期投影是否成功；失败时保留上一个合法目标
  bool updateProgress(const Vec3& current_position,
                      double lookahead_time_s);

  /// Rejoin 低速模式下更新路径进度：扩大局部前向窗口并随偏离缩短预瞄
  bool updateRejoinProgress(const Vec3& current_position,
                            double lookahead_time_s,
                            double trigger_distance_m);

  /// 当前位置在跟随进度附近的局部投影，只查询、不推进进度（R-74）
  /// @param position 飞机当前位置
  /// @param cross_track_m 输出：投影有效时为横向偏差
  /// @return 投影是否有效
  bool probe(const Vec3& position, double& cross_track_m) const;

  /// 获取当前预瞄目标状态
  const trajectory_core::TrajectoryState& getTargetState() const;

  /// 检查空间进度、终点位置和水平速度是否均已收敛
  /// @param current_position 飞机当前位置
  /// @param current_velocity 飞机当前速度
  /// @param position_tolerance_m 终点位置容差
  /// @return 是否完成
  bool isFinished(const Vec3& current_position,
                  const Vec3& current_velocity,
                  double position_tolerance_m) const;

  /// 检查轨迹是否有效
  /// @return 是否有效
  bool isValid() const;

  /// 获取轨迹对象（只读）
  /// @return 轨迹对象指针（可能为nullptr）
  const trajectory_core::Trajectory* getTrajectory() const;

  /// 获取实际位置在路径上的投影弧长
  double getProjectedArcLength() const;

  /// 获取当前预瞄目标弧长
  double getTargetArcLength() const;

  /// 获取最近一次有效投影的横向距离
  double getCrossTrackError() const;

  /// 最近一次路径投影是否有效
  bool hasValidProjection() const;

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

  /// 实际位置在路径上的单调投影进度
  double projected_s_;

  /// 当前发布目标的路径进度
  double target_s_;

  /// 最近一次有效投影的横向距离
  double cross_track_error_;

  /// 是否已有有效投影
  bool projection_valid_;

  /// 当前预瞄目标状态；投影失败时保持最后合法值
  trajectory_core::TrajectoryState target_state_;

  /// 生成轨迹所用的最大速度，用于约束局部投影窗口
  double trajectory_v_max_m_s_;

  /// 最近一次生成轨迹的耗时 (ms)
  double last_generate_cost_ms_;

  /// 最近一次生成失败类型
  FailureKind last_failure_kind_;

  /// 轨迹生成器（复用）
  trajectory_core::TrajectoryGenerator generator_;

  /// 在当前进度附近的局部窗口内投影
  trajectory_core::ProjectionResult projectLocal(const Vec3& position,
                                                 double forward_window_scale) const;

  /// 空间进度更新的共用实现
  bool updateProgressImpl(const Vec3& current_position,
                          double lookahead_time_s,
                          double forward_window_scale,
                          double lookahead_scale_distance_m);
};

}  // namespace domain
}  // namespace px4_backtrack
