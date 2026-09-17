#pragma once

#include "state_machine.hpp"
#include <trajectory_core/trajectory_core.hpp>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// RejoinPolicy配置参数
struct RejoinConfig {
  double trigger_dist_m;      ///< 触发回归的偏离距离 (m)
  double success_dist_m;      ///< 成功回归的距离阈值 (m)
  double v_max_m_s;           ///< 回归时的最大速度 (m/s)
  double lookahead_time_s;    ///< 预瞄时间 (s)
  double trigger_hold_s;      ///< 偏离持续超阈值多久后进入rejoin (s)
  double success_hold_s;      ///< 回归持续满足阈值多久后退出rejoin (s)

  /// 默认构造函数
  RejoinConfig()
    : trigger_dist_m(0.5),
      success_dist_m(0.4),
      v_max_m_s(0.3),
      lookahead_time_s(0.5),
      trigger_hold_s(0.25),
      success_hold_s(0.10)
  {}
};

// ============================================================================
// RejoinPolicy类
// ============================================================================

/// 轨迹回归策略（偏离检测与回归目标计算）
class RejoinPolicy {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit RejoinPolicy(const RejoinConfig& config);

  /// 判断是否需要触发回归
  /// @param cross_track_error 当前位置到局部路径投影点的距离 (m)
  /// @return 是否需要回归
  bool shouldTriggerRejoin(double cross_track_error) const;

  /// 判断是否完成回归（可以退出回归状态）
  /// @param cross_track_error 当前位置到局部路径投影点的距离 (m)
  /// @param current_velocity 飞机实际速度向量 (m/s)
  /// @param path_target_velocity 恢复后的路径目标速度向量 (m/s)
  /// @param published_target_velocity 上周期已发布的目标速度向量 (m/s)
  /// @return 是否完成回归
  bool shouldExitRejoin(double cross_track_error,
                        const Vec3& current_velocity,
                        const Vec3& path_target_velocity,
                        const Vec3& published_target_velocity) const;

  /// 对路径目标施加 Rejoin 平滑限速，不改变位置和路径切向 yaw
  /// @param path_target 原始路径目标
  /// @param cross_track_error 当前横向误差
  /// @param previous_target_velocity 上周期已发布的目标速度
  trajectory_core::TrajectoryState applyRejoinLimits(
      const trajectory_core::TrajectoryState& path_target,
      double cross_track_error,
      const Vec3& previous_target_velocity) const;

private:
  /// 配置参数
  RejoinConfig config_;

  /// 允许退出 Rejoin 的内部速度误差阈值 (m/s)
  static constexpr double kExitVelocityErrorMps = 0.30;

  /// 单周期参考速度向量最大变化量 (m/s)
  static constexpr double kMaxVelocityStepMps = 0.30;
};

}  // namespace domain
}  // namespace px4_backtrack
