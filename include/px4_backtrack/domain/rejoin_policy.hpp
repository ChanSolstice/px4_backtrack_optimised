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

  /// 默认构造函数
  RejoinConfig()
    : trigger_dist_m(0.5),
      success_dist_m(0.1),
      v_max_m_s(0.3),
      lookahead_time_s(0.5)
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
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj_pos 轨迹上的当前进度位置 (ENU, m)
  /// @return 是否需要回归
  bool shouldTriggerRejoin(const Vec3& current_pos, const Vec3& traj_pos) const;

  /// 判断是否完成回归（可以退出回归状态）
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj_pos 轨迹上的当前进度位置 (ENU, m)
  /// @return 是否完成回归
  bool shouldExitRejoin(const Vec3& current_pos, const Vec3& traj_pos) const;

  /// 查找轨迹上距离当前位置最近的时间点
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj 轨迹对象
  /// @return 最近点的时间 (s)
  double findNearestTime(const Vec3& current_pos,
                         const trajectory_core::Trajectory* traj) const;

  /// 计算回归目标状态
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj 轨迹对象
  /// @param t_nearest 最近点的时间 (s)
  /// @return 回归目标状态（包含位置、速度、yaw等）
  trajectory_core::TrajectoryState computeRejoinTarget(
      const Vec3& current_pos,
      const trajectory_core::Trajectory* traj,
      double t_nearest) const;

private:
  /// 配置参数
  RejoinConfig config_;

  /// 限幅速度向量
  /// @param velocity 速度向量 (m/s)
  /// @param v_max 最大速度 (m/s)
  /// @return 限幅后的速度向量
  Vec3 limitVelocity(const Vec3& velocity, double v_max) const;
};

}  // namespace domain
}  // namespace px4_backtrack
