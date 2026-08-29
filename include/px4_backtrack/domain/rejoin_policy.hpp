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
      success_dist_m(0.1),
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
  /// @return 是否完成回归
  bool shouldExitRejoin(double cross_track_error) const;

  /// 查找轨迹上距离当前位置最近的时间点
  /// 当前 ReturnRejoin 主链路不再使用最近点预瞄；该接口保留给测试和后续策略扩展。
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj 轨迹对象
  /// @return 最近点的时间 (s)
  double findNearestTime(const Vec3& current_pos,
                         const trajectory_core::Trajectory* traj) const;

  /// 计算最近点预瞄回归目标状态
  /// 当前 ReturnRejoin 主链路不再使用该策略；实际控制使用 computeFrozenPointTarget()。
  /// @param current_pos 当前位置 (ENU, m)
  /// @param traj 轨迹对象
  /// @param t_nearest 最近点的时间 (s)
  /// @return 回归目标状态（包含位置、速度、yaw等）
  trajectory_core::TrajectoryState computeRejoinTarget(
      const Vec3& current_pos,
      const trajectory_core::Trajectory* traj,
      double t_nearest) const;

  /// 计算回归到冻结进度点的目标状态
  /// @param current_pos 当前位置 (ENU, m)
  /// @param frozen_state 进入 rejoin 时冻结进度对应的轨迹状态
  /// @return 目标状态：位置/yaw 复用冻结点，速度指向冻结点并按距离比例收敛
  trajectory_core::TrajectoryState computeFrozenPointTarget(
      const Vec3& current_pos,
      const trajectory_core::TrajectoryState& frozen_state) const;

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
