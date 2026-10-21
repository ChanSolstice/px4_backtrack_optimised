#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Dense>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 类型别名
// ============================================================================

/// 三维向量（ENU坐标系）
using Vec3 = Eigen::Vector3d;

// ============================================================================
// Setpoint目标数据结构
// ============================================================================

namespace position_target_mask {

/// 忽略加速度/力字段，保留位置、速度、yaw和yaw_rate控制
constexpr uint16_t kIgnoreAcceleration = 0x01C0;

}  // namespace position_target_mask

/// Setpoint目标（领域对象）
struct SetpointTarget {
  /// 位置目标 (ENU, m)
  Vec3 position;

  /// 速度目标 (ENU, m/s)
  Vec3 velocity;

  /// Yaw角目标 (rad)
  double yaw;

  /// Yaw角速度目标 (rad/s)
  double yaw_rate;

  /// 加速度目标（可选，ENU, m/s²）
  std::optional<Vec3> acceleration;

  /// type_mask字段（用于PositionTarget消息）
  uint16_t type_mask;

  /// coordinate_frame字段（用于PositionTarget消息）
  uint8_t coordinate_frame;

  /// 默认构造函数
  SetpointTarget()
    : position(Vec3::Zero()),
      velocity(Vec3::Zero()),
      yaw(0.0),
      yaw_rate(0.0),
      acceleration(std::nullopt),
      type_mask(0),
      coordinate_frame(1)  // FRAME_LOCAL_NED
  {}
};

}  // namespace domain
}  // namespace px4_backtrack
