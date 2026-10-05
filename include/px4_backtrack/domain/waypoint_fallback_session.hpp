#pragma once

#include "recording_buffer.hpp"
#include "state_machine.hpp"

#include <vector>

namespace px4_backtrack {
namespace domain {

struct FallbackWaypoint {
  Vec3 position;
  double yaw;

  FallbackWaypoint(const Vec3& pos, double yaw_in)
    : position(pos), yaw(yaw_in) {}
};

/// 轨迹生成失败后的 waypoint 退化返航会话
class WaypointFallbackSession {
public:
  WaypointFallbackSession(double spacing_m = 0.3, double reach_radius_m = 0.05);

  /// 从录制点构建逆序 waypoint 路径，保留所有非共线顶点
  bool build(const std::vector<RecordPoint>& points);

  /// 清空退化会话
  void reset();

  /// 从头开始执行退化路径（暂停恢复时使用）
  void restart();

  /// 根据当前位置推进 waypoint 索引
  void updateProgress(const Vec3& current_position,
                      const Vec3& current_velocity, double current_yaw);

  /// 是否已构建有效退化路径
  bool isValid() const;

  /// 是否已执行完成
  bool isFinished() const;

  /// 获取当前目标 setpoint
  SetpointTarget currentTarget() const;

  /// 获取稀疏 waypoint 数量
  size_t size() const;

  /// 获取当前目标索引
  size_t currentIndex() const;

private:
  double spacing_m_;
  double reach_radius_m_;
  std::vector<FallbackWaypoint> waypoints_;
  size_t current_index_;
  bool finished_;
};

}  // namespace domain
}  // namespace px4_backtrack
