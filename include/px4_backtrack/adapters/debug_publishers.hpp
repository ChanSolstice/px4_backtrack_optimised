#pragma once

#include "px4_backtrack/domain/recording_buffer.hpp"
#include <trajectory_core/trajectory_core.hpp>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

namespace px4_backtrack {
namespace adapters {

/// Debug 可视化发布器 — 在 rviz 中展示录制航点和返航轨迹
class DebugPublisher {
public:
  /// 构造函数
  /// @param nh ROS 节点句柄
  explicit DebugPublisher(ros::NodeHandle& nh);

  /// 发布录制航点 (MarkerArray: 球体 + 连线)
  void publishWaypoints(const std::vector<domain::RecordPoint>& points);

  /// 发布采样轨迹 (nav_msgs/Path)
  void publishTrajectory(const std::vector<trajectory_core::TrajectoryState>& states);

private:
  ros::Publisher waypoints_pub_;   ///< visualization_msgs::MarkerArray (latched)
  ros::Publisher trajectory_pub_;  ///< nav_msgs::Path (latched)
};

}  // namespace adapters
}  // namespace px4_backtrack
