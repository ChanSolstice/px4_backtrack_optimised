#pragma once

#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/debug_log.hpp"
#include <ros/ros.h>
#include <string>

namespace px4_backtrack {
namespace infrastructure {

// ============================================================================
// ConfigLoader类
// ============================================================================

/// 配置加载器：参数全部必填，缺失或非法时抛异常（重构方案第 10 节）
class ConfigLoader {
public:
  /// 从ROS参数服务器加载返航配置
  /// @param nh 节点私有句柄
  static application::UseCaseConfig loadFromROS(ros::NodeHandle& nh);

  /// 从ROS参数服务器加载调试日志配置
  /// @param nh 节点私有句柄
  static DebugLogConfig loadDebugLog(ros::NodeHandle& nh);

  /// 从ROS参数服务器加载控制周期频率（节点定时器使用）
  /// @param nh 节点私有句柄
  static double loadPublishHz(ros::NodeHandle& nh);

  /// 从YAML文件加载trajectory_core配置
  /// @param path YAML文件路径
  static trajectory_core::TrajectoryConfig loadTrajectoryConfig(const std::string& path);
};

}  // namespace infrastructure
}  // namespace px4_backtrack
