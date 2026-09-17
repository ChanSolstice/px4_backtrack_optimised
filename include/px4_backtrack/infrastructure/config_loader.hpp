#pragma once

#include "px4_backtrack/application/backtrack_usecase.hpp"
#include <ros/ros.h>
#include <string>

namespace px4_backtrack {
namespace infrastructure {

// ============================================================================
// ConfigLoader类
// ============================================================================

/// 配置加载器
class ConfigLoader {
public:
  /// 从ROS参数服务器加载返航配置
  /// @param nh ROS节点句柄
  /// @return 返航配置
  static application::UseCaseConfig loadFromROS(ros::NodeHandle& nh);

  /// 从YAML文件加载trajectory_core配置
  /// @param path YAML文件路径
  /// @return trajectory_core配置
  static trajectory_core::TrajectoryConfig loadTrajectoryConfig(const std::string& path);

private:
  /// 加载录制参数
  static void loadRecordingParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载通道7参数
  static void loadCh7Params(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载通道18参数
  static void loadCh18Params(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载RC丢失参数
  static void loadRCLossParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载低电量参数
  static void loadLowBatteryParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载OFFBOARD参数
  static void loadOffboardParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载回归参数
  static void loadRejoinParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载降落参数
  static void loadLandingParams(ros::NodeHandle& nh, application::UseCaseConfig& config);

  /// 加载Debug/可视化参数
  static void loadDebugParams(ros::NodeHandle& nh, application::UseCaseConfig& config);
};

}  // namespace infrastructure
}  // namespace px4_backtrack
