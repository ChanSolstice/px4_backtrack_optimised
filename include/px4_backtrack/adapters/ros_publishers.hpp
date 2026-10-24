#pragma once

#include "px4_backtrack/domain/types.hpp"
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetpointPublisher类
// ============================================================================

/// Setpoint发布者
class SetpointPublisher {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 发布话题名
  explicit SetpointPublisher(ros::NodeHandle& nh,
                             const std::string& topic = "/mavros/setpoint_raw/local");

  /// 发布setpoint目标
  /// @param target setpoint目标
  void publish(const domain::SetpointTarget& target);

private:
  /// ROS发布者
  ros::Publisher pub_;

  /// 将SetpointTarget转换为PositionTarget消息
  mavros_msgs::PositionTarget toPositionTargetMsg(const domain::SetpointTarget& target) const;
};

// ============================================================================
// StatusPublisher类
// ============================================================================

/// 状态信息发布者
class StatusPublisher {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  explicit StatusPublisher(ros::NodeHandle& nh);

  /// 发布当前状态
  /// @param state 状态字符串（重构方案第 6.1 节）
  void publishState(const std::string& state);

  /// 发布RC丢失状态
  /// @param lost 是否丢失
  void publishRCLoss(bool lost);

  /// 发布低电量状态
  /// @param low_battery 是否低电量
  void publishLowBattery(bool low_battery);

  /// 发布状态 JSON（/px4_backtrack/status，重构方案第 9.1 节）
  void publishStatus(const std::string& json);

private:
  /// ROS发布者
  ros::Publisher state_pub_;
  ros::Publisher rc_loss_pub_;
  ros::Publisher low_battery_pub_;
  ros::Publisher status_json_pub_;
};

}  // namespace adapters
}  // namespace px4_backtrack
