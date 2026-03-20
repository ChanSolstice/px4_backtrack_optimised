#pragma once

#include "px4_backtrack/domain/state_machine.hpp"
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
  /// @param state 状态枚举
  void publishState(domain::State state);

  /// 发布RC丢失状态
  /// @param lost 是否丢失
  void publishRCLoss(bool lost);

private:
  /// ROS发布者
  ros::Publisher state_pub_;
  ros::Publisher rc_loss_pub_;
};

}  // namespace adapters
}  // namespace px4_backtrack
