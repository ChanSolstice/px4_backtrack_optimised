#include "px4_backtrack/adapters/ros_publishers.hpp"
#include <std_msgs/Bool.h>
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetpointPublisher实现
// ============================================================================

SetpointPublisher::SetpointPublisher(ros::NodeHandle& nh, const std::string& topic) {
  pub_ = nh.advertise<mavros_msgs::PositionTarget>(topic, 10);
  ROS_INFO("[SetpointPublisher] Publishing to: %s", topic.c_str());
}

void SetpointPublisher::publish(const domain::SetpointTarget& target) {
  mavros_msgs::PositionTarget msg = toPositionTargetMsg(target);
  pub_.publish(msg);
}

mavros_msgs::PositionTarget SetpointPublisher::toPositionTargetMsg(
    const domain::SetpointTarget& target) const {

  mavros_msgs::PositionTarget msg;

  // Header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  // Coordinate frame: FRAME_LOCAL_NED (直接填充ENU值，MAVROS会转换)
  msg.coordinate_frame = target.coordinate_frame;

  // Type mask
  msg.type_mask = target.type_mask;

  // Position (ENU, m)
  msg.position.x = target.position.x();
  msg.position.y = target.position.y();
  msg.position.z = target.position.z();

  // Velocity (ENU, m/s)
  msg.velocity.x = target.velocity.x();
  msg.velocity.y = target.velocity.y();
  msg.velocity.z = target.velocity.z();

  // Acceleration (ENU, m/s^2)
  if (target.acceleration.has_value()) {
    msg.acceleration_or_force.x = target.acceleration.value().x();
    msg.acceleration_or_force.y = target.acceleration.value().y();
    msg.acceleration_or_force.z = target.acceleration.value().z();
  } else {
    msg.acceleration_or_force.x = 0.0;
    msg.acceleration_or_force.y = 0.0;
    msg.acceleration_or_force.z = 0.0;
  }

  // Yaw (rad)
  msg.yaw = target.yaw;

  // Yaw rate (rad/s)
  msg.yaw_rate = target.yaw_rate;

  return msg;
}

// ============================================================================
// StatusPublisher实现
// ============================================================================

StatusPublisher::StatusPublisher(ros::NodeHandle& nh) {
  state_pub_ = nh.advertise<std_msgs::String>("/px4_backtrack/state", 10);
  rc_loss_pub_ = nh.advertise<std_msgs::Bool>("/status_rc_loss", 10);
  ROS_INFO("[StatusPublisher] Initialized");
}

void StatusPublisher::publishState(domain::State state) {
  std_msgs::String msg;
  msg.data = domain::stateToString(state);
  state_pub_.publish(msg);
}

void StatusPublisher::publishRCLoss(bool lost) {
  std_msgs::Bool msg;
  msg.data = lost;
  rc_loss_pub_.publish(msg);
}

}  // namespace adapters
}  // namespace px4_backtrack
