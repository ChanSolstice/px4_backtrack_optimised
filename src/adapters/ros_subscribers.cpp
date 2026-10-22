#include "px4_backtrack/adapters/ros_subscribers.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

domain::Vec3 velocityInParentFrame(const nav_msgs::Odometry& msg) {
  const tf2::Quaternion orientation(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
  const tf2::Vector3 velocity_child(
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);
  const tf2::Vector3 velocity_parent =
      tf2::Matrix3x3(orientation) * velocity_child;
  return domain::Vec3(
      velocity_parent.x(), velocity_parent.y(), velocity_parent.z());
}

// ============================================================================
// StateSubscriber
// ============================================================================

StateSubscriber::StateSubscriber(ros::NodeHandle& nh, const std::string& topic) {
  sub_ = nh.subscribe(topic, 10, &StateSubscriber::stateCallback, this);
  ROS_INFO("[StateSubscriber] Subscribed to: %s", topic.c_str());
}

void StateSubscriber::setCallback(std::function<void(bool, bool, const std::string&)> callback) {
  callback_ = std::move(callback);
}

void StateSubscriber::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  if (callback_) {
    callback_(msg->connected, msg->armed, msg->mode);
  }
}

// ============================================================================
// OdomSubscriber
// ============================================================================

OdomSubscriber::OdomSubscriber(ros::NodeHandle& nh, const std::string& topic) {
  sub_ = nh.subscribe(topic, 10, &OdomSubscriber::odomCallback, this);
  ROS_INFO("[OdomSubscriber] Subscribed to: %s", topic.c_str());
}

void OdomSubscriber::setCallback(
    std::function<void(const domain::Vec3&, const domain::Vec3&, double, double)> callback) {
  callback_ = std::move(callback);
}

void OdomSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!callback_) {
    return;
  }
  const domain::Vec3 position(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
  // nav_msgs/Odometry 规定 twist 位于 child_frame_id；领域层统一使用父 ENU
  const domain::Vec3 velocity = velocityInParentFrame(*msg);

  const tf2::Quaternion q(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  callback_(position, velocity, yaw, msg->header.stamp.toSec());
}

// ============================================================================
// RCSubscriber
// ============================================================================

RCSubscriber::RCSubscriber(ros::NodeHandle& nh, const std::string& topic) {
  sub_ = nh.subscribe(topic, 10, &RCSubscriber::rcCallback, this);
  ROS_INFO("[RCSubscriber] Subscribed to: %s", topic.c_str());
}

void RCSubscriber::setCallback(std::function<void(const std::vector<uint16_t>&)> callback) {
  callback_ = std::move(callback);
}

void RCSubscriber::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
  if (callback_) {
    callback_(msg->channels);
  }
}

// ============================================================================
// BatterySubscriber
// ============================================================================

BatterySubscriber::BatterySubscriber(ros::NodeHandle& nh, const std::string& topic) {
  sub_ = nh.subscribe(topic, 10, &BatterySubscriber::batteryCallback, this);
  ROS_INFO("[BatterySubscriber] Subscribed to: %s", topic.c_str());
}

void BatterySubscriber::setCallback(std::function<void(double)> callback) {
  callback_ = std::move(callback);
}

void BatterySubscriber::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  if (callback_) {
    callback_(static_cast<double>(msg->percentage));
  }
}

// ============================================================================
// ExtendedStateSubscriber
// ============================================================================

ExtendedStateSubscriber::ExtendedStateSubscriber(ros::NodeHandle& nh,
                                                 const std::string& topic) {
  sub_ = nh.subscribe(topic, 10, &ExtendedStateSubscriber::extendedStateCallback, this);
  ROS_INFO("[ExtendedStateSubscriber] Subscribed to: %s", topic.c_str());
}

void ExtendedStateSubscriber::setCallback(std::function<void(uint8_t)> callback) {
  callback_ = std::move(callback);
}

void ExtendedStateSubscriber::extendedStateCallback(
    const mavros_msgs::ExtendedState::ConstPtr& msg) {
  if (callback_) {
    callback_(msg->landed_state);
  }
}

}  // namespace adapters
}  // namespace px4_backtrack
