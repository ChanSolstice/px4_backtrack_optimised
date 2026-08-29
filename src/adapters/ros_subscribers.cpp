#include "px4_backtrack/adapters/ros_subscribers.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// StateSubscriber实现
// ============================================================================

StateSubscriber::StateSubscriber(ros::NodeHandle& nh, const std::string& topic)
  : armed_(false),
    has_data_(false)
{
  sub_ = nh.subscribe(topic, 10, &StateSubscriber::stateCallback, this);
  ROS_INFO("[StateSubscriber] Subscribed to: %s", topic.c_str());
}

bool StateSubscriber::getLatest(bool& armed, std::string& mode) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_data_) {
    return false;
  }
  armed = armed_;
  mode = mode_;
  return true;
}

void StateSubscriber::setCallback(std::function<void(bool, const std::string&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  external_callback_ = callback;
}

void StateSubscriber::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  std::function<void(bool, const std::string&)> cb;
  bool armed;
  std::string mode;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = msg->armed;
    mode_ = msg->mode;
    has_data_ = true;

    armed = armed_;
    mode = mode_;
    cb = external_callback_;
  }

  if (cb) {
    cb(armed, mode);
  }
}

// ============================================================================
// OdomSubscriber实现
// ============================================================================

OdomSubscriber::OdomSubscriber(ros::NodeHandle& nh, const std::string& topic)
  : position_(domain::Vec3::Zero()),
    velocity_(domain::Vec3::Zero()),
    yaw_(0.0),
    stamp_(0.0),
    has_data_(false)
{
  sub_ = nh.subscribe(topic, 10, &OdomSubscriber::odomCallback, this);
  ROS_INFO("[OdomSubscriber] Subscribed to: %s", topic.c_str());
}

bool OdomSubscriber::getLatest(domain::Vec3& position,
                               domain::Vec3& velocity,
                               double& yaw,
                               double& stamp) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_data_) {
    return false;
  }
  position = position_;
  velocity = velocity_;
  yaw = yaw_;
  stamp = stamp_;
  return true;
}

void OdomSubscriber::setCallback(
    std::function<void(const domain::Vec3&,
                       const domain::Vec3&,
                       double,
                       double)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  external_callback_ = callback;
}

void OdomSubscriber::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 提取位置 (ENU)
  domain::Vec3 pos(msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z);
  domain::Vec3 velocity(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);

  // 提取yaw角（从四元数转换）
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // 提取时间戳
  double stamp = msg->header.stamp.toSec();

  // 更新内部状态（mutex保护）
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = pos;
    velocity_ = velocity;
    yaw_ = yaw;
    stamp_ = stamp;
    has_data_ = true;
  }

  // 调用外部回调（如果设置）
  std::function<void(const domain::Vec3&,
                     const domain::Vec3&,
                     double,
                     double)> cb;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cb = external_callback_;
  }
  if (cb) {
    cb(pos, velocity, yaw, stamp);
  }
}

// ============================================================================
// RCSubscriber实现
// ============================================================================

RCSubscriber::RCSubscriber(ros::NodeHandle& nh, const std::string& topic)
  : has_data_(false)
{
  sub_ = nh.subscribe(topic, 10, &RCSubscriber::rcCallback, this);
  ROS_INFO("[RCSubscriber] Subscribed to: %s", topic.c_str());
}

bool RCSubscriber::getLatest(std::vector<uint16_t>& channels) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_data_) {
    return false;
  }
  channels = channels_;
  return true;
}

void RCSubscriber::setCallback(std::function<void(const std::vector<uint16_t>&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  external_callback_ = callback;
}

void RCSubscriber::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
  std::function<void(const std::vector<uint16_t>&)> cb;
  std::vector<uint16_t> channels;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    channels_ = msg->channels;
    has_data_ = true;

    channels = channels_;
    cb = external_callback_;
  }

  if (cb) {
    cb(channels);
  }
}

}  // namespace adapters
}  // namespace px4_backtrack
