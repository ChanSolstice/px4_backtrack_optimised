#pragma once

#include "px4_backtrack/domain/types.hpp"
#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <functional>
#include <string>
#include <vector>

namespace px4_backtrack {
namespace adapters {

// 订阅器只做消息转换，收到消息即调用回调。节点是单线程 ros::spin，回调与控制周期串行，
// 回调在开始 spin 之前设置，不需要加锁，也不缓存数据。

/// 按 nav_msgs/Odometry 语义，将 child_frame_id 中的线速度旋转到父 ENU 坐标系。
domain::Vec3 velocityInParentFrame(const nav_msgs::Odometry& msg);

/// MAVROS State 订阅者：回调 (connected, armed, mode)
class StateSubscriber {
public:
  explicit StateSubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/state");
  void setCallback(std::function<void(bool, bool, const std::string&)> callback);

private:
  ros::Subscriber sub_;
  std::function<void(bool, bool, const std::string&)> callback_;
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
};

/// 里程计订阅者：回调 (position ENU, velocity 父 ENU, yaw, stamp)
class OdomSubscriber {
public:
  explicit OdomSubscriber(ros::NodeHandle& nh, const std::string& topic = "/Odometry_px4");
  void setCallback(std::function<void(const domain::Vec3&, const domain::Vec3&, double, double)>
                       callback);

private:
  ros::Subscriber sub_;
  std::function<void(const domain::Vec3&, const domain::Vec3&, double, double)> callback_;
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

/// RC 输入订阅者：回调 (channels)
class RCSubscriber {
public:
  explicit RCSubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/rc/in");
  void setCallback(std::function<void(const std::vector<uint16_t>&)> callback);

private:
  ros::Subscriber sub_;
  std::function<void(const std::vector<uint16_t>&)> callback_;
  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
};

/// 电池状态订阅者：回调 (percentage, 0.0~1.0)
class BatterySubscriber {
public:
  explicit BatterySubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/battery");
  void setCallback(std::function<void(double)> callback);

private:
  ros::Subscriber sub_;
  std::function<void(double)> callback_;
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
};

/// MAVROS ExtendedState 订阅者：回调 (landed_state)，飞控落地状态（重构方案第 5.4 节）
class ExtendedStateSubscriber {
public:
  explicit ExtendedStateSubscriber(ros::NodeHandle& nh,
                                   const std::string& topic = "/mavros/extended_state");
  void setCallback(std::function<void(uint8_t)> callback);

private:
  ros::Subscriber sub_;
  std::function<void(uint8_t)> callback_;
  void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
};

}  // namespace adapters
}  // namespace px4_backtrack
