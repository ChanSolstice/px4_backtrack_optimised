#pragma once

#include "px4_backtrack/domain/types.hpp"
#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <mutex>
#include <functional>

namespace px4_backtrack {
namespace adapters {

/// 按 nav_msgs/Odometry 语义，将 child_frame_id 中的线速度旋转到父 ENU 坐标系。
domain::Vec3 velocityInParentFrame(const nav_msgs::Odometry& msg);

// ============================================================================
// StateSubscriber类
// ============================================================================

/// MAVROS State订阅者（线程安全）
class StateSubscriber {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 订阅话题名
  explicit StateSubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/state");

  /// 获取最新状态（线程安全）
  /// @param connected 输出：MAVROS 是否连上飞控
  /// @param armed 输出：是否ARM
  /// @param mode 输出：当前模式字符串
  /// @return 是否收到过数据
  bool getLatest(bool& connected, bool& armed, std::string& mode) const;

  /// 设置外部回调（可选）
  /// @param callback 回调函数 (connected, armed, mode)
  void setCallback(std::function<void(bool, bool, const std::string&)> callback);

private:
  /// ROS订阅者
  ros::Subscriber sub_;

  /// 最新数据（mutex保护）
  mutable std::mutex mutex_;
  bool connected_;
  bool armed_;
  std::string mode_;
  bool has_data_;

  /// 外部回调（可选）
  std::function<void(bool, bool, const std::string&)> external_callback_;

  /// ROS回调
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
};

// ============================================================================
// OdomSubscriber类
// ============================================================================

/// Odometry订阅者（线程安全）
class OdomSubscriber {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 订阅话题名
  explicit OdomSubscriber(ros::NodeHandle& nh, const std::string& topic = "/Odometry_px4");

  /// 获取最新里程计数据（线程安全）
  /// @param position 输出：位置 (ENU, m)
  /// @param velocity 输出：线速度 (ENU, m/s)
  /// @param yaw 输出：yaw角 (rad)
  /// @param stamp 输出：时间戳 (s)
  /// @return 是否收到过数据
  bool getLatest(domain::Vec3& position,
                 domain::Vec3& velocity,
                 double& yaw,
                 double& stamp) const;

  /// 设置外部回调（可选）
  /// @param callback 回调函数 (position, velocity, yaw, stamp)
  void setCallback(std::function<void(const domain::Vec3&,
                                      const domain::Vec3&,
                                      double,
                                      double)> callback);

private:
  /// ROS订阅者
  ros::Subscriber sub_;

  /// 最新数据（mutex保护）
  mutable std::mutex mutex_;
  domain::Vec3 position_;
  domain::Vec3 velocity_;
  double yaw_;
  double stamp_;
  bool has_data_;

  /// 外部回调（可选）
  std::function<void(const domain::Vec3&,
                     const domain::Vec3&,
                     double,
                     double)> external_callback_;

  /// ROS回调
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

// ============================================================================
// RCSubscriber类
// ============================================================================

/// RC输入订阅者（线程安全）
class RCSubscriber {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 订阅话题名
  explicit RCSubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/rc/in");

  /// 获取最新RC通道数据（线程安全）
  /// @param channels 输出：RC通道数组
  /// @return 是否收到过数据
  bool getLatest(std::vector<uint16_t>& channels) const;

  /// 设置外部回调（可选）
  /// @param callback 回调函数 (channels)
  void setCallback(std::function<void(const std::vector<uint16_t>&)> callback);

private:
  /// ROS订阅者
  ros::Subscriber sub_;

  /// 最新数据（mutex保护）
  mutable std::mutex mutex_;
  std::vector<uint16_t> channels_;
  bool has_data_;

  /// 外部回调（可选）
  std::function<void(const std::vector<uint16_t>&)> external_callback_;

  /// ROS回调
  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
};

// ============================================================================
// BatterySubscriber类
// ============================================================================

/// 电池状态订阅者（线程安全）
class BatterySubscriber {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 订阅话题名
  explicit BatterySubscriber(ros::NodeHandle& nh, const std::string& topic = "/mavros/battery");

  /// 获取最新电池电量（线程安全）
  /// @param percentage 输出：电量百分比 (0.0~1.0)
  /// @return 是否收到过数据
  bool getLatest(double& percentage) const;

  /// 设置外部回调（可选）
  /// @param callback 回调函数 (percentage)
  void setCallback(std::function<void(double)> callback);

private:
  /// ROS订阅者
  ros::Subscriber sub_;

  /// 最新数据（mutex保护）
  mutable std::mutex mutex_;
  double percentage_;
  bool has_data_;

  /// 外部回调（可选）
  std::function<void(double)> external_callback_;

  /// ROS回调
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
};

// ============================================================================
// ExtendedStateSubscriber类
// ============================================================================

/// MAVROS ExtendedState订阅者：飞控落地状态（重构方案第 5.4 节）
class ExtendedStateSubscriber {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param topic 订阅话题名
  explicit ExtendedStateSubscriber(ros::NodeHandle& nh,
                                   const std::string& topic = "/mavros/extended_state");

  /// 设置回调
  /// @param callback 回调函数 (landed_state)
  void setCallback(std::function<void(uint8_t)> callback);

private:
  /// ROS订阅者
  ros::Subscriber sub_;

  /// 回调
  std::function<void(uint8_t)> callback_;

  /// ROS回调
  void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
};

}  // namespace adapters
}  // namespace px4_backtrack
