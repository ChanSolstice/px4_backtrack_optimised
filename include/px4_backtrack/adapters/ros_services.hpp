#pragma once

#include "px4_backtrack/domain/state_machine.hpp"
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetModeClient类
// ============================================================================

/// SetMode服务客户端（带节流）
class SetModeClient {
public:
  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param service_name 服务名
  /// @param throttle_interval_s 节流间隔 (s)
  explicit SetModeClient(ros::NodeHandle& nh,
                         const std::string& service_name = "/mavros/set_mode",
                         double throttle_interval_s = 1.0);

  /// 调用SetMode服务（带节流）
  /// @param mode_request 模式请求
  /// @return 是否调用成功
  bool callSetMode(domain::ModeRequest mode_request);

private:
  /// ROS服务客户端
  ros::ServiceClient client_;

  /// 节流参数
  double throttle_interval_s_;
  double last_call_time_;

  /// 将ModeRequest转换为模式字符串
  std::string modeRequestToString(domain::ModeRequest mode_request) const;
};

}  // namespace adapters
}  // namespace px4_backtrack
