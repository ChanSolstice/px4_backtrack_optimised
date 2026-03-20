#include "px4_backtrack/adapters/ros_services.hpp"
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetModeClient实现
// ============================================================================

SetModeClient::SetModeClient(ros::NodeHandle& nh,
                             const std::string& service_name,
                             double throttle_interval_s)
  : throttle_interval_s_(throttle_interval_s),
    last_call_time_(0.0)
{
  client_ = nh.serviceClient<mavros_msgs::SetMode>(service_name);
  ROS_INFO("[SetModeClient] Initialized with service: %s, throttle: %.1f s",
           service_name.c_str(), throttle_interval_s_);
}

bool SetModeClient::callSetMode(domain::ModeRequest mode_request) {
  // 节流检查
  double now = ros::Time::now().toSec();
  if (last_call_time_ > 0.0) {
    double elapsed = now - last_call_time_;
    if (elapsed < throttle_interval_s_) {
      ROS_DEBUG("[SetModeClient] Throttled: %.2f s < %.2f s", elapsed, throttle_interval_s_);
      return false;
    }
  }

  // 准备服务请求
  mavros_msgs::SetMode srv;
  srv.request.custom_mode = modeRequestToString(mode_request);

  // 调用服务
  if (client_.call(srv)) {
    if (srv.response.mode_sent) {
      ROS_INFO("[SetModeClient] Mode set to: %s", srv.request.custom_mode.c_str());
      last_call_time_ = now;
      return true;
    } else {
      ROS_WARN("[SetModeClient] Mode request sent but not accepted: %s",
               srv.request.custom_mode.c_str());
      last_call_time_ = now;
      return false;
    }
  } else {
    ROS_ERROR("[SetModeClient] Failed to call set_mode service");
    return false;
  }
}

std::string SetModeClient::modeRequestToString(domain::ModeRequest mode_request) const {
  switch (mode_request) {
    case domain::ModeRequest::OFFBOARD:
      return "OFFBOARD";
    case domain::ModeRequest::POSCTL:
      return "POSCTL";
    case domain::ModeRequest::AUTO_LAND:
      return "AUTO.LAND";
    default:
      return "UNKNOWN";
  }
}

}  // namespace adapters
}  // namespace px4_backtrack
