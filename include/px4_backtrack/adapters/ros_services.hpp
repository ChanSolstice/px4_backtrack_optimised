#pragma once
#include "px4_backtrack/domain/state_machine.hpp"
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace px4_backtrack::adapters {
class SetModeClient {
public:
  explicit SetModeClient(ros::NodeHandle& nh, const std::string& service_name="/mavros/set_mode",
                         double throttle_interval_s=1.0);
  explicit SetModeClient(std::function<bool(domain::ModeRequest)> transport,
                         double throttle_interval_s=1.0);
  ~SetModeClient();
  // 非阻塞；nullopt 清掉尚未派发的旧请求。已经发给飞控的 RPC 无法撤回。
  void request(std::optional<domain::ModeRequest> mode);
private:
  ros::ServiceClient client_;
  std::function<bool(domain::ModeRequest)> transport_;
  double throttle_interval_s_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<domain::ModeRequest> pending_;
  bool stopping_=false;
  std::thread thread_;
  void run();
};
}
