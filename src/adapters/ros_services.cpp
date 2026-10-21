#include "px4_backtrack/adapters/ros_services.hpp"
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetModeClient实现
// ============================================================================

SetModeClient::SetModeClient(ros::NodeHandle& nh, const std::string& service_name)
{
  client_ = nh.serviceClient<mavros_msgs::SetMode>(service_name);
  worker_ = std::thread(&SetModeClient::workerLoop, this);
  ROS_INFO("[SetModeClient] Initialized with service: %s (async)", service_name.c_str());
}

SetModeClient::~SetModeClient() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  worker_.join();
}

void SetModeClient::request(const std::string& mode) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = mode;
  }
  cv_.notify_one();
}

std::vector<SetModeClient::Result> SetModeClient::takeResults() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Result> results;
  results.swap(results_);
  return results;
}

void SetModeClient::workerLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    cv_.wait(lock, [this] { return stop_ || pending_.has_value(); });
    if (stop_) {
      return;
    }
    const std::string mode = *pending_;
    pending_.reset();
    lock.unlock();

    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    const bool called = client_.call(srv);
    const bool sent = called && srv.response.mode_sent;
    if (!called) {
      ROS_WARN("[SetModeClient] set_mode service unavailable: %s", mode.c_str());
    } else if (!sent) {
      ROS_WARN("[SetModeClient] set_mode not sent: %s", mode.c_str());
    }

    lock.lock();
    results_.push_back({mode, sent});
  }
}

}  // namespace adapters
}  // namespace px4_backtrack
