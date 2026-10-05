#include "px4_backtrack/adapters/ros_services.hpp"
#include <ros/console.h>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace px4_backtrack::adapters {
SetModeClient::SetModeClient(ros::NodeHandle &nh, const std::string &service_name, double interval)
    : throttle_interval_s_(interval) {
  if (!std::isfinite(interval) || interval < 0.0)
    throw std::invalid_argument("Invalid mode throttle");
  client_ = nh.serviceClient<mavros_msgs::SetMode>(service_name);
  transport_ = [this](domain::ModeRequest mode) {
    mavros_msgs::SetMode service;
    service.request.custom_mode = domain::modeRequestToString(mode);
    return client_.call(service) && service.response.mode_sent;
  };
  thread_ = std::thread(&SetModeClient::run, this);
}
SetModeClient::SetModeClient(std::function<bool(domain::ModeRequest)> transport, double interval)
    : transport_(std::move(transport)), throttle_interval_s_(interval) {
  if (!transport_ || !std::isfinite(interval) || interval < 0.0)
    throw std::invalid_argument("Invalid mode transport");
  thread_ = std::thread(&SetModeClient::run, this);
}
SetModeClient::~SetModeClient() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    pending_.reset();
  }
  wake_.notify_one();
  client_.shutdown();
  thread_.join();
}
void SetModeClient::request(std::optional<domain::ModeRequest> mode) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = mode;
  }
  wake_.notify_one();
}
void SetModeClient::run() {
  using Clock = std::chrono::steady_clock;
  auto next_attempt = Clock::now();
  for (;;) {
    domain::ModeRequest mode;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (stopping_)
        return;
      if (Clock::now() < next_attempt) {
        wake_.wait_until(lock, next_attempt);
        continue;
      }
      mode = *pending_;
      pending_.reset();
      next_attempt = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                        std::chrono::duration<double>(throttle_interval_s_));
    }
    try {
      const bool sent = transport_(mode);
      ROS_INFO(
          "[SetModeClient] %s request result: %s (activation requires fresh state confirmation)",
          domain::modeRequestToString(mode).c_str(), sent ? "sent" : "failed");
    } catch (const std::exception &error) {
      ROS_ERROR("[SetModeClient] Exception: %s", error.what());
    }
  }
}
} // namespace px4_backtrack::adapters
