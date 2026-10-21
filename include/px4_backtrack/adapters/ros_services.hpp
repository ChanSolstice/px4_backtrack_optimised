#pragma once

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>

#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace px4_backtrack {
namespace adapters {

// ============================================================================
// SetModeClient类
// ============================================================================

/// SetMode服务客户端（异步，重构方案第 7.7 节）
///
/// 服务调用在单独线程中执行，不阻塞控制回调；节流和结果观察由 ModeCommander 负责。
class SetModeClient {
public:
  /// 一次调用的结果
  struct Result {
    std::string mode;  ///< 请求的模式
    bool sent{false};  ///< 服务调用成功且 mode_sent 为真
  };

  /// 构造函数
  /// @param nh ROS节点句柄
  /// @param service_name 服务名
  explicit SetModeClient(ros::NodeHandle& nh,
                         const std::string& service_name = "/mavros/set_mode");

  /// 析构：等待正在进行的调用结束
  ~SetModeClient();

  SetModeClient(const SetModeClient&) = delete;
  SetModeClient& operator=(const SetModeClient&) = delete;

  /// 请求切换模式（不阻塞）；尚未开始的旧请求被新请求替换
  void request(const std::string& mode);

  /// 取出上次调用以来完成的结果
  std::vector<Result> takeResults();

private:
  /// 后台调用线程
  void workerLoop();

  /// ROS服务客户端
  ros::ServiceClient client_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<std::string> pending_;
  std::vector<Result> results_;
  bool stop_{false};
  std::thread worker_;
};

}  // namespace adapters
}  // namespace px4_backtrack
