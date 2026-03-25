#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/config_loader.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/ros_subscribers.hpp"
#include "px4_backtrack/adapters/ros_publishers.hpp"
#include "px4_backtrack/adapters/ros_services.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <ros/ros.h>
#include <memory>

namespace px4_backtrack {

// ============================================================================
// BacktrackNode类
// ============================================================================

/// 返航节点主类
class BacktrackNode {
public:
  /// 构造函数
  explicit BacktrackNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  /// 运行节点
  void run();

private:
  /// ROS节点句柄
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;

  /// 20Hz定时器
  ros::Timer timer_;

  /// Adapters层对象
  std::unique_ptr<adapters::StateSubscriber> state_sub_;
  std::unique_ptr<adapters::OdomSubscriber> odom_sub_;
  std::unique_ptr<adapters::RCSubscriber> rc_sub_;
  std::unique_ptr<adapters::SetpointPublisher> setpoint_pub_;
  std::unique_ptr<adapters::StatusPublisher> status_pub_;
  std::unique_ptr<adapters::SetModeClient> set_mode_client_;

  /// Application层对象
  std::unique_ptr<application::BacktrackUseCase> usecase_;

  /// 定时器回调（20Hz控制循环）
  void timerCallback(const ros::TimerEvent& event);
};

// ============================================================================
// BacktrackNode实现
// ============================================================================

BacktrackNode::BacktrackNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("=== Initializing Backtrack Node ===");

  // 1. 加载配置
  ROS_INFO("[BacktrackNode] Loading configuration...");
  application::UseCaseConfig config;
  try {
    config = infrastructure::ConfigLoader::loadFromROS(nh_private_);
  } catch (const std::exception& e) {
    ROS_FATAL("[BacktrackNode] Failed to load configuration: %s", e.what());
    ros::shutdown();
    return;
  }

  // 读取publish_hz参数
  double publish_hz = nh_private_.param("publish_hz", 20.0);
  ROS_INFO("[BacktrackNode] Publish frequency: %.1f Hz", publish_hz);

  // 2. 创建Adapters层对象
  ROS_INFO("[BacktrackNode] Creating adapters...");
  state_sub_ = std::make_unique<adapters::StateSubscriber>(nh_);
  odom_sub_ = std::make_unique<adapters::OdomSubscriber>(nh_);
  rc_sub_ = std::make_unique<adapters::RCSubscriber>(nh_);
  setpoint_pub_ = std::make_unique<adapters::SetpointPublisher>(nh_);
  status_pub_ = std::make_unique<adapters::StatusPublisher>(nh_);
  set_mode_client_ = std::make_unique<adapters::SetModeClient>(nh_);

  // 3. 创建UseCase
  ROS_INFO("[BacktrackNode] Creating BacktrackUseCase...");
  usecase_ = std::make_unique<application::BacktrackUseCase>(config);

  // 4. 配置 debug 功能（可选）
  if (config.debug_enable_csv_dump) {
    auto dumper = std::make_shared<infrastructure::DebugDumper>(config.debug_csv_output_dir);
    usecase_->setDebugDumper(dumper);
    ROS_INFO("[BacktrackNode] CSV debug dump enabled");
  }
  if (config.debug_enable_rviz_publish) {
    auto debug_pub = std::make_shared<adapters::DebugPublisher>(nh_);
    usecase_->setDebugPublisher(debug_pub);
    ROS_INFO("[BacktrackNode] RViz debug publish enabled");
  }

  // 5. 设置订阅者回调（可选，用于实时数据传递）
  odom_sub_->setCallback([this](const domain::Vec3& position, double yaw, double stamp) {
    usecase_->onOdomReceived(position, yaw, stamp);
  });

  rc_sub_->setCallback([this](const std::vector<uint16_t>& channels) {
    usecase_->onRCReceived(channels);
  });

  state_sub_->setCallback([this](bool armed, const std::string& mode) {
    usecase_->onStateReceived(armed, mode);
  });

  // 6. 创建定时器
  ROS_INFO("[BacktrackNode] Starting %.1f Hz timer...", publish_hz);
  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_hz),
                           &BacktrackNode::timerCallback,
                           this);

  ROS_INFO("=== Backtrack Node Initialized ===");
}

void BacktrackNode::run() {
  ros::spin();
}

void BacktrackNode::timerCallback(const ros::TimerEvent& event) {
  // 获取当前时间
  double now = ros::Time::now().toSec();

  // 驱动UseCase tick
  application::TickOutput output = usecase_->tick(now);

  // 发布状态信息
  status_pub_->publishState(output.current_state);
  status_pub_->publishRCLoss(output.rc_loss);

  // 处理setpoint发布
  if (output.should_publish_setpoint && output.setpoint.has_value()) {
    setpoint_pub_->publish(output.setpoint.value());
  }

  // 处理模式切换请求
  if (output.mode_request.has_value()) {
    set_mode_client_->callSetMode(output.mode_request.value());
  }
}

}  // namespace px4_backtrack

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "backtrack_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  try {
    px4_backtrack::BacktrackNode node(nh, nh_private);
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("[main] Exception: %s", e.what());
    return 1;
  }

  return 0;
}
