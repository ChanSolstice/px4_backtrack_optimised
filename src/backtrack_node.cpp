#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/config_loader.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/ros_subscribers.hpp"
#include "px4_backtrack/adapters/ros_publishers.hpp"
#include "px4_backtrack/adapters/ros_services.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include "px4_backtrack/adapters/diagnostics_publisher.hpp"
#include <ros/ros.h>
#include <memory>
#include <cmath>
#include <stdexcept>

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
  ros::WallTimer timer_;

  /// Adapters层对象
  std::unique_ptr<adapters::StateSubscriber> state_sub_;
  std::unique_ptr<adapters::OdomSubscriber> odom_sub_;
  std::unique_ptr<adapters::RCSubscriber> rc_sub_;
  std::unique_ptr<adapters::BatterySubscriber> battery_sub_;
  std::unique_ptr<adapters::SetpointPublisher> setpoint_pub_;
  std::unique_ptr<adapters::StatusPublisher> status_pub_;
  std::unique_ptr<adapters::SetModeClient> set_mode_client_;

  std::shared_ptr<adapters::DiagnosticsPublisher> diagnostics_pub_;
  uint64_t tick_sequence_ = 0;
  double previous_tick_time_ = -1.0;

  /// Application层对象
  std::unique_ptr<application::BacktrackUseCase> usecase_;

  /// 定时器回调（20Hz控制循环）
  void timerCallback(const ros::WallTimerEvent& event);
};

// ============================================================================
// BacktrackNode实现
// ============================================================================

BacktrackNode::BacktrackNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("=== Initializing Backtrack Node ===");

  // 配置快照先于加载/校验发布，初始化错误也保留原始配置证据。
  if (nh_private_.param("log_enable", true)) {
    try {
      diagnostics_pub_ = std::make_shared<adapters::DiagnosticsPublisher>(nh_);
      diagnostics_pub_->publishConfiguration(nh_private_);
    } catch (const std::exception &e) {
      ROS_WARN("[BacktrackNode] Diagnostics disabled after initialization error: %s", e.what());
      diagnostics_pub_.reset();
    } catch (...) {
      ROS_WARN("[BacktrackNode] Diagnostics disabled after unknown initialization error");
      diagnostics_pub_.reset();
    }
  }

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
  if (!std::isfinite(publish_hz) || publish_hz<10.0 || publish_hz>100.0)
    throw std::invalid_argument("publish_hz must be within [10,100]");
  ROS_INFO("[BacktrackNode] Publish frequency: %.1f Hz", publish_hz);

  // 2. 创建Adapters层对象
  ROS_INFO("[BacktrackNode] Creating adapters...");
  state_sub_ = std::make_unique<adapters::StateSubscriber>(nh_);
  odom_sub_ = std::make_unique<adapters::OdomSubscriber>(nh_);
  rc_sub_ = std::make_unique<adapters::RCSubscriber>(nh_);
  battery_sub_ = std::make_unique<adapters::BatterySubscriber>(nh_);
  setpoint_pub_ = std::make_unique<adapters::SetpointPublisher>(nh_);
  status_pub_ = std::make_unique<adapters::StatusPublisher>(nh_);
  set_mode_client_ = std::make_unique<adapters::SetModeClient>(nh_);

  // 3. 创建UseCase
  ROS_INFO("[BacktrackNode] Creating BacktrackUseCase...");
  usecase_ = std::make_unique<application::BacktrackUseCase>(config);
  if (diagnostics_pub_) {
    const auto publisher = diagnostics_pub_;
    usecase_->setPlanningObserver([publisher](const application::PlanningTrace &trace) {
      publisher->publishPlanning(trace);
    });
  }

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
  odom_sub_->setCallback(
      [this](const domain::Vec3& position,
             const domain::Vec3& velocity,
             double yaw,
             double stamp) {
        usecase_->onOdomReceived(position, velocity, yaw, stamp, ros::SteadyTime::now().toSec());
      });

  rc_sub_->setCallback([this](const std::vector<uint16_t>& channels) {
    usecase_->onRCReceived(channels, ros::SteadyTime::now().toSec());
  });

  state_sub_->setCallback([this](bool armed, const std::string& mode) {
    usecase_->onStateReceived(armed, mode, ros::SteadyTime::now().toSec());
  });

  battery_sub_->setCallback([this](double percentage) {
    usecase_->onBatteryReceived(percentage);
  });

  // 6. 创建定时器
  ROS_INFO("[BacktrackNode] Starting %.1f Hz timer...", publish_hz);
  timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / publish_hz),
                           &BacktrackNode::timerCallback,
                           this);

  ROS_INFO("=== Backtrack Node Initialized ===");
}

void BacktrackNode::run() {
  ros::spin();
}

void BacktrackNode::timerCallback(const ros::WallTimerEvent&) {
  // 获取当前时间
  const double now = ros::SteadyTime::now().toSec();
  const double tick_interval = previous_tick_time_ < 0.0 ? 0.0 : now - previous_tick_time_;
  previous_tick_time_ = now;
  ++tick_sequence_;

  // 驱动UseCase tick
  application::TickOutput output = usecase_->tick(now);

  // 发布状态信息
  status_pub_->publishState(output.current_state);
  status_pub_->publishRCLoss(output.rc_loss);
  status_pub_->publishLowBattery(output.low_battery);

  // 处理setpoint发布
  if (output.should_publish_setpoint && output.setpoint.has_value()) {
    setpoint_pub_->publish(output.setpoint.value());
  }

  // 处理模式切换请求
  set_mode_client_->request(output.mode_request);

  // 控制任务全部调度后才观测；诊断异常不影响已经派发的控制量。
  if (diagnostics_pub_) {
    const double tick_cost = ros::SteadyTime::now().toSec() - now;
    try {
      diagnostics_pub_->publishControl(usecase_->diagnostics(now, output), tick_sequence_,
                                       tick_interval, tick_cost);
    } catch (const std::exception &e) {
      ROS_WARN_THROTTLE(1.0, "[BacktrackNode] Diagnostics query failed: %s", e.what());
    } catch (...) {
      ROS_WARN_THROTTLE(1.0, "[BacktrackNode] Diagnostics query failed: unknown exception");
    }
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
