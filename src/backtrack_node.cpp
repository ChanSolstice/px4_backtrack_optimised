#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/config_loader.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/infrastructure/debug_log.hpp"
#include "px4_backtrack/adapters/ros_subscribers.hpp"
#include "px4_backtrack/adapters/ros_publishers.hpp"
#include "px4_backtrack/adapters/ros_services.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <ros/ros.h>
#include <ctime>
#include <memory>
#include <sstream>

#ifndef BACKTRACK_GIT_REV
#define BACKTRACK_GIT_REV "unknown"
#endif

namespace px4_backtrack {

namespace {

/// 运行目录名：run_<本地时间>
std::string runName() {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char buf[32];
  std::strftime(buf, sizeof(buf), "run_%Y%m%d_%H%M%S", &local);
  return buf;
}

/// 参数值转为 JSON 文本
std::string toJson(XmlRpc::XmlRpcValue& value) {
  switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return static_cast<bool>(value) ? "true" : "false";
    case XmlRpc::XmlRpcValue::TypeInt:
      return std::to_string(static_cast<int>(value));
    case XmlRpc::XmlRpcValue::TypeDouble: {
      std::ostringstream out;
      out << static_cast<double>(value);
      return out.str();
    }
    case XmlRpc::XmlRpcValue::TypeString:
      return infrastructure::DebugLog::jsonString(static_cast<std::string>(value));
    default:
      return infrastructure::DebugLog::jsonString(value.toXml());
  }
}

/// 节点私有命名空间下的全部参数（调试日志的 config 事件）
std::vector<std::pair<std::string, std::string>> privateParams(ros::NodeHandle& nh_private) {
  std::vector<std::string> names;
  ros::param::getParamNames(names);
  const std::string prefix = nh_private.getNamespace() + "/";
  std::vector<std::pair<std::string, std::string>> params;
  for (const auto& name : names) {
    XmlRpc::XmlRpcValue value;
    if (name.rfind(prefix, 0) == 0 && ros::param::get(name, value)) {
      params.emplace_back(name.substr(prefix.size()), toJson(value));
    }
  }
  return params;
}

/// 事件打到 rosout（完整记录写调试日志）
void logToRosout(const domain::StepEvent& e) {
  const char* type = domain::eventTypeName(e.type);
  switch (e.type) {
    case domain::EventType::State:
    case domain::EventType::Control:
    case domain::EventType::Follow:
    case domain::EventType::Fcu:
      ROS_INFO("[event] %s %s -> %s rule=%s reason=%s %s", type, e.from.c_str(),
               e.to.c_str(), e.rule.c_str(), e.reason.c_str(), e.detail.c_str());
      break;
    case domain::EventType::Mission:
    case domain::EventType::Planning:
      ROS_INFO("[event] %s %s %s %s", type, e.name.c_str(), e.reason.c_str(),
               e.detail.c_str());
      break;
    case domain::EventType::Alert:
      if (e.to == "on") {
        ROS_WARN("[event] alert %s", e.name.c_str());
      }
      break;
    case domain::EventType::ModeRequest:
      if (e.name == "failed") {
        ROS_WARN("[event] mode_request %s failed: %s", e.to.c_str(), e.reason.c_str());
      }
      break;
    case domain::EventType::Stream:
      ROS_WARN("[event] stream %s %s", e.name.c_str(), e.detail.c_str());
      break;
    case domain::EventType::Condition:
      break;
  }
}

}  // namespace

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

  /// 控制定时器
  ros::Timer timer_;

  /// Adapters层对象
  std::unique_ptr<adapters::StateSubscriber> state_sub_;
  std::unique_ptr<adapters::ExtendedStateSubscriber> extended_state_sub_;
  std::unique_ptr<adapters::OdomSubscriber> odom_sub_;
  std::unique_ptr<adapters::RCSubscriber> rc_sub_;
  std::unique_ptr<adapters::BatterySubscriber> battery_sub_;
  std::unique_ptr<adapters::SetpointPublisher> setpoint_pub_;
  std::unique_ptr<adapters::StatusPublisher> status_pub_;
  std::unique_ptr<adapters::SetModeClient> set_mode_client_;

  /// Application层对象
  std::unique_ptr<application::BacktrackUseCase> usecase_;

  /// 调试日志
  std::unique_ptr<infrastructure::DebugLog> debug_log_;

  /// 定时器回调（控制循环）
  void timerCallback(const ros::TimerEvent& event);
};

// ============================================================================
// BacktrackNode实现
// ============================================================================

BacktrackNode::BacktrackNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("=== Initializing Backtrack Node ===");

  // 1. 加载配置（全部必填，缺失时退出）
  application::UseCaseConfig config;
  infrastructure::DebugLogConfig debug_log_config;
  double publish_hz = 0.0;
  try {
    config = infrastructure::ConfigLoader::loadFromROS(nh_private_);
    debug_log_config = infrastructure::ConfigLoader::loadDebugLog(nh_private_);
    publish_hz = infrastructure::ConfigLoader::loadPublishHz(nh_private_);
  } catch (const std::exception& e) {
    ROS_FATAL("[BacktrackNode] Failed to load configuration: %s", e.what());
    ros::shutdown();
    return;
  }

  ROS_INFO("[BacktrackNode] Publish frequency: %.1f Hz", publish_hz);

  // 2. 调试日志：第一条为生效参数和版本
  debug_log_ = std::make_unique<infrastructure::DebugLog>(debug_log_config, runName());
  if (debug_log_->enabled()) {
    debug_log_->writeConfig(ros::Time::now().toSec(), privateParams(nh_private_),
                            BACKTRACK_GIT_REV, "unknown");
    ROS_INFO("[BacktrackNode] Debug log: %s%s", debug_log_->runDir().c_str(),
             debug_log_->failed() ? " (FAILED)" : "");
  }

  // 3. Adapters层对象
  state_sub_ = std::make_unique<adapters::StateSubscriber>(nh_);
  extended_state_sub_ = std::make_unique<adapters::ExtendedStateSubscriber>(nh_);
  odom_sub_ = std::make_unique<adapters::OdomSubscriber>(nh_);
  rc_sub_ = std::make_unique<adapters::RCSubscriber>(nh_);
  battery_sub_ = std::make_unique<adapters::BatterySubscriber>(nh_);
  setpoint_pub_ = std::make_unique<adapters::SetpointPublisher>(nh_);
  status_pub_ = std::make_unique<adapters::StatusPublisher>(nh_);
  set_mode_client_ = std::make_unique<adapters::SetModeClient>(nh_);

  // 4. UseCase
  usecase_ = std::make_unique<application::BacktrackUseCase>(config);
  if (config.debug_enable_csv_dump) {
    usecase_->setDebugDumper(
        std::make_shared<infrastructure::DebugDumper>(config.debug_csv_output_dir));
  }
  if (config.debug_enable_rviz_publish) {
    usecase_->setDebugPublisher(std::make_shared<adapters::DebugPublisher>(nh_));
  }

  // 5. 订阅回调：接收时间统一取回调时刻（第 5.5 节）
  odom_sub_->setCallback(
      [this](const domain::Vec3& position, const domain::Vec3& velocity,
             double yaw, double stamp) {
        usecase_->onOdom(ros::Time::now().toSec(), position, velocity, yaw, stamp);
      });
  rc_sub_->setCallback([this](const std::vector<uint16_t>& channels) {
    usecase_->onRc(ros::Time::now().toSec(), channels);
  });
  state_sub_->setCallback([this](bool connected, bool armed, const std::string& mode) {
    usecase_->onFcuState(ros::Time::now().toSec(), connected, armed, mode);
  });
  extended_state_sub_->setCallback([this](uint8_t landed_state) {
    const auto state = landed_state <= static_cast<uint8_t>(domain::LandedState::Landing)
        ? static_cast<domain::LandedState>(landed_state)
        : domain::LandedState::Undefined;
    usecase_->onExtendedState(ros::Time::now().toSec(), state);
  });
  battery_sub_->setCallback([this](double percentage) {
    usecase_->onBattery(ros::Time::now().toSec(), percentage);
  });

  // 6. 控制定时器
  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_hz),
                           &BacktrackNode::timerCallback, this);

  ROS_INFO("=== Backtrack Node Initialized ===");
}

void BacktrackNode::run() {
  ros::spin();
}

void BacktrackNode::timerCallback(const ros::TimerEvent&) {
  const double now = ros::Time::now().toSec();

  for (const auto& result : set_mode_client_->takeResults()) {
    usecase_->onModeRequestResult(now, result.mode, result.sent);
  }
  usecase_->setDebugLogFailed(debug_log_->failed());

  const application::TickOutput out = usecase_->tick(now);

  status_pub_->publishState(out.state);
  status_pub_->publishRCLoss(out.link_lost);
  status_pub_->publishLowBattery(out.low_battery);
  if (out.setpoint) {
    setpoint_pub_->publish(*out.setpoint);
  }
  if (out.mode_request) {
    set_mode_client_->request(*out.mode_request);
  }

  for (const auto& event : out.events) {
    logToRosout(event);
    debug_log_->write({now, out.flight_id, out.mission_id, event});
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
