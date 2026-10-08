#pragma once

#include "px4_backtrack/application/control_diagnostics.hpp"
#include <ros/ros.h>

namespace px4_backtrack::application {
struct PlanningTrace;
}

namespace px4_backtrack::adapters {

/// 发布结构化诊断；磁盘保存由独立录制进程承担。
class DiagnosticsPublisher {
public:
  explicit DiagnosticsPublisher(ros::NodeHandle &nh);
  void publishControl(const application::ControlDiagnostics &snapshot,
                      uint64_t tick_sequence, double tick_interval, double tick_cost) const;
  /// 初始化时只读取一次参数和 YAML，latched 发布合法 JSON 配置快照。
  void publishConfiguration(ros::NodeHandle &nh_private) const;
  void publishPlanning(const application::PlanningTrace &trace) const;

private:
  const std::string node_instance_id_;
  ros::Publisher control_pub_;
  ros::Publisher configuration_pub_;
  ros::Publisher planning_pub_;
};

} // namespace px4_backtrack::adapters
