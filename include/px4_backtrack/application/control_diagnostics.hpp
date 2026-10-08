#pragma once

#include "px4_backtrack/domain/state_machine.hpp"
#include <cstdint>
#include <optional>
#include <string>

namespace px4_backtrack::application {

/// 每次 tick 后的只读控制快照。只描述业务，不依赖 ROS、写盘或推进会话。
struct ControlDiagnostics {
  uint32_t schema_version = 1;
  uint64_t flight_id = 0;
  uint64_t plan_id = 0;
  double steady_time = 0.0;
  domain::Vec3 actual_position = domain::Vec3::Zero();
  domain::Vec3 actual_velocity = domain::Vec3::Zero();
  double actual_yaw = 0.0;
  bool inputs_fresh = false;
  bool odom_valid = false;
  bool localization_reset = false;
  bool tick_clock_reset = false;
  double odom_stamp = -1.0;
  double odom_receipt_age = -1.0;  // -1 表示尚未收到；允许负值诊断时钟倒退。
  double state_receipt_age = -1.0;
  std::string input_failure_reason;
  std::string odometry_rejection_reason;
  domain::State current_state = domain::State::IdleDisarmed;
  std::string fcu_mode;
  bool armed = false;
  std::string last_trigger;
  std::string last_trigger_reason;
  uint64_t recorded_points = 0;
  bool planning_requested = false;
  bool planning_inflight = false;
  bool trajectory_valid = false;
  bool fallback_valid = false;
  bool projection_valid = false;
  double projected_s = 0.0;
  double target_s = 0.0;
  double cross_track_error = 0.0;
  double reference_time = 0.0;
  double clock_rate = 0.0;
  uint64_t fallback_index = 0;
  uint64_t fallback_count = 0;
  double route_safety_evaluation_time = -1.0;
  bool route_projection_valid = false;
  double route_distance = 0.0;
  bool route_prediction_valid = false;
  double route_predicted_deviation = 0.0;
  bool route_safety_hold = false;
  bool input_hold = false;
  bool yaw_alignment_timeout = false;
  bool auto_land_yield = false;
  bool rc_loss = false;
  bool low_battery = false;
  // 最近一次操作：操作理由变化时递增序号，连续快照/重复拒绝不会重放旧操作。
  uint64_t operation_sequence = 0;
  double operation_steady_time = -1.0;
  std::string operation;
  bool should_publish_setpoint = false;
  std::optional<domain::SetpointTarget> setpoint;
  std::optional<domain::ModeRequest> mode_request;
};

} // namespace px4_backtrack::application
