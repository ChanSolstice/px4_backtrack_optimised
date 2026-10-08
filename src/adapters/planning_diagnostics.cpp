#include "px4_backtrack/adapters/diagnostics_publisher.hpp"
#include "px4_backtrack/application/planning_worker.hpp"
#include <px4_backtrack_optimised/PlanningTrace.h>
#include <iomanip>
#include <sstream>

namespace px4_backtrack::adapters {
namespace {
// Complete configuration of this attempt, including defaults not present in the input YAML.
std::string effectiveConfiguration(trajectory_core::TrajectoryConfig config) {
  config = domain::TrajectorySession::planningConfiguration(config);
  std::ostringstream out;
  out << std::setprecision(17) << std::boolalpha;
#define FIELD(name) out << #name ": " << config.name << '\n'
  FIELD(ds_m);
  FIELD(max_deviation_m);
  FIELD(dt_sample_s);
  FIELD(min_point_dist_m);
  FIELD(max_dt_gap_s);
  FIELD(corner_angle_threshold_deg);
  FIELD(corner_blend_len_m);
  FIELD(corner_blend_ratio);
  FIELD(corner_tangent_factor);
  FIELD(ds_check_m);
  FIELD(enable_yaw_smooth);
  FIELD(max_yaw_deviation_rad);
  FIELD(yaw_rate_max);
  FIELD(max_expected_error_rad);
  FIELD(k_yaw_rate_limiter);
  FIELD(enable_yaw_rate_constraint_on_velocity);
  FIELD(v_yaw_min_warn);
  FIELD(enable_end_converge);
  FIELD(s_stop_m);
  FIELD(r_stop_m);
  FIELD(v_end_max);
  FIELD(r_yaw_hold_m);
  FIELD(end_mode_latch);
  FIELD(v_max);
  FIELD(vz_max);
  FIELD(a_tan_max);
  FIELD(a_lat_max);
  FIELD(az_up_max);
  FIELD(az_down_max);
  FIELD(v_start_m_s);
  FIELD(v_end_m_s);
  FIELD(enable_output_acc);
  FIELD(v_time_eps);
#undef FIELD
  return out.str();
}
} // namespace

void DiagnosticsPublisher::publishPlanning(const application::PlanningTrace &trace) const {
  px4_backtrack_optimised::PlanningTrace message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  message.schema_version = 1;
  message.node_instance_id = node_instance_id_;
  message.flight_id = trace.flight_id;
  message.plan_id = trace.plan_id;
  message.steady_time_s = ros::SteadyTime::now().toSec();
  message.phase = trace.phase;
  message.status = trace.generation.status;
  message.message = trace.generation.message;
  message.warning_flags = trace.generation.warning_flags;
  message.success = trace.success;
  message.fallback_ready = trace.fallback_ready;
  message.generation_cost_ms = trace.generation_cost_ms;
  const auto &report = trace.generation.report;
  message.duration_s = report.profile.total_duration_s;
  message.path_length_m = report.profile.path_length_m;
  message.max_speed_m_s = report.profile.max_speed_m_s;
  message.max_yaw_rate_rad_s = report.yaw.max_yaw_rate_rad_s;
  message.motion_speed_scale = report.profile.motion_speed_scale;
  message.motion_limits_verified = report.profile.motion_limits_verified;
  message.corridor_verified = report.fit.fit_converged;
  message.report = report.toString();
  if (trace.config)
    message.effective_config = effectiveConfiguration(*trace.config);
  message.acceleration_budget_scale = 0.5;
  if (trace.points) {
    message.raw_position.reserve(trace.points->size());
    message.raw_yaw_rad.reserve(trace.points->size());
    message.raw_timestamp_s.reserve(trace.points->size());
    for (const auto &point : *trace.points) {
      geometry_msgs::Point p;
      p.x = point.position.x();
      p.y = point.position.y();
      p.z = point.position.z();
      message.raw_position.push_back(p);
      message.raw_yaw_rad.push_back(point.yaw);
      message.raw_timestamp_s.push_back(point.timestamp);
    }
  }
  planning_pub_.publish(message);
}
} // namespace px4_backtrack::adapters
