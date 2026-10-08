#include "px4_backtrack/adapters/diagnostics_publisher.hpp"
#include <px4_backtrack_optimised/ControlDiagnostics.h>
#include <px4_backtrack_optimised/PlanningTrace.h>
#include <std_msgs/String.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <sstream>
#include <unistd.h>

#ifndef BACKTRACK_BUILD_REVISION
#define BACKTRACK_BUILD_REVISION "unknown"
#endif

namespace px4_backtrack::adapters {

DiagnosticsPublisher::DiagnosticsPublisher(ros::NodeHandle &nh)
    : node_instance_id_(std::to_string(ros::WallTime::now().toNSec()) + "-" +
                        std::to_string(::getpid())) {
  control_pub_ = nh.advertise<px4_backtrack_optimised::ControlDiagnostics>(
      "/px4_backtrack/log/control", 20);
  configuration_pub_ = nh.advertise<std_msgs::String>(
      "/px4_backtrack/log/configuration", 1, true);
  planning_pub_ = nh.advertise<px4_backtrack_optimised::PlanningTrace>(
      "/px4_backtrack/log/planning", 8, true);
}

void DiagnosticsPublisher::publishConfiguration(ros::NodeHandle &nh_private) const {
  try {
    boost::property_tree::ptree configuration;
    configuration.put("build_revision", BACKTRACK_BUILD_REVISION);
    configuration.put("node_instance_id", node_instance_id_);
    configuration.put("private_namespace", nh_private.getNamespace());
    XmlRpc::XmlRpcValue parameters;
    if (nh_private.getParam(nh_private.getNamespace(), parameters))
      configuration.put("parameters_xml", parameters.toXml());
    else
      configuration.put("parameters_xml", "");
    bool use_sim_time = false;
    nh_private.param("/use_sim_time", use_sim_time, false);
    configuration.put("use_sim_time", use_sim_time);
    const std::string path = nh_private.param<std::string>("trajectory_config_path", "");
    configuration.put("trajectory_config_path", path);
    std::ifstream stream(path);
    if (stream) {
      std::ostringstream text;
      text << stream.rdbuf();
      configuration.put("trajectory_config_yaml", text.str());
    } else {
      configuration.put("trajectory_config_yaml", "");
      configuration.put("trajectory_config_error", "configuration file unavailable");
    }
    std::ostringstream json;
    boost::property_tree::write_json(json, configuration);
    std_msgs::String msg;
    msg.data = json.str();
    configuration_pub_.publish(msg);
  } catch (const std::exception &e) {
    ROS_WARN("[DiagnosticsPublisher] Configuration snapshot failed: %s", e.what());
  } catch (...) {
    ROS_WARN("[DiagnosticsPublisher] Configuration snapshot failed: unknown exception");
  }
}

void DiagnosticsPublisher::publishControl(const application::ControlDiagnostics &d,
                                         uint64_t tick_sequence, double tick_interval,
                                         double tick_cost) const {
  try {
    px4_backtrack_optimised::ControlDiagnostics msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.header.seq = static_cast<uint32_t>(tick_sequence);
    msg.schema_version = d.schema_version;
    msg.node_instance_id = node_instance_id_;
    msg.tick_sequence = tick_sequence;
    msg.flight_id = d.flight_id;
    msg.plan_id = d.plan_id;
    msg.steady_time = d.steady_time;
    msg.tick_interval = tick_interval;
    msg.tick_cost = tick_cost;
    msg.actual_position.x = d.actual_position.x();
    msg.actual_position.y = d.actual_position.y();
    msg.actual_position.z = d.actual_position.z();
    msg.actual_velocity.x = d.actual_velocity.x();
    msg.actual_velocity.y = d.actual_velocity.y();
    msg.actual_velocity.z = d.actual_velocity.z();
    msg.actual_yaw = d.actual_yaw;
    msg.inputs_fresh = d.inputs_fresh;
    msg.odom_valid = d.odom_valid;
    msg.localization_reset = d.localization_reset;
    msg.tick_clock_reset = d.tick_clock_reset;
    msg.odom_stamp = d.odom_stamp;
    msg.odom_receipt_age = d.odom_receipt_age;
    msg.state_receipt_age = d.state_receipt_age;
    msg.input_failure_reason = d.input_failure_reason;
    msg.odometry_rejection_reason = d.odometry_rejection_reason;
    msg.current_state = domain::stateToString(d.current_state);
    msg.fcu_mode = d.fcu_mode;
    msg.armed = d.armed;
    msg.last_trigger = d.last_trigger;
    msg.last_trigger_reason = d.last_trigger_reason;
    msg.recorded_points = d.recorded_points;
    msg.planning_requested = d.planning_requested;
    msg.planning_inflight = d.planning_inflight;
    msg.trajectory_valid = d.trajectory_valid;
    msg.fallback_valid = d.fallback_valid;
    msg.projection_valid = d.projection_valid;
    msg.projected_s = d.projected_s;
    msg.target_s = d.target_s;
    msg.cross_track_error = d.cross_track_error;
    msg.reference_time = d.reference_time;
    msg.clock_rate = d.clock_rate;
    msg.fallback_index = d.fallback_index;
    msg.fallback_count = d.fallback_count;
    msg.route_safety_evaluation_time = d.route_safety_evaluation_time;
    msg.route_projection_valid = d.route_projection_valid;
    msg.route_distance = d.route_distance;
    msg.route_prediction_valid = d.route_prediction_valid;
    msg.route_predicted_deviation = d.route_predicted_deviation;
    msg.route_safety_hold = d.route_safety_hold;
    msg.input_hold = d.input_hold;
    msg.yaw_alignment_timeout = d.yaw_alignment_timeout;
    msg.auto_land_yield = d.auto_land_yield;
    msg.rc_loss = d.rc_loss;
    msg.low_battery = d.low_battery;
    msg.operation_sequence = d.operation_sequence;
    msg.operation_steady_time = d.operation_steady_time;
    msg.operation = d.operation;
    msg.should_publish_setpoint = d.should_publish_setpoint;
    msg.setpoint_present = d.setpoint.has_value();
    if (d.setpoint) {
      msg.setpoint_coordinate_frame = d.setpoint->coordinate_frame;
      msg.setpoint_type_mask = d.setpoint->type_mask;
      msg.setpoint_position.x = d.setpoint->position.x();
      msg.setpoint_position.y = d.setpoint->position.y();
      msg.setpoint_position.z = d.setpoint->position.z();
      msg.setpoint_velocity.x = d.setpoint->velocity.x();
      msg.setpoint_velocity.y = d.setpoint->velocity.y();
      msg.setpoint_velocity.z = d.setpoint->velocity.z();
      msg.setpoint_yaw = d.setpoint->yaw;
      msg.setpoint_yaw_rate = d.setpoint->yaw_rate;
      msg.setpoint_acceleration_present = d.setpoint->acceleration.has_value();
      if (d.setpoint->acceleration) {
        msg.setpoint_acceleration.x = d.setpoint->acceleration->x();
        msg.setpoint_acceleration.y = d.setpoint->acceleration->y();
        msg.setpoint_acceleration.z = d.setpoint->acceleration->z();
      }
    }
    msg.mode_request_present = d.mode_request.has_value();
    if (d.mode_request)
      msg.mode_request = domain::modeRequestToString(*d.mode_request);
    control_pub_.publish(msg);
  } catch (const std::exception &e) {
    ROS_WARN_THROTTLE(1.0, "[DiagnosticsPublisher] Control snapshot failed: %s", e.what());
  } catch (...) {
    ROS_WARN_THROTTLE(1.0, "[DiagnosticsPublisher] Control snapshot failed: unknown exception");
  }
}

} // namespace px4_backtrack::adapters
