#include "px4_backtrack/infrastructure/config_loader.hpp"
#include <ros/package.h>
#include <stdexcept>

namespace px4_backtrack {
namespace infrastructure {

namespace {

template <typename T>
void requireParam(ros::NodeHandle& nh, const char* name, T& value) {
  if (nh.getParam(name, value)) {
    return;
  }

  ROS_ERROR("[ConfigLoader] Required parameter '%s' is not set", name);
  throw std::runtime_error(std::string("required parameter missing: ") + name);
}

}  // namespace

application::UseCaseConfig ConfigLoader::loadFromROS(ros::NodeHandle& nh) {
  application::UseCaseConfig config;

  ROS_INFO("[ConfigLoader] Loading configuration from ROS parameter server");

  // 加载各模块参数
  loadRecordingParams(nh, config);
  loadCh7Params(nh, config);
  loadCh18Params(nh, config);
  loadRCLossParams(nh, config);
  loadOffboardParams(nh, config);
  loadRejoinParams(nh, config);
  loadLandingParams(nh, config);
  loadDebugParams(nh, config);

  // 加载trajectory_core配置路径
  std::string trajectory_config_path;
  nh.param<std::string>("trajectory_config_path", trajectory_config_path, "");

  if (trajectory_config_path.empty()) {
    ROS_ERROR("[ConfigLoader] trajectory_config_path is not set!");
    throw std::runtime_error("trajectory_config_path parameter is required");
  }

  // 加载trajectory_core配置
  config.trajectory_config = loadTrajectoryConfig(trajectory_config_path);

  ROS_INFO("[ConfigLoader] Configuration loaded successfully");

  return config;
}

trajectory_core::TrajectoryConfig ConfigLoader::loadTrajectoryConfig(const std::string& path) {
  ROS_INFO("[ConfigLoader] Loading trajectory_core config from: %s", path.c_str());

  try {
    auto config = trajectory_core::TrajectoryConfig::loadFromYaml(path);
    ROS_INFO("[ConfigLoader] Trajectory config loaded successfully");
    return config;
  } catch (const std::exception& e) {
    ROS_ERROR("[ConfigLoader] Failed to load trajectory config: %s", e.what());
    throw;
  }
}

void ConfigLoader::loadRecordingParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  nh.param("record_delay_s", config.record_delay_s, 15.0);
  nh.param("record_min_points", config.record_min_points, 100);
  nh.param("record_max_hz", config.record_max_hz, 10.0);

  ROS_INFO("[ConfigLoader] Recording params:");
  ROS_INFO("  record_delay_s: %.1f", config.record_delay_s);
  ROS_INFO("  record_min_points: %d", config.record_min_points);
  ROS_INFO("  record_max_hz: %.1f", config.record_max_hz);
}

void ConfigLoader::loadCh7Params(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  requireParam(nh, "enable_trigger_ch7", config.trigger_config.enable_ch7);
  requireParam(nh, "ch7_pressed_threshold", config.trigger_config.ch7_pressed_threshold);
  requireParam(nh, "ch7_hold_s", config.trigger_config.ch7_hold_s);

  ROS_INFO("[ConfigLoader] Ch7 params:");
  ROS_INFO("  enable_trigger_ch7: %s", config.trigger_config.enable_ch7 ? "true" : "false");
  ROS_INFO("  ch7_pressed_threshold: %d", config.trigger_config.ch7_pressed_threshold);
  ROS_INFO("  ch7_hold_s: %.1f", config.trigger_config.ch7_hold_s);
}

void ConfigLoader::loadCh18Params(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  requireParam(nh, "enable_trigger_ch18", config.trigger_config.enable_ch18);
  requireParam(nh, "ch18_failsafe_threshold", config.trigger_config.ch18_failsafe_threshold);
  requireParam(nh, "ch18_confirm_s", config.trigger_config.ch18_confirm_s);
  nh.param("ch18_hover_wait_s", config.state_machine_config.ch18_hover_wait_s, 15.0);

  ROS_INFO("[ConfigLoader] Ch18 params:");
  ROS_INFO("  enable_trigger_ch18: %s", config.trigger_config.enable_ch18 ? "true" : "false");
  ROS_INFO("  ch18_failsafe_threshold: %d", config.trigger_config.ch18_failsafe_threshold);
  ROS_INFO("  ch18_confirm_s: %.1f", config.trigger_config.ch18_confirm_s);
  ROS_INFO("  ch18_hover_wait_s: %.1f", config.state_machine_config.ch18_hover_wait_s);
}

void ConfigLoader::loadRCLossParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  requireParam(nh, "enable_trigger_rc_loss", config.trigger_config.enable_rc_loss);
  requireParam(nh, "rc_loss_timeout_s", config.trigger_config.rc_loss_timeout_s);
  requireParam(nh, "rc_loss_debounce_s", config.trigger_config.rc_loss_debounce_s);

  ROS_INFO("[ConfigLoader] RC loss params:");
  ROS_INFO("  enable_trigger_rc_loss: %s", config.trigger_config.enable_rc_loss ? "true" : "false");
  ROS_INFO("  rc_loss_timeout_s: %.2f", config.trigger_config.rc_loss_timeout_s);
  ROS_INFO("  rc_loss_debounce_s: %.2f", config.trigger_config.rc_loss_debounce_s);
}

void ConfigLoader::loadOffboardParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  nh.param("pre_offboard_setpoint_count",
           config.state_machine_config.pre_offboard_setpoint_count, 20);

  ROS_INFO("[ConfigLoader] OFFBOARD params:");
  ROS_INFO("  pre_offboard_setpoint_count: %d",
           config.state_machine_config.pre_offboard_setpoint_count);
}

void ConfigLoader::loadRejoinParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  nh.param("rejoin_trigger_dist_m", config.rejoin_config.trigger_dist_m, 0.5);
  nh.param("rejoin_success_dist_m", config.rejoin_config.success_dist_m, 0.1);
  nh.param("rejoin_v_max_m_s", config.rejoin_config.v_max_m_s, 0.3);
  nh.param("rejoin_lookahead_time_s", config.rejoin_config.lookahead_time_s, 0.5);

  ROS_INFO("[ConfigLoader] Rejoin params:");
  ROS_INFO("  rejoin_trigger_dist_m: %.2f", config.rejoin_config.trigger_dist_m);
  ROS_INFO("  rejoin_success_dist_m: %.2f", config.rejoin_config.success_dist_m);
  ROS_INFO("  rejoin_v_max_m_s: %.2f", config.rejoin_config.v_max_m_s);
  ROS_INFO("  rejoin_lookahead_time_s: %.2f", config.rejoin_config.lookahead_time_s);
}

void ConfigLoader::loadLandingParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  nh.param("landing_hover_s", config.state_machine_config.landing_hover_s, 5.0);

  ROS_INFO("[ConfigLoader] Landing params:");
  ROS_INFO("  landing_hover_s: %.1f", config.state_machine_config.landing_hover_s);
}

void ConfigLoader::loadDebugParams(ros::NodeHandle& nh, application::UseCaseConfig& config) {
  nh.param("debug_enable_csv_dump", config.debug_enable_csv_dump, false);
  nh.param<std::string>("debug_csv_output_dir", config.debug_csv_output_dir, std::string(""));
  nh.param("debug_trajectory_sample_dt", config.debug_trajectory_sample_dt, 0.05);
  nh.param("debug_enable_rviz_publish", config.debug_enable_rviz_publish, false);

  ROS_INFO("[ConfigLoader] Debug params:");
  ROS_INFO("  debug_enable_csv_dump: %s", config.debug_enable_csv_dump ? "true" : "false");
  ROS_INFO("  debug_csv_output_dir: %s",
           config.debug_csv_output_dir.empty() ? "(default: ~/.ros/backtrack_debug/)" : config.debug_csv_output_dir.c_str());
  ROS_INFO("  debug_trajectory_sample_dt: %.3f", config.debug_trajectory_sample_dt);
  ROS_INFO("  debug_enable_rviz_publish: %s", config.debug_enable_rviz_publish ? "true" : "false");
}

}  // namespace infrastructure
}  // namespace px4_backtrack
