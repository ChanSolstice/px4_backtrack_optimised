#include "px4_backtrack/infrastructure/config_loader.hpp"
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
  application::UseCaseConfig c;

  // 录制
  requireParam(nh, "record_max_hz", c.record_max_hz);
  requireParam(nh, "record_min_points", c.record_min_points);

  // 链路：CH18 与 RC 消息超时（第 5.1 节）
  auto& link = c.link_config;
  requireParam(nh, "enable_trigger_ch18", link.enable_ch18);
  requireParam(nh, "ch18_failsafe_threshold", link.ch18_failsafe_threshold);
  requireParam(nh, "ch18_confirm_s", link.ch18_confirm_s);
  requireParam(nh, "link_loss_leak_rate", link.link_loss_leak_rate);
  requireParam(nh, "enable_trigger_rc_loss", link.enable_rc_loss);
  requireParam(nh, "rc_loss_timeout_s", link.rc_loss_timeout_s);
  requireParam(nh, "rc_loss_debounce_s", link.rc_loss_debounce_s);
  requireParam(nh, "link_recover_s", link.link_recover_s);
  requireParam(nh, "link_degraded_window_s", link.link_degraded_window_s);

  // CH7（第 5.2 节）
  auto& pilot = c.pilot_config;
  requireParam(nh, "enable_trigger_ch7", pilot.enable_ch7);
  requireParam(nh, "ch7_pressed_threshold", pilot.ch7_pressed_threshold);
  requireParam(nh, "ch7_hold_s", pilot.ch7_hold_s);
  requireParam(nh, "ch7_exit_min_s", pilot.ch7_exit_min_s);
  pilot.rc_loss_timeout_s = link.rc_loss_timeout_s;

  // 电量（第 5.3 节）
  auto& battery = c.battery_config;
  requireParam(nh, "enable_trigger_low_battery", battery.enable_low_battery);
  requireParam(nh, "low_battery_threshold", battery.low_battery_threshold);
  requireParam(nh, "low_battery_confirm_s", battery.low_battery_confirm_s);

  // 输入新鲜度与在空中判定（第 5.4、5.5 节）
  requireParam(nh, "odom_stale_s", c.odom_stale_s);
  requireParam(nh, "odom_timeout_s", c.odom_timeout_s);
  requireParam(nh, "fcu_state_stale_s", c.fcu_state_stale_s);
  requireParam(nh, "airborne_min_height_m", c.airborne_config.airborne_min_height_m);
  c.airborne_config.fcu_state_stale_s = c.fcu_state_stale_s;

  // 状态机（第 6、7 节）
  auto& sm = c.state_machine_config;
  requireParam(nh, "record_delay_s", sm.record_delay_s);
  requireParam(nh, "ch18_hover_wait_s", sm.ch18_hover_wait_s);
  requireParam(nh, "landing_hover_s", sm.landing_hover_s);
  requireParam(nh, "pre_offboard_setpoint_count", sm.pre_offboard_setpoint_count);
  requireParam(nh, "return_pending_max_s", sm.return_pending_max_s);
  requireParam(nh, "fallback_timeout_s", sm.fallback_timeout_s);
  requireParam(nh, "handover_posctl_timeout_s", sm.handover_posctl_timeout_s);
  requireParam(nh, "safe_distance_m", sm.safe_distance_m);

  // 回退判定与模式请求（D22、D24）
  requireParam(nh, "fallback_gap_s", c.fallback_gap_s);
  requireParam(nh, "fallback_window_s", c.fallback_window_s);
  requireParam(nh, "mode_request_interval_s", c.mode_commander_config.request_interval_s);
  requireParam(nh, "mode_request_timeout_s", c.mode_commander_config.request_timeout_s);

  // 回归
  auto& rejoin = c.rejoin_config;
  requireParam(nh, "rejoin_trigger_dist_m", rejoin.trigger_dist_m);
  requireParam(nh, "rejoin_success_dist_m", rejoin.success_dist_m);
  requireParam(nh, "rejoin_v_max_m_s", rejoin.v_max_m_s);
  requireParam(nh, "rejoin_lookahead_time_s", rejoin.lookahead_time_s);
  requireParam(nh, "rejoin_trigger_hold_s", rejoin.trigger_hold_s);
  requireParam(nh, "rejoin_success_hold_s", rejoin.success_hold_s);
  sm.rejoin_trigger_dist_m = rejoin.trigger_dist_m;

  // Debug/可视化
  requireParam(nh, "debug_enable_csv_dump", c.debug_enable_csv_dump);
  requireParam(nh, "debug_csv_output_dir", c.debug_csv_output_dir);
  requireParam(nh, "debug_trajectory_sample_dt", c.debug_trajectory_sample_dt);
  requireParam(nh, "debug_enable_rviz_publish", c.debug_enable_rviz_publish);

  // 后台规划（第 7.5 节）
  requireParam(nh, "planning_warn_s", c.planning_warn_s);
  requireParam(nh, "planning_timeout_s", c.planning_timeout_s);

  // 退出后新轨迹（第 7.4 节）
  requireParam(nh, "cursor_window_m", c.cursor_window_m);

  // trajectory_core
  std::string trajectory_config_path;
  requireParam(nh, "trajectory_config_path", trajectory_config_path);
  c.trajectory_config = loadTrajectoryConfig(trajectory_config_path);

  ROS_INFO("[ConfigLoader] Configuration loaded");
  return c;
}

double ConfigLoader::loadPublishHz(ros::NodeHandle& nh) {
  double publish_hz = 0.0;
  requireParam(nh, "publish_hz", publish_hz);
  return publish_hz;
}

DebugLogConfig ConfigLoader::loadDebugLog(ros::NodeHandle& nh) {
  DebugLogConfig c;
  requireParam(nh, "debug_log_enable", c.enable);
  requireParam(nh, "debug_log_dir", c.dir);
  if (!c.dir.empty() && c.dir.front() != '/') {
    throw std::runtime_error("debug_log_dir must be an absolute path: " + c.dir);
  }
  return c;
}

trajectory_core::TrajectoryConfig ConfigLoader::loadTrajectoryConfig(const std::string& path) {
  ROS_INFO("[ConfigLoader] Loading trajectory_core config from: %s", path.c_str());
  return trajectory_core::TrajectoryConfig::loadFromYaml(path);
}

}  // namespace infrastructure
}  // namespace px4_backtrack
