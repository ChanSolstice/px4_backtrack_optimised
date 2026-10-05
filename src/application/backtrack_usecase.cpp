#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <ros/console.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_backtrack {
namespace application {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kYawAlignmentToleranceRad = 15.0 * kPi / 180.0;
constexpr double kYawAlignmentTimeoutS = 10.0;
constexpr double kReturnEndPositionToleranceM = 0.20;

double wrappedAngleError(double actual, double target) {
  return std::abs(std::atan2(std::sin(actual - target), std::cos(actual - target)));
}

void validateConfig(const UseCaseConfig &config) {
  const auto require = [](bool valid, const char *name) {
    if (!valid)
      throw std::invalid_argument(std::string("Invalid return configuration: ") + name);
  };
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0.0; };
  require(positive(config.odom_timeout_s), "odom_timeout_s");
  require(positive(config.state_timeout_s), "state_timeout_s");
  require(positive(config.record_max_hz), "record_max_hz");
  require(config.record_min_points >= 2, "record_min_points");
  require(nonnegative(config.record_delay_s), "record_delay_s");
  require(positive(config.route_normal_limit_m) && config.route_normal_limit_m <= 0.30,
          "route_normal_limit_m");
  require(positive(config.route_hard_limit_m) &&
              config.route_hard_limit_m > config.route_normal_limit_m &&
              config.route_hard_limit_m <= 0.50,
          "route_hard_limit_m");
  require(nonnegative(config.route_uncertainty_m) &&
              config.route_uncertainty_m < config.route_normal_limit_m,
          "route_uncertainty_m");
  require(nonnegative(config.route_reaction_s), "route_reaction_s");
  require(positive(config.route_braking_m_s2), "route_braking_m_s2");
  const auto &rejoin = config.rejoin_config;
  require(positive(rejoin.trigger_dist_m) &&
              rejoin.trigger_dist_m < config.route_normal_limit_m - config.route_uncertainty_m,
          "rejoin_trigger_dist_m");
  require(positive(rejoin.success_dist_m) && rejoin.success_dist_m < rejoin.trigger_dist_m,
          "rejoin_success_dist_m");
  require(positive(rejoin.v_max_m_s) && rejoin.v_max_m_s < 1.0, "rejoin_v_max_m_s");
  require(nonnegative(rejoin.lookahead_time_s), "rejoin_lookahead_time_s");
  require(nonnegative(rejoin.trigger_hold_s), "rejoin_trigger_hold_s");
  require(nonnegative(rejoin.success_hold_s), "rejoin_success_hold_s");
  const auto &state = config.state_machine_config;
  require(state.pre_offboard_setpoint_count >= 0, "pre_offboard_setpoint_count");
  require(nonnegative(state.ch18_hover_wait_s), "ch18_hover_wait_s");
  require(nonnegative(state.landing_hover_s), "landing_hover_s");
  require(nonnegative(state.offboard_attempt_interval_s), "offboard_attempt_interval_s");
  const auto &trigger = config.trigger_config;
  if (trigger.enable_ch7)
    require(nonnegative(trigger.ch7_hold_s), "ch7_hold_s");
  if (trigger.enable_ch18)
    require(nonnegative(trigger.ch18_confirm_s), "ch18_confirm_s");
  if (trigger.enable_rc_loss) {
    require(positive(trigger.rc_loss_timeout_s), "rc_loss_timeout_s");
    require(nonnegative(trigger.rc_loss_debounce_s), "rc_loss_debounce_s");
  }
  if (trigger.enable_low_battery) {
    require(positive(trigger.low_battery_threshold) && trigger.low_battery_threshold < 1.0,
            "low_battery_threshold");
    require(nonnegative(trigger.low_battery_confirm_s), "low_battery_confirm_s");
  }
  // 无效 core 配置由规划失败降级处理；有效配置必须满足产品速度和停车边界。
  if (config.trajectory_config.validate()) {
    const auto &trajectory = config.trajectory_config;
    require(trajectory.v_max < 1.0, "trajectory v_max must be below 1m/s");
    require(trajectory.v_start_m_s == 0.0 && trajectory.v_end_m_s == 0.0,
            "trajectory boundary speeds must be zero");
    require(trajectory.max_deviation_m + config.route_uncertainty_m < config.route_normal_limit_m,
            "trajectory corridor budget");
  }
}

} // namespace

BacktrackUseCase::BacktrackUseCase(const UseCaseConfig &config, PlanningWorker::Planner planner)
    : config_(config), recording_buffer_(config.record_max_hz),
      trigger_evaluator_(config.trigger_config),
      trajectory_session_(std::make_unique<domain::TrajectorySession>()),
      planning_worker_(std::move(planner)), rejoin_policy_(config.rejoin_config),
      state_machine_(config.state_machine_config), current_armed_(false), current_mode_(""),
      current_position_(domain::Vec3::Zero()), current_velocity_(domain::Vec3::Zero()),
      current_yaw_(0.0), has_seen_disarm_(false), arm_after_disarm_detected_(false),
      record_delay_start_time_(0.0), record_delay_started_(false),
      trajectory_generation_requested_(false), rejoin_trigger_candidate_start_time_(-1.0),
      rejoin_success_candidate_start_time_(-1.0), yaw_alignment_start_time_(-1.0),
      yaw_alignment_timeout_reported_(false), last_path_target_velocity_(domain::Vec3::Zero()),
      has_last_path_target_velocity_(false), auto_land_yield_active_(false) {
  validateConfig(config);
  ROS_INFO("[BacktrackUseCase] Initialized");
}

TickOutput BacktrackUseCase::tick(double now) {
  if (!std::isfinite(now)) {
    TickOutput output;
    output.current_state = state_machine_.getCurrentState();
    return output;
  }
  const bool clock_reset = last_tick_time_ >= 0.0 && now < last_tick_time_;
  last_tick_time_ = now;
  const bool state_fresh = state_received_at_ >= 0.0 && now >= state_received_at_ &&
                           now - state_received_at_ <= config_.state_timeout_s;
  if (state_fresh && !current_armed_) {
    localization_reset_ = false;
    input_hold_ = route_safety_hold_ = false;
    cancelPlanning();
    trajectory_session_->reset();
    waypoint_fallback_session_.reset();
    recorded_route_.reset();
    domain::TickInput disarm;
    disarm.armed = false;
    disarm.mode = current_mode_;
    disarm.now = now;
    state_machine_.tick(disarm);
    handleFlightCycleReset(false);
    TickOutput output;
    output.current_state = state_machine_.getCurrentState();
    return output;
  }
  // 飞控 AUTO.LAND 确认优先于里程计有效性。
  if (state_fresh && current_mode_ == "AUTO.LAND") {
    handleAutoLandYield();
    domain::TickInput land;
    land.armed = current_armed_;
    land.mode = current_mode_;
    land.now = now;
    state_machine_.tick(land);
    TickOutput output;
    output.current_state = state_machine_.getCurrentState();
    return output;
  }
  const auto state_before_control = state_machine_.getCurrentState();
  const bool owned = state_before_control == domain::State::ReturnActive ||
                     state_before_control == domain::State::ReturnRejoin ||
                     state_before_control == domain::State::LandingHover ||
                     ((state_before_control == domain::State::ReturnPreOffboard ||
                       state_before_control == domain::State::Ch18HoverWait) &&
                      last_confirmed_mode_ == "OFFBOARD");
  last_confirmed_mode_ = current_mode_;
  if (state_fresh && owned && current_mode_ != "OFFBOARD") {
    cancelPlanning();
    trajectory_session_->suspend();
    last_progress_time_ = -1.0;
    route_safety_hold_ = input_hold_ = false;
    state_machine_.yieldControl();
  }

  if (clock_reset || !inputsFresh(now))
    return holdForInvalidInput(now);

  // 更新TriggerEvaluator的当前时间（用于RC丢失检测）
  trigger_evaluator_.updateNow(now);

  // 飞行周期复位检测
  handleFlightCycleReset(current_armed_);

  // AUTO.LAND让步期间不再执行任何返航触发或控制逻辑
  bool auto_land_yielding = handleAutoLandYield();

  if (!auto_land_yielding) {
    handleRecordDelay(now);
    handleTriggers(now);
    handleTrajectoryGeneration(now); // 交付/接收快照，不在控制线程求解
    handleWaypointFallbackProgress();
    handleTrajectoryProgress(now);
    handleRouteSafety();
    handleRejoin(now);
  }

  // 构造TickInput
  domain::TickInput tick_input;
  tick_input.armed = current_armed_;
  tick_input.mode = current_mode_;
  tick_input.now = now;
  tick_input.current_position = current_position_;
  tick_input.current_yaw = current_yaw_;
  const auto state_before_tick = state_machine_.getCurrentState();
  tick_input.trajectory_ready =
      (waypoint_fallback_session_.isValid() || trajectory_session_->isValid()) &&
      (state_before_tick != domain::State::ReturnPreOffboard || isTrajectoryActivationReady(now));
  tick_input.trajectory_finished =
      trajectory_session_->isFinished(current_position_, current_velocity_,
                                      kReturnEndPositionToleranceM) ||
      waypoint_fallback_session_.isFinished();

  // 驱动状态机
  state_machine_.tick(tick_input);

  // 构造输出
  TickOutput output;
  output.should_publish_setpoint = state_machine_.shouldPublishSetpoint();
  output.mode_request = state_machine_.getModeRequest();
  output.current_state = state_machine_.getCurrentState();
  output.rc_loss = trigger_evaluator_.isRCLost();
  output.low_battery = trigger_evaluator_.isLowBattery();

  // 计算setpoint
  if (output.should_publish_setpoint) {
    output.setpoint = computeSetpoint();
  }

  if ((route_safety_hold_ || input_hold_) && current_armed_ && current_mode_ == "OFFBOARD") {
    domain::SetpointTarget hold;
    hold.position = safety_hold_position_;
    hold.yaw = safety_hold_yaw_;
    hold.type_mask = 0x09F8; // position+yaw，冻结进度并等待 POSCTL 确认
    output.setpoint = hold;
    output.should_publish_setpoint = true;
  }
  return output;
}

bool BacktrackUseCase::inputsFresh(double now) const {
  return odom_valid_ && !localization_reset_ && odom_received_at_ >= 0.0 &&
         state_received_at_ >= 0.0 && now >= odom_received_at_ && now >= state_received_at_ &&
         now - odom_received_at_ <= config_.odom_timeout_s &&
         now - state_received_at_ <= config_.state_timeout_s;
}

TickOutput BacktrackUseCase::holdForInvalidInput(double now) {
  const auto state = state_machine_.getCurrentState();
  const bool controlled =
      state == domain::State::ReturnActive || state == domain::State::ReturnRejoin ||
      state == domain::State::ReturnPreOffboard || state == domain::State::Ch18HoverWait ||
      state == domain::State::LandingHover || state == domain::State::LandingTransition;
  if (controlled) {
    ROS_ERROR("[BacktrackUseCase] Input hold: odom_valid=%d localization_reset=%d odom_age=%.3f "
              "state_age=%.3f",
              odom_valid_, localization_reset_, now - odom_received_at_, now - state_received_at_);
    safety_hold_position_ = current_position_;
    safety_hold_yaw_ = current_yaw_;
    input_hold_ = true;
    trajectory_session_->suspend();
    cancelPlanning();
    state_machine_.triggerPause();
  }
  last_progress_time_ = -1.0;
  TickOutput output;
  output.current_state = state_machine_.getCurrentState();
  output.rc_loss = trigger_evaluator_.isRCLost();
  output.low_battery = trigger_evaluator_.isLowBattery();
  const bool state_fresh = state_received_at_ >= 0.0 && now >= state_received_at_ &&
                           now - state_received_at_ <= config_.state_timeout_s;
  if (input_hold_ && current_armed_ && current_mode_ == "OFFBOARD") {
    if (!localization_reset_ && odom_received_at_ >= 0.0) {
      domain::SetpointTarget hold;
      hold.position = safety_hold_position_;
      hold.yaw = safety_hold_yaw_;
      hold.type_mask = 0x09F8;
      output.setpoint = hold;
      output.should_publish_setpoint = true;
    }
    if (state_fresh)
      output.mode_request = domain::ModeRequest::POSCTL;
  }
  return output;
}

void BacktrackUseCase::onOdomReceived(const domain::Vec3 &position, const domain::Vec3 &velocity,
                                      double yaw, double stamp, double received_at) {
  if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(yaw) ||
      !std::isfinite(stamp) || !std::isfinite(received_at) || stamp < 0.0 || received_at < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[BacktrackUseCase] Invalid odometry values or receipt clock");
    odom_valid_ = false;
    return;
  }
  if (odom_received_at_ >= 0.0 && received_at < odom_received_at_) {
    odom_valid_ = false;
    return;
  }
  if (odom_stamp_ >= 0.0 && stamp <= odom_stamp_) {
    odom_valid_ = false;
    if (stamp < odom_stamp_ && current_armed_) {
      if (!localization_reset_)
        ROS_ERROR("[BacktrackUseCase] Odometry stamp moved backward: %.6f -> %.6f", odom_stamp_,
                  stamp);
      localization_reset_ = true;
    }
    return;
  }
  const double elapsed = odom_received_at_ < 0.0 ? 0.0 : received_at - odom_received_at_;
  if (current_armed_ && odom_received_at_ >= 0.0 &&
      (position - current_position_).norm() > 0.30 + 1.0 * elapsed) {
    if (!localization_reset_)
      ROS_ERROR("[BacktrackUseCase] Position reset suspected: delta=%.3f elapsed=%.3f",
                (position - current_position_).norm(), elapsed);
    localization_reset_ = true;
    odom_valid_ = false;
    return;
  }
  current_position_ = position;
  current_velocity_ = velocity;
  current_yaw_ = yaw;
  odom_received_at_ = received_at;
  odom_stamp_ = stamp;
  odom_valid_ = true;
  const auto state = state_machine_.getCurrentState();
  const bool recording =
      state == domain::State::Recording ||
      (trajectory_generation_requested_ &&
       (state == domain::State::ReturnPreOffboard || state == domain::State::Ch18HoverWait));
  if (recording && !localization_reset_ && state_received_at_ >= 0.0 &&
      received_at >= state_received_at_ &&
      received_at - state_received_at_ <= config_.state_timeout_s)
    recording_buffer_.addPoint(position, yaw, stamp);
}

void BacktrackUseCase::onRCReceived(const std::vector<uint16_t> &channels, double received_at) {
  if (!std::isfinite(received_at) || received_at < 0.0)
    return;
  trigger_evaluator_.updateRC(channels);
  trigger_evaluator_.updateLastRcMsgTime(received_at);
}

void BacktrackUseCase::onStateReceived(bool armed, const std::string &mode, double received_at) {
  if (!std::isfinite(received_at) || received_at < 0.0 ||
      (state_received_at_ >= 0.0 && received_at < state_received_at_))
    return;
  current_armed_ = armed;
  current_mode_ = mode;
  state_received_at_ = received_at;
  if (!armed) {
    odom_stamp_ = -1.0;
    localization_reset_ = false;
    has_seen_disarm_ = true;
    arm_after_disarm_detected_ = false;
    cancelPlanning();
    domain::TickInput disarm;
    disarm.armed = false;
    disarm.mode = mode;
    disarm.now = received_at;
    state_machine_.tick(disarm);
  }
}

void BacktrackUseCase::onBatteryReceived(double percentage) {
  trigger_evaluator_.updateBattery(percentage);
}

// ============================================================================
// 私有方法实现
// ============================================================================

void BacktrackUseCase::handleFlightCycleReset(bool armed) {
  // 检测DISARM
  if (!armed) {
    has_seen_disarm_ = true;
    arm_after_disarm_detected_ = false;
  }

  // 检测"DISARM之后的ARM"
  if (armed && has_seen_disarm_ && !arm_after_disarm_detected_) {
    arm_after_disarm_detected_ = true;
    has_seen_disarm_ = false;

    // 复位所有状态
    ROS_INFO("[BacktrackUseCase] Flight cycle reset: ARM after DISARM detected");
    recording_buffer_.reset();
    recorded_route_.reset();
    route_safety_hold_ = false;
    last_progress_time_ = -1.0;
    trajectory_session_->reset();
    waypoint_fallback_session_.reset();
    record_delay_started_ = false;
    cancelPlanning();
    rejoin_trigger_candidate_start_time_ = -1.0;
    rejoin_success_candidate_start_time_ = -1.0;
    yaw_alignment_start_time_ = -1.0;
    yaw_alignment_timeout_reported_ = false;
    last_path_target_velocity_ = domain::Vec3::Zero();
    has_last_path_target_velocity_ = false;
    auto_land_yield_active_ = false;
    input_hold_ = false;
    trigger_evaluator_ = domain::TriggerEvaluator(config_.trigger_config);

    // 进入WaitRecordDelay状态（通过状态机处理）
    // 状态机会在tick中检测ARM并自动转换
  }
}

bool BacktrackUseCase::handleAutoLandYield() {
  if (!current_armed_) {
    auto_land_yield_active_ = false;
    return false;
  }

  if (current_mode_ == "AUTO.LAND" && !auto_land_yield_active_) {
    ROS_WARN("[BacktrackUseCase] AUTO.LAND detected, yielding until DISARM");
    cancelPlanning();
    trajectory_session_->reset();
    waypoint_fallback_session_.reset();
    rejoin_trigger_candidate_start_time_ = -1.0;
    rejoin_success_candidate_start_time_ = -1.0;
    yaw_alignment_start_time_ = -1.0;
    yaw_alignment_timeout_reported_ = false;
    has_last_path_target_velocity_ = false;
    record_delay_started_ = false;
    auto_land_yield_active_ = true;
    route_safety_hold_ = input_hold_ = false;
  }

  return auto_land_yield_active_;
}

void BacktrackUseCase::handleRecordDelay(double now) {
  auto current_state = state_machine_.getCurrentState();

  // WaitRecordDelay状态：等待延时后进入Recording
  if (current_state == domain::State::WaitRecordDelay) {
    if (!record_delay_started_) {
      record_delay_start_time_ = now;
      record_delay_started_ = true;
      ROS_INFO("[BacktrackUseCase] Record delay started: %.1f s", config_.record_delay_s);
    }

    double elapsed = now - record_delay_start_time_;
    if (elapsed >= config_.record_delay_s) {
      ROS_INFO("[BacktrackUseCase] Record delay elapsed, starting recording");
      state_machine_.startRecording();
    }
  } else {
    // 不在WaitRecordDelay状态，重置延时标志
    if (record_delay_started_) {
      record_delay_started_ = false;
    }
  }
}

void BacktrackUseCase::handleTriggers(double now) {
  auto current_state = state_machine_.getCurrentState();

  // AUTO.LAND模式下跳过所有触发检测，让步给外部降落控制
  if (current_mode_ == "AUTO.LAND") {
    return;
  }

  bool rc_loss_event = trigger_evaluator_.checkRCLossEvent();
  bool low_battery_event = trigger_evaluator_.checkLowBatteryEvent();

  // 在Recording状态检测返航触发
  if (current_state == domain::State::Recording) {
    bool has_min_points = checkMinPoints();

    // 通道7长按检测
    if (trigger_evaluator_.checkCh7LongPress()) {
      handleReturnTrigger(domain::Event::Ch7LongPress, has_min_points, "Ch7 long press");
      return;
    }

    // 通道18故障确认检测
    if (trigger_evaluator_.checkCh18Failsafe()) {
      handleReturnTrigger(domain::Event::Ch18FailsafeConfirmed, has_min_points,
                          "Ch18 failsafe confirmed");
      return;
    }

    // RC消息丢失检测（边沿触发，仅触发一次）
    if (rc_loss_event) {
      handleReturnTrigger(domain::Event::RCLossTriggered, has_min_points, "RC loss");
      return;
    }

    // 低电量触发检测（边沿触发，仅触发一次）
    if (low_battery_event) {
      handleReturnTrigger(domain::Event::LowBatteryTriggered, has_min_points, "Low battery");
      return;
    }
  }

  // 在Ch18HoverWait或Return*状态时，检测Ch7按下事件（暂停）
  if (current_state == domain::State::Ch18HoverWait ||
      current_state == domain::State::ReturnPreOffboard ||
      current_state == domain::State::ReturnActive ||
      current_state == domain::State::ReturnRejoin) {
    // 需要先调用checkCh7LongPress()以更新Ch7状态机
    trigger_evaluator_.checkCh7LongPress();

    if (trigger_evaluator_.checkCh7PressEvent()) {
      ROS_INFO("[BacktrackUseCase] Ch7 press event detected, pausing");
      cancelPlanning();
      trajectory_session_->suspend();
      last_progress_time_ = -1.0;
      state_machine_.triggerPause();
      return;
    }
  }

  if (current_state == domain::State::Paused || current_state == domain::State::ControlYield) {
    if (rc_loss_event && current_state == domain::State::Paused && !route_safety_hold_ &&
        !input_hold_) {
      if (!resumeReturn(domain::Event::RCLossTriggered, now))
        ROS_WARN("[BacktrackUseCase] RC loss resume rejected: no valid local remaining path");
      return;
    }
    if (trigger_evaluator_.checkCh7LongPress() && !resumeReturn(domain::Event::Ch7LongPress, now))
      ROS_WARN("[BacktrackUseCase] Resume rejected: no valid local remaining path");
  }
}

void BacktrackUseCase::handleReturnTrigger(domain::Event event, bool has_min_points,
                                           const char *trigger_name) {
  if (has_min_points) {
    ROS_INFO("[BacktrackUseCase] %s detected", trigger_name);
    cancelPlanning();
    trajectory_session_->reset();
    waypoint_fallback_session_.reset();
    route_safety_hold_ = false;
    last_progress_time_ = -1.0;
    if (!recorded_route_.build(recording_buffer_.getPoints())) {
      triggerImmediateLanding();
      return;
    }
    state_machine_.lockPosition(current_position_, current_yaw_);
    state_machine_.triggerReturn(event);
    trajectory_generation_requested_ = true;
    return;
  }

  ROS_WARN(
      "[BacktrackUseCase] %s detected but only %zu points recorded (< %d), triggering AUTO.LAND",
      trigger_name, recording_buffer_.size(), config_.record_min_points);
  triggerImmediateLanding();
}

void BacktrackUseCase::cancelPlanning() {
  trajectory_generation_requested_ = false;
  planning_inflight_ = false;
  planning_worker_.cancel();
}

void BacktrackUseCase::handleTrajectoryGeneration(double now) {
  if (state_machine_.getCurrentState() != domain::State::ReturnPreOffboard)
    return;
  if (trajectory_generation_requested_ && !trajectory_session_->isValid() &&
      !waypoint_fallback_session_.isValid()) {
    // 触发后继续录制实际制动尾段，停稳后才冻结原路并交付规划。
    if (current_velocity_.norm() > 0.10)
      return;
    if (!recorded_route_.build(recording_buffer_.getPoints())) {
      triggerImmediateLanding();
      return;
    }
    PlanningInput input;
    input.points = recording_buffer_.getPointsCopy();
    input.config = config_.trajectory_config;
    input.debug_sample_dt = config_.debug_trajectory_sample_dt;
    input.dumper = debug_dumper_;
    input.publisher = debug_publisher_;
    planning_worker_.request(std::move(input));
    trajectory_generation_requested_ = false;
    planning_inflight_ = true;
  }
  if (!planning_inflight_)
    return;
  auto result = planning_worker_.takeResult();
  if (!result)
    return;
  planning_inflight_ = false;
  // 结果仅在本任务、准备态且输入新鲜时接收。worker 换代已阻止旧任务结果交付。
  if (!inputsFresh(now)) {
    cancelPlanning();
    return;
  }
  trajectory_session_ = std::move(result->trajectory);
  waypoint_fallback_session_ = std::move(result->fallback);
  rejoin_trigger_candidate_start_time_ = rejoin_success_candidate_start_time_ = -1.0;
  yaw_alignment_start_time_ = now;
  yaw_alignment_timeout_reported_ = false;
  has_last_path_target_velocity_ = false;
  last_progress_time_ = -1.0;
  const bool local_ready = trajectory_session_->isValid()
                               ? trajectory_session_->resumeAt(current_position_)
                               : (waypoint_fallback_session_.isValid() &&
                                  waypoint_fallback_session_.resumeAt(current_position_));
  if ((trajectory_session_->isValid() || waypoint_fallback_session_.isValid()) && !local_ready) {
    ROS_WARN("[BacktrackUseCase] Planned route cannot join current local position; pausing");
    pauseForRouteSafety();
    return;
  }
  if (trajectory_session_->isValid() || waypoint_fallback_session_.isValid()) {
    const auto locked = state_machine_.getStateData().locked_position;
    const double yaw = trajectory_session_->isValid()
                           ? trajectory_session_->getTargetState().yaw
                           : waypoint_fallback_session_.currentTarget().yaw;
    state_machine_.lockPosition(locked, yaw);
  } else if (!waypoint_fallback_session_.isValid())
    triggerImmediateLanding();
}

void BacktrackUseCase::handleWaypointFallbackProgress() {
  auto current_state = state_machine_.getCurrentState();
  if (current_state != domain::State::ReturnActive || !waypoint_fallback_session_.isValid()) {
    return;
  }

  waypoint_fallback_session_.updateProgress(current_position_, current_velocity_, current_yaw_);
}

void BacktrackUseCase::handleTrajectoryProgress(double now) {
  const auto state = state_machine_.getCurrentState();
  if ((state != domain::State::ReturnActive && state != domain::State::ReturnRejoin) ||
      !trajectory_session_->isValid()) {
    return;
  }

  const double dt = last_progress_time_ < 0.0 ? 0.05 : now - last_progress_time_;
  last_progress_time_ = now;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1001) {
    trajectory_session_->suspend();
    return;
  }
  const bool projection_ok =
      state == domain::State::ReturnRejoin
          ? trajectory_session_->updateRejoinProgress(
                current_position_, config_.rejoin_config.lookahead_time_s,
                config_.rejoin_config.trigger_dist_m, std::min(0.10, dt),
                config_.rejoin_config.v_max_m_s)
          : trajectory_session_->updateProgress(
                current_position_, config_.rejoin_config.lookahead_time_s, std::min(0.10, dt));
  if (!projection_ok) {
    ROS_ERROR_THROTTLE(
        1.0, "[BacktrackUseCase] Local path projection failed; holding last valid target");
    if (state == domain::State::ReturnActive) {
      ROS_WARN("[BacktrackUseCase] Entering Rejoin because local path projection is unavailable");
      state_machine_.enterRejoin();
      rejoin_trigger_candidate_start_time_ = -1.0;
    }
  }
}

bool BacktrackUseCase::resumeReturn(domain::Event event, double now) {
  if (!checkMinPoints() || !recorded_route_.valid() || !recorded_route_.update(current_position_) ||
      recorded_route_.distance() > config_.rejoin_config.success_dist_m)
    return false;
  if (trajectory_session_->isValid() && !trajectory_session_->resumeAt(current_position_))
    return false;
  if (waypoint_fallback_session_.isValid() &&
      !waypoint_fallback_session_.resumeAt(current_position_))
    return false;
  route_safety_hold_ = input_hold_ = false;
  last_progress_time_ = -1.0;
  const double yaw =
      trajectory_session_->isValid()
          ? trajectory_session_->getTargetState().yaw
          : (waypoint_fallback_session_.isValid() ? waypoint_fallback_session_.currentTarget().yaw
                                                  : current_yaw_);
  state_machine_.lockPosition(current_position_, yaw);
  trajectory_generation_requested_ =
      !trajectory_session_->isValid() && !waypoint_fallback_session_.isValid();
  state_machine_.triggerReturn(event);
  yaw_alignment_start_time_ = now;
  yaw_alignment_timeout_reported_ = false;
  return true;
}

void BacktrackUseCase::handleRouteSafety() {
  const auto state = state_machine_.getCurrentState();
  const bool preparing =
      state == domain::State::ReturnPreOffboard || state == domain::State::Ch18HoverWait;
  if (state != domain::State::ReturnActive && state != domain::State::ReturnRejoin && !preparing)
    return;
  // 冻结前的原路仍在接收制动尾段；坐标有效性保护始终由输入层执行。
  if (preparing && trajectory_generation_requested_)
    return;
  const bool valid = recorded_route_.update(current_position_);
  const double predicted = recorded_route_.predictedDeviation(
      current_position_, current_velocity_, config_.route_reaction_s, config_.route_braking_m_s2,
      config_.route_uncertainty_m);
  const bool unsafe = !valid || recorded_route_.distance() >= config_.route_hard_limit_m ||
                      predicted >= config_.route_normal_limit_m;
  const bool fallback_off_route =
      waypoint_fallback_session_.isValid() &&
      recorded_route_.distance() >= config_.rejoin_config.trigger_dist_m;
  if (unsafe || fallback_off_route) {
    ROS_ERROR("[BacktrackUseCase] Original route safety hold: distance=%.3f predicted=%.3f",
              recorded_route_.distance(), predicted);
    pauseForRouteSafety();
  } else if (state == domain::State::ReturnActive &&
             recorded_route_.distance() >= config_.rejoin_config.trigger_dist_m) {
    state_machine_.enterRejoin();
  }
}

void BacktrackUseCase::pauseForRouteSafety() {
  safety_hold_position_ = current_position_;
  safety_hold_yaw_ = current_yaw_;
  route_safety_hold_ = true;
  cancelPlanning();
  trajectory_session_->suspend();
  state_machine_.triggerPause();
}

bool BacktrackUseCase::isTrajectoryActivationReady(double now) {
  if (!trajectory_session_->isValid() && !waypoint_fallback_session_.isValid())
    return false;
  const auto target_position = trajectory_session_->isValid()
                                   ? trajectory_session_->getTargetState().position
                                   : waypoint_fallback_session_.currentTarget().position;
  if ((target_position - current_position_).norm() > 0.40 || current_velocity_.norm() > 0.20)
    return false;
  const double target_yaw = trajectory_session_->isValid()
                                ? trajectory_session_->getTargetState().yaw
                                : waypoint_fallback_session_.currentTarget().yaw;
  const double yaw_error = wrappedAngleError(current_yaw_, target_yaw);
  if (yaw_error <= kYawAlignmentToleranceRad) {
    return true;
  }

  if (yaw_alignment_start_time_ < 0.0) {
    yaw_alignment_start_time_ = now;
  }
  if (!yaw_alignment_timeout_reported_ &&
      now - yaw_alignment_start_time_ >= kYawAlignmentTimeoutS) {
    ROS_ERROR("[BacktrackUseCase] Start yaw alignment timed out: error=%.1f deg; holding position",
              yaw_error * 180.0 / kPi);
    yaw_alignment_timeout_reported_ = true;
  }
  return false;
}

void BacktrackUseCase::handleRejoin(double now) {
  auto current_state = state_machine_.getCurrentState();

  // 在ReturnActive状态检测偏离
  if (current_state == domain::State::ReturnActive) {
    if (waypoint_fallback_session_.isValid()) {
      rejoin_success_candidate_start_time_ = -1.0;
      rejoin_trigger_candidate_start_time_ = -1.0;
      return;
    }

    rejoin_success_candidate_start_time_ = -1.0;
    if (!trajectory_session_->isValid()) {
      return;
    }

    if (!trajectory_session_->hasValidProjection()) {
      return;
    }

    // 只使用当前位置到局部路径投影点的横向距离，纵向速度落后不触发回归。
    if (rejoin_policy_.shouldTriggerRejoin(trajectory_session_->getCrossTrackError())) {
      if (rejoin_trigger_candidate_start_time_ < 0.0) {
        rejoin_trigger_candidate_start_time_ = now;
      }

      double hold_duration = now - rejoin_trigger_candidate_start_time_;
      if (hold_duration >= config_.rejoin_config.trigger_hold_s) {
        ROS_WARN("[BacktrackUseCase] Deviation detected for %.2f s, entering rejoin mode",
                 hold_duration);
        state_machine_.enterRejoin();
        rejoin_trigger_candidate_start_time_ = -1.0;
      }
    } else {
      rejoin_trigger_candidate_start_time_ = -1.0;
    }
    return;
  }

  // 在ReturnRejoin状态检测回归成功
  if (current_state == domain::State::ReturnRejoin) {
    rejoin_trigger_candidate_start_time_ = -1.0;
    if (!trajectory_session_->isValid() || !trajectory_session_->hasValidProjection()) {
      return;
    }

    const auto &path_target = trajectory_session_->getTargetState();
    const domain::Vec3 published_target_velocity =
        has_last_path_target_velocity_ ? last_path_target_velocity_ : path_target.velocity;
    if (rejoin_policy_.shouldExitRejoin(trajectory_session_->getCrossTrackError(),
                                        current_velocity_, path_target.velocity,
                                        published_target_velocity)) {
      if (rejoin_success_candidate_start_time_ < 0.0) {
        rejoin_success_candidate_start_time_ = now;
      }

      double hold_duration = now - rejoin_success_candidate_start_time_;
      if (hold_duration >= config_.rejoin_config.success_hold_s) {
        ROS_INFO("[BacktrackUseCase] Rejoin successful for %.2f s, returning to active mode",
                 hold_duration);
        state_machine_.exitRejoin();
        rejoin_success_candidate_start_time_ = -1.0;
      }
    } else {
      rejoin_success_candidate_start_time_ = -1.0;
    }
    return;
  }

  rejoin_trigger_candidate_start_time_ = -1.0;
  rejoin_success_candidate_start_time_ = -1.0;
}

domain::SetpointTarget BacktrackUseCase::computeSetpoint() {
  auto current_state = state_machine_.getCurrentState();

  // 优先使用状态机提供的setpoint（固定点悬停）
  auto state_machine_setpoint = state_machine_.getSetpointTarget();
  if (state_machine_setpoint.has_value()) {
    return state_machine_setpoint.value();
  }

  if (current_state == domain::State::ReturnActive && waypoint_fallback_session_.isValid())
    return waypoint_fallback_session_.currentTarget();
  if ((current_state == domain::State::ReturnActive ||
       current_state == domain::State::ReturnRejoin) &&
      trajectory_session_->isValid()) {
    const auto &state = trajectory_session_->getTargetState();
    domain::SetpointTarget target;
    target.position = state.position;
    target.velocity = state.velocity;
    target.yaw = state.yaw;
    target.yaw_rate = state.yaw_rate;
    target.acceleration = state.acceleration;
    target.type_mask =
        state.acceleration ? 0x0000 : domain::position_target_mask::kIgnoreAcceleration;
    last_path_target_velocity_ = target.velocity;
    has_last_path_target_velocity_ = true;
    return target;
  }
  // 状态与会话不一致时保持有效当前位置，避免发送默认地图原点。
  ROS_ERROR_THROTTLE(1.0, "[BacktrackUseCase] Missing control target; holding current position");
  domain::SetpointTarget hold;
  hold.position = current_position_;
  hold.yaw = current_yaw_;
  hold.type_mask = 0x09F8;
  return hold;
}

bool BacktrackUseCase::checkMinPoints() const {
  size_t point_count = recording_buffer_.size();
  if (point_count < static_cast<size_t>(config_.record_min_points)) {
    ROS_WARN_THROTTLE(5.0, "[BacktrackUseCase] Insufficient points for return: %zu < %d",
                      point_count, config_.record_min_points);
    return false;
  }
  return true;
}

void BacktrackUseCase::triggerImmediateLanding() {
  cancelPlanning();
  trajectory_session_->reset();
  waypoint_fallback_session_.reset();
  rejoin_trigger_candidate_start_time_ = -1.0;
  rejoin_success_candidate_start_time_ = -1.0;
  yaw_alignment_start_time_ = -1.0;
  yaw_alignment_timeout_reported_ = false;
  has_last_path_target_velocity_ = false;
  state_machine_.lockPosition(current_position_, current_yaw_);
  state_machine_.triggerLanding();
}

void BacktrackUseCase::setDebugDumper(std::shared_ptr<infrastructure::DebugDumper> dumper) {
  debug_dumper_ = std::move(dumper);
}

void BacktrackUseCase::setDebugPublisher(std::shared_ptr<adapters::DebugPublisher> publisher) {
  debug_publisher_ = std::move(publisher);
}

} // namespace application
} // namespace px4_backtrack
