#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace px4_backtrack {
namespace application {

namespace {

/// 终点位置容差：与 Rejoin 走廊无关（固定 0.2 m）
constexpr double kReturnEndPositionToleranceM = 0.20;

/// 时间比较容差，避免浮点误差让边界判定晚一拍
constexpr double kTimeEpsS = 1e-9;

domain::SetpointTarget holdSetpoint(const domain::Vec3& position, double yaw) {
  domain::SetpointTarget target;
  target.position = position;
  target.velocity = domain::Vec3::Zero();
  target.yaw = yaw;
  target.yaw_rate = 0.0;
  target.acceleration = std::nullopt;
  target.type_mask = domain::position_target_mask::kIgnoreAcceleration;
  return target;
}

domain::SetpointTarget pathSetpoint(const trajectory_core::TrajectoryState& state) {
  domain::SetpointTarget target;
  target.position = state.position;
  target.velocity = state.velocity;
  target.yaw = state.yaw;
  target.yaw_rate = state.yaw_rate;
  target.acceleration = state.acceleration;
  target.type_mask = state.acceleration.has_value()
                         ? 0x0000
                         : domain::position_target_mask::kIgnoreAcceleration;
  return target;
}

domain::StepEvent makeEvent(domain::EventType type, const char* name,
                            const char* reason, std::string detail) {
  domain::StepEvent event;
  event.type = type;
  event.name = name;
  event.reason = reason;
  event.detail = std::move(detail);
  return event;
}

std::string formatSeconds(double seconds) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << seconds;
  return out.str();
}

}  // namespace

BacktrackUseCase::BacktrackUseCase(const UseCaseConfig& config)
  : config_(config),
    link_monitor_(config.link_config),
    pilot_input_(config.pilot_config),
    battery_monitor_(config.battery_config),
    airborne_monitor_(config.airborne_config),
    state_machine_(config.state_machine_config),
    mode_commander_(config.mode_commander_config),
    recording_buffer_(config.record_max_hz),
    rejoin_policy_(config.rejoin_config)
{
}

// ============================================================================
// 输入
// ============================================================================

void BacktrackUseCase::onOdom(double t, const domain::Vec3& position,
                              const domain::Vec3& velocity, double yaw,
                              double stamp) {
  // 非有限值视为没有收到，由新鲜度判定接管（R-73）
  if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(yaw)) {
    return;
  }
  has_odom_ = true;
  odom_t_ = t;
  position_ = position;
  velocity_ = velocity;
  yaw_ = yaw;
  airborne_monitor_.onHeight(position.z());
  if (record_) {
    recording_buffer_.addPoint(position, yaw, stamp);
  }
}

void BacktrackUseCase::onRc(double t, const std::vector<uint16_t>& channels) {
  // LinkMonitor 先于 PilotInput：门控用本帧的链路状态
  link_monitor_.onRcFrame(t, channels);
  pilot_input_.onRcFrame(t, channels, link_monitor_.status());
}

void BacktrackUseCase::onFcuState(double t, bool connected, bool armed,
                                  const std::string& mode) {
  // MAVROS 未连上飞控时也发布 armed=false，不能当作上锁
  if (!connected) {
    return;
  }
  has_fcu_state_ = true;
  fcu_state_t_ = t;
  armed_ = armed;
  mode_ = mode;
  airborne_monitor_.onArmed(armed);
}

void BacktrackUseCase::onExtendedState(double t, domain::LandedState landed_state) {
  airborne_monitor_.onLandedState(t, landed_state);
}

void BacktrackUseCase::onBattery(double t, double percentage) {
  battery_monitor_.onReading(t, percentage);
}

void BacktrackUseCase::onModeRequestResult(double t, const std::string& mode,
                                           bool sent) {
  mode_commander_.onResult(t, mode, sent);
}

// ============================================================================
// 每拍
// ============================================================================

TickOutput BacktrackUseCase::tick(double now) {
  // 条件层推进到当前时间
  link_monitor_.update(now);
  pilot_input_.update(now, link_monitor_.status());
  battery_monitor_.update(now);
  airborne_monitor_.update(now);

  // 用最近一次的执行结果组装条件，再决策（R-71）
  const domain::Conditions c = assembleConditions(now);
  domain::StepOutput step = state_machine_.step(c);

  TickOutput out;
  out.events = std::move(step.events);
  executeCommands(step, out.events);
  updateFollow(step, now);
  out.setpoint = computeSetpoint(step);
  trackPublishing(now, out.setpoint.has_value(), out.events);
  out.mode_request = mode_commander_.update(
      now, domain::modeIntentName(step.mode_intent), mode_, c.fcu_state_fresh);
  for (auto& event : mode_commander_.takeEvents()) {
    out.events.push_back(std::move(event));
  }
  record_ = step.record;

  out.state = domain::reportedStateName(step.phase, step.follow);
  out.control = step.control;
  out.link_lost = c.link.lost;
  out.low_battery = c.battery.low;
  out.alerts = step.alerts;
  out.flight_id = step.flight_id;
  out.mission_id = step.mission_id;
  return out;
}

domain::Conditions BacktrackUseCase::assembleConditions(double now) const {
  domain::Conditions c;
  c.now = now;

  c.fcu_state_fresh = has_fcu_state_ &&
                      now - fcu_state_t_ <= config_.fcu_state_stale_s;
  c.fcu_state_t = fcu_state_t_;
  c.armed = armed_;
  c.mode = mode_;

  c.link = link_monitor_.status();
  c.pilot = pilot_input_.status();
  c.battery = battery_monitor_.status();
  c.airborne = airborne_monitor_.status();

  c.odom_stale = !has_odom_ || now - odom_t_ > config_.odom_stale_s;
  c.odom_lost = !has_odom_ || now - odom_t_ > config_.odom_timeout_s;
  c.position = position_;
  c.yaw = yaw_;

  c.has_min_points = recording_buffer_.size() >=
                     static_cast<size_t>(config_.record_min_points);
  c.plan_status = plan_status_;
  if (plan_status_ == domain::PlanStatus::Ready) {
    c.path_start_yaw = trajectory_session_.getTrajectory()->evaluateByArcLength(0.0).yaw;
    c.path_target_yaw = trajectory_session_.getTargetState().yaw;
    // 按当前位置投影，不推进进度：跟随冻结时也反映当前偏差（R-74）
    c.projection_ok = trajectory_session_.probe(position_, c.cross_track_m);
    c.finished = trajectory_session_.isFinished(position_, velocity_,
                                                kReturnEndPositionToleranceM);
  } else if (plan_status_ == domain::PlanStatus::ReadyFallback) {
    // 航点降级没有切向和投影：不要求对准，按投影有效、偏差 0 提供（第 5.6 节）
    c.projection_ok = true;
    c.cross_track_m = 0.0;
    c.finished = waypoint_fallback_session_.isFinished();
  }
  c.deviation_confirmed = deviation_confirmed_;
  c.rejoin_confirmed = rejoin_confirmed_;

  c.stream_gap_recent = streamGapRecent(now);
  c.mode_request_failed = mode_commander_.failed();
  c.debug_log_failed = debug_log_failed_;
  return c;
}

// ============================================================================
// 执行层
// ============================================================================

void BacktrackUseCase::executeCommands(const domain::StepOutput& step,
                                       std::vector<domain::StepEvent>& events) {
  const auto& commands = step.commands;
  if (commands.new_flight) {
    // 新的飞行周期：清空录制和会话，复位电量锁存与按键（第 6.3 节规则 1）
    recording_buffer_.reset();
    resetPlan();
    pilot_input_.reset();
    battery_monitor_.reset();
    publishing_ = false;
    last_gap_end_t_.reset();
  }
  if (commands.cancel_planning) {
    resetPlan();
  } else if (commands.start_planning) {
    plan(events);
  }
  // commands.trail_cut：R5 实现截断
}

void BacktrackUseCase::plan(std::vector<domain::StepEvent>& events) {
  resetPlan();
  const auto& points = recording_buffer_.getPoints();
  const std::string point_count = "points=" + std::to_string(points.size());

  if (trajectory_session_.generate(points, config_.trajectory_config)) {
    plan_status_ = domain::PlanStatus::Ready;
    events.push_back(makeEvent(
        domain::EventType::Planning, "completed", "READY",
        point_count + " cost_ms=" +
            formatSeconds(trajectory_session_.getLastGenerateCostMs())));
    performDebugDump();
    return;
  }

  switch (trajectory_session_.getLastFailureKind()) {
    case domain::TrajectorySession::FailureKind::TooCloseGeometry:
      plan_status_ = domain::PlanStatus::FailedTooClose;
      events.push_back(makeEvent(domain::EventType::Planning, "failed",
                                 "TOO_CLOSE", point_count));
      return;
    case domain::TrajectorySession::FailureKind::InvalidConfig:
      plan_status_ = domain::PlanStatus::FailedConfig;
      events.push_back(makeEvent(domain::EventType::Planning, "failed",
                                 "INVALID_CONFIG", point_count));
      return;
    default:
      break;
  }

  // 其他失败：航点降级；降级也构建失败则原地降落（第 7.5 节）
  if (waypoint_fallback_session_.build(points)) {
    plan_status_ = domain::PlanStatus::ReadyFallback;
    events.push_back(makeEvent(
        domain::EventType::Planning, "completed", "WAYPOINT_FALLBACK",
        point_count + " waypoints=" +
            std::to_string(waypoint_fallback_session_.size())));
  } else {
    plan_status_ = domain::PlanStatus::FailedFallback;
    events.push_back(makeEvent(domain::EventType::Planning, "failed",
                               "FALLBACK_FAILED", point_count));
  }
}

void BacktrackUseCase::resetPlan() {
  trajectory_session_.reset();
  waypoint_fallback_session_.reset();
  plan_status_ = domain::PlanStatus::None;
  deviation_confirmed_ = false;
  rejoin_confirmed_ = false;
  deviation_since_ = -1.0;
  rejoin_since_ = -1.0;
  has_last_path_velocity_ = false;
}

void BacktrackUseCase::updateFollow(const domain::StepOutput& step, double now) {
  // 跟随方式改变或不推进时，持续判定重新开始
  if (step.follow != last_follow_ || !step.follow_active) {
    deviation_confirmed_ = false;
    rejoin_confirmed_ = false;
    deviation_since_ = -1.0;
    rejoin_since_ = -1.0;
  }
  last_follow_ = step.follow;
  if (!step.follow_active) {
    return;
  }

  if (plan_status_ == domain::PlanStatus::ReadyFallback) {
    waypoint_fallback_session_.updateProgress(position_);
    return;
  }
  if (plan_status_ != domain::PlanStatus::Ready) {
    return;
  }

  const auto& rejoin = config_.rejoin_config;
  if (step.follow == domain::FollowMode::OnPath) {
    const bool projection_ok = trajectory_session_.updateProgress(
        position_, rejoin.lookahead_time_s);
    // 局部投影失败也算偏离（与 dev 相同）；纵向落后不算偏离
    if (!projection_ok) {
      deviation_confirmed_ = true;
      return;
    }
    if (!rejoin_policy_.shouldTriggerRejoin(trajectory_session_.getCrossTrackError())) {
      deviation_since_ = -1.0;
      deviation_confirmed_ = false;
      return;
    }
    if (deviation_since_ < 0.0) {
      deviation_since_ = now;
    }
    deviation_confirmed_ = now - deviation_since_ + kTimeEpsS >= rejoin.trigger_hold_s;
    return;
  }

  const bool projection_ok = trajectory_session_.updateRejoinProgress(
      position_, rejoin.lookahead_time_s, rejoin.trigger_dist_m);
  const auto& target = trajectory_session_.getTargetState();
  const domain::Vec3 published =
      has_last_path_velocity_ ? last_path_velocity_ : target.velocity;
  if (!projection_ok ||
      !rejoin_policy_.shouldExitRejoin(trajectory_session_.getCrossTrackError(),
                                       velocity_, target.velocity, published)) {
    rejoin_since_ = -1.0;
    rejoin_confirmed_ = false;
    return;
  }
  if (rejoin_since_ < 0.0) {
    rejoin_since_ = now;
  }
  rejoin_confirmed_ = now - rejoin_since_ + kTimeEpsS >= rejoin.success_hold_s;
}

std::optional<domain::SetpointTarget> BacktrackUseCase::computeSetpoint(
    const domain::StepOutput& step) {
  switch (step.setpoint) {
    case domain::SetpointKind::None:
      return std::nullopt;

    case domain::SetpointKind::Hold:
      return holdSetpoint(step.hold_position, step.hold_yaw);

    case domain::SetpointKind::PathTarget:
      if (plan_status_ == domain::PlanStatus::ReadyFallback) {
        return waypoint_fallback_session_.currentTarget();
      }
      if (plan_status_ == domain::PlanStatus::Ready) {
        const auto target = pathSetpoint(trajectory_session_.getTargetState());
        last_path_velocity_ = target.velocity;
        has_last_path_velocity_ = true;
        return target;
      }
      break;

    case domain::SetpointKind::PathTargetRejoin:
      if (plan_status_ == domain::PlanStatus::Ready) {
        const auto& path_state = trajectory_session_.getTargetState();
        const domain::Vec3 previous =
            has_last_path_velocity_ ? last_path_velocity_ : path_state.velocity;
        const auto target = pathSetpoint(rejoin_policy_.applyRejoinLimits(
            path_state, trajectory_session_.getCrossTrackError(), previous));
        last_path_velocity_ = target.velocity;
        has_last_path_velocity_ = true;
        return target;
      }
      break;
  }
  // 拿不到目标时保持当前位置，绝不输出原点（P3）
  return holdSetpoint(position_, yaw_);
}

void BacktrackUseCase::trackPublishing(double now, bool published,
                                       std::vector<domain::StepEvent>& events) {
  if (!published) {
    // 不发布即结束这一段（R-69）
    publishing_ = false;
    return;
  }
  if (publishing_) {
    const double gap = now - last_publish_t_;
    if (gap + kTimeEpsS >= config_.fallback_gap_s) {
      last_gap_end_t_ = now;
      events.push_back(makeEvent(domain::EventType::Stream, "setpoint_gap", "",
                                 "gap_s=" + formatSeconds(gap)));
    }
  }
  publishing_ = true;
  last_publish_t_ = now;
}

bool BacktrackUseCase::streamGapRecent(double now) const {
  // 卡顿结束后的第一拍：距上次发布的时间也算一次间隔
  const bool ongoing = publishing_ &&
                       now - last_publish_t_ + kTimeEpsS >= config_.fallback_gap_s;
  const bool recent = last_gap_end_t_ &&
                      now - *last_gap_end_t_ <= config_.fallback_window_s + kTimeEpsS;
  return ongoing || recent;
}

// ============================================================================
// Debug
// ============================================================================

void BacktrackUseCase::setDebugDumper(
    std::shared_ptr<infrastructure::DebugDumper> dumper) {
  debug_dumper_ = std::move(dumper);
}

void BacktrackUseCase::setDebugPublisher(
    std::shared_ptr<adapters::DebugPublisher> publisher) {
  debug_publisher_ = std::move(publisher);
}

void BacktrackUseCase::performDebugDump() {
  if (!debug_dumper_ && !debug_publisher_) return;

  const auto* traj = trajectory_session_.getTrajectory();
  if (!traj || !traj->isValid()) return;

  auto states = traj->sample(config_.debug_trajectory_sample_dt);
  const auto& waypoints = recording_buffer_.getPoints();

  if (debug_dumper_) {
    debug_dumper_->dumpWaypoints(waypoints);
    debug_dumper_->dumpTrajectory(states);
  }

  if (debug_publisher_) {
    debug_publisher_->publishWaypoints(waypoints);
    debug_publisher_->publishTrajectory(states);
  }
}

}  // namespace application
}  // namespace px4_backtrack
