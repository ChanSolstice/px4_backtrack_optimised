#include "px4_backtrack/application/backtrack_usecase.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <ros/console.h>

namespace px4_backtrack {
namespace application {

BacktrackUseCase::BacktrackUseCase(const UseCaseConfig& config)
  : config_(config),
    recording_buffer_(config.record_max_hz),
    trigger_evaluator_(config.trigger_config),
    rejoin_policy_(config.rejoin_config),
    state_machine_(config.state_machine_config),
    current_armed_(false),
    current_mode_(""),
    current_position_(domain::Vec3::Zero()),
    current_yaw_(0.0),
    has_seen_disarm_(false),
    arm_after_disarm_detected_(false),
    record_delay_start_time_(0.0),
    record_delay_started_(false),
    trajectory_generation_requested_(false),
    rejoin_trigger_candidate_start_time_(-1.0),
    rejoin_success_candidate_start_time_(-1.0)
{
  ROS_INFO("[BacktrackUseCase] Initialized");
}

TickOutput BacktrackUseCase::tick(double now) {
  // 更新TriggerEvaluator的当前时间（用于RC丢失检测）
  trigger_evaluator_.updateNow(now);

  // 飞行周期复位检测
  handleFlightCycleReset(current_armed_);

  // 录制延时处理
  handleRecordDelay(now);

  // 触发检测与处理
  handleTriggers(now);

  // 轨迹生成处理
  handleTrajectoryGeneration();

  // 退化 waypoint 会话执行进度
  handleWaypointFallbackProgress();

  // 偏离检测与回归处理
  handleRejoin(now);

  // 构造TickInput
  domain::TickInput tick_input;
  tick_input.armed = current_armed_;
  tick_input.mode = current_mode_;
  tick_input.now = now;
  tick_input.current_position = current_position_;
  tick_input.current_yaw = current_yaw_;
  tick_input.trajectory_ready =
      trajectory_session_.isValid() || waypoint_fallback_session_.isValid();
  tick_input.trajectory_finished =
      trajectory_session_.isFinished() || waypoint_fallback_session_.isFinished();

  // 驱动状态机
  state_machine_.tick(tick_input);

  // 构造输出
  TickOutput output;
  output.should_publish_setpoint = state_machine_.shouldPublishSetpoint();
  output.mode_request = state_machine_.getModeRequest();
  output.current_state = state_machine_.getCurrentState();
  output.rc_loss = trigger_evaluator_.isRCLost();

  // 计算setpoint
  if (output.should_publish_setpoint) {
    output.setpoint = computeSetpoint(now);
  }

  return output;
}

void BacktrackUseCase::onOdomReceived(const domain::Vec3& position, double yaw, double stamp) {
  current_position_ = position;
  current_yaw_ = yaw;

  // 录制处理
  auto current_state = state_machine_.getCurrentState();
  if (current_state == domain::State::Recording) {
    recording_buffer_.addPoint(position, yaw, stamp);
  }
}

void BacktrackUseCase::onRCReceived(const std::vector<uint16_t>& channels) {
  // 更新RC通道数据
  trigger_evaluator_.updateRC(channels);

  // 更新最近一次收到RC消息的时间
  trigger_evaluator_.updateLastRcMsgTime(ros::Time::now().toSec());
}

void BacktrackUseCase::onStateReceived(bool armed, const std::string& mode) {
  current_armed_ = armed;
  current_mode_ = mode;
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
    trajectory_session_.reset();
    waypoint_fallback_session_.reset();
    record_delay_started_ = false;
    trajectory_generation_requested_ = false;
    rejoin_trigger_candidate_start_time_ = -1.0;
    rejoin_success_candidate_start_time_ = -1.0;

    // 进入WaitRecordDelay状态（通过状态机处理）
    // 状态机会在tick中检测ARM并自动转换
  }
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
  bool rc_loss_event = trigger_evaluator_.checkRCLossEvent();

  // 在Recording状态检测返航触发
  if (current_state == domain::State::Recording) {
    bool has_min_points = checkMinPoints();

    // 通道7长按检测
    if (trigger_evaluator_.checkCh7LongPress()) {
      handleReturnTrigger(domain::Event::Ch7LongPress,
                          has_min_points,
                          "Ch7 long press");
      return;
    }

    // 通道18故障确认检测
    if (trigger_evaluator_.checkCh18Failsafe()) {
      handleReturnTrigger(domain::Event::Ch18FailsafeConfirmed,
                          has_min_points,
                          "Ch18 failsafe confirmed");
      return;
    }

    // RC消息丢失检测（边沿触发，仅触发一次）
    if (rc_loss_event) {
      handleReturnTrigger(domain::Event::RCLossTriggered,
                          has_min_points,
                          "RC loss");
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
      state_machine_.triggerPause();
      if (trajectory_session_.isValid()) {
        trajectory_session_.freezeProgress();
      }
      return;
    }
  }

  // 在Paused状态时，检测RC丢失和Ch7长按（恢复）
  if (current_state == domain::State::Paused) {
    bool has_min_points = checkMinPoints();

    if (rc_loss_event) {
      if (has_min_points) {
        ROS_WARN("[BacktrackUseCase] RC loss detected while paused, resuming return");
        state_machine_.lockPosition(current_position_, current_yaw_);
        state_machine_.triggerReturn(domain::Event::RCLossTriggered);
        if (trajectory_session_.isValid()) {
          trajectory_session_.resumeProgress(now);
        } else if (waypoint_fallback_session_.isValid()) {
          waypoint_fallback_session_.restart();
        }
      } else {
        ROS_WARN("[BacktrackUseCase] RC loss detected while paused but only %zu points recorded (< %d), triggering AUTO.LAND",
                 recording_buffer_.size(),
                 config_.record_min_points);
        triggerImmediateLanding();
      }
      return;
    }

    if (trigger_evaluator_.checkCh7LongPress()) {
      if (has_min_points) {
        ROS_INFO("[BacktrackUseCase] Ch7 long press detected, resuming");
        state_machine_.lockPosition(current_position_, current_yaw_);
        state_machine_.triggerReturn(domain::Event::Ch7LongPress);
        if (trajectory_session_.isValid()) {
          trajectory_session_.resumeProgress(now);
        } else if (waypoint_fallback_session_.isValid()) {
          waypoint_fallback_session_.restart();
        }
      } else {
        ROS_WARN("[BacktrackUseCase] Ch7 long press detected while paused but only %zu points recorded (< %d), triggering AUTO.LAND",
                 recording_buffer_.size(),
                 config_.record_min_points);
        triggerImmediateLanding();
      }
      return;
    }
  }
}

void BacktrackUseCase::handleReturnTrigger(domain::Event event,
                                           bool has_min_points,
                                           const char* trigger_name) {
  if (has_min_points) {
    ROS_INFO("[BacktrackUseCase] %s detected", trigger_name);
    trajectory_session_.reset();
    waypoint_fallback_session_.reset();
    state_machine_.lockPosition(current_position_, current_yaw_);
    state_machine_.triggerReturn(event);
    trajectory_generation_requested_ = true;
    return;
  }

  ROS_WARN("[BacktrackUseCase] %s detected but only %zu points recorded (< %d), triggering AUTO.LAND",
           trigger_name,
           recording_buffer_.size(),
           config_.record_min_points);
  triggerImmediateLanding();
}

void BacktrackUseCase::handleTrajectoryGeneration() {
  // 在ReturnPreOffboard状态且轨迹生成被请求时，执行轨迹生成
  auto current_state = state_machine_.getCurrentState();

  if (current_state == domain::State::ReturnPreOffboard &&
      trajectory_generation_requested_ &&
      !trajectory_session_.isValid() &&
      !waypoint_fallback_session_.isValid()) {

    ROS_INFO("[BacktrackUseCase] Generating trajectory from %zu points",
             recording_buffer_.size());

    // 生成轨迹
    bool success = trajectory_session_.generate(
        recording_buffer_.getPoints(),
        config_.trajectory_config);

    if (success) {
      trajectory_generation_requested_ = false;
      waypoint_fallback_session_.reset();
      rejoin_trigger_candidate_start_time_ = -1.0;
      rejoin_success_candidate_start_time_ = -1.0;
      ROS_INFO("[BacktrackUseCase] Trajectory generation successful");
      performDebugDump();
    } else {
      if (trajectory_session_.getLastFailureKind() ==
          domain::TrajectorySession::FailureKind::TooCloseGeometry) {
        ROS_ERROR("[BacktrackUseCase] Trajectory generation failed due to too-close geometry, degrading to immediate landing");
        triggerImmediateLanding();
      } else if (trajectory_session_.getLastFailureKind() ==
                 domain::TrajectorySession::FailureKind::InvalidConfig) {
        trajectory_generation_requested_ = false;
        ROS_ERROR("[BacktrackUseCase] Trajectory generation failed due to invalid trajectory config");
      } else if (waypoint_fallback_session_.build(recording_buffer_.getPoints())) {
        trajectory_generation_requested_ = false;
        rejoin_trigger_candidate_start_time_ = -1.0;
        rejoin_success_candidate_start_time_ = -1.0;
        ROS_WARN("[BacktrackUseCase] Trajectory generation failed, switching to waypoint fallback return (%zu sparse waypoints)",
                 waypoint_fallback_session_.size());
      } else {
        ROS_ERROR("[BacktrackUseCase] Trajectory generation failed and waypoint fallback construction also failed");
        triggerImmediateLanding();
      }
    }
  }
}

void BacktrackUseCase::handleWaypointFallbackProgress() {
  auto current_state = state_machine_.getCurrentState();
  if (current_state != domain::State::ReturnActive ||
      !waypoint_fallback_session_.isValid()) {
    return;
  }

  waypoint_fallback_session_.updateProgress(current_position_);
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
    if (!trajectory_session_.isValid()) {
      return;
    }

    // 获取轨迹上的当前进度位置
    auto traj_state = trajectory_session_.evaluate(now);

    // 检查是否偏离
    if (rejoin_policy_.shouldTriggerRejoin(current_position_, traj_state.position)) {
      if (rejoin_trigger_candidate_start_time_ < 0.0) {
        rejoin_trigger_candidate_start_time_ = now;
      }

      double hold_duration = now - rejoin_trigger_candidate_start_time_;
      if (hold_duration >= config_.rejoin_config.trigger_hold_s) {
        ROS_WARN("[BacktrackUseCase] Deviation detected for %.2f s, entering rejoin mode",
                 hold_duration);
        trajectory_session_.freezeProgress();  // 冻结轨迹进度，防止退出条件目标持续漂移
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
    if (!trajectory_session_.isValid()) {
      return;
    }

    // 获取轨迹上的当前进度位置
    auto traj_state = trajectory_session_.evaluate(now);

    // 检查是否回归成功
    if (rejoin_policy_.shouldExitRejoin(current_position_, traj_state.position)) {
      if (rejoin_success_candidate_start_time_ < 0.0) {
        rejoin_success_candidate_start_time_ = now;
      }

      double hold_duration = now - rejoin_success_candidate_start_time_;
      if (hold_duration >= config_.rejoin_config.success_hold_s) {
        ROS_INFO("[BacktrackUseCase] Rejoin successful for %.2f s, returning to active mode",
                 hold_duration);
        trajectory_session_.resumeProgress(now);  // 恢复轨迹进度，从冻结点继续
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

domain::SetpointTarget BacktrackUseCase::computeSetpoint(double now) {
  auto current_state = state_machine_.getCurrentState();

  // 优先使用状态机提供的setpoint（固定点悬停）
  auto state_machine_setpoint = state_machine_.getSetpointTarget();
  if (state_machine_setpoint.has_value()) {
    return state_machine_setpoint.value();
  }

  // ReturnActive状态：从轨迹评估
  if (current_state == domain::State::ReturnActive) {
    if (waypoint_fallback_session_.isValid()) {
      return waypoint_fallback_session_.currentTarget();
    }

    if (trajectory_session_.isValid()) {
      auto traj_state = trajectory_session_.evaluate(now);

      domain::SetpointTarget target;
      target.position = traj_state.position;
      target.velocity = traj_state.velocity;
      target.yaw = traj_state.yaw;
      target.yaw_rate = traj_state.yaw_rate;
      target.acceleration = traj_state.acceleration;
      target.type_mask = traj_state.acceleration.has_value() ? 0x0000 : 0x0038;
      target.coordinate_frame = 1;  // FRAME_LOCAL_NED

      return target;
    }
  }

  // ReturnRejoin状态：从回归策略计算
  if (current_state == domain::State::ReturnRejoin) {
    if (trajectory_session_.isValid()) {
      double t_nearest = rejoin_policy_.findNearestTime(
          current_position_, trajectory_session_.getTrajectory());
      auto rejoin_state = rejoin_policy_.computeRejoinTarget(
          current_position_, trajectory_session_.getTrajectory(), t_nearest);

      domain::SetpointTarget target;
      target.position = rejoin_state.position;
      target.velocity = rejoin_state.velocity;
      target.yaw = rejoin_state.yaw;
      target.yaw_rate = rejoin_state.yaw_rate;
      target.acceleration = std::nullopt;
      target.type_mask = 0x0038;  // IGNORE_AFX/AFY/AFZ
      target.coordinate_frame = 1;  // FRAME_LOCAL_NED

      return target;
    }
  }

  // 默认：零setpoint
  return domain::SetpointTarget();
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
  trajectory_generation_requested_ = false;
  trajectory_session_.reset();
  waypoint_fallback_session_.reset();
  rejoin_trigger_candidate_start_time_ = -1.0;
  rejoin_success_candidate_start_time_ = -1.0;
  state_machine_.triggerLanding();
}

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
