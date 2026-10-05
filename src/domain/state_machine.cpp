#include "px4_backtrack/domain/state_machine.hpp"
#include <ros/console.h>

namespace px4_backtrack {
namespace domain {

BacktrackStateMachine::BacktrackStateMachine(const StateMachineConfig& config)
  : config_(config),
    current_state_(State::IdleDisarmed),
    should_publish_setpoint_(false)
{
  ROS_INFO("[StateMachine] Initialized in state: %s", stateToString(current_state_).c_str());
}

void BacktrackStateMachine::tick(const TickInput& input) {
  // AUTO.LAND全局优先级处理
  if (isAutoLandMode(input.mode)) {
    // 检测到AUTO.LAND，立即清理返航相关输出
    current_setpoint_ = std::nullopt;
    current_mode_request_ = std::nullopt;
    should_publish_setpoint_ = false;

    if (!input.armed) {
      if (current_state_ != State::IdleDisarmed) {
        ROS_WARN("[StateMachine] AUTO.LAND disarmed, entering IdleDisarmed");
        enterState(State::IdleDisarmed, input);
      }
      last_input_ = input;
      return;
    }

    if (current_state_ != State::AutoLandYield) {
      ROS_WARN("[StateMachine] AUTO.LAND detected, yielding control");
      enterState(State::AutoLandYield, input);
    }

    last_input_ = input;
    return;
  }

  const bool owned=current_state_==State::ReturnActive || current_state_==State::ReturnRejoin ||
      current_state_==State::LandingHover ||
      ((current_state_==State::ReturnPreOffboard || current_state_==State::Ch18HoverWait) && last_input_.mode=="OFFBOARD");
  if (input.armed && owned && input.mode!="OFFBOARD") yieldControl();

  // 正常状态转移处理
  handleStateTransition(input);

  // 根据当前状态执行tick逻辑
  switch (current_state_) {
    case State::IdleDisarmed:
      tickIdleDisarmed(input);
      break;
    case State::WaitRecordDelay:
      tickWaitRecordDelay(input);
      break;
    case State::Recording:
      tickRecording(input);
      break;
    case State::Ch18HoverWait:
      tickCh18HoverWait(input);
      break;
    case State::ReturnPreOffboard:
      tickReturnPreOffboard(input);
      break;
    case State::ReturnActive:
      tickReturnActive(input);
      break;
    case State::ReturnRejoin:
      tickReturnRejoin(input);
      break;
    case State::Paused:
      tickPaused(input);
      break;
    case State::LandingHover:
      tickLandingHover(input);
      break;
    case State::LandingTransition:
      tickLandingTransition(input);
      break;
    case State::AutoLandYield:
    case State::ControlYield:
      break;
  }

  last_input_ = input;
}

State BacktrackStateMachine::getCurrentState() const {
  return current_state_;
}

std::optional<SetpointTarget> BacktrackStateMachine::getSetpointTarget() const {
  return current_setpoint_;
}

std::optional<ModeRequest> BacktrackStateMachine::getModeRequest() const {
  return current_mode_request_;
}

bool BacktrackStateMachine::shouldPublishSetpoint() const {
  return should_publish_setpoint_;
}

void BacktrackStateMachine::triggerReturn(Event event) {
  ROS_INFO("[StateMachine] Return triggered by event: %d", static_cast<int>(event));

  // 触发返航的前置状态检查在UseCase层完成（点数检查等）
  // 这里直接根据事件类型进入对应状态

  if (event == Event::Ch18FailsafeConfirmed) {
    // 进入Ch18HoverWait
    enterState(State::Ch18HoverWait, last_input_);
  } else {
    // Ch7长按或RC丢失，直接进入ReturnPreOffboard
    enterState(State::ReturnPreOffboard, last_input_);
  }
}

void BacktrackStateMachine::triggerPause() {
  ROS_INFO("[StateMachine] Pause triggered");

  // 保存暂停前的状态
  state_data_.paused_from_state = current_state_;

  // 进入Paused状态
  enterState(State::Paused, last_input_);
}

void BacktrackStateMachine::yieldControl() {
  enterState(State::ControlYield,last_input_);
  current_setpoint_.reset(); current_mode_request_.reset(); should_publish_setpoint_=false;
}

void BacktrackStateMachine::lockPosition(const Vec3& position, double yaw) {
  state_data_.locked_position = position;
  state_data_.locked_yaw = yaw;
}

void BacktrackStateMachine::startRecording() {
  if (current_state_ == State::WaitRecordDelay) {
    ROS_INFO("[StateMachine] Record delay completed, entering Recording");
    enterState(State::Recording, last_input_);
  } else {
    ROS_WARN("[StateMachine] startRecording() called in non-WaitRecordDelay state: %s",
             stateToString(current_state_).c_str());
  }
}

void BacktrackStateMachine::triggerLanding() {
  ROS_WARN("[StateMachine] Direct landing triggered");
  enterState(State::LandingTransition, last_input_);
}

void BacktrackStateMachine::enterRejoin() {
  if (current_state_ == State::ReturnActive) {
    ROS_INFO("[StateMachine] Entering Rejoin mode");
    enterState(State::ReturnRejoin, last_input_);
  } else {
    ROS_WARN("[StateMachine] enterRejoin() called in non-ReturnActive state: %s",
             stateToString(current_state_).c_str());
  }
}

void BacktrackStateMachine::exitRejoin() {
  if (current_state_ == State::ReturnRejoin) {
    ROS_INFO("[StateMachine] Exiting Rejoin mode, returning to active tracking");
    enterState(State::ReturnActive, last_input_);
  } else {
    ROS_WARN("[StateMachine] exitRejoin() called in non-ReturnRejoin state: %s",
             stateToString(current_state_).c_str());
  }
}

const StateData& BacktrackStateMachine::getStateData() const {
  return state_data_;
}

// ============================================================================
// 私有方法实现
// ============================================================================

void BacktrackStateMachine::handleStateTransition(const TickInput& input) {
  // 状态转移逻辑在各个tick方法中处理
  // 这里可以添加通用的转移逻辑（如DISARM检测）

  // DISARM检测（全局安全处理）
  if (!input.armed && last_input_.armed) {
    // 检测到DISARM，强制进入IdleDisarmed状态
    ROS_WARN("[StateMachine] DISARM detected, forcing to IdleDisarmed for safety");
    // 立即停止所有setpoint发布和模式请求
    current_setpoint_ = std::nullopt;
    current_mode_request_ = std::nullopt;
    should_publish_setpoint_ = false;
    // 转换到IdleDisarmed状态
    enterState(State::IdleDisarmed, input);
  }

  // ARM检测（仅在IdleDisarmed状态处理）
}

void BacktrackStateMachine::tickIdleDisarmed(const TickInput& input) {
  // IdleDisarmed状态：等待DISARM->ARM转换

  // 检测DISARM->ARM
  if (input.armed && !last_input_.armed) {
    // 检测到ARM，进入WaitRecordDelay
    ROS_INFO("[StateMachine] ARM detected, entering WaitRecordDelay");
    enterState(State::WaitRecordDelay, input);
  }
}

void BacktrackStateMachine::tickWaitRecordDelay(const TickInput& input) {
  // WaitRecordDelay状态：等待record_delay_s后进入Recording
  // 实际的延时判断在UseCase层完成，这里只响应外部信号

  // 暂不实现自动转移，由UseCase层控制
}

void BacktrackStateMachine::tickRecording(const TickInput& input) {
  // Recording状态：录制航迹
  // 触发返航由外部调用triggerReturn()完成

  // 清除setpoint和模式请求
  current_setpoint_ = std::nullopt;
  current_mode_request_ = std::nullopt;
  should_publish_setpoint_ = false;
}

void BacktrackStateMachine::tickCh18HoverWait(const TickInput& input) {
  // Ch18HoverWait状态：固定点悬停等待15s

  // 生成固定点setpoint
  current_setpoint_ = createHoldPositionSetpoint(
      state_data_.locked_position, state_data_.locked_yaw);
  should_publish_setpoint_ = true;

  // 预发送setpoint计数
  if (state_data_.pre_offboard_count < config_.pre_offboard_setpoint_count) {
    state_data_.pre_offboard_count++;
    current_mode_request_ = std::nullopt;  // 未达到计数，不切模式
  } else {
    // 达到计数，尝试切OFFBOARD（带节流）
    if (input.mode != "OFFBOARD") {
      double time_since_last_attempt = input.now - state_data_.last_offboard_attempt_time;
      if (time_since_last_attempt >= config_.offboard_attempt_interval_s) {
        current_mode_request_ = ModeRequest::OFFBOARD;
        state_data_.last_offboard_attempt_time = input.now;
      }

      // 5s告警检查
      if (!state_data_.ch18_offboard_entered) {
        double fail_duration = input.now - state_data_.ch18_offboard_fail_start_time;
        if (fail_duration >= 5.0) {
          ROS_ERROR("[StateMachine] Ch18HoverWait: Failed to enter OFFBOARD for 5s (current mode: %s)",
                    input.mode.c_str());
        }
      }
    } else {
      // 已进入OFFBOARD
      state_data_.ch18_offboard_entered = true;
      current_mode_request_ = std::nullopt;
    }
  }

  // 检查15s倒计时（不依赖OFFBOARD）
  double hover_duration = input.now - state_data_.hover_wait_start_time;
  if (hover_duration >= config_.ch18_hover_wait_s) {
    // 等待结束，进入ReturnPreOffboard
    ROS_INFO("[StateMachine] Ch18HoverWait timeout, entering ReturnPreOffboard");
    enterState(State::ReturnPreOffboard, input);
  }
}

void BacktrackStateMachine::tickReturnPreOffboard(const TickInput& input) {
  // ReturnPreOffboard状态：预发送setpoint并切OFFBOARD

  // 生成固定点setpoint
  current_setpoint_ = createHoldPositionSetpoint(
      state_data_.locked_position, state_data_.locked_yaw);
  should_publish_setpoint_ = true;

  // 预发送setpoint计数
  if (state_data_.pre_offboard_count < config_.pre_offboard_setpoint_count) {
    state_data_.pre_offboard_count++;
    current_mode_request_ = std::nullopt;  // 未达到计数，不切模式
  } else {
    // 达到计数，尝试切OFFBOARD（带节流）
    if (input.mode != "OFFBOARD") {
      double time_since_last_attempt = input.now - state_data_.last_offboard_attempt_time;
      if (time_since_last_attempt >= config_.offboard_attempt_interval_s) {
        current_mode_request_ = ModeRequest::OFFBOARD;
        state_data_.last_offboard_attempt_time = input.now;
      }
    } else {
      // 已进入OFFBOARD，检查轨迹是否就绪
      current_mode_request_ = std::nullopt;
      if (input.trajectory_ready) {
        // 进入ReturnActive
        ROS_INFO("[StateMachine] OFFBOARD entered and trajectory ready, entering ReturnActive");
        enterState(State::ReturnActive, input);
      }
    }
  }
}

void BacktrackStateMachine::tickReturnActive(const TickInput& input) {
  // ReturnActive状态：沿轨迹执行
  // setpoint由UseCase层从轨迹评估结果填充

  current_setpoint_ = std::nullopt;  // 清除固定点，让UseCase层从轨迹计算setpoint
  should_publish_setpoint_ = true;
  current_mode_request_ = std::nullopt;

  // 轨迹完成检测
  if (input.trajectory_finished) {
    ROS_INFO("[StateMachine] Trajectory finished, entering LandingHover");
    // 重新锁定当前位置用于降落前悬停
    state_data_.locked_position = input.current_position;
    state_data_.locked_yaw = input.current_yaw;
    enterState(State::LandingHover, input);
  }

  // 偏离检测在UseCase层完成，通过状态转移信号触发
}

void BacktrackStateMachine::tickReturnRejoin(const TickInput& input) {
  // ReturnRejoin状态：回归到轨迹
  // setpoint由UseCase层从回归策略计算

  current_setpoint_ = std::nullopt;  // 清除固定点，让UseCase层从回归策略计算setpoint
  should_publish_setpoint_ = true;
  current_mode_request_ = std::nullopt;

  if (input.trajectory_finished) {
    state_data_.locked_position=input.current_position;
    state_data_.locked_yaw=input.current_yaw;
    enterState(State::LandingHover,input);
  }
  // 回归成功检测在UseCase层完成，通过状态转移信号触发
}

void BacktrackStateMachine::tickPaused(const TickInput& input) {
  // Paused状态：暂停，不发布setpoint

  current_setpoint_ = std::nullopt;
  current_mode_request_ = input.mode=="OFFBOARD" ? std::optional<ModeRequest>(ModeRequest::POSCTL) : std::nullopt;
  should_publish_setpoint_ = false;

  // 恢复逻辑由外部调用triggerReturn()完成（Ch7长按）
}

void BacktrackStateMachine::tickLandingHover(const TickInput& input) {
  // LandingHover状态：固定点悬停landing_hover_s

  // 生成固定点setpoint
  current_setpoint_ = createHoldPositionSetpoint(
      state_data_.locked_position, state_data_.locked_yaw);
  should_publish_setpoint_ = true;
  current_mode_request_ = std::nullopt;

  // 检查悬停时间
  double hover_duration = input.now - state_data_.landing_hover_start_time;
  if (hover_duration >= config_.landing_hover_s) {
    // 悬停结束，进入LandingTransition
    ROS_INFO("[StateMachine] Landing hover completed, entering LandingTransition");
    enterState(State::LandingTransition, input);
  }
}

void BacktrackStateMachine::tickLandingTransition(const TickInput& input) {
  // LandingTransition状态：执行OFFBOARD->POSCTL->AUTO.LAND过渡
  current_mode_request_ = std::nullopt;

  const int desired_stage = input.mode == "OFFBOARD" ? 0 : 1;
  if (state_data_.landing_transition_stage != desired_stage) {
    state_data_.landing_transition_stage = desired_stage;
    state_data_.last_landing_mode_attempt_time =
        input.now - config_.offboard_attempt_interval_s;
  }

  // POSCTL得到飞控确认前继续发送安全悬停目标，避免OFFBOARD失去setpoint。
  if (desired_stage == 0) {
    current_setpoint_ = createHoldPositionSetpoint(
        state_data_.locked_position, state_data_.locked_yaw);
    should_publish_setpoint_ = true;
  } else {
    current_setpoint_ = std::nullopt;
    should_publish_setpoint_ = false;
  }

  const double elapsed = input.now - state_data_.last_landing_mode_attempt_time;
  if (elapsed < config_.offboard_attempt_interval_s) {
    return;
  }

  current_mode_request_ = desired_stage == 0
                              ? ModeRequest::POSCTL
                              : ModeRequest::AUTO_LAND;
  state_data_.last_landing_mode_attempt_time = input.now;
  ROS_INFO("[StateMachine] Landing transition: requesting %s",
           modeRequestToString(*current_mode_request_).c_str());
}

void BacktrackStateMachine::enterState(State new_state, const TickInput& input) {
  if (new_state == current_state_) {
    return;  // 已在该状态，不重复进入
  }

  State old_state = current_state_;
  exitState(old_state);

  current_state_ = new_state;

  ROS_INFO("[StateMachine] State transition: %s -> %s",
           stateToString(old_state).c_str(),
           stateToString(new_state).c_str());

  // 进入新状态的初始化
  switch (new_state) {
    case State::IdleDisarmed:
      state_data_.reset();
      break;

    case State::WaitRecordDelay:
      state_data_.reset();
      break;

    case State::Recording:
      // 保持状态数据（可能从暂停恢复）
      break;

    case State::Ch18HoverWait:
      // 锁定当前位置（在外部调用lockPosition）
      state_data_.pre_offboard_count = 0;
      state_data_.hover_wait_start_time = input.now;
      state_data_.last_offboard_attempt_time = 0.0;
      state_data_.ch18_offboard_entered = false;
      state_data_.ch18_offboard_fail_start_time = input.now;
      break;

    case State::ReturnPreOffboard:
      // 锁定当前位置（在外部调用lockPosition）
      state_data_.pre_offboard_count = 0;
      state_data_.last_offboard_attempt_time = 0.0;
      break;

    case State::ReturnActive:
      // 轨迹会话已在UseCase层初始化
      current_setpoint_ = std::nullopt;
      should_publish_setpoint_ = true;
      current_mode_request_ = std::nullopt;
      break;

    case State::ReturnRejoin:
      // 回归策略已在UseCase层初始化
      break;

    case State::Paused:
      // 暂停前的状态已在triggerPause中保存
      break;

    case State::LandingHover:
      state_data_.landing_hover_start_time = input.now;
      current_setpoint_ = createHoldPositionSetpoint(
          state_data_.locked_position, state_data_.locked_yaw);
      should_publish_setpoint_ = true;
      current_mode_request_ = std::nullopt;
      break;

    case State::LandingTransition:
      state_data_.landing_transition_stage = input.mode == "OFFBOARD" ? 0 : 1;
      state_data_.last_landing_mode_attempt_time =
          input.now - config_.offboard_attempt_interval_s;
      break;

    case State::AutoLandYield:
    case State::ControlYield:
      current_setpoint_ = std::nullopt;
      current_mode_request_ = std::nullopt;
      should_publish_setpoint_ = false;
      break;
  }
}

void BacktrackStateMachine::exitState(State old_state) {
  // 退出状态时的清理工作
  switch (old_state) {
    case State::Paused:
      // 清除模式请求（避免残留POSCTL请求）
      current_mode_request_ = std::nullopt;
      break;

    default:
      break;
  }
}

bool BacktrackStateMachine::detectArmStateChange(const TickInput& input) {
  return input.armed != last_input_.armed;
}

bool BacktrackStateMachine::isAutoLandMode(const std::string& mode) const {
  return mode == "AUTO.LAND";
}

SetpointTarget BacktrackStateMachine::createHoldPositionSetpoint(
    const Vec3& pos, double yaw) const {

  SetpointTarget target;
  target.position = pos;
  target.velocity = Vec3::Zero();
  target.yaw = yaw;
  target.yaw_rate = 0.0;
  target.acceleration = std::nullopt;

  target.type_mask = position_target_mask::kIgnoreAcceleration;

  // coordinate_frame: FRAME_LOCAL_NED
  target.coordinate_frame = 1;

  return target;
}

void BacktrackStateMachine::clearModeRequest() {
  current_mode_request_ = std::nullopt;
}

}  // namespace domain
}  // namespace px4_backtrack
