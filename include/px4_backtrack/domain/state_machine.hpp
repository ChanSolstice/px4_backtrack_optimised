#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "px4_backtrack/domain/types.hpp"

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 状态枚举
// ============================================================================

/// 状态机状态枚举
enum class State {
  /// 未ARM或启动初期
  IdleDisarmed,

  /// 检测到DISARM之后的ARM，等待record_delay_s
  WaitRecordDelay,

  /// 正在录制航迹（<=10Hz）
  Recording,

  /// 通道18触发后的15s固定点悬停等待
  Ch18HoverWait,

  /// 返航触发后，预发送setpoint并切OFFBOARD
  ReturnPreOffboard,

  /// 沿轨迹执行（含偏离检测）
  ReturnActive,

  /// 偏离过远，慢速回归到轨迹
  ReturnRejoin,

  /// 通道7按下事件导致暂停
  Paused,

  /// 轨迹结束后固定点悬停landing_hover_s
  LandingHover,

  /// 执行OFFBOARD->POSCTL->AUTO.LAND过渡并清理
  LandingTransition,

  /// AUTO.LAND 已由飞控接管，停止发布控制量并等待 DISARM
  AutoLandYield
};

/// 状态转换为字符串（用于日志输出）
inline std::string stateToString(State state) {
  switch (state) {
    case State::IdleDisarmed: return "IdleDisarmed";
    case State::WaitRecordDelay: return "WaitRecordDelay";
    case State::Recording: return "Recording";
    case State::Ch18HoverWait: return "Ch18HoverWait";
    case State::ReturnPreOffboard: return "ReturnPreOffboard";
    case State::ReturnActive: return "ReturnActive";
    case State::ReturnRejoin: return "ReturnRejoin";
    case State::Paused: return "Paused";
    case State::LandingHover: return "LandingHover";
    case State::LandingTransition: return "LandingTransition";
    case State::AutoLandYield: return "AutoLandYield";
    default: return "Unknown";
  }
}

// ============================================================================
// 事件枚举
// ============================================================================

/// 触发事件枚举
enum class Event {
  /// 无事件
  None,

  /// 检测到DISARM
  DisarmDetected,

  /// 检测到ARM
  ArmDetected,

  /// 录制延时到期
  RecordDelayElapsed,

  /// 通道7长按（启动/恢复）
  Ch7LongPress,

  /// 通道7按下事件（暂停）
  Ch7PressEvent,

  /// 通道18故障确认
  Ch18FailsafeConfirmed,

  /// RC消息丢失
  RCLossTriggered,

  /// 低电量触发
  LowBatteryTriggered,

  /// 15s悬停等待结束
  HoverWaitElapsed,

  /// 进入OFFBOARD模式
  OffboardEntered,

  /// 轨迹生成完成
  TrajectoryReady,

  /// 轨迹执行完成
  TrajectoryFinished,

  /// 偏离轨迹
  DeviationDetected,

  /// 回归成功
  RejoinSuccess,

  /// 降落悬停结束
  LandingHoverElapsed,

  /// 降落过渡完成
  LandingTransitionDone,

  /// 检测到AUTO.LAND模式
  AutoLandDetected
};

// ============================================================================
// 模式切换请求
// ============================================================================

/// 飞控模式切换请求
enum class ModeRequest {
  /// 无请求
  None,

  /// 切换到OFFBOARD模式
  OFFBOARD,

  /// 切换到POSCTL模式
  POSCTL,

  /// 切换到AUTO.LAND模式
  AUTO_LAND
};

/// 模式请求转换为字符串
inline std::string modeRequestToString(ModeRequest req) {
  switch (req) {
    case ModeRequest::None: return "None";
    case ModeRequest::OFFBOARD: return "OFFBOARD";
    case ModeRequest::POSCTL: return "POSCTL";
    case ModeRequest::AUTO_LAND: return "AUTO.LAND";
    default: return "Unknown";
  }
}

// ============================================================================
// Setpoint目标数据结构
// ============================================================================

namespace position_target_mask {

/// 忽略加速度/力字段，保留位置、速度、yaw和yaw_rate控制
constexpr uint16_t kIgnoreAcceleration = 0x01C0;

}  // namespace position_target_mask

/// Setpoint目标（领域对象）
struct SetpointTarget {
  /// 位置目标 (ENU, m)
  Vec3 position;

  /// 速度目标 (ENU, m/s)
  Vec3 velocity;

  /// Yaw角目标 (rad)
  double yaw;

  /// Yaw角速度目标 (rad/s)
  double yaw_rate;

  /// 加速度目标（可选，ENU, m/s²）
  std::optional<Vec3> acceleration;

  /// type_mask字段（用于PositionTarget消息）
  uint16_t type_mask;

  /// coordinate_frame字段（用于PositionTarget消息）
  uint8_t coordinate_frame;

  /// 默认构造函数
  SetpointTarget()
    : position(Vec3::Zero()),
      velocity(Vec3::Zero()),
      yaw(0.0),
      yaw_rate(0.0),
      acceleration(std::nullopt),
      type_mask(0),
      coordinate_frame(1)  // FRAME_LOCAL_NED
  {}
};

// ============================================================================
// 状态数据
// ============================================================================

/// 状态机内部数据
struct StateData {
  /// 锁定的位置（用于固定点悬停）
  Vec3 locked_position;

  /// 锁定的yaw（用于固定点悬停）
  double locked_yaw;

  /// 预发送setpoint计数器
  int pre_offboard_count;

  /// 悬停等待开始时间
  double hover_wait_start_time;

  /// 降落悬停开始时间
  double landing_hover_start_time;

  /// 上次切换OFFBOARD尝试时间（用于节流）
  double last_offboard_attempt_time;

  /// 是否已在Ch18HoverWait期间进入OFFBOARD
  bool ch18_offboard_entered;

  /// Ch18HoverWait进入OFFBOARD的连续失败时间（用于5s告警）
  double ch18_offboard_fail_start_time;

  /// 暂停前所处的逻辑态（用于恢复）
  State paused_from_state;

  /// 上次降落模式切换尝试时间
  double last_landing_mode_attempt_time;

  /// 降落过渡目标阶段（0=请求POSCTL，1=请求AUTO.LAND）
  int landing_transition_stage;

  /// 默认构造函数
  StateData()
    : locked_position(Vec3::Zero()),
      locked_yaw(0.0),
      pre_offboard_count(0),
      hover_wait_start_time(0.0),
      landing_hover_start_time(0.0),
      last_offboard_attempt_time(0.0),
      ch18_offboard_entered(false),
      ch18_offboard_fail_start_time(0.0),
      paused_from_state(State::IdleDisarmed),
      last_landing_mode_attempt_time(0.0),
      landing_transition_stage(0)
  {}

  /// 重置状态数据
  void reset() {
    locked_position = Vec3::Zero();
    locked_yaw = 0.0;
    pre_offboard_count = 0;
    hover_wait_start_time = 0.0;
    landing_hover_start_time = 0.0;
    last_offboard_attempt_time = 0.0;
    ch18_offboard_entered = false;
    ch18_offboard_fail_start_time = 0.0;
    paused_from_state = State::IdleDisarmed;
    last_landing_mode_attempt_time = 0.0;
    landing_transition_stage = 0;
  }
};

// ============================================================================
// 配置参数
// ============================================================================

/// BacktrackStateMachine配置参数
struct StateMachineConfig {
  int pre_offboard_setpoint_count;    ///< OFFBOARD预发送setpoint次数
  double ch18_hover_wait_s;            ///< Ch18固定点悬停等待时间 (s)
  double landing_hover_s;              ///< 降落前固定点悬停时间 (s)
  double offboard_attempt_interval_s;  ///< 模式切换尝试间隔 (s)，用于节流

  /// 默认构造函数
  StateMachineConfig()
    : pre_offboard_setpoint_count(20),
      ch18_hover_wait_s(15.0),
      landing_hover_s(5.0),
      offboard_attempt_interval_s(1.0)
  {}
};

// ============================================================================
// Tick输入数据
// ============================================================================

/// 状态机tick输入数据
struct TickInput {
  bool armed;                ///< 是否ARM
  std::string mode;          ///< 当前飞控模式
  double now;                ///< 当前时间 (s)
  Vec3 current_position;     ///< 当前位置 (ENU, m)
  double current_yaw;        ///< 当前yaw (rad)
  bool trajectory_ready;     ///< 轨迹是否就绪
  bool trajectory_finished;  ///< 轨迹是否完成

  /// 默认构造函数
  TickInput()
    : armed(false),
      mode(""),
      now(0.0),
      current_position(Vec3::Zero()),
      current_yaw(0.0),
      trajectory_ready(false),
      trajectory_finished(false)
  {}
};

// ============================================================================
// BacktrackStateMachine类
// ============================================================================

/// 返航状态机
class BacktrackStateMachine {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit BacktrackStateMachine(const StateMachineConfig& config);

  /// 状态机tick（每个20Hz周期调用）
  /// @param input tick输入数据
  void tick(const TickInput& input);

  /// 获取当前状态
  /// @return 当前状态
  State getCurrentState() const;

  /// 获取setpoint目标
  /// @return setpoint目标（如果不应发布则返回nullopt）
  std::optional<SetpointTarget> getSetpointTarget() const;

  /// 获取模式切换请求
  /// @return 模式请求（如果无请求则返回nullopt）
  std::optional<ModeRequest> getModeRequest() const;

  /// 是否应发布setpoint
  /// @return 是否应发布
  bool shouldPublishSetpoint() const;

  /// 触发返航（外部调用，例如Ch7长按、Ch18确认、RC丢失）
  /// @param event 触发事件
  void triggerReturn(Event event);

  /// 触发暂停（外部调用，例如Ch7按下事件）
  void triggerPause();

  /// 锁定当前位置（用于固定点悬停）
  /// @param position 位置 (ENU, m)
  /// @param yaw Yaw角 (rad)
  void lockPosition(const Vec3& position, double yaw);

  /// 启动录制（外部调用，从WaitRecordDelay转换到Recording）
  void startRecording();

  /// 触发直接降落（外部调用，例如返航点数不足时）
  void triggerLanding();

  /// 进入Rejoin状态（外部调用，偏离检测触发）
  void enterRejoin();

  /// 退出Rejoin状态（外部调用，回归成功）
  void exitRejoin();

  /// 获取状态数据（只读）
  /// @return 状态数据const引用
  const StateData& getStateData() const;

private:
  /// 配置参数
  StateMachineConfig config_;

  /// 当前状态
  State current_state_;

  /// 状态数据
  StateData state_data_;

  /// 上次输入数据（用于检测变化）
  TickInput last_input_;

  /// 当前setpoint目标
  std::optional<SetpointTarget> current_setpoint_;

  /// 当前模式请求
  std::optional<ModeRequest> current_mode_request_;

  /// 是否应发布setpoint
  bool should_publish_setpoint_;

  /// 状态转移逻辑
  void handleStateTransition(const TickInput& input);

  /// 各状态的tick处理
  void tickIdleDisarmed(const TickInput& input);
  void tickWaitRecordDelay(const TickInput& input);
  void tickRecording(const TickInput& input);
  void tickCh18HoverWait(const TickInput& input);
  void tickReturnPreOffboard(const TickInput& input);
  void tickReturnActive(const TickInput& input);
  void tickReturnRejoin(const TickInput& input);
  void tickPaused(const TickInput& input);
  void tickLandingHover(const TickInput& input);
  void tickLandingTransition(const TickInput& input);

  /// 进入状态处理
  void enterState(State new_state, const TickInput& input);

  /// 退出状态处理
  void exitState(State old_state);

  /// 检测ARM/DISARM变化
  bool detectArmStateChange(const TickInput& input);

  /// 检测AUTO.LAND模式
  bool isAutoLandMode(const std::string& mode) const;

  /// 生成固定点setpoint
  SetpointTarget createHoldPositionSetpoint(const Vec3& pos, double yaw) const;

  /// 清除模式请求
  void clearModeRequest();
};

}  // namespace domain
}  // namespace px4_backtrack
