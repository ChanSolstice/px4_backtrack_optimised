#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "px4_backtrack/domain/return_conditions.hpp"
#include "px4_backtrack/domain/types.hpp"

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// ReturnStateMachine配置参数
struct ReturnStateMachineConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  double record_delay_s{};             ///< 解锁后开始录制的延时 (s)
  double ch18_hover_wait_s{};          ///< 失控触发后的悬停时间 (s)
  double landing_hover_s{};            ///< 到点悬停时间 (s)
  int pre_offboard_setpoint_count{};   ///< 申请 OFFBOARD 前预发的 setpoint 帧数
  double return_pending_max_s{};       ///< 点数不足时最长等待 (s)
  double fallback_timeout_s{};         ///< 飞控回退后等待自动回到 OFFBOARD 的上限 (s)
  double handover_posctl_timeout_s{};  ///< 交接中 POSCTL 未生效，改为直接请求 AUTO.LAND (s)
  double rejoin_trigger_dist_m{};      ///< 重新接入后横向偏差超过此值进入回归 (m)
  double reclaim_max_offset_m{};       ///< 重新接入允许的最大横向偏差 (m)
};

// ============================================================================
// 三个并行区域（重构方案第 6.1 节）
// ============================================================================

/// 区域 A：任务阶段
enum class MissionPhase {
  Disarmed,     ///< 未解锁（IdleDisarmed）
  RecordDelay,  ///< 解锁后的录制延时（WaitRecordDelay）
  Recording,    ///< 录制（Recording）
  Hover,        ///< 任务·悬停（Ch18HoverWait）
  Prepare,      ///< 任务·准备（ReturnPreOffboard）
  Follow,       ///< 任务·跟随（ReturnActive / ReturnRejoin）
  Arrival,      ///< 任务·到点（LandingHover）
  Handover,     ///< 任务·交接（LandingTransition）
  Exited,       ///< 已退出（Exited）
  FcuLanding    ///< 飞控降落（AutoLandYield）
};

/// 区域 B：控制权（上报字段 control）
enum class ControlState {
  None,        ///< 未参与控制
  Acquiring,   ///< 正在申请 OFFBOARD，尚未确认
  Owned,       ///< 本任务内已确认 OFFBOARD，且当前就是 OFFBOARD
  Fallback,    ///< 本模块刚断流，飞控回退；继续发布，等飞控自动回到 OFFBOARD
  Reclaiming,  ///< 链路丢失或退化时被切出，正在重新申请
  Handover,    ///< 降落交接中，正把控制交给 AUTO.LAND
  Yielded,     ///< 已退出，控制交还飞手
  FcuLanding   ///< 飞控降落已接管
};

/// 区域 C：跟随方式（只在跟随阶段有效）
enum class FollowMode {
  OnPath,  ///< 在路径上（ReturnActive）
  Rejoin   ///< 回归路径（ReturnRejoin）
};

/// 返航任务的原因：首个成立的条件
enum class MissionReason {
  None,
  Pilot,       ///< CH7 长按
  LowBattery,  ///< 低电量
  LinkLost     ///< 链路丢失
};

/// 进入降落交接的原因（SC9：只在这些情形下请求 AUTO.LAND）
enum class HandoverReason {
  None,
  Arrived,             ///< 到点悬停结束（返航完成）
  MissionLanded,       ///< 任务中判定落地（规则 3）
  InsufficientPoints,  ///< 补点等待超时，点数仍不足（起点附近）
  PlanTooClose,        ///< 规划判定点过近（起点附近）
  FallbackFailed       ///< 航点降级也构建失败（起点附近）
};

/// 是否处于返航任务（悬停、准备、跟随、到点、交接）
bool isMissionPhase(MissionPhase phase);

/// 上报给前端的状态字符串（由区域 A、C 推导，与现有字符串一致）
const char* reportedStateName(MissionPhase phase, FollowMode follow);
const char* controlName(ControlState control);
const char* followName(FollowMode follow);
const char* missionReasonName(MissionReason reason);
const char* handoverReasonName(HandoverReason reason);

// ============================================================================
// 输出
// ============================================================================

/// setpoint 策略：具体目标由执行层按策略计算
enum class SetpointKind {
  None,             ///< 不发布
  Hold,             ///< 保持 hold_position / hold_yaw
  PathTarget,       ///< 轨迹目标；拿不到目标时执行层保持当前位置
  PathTargetRejoin  ///< 限速后的轨迹目标
};

/// 模式意图：何时真正发送由 ModeCommander 决定
enum class ModeIntent {
  None,
  Offboard,
  Posctl,
  AutoLand
};

/// 模式意图对应的 MAVROS 模式字符串；None 为空串
const char* modeIntentName(ModeIntent intent);

/// 退出时对轨迹的截断方式（重构方案第 7.4 节，R5 实现）
enum class TrailCut {
  None,      ///< 不截断（悬停、准备阶段退出）
  AtCursor,  ///< 截断到游标（跟随阶段退出）
  ToStart    ///< 截断到起点（到点、交接阶段退出）
};

/// 本拍要求执行层执行的命令（一次性）
struct ExecCommands {
  bool new_flight{false};        ///< 新的飞行周期：清空录制和会话，复位电量与按键
  bool start_planning{false};    ///< 任务开始：基于当前录制缓存提交规划
  bool cancel_planning{false};   ///< 任务结束或进入交接：取消规划，丢弃会话
  TrailCut trail_cut{TrailCut::None};  ///< 退出时截断轨迹
};

/// 告警码（重构方案第 9.1 节）
enum class Alert : uint8_t {
  LinkLost,
  LinkDegraded,
  Ch18Unavailable,
  Ch18Unexpected,
  LowBattery,
  BatteryInvalid,
  ReturnPending,
  TriggerIgnoredLanded,
  MissionLanded,
  ExitRejectedLink,
  ReturnExited,
  ReturnRequestIgnored,
  Reclaiming,
  ReclaimOffset,
  ReclaimTimeout,
  OffboardFallback,
  ModeRequestFailed,
  DebugLogFailed,
  PlanningSlow,
  PlanningTimeoutFallback,
  PrepareSlow,
  PrepareTimeout,
  ProgressStall,
  OdomStale,
  OdomLost,
  HandoverDirectLand,
  AutolandNotAccepted,
  AutolandOverridden,
  EscalatedLand,
  FcuStateStale,
  Count
};

/// 告警码字符串，如 "LINK_LOST"
const char* alertName(Alert alert);

/// 告警集合（电平语义）
class AlertSet {
public:
  void set(Alert alert) { bits_ |= bit(alert); }
  bool has(Alert alert) const { return (bits_ & bit(alert)) != 0; }
  bool empty() const { return bits_ == 0; }
  uint64_t bits() const { return bits_; }
  bool operator==(const AlertSet& other) const { return bits_ == other.bits_; }
  bool operator!=(const AlertSet& other) const { return bits_ != other.bits_; }

private:
  static uint64_t bit(Alert alert) {
    return uint64_t{1} << static_cast<unsigned>(alert);
  }

  uint64_t bits_{0};
};

/// 事件类型（重构方案第 9.2 节；config、mode_request、planning、stream 由外层补充）
enum class EventType {
  State,      ///< 区域 A 转换（from/to 为上报字符串）
  Control,    ///< 区域 B 转换
  Follow,     ///< 区域 C 转换
  Mission,    ///< 任务开始、任务中新出现的条件、任务结束
  Condition,  ///< 条件边沿、CH7 指令
  Alert,      ///< 告警出现或消失
  Fcu         ///< 飞控模式或解锁状态变化
};

const char* eventTypeName(EventType type);

/// 本拍发生的一个事件
struct StepEvent {
  EventType type{EventType::State};
  std::string name;    ///< 条件、告警或任务事件名
  std::string from;    ///< 转换前
  std::string to;      ///< 转换后
  std::string rule;    ///< 触发规则（第 6.3 节编号）
  std::string reason;  ///< 原因
  std::string detail;  ///< 补充信息
};

/// 每拍输出（重构方案第 6.5 节）
struct StepOutput {
  // 上报
  MissionPhase phase{MissionPhase::Disarmed};
  ControlState control{ControlState::None};
  FollowMode follow{FollowMode::OnPath};
  int flight_id{0};     ///< 飞行周期编号，从 1 开始
  int mission_id{0};    ///< 本次飞行内的任务编号；没有任务时为 0
  bool mission_active{false};  ///< 任务进行中（悬停到交接）

  // setpoint 与模式
  SetpointKind setpoint{SetpointKind::None};
  Vec3 hold_position = Vec3::Zero();  ///< setpoint 为 Hold 时有效
  double hold_yaw{0.0};               ///< setpoint 为 Hold 时有效
  bool follow_active{false};          ///< 推进跟随进度（跟随阶段、持有控制权、里程计新鲜）
  ModeIntent mode_intent{ModeIntent::None};
  bool record{false};                 ///< 录制（Recording、Exited）

  ExecCommands commands;
  AlertSet alerts;
  std::vector<StepEvent> events;
};

/// 返航任务数据（重构方案第 7.1 节）
struct MissionInfo {
  bool active{false};         ///< 任务进行中；以 AutoLandYield、Exited 或上锁结束
  int id{0};                  ///< 本次飞行内递增，从 1 开始
  MissionReason reason{MissionReason::None};
  double start_t{0.0};        ///< 开始时间 (s)
  std::string start_mode;     ///< 开始时的飞控模式（规则 4c）
  bool start_link_lost{false};    ///< 开始时成立的条件
  bool start_battery_low{false};
  bool start_pilot{false};
  std::optional<double> seen_link_lost_t;    ///< 任务中首次出现的条件及时间
  std::optional<double> seen_battery_low_t;
  std::optional<double> seen_pilot_t;
  bool offboard_confirmed{false};  ///< 本任务内已观察到 OFFBOARD
};

// ============================================================================
// ReturnStateMachine类
// ============================================================================

/// 返航状态机：三个并行区域，所有转移都在 step 中完成
///
/// 纯逻辑：不访问 ROS、不读时钟，输入只有 Conditions，可以复制快照做穷举检查。
class ReturnStateMachine {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit ReturnStateMachine(const ReturnStateMachineConfig& config);

  /// 每个控制周期调用一次
  /// @param c 本拍条件快照
  /// @return 本拍输出与事件
  StepOutput step(const Conditions& c);

  MissionPhase phase() const { return phase_; }
  ControlState control() const { return control_; }
  FollowMode follow() const { return follow_; }
  const MissionInfo& mission() const { return mission_; }
  /// 最近一次交接的原因和是否直接降落，只在 Handover 阶段有意义
  HandoverReason handoverReason() const { return handover_reason_; }
  bool handoverDirect() const { return handover_direct_; }
  int flightId() const { return flight_id_; }

private:
  /// 待启动的任务（触发后等待在空中、补点）
  struct PendingStart {
    bool active{false};
    double start_t{0.0};
    MissionReason reason{MissionReason::None};
    bool link_lost{false};
    bool battery_low{false};
    bool pilot{false};
  };

  // 各阶段的规则
  void stepRecordDelay(const Conditions& c, StepOutput& out);
  void stepStart(const Conditions& c, bool from_exited, StepOutput& out);
  void stepMission(const Conditions& c, StepOutput& out);
  void stepPrepare(const Conditions& c, StepOutput& out);
  void stepFollow(const Conditions& c, StepOutput& out);
  void stepArrival(const Conditions& c, StepOutput& out);
  void stepHandover(const Conditions& c, StepOutput& out);
  void stepExited(const Conditions& c, StepOutput& out);

  /// 规则 4：任务中的模式变化（飞控状态新鲜时）；返回区域 A 是否已改变
  bool updateControl(const Conditions& c, StepOutput& out);
  /// 被切出且不属于回退：链路正常退出，否则重新申请
  bool switchedOut(const Conditions& c, const char* rule, StepOutput& out);
  /// 重新申请期间观察到 OFFBOARD：检查重新接入条件
  void tryReclaim(const Conditions& c, StepOutput& out);
  /// 规则 5：短按退出；返回是否已退出
  bool handleExitRequest(const Conditions& c, StepOutput& out);

  /// 开始任务；enter_phase 为假时由调用方直接进入交接（补点超时）
  void startMission(const Conditions& c, bool enter_phase, StepOutput& out);
  void enterHandover(const Conditions& c, HandoverReason reason, bool direct,
                     const char* rule, StepOutput& out);
  void exitMission(const Conditions& c, const char* rule, const char* reason,
                   StepOutput& out);
  void endMission(const char* reason, StepOutput& out);
  void trackMissionConditions(const Conditions& c, StepOutput& out);
  void consumeCommands(const Conditions& c);

  // 区域转换（同时记录事件）
  void setPhase(MissionPhase phase, const char* rule, const char* reason,
                StepOutput& out);
  void setControl(ControlState control, const char* rule, const char* reason,
                  StepOutput& out);
  void setFollow(FollowMode follow, const char* rule, const char* reason,
                 StepOutput& out);

  // 输出与上报
  void computeOutputs(const Conditions& c, StepOutput& out);
  void computeMissionOutputs(const Conditions& c, StepOutput& out);
  void computeAlerts(const Conditions& c, StepOutput& out);
  void emitInputEvents(const Conditions& c, StepOutput& out);

  /// 最近一次收到的飞控模式（飞控状态中断时沿用）
  const std::string& knownMode() const { return last_mode_; }
  bool linkClean(const Conditions& c) const;
  bool planReady(const Conditions& c) const;

  /// 配置参数
  ReturnStateMachineConfig config_;

  // 三个区域
  MissionPhase phase_{MissionPhase::Disarmed};
  ControlState control_{ControlState::None};
  FollowMode follow_{FollowMode::OnPath};

  // 飞行与任务
  int flight_id_{0};
  int mission_count_{0};          ///< 本次飞行已开始的任务数
  MissionInfo mission_;
  PendingStart pending_;
  HandoverReason handover_reason_{HandoverReason::None};
  bool handover_direct_{false};   ///< 交接直接请求 AUTO.LAND（不经过 POSCTL）

  // 计时
  double record_delay_start_t_{0.0};
  double hover_start_t_{0.0};
  double arrival_owned_s_{0.0};   ///< 到点悬停中持有控制权的累计时间
  double arrival_last_t_{0.0};
  double fallback_start_t_{0.0};
  double handover_start_t_{0.0};

  // 保持点
  Vec3 trigger_position_ = Vec3::Zero();   ///< 任务开始时（悬停、准备）
  double trigger_yaw_{0.0};
  Vec3 arrival_position_ = Vec3::Zero();   ///< 到点
  double arrival_yaw_{0.0};
  Vec3 reclaim_position_ = Vec3::Zero();   ///< 进入重新申请时
  double reclaim_yaw_{0.0};
  Vec3 handover_position_ = Vec3::Zero();  ///< 进入交接时
  double handover_yaw_{0.0};
  Vec3 exit_position_ = Vec3::Zero();      ///< 退出时
  double exit_yaw_{0.0};
  bool odom_hold_active_{false};           ///< 里程计延迟时的保持点
  Vec3 odom_hold_position_ = Vec3::Zero();
  double odom_hold_yaw_{0.0};

  // 任务中
  int pre_frames_{0};                ///< 已预发的 setpoint 帧数
  bool reclaim_offset_{false};       ///< 重新接入时偏差超限或投影失败
  bool exit_rejected_{false};        ///< 链路异常时收到短按
  bool trigger_ignored_landed_{false};  ///< 本拍有触发但不在空中
  int ignored_return_press_id_{0};   ///< 退出后被忽略的长按

  // CH7 指令消费（按 press_id，两类分别记录）
  int consumed_return_id_{0};
  int consumed_exit_id_{0};

  // 上一拍（边沿检测）
  Conditions prev_;
  std::string last_mode_;
  bool last_armed_{false};
  bool has_fcu_state_{false};
  AlertSet prev_alerts_;
};

}  // namespace domain
}  // namespace px4_backtrack
