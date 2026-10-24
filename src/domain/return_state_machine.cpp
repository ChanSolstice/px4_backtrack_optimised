#include "px4_backtrack/domain/return_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 起始对准与重新接入时允许的 yaw 误差（重构方案第 5.6、7.3 节）
constexpr double kAlignYawToleranceRad = 15.0 * kPi / 180.0;

/// 时间比较容差，避免浮点误差让边界判定晚一拍
constexpr double kTimeEpsS = 1e-9;

const char* const kModeOffboard = "OFFBOARD";
const char* const kModePosctl = "POSCTL";
const char* const kModeAutoLand = "AUTO.LAND";

bool yawAligned(double yaw, double target) {
  const double error = std::atan2(std::sin(yaw - target), std::cos(yaw - target));
  return std::abs(error) <= kAlignYawToleranceRad;
}

std::string onOff(bool value) {
  return value ? "on" : "off";
}

void addEvent(StepOutput& out, EventType type, std::string name,
              std::string from, std::string to, std::string rule,
              std::string reason, std::string detail) {
  StepEvent event;
  event.type = type;
  event.name = std::move(name);
  event.from = std::move(from);
  event.to = std::move(to);
  event.rule = std::move(rule);
  event.reason = std::move(reason);
  event.detail = std::move(detail);
  out.events.push_back(std::move(event));
}

/// 条件边沿事件
void addEdge(StepOutput& out, bool before, bool now, const char* name,
             const std::string& detail = std::string()) {
  if (before != now) {
    addEvent(out, EventType::Condition, name, onOff(before), onOff(now),
             "", "", detail);
  }
}

/// 新出现的 CH7 指令（与上一拍比较编号）
bool isNewCommand(const std::optional<Ch7Command>& before,
                  const std::optional<Ch7Command>& now) {
  return now && (!before || before->press_id != now->press_id);
}

std::string commandDetail(const Ch7Command& command) {
  return "press_id=" + std::to_string(command.press_id) +
         " link_ok=" + (command.link_ok ? "true" : "false");
}

}  // namespace

// ============================================================================
// 名称
// ============================================================================

bool isMissionPhase(MissionPhase phase) {
  switch (phase) {
    case MissionPhase::Hover:
    case MissionPhase::Prepare:
    case MissionPhase::Follow:
    case MissionPhase::Arrival:
    case MissionPhase::Handover:
      return true;
    default:
      return false;
  }
}

const char* reportedStateName(MissionPhase phase, FollowMode follow) {
  switch (phase) {
    case MissionPhase::Disarmed: return "IdleDisarmed";
    case MissionPhase::RecordDelay: return "WaitRecordDelay";
    case MissionPhase::Recording: return "Recording";
    case MissionPhase::Hover: return "Ch18HoverWait";
    case MissionPhase::Prepare: return "ReturnPreOffboard";
    case MissionPhase::Follow:
      return follow == FollowMode::OnPath ? "ReturnActive" : "ReturnRejoin";
    case MissionPhase::Arrival: return "LandingHover";
    case MissionPhase::Handover: return "LandingTransition";
    case MissionPhase::Exited: return "Exited";
    case MissionPhase::FcuLanding: return "AutoLandYield";
  }
  return "Unknown";
}

const char* controlName(ControlState control) {
  switch (control) {
    case ControlState::None: return "none";
    case ControlState::Acquiring: return "acquiring";
    case ControlState::Owned: return "owned";
    case ControlState::Fallback: return "fallback";
    case ControlState::Reclaiming: return "reclaiming";
    case ControlState::Handover: return "handover";
    case ControlState::Yielded: return "yielded";
    case ControlState::FcuLanding: return "fcu_landing";
  }
  return "unknown";
}

const char* followName(FollowMode follow) {
  return follow == FollowMode::OnPath ? "on_path" : "rejoin";
}

const char* missionReasonName(MissionReason reason) {
  switch (reason) {
    case MissionReason::Pilot: return "PILOT_REQUEST";
    case MissionReason::LowBattery: return "LOW_BATTERY";
    case MissionReason::LinkLost: return "LINK_LOST";
    case MissionReason::None: break;
  }
  return "NONE";
}

const char* handoverReasonName(HandoverReason reason) {
  switch (reason) {
    case HandoverReason::Arrived: return "ARRIVED";
    case HandoverReason::MissionLanded: return "MISSION_LANDED";
    case HandoverReason::InsufficientPoints: return "INSUFFICIENT_POINTS";
    case HandoverReason::PlanTooClose: return "PLAN_TOO_CLOSE";
    case HandoverReason::FallbackFailed: return "FALLBACK_FAILED";
    case HandoverReason::Escalated: return "ESCALATED";
    case HandoverReason::None: break;
  }
  return "NONE";
}

const char* modeIntentName(ModeIntent intent) {
  switch (intent) {
    case ModeIntent::Offboard: return kModeOffboard;
    case ModeIntent::Posctl: return kModePosctl;
    case ModeIntent::AutoLand: return kModeAutoLand;
    case ModeIntent::None: break;
  }
  return "";
}

const char* alertName(Alert alert) {
  switch (alert) {
    case Alert::LinkLost: return "LINK_LOST";
    case Alert::LinkDegraded: return "LINK_DEGRADED";
    case Alert::Ch18Unavailable: return "CH18_UNAVAILABLE";
    case Alert::Ch18Unexpected: return "CH18_UNEXPECTED";
    case Alert::LowBattery: return "LOW_BATTERY";
    case Alert::BatteryInvalid: return "BATTERY_INVALID";
    case Alert::ReturnPending: return "RETURN_PENDING";
    case Alert::TriggerIgnoredLanded: return "TRIGGER_IGNORED_LANDED";
    case Alert::MissionLanded: return "MISSION_LANDED";
    case Alert::ExitRejectedLink: return "EXIT_REJECTED_LINK";
    case Alert::ReturnExited: return "RETURN_EXITED";
    case Alert::ReturnRequestIgnored: return "RETURN_REQUEST_IGNORED";
    case Alert::Reclaiming: return "RECLAIMING";
    case Alert::PathOffset: return "PATH_OFFSET";
    case Alert::TrailGap: return "TRAIL_GAP";
    case Alert::ReclaimTimeout: return "RECLAIM_TIMEOUT";
    case Alert::OffboardFallback: return "OFFBOARD_FALLBACK";
    case Alert::ModeRequestFailed: return "MODE_REQUEST_FAILED";
    case Alert::DebugLogFailed: return "DEBUG_LOG_FAILED";
    case Alert::PlanningSlow: return "PLANNING_SLOW";
    case Alert::PlanningTimeoutFallback: return "PLANNING_TIMEOUT_FALLBACK";
    case Alert::PrepareSlow: return "PREPARE_SLOW";
    case Alert::PrepareTimeout: return "PREPARE_TIMEOUT";
    case Alert::ProgressStall: return "PROGRESS_STALL";
    case Alert::OdomStale: return "ODOM_STALE";
    case Alert::HandoverDirectLand: return "HANDOVER_DIRECT_LAND";
    case Alert::AutolandOverridden: return "AUTOLAND_OVERRIDDEN";
    case Alert::EscalatedLand: return "ESCALATED_LAND";
    case Alert::FcuStateStale: return "FCU_STATE_STALE";
    case Alert::Count: break;
  }
  return "UNKNOWN";
}

const char* eventTypeName(EventType type) {
  switch (type) {
    case EventType::State: return "state";
    case EventType::Control: return "control";
    case EventType::Follow: return "follow";
    case EventType::Mission: return "mission";
    case EventType::Condition: return "condition";
    case EventType::Alert: return "alert";
    case EventType::Fcu: return "fcu";
    case EventType::ModeRequest: return "mode_request";
    case EventType::Planning: return "planning";
    case EventType::Stream: return "stream";
  }
  return "unknown";
}

// ============================================================================
// ReturnStateMachine
// ============================================================================

ReturnStateMachine::ReturnStateMachine(const ReturnStateMachineConfig& config)
  : config_(config)
{
}

StepOutput ReturnStateMachine::step(const Conditions& c) {
  StepOutput out;
  trigger_ignored_landed_ = false;
  path_offset_ = false;
  emitInputEvents(c, out);

  if (!c.armed) {
    // 规则 1：未解锁
    if (phase_ != MissionPhase::Disarmed) {
      if (mission_.active) {
        endMission("DISARMED", out);
      }
      out.commands.cancel_planning = true;
      setControl(ControlState::None, "1", "DISARMED", out);
      setPhase(MissionPhase::Disarmed, "1", "DISARMED", out);
    }
    pending_ = PendingStart{};
    consumeCommands(c);
  } else {
    if (phase_ == MissionPhase::Disarmed) {
      // 新的飞行周期：任务编号从 1 重新开始；press_id 不归零（R-57）
      ++flight_id_;
      mission_count_ = 0;
      mission_ = MissionInfo{};
      out.commands.new_flight = true;
      record_delay_start_t_ = c.now;
      setPhase(MissionPhase::RecordDelay, "1", "ARMED", out);
    }

    if (phase_ == MissionPhase::FcuLanding) {
      // 锁存到上锁（D18）
      consumeCommands(c);
    } else if (c.fcu_state_fresh && c.mode == kModeAutoLand) {
      // 规则 2：对 AUTO.LAND 始终让步
      if (mission_.active) {
        endMission("FCU_AUTO_LAND", out);
      }
      out.commands.cancel_planning = true;
      pending_ = PendingStart{};
      setControl(ControlState::FcuLanding, "2", "FCU_AUTO_LAND", out);
      setPhase(MissionPhase::FcuLanding, "2", "FCU_AUTO_LAND", out);
      consumeCommands(c);
    } else {
      switch (phase_) {
        case MissionPhase::RecordDelay:
          stepRecordDelay(c, out);
          break;
        case MissionPhase::Recording:
          stepStart(c, false, out);
          break;
        case MissionPhase::Hover:
        case MissionPhase::Prepare:
        case MissionPhase::Follow:
        case MissionPhase::Arrival:
          stepMission(c, out);
          break;
        case MissionPhase::Handover:
          stepHandover(c, out);
          break;
        case MissionPhase::Exited:
          stepExited(c, out);
          break;
        case MissionPhase::Disarmed:
        case MissionPhase::FcuLanding:
          break;
      }
    }
  }

  computeOutputs(c, out);
  computeAlerts(c, out);
  prev_ = c;
  return out;
}

// ============================================================================
// 各阶段的规则
// ============================================================================

void ReturnStateMachine::stepRecordDelay(const Conditions& c, StepOutput& out) {
  // 录制前忽略 CH7；低电量、链路丢失只告警（D4）
  consumeCommands(c);
  if (c.now - record_delay_start_t_ + kTimeEpsS >= config_.record_delay_s) {
    setPhase(MissionPhase::Recording, "7", "RECORD_DELAY_ELAPSED", out);
  }
}

void ReturnStateMachine::stepStart(const Conditions& c, bool from_exited,
                                   StepOutput& out) {
  bool pilot = false;
  if (c.pilot.return_request &&
      c.pilot.return_request->press_id > consumed_return_id_) {
    const Ch7Command& request = *c.pilot.return_request;
    consumed_return_id_ = request.press_id;
    if (from_exited) {
      // 退出后长按只告警（D1）
      ignored_return_press_id_ = request.press_id;
    } else if (request.link_ok) {
      pilot = true;
    }
  }
  if (c.pilot.exit_request) {
    consumed_exit_id_ = std::max(consumed_exit_id_, c.pilot.exit_request->press_id);
  }

  // 退出后只有链路丢失启动新任务（D1、D7）
  const bool link_lost = c.link.lost;
  const bool battery_low = !from_exited && c.battery.low;
  const bool pilot_request = pilot || (pending_.active && pending_.pilot);
  if (!link_lost && !battery_low && !pilot_request) {
    pending_ = PendingStart{};
    return;
  }

  // 落地保护（D23）：不在空中时不启动，也不计补点时间，恢复后重新判定
  if (!c.airborne.airborne) {
    trigger_ignored_landed_ = true;
    pending_ = PendingStart{};
    return;
  }

  if (!pending_.active) {
    pending_.active = true;
    pending_.start_t = c.now;
    pending_.reason = link_lost ? MissionReason::LinkLost
        : battery_low ? MissionReason::LowBattery
        : MissionReason::Pilot;
  }
  pending_.link_lost = pending_.link_lost || link_lost;
  pending_.battery_low = pending_.battery_low || battery_low;
  pending_.pilot = pilot_request;

  if (c.has_min_points) {
    startMission(c, true, out);
    pending_ = PendingStart{};
    return;
  }

  // 点数不足：最多等 return_pending_max_s 补点，仍不足则原地降落
  if (c.now - pending_.start_t + kTimeEpsS >= config_.return_pending_max_s) {
    startMission(c, false, out);
    pending_ = PendingStart{};
    enterHandover(c, HandoverReason::InsufficientPoints, false, "7", out);
  }
}

void ReturnStateMachine::stepMission(const Conditions& c, StepOutput& out) {
  trackMissionConditions(c, out);
  updateWaits(c);

  // 规则 3：任务中落地（D23）
  const bool landed = (phase_ == MissionPhase::Hover ||
                       phase_ == MissionPhase::Prepare)
      ? !c.airborne.airborne
      : c.airborne.landed_state == LandedState::OnGround;
  if (landed) {
    enterHandover(c, HandoverReason::MissionLanded, true, "3", out);
    return;
  }

  // 规则 4：任务中的模式变化，飞控状态中断时暂停（R-62）
  if (c.fcu_state_fresh && updateControl(c, out)) {
    return;
  }

  // 规则 5：短按退出
  if (handleExitRequest(c, out)) {
    return;
  }

  // 规则 7：当前阶段自身的规则（规则 6 在它之后判定）
  switch (phase_) {
    case MissionPhase::Hover:
      // 悬停照常计时，期间返回的规划结果在准备阶段处理（D3）
      if (c.now - hover_start_t_ + kTimeEpsS >= config_.ch18_hover_wait_s) {
        setPhase(MissionPhase::Prepare, "7", "HOVER_ELAPSED", out);
      }
      break;
    case MissionPhase::Prepare:
      stepPrepare(c, out);
      break;
    case MissionPhase::Follow:
      stepFollow(c, out);
      break;
    case MissionPhase::Arrival:
      stepArrival(c, out);
      break;
    default:
      break;
  }

  // 规则 6：超时升级（第 8.1 节）。按本阶段的规则仍不能继续返航、等待已超时且确认低电量，
  // 才原地降落；电量正常或读数无效时保持并告警（D11、D32、R-85）
  if (c.battery.low && waitTimedOut()) {
    enterHandover(c, HandoverReason::Escalated, true, "6", out);
  }
}

void ReturnStateMachine::stepPrepare(const Conditions& c, StepOutput& out) {
  const bool owned = control_ == ControlState::Owned && !c.odom_stale;
  switch (c.plan_status) {
    case PlanStatus::FailedTooClose:
    case PlanStatus::FailedFallback:
      // 轨迹在断点处截断时飞机不一定靠近起点：不降落（D30）；断点之前有离飞机不超过
      // 安全距离的点就接上，否则原地悬停（D33）
      if (!c.trail_gap) {
        enterHandover(c, c.plan_status == PlanStatus::FailedTooClose
                             ? HandoverReason::PlanTooClose
                             : HandoverReason::FallbackFailed,
                      false, "7", out);
      } else if (owned && c.anchor_past_gap) {
        reanchor(c, "7", "TRAIL_GAP", out);
      }
      return;
    case PlanStatus::Ready:
    case PlanStatus::ReadyFallback:
      break;
    default:
      // 规划中或配置非法：保持；配置非法由准备超时兜底（第 8.1 节）
      return;
  }

  if (!owned) {
    return;
  }
  // 开始跟随前检查偏差：离路径超过安全距离时能找点接上就接上，否则原地保持（D30、D33）
  if (!offsetOk(c)) {
    if (c.anchor_ahead) {
      reanchor(c, "7", "PATH_OFFSET", out);
    } else {
      path_offset_ = true;
    }
    return;
  }
  // 低电量且准备超时：不再等起始对准，能开始跟随就开始（D32、R-85）
  const bool aligned = !c.path_start_yaw || yawAligned(c.yaw, *c.path_start_yaw);
  if (!aligned && !(c.battery.low && waitTimedOut())) {
    return;
  }
  // 进入跟随：停滞重新计时
  stall_wait_s_ = 0.0;
  stall_mark_m_ = c.remaining_m;
  setPhase(MissionPhase::Follow, "7", aligned ? "PLAN_READY" : "PREPARE_TIMEOUT", out);
}

void ReturnStateMachine::stepFollow(const Conditions& c, StepOutput& out) {
  const bool owned = control_ == ControlState::Owned && !c.odom_stale;
  // 离路径超过安全距离（含投影失败）：不回归。能找点接上就从接上的点重新规划（D33），
  // 否则原地保持、冻结进度（D30）
  if (!offsetOk(c)) {
    if (owned && c.anchor_ahead) {
      reanchor(c, "7", "PATH_OFFSET", out);
    } else {
      path_offset_ = true;
    }
    return;
  }
  // 不持有控制权或里程计延迟时冻结进度（R-18）
  if (!owned) {
    return;
  }

  if (c.finished) {
    arrival_position_ = c.position;
    arrival_yaw_ = c.yaw;
    arrival_owned_s_ = 0.0;
    setPhase(MissionPhase::Arrival, "7", "FINISHED", out);
    return;
  }

  if (follow_ == FollowMode::OnPath && c.deviation_confirmed) {
    setFollow(FollowMode::Rejoin, "7", "DEVIATION", out);
  } else if (follow_ == FollowMode::Rejoin && c.rejoin_confirmed) {
    setFollow(FollowMode::OnPath, "7", "REJOINED", out);
  }
}

void ReturnStateMachine::stepArrival(const Conditions& c, StepOutput& out) {
  // 到点悬停只在持有控制权时计时（updateWaits）
  if (arrival_owned_s_ + kTimeEpsS < config_.landing_hover_s) {
    return;
  }
  // 到达的是轨迹断点而不是录制起点：不降落（D30）；断点之前有离飞机不超过安全距离的
  // 点就接上，否则原地悬停（D33）
  if (!c.trail_gap) {
    enterHandover(c, HandoverReason::Arrived, false, "7", out);
  } else if (control_ == ControlState::Owned && !c.odom_stale && c.anchor_past_gap) {
    reanchor(c, "7", "TRAIL_GAP", out);
  }
}

void ReturnStateMachine::stepHandover(const Conditions& c, StepOutput& out) {
  trackMissionConditions(c, out);

  // 规则 3：交接中落地，改为直接降落
  if (c.airborne.landed_state == LandedState::OnGround) {
    handover_direct_ = true;
    handover_reason_ = HandoverReason::MissionLanded;
  }
  // 链路丢失时不经过 POSCTL（第 7.6 节）
  if (c.link.lost) {
    handover_direct_ = true;
  }

  // 规则 4 的例外：只在链路正常时判定飞手接管；POSCTL 是我方请求的模式
  if (c.fcu_state_fresh && linkClean(c) && c.mode != kModeOffboard &&
      c.mode != kModePosctl && c.mode != kModeAutoLand) {
    exitMission(c, "4b", "EXTERNAL_MODE", out);
    return;
  }

  // 规则 5
  if (handleExitRequest(c, out)) {
    return;
  }

  // POSCTL 迟迟不生效，改为直接请求 AUTO.LAND
  if (!handover_direct_ && c.fcu_state_fresh && c.mode == kModeOffboard &&
      c.now - handover_start_t_ + kTimeEpsS >=
          config_.handover_posctl_timeout_s) {
    handover_direct_ = true;
  }
}

void ReturnStateMachine::stepExited(const Conditions& c, StepOutput& out) {
  // 退出后：低电量、长按只告警；链路丢失按与 Recording 相同的检查启动新任务
  stepStart(c, true, out);
}

// ============================================================================
// 规则 4、5
// ============================================================================

bool ReturnStateMachine::updateControl(const Conditions& c, StepOutput& out) {
  const bool in_offboard = c.mode == kModeOffboard;

  switch (control_) {
    case ControlState::Acquiring:
      if (in_offboard) {
        setControl(ControlState::Owned, "4", "OFFBOARD_OBSERVED", out);
        return false;
      }
      // 规则 4c：OFFBOARD 确认前，链路正常时飞手切到其他模式（D25）
      if (linkClean(c) && c.mode != mission_.start_mode) {
        exitMission(c, "4c", "EXTERNAL_MODE", out);
        return true;
      }
      return false;

    case ControlState::Owned:
      if (in_offboard) {
        return false;
      }
      // 规则 4a：本模块刚断流引起的回退（D22）
      if (c.stream_gap_recent) {
        fallback_start_t_ = c.now;
        setControl(ControlState::Fallback, "4a", "OFFBOARD_FALLBACK", out);
        return false;
      }
      return switchedOut(c, "4b", out);

    case ControlState::Fallback:
      if (in_offboard) {
        setControl(ControlState::Owned, "4a", "FALLBACK_RECOVERED", out);
        return false;
      }
      if (c.now - fallback_start_t_ + kTimeEpsS >= config_.fallback_timeout_s) {
        return switchedOut(c, "4b", out);
      }
      return false;

    case ControlState::Reclaiming:
      if (in_offboard) {
        return tryReclaim(c, out);
      }
      // 链路先恢复：飞手当时的模式选择生效（R-63）
      if (linkClean(c)) {
        exitMission(c, "4b", "EXTERNAL_MODE", out);
        return true;
      }
      return false;

    default:
      return false;
  }
}

bool ReturnStateMachine::switchedOut(const Conditions& c, const char* rule,
                                     StepOutput& out) {
  if (linkClean(c)) {
    exitMission(c, rule, "EXTERNAL_MODE", out);
    return true;
  }
  // 链路丢失或退化：重新申请控制权（D2）
  reclaim_position_ = c.position;
  reclaim_yaw_ = c.yaw;
  reclaim_wait_s_ = 0.0;
  setControl(ControlState::Reclaiming, rule, "LINK_ABNORMAL", out);
  return false;
}

bool ReturnStateMachine::tryReclaim(const Conditions& c, StepOutput& out) {
  if (phase_ != MissionPhase::Follow) {
    // 悬停、准备、到点阶段：保持点改为重新接入时的位置，不飞回原来的位置（D30）
    trigger_position_ = reclaim_position_;
    arrival_position_ = reclaim_position_;
    setControl(ControlState::Owned, "4", "RECLAIMED", out);
    return false;
  }

  // 跟随阶段：对准路径目标切向，且局部投影有效、偏差不超限，才重新接入（第 7.3 节）；
  // 偏差超限但能找点接上时，重新接入并从接上的点重新规划（D33）
  const bool offset_ok = offsetOk(c);
  if (!offset_ok && c.anchor_ahead && !c.odom_stale) {
    setControl(ControlState::Owned, "4", "RECLAIMED", out);
    reanchor(c, "4", "PATH_OFFSET", out);
    return true;
  }
  const bool yaw_ok = !c.path_target_yaw || yawAligned(c.yaw, *c.path_target_yaw);
  path_offset_ = !offset_ok;
  if (!yaw_ok || !offset_ok) {
    return false;
  }

  setControl(ControlState::Owned, "4", "RECLAIMED", out);
  if (follow_ == FollowMode::OnPath &&
      c.cross_track_m > config_.rejoin_trigger_dist_m) {
    setFollow(FollowMode::Rejoin, "4", "RECLAIM_DEVIATION", out);
  }
  return false;
}

bool ReturnStateMachine::handleExitRequest(const Conditions& c, StepOutput& out) {
  if (!c.pilot.exit_request ||
      c.pilot.exit_request->press_id <= consumed_exit_id_) {
    return false;
  }
  const Ch7Command& request = *c.pilot.exit_request;
  consumed_exit_id_ = request.press_id;

  // 按下早于任务开始：可能是想触发返航时按下的（S5）
  if (request.press_start_t <= mission_.start_t) {
    return false;
  }
  if (!request.link_ok) {
    exit_rejected_ = true;
    return false;
  }
  exitMission(c, "5", "PILOT_EXIT", out);
  return true;
}

// ============================================================================
// 任务
// ============================================================================

void ReturnStateMachine::startMission(const Conditions& c, bool enter_phase,
                                      StepOutput& out) {
  ++mission_count_;
  mission_ = MissionInfo{};
  mission_.active = true;
  mission_.id = mission_count_;
  mission_.reason = pending_.reason;
  mission_.start_t = c.now;
  mission_.start_mode = knownMode();
  mission_.start_link_lost = pending_.link_lost;
  mission_.start_battery_low = pending_.battery_low;

  trigger_position_ = c.position;
  trigger_yaw_ = c.yaw;
  pre_frames_ = 0;
  pre_frames_done_t_ = c.now;
  prepare_wait_s_ = 0.0;
  out.commands.start_planning = true;

  std::string conditions;
  if (pending_.link_lost) conditions += "link_lost ";
  if (pending_.battery_low) conditions += "battery_low ";
  if (pending_.pilot) conditions += "pilot ";
  if (!conditions.empty()) conditions.pop_back();
  addEvent(out, EventType::Mission, "started", "", "", "7",
           missionReasonName(mission_.reason),
           "id=" + std::to_string(mission_.id) + " conditions=" + conditions);

  if (!enter_phase) {
    return;
  }
  // 只要有链路丢失，就先悬停（D16）
  setControl(ControlState::Acquiring, "7", missionReasonName(mission_.reason), out);
  if (pending_.link_lost) {
    hover_start_t_ = c.now;
    setPhase(MissionPhase::Hover, "7", missionReasonName(mission_.reason), out);
  } else {
    setPhase(MissionPhase::Prepare, "7", missionReasonName(mission_.reason), out);
  }
}

void ReturnStateMachine::enterHandover(const Conditions& c, HandoverReason reason,
                                       bool direct, const char* rule,
                                       StepOutput& out) {
  // 到点后交接沿用到点位置，其他情况保持当前位置
  if (phase_ == MissionPhase::Arrival) {
    handover_position_ = arrival_position_;
    handover_yaw_ = arrival_yaw_;
  } else {
    handover_position_ = c.position;
    handover_yaw_ = c.yaw;
  }
  handover_reason_ = reason;
  handover_direct_ = direct || c.link.lost;
  handover_start_t_ = c.now;
  out.commands.cancel_planning = true;
  setControl(ControlState::Handover, rule, handoverReasonName(reason), out);
  setPhase(MissionPhase::Handover, rule, handoverReasonName(reason), out);
}

void ReturnStateMachine::exitMission(const Conditions& c, const char* rule,
                                     const char* reason, StepOutput& out) {
  // 截断到游标：游标只在跟随时推进，悬停、准备阶段退出等于不截断（R-65、R-78）
  out.commands.trail_cut = true;
  out.commands.cancel_planning = true;
  exit_position_ = c.position;
  exit_yaw_ = c.yaw;
  endMission(reason, out);
  setControl(ControlState::Yielded, rule, reason, out);
  setPhase(MissionPhase::Exited, rule, reason, out);
}

void ReturnStateMachine::reanchor(const Conditions& c, const char* rule,
                                  const char* reason, StepOutput& out) {
  // 执行层截断到接上的点、从该点重新规划；准备阶段原地保持，规划就绪后同样检查偏差、
  // 对准再跟随（D33）
  out.commands.reanchor = true;
  out.commands.start_planning = true;
  trigger_position_ = c.position;
  trigger_yaw_ = c.yaw;
  prepare_wait_s_ = 0.0;
  addEvent(out, EventType::Mission, "reanchored", "", "", rule, reason, "");
  setPhase(MissionPhase::Prepare, rule, "REANCHOR", out);
}

void ReturnStateMachine::updateWaits(const Conditions& c) {
  // 到点悬停和第 8.1 节的等待都只在里程计新鲜时计时，里程计中断不会引起降落（D31）。
  // 准备、到点和停滞只在申请或持有 OFFBOARD 时计时，所以重新申请期间都冻结（R-18）
  if (c.odom_stale) {
    return;
  }
  const double dt = std::max(0.0, c.now - prev_.now);
  if (control_ == ControlState::Reclaiming) {
    reclaim_wait_s_ += dt;
  }
  if (phase_ == MissionPhase::Prepare &&
      (control_ == ControlState::Acquiring || control_ == ControlState::Owned) &&
      c.plan_status != PlanStatus::None && c.plan_status != PlanStatus::Pending) {
    prepare_wait_s_ += dt;
  }
  if (phase_ == MissionPhase::Arrival && control_ == ControlState::Owned) {
    arrival_owned_s_ += dt;
  }
  if ((phase_ == MissionPhase::Follow || phase_ == MissionPhase::Arrival) &&
      control_ == ControlState::Owned) {
    if (c.remaining_m <= stall_mark_m_ - config_.stall_min_progress_m) {
      stall_mark_m_ = c.remaining_m;
      stall_wait_s_ = 0.0;
    } else {
      stall_wait_s_ += dt;
    }
  }
}

bool ReturnStateMachine::waitTimedOut() const {
  if (control_ == ControlState::Reclaiming) {
    return reclaim_wait_s_ + kTimeEpsS >= config_.reclaim_timeout_s;
  }
  switch (phase_) {
    case MissionPhase::Prepare:
      return prepare_wait_s_ + kTimeEpsS >= config_.prepare_timeout_s;
    case MissionPhase::Follow:
    case MissionPhase::Arrival:
      // 到点悬停也计停滞：停在断点处又找不到点时按停滞超时升级（R-85）
      return stall_wait_s_ + kTimeEpsS >= config_.stall_timeout_s;
    default:
      return false;
  }
}

void ReturnStateMachine::endMission(const char* reason, StepOutput& out) {
  mission_.active = false;
  exit_rejected_ = false;
  addEvent(out, EventType::Mission, "ended", "", "", "", reason,
           "id=" + std::to_string(mission_.id));
}

void ReturnStateMachine::trackMissionConditions(const Conditions& c,
                                                StepOutput& out) {
  // 任务中新出现的条件只记录和上报，不改变流程（D3）
  if (c.link.lost && !mission_.start_link_lost && !mission_.seen_link_lost_t) {
    mission_.seen_link_lost_t = c.now;
    addEvent(out, EventType::Mission, "condition_seen", "", "", "", "",
             "link_lost");
  }
  if (c.battery.low && !mission_.start_battery_low &&
      !mission_.seen_battery_low_t) {
    mission_.seen_battery_low_t = c.now;
    addEvent(out, EventType::Mission, "condition_seen", "", "", "", "",
             "battery_low");
  }
  if (c.pilot.return_request &&
      c.pilot.return_request->press_id > consumed_return_id_) {
    consumed_return_id_ = c.pilot.return_request->press_id;
    if (!mission_.seen_pilot_t) {
      mission_.seen_pilot_t = c.now;
      addEvent(out, EventType::Mission, "condition_seen", "", "", "", "",
               "pilot");
    }
  }
}

void ReturnStateMachine::consumeCommands(const Conditions& c) {
  if (c.pilot.return_request) {
    consumed_return_id_ =
        std::max(consumed_return_id_, c.pilot.return_request->press_id);
  }
  if (c.pilot.exit_request) {
    consumed_exit_id_ =
        std::max(consumed_exit_id_, c.pilot.exit_request->press_id);
  }
}

// ============================================================================
// 区域转换
// ============================================================================

void ReturnStateMachine::setPhase(MissionPhase phase, const char* rule,
                                  const char* reason, StepOutput& out) {
  if (phase == phase_) {
    return;
  }
  const std::string from = reportedStateName(phase_, follow_);
  if (phase_ == MissionPhase::Follow) {
    follow_ = FollowMode::OnPath;
  }
  phase_ = phase;
  addEvent(out, EventType::State, "", from, reportedStateName(phase_, follow_),
           rule, reason, "");
}

void ReturnStateMachine::setControl(ControlState control, const char* rule,
                                    const char* reason, StepOutput& out) {
  if (control == control_) {
    return;
  }
  addEvent(out, EventType::Control, "", controlName(control_),
           controlName(control), rule, reason, "");
  control_ = control;
}

void ReturnStateMachine::setFollow(FollowMode follow, const char* rule,
                                   const char* reason, StepOutput& out) {
  if (follow == follow_) {
    return;
  }
  addEvent(out, EventType::Follow, "", followName(follow_), followName(follow),
           rule, reason, "");
  follow_ = follow;
}

// ============================================================================
// 输出与上报
// ============================================================================

void ReturnStateMachine::computeOutputs(const Conditions& c, StepOutput& out) {
  out.phase = phase_;
  out.control = control_;
  out.follow = follow_;
  out.flight_id = flight_id_;
  out.mission_id = mission_.id;

  if (!c.odom_stale) {
    odom_hold_active_ = false;
  }

  // 飞控状态中断时沿用最后一次收到的模式（R-62）
  const bool in_offboard = knownMode() == kModeOffboard;

  switch (phase_) {
    case MissionPhase::Recording:
      out.record = true;
      break;

    case MissionPhase::Hover:
    case MissionPhase::Prepare:
    case MissionPhase::Follow:
    case MissionPhase::Arrival:
      computeMissionOutputs(c, out);
      break;

    case MissionPhase::Handover:
      // 飞控离开 OFFBOARD 之前保持；链路正常时经过 POSCTL，否则直接 AUTO.LAND
      if (in_offboard) {
        out.setpoint = SetpointKind::Hold;
        out.hold_position = handover_position_;
        out.hold_yaw = handover_yaw_;
      }
      out.mode_intent = (!handover_direct_ && in_offboard)
          ? ModeIntent::Posctl
          : ModeIntent::AutoLand;
      break;

    case MissionPhase::Exited:
      // 继续录制；飞控离开 OFFBOARD 之前保持当前位置并请求 POSCTL。
      // 链路丢失时不请求 POSCTL（U10），保持到新任务接管（R-67）
      out.record = true;
      if (in_offboard) {
        out.setpoint = SetpointKind::Hold;
        out.hold_position = exit_position_;
        out.hold_yaw = exit_yaw_;
        if (!c.link.lost) {
          out.mode_intent = ModeIntent::Posctl;
        }
      }
      break;

    case MissionPhase::FcuLanding:
      // 链路丢失期间 AUTO.LAND 被切走时重新请求（第 7.6 节）
      if (c.link.lost && knownMode() != kModeAutoLand) {
        out.mode_intent = ModeIntent::AutoLand;
      }
      break;

    case MissionPhase::Disarmed:
    case MissionPhase::RecordDelay:
      break;
  }
}

void ReturnStateMachine::computeMissionOutputs(const Conditions& c,
                                               StepOutput& out) {
  // 悬停和准备共用同一个输出：保持、预发、申请 OFFBOARD（D16）
  SetpointKind kind = SetpointKind::Hold;
  Vec3 position = trigger_position_;
  double yaw = trigger_yaw_;
  switch (phase_) {
    case MissionPhase::Prepare:
      // 规划就绪后 yaw 改为路径起始切向；找点接上的那一拍规划事实还是上一次的（D33）
      if (planReady(c) && c.path_start_yaw && !out.commands.reanchor) {
        yaw = *c.path_start_yaw;
      }
      break;
    case MissionPhase::Follow:
      if (path_offset_) {
        // 离路径超过安全距离：原地保持（D30）
        if (!offset_hold_active_) {
          offset_hold_active_ = true;
          offset_hold_position_ = c.position;
          offset_hold_yaw_ = c.yaw;
        }
        position = offset_hold_position_;
        yaw = offset_hold_yaw_;
      } else {
        offset_hold_active_ = false;
        kind = follow_ == FollowMode::OnPath ? SetpointKind::PathTarget
                                             : SetpointKind::PathTargetRejoin;
      }
      break;
    case MissionPhase::Arrival:
      position = arrival_position_;
      yaw = arrival_yaw_;
      break;
    default:
      break;
  }

  // 重新申请：拿回 OFFBOARD 之前保持点跟着飞机，拿回后固定，即原地保持（D30）；
  // 跟随阶段 yaw 对准路径目标切向
  if (control_ == ControlState::Reclaiming) {
    if (knownMode() != kModeOffboard) {
      reclaim_position_ = c.position;
      reclaim_yaw_ = c.yaw;
    }
    if (phase_ == MissionPhase::Follow) {
      yaw = c.path_target_yaw ? *c.path_target_yaw : reclaim_yaw_;
    }
    kind = SetpointKind::Hold;
    position = reclaim_position_;
  }

  // 里程计延迟：冻结并保持（第 6.4 节）
  if (c.odom_stale) {
    if (!odom_hold_active_) {
      odom_hold_active_ = true;
      odom_hold_position_ = c.position;
      odom_hold_yaw_ = kind == SetpointKind::Hold ? yaw : c.yaw;
    }
    kind = SetpointKind::Hold;
    position = odom_hold_position_;
    yaw = odom_hold_yaw_;
  }

  out.setpoint = kind;
  if (kind == SetpointKind::Hold) {
    out.hold_position = position;
    out.hold_yaw = yaw;
  }
  out.follow_active = phase_ == MissionPhase::Follow &&
                      control_ == ControlState::Owned && !c.odom_stale && !path_offset_;

  switch (control_) {
    case ControlState::Acquiring:
      // 先预发 pre_offboard_setpoint_count 帧，再申请 OFFBOARD（D27）。链路正常时还要
      // 等到预发结束后的下一条飞控状态，先看到飞手可能的切换（规则 4c，R-75）
      if (pre_frames_ < config_.pre_offboard_setpoint_count) {
        ++pre_frames_;
        pre_frames_done_t_ = c.now;
      } else if (!linkClean(c) || !c.fcu_state_fresh ||
                 c.fcu_state_t > pre_frames_done_t_) {
        out.mode_intent = ModeIntent::Offboard;
      }
      break;
    case ControlState::Owned:
    case ControlState::Reclaiming:
      out.mode_intent = ModeIntent::Offboard;
      break;
    default:
      // 回退：继续发布，不申请模式，等飞控自动回到 OFFBOARD（D22）
      break;
  }
}

void ReturnStateMachine::computeAlerts(const Conditions& c, StepOutput& out) {
  AlertSet alerts;

  if (c.link.lost) {
    alerts.set(Alert::LinkLost);
  } else if (c.link.degraded) {
    alerts.set(Alert::LinkDegraded);
  }
  if (c.link.ch18_unavailable) alerts.set(Alert::Ch18Unavailable);
  if (c.link.ch18_unexpected) alerts.set(Alert::Ch18Unexpected);
  if (c.battery.low) alerts.set(Alert::LowBattery);
  if (c.battery.invalid) alerts.set(Alert::BatteryInvalid);
  if (c.odom_stale) alerts.set(Alert::OdomStale);
  if (!c.fcu_state_fresh) alerts.set(Alert::FcuStateStale);
  if (c.mode_request_failed) alerts.set(Alert::ModeRequestFailed);
  if (c.debug_log_failed) alerts.set(Alert::DebugLogFailed);
  if (c.planning_slow) alerts.set(Alert::PlanningSlow);
  if (c.planning_timed_out) alerts.set(Alert::PlanningTimeoutFallback);

  if (pending_.active) alerts.set(Alert::ReturnPending);
  if (trigger_ignored_landed_) alerts.set(Alert::TriggerIgnoredLanded);
  if (phase_ == MissionPhase::Handover) {
    if (handover_reason_ == HandoverReason::MissionLanded) {
      alerts.set(Alert::MissionLanded);
    }
    if (handover_reason_ == HandoverReason::Escalated) alerts.set(Alert::EscalatedLand);
    if (handover_direct_) alerts.set(Alert::HandoverDirectLand);
  }

  // 等待计时（第 8.1 节）
  if (phase_ == MissionPhase::Prepare) {
    if (prepare_wait_s_ + kTimeEpsS >= config_.prepare_warn_s) alerts.set(Alert::PrepareSlow);
    if (prepare_wait_s_ + kTimeEpsS >= config_.prepare_timeout_s) alerts.set(Alert::PrepareTimeout);
  }
  if ((phase_ == MissionPhase::Follow || phase_ == MissionPhase::Arrival) &&
      stall_wait_s_ + kTimeEpsS >= config_.stall_warn_s) {
    alerts.set(Alert::ProgressStall);
  }
  if (control_ == ControlState::Reclaiming &&
      reclaim_wait_s_ + kTimeEpsS >= config_.reclaim_timeout_s) {
    alerts.set(Alert::ReclaimTimeout);
  }

  // 拒绝退出的告警持续到链路恢复正常或任务结束（R-66，任务结束时由 endMission 清除）
  if (exit_rejected_ && linkClean(c)) {
    exit_rejected_ = false;
  }
  if (exit_rejected_) alerts.set(Alert::ExitRejectedLink);

  if (phase_ == MissionPhase::Exited) alerts.set(Alert::ReturnExited);
  // 退出后长按：按住期间告警（R-64）
  if (ignored_return_press_id_ != 0 &&
      !(phase_ == MissionPhase::Exited && c.pilot.pressed &&
        c.pilot.press_id == ignored_return_press_id_)) {
    ignored_return_press_id_ = 0;
  }
  if (ignored_return_press_id_ != 0) alerts.set(Alert::ReturnRequestIgnored);

  if (control_ == ControlState::Reclaiming) alerts.set(Alert::Reclaiming);
  if (path_offset_) alerts.set(Alert::PathOffset);
  if (mission_.active && c.trail_gap) alerts.set(Alert::TrailGap);
  if (control_ == ControlState::Fallback) alerts.set(Alert::OffboardFallback);
  if (phase_ == MissionPhase::FcuLanding && c.link.lost &&
      knownMode() != kModeAutoLand) {
    alerts.set(Alert::AutolandOverridden);
  }

  // 告警出现和消失
  for (unsigned i = 0; i < static_cast<unsigned>(Alert::Count); ++i) {
    const Alert alert = static_cast<Alert>(i);
    if (alerts.has(alert) != prev_alerts_.has(alert)) {
      addEvent(out, EventType::Alert, alertName(alert),
               onOff(prev_alerts_.has(alert)), onOff(alerts.has(alert)),
               "", "", "");
    }
  }
  prev_alerts_ = alerts;
  out.alerts = alerts;
}

void ReturnStateMachine::emitInputEvents(const Conditions& c, StepOutput& out) {
  // 飞控模式或解锁状态变化（只看新鲜的状态）
  if (c.fcu_state_fresh) {
    if (!has_fcu_state_ || c.mode != last_mode_ || c.armed != last_armed_) {
      addEvent(out, EventType::Fcu, "", has_fcu_state_ ? last_mode_ : "",
               c.mode, "", "", c.armed ? "armed=true" : "armed=false");
    }
    has_fcu_state_ = true;
    last_mode_ = c.mode;
    last_armed_ = c.armed;
  }

  addEdge(out, prev_.link.lost, c.link.lost, "link_lost",
          c.link.source == LinkLossSource::Ch18 ? "source=ch18"
          : c.link.source == LinkLossSource::RcTimeout ? "source=rc_timeout"
          : "");
  addEdge(out, prev_.link.degraded, c.link.degraded, "link_degraded");
  addEdge(out, prev_.link.ch18_unexpected, c.link.ch18_unexpected,
          "ch18_unexpected", "value=" + std::to_string(c.link.ch18_value));
  addEdge(out, prev_.link.ch18_unavailable, c.link.ch18_unavailable,
          "ch18_unavailable");
  addEdge(out, prev_.battery.low, c.battery.low, "battery_low");
  addEdge(out, prev_.battery.invalid, c.battery.invalid, "battery_invalid");
  addEdge(out, prev_.airborne.airborne, c.airborne.airborne, "airborne");
  addEdge(out, prev_.odom_stale, c.odom_stale, "odom_stale");
  addEdge(out, !prev_.fcu_state_fresh, !c.fcu_state_fresh, "fcu_state_stale");

  if (isNewCommand(prev_.pilot.return_request, c.pilot.return_request)) {
    addEvent(out, EventType::Condition, "return_request", "", "", "", "",
             commandDetail(*c.pilot.return_request));
  }
  if (isNewCommand(prev_.pilot.exit_request, c.pilot.exit_request)) {
    addEvent(out, EventType::Condition, "exit_request", "", "", "", "",
             commandDetail(*c.pilot.exit_request));
  }
}

bool ReturnStateMachine::linkClean(const Conditions& c) const {
  return !c.link.lost && !c.link.degraded;
}

bool ReturnStateMachine::offsetOk(const Conditions& c) const {
  return c.projection_ok && c.cross_track_m <= config_.safe_distance_m;
}

bool ReturnStateMachine::planReady(const Conditions& c) const {
  return c.plan_status == PlanStatus::Ready ||
         c.plan_status == PlanStatus::ReadyFallback;
}

}  // namespace domain
}  // namespace px4_backtrack
