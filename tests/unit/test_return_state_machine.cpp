#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "px4_backtrack/domain/return_state_machine.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kDt = 0.05;  // 控制周期 20Hz

ReturnStateMachineConfig makeConfig() {
  ReturnStateMachineConfig config{};
  config.record_delay_s = 15.0;
  config.ch18_hover_wait_s = 15.0;
  config.landing_hover_s = 5.0;
  config.pre_offboard_setpoint_count = 6;
  config.return_pending_max_s = 3.0;
  config.fallback_timeout_s = 3.0;
  config.handover_posctl_timeout_s = 3.0;
  config.rejoin_trigger_dist_m = 0.5;
  config.safe_distance_m = 0.7;
  config.prepare_warn_s = 10.0;
  config.prepare_timeout_s = 30.0;
  config.reclaim_timeout_s = 10.0;
  config.stall_min_progress_m = 0.2;
  config.stall_warn_s = 20.0;
  config.stall_timeout_s = 60.0;
  return config;
}

/// 测试驱动：维护一份"世界"条件，按 20Hz 推进状态机
struct Harness {
  Harness() : sm(makeConfig()) {
    c.fcu_state_fresh = true;
    c.armed = false;
    c.mode = "POSCTL";
    c.airborne.airborne = true;
    c.airborne.landed_state = LandedState::InAir;
    c.odom_stale = false;
    c.position = Vec3(10.0, 20.0, 3.0);
    c.yaw = 0.3;
    c.has_min_points = true;
    c.projection_ok = true;
  }

  /// 下一拍的时间
  double now() const { return k * kDt; }

  const StepOutput& step() {
    c.now = now();
    if (c.fcu_state_fresh && state_every_tick) {
      c.fcu_state_t = c.now;  // 飞控状态新鲜时每拍都收到
    }
    ++k;
    out = sm.step(c);
    history.insert(history.end(), out.events.begin(), out.events.end());
    return out;
  }

  void run(double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < n; ++i) {
      step();
    }
  }

  /// 推进直到条件成立；返回用掉的拍数，超过上限返回 -1
  int runUntil(const std::function<bool(const StepOutput&)>& done,
               double max_s = 60.0) {
    const int n = static_cast<int>(std::lround(max_s / kDt));
    for (int i = 1; i <= n; ++i) {
      if (done(step())) {
        return i;
      }
    }
    return -1;
  }

  // ---- 链路 ----
  void linkOk() { c.link = LinkStatus{}; }
  void linkDegraded() {
    c.link = LinkStatus{};
    c.link.degraded = true;
  }
  void linkLost() {
    c.link = LinkStatus{};
    c.link.lost = true;
    c.link.degraded = true;
    c.link.source = LinkLossSource::Ch18;
  }

  // ---- CH7 指令（与 PilotInput 一样保留到下一条）----
  void longPress(bool link_ok = true) {
    ++press_id;
    c.pilot.return_request = Ch7Command{press_id, now() - 2.0, now(), link_ok};
    c.pilot.press_id = press_id;
  }
  void shortPress(double press_start_t, bool link_ok = true) {
    ++press_id;
    c.pilot.exit_request = Ch7Command{press_id, press_start_t, now(), link_ok};
    c.pilot.press_id = press_id;
  }
  /// 任务开始后按下的短按（状态机只检查按下开始时间，时长由 PilotInput 保证）
  void shortPressNow(bool link_ok = true) { shortPress(now() - kDt, link_ok); }

  // ---- 常用阶段 ----
  void toRecording() {
    c.armed = true;
    run(15.0 + kDt);
    ASSERT_EQ(sm.phase(), MissionPhase::Recording);
  }

  void startLowBatteryMission() {
    toRecording();
    c.battery.low = true;
    step();
    ASSERT_EQ(sm.phase(), MissionPhase::Prepare);
  }

  void startLinkLossMission() {
    toRecording();
    linkLost();
    step();
    ASSERT_EQ(sm.phase(), MissionPhase::Hover);
  }

  /// 长按 CH7 启动任务，电量正常
  void startPilotMission() {
    toRecording();
    longPress();
    step();
    ASSERT_EQ(sm.phase(), MissionPhase::Prepare);
  }

  /// 等到申请 OFFBOARD，然后飞控进入 OFFBOARD
  void acquire() {
    ASSERT_GT(runUntil([](const StepOutput& o) {
      return o.mode_intent == ModeIntent::Offboard;
    }, 5.0), 0);
    c.mode = "OFFBOARD";
    step();
    ASSERT_EQ(sm.control(), ControlState::Owned);
  }

  /// 任务进入跟随；默认是低电量任务
  void toFollow(bool low_battery = true) {
    if (low_battery) {
      startLowBatteryMission();
    } else {
      startPilotMission();
    }
    acquire();
    c.plan_status = PlanStatus::Ready;
    c.path_start_yaw = c.yaw;
    c.path_target_yaw = c.yaw;
    step();
    ASSERT_EQ(sm.phase(), MissionPhase::Follow);
  }

  void toArrival() {
    toFollow();
    c.position = Vec3(1.0, 2.0, 3.0);
    c.finished = true;
    step();
    ASSERT_EQ(sm.phase(), MissionPhase::Arrival);
  }

  void toHandover() {
    toArrival();
    run(5.0);
    ASSERT_EQ(sm.phase(), MissionPhase::Handover);
  }

  bool sawEvent(EventType type, const std::string& to,
                const std::string& rule = std::string(),
                const std::string& reason = std::string()) const {
    return std::any_of(history.begin(), history.end(), [&](const StepEvent& e) {
      return e.type == type && e.to == to && (rule.empty() || e.rule == rule) &&
             (reason.empty() || e.reason == reason);
    });
  }

  bool sawNamed(EventType type, const std::string& name,
                const std::string& detail = std::string()) const {
    return std::any_of(history.begin(), history.end(), [&](const StepEvent& e) {
      return e.type == type && e.name == name &&
             (detail.empty() || e.detail.find(detail) != std::string::npos);
    });
  }

  ReturnStateMachine sm;
  Conditions c;
  StepOutput out;
  std::vector<StepEvent> history;
  int k{0};
  int press_id{0};
  bool state_every_tick{true};  ///< 为假时不再收到新的飞控状态（接收时间不变）
};

// ============================================================================
// 飞行周期
// ============================================================================

TEST(ReturnStateMachineTest, ArmStartsFlightAndRecordsAfterDelay) {
  Harness h;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Disarmed);

  h.c.armed = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::RecordDelay);
  EXPECT_TRUE(h.out.commands.new_flight);
  EXPECT_EQ(h.out.flight_id, 1);
  EXPECT_TRUE(h.sawEvent(EventType::State, "WaitRecordDelay", "1", "ARMED"));

  h.run(15.0 - kDt);
  EXPECT_EQ(h.out.phase, MissionPhase::RecordDelay);
  EXPECT_FALSE(h.out.record);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  EXPECT_TRUE(h.out.record);
  // 任务开始前不发布、不请求模式（SC5）
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
}

TEST(ReturnStateMachineTest, DisarmEndsMissionAndNewFlightRestartsMissionIds) {
  Harness h;
  h.toFollow();
  EXPECT_EQ(h.out.mission_id, 1);

  h.c.armed = false;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Disarmed);
  EXPECT_EQ(h.out.control, ControlState::None);
  EXPECT_TRUE(h.out.commands.cancel_planning);
  EXPECT_FALSE(h.sm.mission().active);
  EXPECT_TRUE(h.sawNamed(EventType::Mission, "ended"));

  h.c.battery.low = false;
  h.c.mode = "POSCTL";
  h.c.plan_status = PlanStatus::None;
  h.c.finished = false;
  h.toRecording();
  EXPECT_EQ(h.out.flight_id, 2);
  h.c.battery.low = true;
  h.step();
  EXPECT_EQ(h.out.mission_id, 1);
}

// ============================================================================
// 启动任务（策略矩阵 Recording 列）
// ============================================================================

TEST(ReturnStateMachineTest, LinkLossInRecordingHoversThenPrepares) {
  Harness h;
  h.toRecording();
  h.c.position = Vec3(5.0, 6.0, 7.0);
  h.linkLost();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  EXPECT_EQ(h.out.control, ControlState::Acquiring);
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LinkLost);
  EXPECT_TRUE(h.out.commands.start_planning);
  EXPECT_FALSE(h.out.record);
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(5.0, 6.0, 7.0)));

  // 预发 6 帧（含本拍）后申请 OFFBOARD（D27）
  for (int i = 0; i < 5; ++i) {
    h.step();
    EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
  }
  h.step();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Offboard);

  h.c.mode = "OFFBOARD";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);

  // 悬停 15 s（从任务开始算）后进入准备
  h.run(15.0 - 8 * kDt);
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
}

TEST(ReturnStateMachineTest, LowBatteryAndPilotRequestGoStraightToPrepare) {
  Harness h;
  h.startLowBatteryMission();
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LowBattery);
  EXPECT_EQ(h.out.control, ControlState::Acquiring);

  Harness p;
  p.toRecording();
  p.longPress();
  p.step();
  EXPECT_EQ(p.out.phase, MissionPhase::Prepare);
  EXPECT_EQ(p.sm.mission().reason, MissionReason::Pilot);
}

TEST(ReturnStateMachineTest, SimultaneousTriggersTakeLinkLossPath) {
  // S10：CH18 与低电量同一周期确认
  Harness h;
  h.toRecording();
  h.linkLost();
  h.c.battery.low = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LinkLost);
  EXPECT_TRUE(h.sm.mission().start_battery_low);
}

TEST(ReturnStateMachineTest, TriggersBeforeRecordingOnlyAlert) {
  // D4：录制前的低电量、失控只告警，长按忽略
  Harness h;
  h.c.armed = true;
  h.step();
  h.c.battery.low = true;
  h.linkLost();
  h.longPress();
  h.run(5.0);
  EXPECT_EQ(h.out.phase, MissionPhase::RecordDelay);
  EXPECT_TRUE(h.out.alerts.has(Alert::LowBattery));
  EXPECT_TRUE(h.out.alerts.has(Alert::LinkLost));
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);

  // 长按已被消费：进入 Recording 后只按电平条件启动
  h.c.battery.low = false;
  h.linkOk();
  h.run(10.0 + kDt);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
}

TEST(ReturnStateMachineTest, LowBatteryDuringDelayReturnsOncePointsSuffice) {
  // S9：录制延时内低电量，进入 Recording 补足点数后返航
  Harness h;
  h.c.armed = true;
  h.c.has_min_points = false;
  h.step();
  h.c.battery.low = true;
  h.run(15.0);
  ASSERT_EQ(h.out.phase, MissionPhase::Recording);
  h.step();
  EXPECT_TRUE(h.out.alerts.has(Alert::ReturnPending));
  h.run(2.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  h.c.has_min_points = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LowBattery);
  EXPECT_FALSE(h.out.alerts.has(Alert::ReturnPending));
}

TEST(ReturnStateMachineTest, InsufficientPointsLandsAfterPendingTimeout) {
  Harness h;
  h.toRecording();
  h.c.has_min_points = false;
  h.c.battery.low = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  EXPECT_TRUE(h.out.alerts.has(Alert::ReturnPending));
  h.run(3.0 - kDt);  // 等待 2.95 s
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  h.step();          // 等待 3.0 s
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_EQ(h.sm.handoverReason(), HandoverReason::InsufficientPoints);
  EXPECT_TRUE(h.sm.mission().active);
  // 不在 OFFBOARD：直接请求 AUTO.LAND，不发布
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_TRUE(h.sawEvent(EventType::State, "LandingTransition", "7",
                         "INSUFFICIENT_POINTS"));
}

TEST(ReturnStateMachineTest, PendingIsCancelledWhenTriggersClear) {
  Harness h;
  h.toRecording();
  h.c.has_min_points = false;
  h.linkLost();
  h.step();
  EXPECT_TRUE(h.out.alerts.has(Alert::ReturnPending));
  h.linkOk();
  h.step();
  EXPECT_FALSE(h.out.alerts.has(Alert::ReturnPending));
  h.run(5.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
}

TEST(ReturnStateMachineTest, NotAirborneBlocksStartUntilAirborne) {
  // S13：不在空中时不启动任务，恢复后重新判定
  Harness h;
  h.toRecording();
  h.c.airborne.airborne = false;
  h.linkLost();
  h.run(5.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
  EXPECT_TRUE(h.out.alerts.has(Alert::TriggerIgnoredLanded));
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_FALSE(h.out.alerts.has(Alert::ReturnPending));

  h.c.airborne.airborne = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  EXPECT_FALSE(h.out.alerts.has(Alert::TriggerIgnoredLanded));
}

TEST(ReturnStateMachineTest, LongPressOnGroundIsDropped) {
  Harness h;
  h.toRecording();
  h.c.airborne.airborne = false;
  h.longPress();
  h.step();
  EXPECT_TRUE(h.out.alerts.has(Alert::TriggerIgnoredLanded));
  h.c.airborne.airborne = true;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
}

// ============================================================================
// 任务中（策略矩阵"返航任务中"列）
// ============================================================================

TEST(ReturnStateMachineTest, ConditionsDuringMissionAreRecordedOnly) {
  // S1、G1：低电量返航中失控、再长按，都只记录
  Harness h;
  h.toFollow();
  h.linkLost();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_TRUE(h.out.alerts.has(Alert::LinkLost));
  EXPECT_TRUE(h.sm.mission().seen_link_lost_t.has_value());
  EXPECT_TRUE(h.sawNamed(EventType::Mission, "condition_seen", "link_lost"));

  h.longPress();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_TRUE(h.sm.mission().seen_pilot_t.has_value());
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LowBattery);
}

TEST(ReturnStateMachineTest, ShortPressExitsMission) {
  // S6：返航中链路正常时短按退出
  Harness h;
  h.toFollow();
  h.c.position = Vec3(-3.0, 4.0, 5.0);
  h.shortPressNow();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_EQ(h.out.control, ControlState::Yielded);
  EXPECT_FALSE(h.sm.mission().active);
  EXPECT_TRUE(h.out.record);
  EXPECT_TRUE(h.out.commands.cancel_planning);
  EXPECT_TRUE(h.out.commands.trail_cut);
  EXPECT_TRUE(h.out.alerts.has(Alert::ReturnExited));
  EXPECT_TRUE(h.sawEvent(EventType::State, "Exited", "5", "PILOT_EXIT"));
  // 飞控离开 OFFBOARD 之前保持当前位置并请求 POSCTL
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(-3.0, 4.0, 5.0)));
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Posctl);

  h.c.mode = "POSCTL";
  h.step();
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
}

TEST(ReturnStateMachineTest, ShortPressRejectedWhileLinkNotClean) {
  Harness h;
  h.toFollow();
  h.linkDegraded();
  h.shortPressNow(false);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_TRUE(h.out.alerts.has(Alert::ExitRejectedLink));
  h.run(1.0);
  EXPECT_TRUE(h.out.alerts.has(Alert::ExitRejectedLink));

  // R-66：链路恢复正常后告警消失；再次短按才退出
  h.linkOk();
  h.step();
  EXPECT_FALSE(h.out.alerts.has(Alert::ExitRejectedLink));
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  h.shortPressNow();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
}

TEST(ReturnStateMachineTest, PressStartedBeforeMissionDoesNotExit) {
  // S5：飞手按住 CH7 时低电量先触发，这次按下不算退出
  Harness h;
  h.toRecording();
  const double press_t = h.now();
  h.run(0.5);
  h.c.battery.low = true;
  h.step();
  ASSERT_EQ(h.out.phase, MissionPhase::Prepare);
  h.run(0.5);
  h.shortPress(press_t);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);

  h.run(1.0);
  h.shortPressNow();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.commands.trail_cut);
}

TEST(ReturnStateMachineTest, LongPressDuringMissionDoesNotExit) {
  // S8
  Harness h;
  h.toFollow();
  h.longPress();
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_FALSE(h.out.commands.cancel_planning);
}

TEST(ReturnStateMachineTest, ModeSwitchWithCleanLinkExits) {
  // S7：链路正常时飞手拨模式开关
  Harness h;
  h.toFollow();
  h.c.mode = "ALTCTL";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.sawEvent(EventType::State, "Exited", "4b", "EXTERNAL_MODE"));
  // 已不在 OFFBOARD：不发布，不请求
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
}

TEST(ReturnStateMachineTest, TakeoverBeforeOffboardConfirmedExits) {
  // S27（H14）：OFFBOARD 确认前，链路正常时飞手拨到 ALTCTL
  Harness h;
  h.startLowBatteryMission();
  h.run(0.2);
  h.c.mode = "ALTCTL";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.sawEvent(EventType::State, "Exited", "4c", "EXTERNAL_MODE"));
  h.run(2.0);
  EXPECT_NE(h.out.mode_intent, ModeIntent::Offboard);

  // 链路退化时不判为接管，继续申请 OFFBOARD（P4）
  Harness d;
  d.startLowBatteryMission();
  d.linkDegraded();
  d.c.mode = "ALTCTL";
  d.run(1.0);
  EXPECT_EQ(d.out.phase, MissionPhase::Prepare);
  EXPECT_EQ(d.out.control, ControlState::Acquiring);
  EXPECT_EQ(d.out.mode_intent, ModeIntent::Offboard);
}

TEST(ReturnStateMachineTest, OffboardRequestWaitsForFcuStateAfterPreFrames) {
  // R-75：链路正常时，预发结束后须等到下一条飞控状态，才申请 OFFBOARD
  Harness h;
  h.startLowBatteryMission();
  h.state_every_tick = false;
  h.run(0.1);
  h.c.fcu_state_t = h.now() - kDt;  // 预发期间收到的状态不算
  h.run(1.0);
  EXPECT_EQ(h.out.control, ControlState::Acquiring);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  h.state_every_tick = true;
  h.step();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Offboard);

  // 飞控状态中断时不等待（规则 4 暂停，不判接管）
  Harness stale;
  stale.startLowBatteryMission();
  stale.state_every_tick = false;
  stale.c.fcu_state_fresh = false;
  stale.run(0.5);
  EXPECT_EQ(stale.out.mode_intent, ModeIntent::Offboard);

  // 链路丢失时不判接管，也不等待：失控接管时延不变（D27）
  Harness lost;
  lost.startLinkLossMission();
  lost.state_every_tick = false;
  lost.run(0.5);
  EXPECT_EQ(lost.out.mode_intent, ModeIntent::Offboard);
}

TEST(ReturnStateMachineTest, OwnStreamGapIsFallbackNotTakeover) {
  // S26（H13）：本模块卡顿，飞控回退 POSCTL，链路正常
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.control, ControlState::Fallback);
  EXPECT_TRUE(h.out.alerts.has(Alert::OffboardFallback));
  // 继续发布当前状态的 setpoint，进度冻结，不申请模式
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTarget);
  EXPECT_FALSE(h.out.follow_active);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);

  h.run(1.0);
  h.c.mode = "OFFBOARD";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_TRUE(h.out.follow_active);
  EXPECT_TRUE(h.sawEvent(EventType::Control, "owned", "4a", "FALLBACK_RECOVERED"));
}

TEST(ReturnStateMachineTest, FallbackTimeoutExitsOrReclaims) {
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.step();
  h.c.stream_gap_recent = false;
  h.run(3.0 - kDt);
  EXPECT_EQ(h.out.control, ControlState::Fallback);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);

  Harness l;
  l.toFollow();
  l.c.mode = "POSCTL";
  l.c.stream_gap_recent = true;
  l.step();
  l.linkLost();
  l.run(3.0);
  EXPECT_EQ(l.out.phase, MissionPhase::Follow);
  EXPECT_EQ(l.out.control, ControlState::Reclaiming);
  EXPECT_EQ(l.out.mode_intent, ModeIntent::Offboard);
}

TEST(ReturnStateMachineTest, SwitchOutWhileLinkLostReclaims) {
  // H08、S12：链路丢失时被切出 OFFBOARD，重新申请
  Harness h;
  h.toFollow();
  h.linkLost();
  h.step();
  h.c.position = Vec3(7.0, 8.0, 9.0);
  h.c.path_target_yaw = 1.0;
  h.c.mode = "POSCTL";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.alerts.has(Alert::Reclaiming));
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(7.0, 8.0, 9.0)));
  EXPECT_DOUBLE_EQ(h.out.hold_yaw, 1.0);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Offboard);
  EXPECT_FALSE(h.out.follow_active);

  // 回到 OFFBOARD，对准且偏差 ≤ 0.5 m：回到原状态
  h.c.mode = "OFFBOARD";
  h.c.yaw = 1.0;
  h.c.cross_track_m = 0.3;
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTarget);
}

TEST(ReturnStateMachineTest, ReclaimChecksYawAndOffset) {
  // S12：0.5～0.7 m 进入 Rejoin；超过 0.7 m 或投影失败时保持并告警
  Harness h;
  h.toFollow();
  h.linkLost();
  h.c.path_target_yaw = 0.3;
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);

  h.c.mode = "OFFBOARD";
  h.c.cross_track_m = 1.5;
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);

  h.c.cross_track_m = 0.3;
  h.c.projection_ok = false;
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  // yaw 未对准：等待，不算偏差超限
  h.c.projection_ok = true;
  h.c.cross_track_m = 0.6;
  h.c.yaw = 0.3 + 0.5;
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_FALSE(h.out.alerts.has(Alert::PathOffset));

  h.c.yaw = 0.3;
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_EQ(h.out.follow, FollowMode::Rejoin);
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTargetRejoin);
}

TEST(ReturnStateMachineTest, ReclaimHoldsWhereOffboardIsRegained) {
  // S32、D30：飞手在链路异常时把飞机移开，拿回 OFFBOARD 时原地保持，不飞回切出时的位置
  Harness h;
  h.toFollow();
  h.linkLost();
  h.c.path_target_yaw = h.c.yaw;
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  for (int i = 1; i <= 20; ++i) {
    h.c.position += Vec3(1.0, 0.0, 0.0);
    h.step();
    EXPECT_TRUE(h.out.hold_position.isApprox(h.c.position)) << i;
  }
  const Vec3 regained = h.c.position;
  h.c.mode = "OFFBOARD";
  h.c.cross_track_m = 20.0;
  h.c.position += Vec3(0.3, 0.0, 0.0);
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(regained));
  h.run(2.0);
  EXPECT_TRUE(h.out.hold_position.isApprox(regained));
}

TEST(ReturnStateMachineTest, ReclaimExitsWhenLinkRecoversInPilotMode) {
  // R-63：重新申请期间链路恢复，模式仍不是 OFFBOARD
  Harness h;
  h.toFollow();
  h.linkLost();
  h.c.mode = "ALTCTL";
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  h.linkDegraded();
  h.run(1.0);
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  h.linkOk();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
}

TEST(ReturnStateMachineTest, ReclaimDuringHoverKeepsHoverTimer) {
  // 悬停阶段重新接入后保持在重新接入时的位置，不飞回触发点（D30）
  Harness h;
  h.startLinkLossMission();
  h.acquire();
  h.c.mode = "POSCTL";
  h.c.position = Vec3(0.5, 0.5, 3.0);
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(0.5, 0.5, 3.0)));
  h.c.position = Vec3(6.0, 0.5, 3.0);  // 飞手把飞机移开
  h.step();
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(6.0, 0.5, 3.0)));
  h.c.mode = "OFFBOARD";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(6.0, 0.5, 3.0)));
  h.c.position = Vec3(6.2, 0.5, 3.0);
  h.step();
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(6.0, 0.5, 3.0)));
  // 悬停计时从任务开始算，不因重新申请重置
  const int ticks = h.runUntil([](const StepOutput& o) {
    return o.phase == MissionPhase::Prepare;
  });
  EXPECT_GT(ticks, 0);
  EXPECT_NEAR(h.c.now, 15.05 + 15.0, 1e-6);
}

TEST(ReturnStateMachineTest, HoverKeepsPlanningResultsUntilPrepare) {
  // D3、R-17：悬停期间的规划结果在悬停结束后才处理，悬停不提前结束
  Harness r;
  r.startLinkLossMission();
  r.acquire();
  r.c.plan_status = PlanStatus::Ready;
  r.c.path_start_yaw = r.c.yaw;
  const int hover_ticks = r.runUntil([](const StepOutput& o) {
    return o.phase != MissionPhase::Hover;
  });
  EXPECT_NEAR(r.c.now - r.sm.mission().start_t, 15.0, 1e-6);
  EXPECT_GT(hover_ticks, 0);
  EXPECT_EQ(r.out.phase, MissionPhase::Prepare);

  // 需要降落的失败结果同样等到悬停结束
  Harness h;
  h.startLinkLossMission();
  h.acquire();
  h.c.plan_status = PlanStatus::FailedTooClose;
  h.run(5.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  const int ticks = h.runUntil([](const StepOutput& o) {
    return o.phase != MissionPhase::Hover;
  });
  EXPECT_GT(ticks, 0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_EQ(h.sm.handoverReason(), HandoverReason::PlanTooClose);
  // 链路丢失时直接 AUTO.LAND
  EXPECT_TRUE(h.sm.handoverDirect());
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
}

TEST(ReturnStateMachineTest, PrepareNeedsOwnedReadyAndAligned) {
  Harness h;
  h.startLowBatteryMission();
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = 1.2;
  h.run(1.0);
  // 未进入 OFFBOARD：不开始跟随；规划就绪后 yaw 改为路径起始切向
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_DOUBLE_EQ(h.out.hold_yaw, 1.2);

  h.c.mode = "OFFBOARD";
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);  // yaw 未对准
  h.c.yaw = 1.2;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_TRUE(h.out.follow_active);
  EXPECT_TRUE(h.sawEvent(EventType::State, "ReturnActive", "7", "PLAN_READY"));

  // 航点降级没有起始切向，不要求对准
  Harness f;
  f.startLowBatteryMission();
  f.acquire();
  f.c.plan_status = PlanStatus::ReadyFallback;
  f.c.yaw = 2.5;
  f.step();
  EXPECT_EQ(f.out.phase, MissionPhase::Follow);
}

TEST(ReturnStateMachineTest, PrepareWaitsUntilWithinSafeDistance) {
  // D30：开始跟随前，离路径超过安全距离（或投影失败）时原地保持并告警
  Harness h;
  h.startLowBatteryMission();
  h.acquire();
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = h.c.yaw;
  h.c.cross_track_m = 1.5;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);

  h.c.cross_track_m = 0.3;
  h.c.projection_ok = false;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  h.c.projection_ok = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_FALSE(h.out.alerts.has(Alert::PathOffset));
}

TEST(ReturnStateMachineTest, FollowHoldsWhenOffsetExceedsSafeDistance) {
  // D30：跟随中离路径超过安全距离或投影失败：原地保持、冻结进度，不回归；回到安全距离内后继续
  Harness h;
  h.toFollow();
  h.c.position = Vec3(4.0, 5.0, 6.0);
  h.c.cross_track_m = 1.5;
  h.c.deviation_confirmed = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(4.0, 5.0, 6.0)));
  EXPECT_FALSE(h.out.follow_active);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  // 保持点不随之后的位置变化
  h.c.position = Vec3(4.2, 5.0, 6.0);
  h.c.cross_track_m = 0.3;
  h.c.projection_ok = false;
  h.step();
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(4.0, 5.0, 6.0)));

  // 回到安全距离内：偏差超过回归阈值时进入回归
  h.c.projection_ok = true;
  h.c.cross_track_m = 0.6;
  h.step();
  EXPECT_EQ(h.out.follow, FollowMode::Rejoin);
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTargetRejoin);
  EXPECT_TRUE(h.out.follow_active);
  EXPECT_FALSE(h.out.alerts.has(Alert::PathOffset));
}

TEST(ReturnStateMachineTest, TrailGapHoldsInsteadOfLanding) {
  // S33、D30：轨迹在断点处截断时，到点、点过近、航点降级也构建失败都原地悬停，不降落
  Harness a;
  a.c.trail_gap = true;
  a.toArrival();
  a.run(10.0);
  EXPECT_EQ(a.out.phase, MissionPhase::Arrival);
  EXPECT_EQ(a.out.setpoint, SetpointKind::Hold);
  EXPECT_NE(a.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_TRUE(a.out.alerts.has(Alert::TrailGap));

  for (const PlanStatus failed : {PlanStatus::FailedTooClose, PlanStatus::FailedFallback}) {
    Harness h;
    h.startLowBatteryMission();
    h.acquire();
    h.c.trail_gap = true;
    h.c.plan_status = failed;
    h.run(2.0);
    EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
    EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
    EXPECT_TRUE(h.out.alerts.has(Alert::TrailGap));
  }
}

/// 找点接上的那一拍：回到准备阶段，在飞机当前位置、当前 yaw 原地保持，重新规划（D33）
void expectReanchored(const Harness& h, const char* reason) {
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_TRUE(h.out.commands.reanchor);
  EXPECT_TRUE(h.out.commands.start_planning);
  EXPECT_FALSE(h.out.commands.cancel_planning);
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(h.c.position));
  EXPECT_DOUBLE_EQ(h.out.hold_yaw, h.c.yaw);  // 不用旧轨迹的起始切向
  EXPECT_FALSE(h.out.follow_active);
  EXPECT_FALSE(h.out.alerts.has(Alert::PathOffset));
  EXPECT_TRUE(std::any_of(h.out.events.begin(), h.out.events.end(), [&](const StepEvent& e) {
    return e.type == EventType::Mission && e.name == "reanchored" && e.reason == reason;
  }));
}

TEST(ReturnStateMachineTest, PathOffsetReanchorsWhenTrailPointIsNearby) {
  // S34、D33：跟随中离路径超过安全距离，游标之前有离飞机不超过安全距离的录制点：从那里重新规划
  Harness h;
  h.toFollow();
  h.c.position = Vec3(4.0, 5.0, 6.0);
  h.c.yaw = 1.2;
  h.c.path_start_yaw = 0.3;
  h.c.cross_track_m = 1.5;
  h.c.anchor_ahead = true;
  h.step();
  expectReanchored(h, "PATH_OFFSET");
  EXPECT_TRUE(h.sawEvent(EventType::State, "ReturnPreOffboard", "7", "REANCHOR"));

  // 规划中原地保持；规划就绪、偏差在安全距离内且对准后继续跟随
  h.c.plan_status = PlanStatus::Pending;
  h.c.position = Vec3(4.1, 5.0, 6.0);
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  EXPECT_FALSE(h.out.commands.reanchor);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(4.0, 5.0, 6.0)));
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = h.c.yaw;
  h.c.cross_track_m = 0.6;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
}

TEST(ReturnStateMachineTest, ReanchorNeedsControlFreshOdomAndAnchor) {
  // D33：找不到点、不持有控制权或里程计延迟时，原地保持，不接上
  Harness h;
  h.toFollow();
  h.c.cross_track_m = 1.5;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  h.c.anchor_ahead = true;
  h.c.odom_stale = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_FALSE(h.out.commands.reanchor);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  // 本模块断流引起的飞控回退：不持有控制权
  h.c.odom_stale = false;
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Fallback);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_FALSE(h.out.commands.reanchor);

  h.c.mode = "OFFBOARD";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  expectReanchored(h, "PATH_OFFSET");
}

TEST(ReturnStateMachineTest, PrepareOffsetReanchors) {
  // D33：开始跟随前离路径超过安全距离，能找点接上就从那里重新规划
  Harness h;
  h.startLowBatteryMission();
  h.acquire();
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = h.c.yaw;
  h.c.cross_track_m = 1.5;
  h.run(1.0);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));
  h.c.path_start_yaw = h.c.yaw + 1.0;
  h.c.anchor_ahead = true;
  h.step();
  expectReanchored(h, "PATH_OFFSET");
}

TEST(ReturnStateMachineTest, ReclaimWithLargeOffsetReanchors) {
  // D33：重新申请时拿回 OFFBOARD，偏差超过安全距离但能找点接上：重新接入，从那里重新规划
  Harness h;
  h.toFollow();
  h.linkLost();
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  h.c.position = Vec3(6.0, 0.5, 3.0);
  h.c.mode = "OFFBOARD";
  h.c.cross_track_m = 1.5;
  h.c.path_target_yaw = h.c.yaw + 1.0;  // 不要求对准路径目标切向，准备阶段会对准新路径
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(h.out.alerts.has(Alert::PathOffset));

  h.c.anchor_ahead = true;
  h.step();
  expectReanchored(h, "PATH_OFFSET");
  EXPECT_TRUE(h.sawEvent(EventType::Control, "owned", "4", "RECLAIMED"));
  EXPECT_TRUE(h.sawEvent(EventType::State, "ReturnPreOffboard", "4", "REANCHOR"));
}

TEST(ReturnStateMachineTest, TrailGapReanchorsOnlyPastGap) {
  // S34、D33：停在断点处时只接断点之前的点；到点悬停结束后才找
  Harness h;
  h.c.trail_gap = true;
  h.toArrival();
  h.c.anchor_ahead = true;  // 游标之前的点在飞机所在的这一段上：不接
  h.run(10.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Arrival);
  EXPECT_FALSE(h.sawNamed(EventType::Mission, "reanchored"));

  Harness a;
  a.c.trail_gap = true;
  a.c.anchor_ahead = true;
  a.c.anchor_past_gap = true;
  a.toArrival();
  a.run(5.0 - 0.1);
  EXPECT_EQ(a.out.phase, MissionPhase::Arrival);
  a.c.position = Vec3(1.1, 2.0, 3.0);
  a.step();
  a.step();
  expectReanchored(a, "TRAIL_GAP");
  EXPECT_TRUE(a.sawEvent(EventType::State, "ReturnPreOffboard", "7", "REANCHOR"));

  // 断点处规划失败（点过近、航点降级也失败）：持有控制权后接上
  for (const PlanStatus failed : {PlanStatus::FailedTooClose, PlanStatus::FailedFallback}) {
    Harness f;
    f.startLowBatteryMission();
    f.c.trail_gap = true;
    f.c.plan_status = failed;
    f.c.anchor_ahead = true;
    f.c.anchor_past_gap = true;
    f.step();
    EXPECT_EQ(f.out.control, ControlState::Acquiring);
    EXPECT_FALSE(f.out.commands.reanchor);
    f.acquire();
    expectReanchored(f, "TRAIL_GAP");
  }
}

TEST(ReturnStateMachineTest, PlanFailuresLandOrHold) {
  Harness t;
  t.startLowBatteryMission();
  t.acquire();
  t.c.plan_status = PlanStatus::FailedFallback;
  t.step();
  EXPECT_EQ(t.out.phase, MissionPhase::Handover);
  EXPECT_EQ(t.sm.handoverReason(), HandoverReason::FallbackFailed);
  // 链路正常且在 OFFBOARD：先请求 POSCTL，期间保持
  EXPECT_FALSE(t.sm.handoverDirect());
  EXPECT_EQ(t.out.mode_intent, ModeIntent::Posctl);
  EXPECT_EQ(t.out.setpoint, SetpointKind::Hold);

  // 配置非法：保持并等待（由 R6 的准备超时兜底）
  Harness cfg;
  cfg.startLowBatteryMission();
  cfg.acquire();
  cfg.c.plan_status = PlanStatus::FailedConfig;
  cfg.run(5.0);
  EXPECT_EQ(cfg.out.phase, MissionPhase::Prepare);
  EXPECT_EQ(cfg.out.setpoint, SetpointKind::Hold);
}

TEST(ReturnStateMachineTest, FollowSwitchesBetweenPathAndRejoin) {
  Harness h;
  h.toFollow();
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTarget);
  h.c.deviation_confirmed = true;
  h.step();
  EXPECT_EQ(h.out.follow, FollowMode::Rejoin);
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTargetRejoin);
  EXPECT_TRUE(h.sawEvent(EventType::Follow, "rejoin", "7", "DEVIATION"));
  h.c.deviation_confirmed = false;
  h.c.rejoin_confirmed = true;
  h.step();
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);
}

TEST(ReturnStateMachineTest, ProgressFrozenWhileNotOwned) {
  // 区域 B 不处于 owned 时，区域 C 冻结，完成也不进入到点（R-18）
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Fallback);
  h.c.deviation_confirmed = true;
  h.c.finished = true;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);

  h.linkLost();
  h.run(3.0);
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);
}

TEST(ReturnStateMachineTest, ArrivalHoversThenStagedHandover) {
  Harness h;
  h.toArrival();
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(1.0, 2.0, 3.0)));
  h.c.finished = false;
  h.c.position = Vec3(1.2, 2.1, 3.0);
  h.run(5.0 - kDt);
  EXPECT_EQ(h.out.phase, MissionPhase::Arrival);
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_EQ(h.sm.handoverReason(), HandoverReason::Arrived);
  // 沿用到点位置；链路正常：OFFBOARD → POSCTL → AUTO.LAND
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(1.0, 2.0, 3.0)));
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Posctl);

  h.c.mode = "POSCTL";
  h.step();
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);

  h.c.mode = "AUTO.LAND";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::FcuLanding);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);
  EXPECT_FALSE(h.sm.mission().active);
}

TEST(ReturnStateMachineTest, ArrivalTimerOnlyRunsWhileOwned) {
  Harness h;
  h.toArrival();
  h.run(2.0);
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.run(2.0);
  ASSERT_EQ(h.out.control, ControlState::Fallback);
  h.c.mode = "OFFBOARD";
  h.c.stream_gap_recent = false;
  h.run(2.5);
  EXPECT_EQ(h.out.phase, MissionPhase::Arrival);
  h.run(0.6);
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
}

TEST(ReturnStateMachineTest, HandoverGoesDirectWhenPosctlDoesNotTakeEffect) {
  Harness h;
  h.toHandover();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Posctl);
  h.run(3.0 - kDt);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Posctl);
  h.step();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_TRUE(h.out.alerts.has(Alert::HandoverDirectLand));
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
}

TEST(ReturnStateMachineTest, HandoverWithLinkLostGoesDirect) {
  Harness h;
  h.toArrival();
  h.linkLost();
  h.run(5.0);
  ASSERT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_TRUE(h.sm.handoverDirect());
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);

  // 交接途中链路丢失：不再经过 POSCTL
  Harness m;
  m.toHandover();
  ASSERT_EQ(m.out.mode_intent, ModeIntent::Posctl);
  m.linkLost();
  m.step();
  EXPECT_EQ(m.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_EQ(m.out.setpoint, SetpointKind::Hold);
}

TEST(ReturnStateMachineTest, HandoverPilotTakeoverOnlyWithCleanLink) {
  // S18：交接中飞手拨开关，进入 Exited
  Harness h;
  h.toHandover();
  h.c.mode = "POSCTL";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);  // POSCTL 是我方请求的模式
  h.c.mode = "ALTCTL";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.commands.trail_cut);
  h.run(2.0);
  EXPECT_NE(h.out.mode_intent, ModeIntent::AutoLand);

  // S23：链路丢失时出现其他模式，不算接管，继续请求 AUTO.LAND
  Harness l;
  l.toHandover();
  l.linkLost();
  l.c.mode = "ALTCTL";
  l.run(2.0);
  EXPECT_EQ(l.out.phase, MissionPhase::Handover);
  EXPECT_EQ(l.out.mode_intent, ModeIntent::AutoLand);

  // 交接中短按退出
  Harness e;
  e.toHandover();
  e.shortPressNow();
  e.step();
  EXPECT_EQ(e.out.phase, MissionPhase::Exited);
}

TEST(ReturnStateMachineTest, LandedDuringHoverLandsDirectly) {
  // S25：悬停阶段高度差降到 0.5 m 以下
  Harness h;
  h.startLinkLossMission();
  h.acquire();
  h.c.airborne.airborne = false;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_EQ(h.sm.handoverReason(), HandoverReason::MissionLanded);
  EXPECT_TRUE(h.sm.handoverDirect());
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.alerts.has(Alert::MissionLanded));
  EXPECT_TRUE(h.sawEvent(EventType::State, "LandingTransition", "3", "MISSION_LANDED"));
}

TEST(ReturnStateMachineTest, LandedDuringFollowUsesFcuLandedState) {
  // 跟随阶段路径末端可能贴近起飞高度：只看飞控的落地判定
  Harness h;
  h.toFollow();
  h.c.airborne.airborne = false;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  h.c.airborne.landed_state = LandedState::OnGround;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_TRUE(h.sm.handoverDirect());
}

// ============================================================================
// AutoLandYield（规则 2）
// ============================================================================

TEST(ReturnStateMachineTest, AutoLandYieldIsLatchedAndReRequestedOnLinkLoss) {
  Harness h;
  h.toFollow();
  h.c.mode = "AUTO.LAND";
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::FcuLanding);
  EXPECT_EQ(h.out.control, ControlState::FcuLanding);
  EXPECT_EQ(h.out.setpoint, SetpointKind::None);
  EXPECT_TRUE(h.out.commands.cancel_planning);
  EXPECT_FALSE(h.sm.mission().active);

  // 链路正常时飞手从 AUTO.LAND 接管：不再干预（D18）
  h.c.mode = "POSCTL";
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::FcuLanding);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);

  // S21：链路丢失期间被切走：重新请求 AUTO.LAND
  h.linkLost();
  h.step();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_TRUE(h.out.alerts.has(Alert::AutolandOverridden));
  h.linkOk();
  h.step();
  EXPECT_EQ(h.out.mode_intent, ModeIntent::None);

  h.c.armed = false;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Disarmed);
}

TEST(ReturnStateMachineTest, AutoLandYieldsFromRecordingAndExited) {
  Harness r;
  r.toRecording();
  r.c.mode = "AUTO.LAND";
  r.step();
  EXPECT_EQ(r.out.phase, MissionPhase::FcuLanding);

  Harness e;
  e.toFollow();
  e.c.mode = "ALTCTL";
  e.step();
  ASSERT_EQ(e.out.phase, MissionPhase::Exited);
  e.c.mode = "AUTO.LAND";
  e.step();
  EXPECT_EQ(e.out.phase, MissionPhase::FcuLanding);
}

// ============================================================================
// Exited
// ============================================================================

TEST(ReturnStateMachineTest, ExitedStartsNewMissionOnLinkLoss) {
  // S11：退出后失控，开始新任务
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.record);
  h.run(10.0);
  h.linkLost();
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Hover);
  EXPECT_EQ(h.out.mission_id, 2);
  EXPECT_EQ(h.sm.mission().reason, MissionReason::LinkLost);
  EXPECT_EQ(h.sm.mission().start_mode, "POSCTL");
  EXPECT_TRUE(h.out.commands.start_planning);
  EXPECT_FALSE(h.out.record);
}

TEST(ReturnStateMachineTest, ExitedOnlyAlertsForBatteryAndLongPress) {
  // D1：退出后低电量、长按只告警
  Harness h;
  h.toFollow();
  h.c.battery.low = false;
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.phase, MissionPhase::Exited);

  h.c.battery.low = true;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.alerts.has(Alert::LowBattery));

  h.longPress();
  h.c.pilot.pressed = true;
  h.step();
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.alerts.has(Alert::ReturnRequestIgnored));
  h.c.pilot.pressed = false;
  h.step();
  EXPECT_FALSE(h.out.alerts.has(Alert::ReturnRequestIgnored));
}

TEST(ReturnStateMachineTest, ExitedDoesNotRestartOnGround) {
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.step();
  h.c.airborne.airborne = false;
  h.linkLost();
  h.run(2.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(h.out.alerts.has(Alert::TriggerIgnoredLanded));
}

TEST(ReturnStateMachineTest, ExitAlwaysRequestsTrailCut) {
  // R-78：任何阶段退出都要求截断，截断位置由执行层的游标决定；只有退出时才截断
  Harness hover;
  hover.startLinkLossMission();
  hover.acquire();
  hover.linkOk();
  hover.shortPressNow();
  hover.step();
  ASSERT_EQ(hover.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(hover.out.commands.trail_cut);
  hover.step();
  EXPECT_FALSE(hover.out.commands.trail_cut);

  Harness arrival;
  arrival.toArrival();
  arrival.shortPressNow();
  arrival.step();
  ASSERT_EQ(arrival.out.phase, MissionPhase::Exited);
  EXPECT_TRUE(arrival.out.commands.trail_cut);

  // 到点悬停结束、进入交接不是退出，不截断
  Harness handover;
  handover.toArrival();
  ASSERT_GT(handover.runUntil([](const StepOutput& o) {
    return o.phase == MissionPhase::Handover;
  }), 0);
  EXPECT_FALSE(handover.out.commands.trail_cut);
}

// ============================================================================
// 输入中断
// ============================================================================

TEST(ReturnStateMachineTest, OdomStaleFreezesAndHolds) {
  Harness h;
  h.toFollow();
  h.c.position = Vec3(4.0, 4.0, 4.0);
  h.c.odom_stale = true;
  h.step();
  EXPECT_EQ(h.out.setpoint, SetpointKind::Hold);
  EXPECT_TRUE(h.out.hold_position.isApprox(Vec3(4.0, 4.0, 4.0)));
  EXPECT_FALSE(h.out.follow_active);
  EXPECT_TRUE(h.out.alerts.has(Alert::OdomStale));
  h.c.deviation_confirmed = true;
  h.c.finished = true;
  h.run(1.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.follow, FollowMode::OnPath);

  h.c.odom_stale = false;
  h.c.finished = false;
  h.step();
  EXPECT_EQ(h.out.follow, FollowMode::Rejoin);
  EXPECT_EQ(h.out.setpoint, SetpointKind::PathTargetRejoin);
}

TEST(ReturnStateMachineTest, StaleFcuStateFreezesModeDecisions) {
  // R-62：飞控状态中断期间不按过时的模式判定
  Harness h;
  h.toFollow();
  h.c.mode = "POSCTL";
  h.c.stream_gap_recent = true;
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Fallback);
  h.c.stream_gap_recent = false;
  h.c.fcu_state_fresh = false;
  h.run(5.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_EQ(h.out.control, ControlState::Fallback);
  EXPECT_TRUE(h.out.alerts.has(Alert::FcuStateStale));

  // 恢复后按当时的数据判定：飞控已自动回到 OFFBOARD
  h.c.fcu_state_fresh = true;
  h.c.mode = "OFFBOARD";
  h.step();
  EXPECT_EQ(h.out.control, ControlState::Owned);
  EXPECT_FALSE(h.out.alerts.has(Alert::FcuStateStale));
}

TEST(ReturnStateMachineTest, ConditionAlertsArePassedThrough) {
  Harness h;
  h.toRecording();
  h.c.mode_request_failed = true;
  h.c.link.ch18_unexpected = true;
  h.c.link.ch18_value = 998;
  h.c.battery.invalid = true;
  h.c.planning_slow = true;
  h.c.planning_timed_out = true;
  h.step();
  EXPECT_TRUE(h.out.alerts.has(Alert::ModeRequestFailed));
  EXPECT_TRUE(h.out.alerts.has(Alert::Ch18Unexpected));
  EXPECT_TRUE(h.out.alerts.has(Alert::BatteryInvalid));
  EXPECT_TRUE(h.out.alerts.has(Alert::PlanningSlow));
  EXPECT_TRUE(h.out.alerts.has(Alert::PlanningTimeoutFallback));
  EXPECT_TRUE(h.sawNamed(EventType::Condition, "ch18_unexpected", "value=998"));
  EXPECT_TRUE(h.sawNamed(EventType::Alert, "CH18_UNEXPECTED"));
  EXPECT_EQ(h.out.phase, MissionPhase::Recording);
}

// ============================================================================
// 事件
// ============================================================================

TEST(ReturnStateMachineTest, EventsTraceLowBatteryThenLinkLoss) {
  // S29 的状态机部分：低电量返航中失控，到点后直接 AUTO.LAND
  Harness h;
  h.toFollow();
  h.linkLost();
  h.step();
  h.c.finished = true;
  h.step();
  h.c.finished = false;
  h.run(5.0);
  h.c.mode = "AUTO.LAND";
  h.step();
  h.c.armed = false;
  h.step();

  const std::vector<std::string> expected = {
      "condition:battery_low", "mission:started", "state:ReturnPreOffboard",
      "fcu:OFFBOARD", "control:owned", "state:ReturnActive",
      "condition:link_lost", "state:LandingHover", "state:LandingTransition",
      "fcu:AUTO.LAND", "state:AutoLandYield", "state:IdleDisarmed"};
  std::vector<std::string> seen;
  for (const auto& e : h.history) {
    std::string key = std::string(eventTypeName(e.type)) + ":" +
                      (e.name.empty() ? e.to : e.name);
    if (std::find(expected.begin(), expected.end(), key) != expected.end()) {
      seen.push_back(key);
    }
  }
  // 按顺序出现（允许重复的键只取第一次）
  std::size_t next = 0;
  for (const auto& key : seen) {
    if (next < expected.size() && key == expected[next]) {
      ++next;
    }
  }
  EXPECT_EQ(next, expected.size());

  // 每条区域转换都带规则编号和原因
  for (const auto& e : h.history) {
    if (e.type == EventType::State || e.type == EventType::Control ||
        e.type == EventType::Follow) {
      EXPECT_FALSE(e.rule.empty()) << e.from << " -> " << e.to;
      EXPECT_FALSE(e.reason.empty()) << e.from << " -> " << e.to;
    }
  }
  // 链路丢失时交接直接 AUTO.LAND，不经过 POSCTL
  EXPECT_TRUE(h.sawEvent(EventType::Alert, "on") &&
              h.sawNamed(EventType::Alert, "HANDOVER_DIRECT_LAND"));
}


// ============================================================================
// 超时与升级（第 8.1 节，R6）
// ============================================================================

/// 升级为原地降落：交接原因 ESCALATED，直接请求 AUTO.LAND（规则 6）
void expectEscalated(const Harness& h) {
  EXPECT_EQ(h.out.phase, MissionPhase::Handover);
  EXPECT_EQ(h.sm.handoverReason(), HandoverReason::Escalated);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::AutoLand);
  EXPECT_TRUE(h.out.alerts.has(Alert::EscalatedLand));
  EXPECT_TRUE(h.sawEvent(EventType::State, "LandingTransition", "6", "ESCALATED"));
}

TEST(ReturnStateMachineTest, PrepareTimeoutLandsOnlyAtLowBattery) {
  // 准备：规划结果出来后才计时，10 s 告警、30 s 超时；电量正常时一直保持（D11），确认低电量后原地降落
  Harness h;
  h.startPilotMission();
  h.c.plan_status = PlanStatus::Pending;  // 规划中不计时，由规划超时负责
  h.run(40.0);
  EXPECT_FALSE(h.out.alerts.has(Alert::PrepareSlow));

  h.c.plan_status = PlanStatus::Ready;    // 飞控一直不进入 OFFBOARD
  h.run(9.9);
  EXPECT_FALSE(h.out.alerts.has(Alert::PrepareSlow));
  h.run(0.2);
  EXPECT_TRUE(h.out.alerts.has(Alert::PrepareSlow));
  h.run(20.0);
  EXPECT_TRUE(h.out.alerts.has(Alert::PrepareTimeout));
  h.run(60.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);

  h.c.battery.low = true;
  h.step();
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, PrepareTimeoutAtLowBatteryStopsWaitingForAlignment) {
  // D32、R-85：低电量且准备超时，只差起始对准时不再等，开始跟随，不降落
  Harness h;
  h.startLowBatteryMission();
  h.acquire();
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = h.c.yaw + 1.0;  // 一直对不准
  h.run(29.9);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  h.run(0.2);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_TRUE(h.sawEvent(EventType::State, "ReturnActive", "7", "PREPARE_TIMEOUT"));
}

TEST(ReturnStateMachineTest, ReclaimTimeoutLandsOnlyAtLowBattery) {
  // S12 的超时部分：重新申请 10 s 仍未重新接入，告警 RECLAIM_TIMEOUT；电量正常时继续申请，
  // 确认低电量后原地降落
  Harness h;
  h.toFollow(false);
  h.linkLost();
  h.c.mode = "POSCTL";
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  h.run(9.9);
  EXPECT_FALSE(h.out.alerts.has(Alert::ReclaimTimeout));
  h.run(0.2);
  EXPECT_TRUE(h.out.alerts.has(Alert::ReclaimTimeout));
  h.run(30.0);
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_EQ(h.out.mode_intent, ModeIntent::Offboard);

  h.c.battery.low = true;
  h.step();
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, StallTimeoutLandsOnlyAtLowBattery) {
  // S17：剩余路径 20 s 没有缩短 0.2 m 以上，告警 PROGRESS_STALL；缩短后重新计时。
  // 60 s 后电量正常时保持，确认低电量后原地降落
  Harness h;
  h.c.remaining_m = 10.0;
  h.toFollow(false);
  h.run(19.9);
  EXPECT_FALSE(h.out.alerts.has(Alert::ProgressStall));
  h.run(0.2);
  EXPECT_TRUE(h.out.alerts.has(Alert::ProgressStall));

  h.c.remaining_m = 9.85;  // 缩短不到 0.2 m
  h.step();
  EXPECT_TRUE(h.out.alerts.has(Alert::ProgressStall));
  h.c.remaining_m = 9.75;
  h.step();
  EXPECT_FALSE(h.out.alerts.has(Alert::ProgressStall));

  h.run(70.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);
  h.c.battery.low = true;
  h.step();
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, TrailGapHoldEscalatesAfterStallTimeout) {
  // R-85：停在断点处又找不到点，按停滞计时；低电量任务 60 s 后原地降落
  Harness h;
  h.c.trail_gap = true;
  h.toArrival();
  h.run(59.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Arrival);
  EXPECT_TRUE(h.out.alerts.has(Alert::ProgressStall));
  h.run(1.1);
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, WaitsFreezeWhileOdomStale) {
  // D31：里程计延迟期间不计时，里程计中断不会引起降落
  Harness h;
  h.startLowBatteryMission();
  h.c.plan_status = PlanStatus::Ready;  // 飞控一直不进入 OFFBOARD
  h.run(20.0);
  h.c.odom_stale = true;
  h.run(100.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  h.c.odom_stale = false;
  h.run(9.9);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);
  h.run(0.2);
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, ReclaimTimerRestartsOnEachReclaim) {
  // 第 8.1 节：重新申请从进入 reclaiming 算起，上一次重新申请的时间不累加
  Harness h;
  h.toFollow();
  h.linkLost();
  h.c.mode = "POSCTL";
  h.run(8.0);
  ASSERT_EQ(h.out.control, ControlState::Reclaiming);
  h.c.mode = "OFFBOARD";
  h.c.path_target_yaw = h.c.yaw;
  h.step();
  ASSERT_EQ(h.out.control, ControlState::Owned);

  h.c.mode = "POSCTL";
  h.run(9.0);
  EXPECT_EQ(h.out.control, ControlState::Reclaiming);
  EXPECT_FALSE(h.out.alerts.has(Alert::ReclaimTimeout));
  h.run(1.1);
  expectEscalated(h);
}

TEST(ReturnStateMachineTest, ReanchorRestartsPrepareAndStallTimers) {
  // 第 8.1 节：找点接上后重新规划，准备计时归零；之后进入跟随，停滞也重新计时
  Harness h;
  h.c.remaining_m = 10.0;
  h.startPilotMission();
  h.acquire();
  h.c.plan_status = PlanStatus::Ready;
  h.c.path_start_yaw = h.c.yaw + 1.0;  // 准备 20 s 后才对准
  h.run(20.0);
  h.c.path_start_yaw = h.c.yaw;
  h.step();
  ASSERT_EQ(h.out.phase, MissionPhase::Follow);
  h.run(50.0);  // 停滞 50 s
  ASSERT_TRUE(h.out.alerts.has(Alert::ProgressStall));
  h.c.cross_track_m = 1.5;
  h.c.anchor_ahead = true;
  h.step();
  ASSERT_TRUE(h.out.commands.reanchor);

  // 低电量时准备超时就不再等对准：对不准却一直留在准备阶段，说明准备计时已归零
  h.c.battery.low = true;
  h.c.anchor_ahead = false;
  h.c.cross_track_m = 0.3;
  h.c.path_start_yaw = h.c.yaw + 1.0;
  h.run(25.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Prepare);

  h.c.path_start_yaw = h.c.yaw;
  h.c.remaining_m = 20.0;
  h.step();
  ASSERT_EQ(h.out.phase, MissionPhase::Follow);
  EXPECT_FALSE(h.out.alerts.has(Alert::ProgressStall));
  h.run(15.0);
  EXPECT_EQ(h.out.phase, MissionPhase::Follow);  // 停滞重新计时，还没到超时
}

TEST(ReturnStateMachineTest, StuckWaitsEscalateWithinLimitsAtLowBattery) {
  // SC8：低电量时，返航卡住的每种等待都在超时时升级为原地降落，不提前也不拖延。每种情形
  // 100 条随机序列：里程计、飞控状态随机中断，链路在退化和丢失之间随机切换（不会被当作飞手
  // 退出）；里程计延迟的时间不计入等待（D31）
  enum class Stuck { OffboardRejected, InvalidConfig, PathOffset, Reclaim, Stall, TrailGap };
  const std::pair<Stuck, double> cases[] = {
      {Stuck::OffboardRejected, 30.0}, {Stuck::InvalidConfig, 30.0}, {Stuck::PathOffset, 30.0},
      {Stuck::Reclaim, 10.0}, {Stuck::Stall, 60.0}, {Stuck::TrailGap, 60.0}};
  for (const auto& [stuck, limit_s] : cases) {
    for (unsigned seed = 0; seed < 100; ++seed) {
      Harness h;
      switch (stuck) {
        case Stuck::OffboardRejected:
          h.startLowBatteryMission();
          h.c.plan_status = PlanStatus::Ready;
          break;
        case Stuck::InvalidConfig:
          h.startLowBatteryMission();
          h.acquire();
          h.c.plan_status = PlanStatus::FailedConfig;
          break;
        case Stuck::PathOffset:
          h.startLowBatteryMission();
          h.acquire();
          h.c.plan_status = PlanStatus::Ready;
          h.c.cross_track_m = 1.5;
          break;
        case Stuck::Reclaim:
          h.toFollow();
          h.linkLost();
          h.c.mode = "POSCTL";
          h.step();
          break;
        case Stuck::Stall:
          h.toFollow();
          break;
        case Stuck::TrailGap:
          h.c.trail_gap = true;
          h.toArrival();
          break;
      }

      std::mt19937 rng(seed);
      auto chance = [&rng](double p) {
        return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < p;
      };
      double waited = 0.0;
      while (h.out.phase != MissionPhase::Handover && waited < limit_s + 1.0) {
        if (chance(0.05)) h.c.odom_stale = !h.c.odom_stale;
        h.c.fcu_state_fresh = !chance(0.05);
        if (chance(0.05)) {
          if (chance(0.5)) {
            h.linkLost();
          } else {
            h.linkDegraded();
          }
        }
        h.step();
        if (!h.c.odom_stale) waited += kDt;
      }
      const std::string label = std::to_string(static_cast<int>(stuck)) + " seed " +
                                std::to_string(seed);
      EXPECT_EQ(h.sm.handoverReason(), HandoverReason::Escalated) << label;
      EXPECT_NEAR(waited, limit_s, 2 * kDt) << label;
    }
  }
}

}  // namespace
}  // namespace px4_backtrack::domain
