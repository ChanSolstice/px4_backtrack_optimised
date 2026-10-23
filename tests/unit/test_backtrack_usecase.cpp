#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "px4_backtrack/application/backtrack_usecase.hpp"

namespace px4_backtrack::application {
namespace {

using domain::Alert;
using domain::ControlState;
using domain::EventType;
using domain::LandedState;
using domain::StepEvent;
using domain::Vec3;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.05;

/// 与 config/backtrack_params.yaml 一致，录制延时、点数和低电量确认缩短以加快用例
UseCaseConfig makeConfig() {
  UseCaseConfig c;
  c.record_max_hz = 100.0;
  c.record_min_points = 10;

  auto& link = c.link_config;
  link.enable_ch18 = true;
  link.ch18_failsafe_threshold = 900;
  link.ch18_confirm_s = 1.0;
  link.link_loss_leak_rate = 0.2;
  link.enable_rc_loss = true;
  link.rc_loss_timeout_s = 0.15;
  link.rc_loss_debounce_s = 0.1;
  link.link_recover_s = 2.0;
  link.link_degraded_window_s = 5.0;

  auto& pilot = c.pilot_config;
  pilot.enable_ch7 = true;
  pilot.ch7_pressed_threshold = 1500;
  pilot.ch7_hold_s = 2.0;
  pilot.ch7_exit_min_s = 0.3;
  pilot.rc_loss_timeout_s = 0.15;

  c.battery_config.enable_low_battery = true;
  c.battery_config.low_battery_threshold = 0.15;
  c.battery_config.low_battery_confirm_s = 0.5;

  c.airborne_config.airborne_min_height_m = 0.5;
  c.airborne_config.fcu_state_stale_s = 3.0;
  c.odom_stale_s = 0.25;
  c.fcu_state_stale_s = 3.0;

  auto& sm = c.state_machine_config;
  sm.record_delay_s = 1.0;
  sm.ch18_hover_wait_s = 15.0;
  sm.landing_hover_s = 5.0;
  sm.pre_offboard_setpoint_count = 6;
  sm.return_pending_max_s = 3.0;
  sm.fallback_timeout_s = 3.0;
  sm.handover_posctl_timeout_s = 3.0;
  sm.rejoin_trigger_dist_m = 0.5;
  sm.safe_distance_m = 0.7;
  sm.prepare_warn_s = 10.0;
  sm.prepare_timeout_s = 30.0;
  sm.reclaim_timeout_s = 10.0;
  sm.stall_min_progress_m = 0.2;
  sm.stall_warn_s = 20.0;
  sm.stall_timeout_s = 60.0;

  c.mode_commander_config.request_interval_s = 1.0;
  c.mode_commander_config.request_timeout_s = 3.0;
  c.fallback_gap_s = 0.5;
  c.fallback_window_s = 3.0;

  auto& rejoin = c.rejoin_config;
  rejoin.trigger_dist_m = 0.5;
  rejoin.success_dist_m = 0.4;
  rejoin.v_max_m_s = 0.3;
  rejoin.lookahead_time_s = 0.5;
  rejoin.trigger_hold_s = 0.25;
  rejoin.success_hold_s = 0.10;

  c.planning_warn_s = 10.0;
  c.planning_timeout_s = 30.0;
  c.cursor_window_m = 2.0;
  c.trajectory_config.ds_m = 0.1;
  c.trajectory_config.min_point_dist_m = 0.01;
  c.debug_trajectory_sample_dt = 0.1;
  return c;
}

/// 简化的闭环：飞控、飞手和链路
/// - 每拍送达全部输入，模式请求立即生效（OFFBOARD 要求最近 1 s 内有 setpoint）；
/// - OFFBOARD 中以 2 m/s 跟踪 setpoint（track 为假时不动，由用例直接设置位置）；
/// - OFFBOARD 中 setpoint 中断超过 1 s 回退 POSCTL，setpoint 恢复后自动回到 OFFBOARD（N1）；
/// - AUTO.LAND 以 0.7 m/s 下降，触地后判定落地，2 s 后自动上锁；
/// - 链路丢失时 CH18 = 800，CH7 保持失控瞬间的值。
class Sim {
public:
  explicit Sim(const UseCaseConfig& config = makeConfig())
    : Sim(config, std::make_unique<PlanningWorker>(planReturn, false)) {}

  /// 规划器不启动线程：每拍 tick 之后执行待处理的规划，下一拍得到结果
  PlanningWorker* planner;
  BacktrackUseCase uc;
  double t = 0.0;
  bool hold_planning = false;  ///< 为真时暂不执行规划，模拟慢规划

  // 飞控
  bool armed = false;
  std::string mode = "POSCTL";
  LandedState landed = LandedState::OnGround;
  bool fcu_connected = true;
  bool reject_offboard = false;
  bool reject_autoland = false;
  bool track = true;

  // 链路与飞手
  bool link_ok = true;
  bool rc_frames = true;
  bool ch7 = false;
  bool ch7_dip = false;  ///< 下一帧注入一次 CH7 按下
  double battery = 0.9;

  // 里程计
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  double yaw = 0.0;

  // 记录
  TickOutput out;
  std::vector<std::string> states;
  std::vector<std::pair<double, std::string>> requests;
  std::vector<StepEvent> events;
  std::vector<std::pair<double, StepEvent>> timed_events;
  double first_failsafe_t = -1.0;

  void step() {
    deliverInputs();
    out = uc.tick(t);
    if (!hold_planning) {
      planner->runPending();
    }
    if (states.empty() || states.back() != out.state) {
      states.push_back(out.state);
    }
    for (const auto& event : out.events) {
      events.push_back(event);
      timed_events.push_back({t, event});
    }
    actuate();
    t += kDt;
  }

  void run(double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < n; ++i) {
      step();
    }
  }

  bool runUntil(const std::function<bool()>& done, double timeout_s) {
    const int n = static_cast<int>(std::lround(timeout_s / kDt));
    for (int i = 0; i < n; ++i) {
      step();
      if (done()) {
        return true;
      }
    }
    return false;
  }

  bool runUntilState(const std::string& state, double timeout_s) {
    return runUntil([&] { return out.state == state; }, timeout_s);
  }

  /// 本模块卡顿：期间不调用任何回调和 tick，飞控照常运行
  void stall(double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < n; ++i) {
      fcuStep();
      t += kDt;
    }
  }

  bool in(const std::string& state) const { return out.state == state; }
  bool alert(Alert a) const { return out.alerts.has(a); }
  bool everIn(const std::string& state) const {
    return std::find(states.begin(), states.end(), state) != states.end();
  }
  int requestsSince(const std::string& m, double since) const {
    return static_cast<int>(std::count_if(requests.begin(), requests.end(), [&](const auto& r) {
      return r.first >= since && r.second == m;
    }));
  }

  /// 解锁：节点在解锁前启动，先收到未解锁的飞控状态（否则按空中重启处理，R-58）
  void arm() {
    step();
    armed = true;
    step();
  }

  /// 解锁、起飞到 2 m，向东飞 length_m 录制
  void flyOutbound(double length_m = 3.0) {
    arm();
    landed = LandedState::InAir;
    pos = Vec3(0.0, 0.0, 2.0);
    ASSERT_TRUE(runUntilState("Recording", 5.0));
    const int n = static_cast<int>(std::lround(length_m * 10.0));
    for (int i = 0; i <= n; ++i) {
      pos = Vec3(0.1 * i, 0.0, 2.0);
      step();
    }
  }

  /// 低电量返航，直到沿路径返航
  void startLowBatteryReturn() {
    flyOutbound();
    battery = 0.10;
    ASSERT_TRUE(runUntilState("ReturnActive", 5.0));
  }

private:
  Sim(const UseCaseConfig& config, std::unique_ptr<PlanningWorker> worker)
    : planner(worker.get()), uc(config, std::move(worker)) {}

  bool prev_link_ok_ = true;
  bool held_ch7_ = false;
  double last_sp_t_ = -1e9;
  bool fallback_ = false;
  double touchdown_t_ = -1.0;

  void deliverInputs() {
    // 里程计在解锁前就一直在发：先送里程计，解锁时才有解锁高度
    uc.onOdom(t, pos, vel, yaw, t);
    uc.onFcuState(t, fcu_connected, armed, mode);
    uc.onExtendedState(t, landed);
    if (rc_frames) {
      if (prev_link_ok_ && !link_ok) {
        held_ch7_ = ch7;  // 接收机保持失控瞬间的开关值
      }
      prev_link_ok_ = link_ok;
      if (!link_ok && first_failsafe_t < 0.0) {
        first_failsafe_t = t;
      }
      std::vector<uint16_t> channels(18, 1500);
      const bool pressed = link_ok ? (ch7 || ch7_dip) : (held_ch7_ || ch7_dip);
      ch7_dip = false;
      channels[6] = pressed ? 1100 : 1900;
      channels[17] = link_ok ? 1400 : 800;
      uc.onRc(t, channels);
    }
    uc.onBattery(t, battery);
  }

  void actuate() {
    if (out.setpoint) {
      last_sp_t_ = t;
      if (fallback_) {
        mode = "OFFBOARD";
        fallback_ = false;
      }
      if (mode == "OFFBOARD" && track) {
        const Vec3 delta = out.setpoint->position - pos;
        const double max_step = 2.0 * kDt;
        pos += delta.norm() > max_step ? Vec3(delta * (max_step / delta.norm())) : delta;
        vel = out.setpoint->velocity;
        yaw = out.setpoint->yaw;
      }
    }
    if (out.mode_request) {
      const std::string m = *out.mode_request;
      requests.push_back({t, m});
      uc.onModeRequestResult(t, m, true);
      const bool offboard_ok = !reject_offboard && t - last_sp_t_ <= 1.0;
      if (armed && (m != "OFFBOARD" || offboard_ok) && (m != "AUTO.LAND" || !reject_autoland)) {
        mode = m;
        fallback_ = false;
      }
    }
    fcuStep();
  }

  void fcuStep() {
    if (mode == "OFFBOARD" && t - last_sp_t_ > 1.0) {
      mode = "POSCTL";
      fallback_ = true;
    }
    if (armed && mode == "AUTO.LAND") {
      vel = Vec3(0.0, 0.0, -0.7);
      pos.z() = std::max(0.0, pos.z() - 0.7 * kDt);
      if (pos.z() <= 0.0 && landed != LandedState::OnGround) {
        landed = LandedState::OnGround;
        touchdown_t_ = t;
      }
    }
    if (armed && touchdown_t_ >= 0.0 && t - touchdown_t_ >= 2.0) {
      armed = false;
      touchdown_t_ = -1.0;
    }
  }
};

bool hasEvent(const std::vector<StepEvent>& events, EventType type,
              const std::string& name, const std::string& to = "") {
  return std::any_of(events.begin(), events.end(), [&](const StepEvent& e) {
    return e.type == type && (name.empty() || e.name == name) && (to.empty() || e.to == to);
  });
}

/// 名为 name 的事件中 key=值 的取值，按出现顺序
std::vector<double> eventValues(const std::vector<StepEvent>& events, const std::string& name,
                                const std::string& key) {
  std::vector<double> values;
  for (const auto& e : events) {
    const auto pos = e.detail.find(key + "=");
    if (e.name == name && pos != std::string::npos) {
      values.push_back(std::stod(e.detail.substr(pos + key.size() + 1)));
    }
  }
  return values;
}

// ============================================================================
// 主流程
// ============================================================================

TEST(BacktrackUseCaseTest, LowBatteryReturnLandsAtRecordingStart) {
  Sim s;
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));

  const std::vector<std::string> expected = {"Recording", "ReturnPreOffboard", "ReturnActive",
                                             "LandingHover", "LandingTransition", "AutoLandYield",
                                             "IdleDisarmed"};
  auto it = s.states.begin();
  for (const auto& state : expected) {
    it = std::find(it, s.states.end(), state);
    ASSERT_NE(it, s.states.end()) << state;
  }
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
  EXPECT_GT(s.requestsSince("OFFBOARD", 0.0), 0);
  EXPECT_GT(s.requestsSince("POSCTL", 0.0), 0);
  EXPECT_GT(s.requestsSince("AUTO.LAND", 0.0), 0);
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, LinkLossDuringReturnContinuesAndLandsDirectly) {
  // S1：低电量返航中失控，继续返航；交接不经过 POSCTL
  Sim s;
  s.startLowBatteryReturn();
  s.link_ok = false;
  const double lost_t = s.t;
  ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 2.0));
  EXPECT_EQ(s.out.control, ControlState::Owned);
  EXPECT_TRUE(s.alert(Alert::LinkLost));

  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));
  EXPECT_TRUE(s.everIn("LandingHover"));
  EXPECT_FALSE(s.everIn("Exited"));
  EXPECT_EQ(s.requestsSince("POSCTL", lost_t), 0);
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
}

TEST(BacktrackUseCaseTest, LinkLossInRecordingHoversThenReturns) {
  // S4：从第一帧失控帧到进入 OFFBOARD ≤ 1.5 s；悬停 15 s 后返航
  Sim s;
  s.flyOutbound();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.mode == "OFFBOARD"; }, 3.0));
  EXPECT_TRUE(s.in("Ch18HoverWait"));
  EXPECT_LE(s.t - s.first_failsafe_t, 1.5);

  const double hover_t = s.t;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 20.0));
  EXPECT_NEAR(s.t - hover_t, 15.0, 1.6);
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
}

// ============================================================================
// CH7 与退出
// ============================================================================

TEST(BacktrackUseCaseTest, Ch7DipAtLinkLossDoesNotExit) {
  // S2：失控瞬间 CH7 单帧低值
  Sim s;
  s.startLowBatteryReturn();
  s.link_ok = false;
  s.ch7_dip = true;
  s.run(5.0);
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, Ch7HeldThroughLossDoesNotExitOnRecovery) {
  // S3：失控期间 CH7 一直为低，恢复瞬间跳回松开
  Sim s;
  s.startLowBatteryReturn();
  s.ch7 = true;
  s.link_ok = false;
  s.run(1.5);
  s.ch7 = false;
  s.link_ok = true;
  s.run(4.0);
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, PressHeldBeforeMissionDoesNotExitButNewShortPressDoes) {
  // S5：飞手按住 CH7 时低电量先触发；这次按下不算退出，重新短按才退出
  Sim s;
  s.flyOutbound();
  s.ch7 = true;
  s.run(0.3);
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 2.0));
  s.run(0.5);
  s.ch7 = false;
  s.run(1.0);
  EXPECT_FALSE(s.everIn("Exited"));

  s.ch7 = true;
  s.run(0.5);
  s.ch7 = false;
  s.run(0.2);
  EXPECT_TRUE(s.in("Exited"));
  EXPECT_TRUE(s.alert(Alert::ReturnExited));
}

TEST(BacktrackUseCaseTest, PilotModeSwitchDuringReturnExits) {
  // S7：链路正常时飞手拨模式开关，进入 Exited，不再抢模式
  Sim s;
  s.startLowBatteryReturn();
  s.mode = "ALTCTL";
  const double switch_t = s.t;
  s.step();
  EXPECT_TRUE(s.in("Exited"));
  EXPECT_EQ(s.out.control, ControlState::Yielded);
  s.run(5.0);
  EXPECT_EQ(s.requestsSince("OFFBOARD", switch_t), 0);
  EXPECT_EQ(s.requestsSince("AUTO.LAND", switch_t), 0);
  EXPECT_FALSE(s.out.setpoint);
}

TEST(BacktrackUseCaseTest, LongPressDuringReturnIsOnlyRecorded) {
  // S8：返航中长按 CH7 超过 2 s：不退出，记录为任务中出现的条件
  Sim s;
  s.startLowBatteryReturn();
  s.track = false;  // 原地保持，返航不提前结束
  s.ch7 = true;
  s.run(2.5);
  s.ch7 = false;
  s.run(1.0);
  EXPECT_FALSE(s.everIn("Exited"));
  EXPECT_TRUE(s.in("ReturnActive"));
  EXPECT_TRUE(hasEvent(s.events, EventType::Mission, "condition_seen"));
}

// ============================================================================
// 启动条件
// ============================================================================

TEST(BacktrackUseCaseTest, LowBatteryInRecordDelayReturnsAfterRecording) {
  // S9：录制延时内低电量只告警，进入 Recording 并补足点数后返航
  Sim s;
  s.arm();
  s.landed = LandedState::InAir;
  s.pos = Vec3(0.0, 0.0, 2.0);
  s.battery = 0.10;
  s.run(0.7);
  EXPECT_TRUE(s.in("WaitRecordDelay"));
  EXPECT_TRUE(s.out.low_battery);
  EXPECT_TRUE(s.alert(Alert::LowBattery));
  EXPECT_FALSE(s.out.setpoint);

  ASSERT_TRUE(s.runUntilState("Recording", 2.0));
  for (int i = 0; i <= 15 && !s.in("ReturnPreOffboard"); ++i) {
    s.pos = Vec3(0.1 * i, 0.0, 2.0);
    s.step();
  }
  EXPECT_TRUE(s.in("ReturnPreOffboard"));
  EXPECT_TRUE(hasEvent(s.events, EventType::Mission, "started"));
}

TEST(BacktrackUseCaseTest, LinkLossAndLowBatteryTogetherStartHoverMission) {
  // S10：两个条件同时成立时走失控路径（悬停），低电量记入任务条件
  Sim s;
  s.arm();
  ASSERT_TRUE(s.runUntilState("Recording", 5.0));
  s.run(0.5);
  s.battery = 0.10;
  s.link_ok = false;
  s.run(2.0);  // 在地面：两个条件都已确认，但不启动任务
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_TRUE(s.alert(Alert::TriggerIgnoredLanded));

  s.landed = LandedState::InAir;
  s.pos = Vec3(0.0, 0.0, 2.0);
  s.step();
  EXPECT_TRUE(s.in("Ch18HoverWait"));
  const auto started = std::find_if(s.events.begin(), s.events.end(), [](const StepEvent& e) {
    return e.type == EventType::Mission && e.name == "started";
  });
  ASSERT_NE(started, s.events.end());
  EXPECT_NE(started->detail.find("link_lost"), std::string::npos);
  EXPECT_NE(started->detail.find("battery_low"), std::string::npos);
}

TEST(BacktrackUseCaseTest, InsufficientPointsLandsAfterPendingWait) {
  // 录制点不足时最多等 3 s 补点，仍不足则原地降落；不在 OFFBOARD，直接请求 AUTO.LAND
  auto config = makeConfig();
  config.record_min_points = 1000;
  Sim s(config);
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntil([&] { return s.alert(Alert::ReturnPending); }, 2.0));
  const double pending_t = s.t;
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_FALSE(s.out.setpoint);

  ASSERT_TRUE(s.runUntilState("LandingTransition", 4.0));
  EXPECT_NEAR(s.t - pending_t, 3.0, 1e-6);
  EXPECT_TRUE(s.out.low_battery);
  EXPECT_FALSE(s.out.setpoint);
  ASSERT_TRUE(s.out.mode_request);
  EXPECT_EQ(*s.out.mode_request, "AUTO.LAND");
  EXPECT_EQ(s.requestsSince("OFFBOARD", 0.0), 0);
}

TEST(BacktrackUseCaseTest, TriggersIgnoredWhileOnGround) {
  // S13：在地面，或触地但飞控未报告落地（高度差 < 0.5 m）时不启动任务
  Sim s;
  s.flyOutbound();
  s.pos = Vec3(3.0, 0.0, 0.3);  // 降到解锁高度上方 0.3 m，飞控仍报告在空中
  s.link_ok = false;
  s.run(2.0);
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_TRUE(s.out.link_lost);
  EXPECT_TRUE(s.alert(Alert::TriggerIgnoredLanded));
  EXPECT_FALSE(s.out.setpoint);
  EXPECT_FALSE(s.out.mode_request);

  s.landed = LandedState::OnGround;
  s.pos = Vec3(3.0, 0.0, 2.0);
  s.run(0.5);
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_TRUE(s.alert(Alert::TriggerIgnoredLanded));
}

TEST(BacktrackUseCaseTest, InvalidBatteryReadingDoesNotTrigger) {
  // S14：电量读数 -0.01
  Sim s;
  s.flyOutbound();
  s.battery = -0.01;
  s.run(2.0);
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_FALSE(s.out.low_battery);
  EXPECT_TRUE(s.alert(Alert::BatteryInvalid));
}

TEST(BacktrackUseCaseTest, IntermittentLossConfirmsAtSixthDrop) {
  // S20：每秒掉 0.3 s，第 6 次掉线时（约 5.2 s）确认失控；之前只报退化
  Sim s;
  s.flyOutbound();
  const double start = s.t;
  bool started = false;
  double started_t = 0.0;
  while (s.t - start < 8.0 && !started) {
    s.link_ok = std::fmod(s.t - start + 1e-6, 1.0) >= 0.3;
    s.step();
    if (s.in("Ch18HoverWait")) {
      started = true;
      started_t = s.t - start;
    } else if (s.t - start > 0.5) {
      ASSERT_FALSE(s.out.link_lost);
      ASSERT_TRUE(s.alert(Alert::LinkDegraded));
    }
  }
  ASSERT_TRUE(started);
  EXPECT_NEAR(started_t, 5.2, 0.15);
}

TEST(BacktrackUseCaseTest, UnconfirmedCh18PublishesNothing) {
  // S24：Recording 中 CH18 低但漏桶未满：不发布 setpoint、不请求模式，只告警退化
  Sim s;
  s.flyOutbound();
  s.link_ok = false;
  s.run(0.5);
  s.link_ok = true;
  s.run(1.0);
  EXPECT_TRUE(s.in("Recording"));
  EXPECT_TRUE(s.alert(Alert::LinkDegraded));
  EXPECT_FALSE(s.out.link_lost);
  EXPECT_TRUE(s.requests.empty());
  EXPECT_FALSE(s.out.setpoint);
}

// ============================================================================
// 控制权
// ============================================================================

TEST(BacktrackUseCaseTest, ReclaimAfterSwitchOutWhileLinkLost) {
  // S12：链路丢失时被切出，重新申请；回到 OFFBOARD 且偏差小时继续返航
  Sim s;
  s.startLowBatteryReturn();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 2.0));
  s.mode = "POSCTL";  // 地面站切出
  s.step();
  EXPECT_EQ(s.out.control, ControlState::Reclaiming);
  EXPECT_TRUE(s.alert(Alert::Reclaiming));
  ASSERT_TRUE(s.runUntil([&] { return s.out.control == ControlState::Owned; }, 3.0));
  EXPECT_TRUE(s.in("ReturnActive"));
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, ReclaimWithMediumOffsetRejoinsAndLargeOffsetHolds) {
  // S12：偏差 0.5～0.7 m 进入 Rejoin；超过 0.7 m 保持并告警 PATH_OFFSET
  for (const double offset : {0.6, 1.3}) {
    Sim s;
    s.startLowBatteryReturn();
    s.link_ok = false;
    ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 2.0));
    s.track = false;
    s.reject_offboard = true;
    s.mode = "POSCTL";
    s.step();
    ASSERT_EQ(s.out.control, ControlState::Reclaiming);
    s.pos.y() += offset;  // 被切出期间飞机漂开
    s.yaw = kPi;          // 已对准路径切向
    s.reject_offboard = false;
    s.run(2.0);
    if (offset < 0.7) {
      EXPECT_EQ(s.out.control, ControlState::Owned) << offset;
      EXPECT_TRUE(s.in("ReturnRejoin")) << offset;
    } else {
      EXPECT_EQ(s.out.control, ControlState::Reclaiming) << offset;
      EXPECT_TRUE(s.alert(Alert::PathOffset)) << offset;
      ASSERT_TRUE(s.out.setpoint);
      EXPECT_LT((s.out.setpoint->position - s.pos).norm(), offset + 0.1);
    }
  }
}

TEST(BacktrackUseCaseTest, OwnStallFallbackIsNotTakeover) {
  // S26：返航中控制循环卡顿 1.3 s，飞控回退 POSCTL：不退出，飞控自动回到 OFFBOARD
  Sim s;
  s.startLowBatteryReturn();
  s.stall(1.3);
  ASSERT_EQ(s.mode, "POSCTL");
  s.step();
  EXPECT_EQ(s.out.control, ControlState::Fallback);
  EXPECT_TRUE(s.alert(Alert::OffboardFallback));
  ASSERT_TRUE(s.out.setpoint);
  s.step();
  EXPECT_EQ(s.mode, "OFFBOARD");
  EXPECT_EQ(s.out.control, ControlState::Owned);
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, TakeoverBeforeOffboardConfirmedExits) {
  // S27：返航开始、OFFBOARD 确认前飞手拨开关到 ALTCTL，不再申请 OFFBOARD
  Sim s;
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 2.0));
  s.mode = "ALTCTL";
  const double switch_t = s.t;
  s.step();
  EXPECT_TRUE(s.in("Exited"));
  s.run(3.0);
  EXPECT_EQ(s.requestsSince("OFFBOARD", switch_t), 0);
}

TEST(BacktrackUseCaseTest, RejectedOffboardReportsModeRequestFailed) {
  // S28：飞控拒绝 OFFBOARD，3 s 内告警 MODE_REQUEST_FAILED，按间隔重发
  Sim s;
  s.reject_offboard = true;
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntil([&] { return !s.requests.empty(); }, 3.0));
  const double first_t = s.requests.front().first;
  ASSERT_TRUE(s.runUntil([&] { return s.alert(Alert::ModeRequestFailed); }, 4.0));
  const double alert_t = s.t - kDt;  // 本拍的时间
  EXPECT_LE(alert_t - first_t, 3.0 + kDt + 1e-9);  // 条件比 ModeCommander 晚一拍（R-71）
  s.run(2.0);
  EXPECT_GE(s.requestsSince("OFFBOARD", first_t), 5);
}

TEST(BacktrackUseCaseTest, NewFlightFirstPublishIsNotAStreamGap) {
  // R-69：上一次飞行的最后一次发布不能算作本次的断流，否则飞手接管会被误判为回退
  Sim s;
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));

  s.run(10.0);
  s.battery = 0.9;
  s.mode = "POSCTL";  // 下一次起飞前飞手把模式拨回
  s.landed = LandedState::OnGround;
  s.pos = Vec3::Zero();
  s.flyOutbound();
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntil([&] { return s.out.control == ControlState::Owned; }, 3.0));
  s.mode = "ALTCTL";
  s.step();
  EXPECT_TRUE(s.in("Exited"));
}

/// S32：返航中链路丢失，飞手切出并把飞机飞开 5 m，之后模块重新拿到 OFFBOARD
void checkReclaimAfterPilotMovedAircraft(Sim& s) {
  ASSERT_TRUE(s.in("ReturnActive"));
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 2.0));
  s.reject_offboard = true;  // 飞手的模式先保持一会儿
  s.mode = "POSCTL";
  s.track = false;
  const Vec3 switch_out = s.pos;
  for (int i = 0; i < 20; ++i) {
    s.pos += Vec3(0.0, 0.25, 0.0);
    s.step();
  }
  ASSERT_TRUE(s.alert(Alert::Reclaiming));
  s.reject_offboard = false;
  s.track = true;
  ASSERT_TRUE(s.runUntil([&] { return s.mode == "OFFBOARD"; }, 2.0));
  const Vec3 regained = s.pos;
  s.run(3.0);
  // 原地保持并告警，不飞回切出时的位置，也不直线飞回路径
  EXPECT_LT((s.pos - regained).norm(), 0.1);
  EXPECT_GT((s.pos - switch_out).norm(), 4.5);
  EXPECT_TRUE(s.alert(Alert::PathOffset));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_LT((s.out.setpoint->position - s.pos).norm(), 0.1);
}

TEST(BacktrackUseCaseTest, PilotMovedAircraftDuringReclaimHoldsInPlace) {
  Sim s;
  s.flyOutbound(10.0);
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  s.run(0.5);
  checkReclaimAfterPilotMovedAircraft(s);
  EXPECT_TRUE(s.in("ReturnActive"));
}

TEST(BacktrackUseCaseTest, PilotMovedAircraftDuringReclaimHoldsInPlaceWithWaypoints) {
  // 航点降级：偏差取到当前目标航点的距离，同样原地保持（D30）
  Sim s;
  s.flyOutbound(10.0);
  s.hold_planning = true;
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntil([&] { return s.alert(Alert::PlanningTimeoutFallback); }, 32.0));
  s.hold_planning = false;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 2.0));
  s.run(0.5);
  checkReclaimAfterPilotMovedAircraft(s);
  EXPECT_TRUE(s.in("ReturnActive"));
}

TEST(BacktrackUseCaseTest, PositionJumpDuringReturnHoldsInPlace) {
  // D30：返航中定位突跳 3 m（飞机实际没动）：原地保持，不朝路径直线飞
  Sim s;
  s.startLowBatteryReturn();
  s.run(0.5);
  s.track = false;
  s.pos += Vec3(0.0, 3.0, 0.0);
  s.run(0.5);
  EXPECT_TRUE(s.in("ReturnActive"));
  EXPECT_TRUE(s.alert(Alert::PathOffset));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_LT((s.out.setpoint->position - s.pos).norm(), 0.1);
}

TEST(BacktrackUseCaseTest, PositionJumpInRecordingStopsReturnAtGap) {
  // S33：录制中定位突跳 5 m：返航只沿突跳之后的一段回到突跳处，原地悬停，不跨过、不降落
  Sim s;
  s.flyOutbound(2.0);
  s.pos = Vec3(2.0, 5.0, 2.0);  // 定位突跳
  const Vec3 gap_end = s.pos;
  for (int i = 1; i <= 20; ++i) {
    s.pos = Vec3(2.0 + 0.1 * i, 5.0, 2.0);
    s.step();
  }
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  EXPECT_FALSE(eventValues(s.events, "submitted", "gap_at").empty());
  double min_y = s.pos.y();
  ASSERT_TRUE(s.runUntil([&] {
    min_y = std::min(min_y, s.pos.y());
    return s.in("LandingHover");
  }, 20.0));
  s.run(10.0);
  EXPECT_TRUE(s.in("LandingHover"));
  EXPECT_TRUE(s.alert(Alert::TrailGap));
  EXPECT_LT((s.pos - gap_end).norm(), 0.3);
  EXPECT_GT(min_y, 4.7);  // 没有朝突跳前的轨迹飞
  EXPECT_EQ(s.requestsSince("AUTO.LAND", 0.0), 0);
}

TEST(BacktrackUseCaseTest, NewMissionDoesNotCrossGapLeftByMovedAircraft) {
  // S33、D30：返航中链路异常，飞手把飞机移开 5 m 后退出；之后失控的新任务
  // 只沿手动段回到退出位置，不跨过截断处与退出位置之间的大间隔直线飞向起点
  Sim s;
  s.flyOutbound(10.0);
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  s.run(0.5);
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 2.0));
  s.reject_offboard = true;
  s.mode = "POSCTL";
  s.track = false;
  for (int i = 0; i < 20; ++i) {
    s.pos += Vec3(0.0, 0.25, 0.0);
    s.step();
  }
  s.link_ok = true;  // 链路恢复，飞手当时的模式生效（R-63）
  ASSERT_TRUE(s.runUntilState("Exited", 10.0));
  const Vec3 exit_pos = s.pos;
  for (int i = 0; i < 30; ++i) {  // 手动向北再飞 3 m
    s.pos += Vec3(0.0, 0.1, 0.0);
    s.step();
  }
  s.reject_offboard = false;
  s.track = true;
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  EXPECT_FALSE(eventValues(s.events, "submitted", "gap_at").empty());
  double min_y = s.pos.y();
  ASSERT_TRUE(s.runUntil([&] {
    min_y = std::min(min_y, s.pos.y());
    return s.in("LandingHover");
  }, 40.0));
  s.run(10.0);
  EXPECT_TRUE(s.in("LandingHover"));
  EXPECT_TRUE(s.alert(Alert::TrailGap));
  EXPECT_LT((s.pos - exit_pos).norm(), 0.3);  // 停在退出位置
  EXPECT_GT(min_y, exit_pos.y() - 0.3);        // 没有越过断点
  EXPECT_EQ(s.requestsSince("AUTO.LAND", 0.0), 0);
}

TEST(BacktrackUseCaseTest, PushedNearEarlierTrailReanchorsAndReturnsHome) {
  // S34、D33：往返航线，返航中链路丢失，飞手把飞机推到去程附近。拿回 OFFBOARD 后从去程上
  // 离飞机不超过安全距离的点接上，沿去程回到起点，不再绕回程
  Sim s;
  s.flyOutbound(3.0);  // 去程：y = 0，x 0→3
  for (int i = 1; i <= 10; ++i) {  // 向北 1 m
    s.pos = Vec3(3.0, 0.1 * i, 2.0);
    s.step();
  }
  for (int i = 1; i <= 20; ++i) {  // 回程：y = 1，x 3→1
    s.pos = Vec3(3.0 - 0.1 * i, 1.0, 2.0);
    s.step();
  }
  s.battery = 0.10;
  s.link_ok = false;  // 低电量返航开始后不久确认链路丢失
  ASSERT_TRUE(s.runUntil([&] { return s.out.link_lost; }, 3.0));
  ASSERT_TRUE(s.in("ReturnActive"));
  ASSERT_LT(s.pos.x(), 2.0);  // 还在回程上
  s.reject_offboard = true;
  s.mode = "POSCTL";
  s.track = false;
  for (int i = 0; i < 8; ++i) {  // 向南推 0.8 m：离回程超过安全距离，离去程 0.2 m
    s.pos += Vec3(0.0, -0.1, 0.0);
    s.step();
  }
  ASSERT_TRUE(s.alert(Alert::Reclaiming));
  const Vec3 pushed = s.pos;
  s.reject_offboard = false;
  s.track = true;
  ASSERT_TRUE(s.runUntil([&] { return hasEvent(s.events, EventType::Mission, "reanchored"); },
                         3.0));
  EXPECT_TRUE(s.in("ReturnPreOffboard"));
  EXPECT_EQ(s.out.control, ControlState::Owned);
  EXPECT_FALSE(eventValues(s.events, "trail_cut", "removed").empty());

  double max_x = s.pos.x();
  double max_y = s.pos.y();
  ASSERT_TRUE(s.runUntil([&] {
    max_x = std::max(max_x, s.pos.x());
    max_y = std::max(max_y, s.pos.y());
    return s.in("IdleDisarmed");
  }, 60.0));
  EXPECT_LT(max_x, pushed.x() + 0.05);  // 没有沿回程继续向东
  EXPECT_LT(max_y, pushed.y() + 0.05);  // 没有回到回程
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
  EXPECT_FALSE(s.everIn("Exited"));
}

TEST(BacktrackUseCaseTest, GapEndNearEarlierTrailReanchorsAndReturnsHome) {
  // S34、D33：录制中定位突跳形成断点，突跳后的位置离突跳前的轨迹不远。返航到断点处，
  // 到点悬停结束后从断点之前的轨迹接上，沿原路回到起点降落
  Sim s;
  s.flyOutbound(3.0);  // y = 0，x 0→3
  for (int i = 1; i <= 20; ++i) {  // 向北 2 m
    s.pos = Vec3(3.0, 0.1 * i, 2.0);
    s.step();
  }
  const Vec3 gap_end(3.3, 0.3, 2.0);  // 定位突跳 1.7 m
  for (int i = 0; i <= 17; ++i) {     // 向东 1.7 m
    s.pos = gap_end + Vec3(0.1 * i, 0.0, 0.0);
    s.step();
  }
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  EXPECT_FALSE(eventValues(s.events, "submitted", "gap_at").empty());
  ASSERT_TRUE(s.runUntilState("LandingHover", 20.0));
  EXPECT_LT((s.pos - gap_end).norm(), 0.3);
  const double arrived_t = s.t;
  ASSERT_TRUE(s.runUntil([&] { return hasEvent(s.events, EventType::Mission, "reanchored"); },
                         6.0));
  EXPECT_GE(s.t - arrived_t, 4.9);  // 到点悬停结束后才找
  EXPECT_TRUE(s.in("ReturnPreOffboard"));

  double max_y = s.pos.y();
  ASSERT_TRUE(s.runUntil([&] {
    max_y = std::max(max_y, s.pos.y());
    return s.in("IdleDisarmed");
  }, 60.0));
  EXPECT_LT(max_y, gap_end.y() + 0.25);  // 接上去程，没有沿向北的一段飞
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
  EXPECT_EQ(eventValues(s.events, "submitted", "gap_at").size(), 1u);  // 接上后的轨迹没有断点
}

TEST(BacktrackUseCaseTest, ShortSegmentAfterGapHoldsWithoutReanchoring) {
  // D30、D33：定位突跳 5 m 后马上低电量，突跳后的一段太短，规划失败：原地悬停；断点之前
  // 没有离飞机不超过安全距离的点，不接上，也不接飞机所在的这一段
  Sim s;
  s.flyOutbound(2.0);
  s.pos = Vec3(2.0, 5.0, 2.0);
  s.run(1.0);
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 5.0));
  s.run(10.0);
  EXPECT_TRUE(s.in("ReturnPreOffboard"));
  EXPECT_TRUE(s.alert(Alert::TrailGap));
  EXPECT_FALSE(hasEvent(s.events, EventType::Mission, "reanchored"));
  EXPECT_EQ(s.requestsSince("AUTO.LAND", 0.0), 0);
  EXPECT_LT((s.pos - Vec3(2.0, 5.0, 2.0)).norm(), 0.1);
}

// ============================================================================
// 降落与让步
// ============================================================================

TEST(BacktrackUseCaseTest, PilotSwitchDuringHandoverExits) {
  // S18：降落交接中飞手拨开关：进入 Exited，不再请求 AUTO.LAND
  Sim s;
  s.reject_autoland = true;
  s.startLowBatteryReturn();
  ASSERT_TRUE(s.runUntil([&] { return s.in("LandingTransition") && s.mode == "POSCTL"; }, 30.0));
  s.mode = "ALTCTL";
  const double switch_t = s.t;
  s.step();
  EXPECT_TRUE(s.in("Exited"));
  s.run(3.0);
  EXPECT_EQ(s.requestsSince("AUTO.LAND", switch_t), 0);
}

TEST(BacktrackUseCaseTest, HandoverWithLinkLostKeepsRequestingAutoLand) {
  // S23：降落交接中链路丢失，模式变为 ALTCTL：不进入 Exited，继续请求 AUTO.LAND
  Sim s;
  s.reject_autoland = true;
  s.startLowBatteryReturn();
  ASSERT_TRUE(s.runUntil([&] { return s.in("LandingTransition") && s.mode == "POSCTL"; }, 30.0));
  s.link_ok = false;
  s.mode = "ALTCTL";
  const double lost_t = s.t;
  s.run(4.0);
  EXPECT_TRUE(s.in("LandingTransition"));
  EXPECT_TRUE(s.alert(Alert::HandoverDirectLand));
  EXPECT_GE(s.requestsSince("AUTO.LAND", lost_t), 2);
}

TEST(BacktrackUseCaseTest, AutoLandOverriddenWhileLinkLostIsRequestedAgain) {
  // S21：AutoLandYield 中链路丢失、模式被切到 POSCTL：每 1 s 请求 AUTO.LAND；链路恢复后停止
  Sim s;
  s.flyOutbound();
  s.mode = "AUTO.LAND";  // 地面站让飞机降落
  s.step();
  ASSERT_TRUE(s.in("AutoLandYield"));
  s.reject_autoland = true;
  s.link_ok = false;
  s.mode = "POSCTL";  // 摇杆超控
  const double lost_t = s.t;
  s.run(4.0);
  EXPECT_TRUE(s.in("AutoLandYield"));
  EXPECT_TRUE(s.alert(Alert::AutolandOverridden));
  EXPECT_GE(s.requestsSince("AUTO.LAND", lost_t), 3);
  EXPECT_FALSE(s.out.setpoint);

  s.link_ok = true;
  s.run(3.0);
  const double recovered_t = s.t;
  s.run(3.0);
  EXPECT_EQ(s.requestsSince("AUTO.LAND", recovered_t), 0);
  EXPECT_FALSE(s.alert(Alert::AutolandOverridden));
}

TEST(BacktrackUseCaseTest, TouchdownDuringHoverLandsDirectly) {
  // S25：失控任务悬停阶段高度差降到 0.5 m 以下：直接请求 AUTO.LAND，不起飞
  Sim s;
  s.flyOutbound();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.mode == "OFFBOARD"; }, 3.0));
  s.track = false;
  s.pos.z() = 0.3;
  s.step();
  EXPECT_TRUE(s.in("LandingTransition"));
  EXPECT_TRUE(s.alert(Alert::MissionLanded));
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 10.0));
  EXPECT_LE(s.pos.z(), 0.3);
}

TEST(BacktrackUseCaseTest, AutoLandYieldIsLatchedUntilDisarm) {
  // 飞控降落接管后锁存到上锁：链路正常时不发 setpoint、不请求模式
  Sim s;
  s.startLowBatteryReturn();
  s.mode = "AUTO.LAND";
  s.step();
  ASSERT_TRUE(s.in("AutoLandYield"));
  const double yield_t = s.t;
  s.mode = "POSCTL";
  s.ch7 = true;
  s.run(3.0);
  EXPECT_TRUE(s.in("AutoLandYield"));
  EXPECT_FALSE(s.out.setpoint);
  EXPECT_TRUE(s.requests.empty() || s.requests.back().first < yield_t);

  s.armed = false;
  s.step();
  EXPECT_TRUE(s.in("IdleDisarmed"));
}

// ============================================================================
// 事件
// ============================================================================

TEST(BacktrackUseCaseTest, EventsTraceLowBatteryReturnWithLinkLoss) {
  // S29：完成一次 S1，事件按时间依次记录，区域转换都带规则编号和原因
  Sim s;
  s.startLowBatteryReturn();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));

  struct Expected {
    EventType type;
    std::string name;
    std::string to;
  };
  const std::vector<Expected> sequence = {
      {EventType::Condition, "battery_low", "on"},
      {EventType::Mission, "started", ""},
      {EventType::Control, "", "acquiring"},
      {EventType::Fcu, "", "OFFBOARD"},
      {EventType::Control, "", "owned"},
      {EventType::Condition, "link_lost", "on"},
      {EventType::State, "", "LandingHover"},
      {EventType::State, "", "LandingTransition"},
      {EventType::ModeRequest, "sent", "AUTO.LAND"},
      {EventType::Fcu, "", "AUTO.LAND"},
      {EventType::State, "", "AutoLandYield"},
      {EventType::State, "", "IdleDisarmed"},
  };
  auto it = s.events.begin();
  for (const auto& e : sequence) {
    it = std::find_if(it, s.events.end(), [&](const StepEvent& event) {
      return event.type == e.type && (e.name.empty() || event.name == e.name) &&
             (e.to.empty() || event.to == e.to);
    });
    ASSERT_NE(it, s.events.end()) << domain::eventTypeName(e.type) << " " << e.name << " " << e.to;
  }
  for (const auto& event : s.events) {
    if (event.type == EventType::State || event.type == EventType::Control) {
      EXPECT_FALSE(event.rule.empty()) << event.from << "->" << event.to;
      EXPECT_FALSE(event.reason.empty()) << event.from << "->" << event.to;
    }
  }
  const auto started = std::find_if(s.events.begin(), s.events.end(), [](const StepEvent& e) {
    return e.type == EventType::Mission && e.name == "started";
  });
  EXPECT_EQ(started->reason, "LOW_BATTERY");
}

// ============================================================================
// 后台规划（R4）
// ============================================================================

bool hasPlanningReason(const std::vector<StepEvent>& events, const std::string& reason) {
  return std::any_of(events.begin(), events.end(), [&](const StepEvent& e) {
    return e.type == EventType::Planning && e.reason == reason;
  });
}

TEST(BacktrackUseCaseTest, BackgroundPlanningKeepsPublishingThroughHover) {
  // S15：规划在失控悬停期间进行，控制循环每拍照常发布；结果在悬停结束后才处理（D3）
  Sim s;
  s.flyOutbound();
  s.hold_planning = true;
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  ASSERT_TRUE(hasEvent(s.events, EventType::Planning, "submitted"));
  const double start_t = s.t;

  int ticks = 0;
  int published = 0;
  double slow_t = -1.0;
  while (s.t - start_t < 12.0 - 1e-9) {
    s.step();
    ++ticks;
    if (s.out.setpoint) ++published;
    if (slow_t < 0.0 && s.alert(Alert::PlanningSlow)) slow_t = s.t;
  }
  EXPECT_EQ(published, ticks);
  EXPECT_NEAR(slow_t - start_t, 10.0, 1e-6);
  EXPECT_TRUE(s.in("Ch18HoverWait"));

  s.hold_planning = false;  // 第 12 s 规划完成
  s.run(0.1);
  EXPECT_TRUE(hasPlanningReason(s.events, "READY"));
  EXPECT_FALSE(s.alert(Alert::PlanningSlow));
  EXPECT_TRUE(s.in("Ch18HoverWait"));

  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 4.0));
  EXPECT_NEAR(s.t - start_t, 15.0, 1e-6);
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));
  EXPECT_TRUE(s.everIn("ReturnActive"));
  EXPECT_LT(s.pos.head<2>().norm(), 0.3);
}

TEST(BacktrackUseCaseTest, PlanningTimeoutFallsBackToWaypoints) {
  // S16：规划 30 s 未完成，丢弃结果，在控制线程构建航点降级返航；迟到的结果不采用
  Sim s;
  s.flyOutbound();
  s.hold_planning = true;
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 2.0));
  const double start_t = s.t;

  ASSERT_TRUE(s.runUntil([&] { return s.alert(Alert::PlanningTimeoutFallback); }, 31.0));
  EXPECT_NEAR(s.t - start_t, 30.0, 1e-6);
  EXPECT_TRUE(hasPlanningReason(s.events, "TIMEOUT"));
  EXPECT_TRUE(hasPlanningReason(s.events, "WAYPOINT_FALLBACK"));
  EXPECT_FALSE(s.alert(Alert::PlanningSlow));

  s.hold_planning = false;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 2.0));
  EXPECT_TRUE(s.alert(Alert::PlanningTimeoutFallback));  // 持续到任务结束
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 60.0));
  EXPECT_FALSE(hasPlanningReason(s.events, "READY"));
  EXPECT_TRUE(s.everIn("LandingHover"));
  EXPECT_FALSE(s.alert(Alert::PlanningTimeoutFallback));
}

TEST(BacktrackUseCaseTest, ExitDuringPlanningCancelsIt) {
  // 规划中退出：取消规划，结果不再采用；之后失控的新任务重新提交
  Sim s;
  s.flyOutbound();
  s.hold_planning = true;
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 2.0));
  s.run(1.0);
  s.ch7 = true;  // 短按退出
  s.run(0.5);
  s.ch7 = false;
  ASSERT_TRUE(s.runUntilState("Exited", 1.0));
  EXPECT_TRUE(hasEvent(s.events, EventType::Planning, "cancelled"));

  s.hold_planning = false;
  s.run(0.5);
  EXPECT_FALSE(hasEvent(s.events, EventType::Planning, "completed"));

  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  s.run(0.1);
  EXPECT_TRUE(hasPlanningReason(s.events, "READY"));
}

// ============================================================================
// 执行层（规划、跟随与输入）
// ============================================================================

TEST(BacktrackUseCaseTest, TooCloseTrajectoryLandsDirectly) {
  // 原地悬停录制：规划判定点过近，原地降落
  Sim s;
  s.arm();
  s.landed = LandedState::InAir;
  s.pos = Vec3(0.0, 0.0, 2.0);
  ASSERT_TRUE(s.runUntilState("Recording", 5.0));
  s.run(1.0);
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("LandingTransition", 3.0));
  EXPECT_TRUE(hasEvent(s.events, EventType::Planning, "failed"));
  EXPECT_GT(s.requestsSince("AUTO.LAND", 0.0), 0);
}

TEST(BacktrackUseCaseTest, InvalidTrajectoryConfigHoldsWithoutFallback) {
  auto config = makeConfig();
  config.trajectory_config.v_end_max = config.trajectory_config.v_max + 1.0;
  Sim s(config);
  s.flyOutbound();
  const Vec3 trigger = s.pos;
  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 2.0));
  s.run(3.0);
  EXPECT_TRUE(s.in("ReturnPreOffboard"));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_TRUE(s.out.setpoint->position.isApprox(trigger, 1e-6));
  EXPECT_TRUE(s.out.setpoint->velocity.isZero());
  EXPECT_TRUE(hasEvent(s.events, EventType::Planning, "failed"));
}

TEST(BacktrackUseCaseTest, AlignsStartYawBeforeHorizontalReturn) {
  // 规划就绪后先保持触发位置并对准路径起始切向，误差 ≤ 15° 才开始水平返航
  Sim s;
  s.flyOutbound();
  s.track = false;
  s.battery = 0.10;
  s.run(2.0);
  ASSERT_TRUE(s.in("ReturnPreOffboard"));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_TRUE(s.out.setpoint->position.isApprox(Vec3(3.0, 0.0, 2.0), 1e-6));
  EXPECT_TRUE(s.out.setpoint->velocity.isZero());
  EXPECT_NEAR(std::abs(s.out.setpoint->yaw), kPi, 1e-3);

  s.yaw = kPi;
  s.step();
  s.step();
  EXPECT_TRUE(s.in("ReturnActive"));
}

TEST(BacktrackUseCaseTest, WallTimeCannotAdvanceSpatialTrajectory) {
  // 长按触发、电量正常：时钟跳变只会让停滞告警，不会升级降落（第 8.1 节）
  Sim s;
  s.flyOutbound();
  s.ch7 = true;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0));
  s.ch7 = false;
  s.track = false;
  s.t += 20000.0;  // 时钟跳变
  s.run(1.0);
  EXPECT_TRUE(s.in("ReturnActive"));

  for (int i = 0; i <= 30 && !s.in("LandingHover"); ++i) {
    s.pos = Vec3(3.0 - 0.1 * i, 0.0, 2.0);
    s.step();
  }
  s.vel = Vec3::Zero();
  s.run(0.2);
  EXPECT_TRUE(s.in("LandingHover"));
}

TEST(BacktrackUseCaseTest, ReturnEndToleranceIsIndependentFromRejoinSuccessDistance) {
  // 回到 Rejoin 的 0.4 m 安全走廊不等于到达终点，终点仍要求 0.2 m
  Sim s;
  s.startLowBatteryReturn();
  s.track = false;
  for (int i = 0; i <= 30; ++i) {
    s.pos = Vec3(3.0 - 0.1 * i, 0.0, 2.3);
    s.step();
  }
  s.vel = Vec3::Zero();
  s.run(0.3);
  EXPECT_TRUE(s.in("ReturnActive"));

  s.pos = Vec3(0.0, 0.0, 2.19);
  s.run(0.3);
  EXPECT_TRUE(s.in("LandingHover"));
}

TEST(BacktrackUseCaseTest, RejoinUsesPathYawAndWaitsForVelocityConvergence) {
  auto config = makeConfig();
  config.rejoin_config.trigger_hold_s = 0.0;
  config.rejoin_config.success_hold_s = 0.0;
  config.trajectory_config.enable_end_converge = false;
  config.trajectory_config.v_max = 1.0;
  config.trajectory_config.v_end_max = 1.0;
  config.trajectory_config.v_start_m_s = 1.0;
  config.trajectory_config.v_end_m_s = 1.0;
  Sim s(config);
  s.startLowBatteryReturn();
  s.track = false;

  s.pos = Vec3(2.9, 0.6, 2.0);  // 回归阈值与安全距离之间
  Vec3 previous_velocity = s.out.setpoint->velocity;
  ASSERT_TRUE(s.runUntil([&] {
    // 进入 Rejoin 前后，相邻两拍目标速度的变化不超过 0.3 m/s
    EXPECT_LE((s.out.setpoint->velocity - previous_velocity).norm(), 0.3 + 1e-9);
    previous_velocity = s.out.setpoint->velocity;
    return s.in("ReturnRejoin");
  }, 1.0));
  EXPECT_NEAR(std::abs(s.out.setpoint->yaw), kPi, 1e-3);

  // 位置已回到路径但速度尚未恢复时，不能退出 Rejoin
  s.pos = Vec3(2.8, 0.0, 2.0);
  s.vel = Vec3::Zero();
  s.run(0.2);
  EXPECT_TRUE(s.in("ReturnRejoin"));

  s.vel = Vec3(-1.0, 0.0, 0.0);
  ASSERT_TRUE(s.runUntilState("ReturnActive", 3.0));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_NEAR(std::abs(s.out.setpoint->yaw), kPi, 1e-3);
}

TEST(BacktrackUseCaseTest, NonFiniteOdometryIsIgnoredAndHolds) {
  // R-73：含 NaN 的里程计视为没有收到，0.25 s 后保持在最后一个有效位置
  Sim s;
  s.startLowBatteryReturn();
  s.track = false;
  s.step();
  const Vec3 last_valid = s.pos;
  s.pos.x() = std::numeric_limits<double>::quiet_NaN();
  s.run(0.5);
  EXPECT_TRUE(s.alert(Alert::OdomStale));
  ASSERT_TRUE(s.out.setpoint);
  EXPECT_TRUE(s.out.setpoint->position.allFinite());
  EXPECT_TRUE(s.out.setpoint->position.isApprox(last_valid, 1e-6));
  EXPECT_TRUE(s.out.setpoint->velocity.isZero());
}

TEST(BacktrackUseCaseTest, DisconnectedFcuStateIsIgnored) {
  // MAVROS 未连上飞控时也发布 armed=false，不能当作上锁
  Sim s;
  s.flyOutbound();
  s.fcu_connected = false;
  s.armed = false;
  s.run(1.0);
  EXPECT_TRUE(s.in("Recording"));
  s.run(2.5);
  EXPECT_TRUE(s.alert(Alert::FcuStateStale));
  EXPECT_TRUE(s.in("Recording"));
}

TEST(BacktrackUseCaseTest, RecordingResumesAfterExitButNotDuringMission) {
  // 退出后从截断处继续录制；新任务基于"截断后的轨迹 + 退出后的手动段"规划
  Sim s;
  s.startLowBatteryReturn();
  s.run(2.0);
  s.mode = "ALTCTL";
  s.step();
  ASSERT_TRUE(s.in("Exited"));
  int exited_ticks = 0;
  for (int i = 0; i < 20; ++i) {
    s.pos += Vec3(0.0, 0.1, 0.0);
    s.step();
    ++exited_ticks;
  }
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  exited_ticks += 21;  // 等待确认失控的这段时间仍在 Exited
  const auto points = eventValues(s.events, "submitted", "points");
  const auto kept = eventValues(s.events, "trail_cut", "kept");
  ASSERT_EQ(points.size(), 2u);
  ASSERT_EQ(kept.size(), 1u);
  EXPECT_LT(kept[0], points[0]);  // 返航了一段，截断了
  EXPECT_GT(points[1], kept[0]);
  EXPECT_LE(points[1] - kept[0], exited_ticks + 1);
}

// ============================================================================
// 退出后新轨迹（R5）
// ============================================================================

TEST(BacktrackUseCaseTest, ShortPressDuringReturnCutsTrailAtCursor) {
  // S6：返航中短按退出：录制截断到游标，继续录制，请求 POSCTL
  Sim s;
  s.startLowBatteryReturn();
  s.run(1.0);
  s.ch7 = true;
  s.run(0.5);
  s.ch7 = false;
  ASSERT_TRUE(s.runUntilState("Exited", 1.0));
  const double exit_t = s.t;

  const auto recorded = eventValues(s.events, "submitted", "points");
  const auto kept = eventValues(s.events, "trail_cut", "kept");
  const auto removed = eventValues(s.events, "trail_cut", "removed");
  ASSERT_EQ(kept.size(), 1u);
  EXPECT_GT(removed[0], 0.0);
  EXPECT_EQ(kept[0] + removed[0], recorded[0]);
  EXPECT_GT(s.requestsSince("POSCTL", exit_t - kDt), 0);

  // 继续录制：下一次规划的点数 = 保留的点 + 退出后录到的点
  s.run(1.0);
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  const auto points = eventValues(s.events, "submitted", "points");
  ASSERT_EQ(points.size(), 2u);
  EXPECT_GT(points[1], kept[0]);
}

TEST(BacktrackUseCaseTest, NewMissionAfterExitFollowsCutTrailAndManualSegment) {
  // S11：退出后失控：新任务悬停 15 s，沿"截断后的轨迹 + 手动段"返航，从当前位置出发，
  // 不再回到原触发点（x = 3）；续录的时间戳接在截断处之后，不含第一次任务的时长
  Sim s;
  s.startLowBatteryReturn();
  s.run(1.0);
  s.ch7 = true;
  s.run(0.5);
  s.ch7 = false;
  ASSERT_TRUE(s.runUntilState("Exited", 1.0));
  const double exit_t = s.t;
  const Vec3 exit_pos = s.pos;
  ASSERT_LT(exit_pos.x(), 2.8);  // 已经往回飞了一段

  for (int i = 0; i < 15; ++i) {  // 手动向北飞 1.5 m
    s.pos += Vec3(0.0, 0.1, 0.0);
    s.step();
  }
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  const double exited_s = s.t - exit_t;
  const Vec3 start_pos = s.pos;

  const auto duration = eventValues(s.events, "submitted", "duration_s");
  ASSERT_EQ(duration.size(), 2u);
  EXPECT_LT(duration[1], duration[0] + exited_s + 0.1);

  ASSERT_TRUE(s.runUntilState("ReturnPreOffboard", 16.0));
  EXPECT_LT((s.pos - start_pos).norm(), 0.05);  // 悬停在当前位置
  double max_x = s.pos.x();
  double min_to_exit = (s.pos - exit_pos).norm();
  ASSERT_TRUE(s.runUntil([&] {
    max_x = std::max(max_x, s.pos.x());
    min_to_exit = std::min(min_to_exit, (s.pos - exit_pos).norm());
    return s.in("IdleDisarmed");
  }, 60.0));
  EXPECT_LT(max_x, exit_pos.x() + 0.3);   // 没有回到原触发点
  EXPECT_LT(min_to_exit, 0.3);            // 先沿手动段回到退出位置
  EXPECT_LT(s.pos.head<2>().norm(), 0.3); // 再沿截断后的轨迹回到录制起点
}

TEST(BacktrackUseCaseTest, ExitDuringHoverKeepsWholeTrail) {
  // R-65、R-78：悬停阶段还没沿返航路径移动，游标仍在末点，退出时不截断；任务期间不录制
  Sim s;
  s.flyOutbound();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("Ch18HoverWait", 2.0));
  s.link_ok = true;
  s.run(5.5);  // 链路恢复，退化窗口结束
  ASSERT_TRUE(s.in("Ch18HoverWait"));
  s.ch7 = true;
  s.run(0.5);
  s.ch7 = false;
  ASSERT_TRUE(s.runUntilState("Exited", 1.0));
  const auto removed = eventValues(s.events, "trail_cut", "removed");
  ASSERT_EQ(removed.size(), 1u);
  EXPECT_EQ(removed[0], 0.0);
}

TEST(BacktrackUseCaseTest, SwitchOutRightAfterNewMissionReclaimsWithoutFallback) {
  // R-69：退出后不发布，新任务第一次发布不算断流；链路丢失时被切出直接重新申请，不当作回退
  Sim s;
  s.startLowBatteryReturn();
  s.mode = "ALTCTL";
  s.step();
  ASSERT_TRUE(s.in("Exited"));
  s.run(5.0);
  s.link_ok = false;
  ASSERT_TRUE(s.runUntil([&] { return s.out.control == ControlState::Owned; }, 4.0));
  s.mode = "POSCTL";
  s.step();
  EXPECT_EQ(s.out.control, ControlState::Reclaiming);
}


// ============================================================================
// 超时与升级（R6）
// ============================================================================

TEST(BacktrackUseCaseTest, StallHoldsUntilLowBatteryThenLandsInPlace) {
  // S17：链路丢失时返航，定位突跳 3 m 后离路径超过安全距离、找不到点接上，原地保持；
  // 20 s 告警 PROGRESS_STALL，电量正常时一直保持；确认低电量后原地降落
  Sim s;
  s.flyOutbound();
  s.link_ok = false;
  ASSERT_TRUE(s.runUntilState("ReturnActive", 20.0));
  s.run(0.5);
  s.track = false;
  s.pos += Vec3(0.0, 3.0, 0.0);
  const Vec3 hold = s.pos;
  s.run(21.0);
  EXPECT_TRUE(s.alert(Alert::PathOffset));
  EXPECT_TRUE(s.alert(Alert::ProgressStall));
  s.run(60.0);
  EXPECT_TRUE(s.in("ReturnActive"));
  EXPECT_EQ(s.requestsSince("AUTO.LAND", 0.0), 0);

  s.battery = 0.10;
  ASSERT_TRUE(s.runUntilState("LandingTransition", 2.0));
  EXPECT_TRUE(s.alert(Alert::EscalatedLand));
  ASSERT_TRUE(s.runUntilState("IdleDisarmed", 15.0));
  EXPECT_LT((s.pos - hold).head<2>().norm(), 0.1);  // 原地降落
}

TEST(BacktrackUseCaseTest, LongReturnRaisesNoProgressStall) {
  // 停滞判定用执行层的剩余路径：轨迹和航点降级的正常返航超过 20 s 也不告警停滞、不升级
  for (const bool fallback : {false, true}) {
    Sim s;
    s.flyOutbound(45.0);
    s.hold_planning = fallback;
    s.battery = 0.10;
    if (fallback) {
      ASSERT_TRUE(s.runUntil([&] { return s.alert(Alert::PlanningTimeoutFallback); }, 32.0));
      s.hold_planning = false;
    }
    ASSERT_TRUE(s.runUntilState("ReturnActive", 5.0)) << fallback;
    const double start_t = s.t;
    ASSERT_TRUE(s.runUntilState("IdleDisarmed", 120.0)) << fallback;
    EXPECT_GT(s.t - start_t, 25.0) << fallback;
    EXPECT_FALSE(hasEvent(s.events, EventType::Alert, "PROGRESS_STALL", "on")) << fallback;
    EXPECT_FALSE(hasEvent(s.events, EventType::Alert, "ESCALATED_LAND", "on")) << fallback;
    EXPECT_LT(s.pos.head<2>().norm(), 0.3) << fallback;
  }
}

}  // namespace
}  // namespace px4_backtrack::application
