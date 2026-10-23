#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "px4_backtrack/domain/return_state_machine.hpp"

// 穷举不变量检查（重构方案第 13 节 R2、系统性分析第 4 节）：
// 1. 随机游走收集可达的状态机快照，游走本身也逐拍检查；
// 2. 对每个快照，遍历关键条件的全部组合各走一拍，检查 SC1–SC7、SC9–SC11 和结构一致性。

namespace px4_backtrack::domain {
namespace {

constexpr double kDt = 0.05;
const char* const kModes[] = {"OFFBOARD", "POSCTL", "AUTO.LAND", "ALTCTL"};

constexpr double kSafeDistanceM = 1.0;

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
  config.safe_distance_m = kSafeDistanceM;
  return config;
}

/// 第 n 拍的位置。保持点必须是某一拍的位置，不能凭空产生或为原点（SC7）
Vec3 positionFor(long n) {
  return Vec3(1000.0 + static_cast<double>(n), 2000.0 + 2.0 * n, 3.0);
}

bool isRecordedPosition(const Vec3& p) {
  return std::isfinite(p.x()) && p.x() >= 1000.0 - 1e-9 &&
         std::abs(p.y() - 2.0 * p.x()) < 1e-6 && std::abs(p.z() - 3.0) < 1e-9;
}

bool holdsPhase(MissionPhase phase) {
  return phase == MissionPhase::Hover || phase == MissionPhase::Prepare ||
         phase == MissionPhase::Follow || phase == MissionPhase::Arrival;
}

/// 区域 A 与区域 B 的组合是否合法（第 6.1 节）
bool consistent(MissionPhase phase, ControlState control) {
  switch (phase) {
    case MissionPhase::Disarmed:
    case MissionPhase::RecordDelay:
    case MissionPhase::Recording:
      return control == ControlState::None;
    case MissionPhase::Hover:
    case MissionPhase::Prepare:
      return control == ControlState::Acquiring || control == ControlState::Owned ||
             control == ControlState::Fallback || control == ControlState::Reclaiming;
    case MissionPhase::Follow:
    case MissionPhase::Arrival:
      return control == ControlState::Owned || control == ControlState::Fallback ||
             control == ControlState::Reclaiming;
    case MissionPhase::Handover:
      return control == ControlState::Handover;
    case MissionPhase::Exited:
      return control == ControlState::Yielded;
    case MissionPhase::FcuLanding:
      return control == ControlState::FcuLanding;
  }
  return false;
}

const char* linkText(const LinkStatus& link) {
  return link.lost ? "lost" : link.degraded ? "degraded" : "ok";
}

class InvariantChecker {
public:
  /// 检查一拍：before 为这一拍之前的状态机，after 为之后的
  void check(const ReturnStateMachine& before, const Conditions& c,
             const StepOutput& out, const ReturnStateMachine& after) {
    ++steps_;
    const MissionPhase from = before.phase();
    const bool armed_flow = c.armed && from != MissionPhase::FcuLanding &&
                            !(c.fcu_state_fresh && c.mode == "AUTO.LAND");
    const bool link_clean = !c.link.lost && !c.link.degraded;

    // 结构一致性
    require(consistent(out.phase, out.control), "STRUCT phase/control", before, c, out);
    require(!out.follow_active ||
                (out.phase == MissionPhase::Follow && out.control == ControlState::Owned),
            "STRUCT follow_active", before, c, out);
    require(out.record == (out.phase == MissionPhase::Recording ||
                           out.phase == MissionPhase::Exited),
            "STRUCT record", before, c, out);
    require(out.mission_active == isMissionPhase(out.phase), "STRUCT mission_active",
            before, c, out);

    // SC1：在地面不启动返航；任务中落地后不再起飞
    if (armed_flow && (from == MissionPhase::Recording || from == MissionPhase::Exited) &&
        !c.airborne.airborne) {
      require(!isMissionPhase(out.phase) && !out.commands.start_planning,
              "SC1 start while not airborne", before, c, out);
    }
    if (armed_flow && (from == MissionPhase::Hover || from == MissionPhase::Prepare) &&
        !c.airborne.airborne) {
      require(out.phase == MissionPhase::Handover && out.mode_intent == ModeIntent::AutoLand,
              "SC1 landed in hover/prepare", before, c, out);
    }
    if (armed_flow && (from == MissionPhase::Follow || from == MissionPhase::Arrival) &&
        c.airborne.landed_state == LandedState::OnGround) {
      require(out.phase == MissionPhase::Handover && out.mode_intent == ModeIntent::AutoLand,
              "SC1 landed in follow/arrival", before, c, out);
    }
    if (armed_flow && from == MissionPhase::Handover &&
        c.airborne.landed_state == LandedState::OnGround &&
        out.phase == MissionPhase::Handover) {
      require(out.mode_intent == ModeIntent::AutoLand, "SC1 landed in handover",
              before, c, out);
    }
    if (out.phase == MissionPhase::Handover) {
      require(out.setpoint == SetpointKind::None || out.setpoint == SetpointKind::Hold,
              "SC1 handover follows no path", before, c, out);
    }

    // SC2：链路丢失时任务中只处于 OFFBOARD 或 AUTO.LAND；任何时候都不请求 POSCTL（U10）
    if (c.link.lost) {
      require(out.mode_intent != ModeIntent::Posctl, "SC2 POSCTL while link lost",
              before, c, out);
      if (out.phase == MissionPhase::Handover) {
        require(out.mode_intent == ModeIntent::AutoLand, "SC2 handover not direct",
                before, c, out);
      } else if (holdsPhase(out.phase)) {
        // 不申请模式只允许在预发（Acquiring，游走中另查上限）和回退（≤ 3 s）时
        require(out.mode_intent == ModeIntent::Offboard ||
                    out.control == ControlState::Fallback ||
                    out.control == ControlState::Acquiring,
                "SC2 mission intent", before, c, out);
      }
    }

    // SC3：只有有效的飞手意图才能结束任务
    if (isMissionPhase(from) && out.phase == MissionPhase::Exited) {
      std::string rule;
      for (const auto& e : out.events) {
        if (e.type == EventType::State && e.to == "Exited") {
          rule = e.rule;
        }
      }
      if (rule == "5") {
        const bool valid = c.pilot.exit_request && c.pilot.exit_request->link_ok &&
            c.pilot.exit_request->press_start_t > before.mission().start_t;
        require(valid, "SC3 invalid exit request", before, c, out);
      } else {
        require((rule == "4b" || rule == "4c") && link_clean && c.fcu_state_fresh,
                "SC3 exit without clean link", before, c, out);
        if (rule == "4b" && before.control() == ControlState::Owned) {
          require(!c.stream_gap_recent, "SC3 own fallback taken as takeover", before, c, out);
        }
      }
    }

    // SC4：持有或争取 OFFBOARD 时每拍发布；交接、退出在飞控离开 OFFBOARD 之前保持
    if (holdsPhase(out.phase)) {
      require(out.setpoint != SetpointKind::None, "SC4 mission not publishing", before, c, out);
    }
    if ((out.phase == MissionPhase::Handover || out.phase == MissionPhase::Exited) &&
        c.fcu_state_fresh && c.mode == "OFFBOARD") {
      require(out.setpoint != SetpointKind::None, "SC4 left OFFBOARD without hold",
              before, c, out);
    }

    // SC5：任务开始前不发布、不请求模式
    if (out.phase == MissionPhase::Disarmed || out.phase == MissionPhase::RecordDelay ||
        out.phase == MissionPhase::Recording) {
      require(out.setpoint == SetpointKind::None && out.mode_intent == ModeIntent::None,
              "SC5 output before mission", before, c, out);
    }

    // SC6：飞手接管后不再争夺
    if (out.phase == MissionPhase::Exited) {
      require(out.mode_intent == ModeIntent::None || out.mode_intent == ModeIntent::Posctl,
              "SC6 exited requests control", before, c, out);
    }
    if (out.phase == MissionPhase::FcuLanding && !c.link.lost) {
      require(out.mode_intent == ModeIntent::None, "SC6 fcu landing requests mode",
              before, c, out);
    }

    // SC7：保持点来自某一拍的位置，不为原点
    if (out.setpoint == SetpointKind::Hold) {
      require(isRecordedPosition(out.hold_position) && std::isfinite(out.hold_yaw),
              "SC7 hold target", before, c, out);
    }

    // SC10：跟随阶段离路径超过安全距离或投影失败时，不输出路径目标（D30）
    if (out.phase == MissionPhase::Follow &&
        (!c.projection_ok || c.cross_track_m > kSafeDistanceM)) {
      require(out.setpoint == SetpointKind::Hold && !out.follow_active,
              "SC10 path target beyond safe distance", before, c, out);
    }

    // SC11：轨迹在断点处截断时，不因到点、点过近或航点降级失败降落（D30）
    if (c.trail_gap && out.phase == MissionPhase::Handover && from != MissionPhase::Handover) {
      const HandoverReason reason = after.handoverReason();
      require(reason != HandoverReason::Arrived && reason != HandoverReason::PlanTooClose &&
                  reason != HandoverReason::FallbackFailed,
              "SC11 landing at trail gap", before, c, out);
    }

    // SC9：只在返航完成、已落地、起点附近或飞控已在降落时请求 AUTO.LAND
    if (out.mode_intent == ModeIntent::AutoLand) {
      require((out.phase == MissionPhase::Handover &&
               after.handoverReason() != HandoverReason::None) ||
                  (out.phase == MissionPhase::FcuLanding && c.link.lost),
              "SC9 AUTO.LAND without reason", before, c, out);
    }
  }

  long steps() const { return steps_; }
  const std::map<std::string, long>& counts() const { return counts_; }
  const std::vector<std::string>& examples() const { return examples_; }

private:
  void require(bool ok, const char* id, const ReturnStateMachine& before,
               const Conditions& c, const StepOutput& out) {
    if (ok) {
      return;
    }
    ++counts_[id];
    if (examples_.size() < 20) {
      std::ostringstream text;
      text << id << " | before " << reportedStateName(before.phase(), before.follow())
           << "/" << controlName(before.control())
           << " | armed=" << c.armed << " fresh=" << c.fcu_state_fresh
           << " mode=" << c.mode << " link=" << linkText(c.link)
           << " airborne=" << c.airborne.airborne
           << " on_ground=" << (c.airborne.landed_state == LandedState::OnGround)
           << " gap=" << c.stream_gap_recent << " points=" << c.has_min_points
           << " | after " << reportedStateName(out.phase, out.follow) << "/"
           << controlName(out.control) << " intent=" << modeIntentName(out.mode_intent)
           << " setpoint=" << static_cast<int>(out.setpoint);
      examples_.push_back(text.str());
    }
  }

  long steps_{0};
  std::map<std::string, long> counts_;
  std::vector<std::string> examples_;
};

/// 随机世界：大体按状态机的意图切换模式，其余条件随机扰动
class RandomWorld {
public:
  explicit RandomWorld(unsigned seed) : rng_(seed) {
    c_.fcu_state_fresh = true;
    c_.mode = "POSCTL";
    c_.airborne.airborne = true;
    c_.airborne.landed_state = LandedState::InAir;
    c_.odom_stale = false;
    c_.has_min_points = true;
    c_.projection_ok = true;
  }

  /// 生成下一拍的条件；last 为上一拍的输出，start_t 为当前任务的开始时间
  const Conditions& next(const StepOutput& last, double mission_start_t) {
    const double r = uniform();
    now_ += r < 0.85 ? kDt : r < 0.93 ? 0.5 : r < 0.98 ? 3.0 : 16.0;
    c_.now = now_;
    c_.position = positionFor(++index_);
    if (chance(0.1)) c_.yaw = uniform() * 6.0 - 3.0;

    if (!c_.armed) {
      c_.armed = chance(0.3);
    } else if (chance(0.003)) {
      c_.armed = false;
    }
    c_.fcu_state_fresh = !chance(0.03);
    if (c_.fcu_state_fresh) {
      c_.fcu_state_t = now_;  // 新鲜时本拍收到了飞控状态
    }

    // 飞控大多接受模式请求；偶尔被飞手或其他节点切换
    if (last.mode_intent != ModeIntent::None && chance(0.6)) {
      c_.mode = modeIntentName(last.mode_intent);
    } else if (chance(0.03)) {
      const double m = uniform();
      c_.mode = m < 0.4 ? "OFFBOARD" : m < 0.7 ? "POSCTL" : m < 0.99 ? "ALTCTL" : "AUTO.LAND";
    }

    if (chance(0.03)) {
      const double l = uniform();
      c_.link = LinkStatus{};
      c_.link.degraded = l >= 0.5;
      c_.link.lost = l >= 0.75;
      c_.link.source = c_.link.lost ? LinkLossSource::Ch18 : LinkLossSource::None;
    }
    if (chance(0.01)) c_.battery.low = !c_.battery.low;
    if (chance(0.01)) c_.airborne.airborne = !c_.airborne.airborne;
    c_.airborne.landed_state = (!c_.airborne.airborne && chance(0.5))
        ? LandedState::OnGround : LandedState::InAir;
    c_.has_min_points = !chance(0.2);

    if (chance(0.05)) {
      c_.plan_status = static_cast<PlanStatus>(rng_() % 7);
    }
    c_.path_start_yaw = chance(0.7) ? std::optional<double>(c_.yaw + (chance(0.8) ? 0.0 : 1.0))
                                    : std::nullopt;
    c_.path_target_yaw = chance(0.7) ? std::optional<double>(c_.yaw) : std::nullopt;
    c_.deviation_confirmed = chance(0.05);
    c_.rejoin_confirmed = chance(0.1);
    c_.finished = chance(0.01);
    c_.projection_ok = !chance(0.1);
    c_.cross_track_m = uniform() * 2.0;
    c_.trail_gap = chance(0.05);
    c_.stream_gap_recent = chance(0.05);
    c_.odom_stale = chance(0.02);
    c_.mode_request_failed = chance(0.02);

    // CH7 指令：与 PilotInput 一样保留到下一条
    if (chance(0.01)) {
      c_.pilot.return_request =
          Ch7Command{++press_id_, now_ - 2.0, now_, !c_.link.lost};
      c_.pilot.press_id = press_id_;
    }
    if (chance(0.01)) {
      const double start = chance(0.5) ? now_ - 0.5 : mission_start_t - 1.0;
      c_.pilot.exit_request =
          Ch7Command{++press_id_, start, now_, !c_.link.lost && !c_.link.degraded};
      c_.pilot.press_id = press_id_;
    }
    c_.pilot.pressed = chance(0.2);
    return c_;
  }

private:
  double uniform() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_); }
  bool chance(double p) { return uniform() < p; }

  std::mt19937 rng_;
  Conditions c_;
  double now_{0.0};
  long index_{0};
  int press_id_{0};
};

struct Snapshot {
  ReturnStateMachine sm;
  Conditions last;
};

using Signature = std::tuple<int, int, int, bool, bool, bool, int, uint64_t>;

Signature signatureOf(const ReturnStateMachine& sm, const StepOutput& out) {
  // 区域状态、任务标志和与判定相关的告警，作为快照去重的依据
  const uint64_t alert_mask =
      out.alerts.bits() &
      ((uint64_t{1} << static_cast<unsigned>(Alert::ReturnPending)) |
       (uint64_t{1} << static_cast<unsigned>(Alert::ExitRejectedLink)) |
       (uint64_t{1} << static_cast<unsigned>(Alert::PathOffset)) |
       (uint64_t{1} << static_cast<unsigned>(Alert::HandoverDirectLand)) |
       (uint64_t{1} << static_cast<unsigned>(Alert::MissionLanded)));
  return Signature(static_cast<int>(sm.phase()), static_cast<int>(sm.control()),
                   static_cast<int>(sm.follow()), sm.mission().active,
                   sm.mission().offboard_confirmed, sm.handoverDirect(),
                   static_cast<int>(out.mode_intent), alert_mask);
}

struct WalkResult {
  std::vector<Snapshot> snapshots;
  std::set<int> phases;
  std::set<int> controls;
  long max_preframe_ticks{0};
};

WalkResult randomWalks(InvariantChecker& checker, int walks, int steps) {
  WalkResult result;
  std::map<Signature, int> per_signature;
  const auto config = makeConfig();

  for (int w = 0; w < walks; ++w) {
    RandomWorld world(1000u + static_cast<unsigned>(w));
    ReturnStateMachine sm(config);
    StepOutput last;
    long preframe_ticks = 0;
    int preframe_mission = -1;

    for (int i = 0; i < steps; ++i) {
      const Conditions& c = world.next(last, sm.mission().start_t);
      const ReturnStateMachine before = sm;
      last = sm.step(c);
      checker.check(before, c, last, sm);

      // SC2：预发期间不申请模式的拍数有上限
      const bool preframe = c.link.lost && last.control == ControlState::Acquiring &&
                            last.mode_intent == ModeIntent::None;
      if (preframe && preframe_mission == last.mission_id) {
        ++preframe_ticks;
      } else {
        preframe_ticks = preframe ? 1 : 0;
      }
      preframe_mission = last.mission_id;
      result.max_preframe_ticks = std::max(result.max_preframe_ticks, preframe_ticks);

      result.phases.insert(static_cast<int>(sm.phase()));
      result.controls.insert(static_cast<int>(sm.control()));
      int& count = per_signature[signatureOf(sm, last)];
      if (count < 3) {
        ++count;
        result.snapshots.push_back(Snapshot{sm, c});
      }
    }
  }
  return result;
}

TEST(ReturnStateMachineInvariantTest, RandomWalksReachAllRegionsWithoutViolations) {
  InvariantChecker checker;
  const WalkResult result = randomWalks(checker, 300, 400);

  // 覆盖：区域 A 的每个阶段、区域 B 的每个值都要到达，穷举才有意义
  EXPECT_EQ(result.phases.size(), 10u);
  EXPECT_EQ(result.controls.size(), 8u);
  EXPECT_LE(result.max_preframe_ticks, makeConfig().pre_offboard_setpoint_count);

  for (const auto& [id, count] : checker.counts()) {
    ADD_FAILURE() << id << ": " << count << " 次";
  }
  for (const auto& example : checker.examples()) {
    ADD_FAILURE() << example;
  }
  EXPECT_GT(checker.steps(), 100000);
}

TEST(ReturnStateMachineInvariantTest, ExhaustiveSingleStepFromReachableStates) {
  InvariantChecker walk_checker;
  const WalkResult result = randomWalks(walk_checker, 300, 400);
  ASSERT_GT(result.snapshots.size(), 100u);

  InvariantChecker checker;
  std::mt19937 rng(7);
  auto chance = [&rng](double p) {
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < p;
  };
  int press_id = 1000000;
  long index = 10000000;

  for (const Snapshot& snapshot : result.snapshots) {
    for (int armed = 0; armed < 2; ++armed)
    for (int fresh = 0; fresh < 2; ++fresh)
    for (const char* mode : kModes)
    for (int link = 0; link < 3; ++link)
    for (int ground = 0; ground < 3; ++ground)
    for (int ret = 0; ret < 2; ++ret)
    for (int exit = 0; exit < 3; ++exit)
    for (int gap = 0; gap < 2; ++gap)
    for (int points = 0; points < 2; ++points) {
      Conditions c = snapshot.last;
      c.now = snapshot.last.now + kDt;
      c.position = positionFor(++index);
      c.armed = armed == 1;
      c.fcu_state_fresh = fresh == 1;
      if (c.fcu_state_fresh) {
        c.fcu_state_t = c.now;
      }
      c.mode = mode;
      c.link = LinkStatus{};
      c.link.degraded = link >= 1;
      c.link.lost = link == 2;
      c.airborne.airborne = ground == 0;
      c.airborne.landed_state = ground == 2 ? LandedState::OnGround : LandedState::InAir;
      c.stream_gap_recent = gap == 1;
      c.has_min_points = points == 1;
      const bool clean = link == 0;
      if (ret == 1) {
        c.pilot.return_request = Ch7Command{++press_id, c.now - 2.0, c.now, link != 2};
      }
      if (exit == 1) {
        c.pilot.exit_request = Ch7Command{++press_id, c.now - kDt, c.now, clean};
      } else if (exit == 2) {
        c.pilot.exit_request = Ch7Command{
            ++press_id, snapshot.sm.mission().start_t - 1.0, c.now, clean};
      }
      // 其余事实随机取值
      c.plan_status = static_cast<PlanStatus>(rng() % 7);
      c.path_start_yaw = chance(0.5) ? std::optional<double>(c.yaw) : std::nullopt;
      c.path_target_yaw = chance(0.5) ? std::optional<double>(c.yaw) : std::nullopt;
      c.deviation_confirmed = chance(0.3);
      c.rejoin_confirmed = chance(0.3);
      c.finished = chance(0.2);
      c.projection_ok = chance(0.8);
      c.cross_track_m = chance(0.5) ? 0.2 : 1.5;
      c.trail_gap = chance(0.3);
      c.odom_stale = chance(0.1);
      c.battery.low = chance(0.3);

      ReturnStateMachine sm = snapshot.sm;
      const StepOutput out = sm.step(c);
      checker.check(snapshot.sm, c, out, sm);
    }
  }

  for (const auto& [id, count] : checker.counts()) {
    ADD_FAILURE() << id << ": " << count << " 次";
  }
  for (const auto& example : checker.examples()) {
    ADD_FAILURE() << example;
  }
  RecordProperty("snapshots", static_cast<int>(result.snapshots.size()));
  RecordProperty("steps", static_cast<int>(checker.steps()));
  std::cout << "快照 " << result.snapshots.size() << " 个，单拍检查 "
            << checker.steps() << " 次" << std::endl;
}

}  // namespace
}  // namespace px4_backtrack::domain
