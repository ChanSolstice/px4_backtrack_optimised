// 闭环危害场景：用 PX4 行为模型（假飞控）驱动真实的 BacktrackUseCase，逐场景检查安全约束。
// - 飞控规则逐条注明 PX4 v1.15.4 源码出处，参数取 v1.15.4 默认值；
// - 接收机失控时（用户确认）：持续输出帧，摇杆回中，开关和 CH7 保持，CH18 = 800；
// - 控制循环的阻塞按真实耗时计入仿真时间：节点是单线程 ros::spin，tick 阻塞期间不发 setpoint、不处理回调；
// - 模式请求由 UseCase 内的 ModeCommander 决定何时发出，服务调用按异步处理（结果随后送回）。
// 用法：hazard_scenarios <trajectory_config.yaml> [场景编号 ...]，例如 H01 H06a；不带编号运行全部场景
#include "px4_backtrack/application/backtrack_usecase.hpp"

#include <ros/console.h>
#include <ros/time.h>
#include <trajectory_core/trajectory_core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace {

using px4_backtrack::application::BacktrackUseCase;
using px4_backtrack::application::TickOutput;
using px4_backtrack::application::UseCaseConfig;
using px4_backtrack::domain::LandedState;
using px4_backtrack::domain::SetpointTarget;
using px4_backtrack::domain::Vec3;

// ---- PX4 v1.15.4 默认参数 ----
constexpr double kComOfLossT = 1.0;      // COM_OF_LOSS_T（commander_params.c）
constexpr double kComRcStickOv = 0.30;   // COM_RC_STICK_OV = 30 %，按归一化摇杆量比较（ManualControl.cpp）
constexpr double kLndmcTrigTime = 1.0;   // LNDMC_TRIG_TIME
constexpr double kComDisarmLand = 2.0;   // COM_DISARM_LAND
constexpr double kMpcLandSpeed = 0.7;    // MPC_LAND_SPEED
constexpr double kLndmcZVelMax = 0.5;    // LNDMC_Z_VEL_MAX：地面接触要求指令下降速度 >= 1.1 倍此值

// ---- 话题频率与调度 ----
constexpr double kStatePeriod = 1.0;        // /mavros/state 由 1 Hz HEARTBEAT 驱动（mavlink_main.cpp）
constexpr double kExtStatePeriod = 0.2;     // onboard 模式 EXTENDED_SYS_STATE 5 Hz（mavlink_main.cpp）
constexpr double kRcPeriod = 0.05;          // onboard 模式 RC_CHANNELS 20 Hz（mavlink_main.cpp）
constexpr double kOdomPeriod = 1.0 / 30.0;  // SLAM 里程计
constexpr double kBatteryPeriod = 1.0;
constexpr double kTickPeriod = 0.05;        // publish_hz = 20
constexpr double kModeCmdLatency = 0.05;
constexpr double kSimDt = 0.01;
constexpr double kTimeOffset = 1000.0;      // ros::Time 不能为 0

constexpr uint16_t kCh7Released = 1900;
constexpr uint16_t kCh7Pressed = 1100;      // 按下 = CH7 < ch7_pressed_threshold(1500)
constexpr uint16_t kCh18Ok = 1400;          // 改动后的固件：正常 1400，失控 800
constexpr uint16_t kCh18Lost = 800;

bool isMission(const std::string& s) {
  return s == "Ch18HoverWait" || s == "ReturnPreOffboard" || s == "ReturnActive" ||
         s == "ReturnRejoin" || s == "LandingHover" || s == "LandingTransition";
}

std::string fmt(const char* f, double a) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), f, a);
  return buf;
}

// 飞手的操作；链路丢失时飞控看到的是接收机的失控输出，而不是这里的值
struct Pilot {
  Vec3 vel_cmd = Vec3::Zero();   // 位置模式下摇杆给出的速度（抽象）
  double throttle = 0.0;         // 油门杆 [-1, 1]，0 为中位
  bool ch7_pressed = false;
  std::string mode_switch = "POSCTL";
};

// ---- 假飞控：PX4 v1.15.4 的相关行为 ----
class FakeFcu {
 public:
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  double yaw = 0.0;
  bool armed = false;
  bool landed = true;
  std::string intention = "POSCTL";  // 用户意图模式
  std::string nav = "POSCTL";        // 实际 nav_state
  int offboard_fallbacks = 0;
  int stick_overrides = 0;
  std::vector<std::pair<double, std::string>> events;

  void arm(double t) {
    armed = true;
    landed = true;
    log(t, "解锁");
  }

  // offboardCheck.cpp：最近一次 setpoint 在 COM_OF_LOSS_T 内才算 OFFBOARD 信号可用
  bool offboardAvailable(double t) const { return has_sp_ && t - last_sp_t_ < kComOfLossT; }

  void onSetpoint(double t, const SetpointTarget& sp) {
    sp_ = sp;
    has_sp_ = true;
    last_sp_t_ = t;
  }

  void onModeRequest(double t, const std::string& mode) { pending_.push_back({t + kModeCmdLatency, mode}); }

  // 外部改模式（地面站等）
  void setIntention(double t, const std::string& mode, const std::string& why) {
    if (intention != mode) {
      log(t, "用户意图 " + intention + " → " + mode + "（" + why + "）");
    }
    intention = mode;
  }

  // 每个 RC 帧：开关按变化动作（ManualControl.cpp processSwitches）；
  // 自动模式下摇杆变化超过 COM_RC_STICK_OV 切 POSCTL（Commander.cpp，COM_RC_OVERRIDE 第 0 位，默认开）
  void onRcFrame(double t, double throttle, const std::string& sw) {
    if (!rc_init_) {
      rc_init_ = true;
      rc_throttle_ = throttle;
      rc_switch_ = sw;
      return;
    }
    if (armed && sw != rc_switch_) {
      setIntention(t, sw, "遥控模式开关变化");
    }
    if (armed && nav.rfind("AUTO.", 0) == 0 && std::fabs(throttle - rc_throttle_) > kComRcStickOv) {
      setIntention(t, "POSCTL", "摇杆超控");
      ++stick_overrides;
    }
    rc_throttle_ = throttle;
    rc_switch_ = sw;
  }

  void step(double t, double dt, const Pilot& pilot, bool link_ok) {
    applyModeCommands(t);
    updateNavState(t);
    if (!armed) {
      vel = Vec3::Zero();
      return;
    }

    // 失控时接收机输出回中摇杆
    const Vec3 stick_vel = link_ok ? pilot.vel_cmd : Vec3::Zero();
    Vec3 v_des = Vec3::Zero();
    if (nav == "OFFBOARD") {
      v_des = sp_.velocity + (sp_.position - pos);
      limit(v_des, 3.0, 1.5);
      const double dyaw = std::remainder(sp_.yaw - yaw, 2.0 * M_PI);
      yaw += std::clamp(dyaw, -1.0 * dt, 1.0 * dt);
    } else if (nav == "AUTO.LAND") {
      v_des = Vec3(0.0, 0.0, -kMpcLandSpeed);
    } else {
      v_des = stick_vel;  // POSCTL/ALTCTL：摇杆回中即刹停悬停
    }

    // mc_pos_control 起飞处理：已落地时，只有爬升指令才会离地
    const bool climb_demand = v_des.z() > 0.3;
    if (landed && !climb_demand) {
      v_des = Vec3::Zero();
    }
    if (landed && climb_demand) {
      landed = false;
      ground_contact_since_ = -1.0;
      landed_since_ = -1.0;
      log(t, "离地");
    }

    vel += (v_des - vel) * std::min(1.0, dt / 0.3);
    pos += vel * dt;
    if (pos.z() <= 0.0) {
      pos.z() = 0.0;
      vel.z() = std::max(0.0, vel.z());
    }

    // MulticopterLandDetector.cpp：贴地、低推力；爬升率控制模式下还要求"指令下降"（>= 1.1 * LNDMC_Z_VEL_MAX）
    const bool on_ground = pos.z() <= 0.02 && vel.head<2>().norm() < 0.5;
    const bool commanded_descent = (nav == "AUTO.LAND") || (v_des.z() <= -1.1 * kLndmcZVelMax);
    if (!landed) {
      if (on_ground && commanded_descent) {
        if (ground_contact_since_ < 0.0) ground_contact_since_ = t;
        if (t - ground_contact_since_ >= kLndmcTrigTime) {
          landed = true;
          landed_since_ = t;
          log(t, "飞控判定落地");
        }
      } else {
        ground_contact_since_ = -1.0;
      }
    }
    // Commander 自动上锁：落地持续 COM_DISARM_LAND
    if (landed && landed_since_ >= 0.0 && t - landed_since_ >= kComDisarmLand) {
      armed = false;
      log(t, "落地自动上锁");
    }
  }

 private:
  struct Cmd {
    double t;
    std::string mode;
  };
  std::vector<Cmd> pending_;
  SetpointTarget sp_{};
  bool has_sp_ = false;
  double last_sp_t_ = -1e9;
  bool rc_init_ = false;
  double rc_throttle_ = 0.0;
  std::string rc_switch_;
  double ground_contact_since_ = -1.0;
  double landed_since_ = -1.0;
  bool in_fallback_ = false;

  void log(double t, const std::string& s) { events.push_back({t, s}); }

  static void limit(Vec3& v, double xy_max, double z_max) {
    const double xy = v.head<2>().norm();
    if (xy > xy_max) v.head<2>() *= xy_max / xy;
    v.z() = std::clamp(v.z(), -z_max, z_max);
  }

  void applyModeCommands(double t) {
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->t > t) {
        ++it;
        continue;
      }
      if (armed) {
        if (it->mode == "OFFBOARD" && !offboardAvailable(t)) {
          log(t, "拒绝 OFFBOARD：没有 setpoint 流");
        } else {
          setIntention(t, it->mode, "机载请求");
        }
      }
      it = pending_.erase(it);
    }
  }

  // Commander.cpp：nav_state = modeFromAction(selectedAction, user_intention)。
  // failsafe.cpp checkModeFallback：OFFBOARD 信号丢失时回退 POSCTL（COM_OBL_RC_ACT=0），不改用户意图，
  // 所以 setpoint 恢复后自动回到 OFFBOARD。
  void updateNavState(double t) {
    std::string next = intention;
    if (intention == "OFFBOARD" && !offboardAvailable(t)) next = "POSCTL";
    if (next == nav) return;
    if (intention == "OFFBOARD" && next == "POSCTL") {
      ++offboard_fallbacks;
      in_fallback_ = true;
      log(t, "OFFBOARD 信号丢失，回退 POSCTL");
    } else if (in_fallback_ && next == "OFFBOARD") {
      log(t, "setpoint 恢复，自动回到 OFFBOARD");
    } else {
      log(t, "模式 " + nav + " → " + next);
    }
    if (next != "POSCTL" || intention != "OFFBOARD") in_fallback_ = false;
    nav = next;
  }
};

// ---- 场景运行环境 ----
struct World {
  FakeFcu fcu;
  Pilot pilot;
  bool link_ok = true;
  double battery = 0.90;
  bool inject_ch7_dip = false;  // 下一帧注入一次 CH7 按下值
  bool pilot_took_over = false; // 飞手已在链路正常时拨开关接管
  std::string held_switch = "POSCTL";
  uint16_t held_ch7 = kCh7Released;
  bool prev_link_ok = true;
  std::string module_state = "IdleDisarmed";  // 供场景脚本按模块状态触发动作
  double prepare_seen_t = -1.0;              // 首次进入 ReturnPreOffboard 的时间
  double stall_at = -1.0;                    // 注入一次控制循环卡顿（例如 CPU 被抢占）
  double stall_s = 0.0;
};

struct Metrics {
  std::vector<std::pair<double, std::string>> states;
  std::string last_state = "IdleDisarmed";
  double mission_start_t = -1.0;
  double mission_start_z = 0.0;
  bool mission_started_on_ground = false;
  Vec3 trail_start = Vec3::Zero();
  bool exited = false;                  // 进入了 Exited
  double posctl_lost_since = -1.0;
  double max_posctl_while_lost = 0.0;   // 任务开始后、链路丢失时，在 POSCTL 下的最长连续时间
  double touchdown_t = -1.0;
  double max_z_after_touchdown = 0.0;
  bool was_high = false;
  int fight_requests = 0;               // 飞手接管后模块仍发出的 OFFBOARD/AUTO.LAND 请求
  int mode_requests_sent = 0;
  double last_sp_t = -1.0;
  double max_sp_gap_in_offboard = 0.0;
  double max_tick_ms = 0.0;
};

struct Check {
  std::string name;
  bool pass;
  std::string detail;
};

struct Scenario {
  std::string id;
  std::string title;
  double duration;
  std::function<void(double, World&)> script;
  std::function<std::vector<Check>(const World&, const Metrics&)> checks;
};

UseCaseConfig productionConfig(const std::string& traj_cfg) {
  UseCaseConfig c;  // 取值与 config/backtrack_params.yaml 一致
  c.record_max_hz = 15.0;
  c.record_min_points = 30;
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
  c.battery_config.low_battery_confirm_s = 3.0;
  c.airborne_config.airborne_min_height_m = 0.5;
  c.airborne_config.fcu_state_stale_s = 3.0;
  c.odom_stale_s = 0.25;
  c.odom_timeout_s = 2.0;
  c.fcu_state_stale_s = 3.0;
  auto& sm = c.state_machine_config;
  sm.record_delay_s = 15.0;
  sm.ch18_hover_wait_s = 15.0;
  sm.landing_hover_s = 5.0;
  sm.pre_offboard_setpoint_count = 6;
  sm.return_pending_max_s = 3.0;
  sm.fallback_timeout_s = 3.0;
  sm.handover_posctl_timeout_s = 3.0;
  sm.rejoin_trigger_dist_m = 0.5;
  sm.reclaim_max_offset_m = 1.0;
  c.mode_commander_config.request_interval_s = 1.0;
  c.mode_commander_config.request_timeout_s = 3.0;
  c.fallback_gap_s = 0.5;
  c.fallback_window_s = 3.0;
  auto& r = c.rejoin_config;
  r.trigger_dist_m = 0.5;
  r.success_dist_m = 0.4;
  r.v_max_m_s = 0.3;
  r.lookahead_time_s = 0.5;
  r.trigger_hold_s = 0.25;
  r.success_hold_s = 0.10;
  c.trajectory_config = trajectory_core::TrajectoryConfig::loadFromYaml(traj_cfg);
  c.debug_trajectory_sample_dt = 0.1;
  return c;
}

// 标准出航：1 s 后起飞到 3 m，再按东、北、东三段各约 20、15、20 m 飞行，59 s 后悬停
void outbound(double t, World& w) {
  Pilot p = w.pilot;
  p.vel_cmd = Vec3::Zero();
  p.throttle = 0.0;
  if (t >= 1.0 && t < 4.0) {
    p.vel_cmd = Vec3(0, 0, 1.0);
    p.throttle = 0.6;
  } else if (t >= 4.0 && t < 24.0) {
    p.vel_cmd = Vec3(1.0, 0, 0);
  } else if (t >= 24.0 && t < 39.0) {
    p.vel_cmd = Vec3(0, 1.0, 0);
  } else if (t >= 39.0 && t < 59.0) {
    p.vel_cmd = Vec3(1.0, 0, 0);
  }
  w.pilot.vel_cmd = p.vel_cmd;
  w.pilot.throttle = p.throttle;
}

// 长时间出航：每 5 s 换一次方向，用于制造长录制
void longOutbound(double t, World& w, double minutes) {
  w.pilot.vel_cmd = Vec3::Zero();
  w.pilot.throttle = 0.0;
  if (t >= 1.0 && t < 4.0) {
    w.pilot.vel_cmd = Vec3(0, 0, 1.0);
    w.pilot.throttle = 0.6;
  } else if (t >= 4.0 && t < 4.0 + minutes * 60.0) {
    const int leg = static_cast<int>((t - 4.0) / 5.0);
    const double heading = 0.9 * std::sin(0.7 * leg) + 0.5 * leg;  // 确定性的折线
    w.pilot.vel_cmd = Vec3(0.8 * std::cos(heading), 0.8 * std::sin(heading), 0.0);
  }
}

bool landedNearTrailStart(const World& w, const Metrics& m, std::string& detail) {
  const double d = (w.fcu.pos - m.trail_start).head<2>().norm();
  detail = std::string(w.fcu.armed ? "未上锁" : "已上锁") + "，距录制起点 " + fmt("%.1f m", d);
  return !w.fcu.armed && d < 3.0;
}

Check noPausedCheck(const Metrics& m) {
  return {"任务不被非预期结束", !m.exited, m.exited ? "进入了 Exited（飞手没有发出退出指令）" : "未退出"};
}

Check returnPersistsCheck(const Metrics& m) {
  return {"链路丢失期间返航不中断", m.max_posctl_while_lost < 3.0,
          fmt("任务中链路丢失时最长连续处于 POSCTL %.1f s（阈值 3 s）", m.max_posctl_while_lost)};
}

// ---- 场景定义 ----
std::vector<Scenario> buildScenarios() {
  std::vector<Scenario> s;

  s.push_back({"H01", "录制中单独失控（基线）", 260.0,
               [](double t, World& w) {
                 outbound(t, w);
                 w.link_ok = !(t >= 62.0);
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 return std::vector<Check>{{"返航完成并在录制起点上锁", landedNearTrailStart(w, m, d), d},
                                           returnPersistsCheck(m), noPausedCheck(m)};
               }});

  s.push_back({"H02", "低电量返航中失控，CH7 无跳变", 260.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 w.link_ok = !(t >= 80.0);
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 return std::vector<Check>{{"返航完成并在录制起点上锁", landedNearTrailStart(w, m, d), d},
                                           returnPersistsCheck(m), noPausedCheck(m)};
               }});

  s.push_back({"H03", "低电量返航中失控，失控瞬间 CH7 跳变一帧", 260.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 w.link_ok = !(t >= 80.0);
                 if (std::fabs(t - 80.0) < 0.005) w.inject_ch7_dip = true;
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 return std::vector<Check>{{"返航完成并在录制起点上锁", landedNearTrailStart(w, m, d), d},
                                           returnPersistsCheck(m), noPausedCheck(m)};
               }});

  s.push_back({"H04", "飞手按着 CH7 时低电量抢先触发", 200.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 w.pilot.ch7_pressed = (t >= 62.5 && t < 63.8);
               },
               [](const World&, const Metrics& m) { return std::vector<Check>{noPausedCheck(m)}; }});

  s.push_back({"H05", "录制延时内低电量", 150.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 8.0) w.battery = 0.10;
               },
               [](const World&, const Metrics& m) {
                 return std::vector<Check>{{"进入 Recording 后执行低电量返航", m.mission_start_t > 0.0,
                                            m.mission_start_t > 0.0 ? fmt("%.1f s 开始返航", m.mission_start_t)
                                                                    : "整个飞行都没有返航"}};
               }});

  // 降落：59 s 后以 0.7 m/s 下降，约 63.3 s 触地
  auto landing_pilot = [](double t, World& w) {
    outbound(t, w);
    if (t >= 59.0) {
      w.pilot.vel_cmd = Vec3(0, 0, -0.7);
      w.pilot.throttle = -0.5;
    }
  };
  s.push_back({"H06a", "手动降落触地瞬间关遥控器（飞控尚未判定落地）", 140.0,
               [landing_pilot](double t, World& w) {
                 landing_pilot(t, w);
                 if (w.fcu.pos.z() <= 0.02 && w.fcu.armed && t > 30.0 && w.link_ok) w.link_ok = false;
               },
               [](const World&, const Metrics& m) {
                 return std::vector<Check>{
                     {"落地后不起飞", m.max_z_after_touchdown < 0.5,
                      fmt("触地后最大高度 %.2f m", m.max_z_after_touchdown)},
                     {"在地面不启动返航", !m.mission_started_on_ground,
                      m.mission_started_on_ground ? fmt("%.1f s 在地面启动了返航", m.mission_start_t) : "未在地面启动"}};
               }});

  s.push_back({"H06b", "飞控判定落地后、自动上锁前关遥控器", 140.0,
               [landing_pilot](double t, World& w) {
                 landing_pilot(t, w);
                 static double touch = -1.0;
                 if (t < 1.0) touch = -1.0;
                 if (w.fcu.pos.z() <= 0.02 && t > 30.0 && touch < 0.0) touch = t;
                 if (touch > 0.0 && t >= touch + 1.5) w.link_ok = false;
               },
               [](const World&, const Metrics& m) {
                 return std::vector<Check>{
                     {"落地后不起飞", m.max_z_after_touchdown < 0.5,
                      fmt("触地后最大高度 %.2f m", m.max_z_after_touchdown)},
                     {"在地面不启动返航", !m.mission_started_on_ground,
                      m.mission_started_on_ground ? fmt("%.1f s 在地面启动了返航", m.mission_start_t) : "未在地面启动"}};
               }});

  s.push_back({"H07", "30 分钟录制后失控（同步规划阻塞）", 1890.0,
               [](double t, World& w) {
                 longOutbound(t, w, 30.0);
                 w.link_ok = !(t >= 1810.0);
               },
               [](const World& w, const Metrics& m) {
                 return std::vector<Check>{
                     {"OFFBOARD 不因断流回退", w.fcu.offboard_fallbacks == 0,
                      fmt("回退 %.0f 次", w.fcu.offboard_fallbacks) +
                          fmt("，OFFBOARD 中 setpoint 最大间隔 %.2f s", m.max_sp_gap_in_offboard) +
                          fmt("，最长 tick %.0f ms", m.max_tick_ms)}};
               }});

  s.push_back({"H08", "低电量返航中失控，随后被外部切出 OFFBOARD", 200.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 w.link_ok = !(t >= 80.0);
                 if (std::fabs(t - 85.0) < 0.005) w.fcu.setIntention(t, "POSCTL", "地面站切模式");
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 return std::vector<Check>{returnPersistsCheck(m),
                                           {"返航完成并在录制起点上锁", landedNearTrailStart(w, m, d), d}};
               }});

  s.push_back({"H09", "到点悬停时飞手拨开关接管（链路正常）", 200.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 // 到点悬停阶段，飞手把模式开关拨到 ALTCTL
                 if (!w.pilot_took_over && w.module_state == "LandingHover") {
                   w.pilot.mode_switch = "ALTCTL";
                   w.pilot_took_over = true;
                 }
               },
               [](const World&, const Metrics& m) {
                 return std::vector<Check>{{"飞手接管后模块不再抢模式", m.fight_requests == 0,
                                            fmt("接管后模块又发出 %.0f 次 OFFBOARD/AUTO.LAND 请求", m.fight_requests)}};
               }});

  s.push_back({"H10", "AUTO.LAND 降落中失控，油门杆原本拉到底", 220.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 if (t >= 70.0) w.pilot.throttle = -1.0;  // OFFBOARD 中油门杆不起作用
                 static bool lost = false;
                 if (t < 1.0) lost = false;
                 if (!lost && w.fcu.nav == "AUTO.LAND" && w.fcu.pos.z() < 2.5) {
                   w.link_ok = false;
                   lost = true;
                 } else if (!lost) {
                   w.link_ok = true;
                 }
               },
               [](const World& w, const Metrics&) {
                 return std::vector<Check>{{"降落完成并上锁", !w.fcu.armed,
                                            std::string(w.fcu.armed ? "未上锁" : "已上锁") + "，最终模式 " + w.fcu.nav +
                                                fmt("，高度 %.1f m", w.fcu.pos.z()) +
                                                fmt("，摇杆超控 %.0f 次", w.fcu.stick_overrides)}};
               }});

  s.push_back({"H11", "电量读数无效（-0.01）", 120.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = -0.01;
               },
               [](const World&, const Metrics& m) {
                 return std::vector<Check>{{"无效电量不触发返航", m.mission_start_t < 0.0,
                                            m.mission_start_t < 0.0 ? "未触发" : fmt("%.1f s 触发了返航", m.mission_start_t)}};
               }});

  s.push_back({"H12", "返航中 CH7 单帧跳变（链路正常）", 200.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 if (std::fabs(t - 85.0) < 0.005) w.inject_ch7_dip = true;
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 return std::vector<Check>{noPausedCheck(m), {"返航完成并在录制起点上锁", landedNearTrailStart(w, m, d), d}};
               }});
  // M1：自身卡顿导致飞控回退 POSCTL，不能当作飞手接管
  s.push_back({"H13", "返航中控制循环卡顿 1.3 s（链路正常）", 200.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 if (std::fabs(t - 85.0) < 0.005) {
                   w.stall_at = t;
                   w.stall_s = 1.3;
                 }
               },
               [](const World& w, const Metrics& m) {
                 std::string d;
                 const bool landed = landedNearTrailStart(w, m, d);
                 return std::vector<Check>{
                     noPausedCheck(m),
                     {"返航完成并在录制起点上锁", landed, d + fmt("，飞控回退 %.0f 次", w.fcu.offboard_fallbacks)}};
               }});

  // M4：OFFBOARD 确认之前，飞手在链路正常时拨开关
  s.push_back({"H14", "返航刚开始、OFFBOARD 确认前飞手拨开关到 ALTCTL（链路正常）", 150.0,
               [](double t, World& w) {
                 outbound(t, w);
                 if (t >= 60.0) w.battery = 0.10;
                 if (!w.pilot_took_over && w.prepare_seen_t > 0.0 && t >= w.prepare_seen_t + 0.2) {
                   w.pilot.mode_switch = "ALTCTL";
                   w.pilot_took_over = true;
                 }
               },
               [](const World& w, const Metrics& m) {
                 return std::vector<Check>{{"飞手接管后模块不再抢模式", m.fight_requests == 0,
                                            fmt("接管后模块又发出 %.0f 次 OFFBOARD/AUTO.LAND 请求", m.fight_requests) +
                                                "，最终模式 " + w.fcu.nav}};
               }});
  return s;
}

void runScenario(const Scenario& sc, const std::string& traj_cfg, std::vector<std::pair<std::string, bool>>& summary) {
  World w;
  Metrics m;
  BacktrackUseCase uc(productionConfig(traj_cfg));

  auto setNow = [](double t) { ros::Time::setNow(ros::Time(kTimeOffset + t)); };
  double next_tick = 0.0, next_rc = 0.0, next_odom = 0.0, next_state = 0.0, next_ext = 0.0, next_batt = 0.0;

  auto deliverCallbacks = [&](double t, bool force) {
    setNow(t);
    const double now = kTimeOffset + t;  // 回调时刻即接收时间
    if (force || t >= next_odom) {
      uc.onOdom(now, w.fcu.pos, w.fcu.vel, w.fcu.yaw, now);
      next_odom = t + kOdomPeriod;
    }
    if (force || t >= next_rc) {
      // 链路状态变化：失控瞬间锁存开关和 CH7（接收机"保持"）
      if (w.prev_link_ok && !w.link_ok) {
        w.held_switch = w.pilot.mode_switch;
        w.held_ch7 = w.pilot.ch7_pressed ? kCh7Pressed : kCh7Released;
      }
      w.prev_link_ok = w.link_ok;
      std::vector<uint16_t> ch(18, 1500);
      const double throttle_seen = w.link_ok ? w.pilot.throttle : 0.0;
      const std::string switch_seen = w.link_ok ? w.pilot.mode_switch : w.held_switch;
      ch[2] = static_cast<uint16_t>(1500 + 500 * throttle_seen);
      ch[6] = w.link_ok ? (w.pilot.ch7_pressed ? kCh7Pressed : kCh7Released) : w.held_ch7;
      if (w.inject_ch7_dip) {
        ch[6] = kCh7Pressed;
        w.inject_ch7_dip = false;
      }
      ch[17] = w.link_ok ? kCh18Ok : kCh18Lost;
      uc.onRc(now, ch);
      w.fcu.onRcFrame(t, throttle_seen, switch_seen);
      next_rc = t + kRcPeriod;
    }
    if (force || t >= next_state) {
      uc.onFcuState(now, true, w.fcu.armed, w.fcu.nav);
      next_state = t + kStatePeriod;
    }
    if (force || t >= next_ext) {
      uc.onExtendedState(now, w.fcu.landed ? LandedState::OnGround : LandedState::InAir);
      next_ext = t + kExtStatePeriod;
    }
    if (force || t >= next_batt) {
      uc.onBattery(now, w.battery);
      next_batt = t + kBatteryPeriod;
    }
  };

  auto updateMetrics = [&](double t) {
    // 飞控意图为 OFFBOARD 期间的 setpoint 间隔
    if (w.fcu.intention == "OFFBOARD" && m.last_sp_t > 0.0) {
      m.max_sp_gap_in_offboard = std::max(m.max_sp_gap_in_offboard, t - m.last_sp_t);
    }
    // 任务中链路丢失、处于 POSCTL 的连续时长
    if (m.mission_start_t > 0.0 && !w.link_ok && w.fcu.armed && w.fcu.nav == "POSCTL") {
      if (m.posctl_lost_since < 0.0) m.posctl_lost_since = t;
      m.max_posctl_while_lost = std::max(m.max_posctl_while_lost, t - m.posctl_lost_since);
    } else {
      m.posctl_lost_since = -1.0;
    }
    // 触地后的最大高度
    if (w.fcu.pos.z() > 1.0) m.was_high = true;
    if (m.was_high && m.touchdown_t < 0.0 && w.fcu.pos.z() <= 0.02) m.touchdown_t = t;
    if (m.touchdown_t > 0.0) m.max_z_after_touchdown = std::max(m.max_z_after_touchdown, w.fcu.pos.z());
  };

  // 节点在解锁前启动：先收到 t=0 的未解锁状态，0.5 s 时解锁（否则按空中重启处理，R-58）
  for (double t = 0.0; t < sc.duration;) {
    if (!w.fcu.armed && t >= 0.5 && t < 1.0) w.fcu.arm(t);
    sc.script(t, w);
    deliverCallbacks(t, false);

    if (t >= next_tick && w.stall_s > 0.0 && t >= w.stall_at) {
      // 控制循环卡顿：期间飞控照常运行，模块不发 setpoint、不处理回调
      for (double tb = t; tb < t + w.stall_s; tb += kSimDt) {
        w.fcu.step(tb, kSimDt, w.pilot, w.link_ok);
        updateMetrics(tb);
      }
      t += w.stall_s;
      w.stall_s = 0.0;
      deliverCallbacks(t, true);
      next_tick = t;
      continue;
    }

    if (t >= next_tick) {
      setNow(t);
      const auto wall0 = std::chrono::steady_clock::now();
      const TickOutput out = uc.tick(kTimeOffset + t);
      const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
      m.max_tick_ms = std::max(m.max_tick_ms, wall * 1000.0);

      // tick 阻塞期间：飞控照常运行，模块不发 setpoint、不处理回调
      double t_after = t;
      if (wall > kTickPeriod) {
        for (double tb = t; tb < t + wall; tb += kSimDt) {
          w.fcu.step(tb, kSimDt, w.pilot, w.link_ok);
          updateMetrics(tb);
        }
        t_after = t + wall;
      }

      w.module_state = out.state;
      if (out.state == "ReturnPreOffboard" && w.prepare_seen_t < 0.0) w.prepare_seen_t = t_after;
      if (out.state != m.last_state) {
        m.states.push_back({t_after, out.state});
        if (out.state == "Recording" && m.last_state == "WaitRecordDelay") {
          m.trail_start = w.fcu.pos;
        }
        if (out.state == "Exited") m.exited = true;
        if (isMission(out.state) && m.mission_start_t < 0.0) {
          m.mission_start_t = t_after;
          m.mission_start_z = w.fcu.pos.z();
          m.mission_started_on_ground = w.fcu.pos.z() < 0.3;
        }
        m.last_state = out.state;
      }
      if (out.setpoint) {
        w.fcu.onSetpoint(t_after, *out.setpoint);
        m.last_sp_t = t_after;
      }
      if (out.mode_request) {
        // ModeCommander 决定何时发出；服务调用异步，mode_sent 为真（MAVROS 已发出 SET_MODE）
        const std::string& mode = *out.mode_request;
        ++m.mode_requests_sent;
        if (w.pilot_took_over && w.link_ok && mode != "POSCTL") ++m.fight_requests;
        w.fcu.onModeRequest(t_after, mode);
        uc.onModeRequestResult(kTimeOffset + t_after, mode, true);
      }
      if (t_after > t) {
        t = t_after;
        deliverCallbacks(t, true);  // 阻塞结束后先处理排队的消息
        next_tick = t + kTickPeriod;
        continue;
      }
      next_tick += kTickPeriod;
    }

    w.fcu.step(t, kSimDt, w.pilot, w.link_ok);
    updateMetrics(t);
    t += kSimDt;
  }

  std::printf("\n## %s %s\n\n", sc.id.c_str(), sc.title.c_str());
  std::printf("模块状态：");
  for (size_t i = 0; i < m.states.size(); ++i) {
    std::printf("%s%.1f %s", i ? " → " : "", m.states[i].first, m.states[i].second.c_str());
  }
  std::printf("\n飞控事件：");
  for (size_t i = 0; i < w.fcu.events.size(); ++i) {
    std::printf("%s%.1f %s", i ? "；" : "", w.fcu.events[i].first, w.fcu.events[i].second.c_str());
  }
  std::printf("\n\n");
  for (const auto& c : sc.checks(w, m)) {
    std::printf("- [%s] %s：%s\n", c.pass ? "通过" : "失败", c.name.c_str(), c.detail.c_str());
    summary.push_back({sc.id + " " + c.name, c.pass});
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <trajectory_config.yaml> [scenario ...]\n", argv[0]);
    return 2;
  }
  ros::Time::init();
  if (ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Error)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  std::set<std::string> only(argv + 2, argv + argc);
  std::vector<std::pair<std::string, bool>> summary;
  for (const auto& sc : buildScenarios()) {
    if (!only.empty() && !only.count(sc.id)) continue;
    runScenario(sc, argv[1], summary);
  }
  int failed = 0;
  std::printf("\n## 汇总\n\n");
  for (const auto& [name, pass] : summary) {
    std::printf("- [%s] %s\n", pass ? "通过" : "失败", name.c_str());
    failed += pass ? 0 : 1;
  }
  std::printf("\n%zu 项检查，%d 项失败\n", summary.size(), failed);
  return 0;
}
