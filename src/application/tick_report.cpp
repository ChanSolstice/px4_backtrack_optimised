#include "px4_backtrack/application/tick_report.hpp"

#include <cstdio>
#include <iomanip>
#include <optional>
#include <sstream>
#include <type_traits>
#include <vector>

#include "px4_backtrack/infrastructure/debug_log.hpp"

namespace px4_backtrack {
namespace application {

using infrastructure::DebugLog;

namespace {

using Cells = std::vector<std::string>;

/// 条件快照的全部字段，按列的顺序。写和解析共用这一份字段表；Conditions 增加字段时要加在
/// 这里，漏掉的字段回放时取默认值，S31 会发现
template <typename C, typename F>
void forEachConditionField(C& c, F&& f) {
  f("now", c.now);
  f("fcu_state_fresh", c.fcu_state_fresh);
  f("fcu_state_t", c.fcu_state_t);
  f("armed", c.armed);
  f("mode", c.mode);
  f("link.lost", c.link.lost);
  f("link.source", c.link.source);
  f("link.degraded", c.link.degraded);
  f("link.ch18_low_now", c.link.ch18_low_now);
  f("link.rc_timeout", c.link.rc_timeout);
  f("link.loss_bucket_s", c.link.loss_bucket_s);
  f("link.ch18_unavailable", c.link.ch18_unavailable);
  f("link.ch18_unexpected", c.link.ch18_unexpected);
  f("link.ch18_value", c.link.ch18_value);
  f("pilot.pressed", c.pilot.pressed);
  f("pilot.press_id", c.pilot.press_id);
  f("pilot.press_start_t", c.pilot.press_start_t);
  f("pilot.wait_release", c.pilot.wait_release);
  f("pilot.return_request", c.pilot.return_request);
  f("pilot.exit_request", c.pilot.exit_request);
  f("battery.low", c.battery.low);
  f("battery.invalid", c.battery.invalid);
  f("battery.percentage", c.battery.percentage);
  f("airborne.airborne", c.airborne.airborne);
  f("airborne.landed_state", c.airborne.landed_state);
  f("airborne.arm_height", c.airborne.arm_height);
  f("airborne.height_above_arm_m", c.airborne.height_above_arm_m);
  f("odom_stale", c.odom_stale);
  f("position", c.position);
  f("yaw", c.yaw);
  f("has_min_points", c.has_min_points);
  f("plan_status", c.plan_status);
  f("trail_gap", c.trail_gap);
  f("anchor_ahead", c.anchor_ahead);
  f("anchor_past_gap", c.anchor_past_gap);
  f("planning_slow", c.planning_slow);
  f("planning_timed_out", c.planning_timed_out);
  f("path_start_yaw", c.path_start_yaw);
  f("path_target_yaw", c.path_target_yaw);
  f("deviation_confirmed", c.deviation_confirmed);
  f("rejoin_confirmed", c.rejoin_confirmed);
  f("finished", c.finished);
  f("remaining_m", c.remaining_m);
  f("projection_ok", c.projection_ok);
  f("cross_track_m", c.cross_track_m);
  f("stream_gap_recent", c.stream_gap_recent);
  f("mode_request_failed", c.mode_request_failed);
  f("debug_log_failed", c.debug_log_failed);
}

/// 17 位有效数字：解析回来与原值完全一致
std::string number(double value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", value);
  return buf;
}

std::string join(const Cells& cells) {
  std::string line;
  for (std::size_t i = 0; i < cells.size(); ++i) {
    line += (i ? "," : "") + cells[i];
  }
  return line;
}

/// 列名：向量和 CH7 指令各占几列
struct HeaderCells {
  Cells& cells;
  void operator()(const char* name, const domain::Vec3&) const {
    for (const char* axis : {".x", ".y", ".z"}) {
      cells.push_back(name + std::string(axis));
    }
  }
  void operator()(const char* name, const std::optional<domain::Ch7Command>&) const {
    for (const char* field : {".press_id", ".press_start_t", ".t", ".link_ok"}) {
      cells.push_back(name + std::string(field));
    }
  }
  template <typename T>
  void operator()(const char* name, const T&) const {
    cells.push_back(name);
  }
};

/// 取值：布尔写 0/1，枚举写数值，可选值缺失时留空
struct ValueCells {
  Cells& cells;
  void operator()(const char*, double v) const { cells.push_back(number(v)); }
  void operator()(const char*, bool v) const { cells.push_back(v ? "1" : "0"); }
  void operator()(const char*, int v) const { cells.push_back(std::to_string(v)); }
  void operator()(const char*, const std::string& v) const { cells.push_back(v); }
  void operator()(const char*, const domain::Vec3& v) const {
    for (int k = 0; k < 3; ++k) {
      cells.push_back(number(v[k]));
    }
  }
  void operator()(const char*, const std::optional<double>& v) const {
    cells.push_back(v ? number(*v) : "");
  }
  void operator()(const char*, const std::optional<domain::Ch7Command>& v) const {
    if (!v) {
      cells.insert(cells.end(), 4, "");
      return;
    }
    cells.push_back(std::to_string(v->press_id));
    cells.push_back(number(v->press_start_t));
    cells.push_back(number(v->t));
    cells.push_back(v->link_ok ? "1" : "0");
  }
  template <typename E>
  std::enable_if_t<std::is_enum_v<E>> operator()(const char*, E v) const {
    cells.push_back(std::to_string(static_cast<int>(v)));
  }
};

/// 按列解析，与 ValueCells 对应
struct ParseCells {
  const Cells& cells;
  std::size_t& next_index;
  const std::string& next() const { return cells.at(next_index++); }
  void operator()(const char*, double& v) const { v = std::stod(next()); }
  void operator()(const char*, bool& v) const { v = next() == "1"; }
  void operator()(const char*, int& v) const { v = std::stoi(next()); }
  void operator()(const char*, std::string& v) const { v = next(); }
  void operator()(const char*, domain::Vec3& v) const {
    for (int k = 0; k < 3; ++k) {
      v[k] = std::stod(next());
    }
  }
  void operator()(const char*, std::optional<double>& v) const {
    const std::string& text = next();
    v = text.empty() ? std::nullopt : std::optional<double>(std::stod(text));
  }
  void operator()(const char*, std::optional<domain::Ch7Command>& v) const {
    const std::string& press_id = next();
    if (press_id.empty()) {
      next_index += 3;
      v.reset();
      return;
    }
    domain::Ch7Command command;
    command.press_id = std::stoi(press_id);
    command.press_start_t = std::stod(next());
    command.t = std::stod(next());
    command.link_ok = next() == "1";
    v = command;
  }
  template <typename E>
  std::enable_if_t<std::is_enum_v<E>> operator()(const char*, E& v) const {
    v = static_cast<E>(std::stoi(next()));
  }
};

}  // namespace

std::string statusJson(double t, const TickOutput& out) {
  std::ostringstream json;
  json << std::fixed << std::setprecision(3) << "{\"t\":" << t
       << ",\"flight\":" << out.flight_id
       << ",\"state\":" << DebugLog::jsonString(out.state)
       << ",\"control\":" << DebugLog::jsonString(domain::controlName(out.control))
       << ",\"mission\":" << out.mission_id
       << ",\"reason\":" << DebugLog::jsonString(domain::missionReasonName(out.mission_reason))
       << ",\"mode_request\":" << DebugLog::jsonString(out.mode_status.mode)
       << ",\"mode_result\":"
       << DebugLog::jsonString(modeRequestResultName(out.mode_status.result))
       << ",\"alerts\":[";
  const char* separator = "";
  for (int i = 0; i < static_cast<int>(domain::Alert::Count); ++i) {
    const auto alert = static_cast<domain::Alert>(i);
    if (out.alerts.has(alert)) {
      json << separator << DebugLog::jsonString(domain::alertName(alert));
      separator = ",";
    }
  }
  json << "]}";
  return json.str();
}

std::string traceHeader() {
  Cells cells;
  const domain::Conditions names_only;
  forEachConditionField(names_only, HeaderCells{cells});
  for (const char* name : {"state", "control", "mode_intent", "setpoint_kind", "alerts",
                           "sp.x", "sp.y", "sp.z", "sp.yaw", "tick_ms"}) {
    cells.push_back(name);
  }
  return join(cells);
}

std::string traceRow(const TickOutput& out, double tick_ms) {
  Cells cells;
  forEachConditionField(out.conditions, ValueCells{cells});
  cells.push_back(out.state);
  cells.push_back(domain::controlName(out.control));
  cells.push_back(out.mode_status.mode);
  cells.push_back(std::to_string(static_cast<int>(out.setpoint_kind)));
  cells.push_back(std::to_string(out.alerts.bits()));
  if (out.setpoint) {
    for (int k = 0; k < 3; ++k) {
      cells.push_back(number(out.setpoint->position[k]));
    }
    cells.push_back(number(out.setpoint->yaw));
  } else {
    cells.insert(cells.end(), 4, "");
  }
  cells.push_back(number(tick_ms));
  return join(cells);
}

TraceRecord parseTraceRow(const std::string& line) {
  Cells cells;
  std::size_t start = 0;
  for (std::size_t comma; (comma = line.find(',', start)) != std::string::npos;
       start = comma + 1) {
    cells.push_back(line.substr(start, comma - start));
  }
  cells.push_back(line.substr(start));

  TraceRecord record;
  std::size_t next_index = 0;
  forEachConditionField(record.conditions, ParseCells{cells, next_index});
  record.state = cells.at(next_index++);
  record.control = cells.at(next_index++);
  record.mode_intent = cells.at(next_index++);
  record.setpoint = static_cast<domain::SetpointKind>(std::stoi(cells.at(next_index++)));
  record.alerts = std::stoull(cells.at(next_index++));
  return record;
}

}  // namespace application
}  // namespace px4_backtrack
