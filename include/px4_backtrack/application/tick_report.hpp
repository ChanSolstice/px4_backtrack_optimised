#pragma once

#include <cstdint>
#include <string>

#include "px4_backtrack/application/backtrack_usecase.hpp"

namespace px4_backtrack {
namespace application {

/// 状态话题 /px4_backtrack/status 的 JSON（重构方案第 9.1 节，R-88）
/// @param t 本拍时间 (s)
/// @param out 本拍输出
std::string statusJson(double t, const TickOutput& out);

/// 逐拍记录 trace.csv 的表头（重构方案第 9.2 节）
std::string traceHeader();

/// 逐拍记录的一行：条件快照的全部字段，以及上报状态、控制权、模式意图、setpoint 种类、
/// 告警位、实际发布的 setpoint 和本拍耗时
/// @param tick_ms 本拍耗时 (ms)
std::string traceRow(const TickOutput& out, double tick_ms);

/// 逐拍记录中回放需要的内容
struct TraceRecord {
  domain::Conditions conditions;  ///< 条件快照，与当时的数值完全一致
  std::string state;              ///< 上报状态
  std::string control;            ///< 控制权
  std::string mode_intent;        ///< 模式意图（MAVROS 模式字符串，没有为空）
  domain::SetpointKind setpoint{domain::SetpointKind::None};
  uint64_t alerts{0};             ///< 告警位
};

/// 解析 traceRow 写出的一行，用于逐拍回放到 ReturnStateMachine::step()（S31）
TraceRecord parseTraceRow(const std::string& line);

}  // namespace application
}  // namespace px4_backtrack
