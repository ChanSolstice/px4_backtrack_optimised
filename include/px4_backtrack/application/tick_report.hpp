#pragma once

#include <string>

#include "px4_backtrack/application/backtrack_usecase.hpp"

namespace px4_backtrack {
namespace application {

/// 状态话题 /px4_backtrack/status 的 JSON（重构方案第 9.1 节，R-88）
/// @param t 本拍时间 (s)
/// @param out 本拍输出
std::string statusJson(double t, const TickOutput& out);

}  // namespace application
}  // namespace px4_backtrack
