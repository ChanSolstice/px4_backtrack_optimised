#pragma once

#include "state_machine.hpp"

#include <vector>

namespace px4_backtrack {
namespace domain {

/// 根据路径水平切线计算各点yaw；无有效水平切线时使用fallback_yaw
std::vector<double> computePathTangentYaws(
    const std::vector<Vec3>& positions,
    double fallback_yaw);

}  // namespace domain
}  // namespace px4_backtrack
