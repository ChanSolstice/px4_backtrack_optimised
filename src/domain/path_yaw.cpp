#include "px4_backtrack/domain/path_yaw.hpp"

#include <cmath>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kHorizontalDirectionEps = 1e-6;

bool hasHorizontalDirection(const Vec3& direction) {
  return std::hypot(direction.x(), direction.y()) > kHorizontalDirectionEps;
}

}  // namespace

std::vector<double> computePathTangentYaws(
    const std::vector<Vec3>& positions,
    double fallback_yaw) {
  std::vector<double> yaws(positions.size(), fallback_yaw);

  for (size_t i = 0; i < positions.size(); ++i) {
    Vec3 tangent = Vec3::Zero();

    if (i > 0 && i + 1 < positions.size()) {
      tangent = positions[i + 1] - positions[i - 1];
    } else if (i + 1 < positions.size()) {
      tangent = positions[i + 1] - positions[i];
    } else if (i > 0) {
      tangent = positions[i] - positions[i - 1];
    }

    // 垂直段或重复水平点没有可用切线，优先寻找后续飞行方向。
    if (!hasHorizontalDirection(tangent)) {
      for (size_t next = i + 1; next < positions.size(); ++next) {
        tangent = positions[next] - positions[i];
        if (hasHorizontalDirection(tangent)) {
          break;
        }
      }
    }

    // 终点附近没有后续方向时，保持最近一个有效的进入方向。
    if (!hasHorizontalDirection(tangent)) {
      for (size_t previous = i; previous > 0; --previous) {
        tangent = positions[i] - positions[previous - 1];
        if (hasHorizontalDirection(tangent)) {
          break;
        }
      }
    }

    if (hasHorizontalDirection(tangent)) {
      yaws[i] = std::atan2(tangent.y(), tangent.x());
    } else if (i > 0) {
      yaws[i] = yaws[i - 1];
    }
  }

  return yaws;
}

}  // namespace domain
}  // namespace px4_backtrack
