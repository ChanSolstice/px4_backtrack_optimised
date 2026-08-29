#include "px4_backtrack/domain/path_yaw.hpp"

#include <cmath>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kHorizontalDirectionEps = 1e-6;
constexpr double kTangentChordMinM = 0.20;

bool hasHorizontalDirection(const Vec3& direction) {
  return std::hypot(direction.x(), direction.y()) > kHorizontalDirectionEps;
}

bool reachesTangentChord(const Vec3& direction) {
  return std::hypot(direction.x(), direction.y()) >= kTangentChordMinM;
}

}  // namespace

std::vector<double> computePathTangentYaws(
    const std::vector<Vec3>& positions,
    double fallback_yaw) {
  std::vector<double> yaws(positions.size(), fallback_yaw);

  for (size_t i = 0; i < positions.size(); ++i) {
    Vec3 tangent = Vec3::Zero();
    size_t previous = i;
    size_t next = i;

    // 用至少 0.2 m 的水平弦估计切向，避免录制起止悬停的厘米级漂移主导 yaw。
    for (size_t candidate = i + 1; candidate < positions.size(); ++candidate) {
      if (reachesTangentChord(positions[candidate] - positions[i])) {
        next = candidate;
        break;
      }
    }
    for (size_t candidate = i; candidate > 0; --candidate) {
      if (reachesTangentChord(positions[i] - positions[candidate - 1])) {
        previous = candidate - 1;
        break;
      }
    }

    if (previous < i && next > i) {
      tangent = positions[next] - positions[previous];
    } else if (next > i) {
      tangent = positions[next] - positions[i];
    } else if (previous < i) {
      tangent = positions[i] - positions[previous];
    }

    // 短于最小弦长的整段路径仍退化到任意有效水平差分。
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
