#pragma once
#include "recording_buffer.hpp"
#include <limits>

namespace px4_backtrack::domain {
// 有序原始录制折线，供三维偏离监控使用；与生成轨迹和降级路径独立。
class RecordedRoute {
public:
  bool build(const std::vector<RecordPoint>& points);
  void reset();
  bool update(const Vec3& position);
  double distance() const { return distance_; }
  bool valid() const { return points_.size() >= 2; }
  double predictedDeviation(const Vec3& position, const Vec3& velocity,
      double reaction_s, double braking_m_s2, double uncertainty_m) const;
private:
  std::vector<Vec3> points_;
  std::vector<double> arcs_;
  double progress_ = 0.0;
  double distance_ = std::numeric_limits<double>::infinity();
  Vec3 nearest_ = Vec3::Zero();
};
}
