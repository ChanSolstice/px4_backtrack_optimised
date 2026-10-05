#include "px4_backtrack/domain/waypoint_fallback_session.hpp"
#include "px4_backtrack/domain/path_yaw.hpp"

#include <algorithm>
#include <cmath>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kEps = 1e-9;
constexpr uint16_t kPositionYawTypeMask = 0x09F8;  // ignore vel/acc/yaw_rate

std::vector<FallbackWaypoint> buildSparseReverseWaypoints(
    const std::vector<RecordPoint>& points,
    double spacing_m) {
  std::vector<FallbackWaypoint> sparse;
  if (points.size() < 2) {
    return sparse;
  }

  if (!std::isfinite(spacing_m) || spacing_m <= 0.0) return sparse;
  for (const auto& point : points) {
    if (!point.position.allFinite() || !std::isfinite(point.yaw)) return {};
  }

  // 只消除同方向共线点。每个折线顶点保留，连接线严格位于原路上。
  std::vector<Vec3> vertices;
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    if (!vertices.empty() && (it->position - vertices.back()).norm() <= kEps) continue;
    while (vertices.size() >= 2) {
      const Vec3 incoming = vertices.back() - vertices[vertices.size()-2];
      const Vec3 outgoing = it->position - vertices.back();
      if (incoming.dot(outgoing) <= 0.0 ||
          incoming.cross(outgoing).norm() > 1e-12 * incoming.norm() * outgoing.norm()) break;
      vertices.pop_back();
    }
    vertices.push_back(it->position);
  }
  if (vertices.size() < 2) return sparse;
  sparse.emplace_back(vertices.front(), points.back().yaw);
  for (size_t i = 1; i < vertices.size(); ++i) {
    const Vec3 delta = vertices[i] - vertices[i-1];
    const double count = std::ceil(delta.norm() / spacing_m);
    if (!std::isfinite(count) || count > 1000000.0 - sparse.size()) return {};
    const size_t steps = static_cast<size_t>(count);
    for (size_t k = 1; k <= steps; ++k) {
      sparse.emplace_back(vertices[i-1] + delta * (static_cast<double>(k) / steps), 0.0);
    }
  }

  std::vector<Vec3> positions;
  positions.reserve(sparse.size());
  for (const auto& waypoint : sparse) {
    positions.push_back(waypoint.position);
  }
  const auto yaws = computePathTangentYaws(positions, points.back().yaw);
  for (size_t i = 0; i < sparse.size(); ++i) {
    sparse[i].yaw = yaws[i];
  }

  return sparse;
}

}  // namespace

WaypointFallbackSession::WaypointFallbackSession(double spacing_m, double reach_radius_m)
  : spacing_m_(spacing_m),
    reach_radius_m_(reach_radius_m),
    current_index_(0),
    finished_(false) {}

bool WaypointFallbackSession::build(const std::vector<RecordPoint>& points) {
  reset();
  waypoints_ = buildSparseReverseWaypoints(points, spacing_m_);
  return isValid();
}

void WaypointFallbackSession::reset() {
  waypoints_.clear();
  current_index_ = 0;
  finished_ = false;
}

void WaypointFallbackSession::updateProgress(const Vec3& current_position,
                                             const Vec3& current_velocity,
                                             double current_yaw) {
  if (!isValid() || finished_ || !current_position.allFinite() ||
      !current_velocity.allFinite() || !std::isfinite(current_yaw)) {
    return;
  }

  if (current_index_ >= waypoints_.size()) {
    return;
  }

  double dist = (current_position - waypoints_[current_index_].position).norm();
  if (dist > reach_radius_m_) {
    return;
  }

  if (current_index_ + 1 < waypoints_.size()) {
    // 到点停稳，再朝向下一条连接线；PX4 负责限制原地 yaw 转速。
    if (current_velocity.norm() > 0.10) return;
    const Vec3 outgoing = waypoints_[current_index_+1].position - waypoints_[current_index_].position;
    const double next_yaw = outgoing.head<2>().norm() > kEps
        ? std::atan2(outgoing.y(), outgoing.x()) : waypoints_[current_index_].yaw;
    waypoints_[current_index_].yaw = next_yaw;
    const double error = std::atan2(std::sin(current_yaw-next_yaw), std::cos(current_yaw-next_yaw));
    if (std::abs(error) > 15.0 * std::acos(-1.0) / 180.0) return;
    ++current_index_;
    return;
  }

  finished_ = true;
}

bool WaypointFallbackSession::isValid() const {
  return waypoints_.size() >= 2;
}

bool WaypointFallbackSession::isFinished() const {
  return finished_;
}

SetpointTarget WaypointFallbackSession::currentTarget() const {
  SetpointTarget target;
  if (!isValid()) {
    return target;
  }

  const size_t clamped_index = std::min(current_index_, waypoints_.size() - 1);
  const auto& waypoint = waypoints_[clamped_index];
  target.position = waypoint.position;
  target.velocity = Vec3::Zero();
  target.yaw = waypoint.yaw;
  target.yaw_rate = 0.0;
  target.acceleration = std::nullopt;
  target.type_mask = kPositionYawTypeMask;
  target.coordinate_frame = 1;  // FRAME_LOCAL_NED
  return target;
}

size_t WaypointFallbackSession::size() const {
  return waypoints_.size();
}

size_t WaypointFallbackSession::currentIndex() const {
  return current_index_;
}

}  // namespace domain
}  // namespace px4_backtrack
