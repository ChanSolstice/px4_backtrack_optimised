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

  const size_t n = points.size();
  sparse.emplace_back(points.back().position, points.back().yaw);

  Vec3 segment_start_pos = points.back().position;
  double distance_since_last = 0.0;

  for (size_t rev_idx = n - 1; rev_idx > 0; --rev_idx) {
    const auto& next_point = points[rev_idx - 1];
    Vec3 segment_end_pos = next_point.position;
    Vec3 segment_vec = segment_end_pos - segment_start_pos;
    double segment_len = segment_vec.norm();

    if (segment_len <= kEps) {
      segment_start_pos = segment_end_pos;
      continue;
    }

    while (distance_since_last + segment_len >= spacing_m) {
      double remain = spacing_m - distance_since_last;
      double ratio = remain / segment_len;
      Vec3 interp_pos = segment_start_pos + ratio * segment_vec;
      sparse.emplace_back(interp_pos, 0.0);

      segment_start_pos = interp_pos;
      segment_vec = segment_end_pos - segment_start_pos;
      segment_len = segment_vec.norm();
      distance_since_last = 0.0;

      if (segment_len <= kEps) {
        break;
      }
    }

    distance_since_last += segment_len;
    segment_start_pos = segment_end_pos;
  }

  const auto& home = points.front();
  if ((sparse.back().position - home.position).norm() > kEps) {
    sparse.emplace_back(home.position, 0.0);
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

void WaypointFallbackSession::updateProgress(const Vec3& current_position) {
  if (!isValid() || finished_) {
    return;
  }

  double dist = (current_position - waypoints_[current_index_].position).norm();
  if (dist > reach_radius_m_) {
    return;
  }

  if (current_index_ + 1 < waypoints_.size()) {
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
  const auto& waypoint = waypoints_[current_index_];
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

double WaypointFallbackSession::remainingLength() const {
  double length = 0.0;
  for (size_t i = current_index_; i + 1 < waypoints_.size(); ++i) {
    length += (waypoints_[i + 1].position - waypoints_[i].position).norm();
  }
  return length;
}

}  // namespace domain
}  // namespace px4_backtrack
