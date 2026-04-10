#include "px4_backtrack/domain/waypoint_fallback_session.hpp"

#include <algorithm>
#include <cmath>

namespace px4_backtrack {
namespace domain {

namespace {

constexpr double kEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr uint16_t kPositionYawTypeMask = 0x09F8;  // ignore vel/acc/yaw_rate

double shortestAngleDelta(double from, double to) {
  double delta = std::fmod(to - from, 2.0 * kPi);
  if (delta > kPi) {
    delta -= 2.0 * kPi;
  } else if (delta < -kPi) {
    delta += 2.0 * kPi;
  }
  return delta;
}

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
  double segment_start_yaw = points.back().yaw;
  double distance_since_last = 0.0;

  for (size_t rev_idx = n - 1; rev_idx > 0; --rev_idx) {
    const auto& next_point = points[rev_idx - 1];
    Vec3 segment_end_pos = next_point.position;
    double segment_end_yaw = next_point.yaw;
    Vec3 segment_vec = segment_end_pos - segment_start_pos;
    double segment_len = segment_vec.norm();

    if (segment_len <= kEps) {
      segment_start_pos = segment_end_pos;
      segment_start_yaw = segment_end_yaw;
      continue;
    }

    while (distance_since_last + segment_len >= spacing_m) {
      double remain = spacing_m - distance_since_last;
      double ratio = remain / segment_len;
      Vec3 interp_pos = segment_start_pos + ratio * segment_vec;
      double interp_yaw = segment_start_yaw +
                          shortestAngleDelta(segment_start_yaw, segment_end_yaw) * ratio;
      sparse.emplace_back(interp_pos, interp_yaw);

      segment_start_pos = interp_pos;
      segment_start_yaw = interp_yaw;
      segment_vec = segment_end_pos - segment_start_pos;
      segment_len = segment_vec.norm();
      distance_since_last = 0.0;

      if (segment_len <= kEps) {
        break;
      }
    }

    distance_since_last += segment_len;
    segment_start_pos = segment_end_pos;
    segment_start_yaw = segment_end_yaw;
  }

  const auto& home = points.front();
  if ((sparse.back().position - home.position).norm() > kEps) {
    sparse.emplace_back(home.position, home.yaw);
  } else {
    sparse.back().yaw = home.yaw;
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

void WaypointFallbackSession::restart() {
  if (!isValid()) {
    return;
  }
  current_index_ = 0;
  finished_ = false;
}

void WaypointFallbackSession::updateProgress(const Vec3& current_position) {
  if (!isValid() || finished_) {
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
