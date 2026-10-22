#include "px4_backtrack/domain/recording_buffer.hpp"
#include <algorithm>
#include <cstddef>

namespace px4_backtrack {
namespace domain {

RecordingBuffer::RecordingBuffer(double max_hz)
  : max_hz_(max_hz),
    min_dt_(1.0 / max_hz),
    time_origin_(0.0),
    resume_pending_(false),
    last_record_stamp_(0.0),
    has_first_point_(false)
{
  points_.reserve(1000);  // 预分配空间，减少动态扩容
}

void RecordingBuffer::reset() {
  points_.clear();
  time_origin_ = 0.0;
  resume_pending_ = false;
  last_record_stamp_ = 0.0;
  has_first_point_ = false;
}

bool RecordingBuffer::addPoint(const Vec3& position, double yaw, double odom_stamp) {
  // 第一个点：初始化时间参考
  if (!has_first_point_) {
    time_origin_ = odom_stamp;
    last_record_stamp_ = odom_stamp;
    has_first_point_ = true;

    // 第一个点的相对时间戳为0
    points_.emplace_back(position, yaw, 0.0);
    return true;
  }

  // 续录：第一个新点接在末点之后 min_dt_，之后按里程计时间递增（第 7.4 节）
  if (resume_pending_) {
    resume_pending_ = false;
    const double timestamp = points_.back().timestamp + min_dt_;
    time_origin_ = odom_stamp - timestamp;
    last_record_stamp_ = odom_stamp;
    points_.emplace_back(position, yaw, timestamp);
    return true;
  }

  // 节流检查：距离上次录制时间是否足够
  double dt = odom_stamp - last_record_stamp_;
  if (dt < min_dt_) {
    return false;  // 被节流过滤
  }

  // 相对时间戳：节流已保证比上一个点至少晚 min_dt_，单调递增
  points_.emplace_back(position, yaw, odom_stamp - time_origin_);
  last_record_stamp_ = odom_stamp;

  return true;
}

size_t RecordingBuffer::size() const {
  return points_.size();
}

const std::vector<RecordPoint>& RecordingBuffer::getPoints() const {
  return points_;
}

std::vector<RecordPoint> RecordingBuffer::getPointsCopy() const {
  return points_;
}

bool RecordingBuffer::empty() const {
  return points_.empty();
}

void RecordingBuffer::truncate(size_t last_index) {
  if (last_index + 1 < points_.size()) {
    points_.erase(points_.begin() + static_cast<std::ptrdiff_t>(last_index + 1), points_.end());
  }
}

void RecordingBuffer::resume() {
  resume_pending_ = !points_.empty();
}

size_t RecordingBuffer::continuousFrom(double max_gap_m) const {
  size_t begin = points_.empty() ? 0 : points_.size() - 1;
  while (begin > 0 &&
         (points_[begin].position - points_[begin - 1].position).norm() <= max_gap_m) {
    --begin;
  }
  return begin;
}

size_t RecordingBuffer::nearestBefore(size_t cursor, const Vec3& reference,
                                      double window_m) const {
  size_t best = cursor;
  double best_distance = (points_[cursor].position - reference).norm();
  double arc = 0.0;
  for (size_t i = cursor; i > 0; --i) {
    arc += (points_[i].position - points_[i - 1].position).norm();
    if (arc > window_m) {
      break;
    }
    const double distance = (points_[i - 1].position - reference).norm();
    if (distance <= best_distance) {  // 距离相同时取更小的索引
      best = i - 1;
      best_distance = distance;
    }
  }
  return best;
}

}  // namespace domain
}  // namespace px4_backtrack
