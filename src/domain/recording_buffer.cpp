#include "px4_backtrack/domain/recording_buffer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_backtrack {
namespace domain {

RecordingBuffer::RecordingBuffer(double max_hz)
  : max_hz_(max_hz),
    min_dt_(1.0 / max_hz),
    first_odom_stamp_(0.0),
    last_record_stamp_(0.0),
    has_first_point_(false)
{
  if (!std::isfinite(max_hz) || max_hz<=0.0) throw std::invalid_argument("Invalid recording frequency");
  points_.reserve(1000);  // 预分配空间，减少动态扩容
}

void RecordingBuffer::reset() {
  points_.clear();
  first_odom_stamp_ = 0.0;
  last_record_stamp_ = 0.0;
  has_first_point_ = false;
}

bool RecordingBuffer::addPoint(const Vec3& position, double yaw, double odom_stamp) {
  if (!position.allFinite() || !std::isfinite(yaw) || !std::isfinite(odom_stamp) || odom_stamp<0.0) return false;
  // 第一个点：初始化时间参考
  if (!has_first_point_) {
    first_odom_stamp_ = odom_stamp;
    last_record_stamp_ = odom_stamp;
    has_first_point_ = true;

    // 第一个点的相对时间戳为0
    points_.emplace_back(position, yaw, 0.0);
    return true;
  }

  // 节流检查：距离上次录制时间是否足够
  double dt = odom_stamp - last_record_stamp_;
  if (dt < min_dt_) {
    return false;  // 被节流过滤
  }

  // 计算相对时间戳（从第一个点开始）
  double relative_timestamp = odom_stamp - first_odom_stamp_;

  // 单调性检查（防御性编程）
  if (!points_.empty() && relative_timestamp <= points_.back().timestamp) {
    // 时间戳未增加或回退，跳过此点
    return false;
  }

  // 添加录制点
  points_.emplace_back(position, yaw, relative_timestamp);
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

}  // namespace domain
}  // namespace px4_backtrack
