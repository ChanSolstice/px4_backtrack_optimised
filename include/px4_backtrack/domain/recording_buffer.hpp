#pragma once

#include "px4_backtrack/domain/types.hpp"
#include <vector>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 录制点数据结构
// ============================================================================

/// 单个录制点
struct RecordPoint {
  /// 位置 (ENU, m)
  Vec3 position;

  /// Yaw角 (rad)
  double yaw;

  /// 相对时间戳 (s) - 从第一个点开始计时
  double timestamp;

  /// 构造函数
  RecordPoint(const Vec3& pos, double y, double t)
    : position(pos), yaw(y), timestamp(t) {}
};

// ============================================================================
// 录制缓存类
// ============================================================================

/// 航迹录制缓存（支持节流<=10Hz）
class RecordingBuffer {
public:
  /// 构造函数
  /// @param max_hz 最大录制频率 (Hz)
  explicit RecordingBuffer(double max_hz = 10.0);

  /// 重置缓存（清空所有录制点）
  void reset();

  /// 添加录制点（带节流逻辑）
  /// @param position 位置 (ENU, m)
  /// @param yaw Yaw角 (rad)
  /// @param odom_stamp 里程计时间戳 (s)
  /// @return 是否成功添加（false表示被节流过滤）
  bool addPoint(const Vec3& position, double yaw, double odom_stamp);

  /// 获取已录制点数
  /// @return 点数
  size_t size() const;

  /// 获取录制点序列（只读）
  /// @return 录制点序列的const引用
  const std::vector<RecordPoint>& getPoints() const;

  /// 获取录制点序列（拷贝）
  /// @return 录制点序列的拷贝
  std::vector<RecordPoint> getPointsCopy() const;

  /// 检查缓存是否为空
  /// @return 是否为空
  bool empty() const;

private:
  /// 最大录制频率 (Hz)
  double max_hz_;

  /// 最小录制时间间隔 (s) = 1.0 / max_hz_
  double min_dt_;

  /// 第一个里程计时间戳（用作时间参考原点）
  double first_odom_stamp_;

  /// 上次录制的里程计时间戳
  double last_record_stamp_;

  /// 是否已记录第一个点
  bool has_first_point_;

  /// 录制点缓存
  std::vector<RecordPoint> points_;
};

}  // namespace domain
}  // namespace px4_backtrack
