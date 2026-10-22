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

  /// 截断：只保留 [0, last_index]（重构方案第 7.4 节）
  void truncate(size_t last_index);

  /// 录制在任务期间暂停后恢复：下一个点的时间戳接在末点之后 1/max_hz（第 7.4 节，R-77）
  void resume();

  /// 游标搜索（第 7.4 节）：从 cursor 往前、与 cursor 的累计距离不超过 window_m 的录制点中，
  /// 找离 reference 最近的点，距离相同时取更小的索引
  /// @param cursor 当前游标，须小于 size()
  /// @return 新的游标，不大于 cursor
  size_t nearestBefore(size_t cursor, const Vec3& reference, double window_m) const;

  /// 从末点往前，相邻两点间距都不超过 max_gap_m 的连续段的起点索引；没有断点时为 0（D30）
  size_t continuousFrom(double max_gap_m) const;

private:
  /// 最大录制频率 (Hz)
  double max_hz_;

  /// 最小录制时间间隔 (s) = 1.0 / max_hz_
  double min_dt_;

  /// 相对时间 0 对应的里程计时间戳；续录时平移
  double time_origin_;

  /// 下一个点按续录接续时间戳
  bool resume_pending_;

  /// 上次录制的里程计时间戳
  double last_record_stamp_;

  /// 是否已记录第一个点
  bool has_first_point_;

  /// 录制点缓存
  std::vector<RecordPoint> points_;
};

}  // namespace domain
}  // namespace px4_backtrack
