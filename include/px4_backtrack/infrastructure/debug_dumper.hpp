#pragma once

#include "px4_backtrack/domain/recording_buffer.hpp"
#include <trajectory_core/trajectory_core.hpp>
#include <string>
#include <vector>

namespace px4_backtrack {
namespace infrastructure {

/// CSV debug dumper — 将录制航点和采样轨迹写入带时间戳的 CSV 文件
class DebugDumper {
public:
  /// 构造函数
  /// @param output_dir 输出目录，空字符串则默认 ~/.ros/backtrack_debug/
  explicit DebugDumper(const std::string& output_dir = "");

  /// 导出录制航点到 CSV
  /// @param points 录制点序列
  /// @return 写入的文件路径，失败返回空字符串
  std::string dumpWaypoints(const std::vector<domain::RecordPoint>& points);

  /// 导出采样轨迹到 CSV
  /// @param states 采样后的轨迹状态序列
  /// @return 写入的文件路径，失败返回空字符串
  std::string dumpTrajectory(const std::vector<trajectory_core::TrajectoryState>& states);

private:
  std::string output_dir_;

  /// 确保输出目录存在
  bool ensureDirectory();

  /// 生成带时间戳的文件名
  std::string makeFilename(const std::string& prefix) const;
};

}  // namespace infrastructure
}  // namespace px4_backtrack
