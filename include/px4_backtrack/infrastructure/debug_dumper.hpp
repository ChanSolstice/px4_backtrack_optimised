#pragma once

#include "px4_backtrack/domain/recording_buffer.hpp"
#include <trajectory_core/trajectory_core.hpp>
#include <string>
#include <vector>

namespace px4_backtrack {
namespace infrastructure {

/// 规划 CSV：把规划输入和采样后的轨迹写到调试日志的运行目录（重构方案第 9.2 节）
///
/// 文件名按本次运行的第几次规划编号：plan_<n>_input.csv、plan_<n>_trajectory.csv。
class DebugDumper {
public:
  /// 构造函数
  /// @param dir 输出目录（调试日志的运行目录，已存在）
  explicit DebugDumper(std::string dir);

  /// 规划输入（录制点）
  void dumpInput(int plan, const std::vector<domain::RecordPoint>& points) const;

  /// 采样后的轨迹
  void dumpTrajectory(int plan, const std::vector<trajectory_core::TrajectoryState>& states) const;

private:
  std::string dir_;
};

}  // namespace infrastructure
}  // namespace px4_backtrack
