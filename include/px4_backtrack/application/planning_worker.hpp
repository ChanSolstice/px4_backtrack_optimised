#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <trajectory_core/trajectory_core.hpp>

#include "px4_backtrack/domain/recording_buffer.hpp"
#include "px4_backtrack/domain/return_conditions.hpp"
#include "px4_backtrack/domain/trajectory_session.hpp"
#include "px4_backtrack/domain/waypoint_fallback_session.hpp"

namespace px4_backtrack {

namespace infrastructure { class DebugDumper; }
namespace adapters { class DebugPublisher; }

namespace application {

// ============================================================================
// 规划的输入与结果
// ============================================================================

/// 一次规划的输入：提交时的录制快照
struct PlanningInput {
  std::vector<domain::RecordPoint> points;
  trajectory_core::TrajectoryConfig config;
  double debug_sample_dt{};                              ///< 导出轨迹的采样步长 (s)
  std::shared_ptr<infrastructure::DebugDumper> dumper;   ///< 可选：规划 CSV
  std::shared_ptr<adapters::DebugPublisher> publisher;   ///< 可选：RViz
};

/// 一次规划的结果，失败已按第 7.5 节处理
struct PlanningResult {
  /// Ready / ReadyFallback / FailedTooClose / FailedFallback
  domain::PlanStatus status{domain::PlanStatus::None};
  std::unique_ptr<domain::TrajectorySession> trajectory;  ///< Ready 时有效
  domain::WaypointFallbackSession fallback;               ///< ReadyFallback 时有效
  std::size_t point_count{0};                             ///< 输入点数
  double cost_ms{0.0};                                    ///< 轨迹生成耗时 (ms)
};

/// 规划（重构方案第 7.5 节）：生成轨迹；点过近直接返回失败，其他失败改为航点降级
PlanningResult planReturn(const PlanningInput& input);

/// 航点降级：逆序稀疏航点，O(n)；规划超时时在控制线程直接调用
PlanningResult planFallback(const std::vector<domain::RecordPoint>& points);

// ============================================================================
// PlanningWorker类
// ============================================================================

/// 后台规划（重构方案第 7.5 节）
///
/// 单工作线程，最多一个待处理请求；新请求替换尚未开始的请求。用代次号丢弃
/// 过期结果：正在运行的求解无法终止，取消或有新请求时只丢弃它的结果。
/// 规划成功后在规划线程导出 CSV 和 RViz。
class PlanningWorker {
public:
  using Planner = std::function<PlanningResult(const PlanningInput&)>;

  /// 构造函数
  /// @param planner 规划函数
  /// @param start_thread 是否启动工作线程；为假时由 runPending 在调用者线程执行（单测用，R-76）
  explicit PlanningWorker(Planner planner = planReturn, bool start_thread = true);

  /// 析构：等正在运行的求解结束后退出
  ~PlanningWorker();

  PlanningWorker(const PlanningWorker&) = delete;
  PlanningWorker& operator=(const PlanningWorker&) = delete;

  /// 提交请求；之前的请求和结果作废
  void request(PlanningInput input);

  /// 取消：丢弃待处理的请求和结果
  void cancel();

  /// 取走最新请求的结果；还没有时为空
  std::unique_ptr<PlanningResult> takeResult();

  /// 在调用者线程执行待处理的请求（只在没有工作线程时使用）
  void runPending();

private:
  void run();

  /// 执行一个请求；代次号过期时丢弃结果
  void execute(const PlanningInput& input, std::uint64_t generation);

  Planner planner_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<PlanningInput> pending_;
  std::unique_ptr<PlanningResult> result_;
  std::uint64_t generation_{0};
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace application
}  // namespace px4_backtrack
