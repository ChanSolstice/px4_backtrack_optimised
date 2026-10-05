#pragma once
#include "px4_backtrack/domain/trajectory_session.hpp"
#include "px4_backtrack/domain/waypoint_fallback_session.hpp"
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace px4_backtrack {
namespace infrastructure {
class DebugDumper;
}
namespace adapters {
class DebugPublisher;
}
namespace application {
struct PlanningInput {
  std::vector<domain::RecordPoint> points;
  trajectory_core::TrajectoryConfig config;
  double debug_sample_dt = 0.1;
  std::shared_ptr<infrastructure::DebugDumper> dumper;
  std::shared_ptr<adapters::DebugPublisher> publisher;
};
struct PlanningResult {
  std::unique_ptr<domain::TrajectorySession> trajectory;
  domain::WaypointFallbackSession fallback;
};
// 单工作线程、最多一个待处理请求。主循环只交付快照/取结果，不等待求解。
class PlanningWorker {
public:
  using Planner =
      std::function<bool(domain::TrajectorySession &, const std::vector<domain::RecordPoint> &,
                         const trajectory_core::TrajectoryConfig &)>;
  explicit PlanningWorker(Planner planner = {});
  ~PlanningWorker();
  void request(PlanningInput input);
  void cancel(); // 换代丢弃旧结果；不强杀正在运行的算法。
  std::unique_ptr<PlanningResult> takeResult();

private:
  Planner planner_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<PlanningInput> pending_;
  std::unique_ptr<PlanningResult> result_;
  uint64_t generation_ = 0;
  bool stopping_ = false;
  std::thread thread_;
  void run();
};
} // namespace application
} // namespace px4_backtrack
