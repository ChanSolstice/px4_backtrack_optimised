#include "px4_backtrack/application/planning_worker.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"

#include <utility>

namespace px4_backtrack {
namespace application {

// ============================================================================
// 规划
// ============================================================================

PlanningResult planReturn(const PlanningInput& input) {
  PlanningResult result;
  result.point_count = input.points.size();
  result.trajectory = std::make_unique<domain::TrajectorySession>();

  if (result.trajectory->generate(input.points, input.config)) {
    result.status = domain::PlanStatus::Ready;
    result.cost_ms = result.trajectory->getLastGenerateCostMs();
    return result;
  }

  if (result.trajectory->getLastFailureKind() ==
      domain::TrajectorySession::FailureKind::TooCloseGeometry) {
    result.status = domain::PlanStatus::FailedTooClose;
    result.trajectory.reset();
    return result;
  }

  // 其他失败：航点降级；降级也构建失败则原地降落
  return planFallback(input.points);
}

PlanningResult planFallback(const std::vector<domain::RecordPoint>& points) {
  PlanningResult result;
  result.point_count = points.size();
  result.status = result.fallback.build(points) ? domain::PlanStatus::ReadyFallback
                                                : domain::PlanStatus::FailedFallback;
  return result;
}

// ============================================================================
// PlanningWorker
// ============================================================================

PlanningWorker::PlanningWorker(Planner planner, bool start_thread)
  : planner_(std::move(planner))
{
  if (start_thread) {
    thread_ = std::thread(&PlanningWorker::run, this);
  }
}

PlanningWorker::~PlanningWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    ++generation_;
    pending_.reset();
  }
  wake_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void PlanningWorker::request(PlanningInput input) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    pending_ = std::move(input);
    result_.reset();
  }
  wake_.notify_one();
}

void PlanningWorker::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_.reset();
  result_.reset();
}

std::unique_ptr<PlanningResult> PlanningWorker::takeResult() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::move(result_);
}

void PlanningWorker::runPending() {
  std::optional<PlanningInput> input;
  std::uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    input.swap(pending_);
    generation = generation_;
  }
  if (input) {
    execute(*input, generation);
  }
}

void PlanningWorker::run() {
  for (;;) {
    std::optional<PlanningInput> input;
    std::uint64_t generation = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (stopping_) {
        return;
      }
      input.swap(pending_);
      generation = generation_;
    }
    execute(*input, generation);
  }
}

void PlanningWorker::execute(const PlanningInput& input, std::uint64_t generation) {
  auto result = std::make_unique<PlanningResult>(planner_(input));

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_) {
      return;  // 已取消或有新请求
    }
  }

  // 导出在规划线程完成，之后再交出结果，控制线程不会与导出同时访问轨迹
  if (result->status == domain::PlanStatus::Ready && (input.dumper || input.publisher)) {
    const auto states = result->trajectory->getTrajectory()->sample(input.debug_sample_dt);
    if (input.dumper) {
      input.dumper->dumpWaypoints(input.points);
      input.dumper->dumpTrajectory(states);
    }
    if (input.publisher) {
      input.publisher->publishWaypoints(input.points);
      input.publisher->publishTrajectory(states);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (generation == generation_) {
    result_ = std::move(result);
  }
}

}  // namespace application
}  // namespace px4_backtrack
