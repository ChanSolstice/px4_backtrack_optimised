#include "px4_backtrack/application/planning_worker.hpp"
#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <ros/console.h>
#include <algorithm>
#include <cmath>

namespace px4_backtrack::application {
PlanningWorker::PlanningWorker(Planner planner)
    : planner_(std::move(planner)), thread_(&PlanningWorker::run, this) {}
PlanningWorker::~PlanningWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    ++generation_;
    pending_.reset();
  }
  wake_.notify_one();
  thread_.join();
}
void PlanningWorker::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_.reset();
  result_.reset();
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
std::unique_ptr<PlanningResult> PlanningWorker::takeResult() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::move(result_);
}
void PlanningWorker::run() {
  for (;;) {
    PlanningInput input;
    uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (stopping_)
        return;
      generation = generation_;
      input = std::move(*pending_);
      pending_.reset();
    }
    auto result = std::make_unique<PlanningResult>();
    result->trajectory = std::make_unique<domain::TrajectorySession>();
    bool success = false;
    try {
      success = planner_ ? planner_(*result->trajectory, input.points, input.config)
                         : result->trajectory->generate(input.points, input.config);
    } catch (const std::exception &error) {
      ROS_ERROR("[PlanningWorker] Planning exception: %s", error.what());
    }
    if (!success || !result->trajectory->isValid()) {
      result->trajectory->reset();
      result->fallback.build(input.points);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || generation != generation_)
        continue;
    }
    // Debug 与规划共用非控制线程，最多 20000 个轨迹样本；任何导出异常不影响返航。
    if (success && result->trajectory->isValid() && (input.dumper || input.publisher)) {
      try {
        const auto *trajectory = result->trajectory->getTrajectory();
        if (!std::isfinite(input.debug_sample_dt) || input.debug_sample_dt <= 0.0)
          throw std::invalid_argument("Invalid debug sampling step");
        const double dt = std::max(input.debug_sample_dt, trajectory->duration() / 19998.0);
        const auto states = trajectory->sample(dt);
        if (input.dumper) {
          input.dumper->dumpWaypoints(input.points);
          input.dumper->dumpTrajectory(states);
        }
        if (input.publisher) {
          input.publisher->publishWaypoints(input.points);
          input.publisher->publishTrajectory(states);
        }
      } catch (const std::exception &error) {
        ROS_WARN("[PlanningWorker] Debug skipped: %s", error.what());
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stopping_ && generation == generation_)
        result_ = std::move(result);
    }
  }
}
} // namespace px4_backtrack::application
