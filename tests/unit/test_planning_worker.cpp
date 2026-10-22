#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "px4_backtrack/application/planning_worker.hpp"

namespace px4_backtrack::application {
namespace {

using domain::PlanStatus;
using domain::RecordPoint;
using domain::Vec3;

/// 按点数区分请求
PlanningInput inputWithPoints(std::size_t n) {
  PlanningInput input;
  for (std::size_t i = 0; i < n; ++i) {
    input.points.emplace_back(Vec3(0.1 * static_cast<double>(i), 0.0, 1.0), 0.0,
                              0.1 * static_cast<double>(i));
  }
  return input;
}

/// 规划调用阻塞到 release，之后的调用直接返回；记录开始执行的请求
class Gate {
public:
  PlanningResult plan(const PlanningInput& input) {
    std::unique_lock<std::mutex> lock(mutex_);
    started_.push_back(input.points.size());
    cv_.notify_all();
    cv_.wait(lock, [this] { return open_; });
    PlanningResult result;
    result.status = PlanStatus::Ready;
    result.point_count = input.points.size();
    return result;
  }

  void release() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open_ = true;
    }
    cv_.notify_all();
  }

  bool waitStarted(std::size_t n) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, std::chrono::seconds(2), [&] { return started_.size() >= n; });
  }

  std::vector<std::size_t> started() {
    std::lock_guard<std::mutex> lock(mutex_);
    return started_;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_{false};
  std::vector<std::size_t> started_;
};

PlanningWorker::Planner gated(Gate& gate) {
  return [&gate](const PlanningInput& input) { return gate.plan(input); };
}

std::unique_ptr<PlanningResult> waitResult(PlanningWorker& worker) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto result = worker.takeResult()) {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return nullptr;
}

// ============================================================================
// 工作线程
// ============================================================================

TEST(PlanningWorkerTest, RunsRequestInBackground) {
  Gate gate;
  PlanningWorker worker(gated(gate));
  worker.request(inputWithPoints(3));
  ASSERT_TRUE(gate.waitStarted(1));
  EXPECT_FALSE(worker.takeResult());  // 求解期间没有结果，调用方不阻塞

  gate.release();
  const auto result = waitResult(worker);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->point_count, 3u);
  EXPECT_FALSE(worker.takeResult());  // 结果只取一次
}

TEST(PlanningWorkerTest, CancelDiscardsResultOfRunningSolve) {
  Gate gate;
  PlanningWorker worker(gated(gate));
  worker.request(inputWithPoints(1));
  ASSERT_TRUE(gate.waitStarted(1));
  worker.cancel();
  worker.request(inputWithPoints(2));
  gate.release();

  // 单线程按顺序执行：拿到第二个请求的结果时，第一个已经结束并被丢弃
  const auto result = waitResult(worker);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->point_count, 2u);
  EXPECT_FALSE(worker.takeResult());
}

TEST(PlanningWorkerTest, NewRequestReplacesPendingOne) {
  Gate gate;
  PlanningWorker worker(gated(gate));
  worker.request(inputWithPoints(1));
  ASSERT_TRUE(gate.waitStarted(1));
  worker.request(inputWithPoints(2));
  worker.request(inputWithPoints(3));
  gate.release();

  const auto result = waitResult(worker);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->point_count, 3u);
  EXPECT_EQ(gate.started(), (std::vector<std::size_t>{1, 3}));  // 第二个请求没有执行
}

TEST(PlanningWorkerTest, DestructorWaitsForRunningSolve) {
  Gate gate;
  auto worker = std::make_unique<PlanningWorker>(gated(gate));
  worker->request(inputWithPoints(1));
  ASSERT_TRUE(gate.waitStarted(1));
  std::thread releaser([&gate] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    gate.release();
  });
  worker.reset();  // 等求解结束后退出，不死锁
  releaser.join();
  EXPECT_EQ(gate.started().size(), 1u);
}

// ============================================================================
// 不启动线程（单测用）
// ============================================================================

TEST(PlanningWorkerTest, ManualModeRunsPendingOnDemand) {
  int calls = 0;
  PlanningWorker worker(
      [&calls](const PlanningInput& input) {
        ++calls;
        PlanningResult result;
        result.status = PlanStatus::Ready;
        result.point_count = input.points.size();
        return result;
      },
      false);

  worker.request(inputWithPoints(1));
  worker.request(inputWithPoints(2));
  EXPECT_FALSE(worker.takeResult());
  worker.runPending();
  auto result = worker.takeResult();
  ASSERT_TRUE(result);
  EXPECT_EQ(result->point_count, 2u);
  EXPECT_EQ(calls, 1);

  worker.request(inputWithPoints(3));
  worker.cancel();
  worker.runPending();
  EXPECT_FALSE(worker.takeResult());
  EXPECT_EQ(calls, 1);
}

// ============================================================================
// 规划结果分类（第 7.5 节）
// ============================================================================

TEST(PlanningWorkerTest, PlanReturnClassifiesResults) {
  PlanningInput ok = inputWithPoints(31);  // 3 m 直线
  const auto ready = planReturn(ok);
  EXPECT_EQ(ready.status, PlanStatus::Ready);
  ASSERT_TRUE(ready.trajectory);
  EXPECT_TRUE(ready.trajectory->isValid());
  EXPECT_EQ(ready.point_count, 31u);

  PlanningInput hover;
  for (int i = 0; i < 30; ++i) {
    hover.points.emplace_back(Vec3(0.0, 0.0, 1.0), 0.0, 0.1 * i);
  }
  const auto too_close = planReturn(hover);
  EXPECT_EQ(too_close.status, PlanStatus::FailedTooClose);
  EXPECT_FALSE(too_close.trajectory);

  PlanningInput invalid = inputWithPoints(31);
  invalid.config.v_end_max = invalid.config.v_max + 1.0;
  EXPECT_EQ(planReturn(invalid).status, PlanStatus::FailedConfig);

  // 其他失败（点数不足）改为航点降级，降级也构建不了
  EXPECT_EQ(planReturn(inputWithPoints(1)).status, PlanStatus::FailedFallback);

  const auto fallback = planFallback(ok.points);
  EXPECT_EQ(fallback.status, PlanStatus::ReadyFallback);
  EXPECT_TRUE(fallback.fallback.isValid());
  EXPECT_FALSE(fallback.trajectory);
}

}  // namespace
}  // namespace px4_backtrack::application
