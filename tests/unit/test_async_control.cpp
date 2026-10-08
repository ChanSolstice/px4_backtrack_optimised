#include <gtest/gtest.h>
#include "px4_backtrack/application/planning_worker.hpp"
#include "px4_backtrack/adapters/ros_services.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <thread>
using namespace px4_backtrack;
namespace {
application::PlanningInput input() {
  application::PlanningInput i;
  i.points = {domain::RecordPoint(domain::Vec3(0, 0, 1), 0, 0),
              domain::RecordPoint(domain::Vec3(2, 0, 1), 0, 1)};
  return i;
}
TEST(AsyncControlTest, BlockedPlannerDoesNotBlockAndCanceledResultIsNeverDelivered) {
  std::promise<void> release, entered;
  auto gate = release.get_future().share();
  auto started = entered.get_future();
  application::PlanningWorker worker([&](auto &session, const auto &points, const auto &config) {
    entered.set_value();
    gate.wait();
    return session.generate(points, config);
  });
  worker.request(input());
  const auto ready = started.wait_for(std::chrono::seconds(2));
  if (ready != std::future_status::ready) {
    release.set_value();
    FAIL() << "Planner did not start";
  }
  const auto start = std::chrono::steady_clock::now();
  for (int n = 0; n < 10000; ++n)
    EXPECT_FALSE(worker.takeResult());
  worker.cancel();
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 0.10);
  release.set_value();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(worker.takeResult());
}
TEST(AsyncControlTest, PlannerExceptionFallsBack) {
  application::PlanningWorker worker([](auto &, const auto &, const auto &) -> bool {
    throw std::runtime_error("solver failure");
  });
  worker.request(input());
  std::unique_ptr<application::PlanningResult> result;
  for (int n = 0; n < 2000 && !result; ++n) {
    result = worker.takeResult();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->fallback.isValid());
}
TEST(AsyncControlTest, CorridorFailureUsesOriginalCornerWaypoints) {
  auto job = input();
  job.config.max_deviation_m = 0.001;
  job.config.corner_blend_len_m = 0.03;
  job.config.ds_check_m = 0.001;
  job.points = {domain::RecordPoint(domain::Vec3(0, 0, 1), 0, 0),
                domain::RecordPoint(domain::Vec3(5, 0, 1), 0, 1),
                domain::RecordPoint(domain::Vec3(5, 5, 1), 0, 2)};
  application::PlanningWorker worker;
  worker.request(std::move(job));
  std::unique_ptr<application::PlanningResult> result;
  for (int i = 0; i < 2000 && !result; ++i) {
    result = worker.takeResult();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->trajectory->isValid());
  EXPECT_TRUE(result->fallback.isValid());
}

TEST(AsyncControlTest, PlanningTracePreservesFailureInputAndStatusBeforeSessionReset) {
  auto job = input();
  job.flight_id = 7;
  job.plan_id = 11;
  job.config.v_max = -1.0;
  std::mutex trace_mutex;
  std::vector<std::string> phases;
  std::string failure_status;
  size_t input_count = 0;
  bool fallback_ready = false;
  job.observer = [&](const application::PlanningTrace &trace) {
    std::lock_guard<std::mutex> lock(trace_mutex);
    EXPECT_EQ(trace.flight_id, 7u);
    EXPECT_EQ(trace.plan_id, 11u);
    ASSERT_NE(trace.points, nullptr);
    ASSERT_NE(trace.config, nullptr);
    EXPECT_EQ(trace.points->back().position.x(), 2.0);
    EXPECT_EQ(trace.config->v_max, -1.0);
    phases.push_back(trace.phase);
    if (trace.phase == "finished") {
      failure_status = trace.generation.status;
      input_count = trace.points->size();
      fallback_ready = trace.fallback_ready;
    }
  };
  application::PlanningWorker worker;
  worker.request(std::move(job));
  std::unique_ptr<application::PlanningResult> result;
  for (int n = 0; n < 2000 && !result; ++n) {
    result = worker.takeResult();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->fallback.isValid());
  std::lock_guard<std::mutex> lock(trace_mutex);
  EXPECT_EQ(phases, (std::vector<std::string>{"started", "finished"}));
  EXPECT_EQ(failure_status, "INVALID_INPUT");
  EXPECT_EQ(input_count, 2u);
  EXPECT_TRUE(fallback_ready);
}

TEST(AsyncControlTest, ObserverExceptionCannotPreventPlanningOrFallback) {
  auto job = input();
  job.observer = [](const auto &) { throw std::runtime_error("logging unavailable"); };
  application::PlanningWorker worker;
  worker.request(std::move(job));
  std::unique_ptr<application::PlanningResult> result;
  for (int n = 0; n < 2000 && !result; ++n) {
    result = worker.takeResult();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->trajectory->isValid());
}

TEST(AsyncControlTest, CanceledRunningPlanReportsDiscardWithoutDeliveringResult) {
  std::promise<void> entered, release, discarded;
  auto gate = release.get_future().share();
  auto began = entered.get_future();
  auto discarded_future = discarded.get_future();
  auto job = input();
  job.plan_id = 21;
  job.observer = [&](const auto &trace) {
    if (trace.phase == "discarded") {
      EXPECT_EQ(trace.plan_id, 21u);
      EXPECT_EQ(trace.points->size(), 2u);
      discarded.set_value();
    }
  };
  application::PlanningWorker worker([&](auto &session, const auto &points, const auto &config) {
    entered.set_value();
    gate.wait();
    return session.generate(points, config);
  });
  worker.request(std::move(job));
  if (began.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    release.set_value();
    FAIL() << "Planner did not start";
  }
  worker.cancel();
  release.set_value();
  ASSERT_EQ(discarded_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(worker.takeResult());
}

TEST(AsyncControlTest, BlockedModeServiceDoesNotBlockAndCanceledQueuedModeIsNotSent) {
  std::promise<void> release, entered;
  auto gate = release.get_future().share();
  auto started = entered.get_future();
  std::atomic<int> calls{0};
  adapters::SetModeClient client(
      [&](domain::ModeRequest) {
        ++calls;
        entered.set_value();
        gate.wait();
        return true;
      },
      0.0);
  client.request(domain::ModeRequest::OFFBOARD);
  const auto ready = started.wait_for(std::chrono::seconds(2));
  if (ready != std::future_status::ready) {
    release.set_value();
    FAIL() << "Mode worker did not start";
  }
  const auto start = std::chrono::steady_clock::now();
  for (int n = 0; n < 10000; ++n)
    client.request(domain::ModeRequest::AUTO_LAND);
  client.request(std::nullopt);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 0.10);
  release.set_value();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(calls.load(), 1);
}
} // namespace
