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
  i.points={domain::RecordPoint(domain::Vec3(0,0,1),0,0),domain::RecordPoint(domain::Vec3(2,0,1),0,1)};
  return i;
}
TEST(AsyncControlTest, BlockedPlannerDoesNotBlockAndCanceledResultIsNeverDelivered) {
  std::promise<void> release, entered;
  auto gate=release.get_future().share(); auto started=entered.get_future();
  application::PlanningWorker worker([&](auto& session,const auto& points,const auto& config) {
    entered.set_value(); gate.wait(); return session.generate(points,config);
  });
  worker.request(input());
  const auto ready=started.wait_for(std::chrono::seconds(2));
  if (ready!=std::future_status::ready) { release.set_value(); FAIL()<<"Planner did not start"; }
  const auto start=std::chrono::steady_clock::now();
  for(int n=0;n<10000;++n) EXPECT_FALSE(worker.takeResult());
  worker.cancel();
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),0.10);
  release.set_value();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(worker.takeResult());
}
TEST(AsyncControlTest, PlannerExceptionFallsBack) {
  application::PlanningWorker worker([](auto&,const auto&,const auto&)->bool { throw std::runtime_error("solver failure"); });
  worker.request(input());
  std::unique_ptr<application::PlanningResult> result;
  for(int n=0;n<2000 && !result;++n) { result=worker.takeResult(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  ASSERT_TRUE(result); EXPECT_TRUE(result->fallback.isValid());
}
TEST(AsyncControlTest, BlockedModeServiceDoesNotBlockAndCanceledQueuedModeIsNotSent) {
  std::promise<void> release, entered;
  auto gate=release.get_future().share(); auto started=entered.get_future();
  std::atomic<int> calls{0};
  adapters::SetModeClient client([&](domain::ModeRequest) { ++calls; entered.set_value(); gate.wait(); return true; },0.0);
  client.request(domain::ModeRequest::OFFBOARD);
  const auto ready=started.wait_for(std::chrono::seconds(2));
  if (ready!=std::future_status::ready) { release.set_value(); FAIL()<<"Mode worker did not start"; }
  const auto start=std::chrono::steady_clock::now();
  for(int n=0;n<10000;++n) client.request(domain::ModeRequest::AUTO_LAND);
  client.request(std::nullopt);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),0.10);
  release.set_value(); std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(calls.load(),1);
}
}
