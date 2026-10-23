#include <gtest/gtest.h>

#include <cmath>

#include "px4_backtrack/domain/waypoint_fallback_session.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<RecordPoint> makeRecordedLine() {
  return {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.1, 1.0),
      RecordPoint(Vec3(2.0, 0.0, 1.0), 0.2, 2.0),
  };
}

TEST(WaypointFallbackSessionTest, BuildCreatesReverseSparsePath) {
  WaypointFallbackSession session(0.5, 0.2);

  ASSERT_TRUE(session.build(makeRecordedLine()));
  EXPECT_GE(session.size(), 4u);

  auto target = session.currentTarget();
  EXPECT_NEAR(target.position.x(), 2.0, 1e-6);
  EXPECT_NEAR(target.position.z(), 1.0, 1e-6);
  EXPECT_NEAR(std::abs(target.yaw), kPi, 1e-6);
}

TEST(WaypointFallbackSessionTest, YawFollowsCurvedReturnPathDirection) {
  const std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.7, 0.0),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.7, 1.0),
      RecordPoint(Vec3(1.0, 1.0, 1.0), 0.7, 2.0),
  };
  WaypointFallbackSession session(0.5, 0.2);

  ASSERT_TRUE(session.build(points));
  auto target = session.currentTarget();

  EXPECT_NEAR(target.yaw, -kPi / 2.0, 1e-6);
}

TEST(WaypointFallbackSessionTest, ProgressAdvancesFromHead) {
  WaypointFallbackSession session(0.5, 0.25);
  ASSERT_TRUE(session.build(makeRecordedLine()));
  EXPECT_NEAR(session.currentTarget().position.x(), 2.0, 1e-6);

  session.updateProgress(Vec3(2.0, 0.0, 1.0));
  EXPECT_LT(session.currentTarget().position.x(), 2.0);
}

TEST(WaypointFallbackSessionTest, ProgressAdvancesAtMostOneWaypointPerTick) {
  WaypointFallbackSession session(0.3, 0.2);
  ASSERT_TRUE(session.build(makeRecordedLine()));

  session.updateProgress(Vec3(2.0, 0.0, 1.0));
  EXPECT_EQ(session.currentIndex(), 1u);

  session.updateProgress(Vec3(1.7, 0.0, 1.0));
  EXPECT_EQ(session.currentIndex(), 2u);

  auto target = session.currentTarget();
  EXPECT_GT(target.position.x(), 1.0);
}

TEST(WaypointFallbackSessionTest, FinalWaypointMarksSessionFinished) {
  WaypointFallbackSession session(0.5, 0.25);
  ASSERT_TRUE(session.build(makeRecordedLine()));

  session.updateProgress(Vec3(2.0, 0.0, 1.0));
  session.updateProgress(Vec3(1.5, 0.0, 1.0));
  session.updateProgress(Vec3(1.0, 0.0, 1.0));
  session.updateProgress(Vec3(0.5, 0.0, 1.0));
  session.updateProgress(Vec3(0.0, 0.0, 1.0));

  EXPECT_TRUE(session.isFinished());
}

TEST(WaypointFallbackSessionTest, RemainingLengthShrinksAsTargetsAdvance) {
  // 第 8.1 节：停滞判定用当前目标航点到终点的折线长度
  WaypointFallbackSession session(0.5, 0.25);
  ASSERT_TRUE(session.build(makeRecordedLine()));
  EXPECT_NEAR(session.remainingLength(), 2.0, 1e-6);

  session.updateProgress(Vec3(2.0, 0.0, 1.0));
  EXPECT_NEAR(session.remainingLength(), 1.5, 1e-6);
  session.updateProgress(Vec3(1.2, 0.0, 1.0));  // 没到下一个航点
  EXPECT_NEAR(session.remainingLength(), 1.5, 1e-6);

  for (const double x : {1.5, 1.0, 0.5, 0.0}) {
    session.updateProgress(Vec3(x, 0.0, 1.0));
  }
  EXPECT_TRUE(session.isFinished());
  EXPECT_NEAR(session.remainingLength(), 0.0, 1e-9);
}

}  // namespace
}  // namespace px4_backtrack::domain
