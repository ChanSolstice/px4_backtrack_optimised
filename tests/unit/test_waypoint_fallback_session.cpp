#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "px4_backtrack/domain/waypoint_fallback_session.hpp"

namespace px4_backtrack::domain {
namespace {

void advance(WaypointFallbackSession& session, const Vec3& position) {
  session.updateProgress(position, Vec3::Zero(), session.currentTarget().yaw);
}

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

TEST(WaypointFallbackSessionTest, RetainsRemainingTargetAcrossPause) {
  WaypointFallbackSession session(0.5, 0.25);
  ASSERT_TRUE(session.build(makeRecordedLine()));

  advance(session, Vec3(2.0, 0.0, 1.0));
  auto target = session.currentTarget();
  EXPECT_LT(target.position.x(), 2.0);

  const auto retained_index = session.currentIndex();
  target = session.currentTarget();
  EXPECT_NEAR(target.position.x(), 1.5, 1e-6);
  advance(session, Vec3(2.0, 0.0, 1.0));
  EXPECT_EQ(session.currentIndex(), retained_index);
}

TEST(WaypointFallbackSessionTest, ProgressAdvancesAtMostOneWaypointPerTick) {
  WaypointFallbackSession session(0.3, 0.2);
  ASSERT_TRUE(session.build(makeRecordedLine()));

  advance(session, Vec3(2.0, 0.0, 1.0));
  EXPECT_EQ(session.currentIndex(), 1u);

  advance(session, Vec3(1.7, 0.0, 1.0));
  EXPECT_EQ(session.currentIndex(), 2u);

  auto target = session.currentTarget();
  EXPECT_GT(target.position.x(), 1.0);
}

TEST(WaypointFallbackSessionTest, PreservesShortRightAngleCornerAndWaitsForStopAndYaw) {
  WaypointFallbackSession session(0.3, 0.01);
  ASSERT_TRUE(session.build({RecordPoint(Vec3(0,0,1),0,0),
      RecordPoint(Vec3(0.1,0,1),0,1), RecordPoint(Vec3(0.1,0.1,1),0,2)}));
  advance(session, Vec3(0.1,0.1,1));
  ASSERT_EQ(session.currentIndex(), 1u);
  EXPECT_TRUE(session.currentTarget().position.isApprox(Vec3(0.1,0,1)));
  session.updateProgress(Vec3(0.1,0,1), Vec3(0,0,-0.2), -kPi/2);
  EXPECT_EQ(session.currentIndex(), 1u);
  session.updateProgress(Vec3(0.1,0,1), Vec3::Zero(), -kPi/2);
  EXPECT_EQ(session.currentIndex(), 1u);
  EXPECT_NEAR(std::abs(session.currentTarget().yaw), kPi, 1e-9);
  advance(session, Vec3(0.1,0,1));
  EXPECT_EQ(session.currentIndex(), 2u);
}

TEST(WaypointFallbackSessionTest, RejectsInvalidParametersAndGeometry) {
  WaypointFallbackSession invalid(0.0);
  EXPECT_FALSE(invalid.build(makeRecordedLine()));
  WaypointFallbackSession session;
  auto points = makeRecordedLine();
  points[1].position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(session.build(points));
}

TEST(WaypointFallbackSessionTest, FinalWaypointMarksSessionFinished) {
  WaypointFallbackSession session(0.5, 0.25);
  ASSERT_TRUE(session.build(makeRecordedLine()));

  advance(session, Vec3(2.0, 0.0, 1.0));
  advance(session, Vec3(1.5, 0.0, 1.0));
  advance(session, Vec3(1.0, 0.0, 1.0));
  advance(session, Vec3(0.5, 0.0, 1.0));
  advance(session, Vec3(0.0, 0.0, 1.0));

  EXPECT_TRUE(session.isFinished());
}

}  // namespace
}  // namespace px4_backtrack::domain
