#include <gtest/gtest.h>

#include <cmath>

#include "px4_backtrack/domain/trajectory_session.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<RecordPoint> makeRecordPoints() {
  return {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.1, 0.2),
      RecordPoint(Vec3(2.0, 0.0, 1.0), 0.2, 1.0),
  };
}

std::vector<RecordPoint> makeDensePauseClusterPoints() {
  std::vector<RecordPoint> points;

  double stamp = 0.0;
  for (int i = 0; i <= 50; ++i) {
    points.emplace_back(Vec3(i * 0.02, 0.0, 1.0), 0.0, stamp);
    stamp += 0.1;
  }

  for (int i = 0; i < 20; ++i) {
    double y = (i % 2 == 0) ? 0.02 : -0.02;
    points.emplace_back(Vec3(1.0 + 0.01 * (i % 3), y, 1.0), 0.0, stamp);
    stamp += 0.1;
  }

  for (int i = 1; i <= 50; ++i) {
    points.emplace_back(Vec3(1.0 + i * 0.02, 0.0, 1.0), 0.0, stamp);
    stamp += 0.1;
  }

  return points;
}

std::vector<RecordPoint> makeSlowStraightLinePoints() {
  std::vector<RecordPoint> points;

  double stamp = 0.0;
  for (int i = 0; i <= 50; ++i) {
    points.emplace_back(Vec3(i * 0.02, 0.0, 1.0), 0.0, stamp);
    stamp += 0.1;
  }

  return points;
}

double yawError(double actual, double expected) {
  return std::abs(std::atan2(std::sin(actual - expected),
                            std::cos(actual - expected)));
}

TEST(TrajectorySessionTest, RejectsInsufficientPoints) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 0.0), 0.0, 0.0),
  };

  EXPECT_FALSE(session.generate(points, config));
  EXPECT_FALSE(session.isValid());
  EXPECT_EQ(session.getLastFailureKind(), TrajectorySession::FailureKind::Other);
}

TEST(TrajectorySessionTest, MarksTooCloseGeometryFailure) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.ds_m = 0.1;
  config.min_point_dist_m = 0.01;

  std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.1),
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.2),
  };

  EXPECT_FALSE(session.generate(points, config));
  EXPECT_FALSE(session.isValid());
  EXPECT_EQ(session.getLastFailureKind(), TrajectorySession::FailureKind::TooCloseGeometry);
}

TEST(TrajectorySessionTest, MarksInvalidConfigFailure) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.v_end_max = config.v_max + 1.0;

  EXPECT_FALSE(session.generate(makeRecordPoints(), config));
  EXPECT_FALSE(session.isValid());
  EXPECT_EQ(session.getLastFailureKind(), TrajectorySession::FailureKind::InvalidConfig);
}

TEST(TrajectorySessionTest, GenerateProducesValidTrajectoryInReverseOrder) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  ASSERT_TRUE(session.isValid());
  ASSERT_NE(session.getTrajectory(), nullptr);
  EXPECT_GT(session.getDuration(), 0.0);

  auto start_state = session.evaluate(10.0);
  EXPECT_NEAR(start_state.position.x(), 2.0, 0.25);
  EXPECT_NEAR(start_state.position.z(), 1.0, 1e-6);
  EXPECT_NEAR(yawError(start_state.yaw, kPi), 0.0, 1e-3);

  auto end_state = session.getTrajectory()->evaluate(session.getDuration());
  EXPECT_NEAR(end_state.position.x(), 0.0, 0.25);
  EXPECT_NEAR(end_state.position.z(), 1.0, 1e-6);
  EXPECT_NEAR(yawError(end_state.yaw, kPi), 0.0, 1e-3);
}

TEST(TrajectorySessionTest, FreezeAndResumeKeepProgressContinuous) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));

  session.evaluate(10.0);
  session.evaluate(10.5);
  EXPECT_NEAR(session.getProgress(), 0.5, 1e-6);

  session.freezeProgress();
  session.evaluate(11.5);
  EXPECT_NEAR(session.getProgress(), 0.5, 1e-6);

  session.resumeProgress(20.0);
  session.evaluate(20.5);
  EXPECT_NEAR(session.getProgress(), 1.0, 1e-6);
}

TEST(TrajectorySessionTest, DensePauseClusterDoesNotBlowUpPathLength) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.ds_m = 0.15;
  config.min_point_dist_m = 0.01;

  ASSERT_TRUE(session.generate(makeDensePauseClusterPoints(), config));
  ASSERT_TRUE(session.isValid());
  ASSERT_NE(session.getTrajectory(), nullptr);

  double path_length = session.getTrajectory()->pathLength();
  EXPECT_GT(path_length, 1.8);
  EXPECT_LT(path_length, 2.4);
}

TEST(TrajectorySessionTest, SlowStraightLineKeepsAccumulatedDistanceWhenThinning) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.ds_m = 0.15;
  config.min_point_dist_m = 0.08;

  ASSERT_TRUE(session.generate(makeSlowStraightLinePoints(), config));
  ASSERT_TRUE(session.isValid());
  ASSERT_NE(session.getTrajectory(), nullptr);

  double path_length = session.getTrajectory()->pathLength();
  EXPECT_GT(path_length, 0.8);
  EXPECT_LT(path_length, 1.2);
}

TEST(TrajectorySessionTest, SetProgressControlsFinishedState) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  ASSERT_GT(session.getDuration(), 0.0);

  session.setProgress(session.getDuration() + 0.01);
  EXPECT_TRUE(session.isFinished());

  session.reset();
  EXPECT_FALSE(session.isValid());
  EXPECT_DOUBLE_EQ(session.getDuration(), 0.0);
}

}  // namespace
}  // namespace px4_backtrack::domain
