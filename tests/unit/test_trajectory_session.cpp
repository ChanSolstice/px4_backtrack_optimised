#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

std::vector<RecordPoint> makeSelfCrossingPoints() {
  // TrajectorySession 会逆序，因此返航路径为：左下 -> 右上 -> 左上 -> 右下。
  return {
      RecordPoint(Vec3(2.0, -2.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(-2.0, 2.0, 1.0), 0.0, 1.0),
      RecordPoint(Vec3(2.0, 2.0, 1.0), 0.0, 2.0),
      RecordPoint(Vec3(-2.0, -2.0, 1.0), 0.0, 3.0),
  };
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

  const auto& start_state = session.getTargetState();
  EXPECT_NEAR(start_state.position.x(), 2.0, 0.25);
  EXPECT_NEAR(start_state.position.z(), 1.0, 1e-6);
  EXPECT_NEAR(yawError(start_state.yaw, kPi), 0.0, 1e-3);

  auto end_state = session.getTrajectory()->evaluate(session.getDuration());
  EXPECT_NEAR(end_state.position.x(), 0.0, 0.25);
  EXPECT_NEAR(end_state.position.z(), 1.0, 1e-6);
  EXPECT_NEAR(yawError(end_state.yaw, kPi), 0.0, 1e-3);
}

TEST(TrajectorySessionTest, SlowAlongTrackMotionKeepsCrossTrackErrorNearZero) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));

  ASSERT_TRUE(session.updateProgress(Vec3(2.0, 0.0, 1.0), 0.5));
  EXPECT_NEAR(session.getCrossTrackError(), 0.0, 1e-6);

  ASSERT_TRUE(session.updateProgress(Vec3(1.9, 0.0, 1.0), 0.5));
  EXPECT_NEAR(session.getCrossTrackError(), 0.0, 1e-5);
  EXPECT_GT(session.getProjectedArcLength(), 0.05);
  EXPECT_GT(session.getTargetArcLength(), session.getProjectedArcLength());
}

TEST(TrajectorySessionTest, ProjectedProgressDoesNotMoveBackward) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  ASSERT_TRUE(session.updateProgress(Vec3(1.4, 0.0, 1.0), 0.5));
  const double forward_progress = session.getProjectedArcLength();
  ASSERT_GT(forward_progress, 0.4);

  ASSERT_TRUE(session.updateProgress(Vec3(1.7, 0.0, 1.0), 0.5));
  EXPECT_NEAR(session.getProjectedArcLength(), forward_progress, 1e-9);
}

TEST(TrajectorySessionTest, LocalProjectionDoesNotJumpAcrossSelfIntersection) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.v_max = 1.0;
  config.corner_blend_len_m = 0.2;

  ASSERT_TRUE(session.generate(makeSelfCrossingPoints(), config));
  const double path_length = session.getTrajectory()->pathLength();
  ASSERT_GT(path_length, 10.0);

  const std::vector<Vec3> first_branch_positions = {
      Vec3(-2.0, -2.0, 1.0),
      Vec3(-1.5, -1.5, 1.0),
      Vec3(-1.0, -1.0, 1.0),
      Vec3(-0.5, -0.5, 1.0),
      Vec3(0.0, 0.0, 1.0),
  };
  for (const auto& position : first_branch_positions) {
    ASSERT_TRUE(session.updateProgress(position, 0.5));
  }

  // 原点也位于后续返程支路；局部窗口必须保留在第一次穿越处。
  EXPECT_LT(session.getProjectedArcLength(), path_length * 0.5);
  EXPECT_NEAR(session.getCrossTrackError(), 0.0, 0.05);
}

TEST(TrajectorySessionTest, RejoinDoesNotMoveTargetArcLengthBackward) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.enable_end_converge = false;
  config.v_max = 1.0;
  config.v_end_max = 1.0;
  config.v_start_m_s = 1.0;
  config.v_end_m_s = 1.0;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  ASSERT_TRUE(session.updateProgress(Vec3(2.0, 0.0, 1.0), 0.5));
  const double normal_target_s = session.getTargetArcLength();
  ASSERT_GT(normal_target_s, 0.4);

  ASSERT_TRUE(session.updateRejoinProgress(
      Vec3(2.0, 1.0, 1.0), 0.5, 0.5));
  EXPECT_GE(session.getTargetArcLength(), normal_target_s);
  EXPECT_GT(session.getCrossTrackError(), 0.9);
}

TEST(TrajectorySessionTest, ProjectionFailureKeepsLastValidTarget) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  ASSERT_TRUE(session.updateProgress(Vec3(2.0, 0.0, 1.0), 0.5));
  const auto previous_target = session.getTargetState();
  const double previous_target_s = session.getTargetArcLength();

  Vec3 invalid_position = Vec3::Zero();
  invalid_position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(session.updateProgress(invalid_position, 0.5));
  EXPECT_FALSE(session.hasValidProjection());
  EXPECT_DOUBLE_EQ(session.getTargetArcLength(), previous_target_s);
  EXPECT_TRUE(session.getTargetState().position.isApprox(previous_target.position));
  EXPECT_TRUE(session.getTargetState().velocity.isApprox(previous_target.velocity));
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

TEST(TrajectorySessionTest, DenseEndpointClusterKeepsTrueEndpointWithoutShortFinalGap) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.ds_m = 0.6;
  config.min_point_dist_m = 0.3;

  std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(0.6, 0.0, 1.0), 0.0, 0.1),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.0, 0.2),
      RecordPoint(Vec3(1.20, 0.0, 1.0), 0.0, 0.3),
      RecordPoint(Vec3(1.24, 0.0, 1.0), 0.0, 0.4),
      RecordPoint(Vec3(1.26, 0.0, 1.0), 0.0, 0.5),
  };

  ASSERT_TRUE(session.generate(points, config));
  const auto start = session.getTrajectory()->evaluate(0.0);
  EXPECT_TRUE(start.position.isApprox(points.back().position, 1e-6));
}

TEST(TrajectorySessionTest, ReversingEndpointRemovesConflictingRetainedPoint) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;
  config.ds_m = 1.0;
  config.min_point_dist_m = 0.5;

  std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(0.5, 0.0, 1.0), 0.0, 0.1),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.0, 0.2),
      RecordPoint(Vec3(1.4, 0.0, 1.0), 0.0, 0.3),
      RecordPoint(Vec3(1.05, 0.0, 1.0), 0.0, 0.4),
  };

  ASSERT_TRUE(session.generate(points, config));
  const auto start = session.getTrajectory()->evaluate(0.0);
  EXPECT_TRUE(start.position.isApprox(points.back().position, 1e-6));
}

TEST(TrajectorySessionTest, FinishedRequiresArcPositionAndLowHorizontalSpeed) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  ASSERT_TRUE(session.generate(makeRecordPoints(), config));
  const Vec3 goal = session.getTrajectory()->goalPosition();

  ASSERT_TRUE(session.updateProgress(Vec3(1.0, 0.0, 1.0), 0.5));
  ASSERT_TRUE(session.updateProgress(goal, 0.5));
  EXPECT_FALSE(session.isFinished(
      goal + Vec3(0.3, 0.0, 0.0), Vec3::Zero(), 0.2));
  EXPECT_FALSE(session.isFinished(
      goal, Vec3(0.3, 0.0, 0.0), 0.2));
  EXPECT_TRUE(session.isFinished(goal, Vec3(0.1, 0.0, 0.0), 0.2));

  session.reset();
  EXPECT_FALSE(session.isValid());
  EXPECT_FALSE(session.isFinished(goal, Vec3::Zero(), 0.2));
  EXPECT_DOUBLE_EQ(session.getDuration(), 0.0);
}

}  // namespace
}  // namespace px4_backtrack::domain
