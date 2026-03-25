#include <gtest/gtest.h>

#include "px4_backtrack/domain/trajectory_session.hpp"

namespace px4_backtrack::domain {
namespace {

std::vector<RecordPoint> makeRecordPoints() {
  return {
      RecordPoint(Vec3(0.0, 0.0, 1.0), 0.0, 0.0),
      RecordPoint(Vec3(1.0, 0.0, 1.0), 0.1, 0.2),
      RecordPoint(Vec3(2.0, 0.0, 1.0), 0.2, 1.0),
  };
}

TEST(TrajectorySessionTest, RejectsInsufficientPoints) {
  TrajectorySession session;
  trajectory_core::TrajectoryConfig config;

  std::vector<RecordPoint> points = {
      RecordPoint(Vec3(0.0, 0.0, 0.0), 0.0, 0.0),
  };

  EXPECT_FALSE(session.generate(points, config));
  EXPECT_FALSE(session.isValid());
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

  auto end_state = session.getTrajectory()->evaluate(session.getDuration());
  EXPECT_NEAR(end_state.position.x(), 0.0, 0.25);
  EXPECT_NEAR(end_state.position.z(), 1.0, 1e-6);
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
